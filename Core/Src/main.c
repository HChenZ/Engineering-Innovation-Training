/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "avoidance.h"
#include "bluetooth.h"
#include "delay_us.h"
#include "encoder.h"
#include "greyscale_sensor.h"
#include "led.h"
#include "motor.h"
#include "odom.h"
#include "oled_software_i2c.h"
#include "pid.h"
#include "tracking.h"
#include "teach_finish.h"
#include "line_guard.h"
#include "ultrasonic.h"

#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  CAR_STATE_STOPPED = 0,
  CAR_STATE_RUNNING,
  CAR_STATE_FINISHED
} CarState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TRACK_BASE_SPEED 35.0f
#define PID_PERIOD_S 0.01f
#define TRACK_ALL_BLACK_BITMAP 0xFFU
#define OLED_ROW_PERIOD_MS 25U /* 每次只刷一行，8行约200ms，避免主循环一次长时间占用 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void Car_RequestStart(void);
static void Car_RequestStop(void);
static uint8_t Car_CanStart(void);
static void Car_PrepareStart(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static PID_Controller pos_pid;
static PID_Controller speed_left_pid;
static PID_Controller speed_right_pid;

static volatile uint8_t car_state = CAR_STATE_STOPPED;
static volatile uint8_t car_start_pending = 0U;
static volatile uint8_t car_stop_pending = 0U;
static volatile uint8_t teach_run = 0U; /* 阶段1：学习及未保存缓存，到终点仅停车 */
static volatile uint8_t map_save_pending = 0U;
static volatile uint8_t teach_ready_notice = 0U;
static volatile uint8_t teach_error_notice = 0U;
static volatile uint8_t teach_abort_requested = 0U; /* 主循环或急停中断置位 */
static volatile uint32_t map_save_tick = 0U;
static uint32_t last_save_sequence = 0U; /* 仅TIM1消费，数字2不用等主循环停车 */

/* 仅在 TIM1 中断中访问。 */
static uint8_t line_is_lost = 1U;
static uint8_t avoidance_was_active = 0U;
static uint8_t recovery_was_active = 0U;
static uint8_t endpoint_inhibit_ticks = 0U;
static TeachFinishDetector teach_finish;
static LineGuard return_guard;
static volatile uint8_t return_pending = 0U;
static volatile uint8_t line_verify_active = 0U;
static uint16_t return_ticks = 0U;
static volatile uint8_t return_failure_notice = 0U;
static volatile uint8_t recovery_path_blocked = 1U;
static volatile uint32_t recovery_sensor_tick = 0U;

volatile float left_output;
volatile float right_output;
volatile int16_t speed_left;
volatile int16_t speed_right;
volatile uint8_t bitmap;
static uint32_t last_oled_tick = 0U;
static uint8_t oled_row = 0U;

static uint8_t Car_LineCandidate(uint8_t bits, float angle_limit)
{
  return LineGuard_Candidate(bits, Odom_MapReady(), Odom_GetMapDistanceCm(),
                              Odom_GetHeadingErrDeg(), angle_limit);
}

/* 停车后选择模式。临界区防止手推标定的TIM1更新看到一半初始化的学习缓存。
 * Flash保存不在临界区执行。 */
static void Car_SetTeachMode(uint8_t enabled)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (enabled)
  {
    teach_abort_requested = 0U;
    teach_error_notice = 0U;
    Odom_StartTeach();
  }
  else
    Odom_CancelTeach();
  teach_run = enabled;
  __set_PRIMASK(primask);
}

/* 主循环打印；错误后缓存和位姿冻结，方便区分丢线/容量/坐标故障。 */
static void Car_SendTeachStatus(const char *prefix)
{
  char msg[64];
  float cm = Odom_GetDistCm();
  snprintf(msg, sizeof(msg), "%s %s N%u K%lu GS%02X\r\n", prefix,
           Odom_GetTeachErrorName(), (unsigned)Odom_GetMapCount(),
           (unsigned long)(cm > 0.0f ? cm : 0.0f), (unsigned)bitmap);
  Bluetooth_SendString(msg);
}

/* 故障停车后的最近640ms位图，分行发送，避免9600波特率超过单次50ms超时。 */
static void Car_SendTeachTrace(void)
{
  static const char hex[] = "0123456789ABCDEF";
  uint8_t frames[TEACH_FINISH_TRACE_SIZE];
  uint8_t n = TeachFinish_CopyTrace(&teach_finish, frames);
  for (unsigned i = 0U; i < n; i += 16U)
  {
    char msg[40] = "GS ";
    unsigned pos = 3U;
    for (unsigned j = i; j < n && j < i + 16U; j++)
    {
      msg[pos++] = hex[frames[j] >> 4];
      msg[pos++] = hex[frames[j] & 15U];
    }
    msg[pos++] = '\r'; msg[pos++] = '\n'; msg[pos] = '\0';
    Bluetooth_SendString(msg);
  }
}

static void Car_SendFinishStatus(void)
{
  char msg[48];
  snprintf(msg, sizeof(msg), "END REJ%02X P%lu L%lu N%u\r\n", (unsigned)teach_finish.reject_flags,
           (unsigned long)(teach_finish.rejected_progress_cm > 0.0f ? teach_finish.rejected_progress_cm : 0.0f),
           (unsigned long)(teach_finish.route_length_cm > 0.0f ? teach_finish.route_length_cm : 0.0f),
           (unsigned)teach_finish.reject_count);
  Bluetooth_SendString(msg);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();

  Bluetooth_Init();
  Motor_Init();
  Encoder_Init();
  Odom_Init();
  OLED_Init();
  Ultrasonic_Init();
  Avoidance_Init();

  PID_Init(&pos_pid, 0.07f, 0.5f, 0.0f, -40.0f, 40.0f); // ki = 0.5f
  PID_SetIntegralLimit(&pos_pid, -250.0f, 250.0f);
  PID_Init(&speed_left_pid, 3.0f, 5.0f, 0.0f, -75.0f, 75.0f); // ki = 5.0f
  PID_SetIntegralLimit(&speed_left_pid, -150.0f, 150.0f);
  PID_Init(&speed_right_pid, 3.0f, 5.0f, 0.0f, -75.0f, 75.0f); // ki = 5.0f
  PID_SetIntegralLimit(&speed_right_pid, -150.0f, 150.0f);

  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 超声波驱动是非阻塞状态机，主循环需要尽可能频繁地调用。 */
    Ultrasonic_Process();

    /* 先发停车请求再清UART锁存；TIM1即使在此期间触发也不可能恢复运动。
     * 数字0已在接收回调锁存，消息显示的时间不等于电机实际开始刹车时间。 */
    if (Bluetooth_EmergencyStopLatched())
    {
      Car_RequestStop();
      map_save_pending = teach_ready_notice = 0U;
      if (teach_run) teach_abort_requested = 1U;
      Bluetooth_ClearEmergencyStop();
      Bluetooth_SendString("OK STOP\r\n");
    }

    if (return_failure_notice != 0U)
    {
      return_failure_notice = 0U;
      Bluetooth_SendString("RETURN FAIL NOT VERIFIED\r\n");
    }

    if (teach_ready_notice != 0U)
    {
      char msg[48];
      teach_ready_notice = 0U;
      snprintf(msg, sizeof(msg), "TEACH FINISH GS%02X K%lu N%u\r\n", (unsigned)teach_finish.hit_bitmap,
               (unsigned long)Odom_GetDistCm(), (unsigned)Odom_GetMapCount());
      Bluetooth_SendString(msg);
      Bluetooth_SendString("TEACH READY SEND 2 TO SAVE\r\n");
    }
    if (teach_error_notice != 0U)
    {
      teach_error_notice = 0U;
      Car_SendTeachStatus("TEACH FAIL");
      Car_SendTeachTrace();
      Car_SendFinishStatus();
    }
    if (teach_abort_requested != 0U && car_state == CAR_STATE_STOPPED && car_stop_pending == 0U &&
        car_start_pending == 0U && map_save_pending == 0U && teach_run != 0U)
    {
      Car_SetTeachMode(0U);
      teach_abort_requested = 0U;
      Bluetooth_SendString("TEACH CANCELLED KEEP OLD\r\n");
    }

    BluetoothCmd bt_cmd;

    while ((bt_cmd = Bluetooth_GetCommand()) != BT_CMD_NONE)
    {
      switch (bt_cmd)
      {
      case BT_CMD_START:
        if (teach_run != 0U)
          Bluetooth_SendString("ERR SEND 2 TO SAVE OR 0 TO ABORT\r\n");
        else if (Car_CanStart())
        {
          if (!Odom_MapReady())
          {
            Bluetooth_SendString("ERR NO MAP SEND 1 TO TEACH\r\n");
            break;
          }
          Car_PrepareStart();
          Car_RequestStart();
          Bluetooth_SendString("OK START MAP\r\n");
        }
        else
        {
          Bluetooth_SendString("ERR BUSY\r\n");
        }
        break;

      case BT_CMD_TEACH:
        if (!ODOM_ENABLE)
          Bluetooth_SendString("ERR MAP DISABLED\r\n");
        else if (teach_run != 0U)
          Bluetooth_SendString("ERR TEACH EXISTS SAVE 2 OR ABORT 0\r\n");
        else if (Car_CanStart())
        {
          /* 阶段1：先移走障碍物，只学习到RAM，不自动写Flash。 */
          Avoidance_Reset();
          Car_SetTeachMode(1U);
          Car_RequestStart();
          Bluetooth_SendString((TEACH_FINISH_EXPECTED_CM > 0.0f || Odom_GetStoredLengthCm() > 0.0f)
                               ? "OK TEACH\r\n" : "OK TEACH MANUAL END SEND 2\r\n");
        }
        else
        {
          Bluetooth_SendString("ERR BUSY\r\n");
        }
        break;

      case BT_CMD_SAVE:
        if (map_save_pending != 0U)
          Bluetooth_SendString("OK SAVE PENDING\r\n");
        else if (teach_run == 0U || Odom_GetMode() != ODOM_MODE_TEACH)
          Bluetooth_SendString("ERR NO TEACH DATA\r\n");
        else if (car_start_pending != 0U || car_stop_pending != 0U)
          Bluetooth_SendString("ERR WAIT\r\n");
        else if (!Odom_TeachValid())
          Car_SendTeachStatus("ERR TEACH");
        else if (Odom_GetDistCm() < ODOM_TEACH_MIN_CM)
          Bluetooth_SendString("ERR TEACH TOO SHORT MIN 100CM\r\n");
        else
        {
          /* 阶段2：允许手动结束完整学习；也允许到终点停车后再发送。 */
          map_save_pending = 1U;
          map_save_tick = HAL_GetTick();
          Car_RequestStop();
          Bluetooth_SendString("OK STOPPING TO SAVE\r\n");
        }
        break;

      case BT_CMD_MAP:
      {
        char msg[40];

        if (Odom_GetMode() != ODOM_MODE_NONE)
          snprintf(msg, sizeof(msg), "MAP %u STEP %u MODE %u R %u\r\n",
                   Odom_GetMapCount(), Odom_GetMapStepCm(), (unsigned)Odom_GetMode(),
                   line_verify_active ? 4U : (unsigned)Odom_GetRecoveryState());
        else
          snprintf(msg, sizeof(msg), "MAP NONE\r\n");
        Bluetooth_SendString(msg);
        if (teach_run != 0U) Car_SendTeachStatus("TEACH");
        Car_SendFinishStatus();
        break;
      }

      case BT_CMD_STOP:
        map_save_pending = 0U; /* 已排队但尚未开始的保存可以取消 */
        teach_ready_notice = 0U;
        Car_RequestStop();
        if (teach_run != 0U)
        {
          teach_abort_requested = 1U;
          /* 等TIM1真正停车后才恢复旧图，不和正在写学习缓存的中断并发。 */
          Bluetooth_SendString("OK STOP TEACH ABORT\r\n");
        }
        else
        {
          Bluetooth_SendString("OK STOP\r\n");
        }
        break;

      default:
        Bluetooth_SendString("ERR\r\n");
        break;
      }
    }

    /* 先处理收到的命令再执行保存：排队期间收到'0'可取消写Flash。 */
    if (map_save_pending != 0U && car_state != CAR_STATE_RUNNING &&
        car_stop_pending == 0U && car_start_pending == 0U &&
        HAL_GetTick() - map_save_tick >= 150U)
    {
      char msg[40];
      uint16_t n = Odom_Save();
      if (n != 0U)
        snprintf(msg, sizeof(msg), "OK MAP %u STEP %u\r\n", n, Odom_GetMapStepCm());
      else
      {
        Car_SetTeachMode(0U);
        snprintf(msg, sizeof(msg), "ERR MAP SAVE KEEP OLD\r\n");
      }
      map_save_pending = 0U;
      teach_run = 0U;
      Bluetooth_SendString(msg);
    }

    Ultrasonic_Reading readings[ULTRASONIC_SENSOR_COUNT];
    Ultrasonic_GetAll(readings);
    /* 救援追点不具备路径规划：附近有有效障碍读数时刹停。
     * 正常避障参数和判据不变；救援用额外的保守刹停保护。 */
    uint8_t blocked = 0U;
    for (unsigned i = 0U; i < ULTRASONIC_SENSOR_COUNT; i++)
    {
      uint16_t limit = i == ULTRASONIC_CENTER ? AVOIDANCE_CENTER_TRIGGER_MM : AVOIDANCE_SIDE_TRIGGER_MM;
      if (readings[i].valid && readings[i].age_ms <= AVOIDANCE_SENSOR_MAX_AGE_MS &&
          readings[i].filtered_mm <= limit) blocked = 1U;
    }
    recovery_path_blocked = blocked;
    recovery_sensor_tick = HAL_GetTick();

    if ((car_state == CAR_STATE_RUNNING) && (teach_run == 0U))
    {
      uint8_t current_bitmap = bitmap;

      Avoidance_ProcessSensors(readings, current_bitmap, Track_GetPositionFromBitmap(current_bitmap));
    }

    if ((HAL_GetTick() - last_oled_tick) >= OLED_ROW_PERIOD_MS)
    {
      last_oled_tick = HAL_GetTick();

      switch (oled_row)
      {
      /* 电机占空比显示 */
      case 0:
        OLED_ShowString(0, 0, "PWML:", 8);
        OLED_ShowSignedNum(30, 0, (int32_t)left_output, 3, 8);

        OLED_ShowString(64, 0, "PWMR:", 8);
        OLED_ShowSignedNum(94, 0, (int32_t)right_output, 3, 8);
        break;

      /* 电机速度显示 */
      case 1:
        OLED_ShowString(0, 1, "SPL:", 8);
        OLED_ShowSignedNum(24, 1, (int32_t)speed_left, 4, 8);

        OLED_ShowString(64, 1, "SPR:", 8);
        OLED_ShowSignedNum(88, 1, (int32_t)speed_right, 4, 8);
        break;

      /* 灰度循迹显示 */
      case 2:
        OLED_ShowString(0, 2, "GS:", 8);
        OLED_ShowNum(12, 2, (int32_t)bitmap, 3, 8);
        break;

      /* 超声波测距显示 */
      case 3:
        OLED_ShowString(0, 3, "UL:", 8);
        if (readings[ULTRASONIC_LEFT].valid)
          OLED_ShowNum(18, 3, readings[ULTRASONIC_LEFT].filtered_mm, 4, 8);
        else
          OLED_ShowString(18, 3, "----", 8);
        OLED_ShowString(48, 3, "mm", 8);
        break;

      case 4:
        OLED_ShowString(0, 4, "UC:", 8);
        if (readings[ULTRASONIC_CENTER].valid)
          OLED_ShowNum(18, 4, readings[ULTRASONIC_CENTER].filtered_mm, 4, 8);
        else
          OLED_ShowString(18, 4, "----", 8);
        OLED_ShowString(48, 4, "mm", 8);
        break;

      case 5:
        OLED_ShowString(0, 5, "UR:", 8);
        if (readings[ULTRASONIC_RIGHT].valid)
          OLED_ShowNum(18, 5, readings[ULTRASONIC_RIGHT].filtered_mm, 4, 8);
        else
          OLED_ShowString(18, 5, "----", 8);
        OLED_ShowString(48, 5, "mm", 8);
        break;

      /* D=传感器到地图线段的估计距离cm；M=无图/学习/有图；N=点数；R=救援状态 */
      case 6:
        OLED_ShowString(0, 6, "D:", 8);
        OLED_ShowSignedNum(12, 6, (int32_t)Odom_GetMapDistanceCm(), 3, 8);
        OLED_ShowString(48, 6, "M:", 8);
        OLED_ShowNum(60, 6, (uint32_t)Odom_GetMode(), 1, 8);
        OLED_ShowString(72, 6, "N:", 8);
        OLED_ShowNum(84, 6, (uint32_t)Odom_GetMapCount(), 3, 8);
        OLED_ShowString(108, 6, "R", 8);
        OLED_ShowNum(114, 6, line_verify_active ? 4U : (uint32_t)Odom_GetRecoveryState(), 1, 8);
        break;

      /* H = 累计航向(度，左转为正)  K = 累计里程(cm)；标定轮距 / 轮径用 */
      default:
        OLED_ShowString(0, 7, "H:", 8);
        OLED_ShowSignedNum(12, 7, (int32_t)Odom_GetHeadingDeg(), 4, 8);
        OLED_ShowString(64, 7, "K:", 8);
        OLED_ShowSignedNum(76, 7, (int32_t)Odom_GetDistCm(), 4, 8);
        break;
      }
      oled_row = (uint8_t)((oled_row + 1U) % 8U);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void Car_ResetControllers(void)
{
  PID_Reset(&pos_pid);
  PID_Reset(&speed_left_pid);
  PID_Reset(&speed_right_pid);
  line_is_lost = 1U;
  avoidance_was_active = 0U;
}

static void Car_RequestStart(void)
{
  car_stop_pending = 0U;
  car_start_pending = 1U;
}

static void Car_RequestStop(void)
{
  car_start_pending = 0U;
  car_stop_pending = 1U;
}

static uint8_t Car_CanStart(void)
{
  uint8_t state = car_state;

  return (uint8_t)(((state == CAR_STATE_STOPPED) || (state == CAR_STATE_FINISHED)) &&
                   (car_start_pending == 0U) && (car_stop_pending == 0U) && (map_save_pending == 0U) &&
                   !Bluetooth_EmergencyStopLatched());
}

static void Car_PrepareStart(void)
{
  Ultrasonic_Reading readings[ULTRASONIC_SENSOR_COUNT];
  uint8_t start_bitmap = Grayscale_Sensor_Read_All();

  /*
   * 发车前用最新超声波和灰度状态判断一次，避免已经进入危险距离时
   * 仍先按正常循迹加速。
   */
  bitmap = start_bitmap;
  Ultrasonic_GetAll(readings);

  Avoidance_Reset();
  Avoidance_ProcessSensors(readings, start_bitmap, Track_GetPositionFromBitmap(start_bitmap));
}

static void Car_BeginRun(void)
{
  Car_ResetControllers();
  Odom_Reset(); /* 发车点 = 地图原点，当前车头朝向 = 0° */
  recovery_was_active = 0U;
  endpoint_inhibit_ticks = 0U;
  TeachFinish_Reset(&teach_finish);
  TeachFinish_SetRouteLength(&teach_finish,
      (teach_run && TEACH_FINISH_EXPECTED_CM > 0.0f) ? TEACH_FINISH_EXPECTED_CM : Odom_GetStoredLengthCm());
  return_pending = line_verify_active = return_failure_notice = 0U;
  return_ticks = 0U;
  LineGuard_Reset(&return_guard);
  car_state = CAR_STATE_RUNNING;
}

static void Car_StopNow(void)
{
  Motor_BrakeAll(); /* 制动优先，状态清理和消息随后处理 */
  left_output = right_output = 0.0f;
  line_verify_active = 0U;
  return_pending = 0U;
  return_ticks = 0U;
  LineGuard_Reset(&return_guard);
  if (Odom_GetRecoveryState() != ODOM_RECOVERY_FAILED)
  {
    float unused_base, unused_steer;
    (void)Odom_RecoveryControl(0U, 0U, 0U, 0U, &unused_base, &unused_steer);
  }
  Car_ResetControllers();
  car_state = CAR_STATE_STOPPED;
  Avoidance_Reset();
}

static void Car_Finish(void)
{
  Motor_BrakeAll();
  left_output = right_output = 0.0f;
  if (teach_run != 0U) teach_ready_notice = 1U; /* 只通知，等待ASCII '2' */
  Car_ResetControllers();
  car_state = CAR_STATE_FINISHED;
  Avoidance_Reset();
}

static int16_t Motor_DutyFromFloat(float duty)
{
  if (duty > 100.0f)
  {
    return 100;
  }
  if (duty < -100.0f)
  {
    return -100;
  }

  return (int16_t)((duty >= 0.0f) ? (duty + 0.5f) : (duty - 0.5f));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    /* 每周期都采样，停车期间也不会累积编码器计数差。 */
    speed_left = -Encoder_GetSpeed(&encoder_left);
    speed_right = Encoder_GetSpeed(&encoder_right);

    /* 接收后的下一拍优先制动：跳过地图匹配、灰度扫描、PID及显示。 */
    if (Bluetooth_EmergencyStopLatched())
    {
      car_start_pending = car_stop_pending = map_save_pending = teach_ready_notice = 0U;
      if (teach_run) teach_abort_requested = 1U;
      Car_StopNow();
      return;
    }

    uint32_t save_sequence = Bluetooth_GetSaveSequence();
    if (save_sequence != last_save_sequence)
    {
      last_save_sequence = save_sequence;
      if (teach_run && !car_start_pending && !car_stop_pending && Odom_GetMode() == ODOM_MODE_TEACH)
      {
        /* 数字2的停车和Flash写入分开：先在本控制拍制动，主循环150ms后写图。
         * 即便数据太短/无效也先停车，稍后报告错误，不能收到保存命令还继续开。 */
        Car_StopNow();
        if (Odom_TeachValid() && Odom_GetDistCm() >= ODOM_TEACH_MIN_CM)
        {
          map_save_tick = HAL_GetTick();
          map_save_pending = 1U;
        }
        return;
      }
    }

    /* 停车时也照算，方便手推小车标定 K / H；循迹居中的判断在下面运行时才给 */
    if (car_state != CAR_STATE_RUNNING && teach_run == 0U && map_save_pending == 0U)
    {
      Odom_Update(speed_left, speed_right, 0U);
    }

    if (car_stop_pending != 0U)
    {
      car_start_pending = 0U;
      car_stop_pending = 0U;
      Car_StopNow();
      return;
    }

    if (car_start_pending != 0U)
    {
      car_start_pending = 0U;
      Car_BeginRun();
    }

    if (car_state != CAR_STATE_RUNNING)
    {
      Motor_BrakeAll();
      return;
    }

    bitmap = Grayscale_Sensor_Read_All();
    if (endpoint_inhibit_ticks > 0U) endpoint_inhibit_ticks--;

    /* 先积分本拍位姿，再用于地图候选/终点判断，不再使用上一拍的地图距离。
     * 未确认的回线、救援以及散乱黑点不能移动地图匹配锚点。 */
    int16_t raw_position = Track_GetPositionFromBitmap(bitmap);
    uint8_t map_online = (uint8_t)(teach_run == 0U && return_pending == 0U &&
        Avoidance_GetState() == AVOIDANCE_STATE_IDLE && Odom_GetRecoveryState() == ODOM_RECOVERY_IDLE &&
        LineGuard_Shape(bitmap) && raw_position >= -300 && raw_position <= 300);
    Odom_Update(speed_left, speed_right, map_online);
    Odom_ObserveLine(bitmap);

    /* 教学用已量圈长/旧图长度；比赛用地图进度，不能把绕障多走的路当作赛道进度。
     * 无参考长度时不自动判终点，首次学习必须发2手动结束。 */
    uint8_t finish_forward = (uint8_t)(speed_left > 0 && speed_right > 0);
    if (!teach_run)
      finish_forward = (uint8_t)(finish_forward && endpoint_inhibit_ticks == 0U && !return_pending &&
          Avoidance_GetState() == AVOIDANCE_STATE_IDLE && Odom_GetRecoveryState() == ODOM_RECOVERY_IDLE &&
          Odom_MapReady() && Odom_GetMapDistanceCm() <= LINE_GUARD_DISTANCE_CM &&
          fabsf(Odom_GetHeadingErrDeg()) <= LINE_GUARD_CONFIRMED_DEG);
    if (TeachFinish_Update(&teach_finish, bitmap,
          teach_run ? Odom_GetDistCm() : Odom_GetProgressCm(), finish_forward))
    {
      if (!teach_run || Odom_TeachValid()) Car_Finish();
      else { teach_error_notice = 1U; Car_StopNow(); }
      return;
    }
    if (teach_run != 0U && !Odom_TeachValid())
    {
      teach_error_notice = 1U;
      Car_StopNow();
      return;
    }

    Avoidance_Control avoidance_control = {1U, 0.0f, 0.0f};
    if (teach_run == 0U)
      avoidance_control = Avoidance_UpdateControl(bitmap, (float)speed_left, (float)speed_right,
                              Car_LineCandidate(bitmap, LINE_GUARD_APPROACH_DEG));
    float base_speed;
    float steer = 0.0f;
    uint8_t candidate = Car_LineCandidate(bitmap, LINE_GUARD_APPROACH_DEG);
    uint8_t tracking_bitmap = bitmap;
    if (!teach_run && return_pending && !candidate) tracking_bitmap = 0U;

    if (avoidance_control.use_line_pid != 0U)
    {
      if (avoidance_was_active != 0U)
      {
        PID_Reset(&pos_pid);
        line_is_lost = 1U;
      }
      avoidance_was_active = 0U;
      base_speed = TRACK_BASE_SPEED;

      if (tracking_bitmap == 0U)
      {
        if (line_is_lost == 0U)
        {
          PID_Reset(&pos_pid);
        }
        line_is_lost = 1U;

        /* 门槛之前保留原来的丢线策略；地图只作为下面的最后保障。 */
      }
      else
      {
        int16_t position = Track_GetPositionFromBitmap(tracking_bitmap);

        if (line_is_lost != 0U)
        {
          PID_Reset(&pos_pid);
        }
        line_is_lost = 0U;

        /*
         * 黑线偏右时位置为正，正转向差使左轮快、右轮慢，车向右修正；
         * 黑线偏左时位置为负，负转向差使右轮快、左轮慢。
         */
        steer = PID_UpdateError(&pos_pid, (float)position, PID_PERIOD_S);
      }
    }
    else
    {
      avoidance_was_active = 1U;
      base_speed = avoidance_control.base_speed;
      steer = avoidance_control.steer;
      return_pending = 1U; /* 动作做完不等于回到真线，第三段之后也必须验真。 */
      return_ticks = 0U;
      LineGuard_Reset(&return_guard);
    }

    uint8_t allowed = (uint8_t)(teach_run == 0U && avoidance_control.use_line_pid != 0U);
    if (allowed && Odom_GetRecoveryState() != ODOM_RECOVERY_IDLE) return_pending = 1U;
    uint8_t confirmed = 0U;
    line_verify_active = 0U;
    if (allowed && return_pending)
    {
      if (++return_ticks >= LINE_GUARD_MAX_TICKS)
      {
        return_failure_notice = 1U;
        Car_StopNow();
        return;
      }
      confirmed = LineGuard_Update(&return_guard, candidate,
                      (uint8_t)(speed_left > 0 && speed_right > 0), Odom_GetStepCm(),
                      Odom_GetHeadingErrDeg());
      line_verify_active = (uint8_t)(candidate && !confirmed);
    }

    float recovery_base, recovery_steer;
    uint8_t trusted = return_pending ? confirmed : Car_LineCandidate(bitmap, LINE_GUARD_CONFIRMED_DEG);
    uint8_t recovering = Odom_RecoveryControl(trusted, allowed, line_verify_active,
                          (uint8_t)(return_pending && !candidate), &recovery_base, &recovery_steer);
    if (recovering && !return_pending)
    {
      return_pending = 1U;
      return_ticks = 0U;
      LineGuard_Reset(&return_guard);
    }
    uint8_t return_controlled = (uint8_t)(allowed && (recovering || return_pending));
    if (return_controlled || recovery_was_active)
    {
      endpoint_inhibit_ticks = 50U; /* 救援横扫入线后500ms，不把全黑误当成终点 */
      if (return_controlled != recovery_was_active)
      {
        PID_Reset(&pos_pid);
        PID_Reset(&speed_left_pid);
        PID_Reset(&speed_right_pid);
      }
    }
    recovery_was_active = return_controlled;
    if (Odom_GetRecoveryState() == ODOM_RECOVERY_FAILED ||
        (return_controlled && (recovery_path_blocked || HAL_GetTick() - recovery_sensor_tick > AVOIDANCE_SENSOR_MAX_AGE_MS)))
    {
      return_failure_notice = 1U;
      Car_StopNow();
      return;
    }
    if (line_verify_active)
    {
      /* 真正移动着验证，不在一个字上停车计时；沿灰度纠偏并参考地图切向。 */
      base_speed = LINE_GUARD_SPEED;
      steer += LINE_GUARD_HEADING_KP * Odom_GetHeadingErrDeg();
      if (steer > LINE_GUARD_STEER_MAX) steer = LINE_GUARD_STEER_MAX;
      if (steer < -LINE_GUARD_STEER_MAX) steer = -LINE_GUARD_STEER_MAX;
    }
    else if (recovering)
    {
      base_speed = recovery_base;
      steer = recovery_steer;
      if (base_speed == 0.0f && steer == 0.0f)
      {
        Motor_BrakeAll(); /* 正在确认压线，不能让速度环积分继续推车 */
        return;
      }
    }
    if (allowed && return_pending && confirmed && !recovering)
    {
      return_pending = 0U;
      return_ticks = 0U;
      LineGuard_Reset(&return_guard);
      endpoint_inhibit_ticks = 50U;
    }

    /* 仅已知终点附近且正常在线时减速；不动避障/回线速度，不因未知圈长盲目减速。 */
    if (avoidance_control.use_line_pid && !return_controlled && finish_forward && LineGuard_Shape(bitmap) &&
        Finish_ApproachGate(teach_run ? Odom_GetDistCm() : Odom_GetProgressCm(), teach_finish.route_length_cm) &&
        base_speed > FINISH_APPROACH_SPEED)
      base_speed = FINISH_APPROACH_SPEED;

    float left_target = base_speed + steer;
    float right_target = base_speed - steer;

    left_output = PID_Update(&speed_left_pid, left_target, (float)speed_left, PID_PERIOD_S);
    right_output = PID_Update(&speed_right_pid, right_target, (float)speed_right, PID_PERIOD_S);

    Motor_SetSpeed(MOTOR_LEFT, MOTOR_START, Motor_DutyFromFloat(left_output));
    Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, Motor_DutyFromFloat(right_output));
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
