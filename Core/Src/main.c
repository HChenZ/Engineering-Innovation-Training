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
#include "bluetooth.h"
#include "delay_us.h"
#include "encoder.h"
#include "greyscale_sensor.h"
#include "led.h"
#include "motor.h"
#include "oled_software_i2c.h"
#include "pid.h"
#include "tracking.h"
#include "ultrasonic.h"

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

#define TRACK_BASE_SPEED 25.0f
#define PID_PERIOD_S 0.01f
#define TRACK_ALL_BLACK_BITMAP 0xFFU

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static PID_Controller pos_pid;
static PID_Controller speed_left_pid;
static PID_Controller speed_right_pid;

static volatile uint8_t car_state = CAR_STATE_STOPPED;
static volatile uint8_t car_start_pending = 0U;
static volatile uint8_t car_stop_pending = 0U;
static volatile uint8_t car_endpoint_armed = 0U;

/* 仅在 TIM1 中断中访问。 */
static uint8_t line_is_lost = 1U;

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
  OLED_Init();
  Ultrasonic_Init();

  PID_Init(&pos_pid, 0.07f, 0.0f, 0.0f, -40.0f, 40.0f); // ki = 0.5f
  PID_SetIntegralLimit(&pos_pid, -250.0f, 250.0f);
  PID_Init(&speed_left_pid, 3.0f, 0.0f, 0.0f, -100.0f, 100.0f); // ki = 5.0f
  PID_SetIntegralLimit(&speed_left_pid, -150.0f, 150.0f);
  PID_Init(&speed_right_pid, 3.0f, 0.0f, 0.0f, -100.0f, 100.0f); // ki = 5.0f
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

    BluetoothCmd bt_cmd;

    while ((bt_cmd = Bluetooth_GetCommand()) != BT_CMD_NONE)
    {
      switch (bt_cmd)
      {
      case BT_CMD_START:
        if (Car_CanStart())
        {
          Car_RequestStart();
          Bluetooth_SendString("OK START\r\n");
        }
        else
        {
          Bluetooth_SendString("OK RUNNING\r\n");
        }
        break;

      case BT_CMD_STOP:
        Car_RequestStop();
        Bluetooth_SendString("OK STOP\r\n");
        break;

      default:
        Bluetooth_SendString("ERR\r\n");
        break;
      }
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

  return (uint8_t)(((state == CAR_STATE_STOPPED) || (state == CAR_STATE_FINISHED)) && (car_start_pending == 0U) && (car_stop_pending == 0U));
}

static void Car_BeginRun(void)
{
  Car_ResetControllers();
  car_endpoint_armed = 0U;
  car_state = CAR_STATE_RUNNING;
}

static void Car_StopNow(void)
{
  Car_ResetControllers();
  car_endpoint_armed = 0U;
  car_state = CAR_STATE_STOPPED;
  Motor_BrakeAll();
}

static void Car_Finish(void)
{
  Car_ResetControllers();
  car_endpoint_armed = 0U;
  car_state = CAR_STATE_FINISHED;
  Motor_BrakeAll();
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
    int16_t speed_left = Encoder_GetSpeed(&encoder_left);
    int16_t speed_right = Encoder_GetSpeed(&encoder_right);

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

    uint8_t bitmap = Grayscale_Sensor_Read_All();

    /*
     * 启动后先等待出现非全黑的一帧，再武装终点检测。
     * 这样停在终点线上发 start 不会被下一个周期立即停车。
     */
    if (bitmap != TRACK_ALL_BLACK_BITMAP)
    {
      car_endpoint_armed = 1U;
    }
    else if (car_endpoint_armed != 0U)
    {
      Car_Finish();
      return;
    }

    float steer = 0.0f;

    if (bitmap == 0U)
    {
      if (line_is_lost == 0U)
      {
        PID_Reset(&pos_pid);
      }
      line_is_lost = 1U;
    }
    else
    {
      int16_t position = Track_GetPositionFromBitmap(bitmap);

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

    float left_target = TRACK_BASE_SPEED + steer;
    float right_target = TRACK_BASE_SPEED - steer;

    float left_output = PID_Update(&speed_left_pid, left_target, (float)speed_left, PID_PERIOD_S);
    float right_output = PID_Update(&speed_right_pid, right_target, (float)speed_right, PID_PERIOD_S);

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
