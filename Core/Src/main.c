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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "led.h"
#include "motor.h"
#include "oled_software_i2c.h"
#include "avoid.h"
#include "track.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 目标直行速度（单位：脉冲/10ms，取决于车速，需实车标定）
 * 例：车轮输出轴 1 圈/秒 ≈ 1040 脉冲/秒 ≈ 10.4 脉冲/10ms */
#define TRACK_SPEED_NORMAL 25   /* 正常循迹目标速度 */
#define TRACK_SPEED_SLOW   10   /* 避障减速目标速度 */

/* 测试目标速度（脉冲/10ms，约 2 圈/秒） */
#define MOTOR_TEST_SPEED 20

/* 超声波调试显示：1 = OLED 显示左右测距（cm）+ 原始回波宽度（µs），
 * 方便核对读数是否正常；调试完改 0 即可关闭（关闭后不占用主循环时间） */
#define OLED_ULTRASONIC_DEBUG 1

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ==================== PID 循迹控制（TIM1 中断里跑） ==================== */
static PID_t pid_pos;      /* 位置环：循迹偏差 -> 转向速度差 */
static PID_t pid_speed_l;  /* 左轮速度环 */
static PID_t pid_speed_r;  /* 右轮速度环 */

/* 控制状态（主循环写，TIM1 中断读，volatile 保证可见性） */
volatile uint8_t g_track_enable = 0;   /* 1=中断循迹，0=避障接管 */
volatile uint8_t g_stop = 0;          /* 终止停车标志：8 路全黑触发，置 1 后永久停止 */
volatile int16_t g_target_speed = 20;  /* 目标直行速度（脉冲/10ms） */

/* 避障接管（主循环写，TIM1 中断读）：
 * g_avoid_drive=1 时中断跳过循迹位置环，直接用 g_avoid_vl/vr 作为
 * 左右轮目标速度（弧线由速度环闭环执行，转弯半径不受电池电压影响） */
volatile uint8_t g_avoid_drive = 0;
volatile int16_t g_avoid_vl = 0;
volatile int16_t g_avoid_vr = 0;

/* 调试观测值（中断写，主循环读，OLED 显示用） */
volatile int16_t g_speed_l = 0;  /* 左轮实测速度（脉冲/10ms） */
volatile int16_t g_speed_r = 0;  /* 右轮实测速度（脉冲/10ms） */
volatile int16_t g_pwm_l = 0;    /* 左轮 PWM 输出（-100~100） */
volatile int16_t g_pwm_r = 0;    /* 右轮 PWM 输出（-100~100） */

#if OLED_ULTRASONIC_DEBUG
/* 超声波调试显示：一页显示 L/R 两路的 距离(cm) + 原始回波宽度(µs)。
 * 距离显示 "---" = 无目标/-1；宽度是硬件捕获的 Echo 高电平时间，
 * 可以和尺子对照标定（1cm ≈ 58µs，超量程的 66ms 脉冲会显示 65535） */
static void OLED_ShowSonarLine(uint8_t y, uint8_t tag, int16_t cm, uint16_t width_us)
{
    OLED_ClearCharArea(0, y, 21); /* 先清整行，避免新旧数字叠在一起 */

    OLED_ShowChar(0, y, tag, 8);
    OLED_ShowChar(6, y, ':', 8);

    if (cm < 0)
        OLED_ShowString(12, y, (uint8_t *)"---", 8);
    else
        OLED_ShowNum(12, y, (uint32_t)cm, 3, 8);

    OLED_ShowString(30, y, (uint8_t *)"cm", 8);
    OLED_ShowString(54, y, (uint8_t *)"W:", 8);
    OLED_ShowNum(66, y, (uint32_t)width_us, 5, 8);
    OLED_ShowString(96, y, (uint8_t *)"us", 8);
}

static uint32_t s_oled_tick = 0; /* 显示刷新计时 */
#endif

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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  Encoder_Init();
  OLED_Init();

#if OLED_ULTRASONIC_DEBUG
  /* 标题只画一次，下面几行循环里局部刷新 */
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"SONAR DEBUG", 8);
#endif

  Avoid_Init();
  Track_Init();

  /* TIM1 定时中断：10ms PID 控制周期 */
  MX_TIM1_Init();
  HAL_TIM_Base_Start_IT(&htim1);

  /* 超声波 Echo 输入捕获（TIM1 CH3/CH4）：必须在 TIM1 启动之后使能 */
  HCSR04_Start();

  /* PID 参数初始化（需实车调，见各环注释） */
  PID_Init(&pid_pos,     0.05f, 0.0f, 0.0f, 500.0f, -30.0f, 30.0f);   /* 位置环：纯 P，限幅防边缘掉头 */
  PID_Init(&pid_speed_l, 3.0f,  0.05f, 0.0f, 300.0f, -100.0f, 100.0f); /* 左速度环 */
  PID_Init(&pid_speed_r, 3.0f,  0.05f, 0.0f, 300.0f, -100.0f, 100.0f); /* 右速度环 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (g_stop)
    {
        /* 终止停车已触发：电机保持刹车状态，不再做任何控制 */
#if OLED_ULTRASONIC_DEBUG
        /* 把停车原因显示出来：看到 STOP 说明是 8 路全黑的停车保护触发了 */
        static uint8_t stop_shown = 0U;
        if (!stop_shown)
        {
            stop_shown = 1U;
            OLED_ClearCharArea(0, 6, 21);
            OLED_ShowString(0, 6, (uint8_t *)"STOP ALLBLACK", 8);
        }
#endif
        HAL_Delay(10);
        continue;
    }

    /* ----- 避障决策（非阻塞；循迹 PID 和避障速度环都在 TIM1 中断里跑） ----- */
    switch (Avoid_Update())
    {
    case AVOID_AVOIDING: /* 绕行中：右前/左前弧线交给速度环执行 */
        g_track_enable = 1;
        g_avoid_vl = Avoid_GetDriveL();
        g_avoid_vr = Avoid_GetDriveR();
        g_avoid_drive = 1;
        break;

    case AVOID_SLOW: /* 循迹但减速 */
        g_avoid_drive = 0;
        g_track_enable = 1;
        g_target_speed = TRACK_SPEED_SLOW;
        break;

    case AVOID_FOLLOW: /* 正常循迹 */
    default:
        g_avoid_drive = 0;
        g_track_enable = 1;
        g_target_speed = TRACK_SPEED_NORMAL;
        break;
    }

#if OLED_ULTRASONIC_DEBUG
    /* 超声波调试显示：每 200ms 刷新一次（软件 I2C 阻塞约几毫秒，仅调试用） */
    if ((HAL_GetTick() - s_oled_tick) >= 200U)
    {
        int16_t front;
        uint8_t act_char;

        s_oled_tick = HAL_GetTick();

        OLED_ShowSonarLine(1, 'L', HCSR04_LastCm(HCSR04_LEFT),
                           HCSR04_LastWidthUs(HCSR04_LEFT));
        OLED_ShowSonarLine(2, 'R', HCSR04_LastCm(HCSR04_RIGHT),
                           HCSR04_LastWidthUs(HCSR04_RIGHT));

        /* 第三行：前方最小值 + 避障动作（F=循迹 S=减速 A=绕行） */
        switch (Avoid_GetAction())
        {
        case AVOID_AVOIDING: act_char = 'A'; break;
        case AVOID_SLOW:     act_char = 'S'; break;
        default:             act_char = 'F'; break;
        }

        front = Avoid_GetFrontCm();
        OLED_ClearCharArea(0, 4, 21);
        OLED_ShowString(0, 4, (uint8_t *)"F:", 8);
        if (front < 0)
            OLED_ShowString(12, 4, (uint8_t *)"---", 8);
        else
            OLED_ShowNum(12, 4, (uint32_t)front, 3, 8);
        OLED_ShowString(30, 4, (uint8_t *)"cm", 8);
        OLED_ShowString(54, 4, (uint8_t *)"A:", 8);
        OLED_ShowChar(66, 4, act_char, 8);

        /* 第五行：避障内部阶段号（调试）
         * 0=循迹 1=右让 2=左让 3=右让后回线 4=左让后回线 5=冷却 */
        OLED_ClearCharArea(0, 5, 21);
        OLED_ShowString(0, 5, (uint8_t *)"P:", 8);
        OLED_ShowChar(12, 5, (uint8_t)('0' + Avoid_GetState()), 8);
    }
#endif

    /* 主循环节拍（5ms）：超声波按 40ms 轮流采样、避障状态按毫秒计时，
     * 各任务用自己的时间片，不用把整个循环拖满 */
    HAL_Delay(5);
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
/* ==================== 循迹 PID 控制（TIM1 每 10ms 中断一次） ==================== */
/* 双环结构：
 *   外环（位置环）：循迹偏差 pos -> 转向速度差 turn
 *   内环（速度环）：左右轮目标速度 vs 编码器实测速度 -> PWM
 *
 * 注意编码器方向：前进时 encoder_left / encoder_right 必须都返回正值。
 * 如果某个轮子前进时速度为负（安装方向反了），把对应编码器 A/B 相对调，
 * 或在 Encoder_GetSpeed 后取反。方向错了速度环会发散（飞转/堵转）。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1)
        return;

    if (g_stop)
        return;  /* 已触发终止停车，不再控制 */

    if (!g_track_enable)
        return;  /* 避障接管，中断不控电机 */

    /* 终止条件：8 路全部压线（黑块/终点线）-> 刹车停车。
     * 只在非避障时检测：绕行过程中车身会横扫黑线，出现"黑线横在整排
     * 传感器下面"的姿态会被误判成终点线，一旦误停就是永久刹车 */
    if ((g_avoid_drive == 0U) && Track_AllBlack())
    {
        g_stop = 1;
        g_track_enable = 0;
        Motor_BrakeAll();
        return;
    }

    /* 1. 读编码器速度（脉冲/10ms，带符号） */
    int16_t vL = -Encoder_GetSpeed(&encoder_left);
    int16_t vR = Encoder_GetSpeed(&encoder_right);

    float vL_target;
    float vR_target;

    if (g_avoid_drive)
    {
        /* 避障接管：跳过位置环，直接用避障给的左右轮目标速度 */
        vL_target = (float)g_avoid_vl;
        vR_target = (float)g_avoid_vr;
    }
    else
    {
        /* 2. 读循迹位置偏差（-1000 ~ +1000） */
        int16_t pos = Track_GetPosition();

        /* 3. 位置环：偏差 -> 转向速度差（脉冲/10ms） */
        float turn = PID_Update(&pid_pos, (float)pos);

        /* 4. 目标速度 = 直行速度 ± 转向（线偏右则左快右慢，向右纠偏） */
        vL_target = (float)g_target_speed + turn;
        vR_target = (float)g_target_speed - turn;
    }

    /* 5. 速度环：目标 vs 实测 -> PWM 占空比 */
    float pwm_l = PID_Update(&pid_speed_l, vL_target - (float)vL);
    float pwm_r = PID_Update(&pid_speed_r, vR_target - (float)vR);

    /* 记录观测值（供 OLED 显示 / 调试观察） */
    g_speed_l = vL;
    g_speed_r = vR;
    g_pwm_l = (int16_t)pwm_l;
    g_pwm_r = (int16_t)pwm_r;

    /* 6. 更新电机 */
    Motor_SetSpeed(MOTOR_LEFT,  MOTOR_START, (int16_t)pwm_l);
    Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, (int16_t)pwm_r);
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
