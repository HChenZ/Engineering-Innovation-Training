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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
static void Track_Control(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  Avoid_Init();
  Track_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ----- 编码器使用示例（在定时中断中调用） ----- */
    int16_t speed_left = Encoder_GetSpeed(&encoder_left);
    int16_t speed_right = Encoder_GetSpeed(&encoder_right);
    /* ----- 超声波避障 ----- */
    switch (Avoid_Update())
    {
    case AVOID_FOLLOW: /* 正常循迹，交还给寻迹逻辑 */
        Track_Control();
        break;

    case AVOID_SLOW: /* 循迹但减速 */
        break;

    case AVOID_BRAKE: /* 立即刹车 */
        Motor_BrakeAll();
        break;

    case AVOID_TURN_LEFT: /* 原地左转 */
        Motor_SetSpeed(MOTOR_LEFT, MOTOR_START, -40);
        Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, 40);
        break;

    case AVOID_TURN_RIGHT: /* 原地右转 */
        Motor_SetSpeed(MOTOR_LEFT, MOTOR_START, 40);
        Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, -40);
        break;

    case AVOID_BACK: /* 后退（默认关掉了，一般不会走到） */
        Motor_SetSpeed(MOTOR_LEFT, MOTOR_START, -40);
        Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, -40);
        break;
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
/* ==================== 循迹比例控制 ==================== */
/* 循迹基础速度（占空比 0~100），按实车调 */
#define TRACK_BASE_SPEED 40
/* 转向增益：偏差满量程(1000)时对应的左右轮速度差 */
#define TRACK_KP         40

/* 循迹差速控制：根据位置偏差做比例转向。
 * pos > 0（线偏右）-> 左轮快、右轮慢 -> 右转，反之左转 */
static void Track_Control(void)
{
    int16_t pos = Track_GetPosition();
    int16_t steer = (int16_t)((int32_t)pos * TRACK_KP / TRACK_POS_MAX);
    int16_t left = TRACK_BASE_SPEED + steer;
    int16_t right = TRACK_BASE_SPEED - steer;

    /* 限幅到 0~100，避免负值（倒退可能被判逆行） */
    if (left < 0)
        left = 0;
    if (right < 0)
        right = 0;
    if (left > 100)
        left = 100;
    if (right > 100)
        right = 100;

    Motor_SetSpeed(MOTOR_LEFT, MOTOR_START, left);
    Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, right);
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
