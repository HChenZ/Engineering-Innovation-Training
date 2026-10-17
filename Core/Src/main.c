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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TRACK_BASE_SPEED 25.0f
#define PID_PERIOD_S 0.01f

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

static PID_Controller pos_pid;
static PID_Controller speed_left_pid;
static PID_Controller speed_right_pid;

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

  HAL_TIM_Base_Start_IT(&htim1);

  PID_Init(&pos_pid, 0.07f, 0.5f, 0.0f, -40.0f, 40.0f);
  PID_SetIntegralLimit(&pos_pid, -250.0f, 250.0f);
  PID_Init(&speed_left_pid, 3.0f, 5.0f, 0.0f, -100.0f, 100.0f);
  PID_SetIntegralLimit(&speed_left_pid, -150.0f, 150.0f);
  PID_Init(&speed_right_pid, 3.0f, 5.0f, 0.0f, -100.0f, 100.0f);
  PID_SetIntegralLimit(&speed_right_pid, -150.0f, 150.0f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---------- 编码器使用示例（在定时中断中调用） ---------- */
    // int16_t speed_left = Encoder_GetSpeed(&encoder_left);
    // int16_t speed_right = Encoder_GetSpeed(&encoder_right);

    /* ---------- 蓝牙使用示例：启动 / 停车 ---------- */
    // BluetoothCmd bt_cmd;

    // while ((bt_cmd = Bluetooth_GetCommand()) != BT_CMD_NONE)
    // {
    //     switch (bt_cmd)
    //     {
    //     case BT_CMD_START: /* 手机发 start / s / 1 */
    //         /* !g_run：待机中重新起步；g_stop：终点停车后想再跑一轮
    //          * （终点停车时 g_run 仍然是 1，所以必须把 g_stop 也算进来，
    //          *   否则这里会走下面的 "OK RUNNING" 分支，车永远起不来） */
    //         if (!g_run || g_stop)
    //         {
    //             g_stop = 0; /* 解除终点停车 */
    //             /* 清 PID 内部状态，避免上次跑完的积分残留让车"窜"一下 */
    //             PID_Reset(&pid_pos);
    //             PID_Reset(&pid_speed_l);
    //             PID_Reset(&pid_speed_r);
    //             /* 这里不直接使能电机：本轮往下就会调用 Avoid_Update()，
    //              * 先用最新距离判一次——车头已经贴障碍物就先刹车/绕行，
    //              * 不会带着速度往前冲（停车期间距离一直在更新） */
    //             g_run = 1;
    //             car_cmd_text = "CMD:START";
    //             Car_ShowAll(); /* 立刻刷新状态行 / LED */
    //             Bluetooth_SendString("OK START\r\n");
    //         }
    //         else
    //         {
    //             Bluetooth_SendString("OK RUNNING\r\n"); /* 已经在跑，忽略重复命令 */
    //         }
    //         break;

    //     case BT_CMD_STOP: /* 手机发 stop / p / 0 */
    //         g_run = 0;
    //         g_track_enable = 0;
    //         Motor_BrakeAll();
    //         car_cmd_text = "CMD:STOP";
    //         Car_ShowAll();
    //         Bluetooth_SendString("OK STOP\r\n");
    //         break;

    //     default: /* BT_CMD_UNKNOWN */
    //         Bluetooth_SendString("ERR\r\n");
    //         break;
    //     }
    // }
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
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
