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
#include "usart.h"
#include "bluetooth.h"

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

/* 电机一致性测试模式：
 *   1 = 不循迹、不避障，左右轮速度环各自闭环到相同目标速度，
 *       用于排查电机硬件差异（看左右 PWM 是否一致）；测完改回 0 */
#define MOTOR_TEST_MODE 0
/* 测试目标速度（脉冲/10ms，约 2 圈/秒） */
#define MOTOR_TEST_SPEED 20

/* 蓝牙启动开关：
 *   1 = 上电待机，必须收到蓝牙 start 才跑（调试/发车方便）
 *   0 = 上电自动开跑，蓝牙只留着当紧急停车用
 * （比赛规则写的是"全自动运行"，若裁判对手机发车有意见就改成 0） */
#define BT_START_GATE 1

/* 编码器方向修正：前进时该轮速度应为正。
 * 若某轮编码器方向反了（速度闭环下该轮会飞转），把对应项设为 1 取反 */
#define ENC_LEFT_INVERT  1   /* 左轮已确认反了 */
#define ENC_RIGHT_INVERT 0
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

/* OLED 状态/距离显示 */
static void Car_ShowAll(void);
static void Car_ShowDistanceLine(uint8_t y, const char *tag, int16_t cm);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ==================== PID 循迹控制（TIM1 中断里跑） ==================== */
static PID_t pid_pos;      /* 位置环：循迹偏差 -> 转向速度差 */
static PID_t pid_speed_l;  /* 左轮速度环 */
static PID_t pid_speed_r;  /* 右轮速度环 */

/* 控制状态（主循环写，TIM1 中断读，volatile 保证可见性） */
volatile uint8_t g_track_enable = 0;   /* 1=中断循迹，0=避障接管 */
volatile uint8_t g_stop = 0;          /* 终止停车标志：8 路全黑触发，置 1 后停车，蓝牙再发 start 才解除 */
volatile int16_t g_target_speed = 20;  /* 目标直行速度（脉冲/10ms） */

/* 蓝牙启动控制（只在主循环里用）
 *   0 = 待机：上电默认，电机不动，等手机 APP 发 start
 *   1 = 运行：循迹 + 避障都打开 */
static uint8_t g_run = 0;

/* 终点线锁存：只有"离开过全黑区"之后，才重新允许 8 路全黑触发停车。
 * 否则车停在终点线上再发 start，会被立刻又判成终点，车根本动不了。
 * 只在 TIM1 中断里读写，主循环不碰。 */
static uint8_t finish_armed = 0;

/* OLED 显示（状态 + 两路超声波距离），每 CAR_DISP_MS 刷一遍：
 *   页 0：状态（STOP / RUN / FINISH）
 *   页 2：最近一条蓝牙命令
 *   页 4/6：车头左侧 U4、右侧 U1 的实测距离（-1 显示 --）
 * 待机/停车时距离也在实时刷新，发车前就能确认前方空间。 */
#define CAR_DISP_MS 200U
static uint32_t car_disp_tick = 0;
static const char *car_cmd_text = "WAIT CMD";

/* 调试观测值（中断写，主循环读，OLED 显示用） */
volatile int16_t g_speed_l = 0;  /* 左轮实测速度（脉冲/10ms） */
volatile int16_t g_speed_r = 0;  /* 右轮实测速度（脉冲/10ms） */
volatile int16_t g_pwm_l = 0;    /* 左轮 PWM 输出（-100~100） */
volatile int16_t g_pwm_r = 0;    /* 右轮 PWM 输出（-100~100） */

#if MOTOR_TEST_MODE
static uint32_t disp_tick = 0;   /* OLED 显示刷新计时（仅测试模式用） */
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* 蓝牙模块（USART2，9600）：先配串口，再开中断接收 */
  MX_USART2_UART_Init();
  Bluetooth_Init();

  Motor_Init();
  Encoder_Init();
  OLED_Init();
  Avoid_Init();
  Track_Init();

#if BT_START_GATE
  /* 上电默认停车待机，等蓝牙 start（LED 灭 = 没在跑） */
  Motor_StopAll();
  Car_ShowAll(); /* 状态行 + 两路距离（还没测过，先显示 --） */
#else
  /* 不用蓝牙发车：上电直接开跑（蓝牙仍可用来停车） */
  g_run = 1;
  car_cmd_text = "AUTO RUN";
  Car_ShowAll();
#endif

  /* TIM1 定时中断：10ms PID 控制周期 */
  HAL_TIM_Base_Start_IT(&htim1);

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

    /* ----- 蓝牙命令：启动 / 停车 ----- */
    BluetoothCmd bt_cmd;

    while ((bt_cmd = Bluetooth_GetCommand()) != BT_CMD_NONE)
    {
        switch (bt_cmd)
        {
        case BT_CMD_START: /* 手机发 start / s / 1 */
            /* !g_run：待机中重新起步；g_stop：终点停车后想再跑一轮
             * （终点停车时 g_run 仍然是 1，所以必须把 g_stop 也算进来，
             *   否则这里会走下面的 "OK RUNNING" 分支，车永远起不来） */
            if (!g_run || g_stop)
            {
                g_stop = 0; /* 解除终点停车 */
                /* 清 PID 内部状态，避免上次跑完的积分残留让车"窜"一下 */
                PID_Reset(&pid_pos);
                PID_Reset(&pid_speed_l);
                PID_Reset(&pid_speed_r);
                /* 这里不直接使能电机：本轮往下就会调用 Avoid_Update()，
                 * 先用最新距离判一次——车头已经贴障碍物就先刹车/绕行，
                 * 不会带着速度往前冲（停车期间距离一直在更新） */
                g_run = 1;
                car_cmd_text = "CMD:START";
                Car_ShowAll(); /* 立刻刷新状态行 / LED */
                Bluetooth_SendString("OK START\r\n");
            }
            else
            {
                Bluetooth_SendString("OK RUNNING\r\n"); /* 已经在跑，忽略重复命令 */
            }
            break;

        case BT_CMD_STOP: /* 手机发 stop / p / 0 */
            g_run = 0;
            g_track_enable = 0;
            Motor_BrakeAll();
            car_cmd_text = "CMD:STOP";
            Car_ShowAll();
            Bluetooth_SendString("OK STOP\r\n");
            break;

        default: /* BT_CMD_UNKNOWN */
            Bluetooth_SendString("ERR\r\n");
            break;
        }
    }

    /* ----- OLED 状态/距离显示：每 CAR_DISP_MS 刷一遍 ----- */
    if (HAL_GetTick() - car_disp_tick >= CAR_DISP_MS)
    {
        car_disp_tick = HAL_GetTick();
        Car_ShowAll();
    }

    if (!g_run)
    {
        /* 还没收到启动命令：电机不动，但超声波继续测距（只测不做动作），
         * 这样发车时的第一帧决策用的是当前距离，而不是停车前的旧值 */
        Avoid_Sample();
        HAL_Delay(5);
        continue;
    }

    if (g_stop)
    {
        /* 终止停车已触发：电机保持刹车状态，不再做任何控制
         * （蓝牙重新下发 start 会清掉 g_stop） */
        Avoid_Sample(); /* 同样保持测距，方便观察/重启 */
        HAL_Delay(10);
        continue;
    }

#if MOTOR_TEST_MODE
    /* ----- 电机一致性测试：固定目标速度，左右轮速度环各自闭环 ----- */
    g_track_enable = 1;
    g_target_speed = MOTOR_TEST_SPEED;
#else
    /* ----- 超声波避障决策（循迹 PID 控制在 TIM1 中断里跑） ----- */
    switch (Avoid_Update())
    {
    case AVOID_FOLLOW: /* 正常循迹 */
        g_track_enable = 1;
        g_target_speed = TRACK_SPEED_NORMAL;
        break;

    case AVOID_SLOW: /* 循迹但减速 */
        g_track_enable = 1;
        g_target_speed = TRACK_SPEED_SLOW;
        break;

    case AVOID_BRAKE: /* 立即刹车 */
        g_track_enable = 0;
        Motor_BrakeAll();
        break;

    case AVOID_TURN_LEFT: /* 原地左转 */
        g_track_enable = 0;
        Motor_SetSpeed(MOTOR_LEFT, MOTOR_START, -40);
        Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, 40);
        break;

    case AVOID_TURN_RIGHT: /* 原地右转 */
        g_track_enable = 0;
        Motor_SetSpeed(MOTOR_LEFT, MOTOR_START, 40);
        Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, -40);
        break;

    case AVOID_BACK: /* 后退（默认关掉了，一般不会走到） */
        g_track_enable = 0;
        Motor_SetSpeed(MOTOR_LEFT, MOTOR_START, -40);
        Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, -40);
        break;
    }
#endif  /* MOTOR_TEST_MODE */

#if MOTOR_TEST_MODE
    /* OLED 显示左右轮速度与 PWM（200ms 刷新，避免闪烁） */
    if (HAL_GetTick() - disp_tick >= 200)
    {
        disp_tick = HAL_GetTick();
        OLED_Clear();
        OLED_ShowString(0, 0, (uint8_t *)"L", 8);
        OLED_ShowSignedNum(16, 0, g_speed_l, 4, 8);
        OLED_ShowSignedNum(72, 0, g_pwm_l, 4, 8);
        OLED_ShowString(0, 2, (uint8_t *)"R", 8);
        OLED_ShowSignedNum(16, 2, g_speed_r, 4, 8);
        OLED_ShowSignedNum(72, 2, g_pwm_r, 4, 8);
    }
#endif

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
/* ==================== OLED 状态/距离显示 ====================
 * 页 0：状态（STOP / RUN / FINISH）
 * 页 2：最近一条蓝牙命令
 * 页 4/6：左（U4）/ 右（U1）超声波距离，-1 显示 --
 * LED 同步：跑着才亮（终点停车时中断把 g_stop 置 1，这里会自动灭）。 */

/* 画一行距离：标签 + 3 位数字 + cm */
static void Car_ShowDistanceLine(uint8_t y, const char *tag, int16_t cm)
{
    OLED_ShowString(0, y, (uint8_t *)tag, 8);

    if (cm < 0)
        OLED_ShowString(12, y, (uint8_t *)"-- ", 8); /* 无回波，补空格保持宽度 */
    else
        OLED_ShowNum(12, y, (uint32_t)cm, 3, 8);

    OLED_ShowString(36, y, (uint8_t *)"cm", 8);
}

static void Car_ShowAll(void)
{
    /* LED 跟随状态：跑着才亮 */
    if (g_run && !g_stop)
        LED_On();
    else
        LED_Off();

#if MOTOR_TEST_MODE
    /* 测试模式：OLED 留给速度显示 */
#else
    /* 状态行、命令行先按整行清掉再写，避免长短字符串叠加留残字
     * （"RUN" 比 "STOP" 短、"CMD:STOP" 比 "CMD:START" 短，不清会留尾巴） */
    OLED_ClearCharArea(0, 0, 21);
    OLED_ClearCharArea(0, 2, 21);

    if (g_stop)
        OLED_ShowString(0, 0, (uint8_t *)"FINISH", 8);
    else if (g_run)
        OLED_ShowString(0, 0, (uint8_t *)"RUN", 8);
    else
        OLED_ShowString(0, 0, (uint8_t *)"STOP", 8);

    OLED_ShowString(0, 2, (uint8_t *)car_cmd_text, 8);

    /* 两路超声波距离（厘米），待机/停车时也在实时刷新 */
    Car_ShowDistanceLine(4, "L", Avoid_GetLeftCm());
    Car_ShowDistanceLine(6, "R", Avoid_GetRightCm());
#endif
}

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

    /* 终止条件：8 路全部压线（黑块/终点线）-> 刹车停车
     *
     * finish_armed 是"离开黑区才重新武装"的锁存：
     * 车正停在终点线上时再发 start，如果直接用 Track_AllBlack() 判断，
     * 会在 10ms 内又被判成终点，车动不了；这里改成要先看到一次
     * "不全黑"，下一次全黑才算新一轮的终点。 */
    if (!Track_AllBlack())
    {
        finish_armed = 1;
    }
    else if (finish_armed)
    {
        finish_armed = 0;
        g_stop = 1;
        g_track_enable = 0;
        Motor_BrakeAll();
        return;
    }

    /* 1. 读编码器速度（脉冲/10ms，带符号） */
    int16_t vL = Encoder_GetSpeed(&encoder_left);
    int16_t vR = Encoder_GetSpeed(&encoder_right);

#if ENC_LEFT_INVERT
    vL = -vL;  /* 左轮编码器方向修正 */
#endif
#if ENC_RIGHT_INVERT
    vR = -vR;  /* 右轮编码器方向修正 */
#endif

#if MOTOR_TEST_MODE
    /* 测试模式：不循迹，位置环输出置 0，只跑左右速度环 */
    float turn = 0.0f;
#else
    /* 2. 读循迹位置偏差（-1000 ~ +1000） */
    int16_t pos = Track_GetPosition();

    /* 3. 位置环：偏差 -> 转向速度差（脉冲/10ms） */
    float turn = PID_Update(&pid_pos, (float)pos);
#endif

    /* 4. 目标速度 = 直行速度 ± 转向（线偏右则左快右慢，向右纠偏） */
    float vL_target = (float)g_target_speed + turn;
    float vR_target = (float)g_target_speed - turn;

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
