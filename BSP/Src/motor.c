#include "motor.h"

/* PWM 周期值 */
#define PWM_PERIOD 100

/* 电机硬件映射结构体 */
typedef struct
{
    GPIO_TypeDef *in1_port;
    uint32_t in1_pin;

    GPIO_TypeDef *in2_port;
    uint32_t in2_pin;

    uint32_t pwm_channel;
} MotorPinConfig;

/* 全局电机引脚映射表 */
static const MotorPinConfig motorConfig[] = {
    {/* MOTOR_LEFT */
     Motor_AIN2_GPIO_Port, Motor_AIN2_Pin,
     Motor_AIN1_GPIO_Port, Motor_AIN1_Pin,
     TIM_CHANNEL_1},
    {/* MOTOR_RIGHT */
     Motor_BIN1_GPIO_Port, Motor_BIN1_Pin,
     Motor_BIN2_GPIO_Port, Motor_BIN2_Pin,
     TIM_CHANNEL_2}};

/* 设置 PWM 占空比 */
static void PWM_SetDuty(uint32_t channel, int16_t duty)
{
    if (duty > PWM_PERIOD)
        duty = PWM_PERIOD;
    __HAL_TIM_SET_COMPARE(&htim2, channel, duty);
}

/* 设置初始状态 */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void Motor_SetSpeed(MotorID motor, MotorMode mode, int16_t duty)
{
    if (motor >= (sizeof(motorConfig) / sizeof(motorConfig[0])))
        return;
    
    if (duty >= PWM_PERIOD)
        duty = PWM_PERIOD;
    else if (duty <= -PWM_PERIOD)
        duty = -PWM_PERIOD;

    const MotorPinConfig *motor_config = &motorConfig[motor];

    switch (mode)
    {
    case MOTOR_STOP: /* 滑行停止 */
        HAL_GPIO_WritePin(motor_config->in1_port, motor_config->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_config->in2_port, motor_config->in2_pin, GPIO_PIN_RESET);
        PWM_SetDuty(motor_config->pwm_channel, 0);
        break;

    case MOTOR_START: /* 启动 */
        if (duty > 0) /* 正转*/
        {
            HAL_GPIO_WritePin(motor_config->in1_port, motor_config->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor_config->in2_port, motor_config->in2_pin, GPIO_PIN_RESET);
            PWM_SetDuty(motor_config->pwm_channel, duty);
        }
        else if (duty < 0) /* 反转 */
        {
            HAL_GPIO_WritePin(motor_config->in1_port, motor_config->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor_config->in2_port, motor_config->in2_pin, GPIO_PIN_SET);
            PWM_SetDuty(motor_config->pwm_channel, -duty);
        }
        else /* 停止 */
        {
            HAL_GPIO_WritePin(motor_config->in1_port, motor_config->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor_config->in2_port, motor_config->in2_pin, GPIO_PIN_RESET);
            PWM_SetDuty(motor_config->pwm_channel, 0);
        }
        break;

    case MOTOR_BRAKE: /* 短接制动 */
        HAL_GPIO_WritePin(motor_config->in1_port, motor_config->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_config->in2_port, motor_config->in2_pin, GPIO_PIN_SET);
        PWM_SetDuty(motor_config->pwm_channel, 0);
        break;

    default: /* 未知 */
        HAL_GPIO_WritePin(motor_config->in1_port, motor_config->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_config->in2_port, motor_config->in2_pin, GPIO_PIN_RESET);
        PWM_SetDuty(motor_config->pwm_channel, 0);
        break;
    }
}

void Motor_StopAll(void)
{
    Motor_SetSpeed(MOTOR_LEFT, MOTOR_STOP, 0);
    Motor_SetSpeed(MOTOR_RIGHT, MOTOR_STOP, 0);
}

void Motor_BrakeAll(void)
{
    Motor_SetSpeed(MOTOR_LEFT, MOTOR_BRAKE, 0);
    Motor_SetSpeed(MOTOR_RIGHT, MOTOR_BRAKE, 0);
}
