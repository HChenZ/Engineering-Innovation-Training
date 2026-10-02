#ifndef MOTOR_H
#define MOTOR_H

// PB11 - AIN1
// PB10 - AIN2
// PB1 - BIN1
// PB0 - BIN2

// PA0 - PWMA
// PA1 - PWMB

#include "main.h"
#include "gpio.h"
#include "tim.h"

/* 电机方向 */
typedef enum
{
    MOTOR_STOP = 0, // 停止（滑行）
    MOTOR_START,    // 启动
    MOTOR_BRAKE     // 刹车（短接制动）
} MotorMode;

/* 电机编号 */
typedef enum
{
    MOTOR_LEFT = 0, // 电机 A（左轮）
    MOTOR_RIGHT     // 电机 B（右轮）
} MotorID;

/* 初始化 */
void Motor_Init(void);

/* 设置电机速度和方向
 * @param motor: 电机编号
 * @param dir: 方向
 * @param duty: 占空比 -100~100（对应正反转的 0%~100%）
 */
void Motor_SetSpeed(MotorID motor, MotorMode mode, int16_t duty);

/* 停止 / 刹车所有电机 */
void Motor_StopAll(void);
void Motor_BrakeAll(void);

#endif
