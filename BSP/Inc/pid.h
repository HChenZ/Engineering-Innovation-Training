#ifndef PID_H
#define PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 位置式 PID 控制器
 *
 * 循迹时通常令目标值 setpoint = 0，把灰度传感器计算出的
 * 巡线偏差（例如 -3.5 ~ +3.5）作为 feedback 传入 PID_Update()。
 *
 * 控制器没有使用动态内存，也没有依赖 HAL，可以直接在定时器
 * 中断或主循环中周期调用。
 */
typedef struct
{
    float kp;                    /* 比例系数 */
    float ki;                    /* 积分系数 */
    float kd;                    /* 微分系数 */

    float integral;              /* 积分项，单位与 PID 输出相同 */
    float last_error;            /* 上一次误差 */
    float last_derivative;       /* 上一次滤波后的微分值 */
    float output;                /* 最近一次输出 */

    float output_min;            /* 输出下限 */
    float output_max;            /* 输出上限 */
    float integral_min;          /* 积分项下限（输出单位） */
    float integral_max;          /* 积分项上限（输出单位） */

    float derivative_alpha;      /* 微分低通系数：0 不滤波，越大越平滑 */
    float dead_zone;             /* 误差死区，绝对值小于它时按 0 处理 */

    uint8_t initialized;         /* 是否已经执行过第一次计算 */
} PID_Controller;

/*
 * 初始化 PID。
 *
 * output_min / output_max 是控制器输出范围。例如用于转向修正时，
 * 可设置为 -30 ~ +30（对应电机 PWM 的 ±30 个占空比单位）。
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float output_min, float output_max);

/* 清除积分、微分历史和输出，但保留 PID 参数。起步或重新循迹时调用。 */
void PID_Reset(PID_Controller *pid);

/* 运行中修改 PID 参数。 */
void PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd);

/* 设置输出限幅。 */
void PID_SetOutputLimit(PID_Controller *pid, float output_min, float output_max);

/* 设置积分项限幅（单位与控制器输出相同），用于抗积分饱和。 */
void PID_SetIntegralLimit(PID_Controller *pid, float integral_min, float integral_max);

/* 设置微分低通系数。取值范围 0 ~ 1：0 表示不滤波，0.8 表示较强滤波。 */
void PID_SetDerivativeFilter(PID_Controller *pid, float alpha);

/* 设置误差死区，单位与输入的 setpoint / feedback 相同。 */
void PID_SetDeadZone(PID_Controller *pid, float dead_zone);

/*
 * 标准 PID 计算。
 *
 * error = setpoint - feedback
 * dt    = 调用周期，单位为秒（例如 10 ms 填 0.01f）
 *
 * 返回本次 PID 输出，并保存在 pid->output 中。
 *
 * 注意：必须使用固定且准确的调用周期，否则积分和微分会失真。
 */
float PID_Update(PID_Controller *pid, float setpoint, float feedback, float dt);

/*
 * 直接使用误差进行计算，适合循迹。
 *
 * 例如传感器得到的偏差为 line_error：
 *     steer = PID_UpdateError(&line_pid, line_error, 0.01f);
 *
 * 此函数与 PID_Update(pid, 0.0f, -line_error, dt) 等价。
 */
float PID_UpdateError(PID_Controller *pid, float error, float dt);

#ifdef __cplusplus
}
#endif

#endif
