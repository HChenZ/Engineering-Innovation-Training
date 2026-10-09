#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float integral_max, float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->integral_max = integral_max;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float PID_Update(PID_t *pid, float error)
{
    float p, i, d, out;

    /* 比例 */
    p = pid->Kp * error;

    /* 积分（带限幅抗饱和） */
    pid->integral += error;
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;
    i = pid->Ki * pid->integral;

    /* 微分 */
    d = pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;

    /* 合成 + 输出限幅 */
    out = p + i + d;
    if (out > pid->out_max)
        out = pid->out_max;
    else if (out < pid->out_min)
        out = pid->out_min;

    return out;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
