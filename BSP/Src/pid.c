#include "pid.h"

/* 将 value 限制到 [min_value, max_value] 范围内。 */
static float PID_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/* 保证左右限幅值顺序正确。 */
static void PID_NormalizeLimits(float *min_value, float *max_value)
{
    float temp;

    if (*min_value > *max_value)
    {
        temp = *min_value;
        *min_value = *max_value;
        *max_value = temp;
    }
}

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float output_min, float output_max)
{
    if (pid == 0)
    {
        return;
    }

    PID_NormalizeLimits(&output_min, &output_max);

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output_min = output_min;
    pid->output_max = output_max;

    /*
     * 默认让积分项不超过输出限幅。
     * 如果需要更激进的积分，可通过 PID_SetIntegralLimit() 调整。
     */
    if ((output_min < 0.0f) && (output_max > 0.0f))
    {
        float limit = (-output_min > output_max) ? -output_min : output_max;
        pid->integral_min = -limit;
        pid->integral_max = limit;
    }
    else
    {
        pid->integral_min = output_min;
        pid->integral_max = output_max;
    }

    /* 轻度低通，抑制数字灰度传感器跳变带来的微分噪声。 */
    pid->derivative_alpha = 0.70f;
    pid->dead_zone = 0.0f;

    PID_Reset(pid);
}

void PID_Reset(PID_Controller *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_derivative = 0.0f;
    pid->output = 0.0f;
    pid->initialized = 0;
}

void PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd)
{
    if (pid == 0)
        return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetOutputLimit(PID_Controller *pid, float output_min, float output_max)
{
    if (pid == 0)
    {
        return;
    }

    PID_NormalizeLimits(&output_min, &output_max);
    pid->output_min = output_min;
    pid->output_max = output_max;

    pid->output = PID_Clamp(pid->output, output_min, output_max);
    pid->integral = PID_Clamp(pid->integral, pid->integral_min, pid->integral_max);
}

void PID_SetIntegralLimit(PID_Controller *pid, float integral_min, float integral_max)
{
    if (pid == 0)
        return;

    PID_NormalizeLimits(&integral_min, &integral_max);
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->integral = PID_Clamp(pid->integral, pid->integral_min, pid->integral_max);
}

void PID_SetDerivativeFilter(PID_Controller *pid, float alpha)
{
    if (pid == 0)
        return;

    pid->derivative_alpha = PID_Clamp(alpha, 0.0f, 1.0f);
}

void PID_SetDeadZone(PID_Controller *pid, float dead_zone)
{
    if (pid == 0)
        return;

    if (dead_zone < 0.0f)
    {
        dead_zone = -dead_zone;
    }
    pid->dead_zone = dead_zone;
}

float PID_Update(PID_Controller *pid, float setpoint, float feedback, float dt)
{
    return PID_UpdateError(pid, setpoint - feedback, dt);
}

float PID_UpdateError(PID_Controller *pid, float error, float dt)
{
    float proportional;
    float derivative = 0.0f;
    float integral_candidate;
    float output_without_limit;

    if ((pid == 0) || (dt <= 0.0f))
    {
        return 0.0f;
    }

    /* 消除中心附近的来回抖动，死区单位与输入误差相同。 */
    if ((error <= pid->dead_zone) && (error >= -pid->dead_zone))
    {
        error = 0.0f;
    }

    proportional = pid->kp * error;

    /*
     * 第一次计算不使用微分，避免 last_error 的初始值造成输出冲击。
     * 之后先由相邻误差求微分，再进行一阶低通滤波。
     */
    if (pid->initialized != 0)
    {
        float derivative_raw = (error - pid->last_error) / dt;
        pid->last_derivative = pid->derivative_alpha * pid->last_derivative + (1.0f - pid->derivative_alpha) * derivative_raw;
    }

    derivative = pid->kd * pid->last_derivative;

    /* 先计算候选积分，再根据是否饱和决定是否接受它。 */
    integral_candidate = pid->integral + pid->ki * error * dt;
    integral_candidate = PID_Clamp(integral_candidate, pid->integral_min, pid->integral_max);

    output_without_limit = proportional + integral_candidate + derivative;

    /*
     * 条件积分抗饱和：
     * 输出已到正上限且误差仍为正时停止继续积分；
     * 输出已到负下限且误差仍为负时同理。
     * 反向误差仍可让积分退出饱和。
     */
    if (((output_without_limit > pid->output_max) && (error > 0.0f)) || ((output_without_limit < pid->output_min) && (error < 0.0f)))
    {
        integral_candidate = pid->integral;
    }

    pid->integral = integral_candidate;
    pid->output = PID_Clamp(proportional + pid->integral + derivative, pid->output_min, pid->output_max);
    pid->last_error = error;
    pid->initialized = 1;

    return pid->output;
}
