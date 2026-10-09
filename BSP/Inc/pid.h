#ifndef PID_H
#define PID_H

/* 位置式 PID 控制器（带抗积分饱和 + 输出限幅）
 *
 * 纯算法模块，不依赖硬件。用法：
 *   PID_t pid;
 *   PID_Init(&pid, Kp, Ki, Kd, integral_max, out_min, out_max);
 *   每个控制周期调用： out = PID_Update(&pid, error);
 *   error = 目标值 - 测量值
 *
 * 调参顺序（经典做法）：
 *   1. 只给 Kp，从小到大加到临界振荡再回退；
 *   2. 加 Kd 抑制超调和摆动；
 *   3. 需要消除稳态误差才加 Ki（加之前先把 Kp 调小）。 */

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral;      /* 积分累加（内部状态） */
    float integral_max;  /* 积分限幅，抗饱和 */
    float prev_error;    /* 上次误差（内部状态，用于微分） */
    float out_min;       /* 输出下限 */
    float out_max;       /* 输出上限 */
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float integral_max, float out_min, float out_max);

/* 单次更新。error = 目标 - 测量，返回限幅后的输出。 */
float PID_Update(PID_t *pid, float error);

/* 清零积分和上次误差（切换目标/恢复运行时调用，防止积分残留突变） */
void PID_Reset(PID_t *pid);

#endif
