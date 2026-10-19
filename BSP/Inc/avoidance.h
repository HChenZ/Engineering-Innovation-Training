#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include "main.h"
#include "ultrasonic.h"

/* 距离单位 mm，速度单位与编码器速度环一致（脉冲/10 ms）。 */
#ifndef AVOIDANCE_SIDE_TRIGGER_MM
#define AVOIDANCE_SIDE_TRIGGER_MM 265U
#endif

#ifndef AVOIDANCE_CENTER_TRIGGER_MM
#define AVOIDANCE_CENTER_TRIGGER_MM 265U
#endif

#ifndef AVOIDANCE_RELEASE_EXTRA_MM
#define AVOIDANCE_RELEASE_EXTRA_MM 60U
#endif

#ifndef AVOIDANCE_SENSOR_MAX_AGE_MS
#define AVOIDANCE_SENSOR_MAX_AGE_MS 300U
#endif

#ifndef AVOIDANCE_BASE_SPEED
#define AVOIDANCE_BASE_SPEED 18.0f
#endif

#ifndef AVOIDANCE_SPEED_DIFF
#define AVOIDANCE_SPEED_DIFF 9.0f
#endif

#ifndef AVOIDANCE_WAIT_SPEED
#define AVOIDANCE_WAIT_SPEED 12.0f
#endif

/* 第一段弧线的编码器累计差速目标；达到目标或超时即结束。 */
#ifndef AVOIDANCE_OUT_TURN_DIFF_COUNTS
#define AVOIDANCE_OUT_TURN_DIFF_COUNTS 900.0f
#endif

/* 理论累计差速上限约为 2 * AVOIDANCE_SPEED_DIFF * AVOIDANCE_OUT_TIMEOUT_TICKS。 */
#ifndef AVOIDANCE_OUT_TIMEOUT_TICKS
#define AVOIDANCE_OUT_TIMEOUT_TICKS 75U
#endif

/* 第三段小弧线：使用更小的速度差 */
#ifndef AVOIDANCE_EXTRA_SPEED_DIFF
#define AVOIDANCE_EXTRA_SPEED_DIFF 5.0f
#endif

/* 灰度第 3~4 路（中间两路）任意一路检测到黑线时结束第三段 */
#ifndef AVOIDANCE_MIDDLE_FOUR_MASK
#define AVOIDANCE_MIDDLE_FOUR_MASK 0x18U
#endif

/* 安全兜底：正常应由中间四路结束，超过该时间则强制交回循迹 */
#ifndef AVOIDANCE_EXTRA_TIMEOUT_TICKS
#define AVOIDANCE_EXTRA_TIMEOUT_TICKS 30U
#endif

typedef enum
{
    AVOIDANCE_STATE_IDLE = 0,
    AVOIDANCE_STATE_TURN_OUT,
    AVOIDANCE_STATE_TURN_IN,
    AVOIDANCE_STATE_EXTRA_TURN
} Avoidance_State;

/* LEFT/RIGHT 表示小车向哪个方向避障，不表示障碍物所在方向。 */
typedef enum
{
    AVOIDANCE_DIRECTION_NONE = 0,
    AVOIDANCE_DIRECTION_LEFT,
    AVOIDANCE_DIRECTION_RIGHT
} Avoidance_Direction;

typedef struct
{
    uint8_t use_line_pid;
    float base_speed;
    float steer;
} Avoidance_Control;

void Avoidance_Init(void);
void Avoidance_Reset(void);

/*
 * 在主循环中调用。
 * 位置为 0 时，中间探头触发会使用最近一次有效且非零的位置。
 */
void Avoidance_ProcessSensors(const Ultrasonic_Reading readings[ULTRASONIC_SENSOR_COUNT], uint8_t bitmap, int16_t line_position);

/*
 * 在 TIM1 的 10 ms 中断中调用。
 * use_line_pid=0 时直接使用返回的基础速度和转向差；
 * use_line_pid=1 时继续执行灰度位置环。
 */
Avoidance_Control Avoidance_UpdateControl(uint8_t bitmap, float left_speed, float right_speed);

Avoidance_State Avoidance_GetState(void);

#endif
