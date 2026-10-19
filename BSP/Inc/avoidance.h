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

/* 第三段小弧线：使用更小的速度差和独立的编码器累计差速目标。 */
#ifndef AVOIDANCE_EXTRA_SPEED_DIFF
#define AVOIDANCE_EXTRA_SPEED_DIFF 5.0f
#endif

#ifndef AVOIDANCE_EXTRA_TURN_DIFF_COUNTS
#define AVOIDANCE_EXTRA_TURN_DIFF_COUNTS 190.0f
#endif

#ifndef AVOIDANCE_EXTRA_TIMEOUT_TICKS
#define AVOIDANCE_EXTRA_TIMEOUT_TICKS 30U
#endif

/* 回线候选由主控制器的地图/形态检查给出，连续命中后才开始第三段。 */
#define AVOIDANCE_LINE_HINT_TICKS 3U
#define AVOIDANCE_IN_TIMEOUT_TICKS 250U /* 找不到可信线最多2.5s，交给回线验证/地图兜底 */

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
 * line_candidate由地图距离/方向和灰度形态共同判断，不是bitmap非零。
 * use_line_pid=1只表示动作结束或空闲，主控制器仍须完成滚动回线验证，不能直接算成功。
 */
Avoidance_Control Avoidance_UpdateControl(uint8_t bitmap, float left_speed, float right_speed,
                                          uint8_t line_candidate);

Avoidance_State Avoidance_GetState(void);

/* 当前这次避障往哪边避（IDLE 时为 NONE） */
Avoidance_Direction Avoidance_GetDirection(void);

#endif
