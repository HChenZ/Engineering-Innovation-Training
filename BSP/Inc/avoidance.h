#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include "main.h"
#include "ultrasonic.h"

/* 距离单位 mm，速度单位与编码器速度环一致（脉冲/10 ms）。 */
#ifndef AVOIDANCE_SIDE_TRIGGER_MM
#define AVOIDANCE_SIDE_TRIGGER_MM 280U
#endif

#ifndef AVOIDANCE_RELEASE_EXTRA_MM
#define AVOIDANCE_RELEASE_EXTRA_MM 60U
#endif

#ifndef AVOIDANCE_SENSOR_MAX_AGE_MS
#define AVOIDANCE_SENSOR_MAX_AGE_MS 300U
#endif

/* 绕行结束交回循迹后的触发冷却：这段时间内不响应超声波触发，
 * 防止刚绕完还贴着障碍物时"刚回线又立刻绕出去"。 */
#ifndef AVOIDANCE_REARM_MS
#define AVOIDANCE_REARM_MS 1000U
#endif

#ifndef AVOIDANCE_BASE_SPEED
#define AVOIDANCE_BASE_SPEED 18.0f
#endif

#ifndef AVOIDANCE_SPEED_DIFF
#define AVOIDANCE_SPEED_DIFF 8.5f
#endif

/* 第一段弧线的编码器累计差速目标；达到目标或超时即结束。 */
#ifndef AVOIDANCE_OUT_TURN_DIFF_COUNTS
#define AVOIDANCE_OUT_TURN_DIFF_COUNTS 900.0f
#endif

/* 理论累计差速上限约为 2 * AVOIDANCE_SPEED_DIFF * AVOIDANCE_OUT_TIMEOUT_TICKS。 */
#ifndef AVOIDANCE_OUT_TIMEOUT_TICKS
#define AVOIDANCE_OUT_TIMEOUT_TICKS 75U
#endif

/* ==================== 转入段的"保底转圈时间" ====================
 *
 * 第二段（TURN_IN）是边转边扫线的：一旦读到"连续 ≥2 个黑点"就认为抓到了线，
 * 切到第三段小弧线，然后很快交回循迹。
 *
 * 问题是弯道旁边常印着黑色字/深色块，也会被读成"连续黑" —— 于是障碍还没绕过去
 * 就提前结束避障、交回循迹，紧接着就彻底丢线（实测就是这么跑的）。
 *
 * 这个参数给转入段加一个【保底转圈时间】：这段时间内只转、不判线，
 * 时间到了之后才开始认黑线。单位是 TIM1 中断拍数（1 拍 = 10ms）。
 *
 * 现在是 40（0.4s）：进入转入段后的前 0.4 秒只转、不判线。
 * 按转入段的速度/差速换算，40 拍 ≈ 车身走 10cm、车头转过约 38°——
 * 也就是"先把车从弯道旁那股黑字上带开，再开始认真找线"。
 * 调法：黑字还会触发 → 加（50、60 一格一格试）；
 *       该回线时反而回不来、绕太远 → 减（30、20）。 */
#ifndef AVOIDANCE_TURN_IN_MIN_TICKS
#define AVOIDANCE_TURN_IN_MIN_TICKS 45U
#endif

/* 第三段速度差根据第二段捕获黑线时最长连续黑点数选择。 */
#ifndef AVOIDANCE_EXTRA_SPEED_DIFF_2
#define AVOIDANCE_EXTRA_SPEED_DIFF_2 2.5f
#endif

#ifndef AVOIDANCE_EXTRA_SPEED_DIFF_3
#define AVOIDANCE_EXTRA_SPEED_DIFF_3 5.0f
#endif

#ifndef AVOIDANCE_EXTRA_SPEED_DIFF_4_PLUS
#define AVOIDANCE_EXTRA_SPEED_DIFF_4_PLUS 8.0f
#endif

/*
 * 第三段最小行驶距离，单位为左右轮平均编码器增量。
 * 达到该值之前，即使检测到黑线也不交回循迹。
 */
#ifndef AVOIDANCE_EXTRA_MIN_TRAVEL_COUNTS
#define AVOIDANCE_EXTRA_MIN_TRAVEL_COUNTS 90.0f
#endif

/* 灰度第 3~4 路（中间两路）任意一路检测到黑线时结束第三段 */
#ifndef AVOIDANCE_MIDDLE_FOUR_MASK
#define AVOIDANCE_MIDDLE_FOUR_MASK 0x18U
#endif

/* 达到最小距离后开始计时；中间两路未捕获黑线时超时交回循迹。 */
#ifndef AVOIDANCE_EXTRA_TIMEOUT_TICKS
#define AVOIDANCE_EXTRA_TIMEOUT_TICKS 35U
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

/* 在主循环中调用；只使用左右两路超声波数据做触发判断。 */
void Avoidance_ProcessSensors(const Ultrasonic_Reading readings[ULTRASONIC_SENSOR_COUNT]);

/*
 * 在 TIM1 的 10 ms 中断中调用。
 * use_line_pid=0 时直接使用返回的基础速度和转向差；
 * use_line_pid=1 时继续执行灰度位置环。
 */
Avoidance_Control Avoidance_UpdateControl(uint8_t bitmap, float left_speed, float right_speed);

Avoidance_State Avoidance_GetState(void);

#endif
