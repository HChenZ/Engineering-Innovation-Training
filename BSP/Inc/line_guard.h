#ifndef LINE_GUARD_H
#define LINE_GUARD_H

#include <stdint.h>

/* 黑字抑制：候选不是成功。以下均为待实车标定初值。 */
#define LINE_GUARD_DISTANCE_CM 8.0f /* 传感器到地图线段，不是轮轴到地图 */
#define LINE_GUARD_APPROACH_DEG 75.0f /* 入线候选允许斜入，尚不能交回正常循迹 */
#define LINE_GUARD_CONFIRMED_DEG 35.0f
#define LINE_GUARD_TICKS 15U /* 连续150ms，且必须实际前进足够距离 */
#define LINE_GUARD_TRAVEL_CM 3.0f
#define LINE_GUARD_SPEED 10.0f /* 脉冲/10ms */
#define LINE_GUARD_STEER_MAX 6.0f /* 小于速度，验证时两轮保持前进 */
#define LINE_GUARD_HEADING_KP 0.15f
#define LINE_GUARD_MAX_TICKS 600U /* 退出避障后最多6s验证/救援，失败停车 */

typedef struct
{
    uint16_t ticks;
    float forward_cm;
} LineGuard;

uint8_t LineGuard_Shape(uint8_t bitmap); /* 1~4个相邻黑点，散点/大黑块不是窄线 */
uint8_t LineGuard_Candidate(uint8_t bitmap, uint8_t map_ready, float distance_cm,
                            float heading_error_deg, float angle_limit_deg);
void LineGuard_Reset(LineGuard *guard);
/* 一帧不可信、静止、倒车都重新计数；不能停在一个字上计时完成验证。 */
uint8_t LineGuard_Update(LineGuard *guard, uint8_t candidate, uint8_t forward,
                         float step_cm, float heading_error_deg);

#endif
