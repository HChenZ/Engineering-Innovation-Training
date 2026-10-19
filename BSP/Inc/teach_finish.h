#ifndef TEACH_FINISH_H
#define TEACH_FINISH_H

#include <stdint.h>

/* TIM1每10ms调用。学习/比赛共用窄线历史和终点窗口，比赛额外由主控做地图空间校验。
 * 假定终点是一条横跨8路灰度的黑色横线，不把普通丢线当终点。 */
#define TEACH_FINISH_MIN_CM 100.0f
#define TEACH_FINISH_EXPECTED_CM 0.0f /* 已量圈长可填；0且无旧地图时只能发2手动结束学习 */
#define TEACH_FINISH_WINDOW_CM 30.0f /* 只在预期终点前后30cm内接受全黑 */
#define FINISH_APPROACH_CM 15.0f /* 已知终点前15cm减速，减少采样间隔内位移和刹车滑行 */
#define FINISH_APPROACH_SPEED 20.0f /* 脉冲/10ms；不改变普通循迹基础速度 */
#define TEACH_FINISH_ARM_TICKS 10U /* 先正常循迹100ms，屏蔽起点横线 */
#define TEACH_FINISH_RECENT_TICKS 10U /* 正常窄线到全黑之间最多100ms */
#define TEACH_FINISH_TRACE_SIZE 64U

#define FINISH_REJECT_MIN_DISTANCE 0x01U
#define FINISH_REJECT_UNKNOWN_ROUTE 0x02U
#define FINISH_REJECT_PROGRESS 0x04U
#define FINISH_REJECT_LINE_HISTORY 0x08U
#define FINISH_REJECT_CONTROL 0x10U /* 非正常前进，或主控地图/避障条件不允许 */

typedef struct
{
    uint8_t normal_ticks, normal_age, armed, latched;
    uint8_t hit_bitmap;
    float route_length_cm;
    uint8_t reject_flags;
    uint16_t reject_count;
    float rejected_progress_cm;
    uint8_t trace[TEACH_FINISH_TRACE_SIZE];
    uint8_t trace_next, trace_count;
} TeachFinishDetector;

void TeachFinish_Reset(TeachFinishDetector *detector);
void TeachFinish_SetRouteLength(TeachFinishDetector *detector, float length_cm);
uint8_t Finish_ProgressGate(float progress_cm, float route_length_cm);
uint8_t Finish_ApproachGate(float progress_cm, float route_length_cm);
/* forward=两轮正在前进；只需采到1帧合格FF就锁存，不能被后面的白区清掉。 */
uint8_t TeachFinish_Update(TeachFinishDetector *detector, uint8_t bitmap, float distance_cm, uint8_t forward);
/* 停车后读取，按时间从旧到新复制，用于诊断是否实际采到终点横线。 */
uint8_t TeachFinish_CopyTrace(const TeachFinishDetector *detector, uint8_t *out);

#endif
