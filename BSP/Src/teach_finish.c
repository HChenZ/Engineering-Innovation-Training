#include "teach_finish.h"
#include <string.h>
#include <math.h>

void TeachFinish_Reset(TeachFinishDetector *detector)
{
    memset(detector, 0, sizeof(*detector));
    detector->normal_age = 255U;
    detector->route_length_cm = TEACH_FINISH_EXPECTED_CM;
}

void TeachFinish_SetRouteLength(TeachFinishDetector *detector, float length_cm)
{
    detector->route_length_cm = length_cm;
}

uint8_t Finish_ProgressGate(float progress_cm, float route_length_cm)
{
    return isfinite(progress_cm) && isfinite(route_length_cm) && route_length_cm >= TEACH_FINISH_MIN_CM &&
           fabsf(progress_cm - route_length_cm) <= TEACH_FINISH_WINDOW_CM;
}

uint8_t Finish_ApproachGate(float progress_cm, float route_length_cm)
{
    return Finish_ProgressGate(progress_cm, route_length_cm) &&
           progress_cm >= route_length_cm - FINISH_APPROACH_CM;
}

uint8_t TeachFinish_Update(TeachFinishDetector *detector, uint8_t bitmap, float distance_cm, uint8_t forward)
{
    if (detector->latched) return 1U;
    detector->trace[detector->trace_next] = bitmap;
    detector->trace_next = (uint8_t)((detector->trace_next + 1U) % TEACH_FINISH_TRACE_SIZE);
    if (detector->trace_count < TEACH_FINISH_TRACE_SIZE) detector->trace_count++;

    uint8_t count = 0U, run = 0U, longest = 0U;
    for (unsigned bit = 0U; bit < 8U; bit++)
    {
        if (bitmap & (1U << bit))
        {
            count++;
            run++;
            if (run > longest) longest = run;
        }
        else run = 0U;
    }

    /* 先用正常连续窄线武装，起点全黑本身不能武装。
     * 到达横线时可能先出现6/7路，保留最近一次窄线的资格，不要求紧挨FF。 */
    if (count > 0U && count <= 4U && count == longest && forward)
    {
        detector->normal_age = 0U;
        if (detector->normal_ticks < TEACH_FINISH_ARM_TICKS) detector->normal_ticks++;
        if (detector->normal_ticks >= TEACH_FINISH_ARM_TICKS) detector->armed = 1U;
    }
    else
    {
        detector->normal_ticks = 0U;
        if (detector->normal_age < 255U) detector->normal_age++;
    }

    if (bitmap == 0xFFU)
    {
        uint8_t flags = 0U;
        if (!isfinite(distance_cm) || distance_cm < TEACH_FINISH_MIN_CM) flags |= FINISH_REJECT_MIN_DISTANCE;
        if (!isfinite(detector->route_length_cm) || detector->route_length_cm < TEACH_FINISH_MIN_CM)
            flags |= FINISH_REJECT_UNKNOWN_ROUTE;
        else if (!Finish_ProgressGate(distance_cm, detector->route_length_cm)) flags |= FINISH_REJECT_PROGRESS;
        if (!detector->armed || detector->normal_age > TEACH_FINISH_RECENT_TICKS) flags |= FINISH_REJECT_LINE_HISTORY;
        if (!forward) flags |= FINISH_REJECT_CONTROL;
        if (flags == 0U)
        {
            detector->hit_bitmap = bitmap;
            detector->latched = 1U;
        }
        else
        {
            detector->reject_flags = flags;
            detector->rejected_progress_cm = distance_cm;
            if (detector->reject_count < 65535U) detector->reject_count++;
        }
    }
    return detector->latched;
}

uint8_t TeachFinish_CopyTrace(const TeachFinishDetector *detector, uint8_t *out)
{
    uint8_t start = (uint8_t)((detector->trace_next + TEACH_FINISH_TRACE_SIZE - detector->trace_count) %
                             TEACH_FINISH_TRACE_SIZE);
    for (uint8_t i = 0U; i < detector->trace_count; i++)
        out[i] = detector->trace[(start + i) % TEACH_FINISH_TRACE_SIZE];
    return detector->trace_count;
}
