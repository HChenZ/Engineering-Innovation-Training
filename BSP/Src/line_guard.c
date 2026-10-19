#include "line_guard.h"
#include <math.h>

uint8_t LineGuard_Shape(uint8_t bitmap)
{
    uint8_t count = 0U, run = 0U, longest = 0U;
    for (unsigned bit = 0U; bit < 8U; bit++)
    {
        if (bitmap & (1U << bit))
        {
            count++; run++;
            if (run > longest) longest = run;
        }
        else run = 0U;
    }
    return count > 0U && count <= 4U && count == longest;
}

uint8_t LineGuard_Candidate(uint8_t bitmap, uint8_t map_ready, float distance_cm,
                            float heading_error_deg, float angle_limit_deg)
{
    return LineGuard_Shape(bitmap) && map_ready && isfinite(distance_cm) &&
           distance_cm >= 0.0f && distance_cm <= LINE_GUARD_DISTANCE_CM &&
           isfinite(heading_error_deg) && fabsf(heading_error_deg) <= angle_limit_deg;
}

void LineGuard_Reset(LineGuard *guard)
{
    guard->ticks = 0U;
    guard->forward_cm = 0.0f;
}

uint8_t LineGuard_Update(LineGuard *guard, uint8_t candidate, uint8_t forward,
                         float step_cm, float heading_error_deg)
{
    if (!candidate || !forward || !isfinite(step_cm) || step_cm <= 0.0f)
    {
        LineGuard_Reset(guard);
        return 0U;
    }
    if (guard->ticks < LINE_GUARD_MAX_TICKS) guard->ticks++;
    if (guard->forward_cm < 100.0f) guard->forward_cm += step_cm;
    return guard->ticks >= LINE_GUARD_TICKS && guard->forward_cm >= LINE_GUARD_TRAVEL_CM &&
           isfinite(heading_error_deg) && fabsf(heading_error_deg) <= LINE_GUARD_CONFIRMED_DEG;
}
