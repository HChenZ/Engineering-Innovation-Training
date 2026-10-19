/* gcc -std=c11 -Wall -Wextra -Werror -IBSP/Inc
 *     tests/odom_host/test_teach_finish.c BSP/Src/teach_finish.c -o <temp>/test_teach_finish.exe */
#include <assert.h>
#include <stdio.h>
#include "teach_finish.h"

static void Arm(TeachFinishDetector *d)
{
    for (unsigned i = 0U; i < TEACH_FINISH_ARM_TICKS; i++)
        assert(!TeachFinish_Update(d, 0x18U, 500.0f, 1U));
    assert(d->armed);
}

int main(void)
{
    TeachFinishDetector d;
    TeachFinish_Reset(&d);
    TeachFinish_SetRouteLength(&d, 750.0f);
    for (unsigned i = 0; i < 20U; i++) assert(!TeachFinish_Update(&d, 0xFFU, 0.0f, 1U));
    assert(!d.armed);
    Arm(&d);
    assert(!TeachFinish_Update(&d, 0xFFU, 50.0f, 1U));
    puts("PASS start marker and minimum distance gate");

    TeachFinish_Reset(&d);
    TeachFinish_SetRouteLength(&d, 750.0f);
    Arm(&d);
    assert(!TeachFinish_Update(&d, 0x7EU, 750.0f, 1U));
    assert(!TeachFinish_Update(&d, 0x7FU, 750.5f, 1U));
    assert(TeachFinish_Update(&d, 0xFFU, 751.0f, 1U)); /* only one FF sample */
    assert(TeachFinish_Update(&d, 0x00U, 751.5f, 1U)); /* latch survives white after finish */
    assert(d.hit_bitmap == 0xFFU);
    puts("PASS narrow-line / broadening / single-frame FF / latched finish");

    TeachFinish_Reset(&d);
    TeachFinish_SetRouteLength(&d, 750.0f);
    Arm(&d);
    for (unsigned i = 0U; i < 30U; i++) assert(!TeachFinish_Update(&d, 0x00U, 770.0f, 1U));
    assert(!TeachFinish_Update(&d, 0xFFU, 771.0f, 1U)); /* stale line history */
    puts("PASS ordinary loss and late isolated black are not a finish");

    TeachFinish_Reset(&d);
    TeachFinish_SetRouteLength(&d, 750.0f);
    Arm(&d);
    assert(!TeachFinish_Update(&d, 0xFFU, 750.0f, 0U)); /* pivot/reverse is not normal arrival */
    TeachFinish_Reset(&d);
    TeachFinish_SetRouteLength(&d, 750.0f);
    for (unsigned i = 0; i < 8U; i++) assert(!TeachFinish_Update(&d, 0x18U, 500.0f, 1U));
    assert(!TeachFinish_Update(&d, 0xFFU, 750.0f, 1U));
    puts("PASS forward-motion and arming-duration gates");

    TeachFinish_Reset(&d); /* no known route: a black word cannot imply a completed lap */
    Arm(&d);
    assert(!TeachFinish_Update(&d, 0xFFU, 751.0f, 1U));
    assert(d.reject_flags & FINISH_REJECT_UNKNOWN_ROUTE);
    TeachFinish_SetRouteLength(&d, 750.0f);
    assert(!TeachFinish_Update(&d, 0xFFU, 400.0f, 1U));
    assert(d.reject_flags & FINISH_REJECT_PROGRESS);
    assert(!TeachFinish_Update(&d, 0xFFU, 790.0f, 1U));
    assert(Finish_ProgressGate(750.0f, 750.0f));
    assert(!Finish_ProgressGate(400.0f, 750.0f));
    assert(!Finish_ProgressGate(750.0f, 0.0f));
    assert(!Finish_ApproachGate(700.0f, 750.0f));
    assert(Finish_ApproachGate(740.0f, 750.0f));
    assert(!Finish_ApproachGate(740.0f, 0.0f));
    puts("PASS unknown route / mid-course black words / overshoot rejected");

    TeachFinish_Reset(&d);
    uint8_t trace[TEACH_FINISH_TRACE_SIZE];
    for (unsigned i = 0; i < 80U; i++) (void)TeachFinish_Update(&d, (uint8_t)i, 0.0f, 0U);
    assert(TeachFinish_CopyTrace(&d, trace) == TEACH_FINISH_TRACE_SIZE);
    for (unsigned i = 0; i < TEACH_FINISH_TRACE_SIZE; i++) assert(trace[i] == (uint8_t)(i + 16U));
    puts("PASS chronological 640ms diagnostic trace");
    puts("ALL TESTS PASSED");
    return 0;
}
