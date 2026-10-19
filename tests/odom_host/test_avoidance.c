/* gcc -std=c11 -Wall -Wextra -Werror -Itests/odom_host -IBSP/Inc
 *     tests/odom_host/test_avoidance.c BSP/Src/line_guard.c -lm -o <temp>/test_avoidance.exe */
#include <assert.h>
#include <stdio.h>
#include "line_guard.h"
#include "../../BSP/Src/tracking.c"
#include "../../BSP/Src/avoidance.c"
uint8_t Grayscale_Sensor_Read_All(void) { return 0U; }

static void TurnIn(void)
{
    Avoidance_Reset();
    avoidance_state = AVOIDANCE_STATE_TURN_IN;
    active_direction = AVOIDANCE_DIRECTION_RIGHT;
}

int main(void)
{
    Avoidance_Control c;
    TurnIn();
    for (unsigned i = 0; i < 30U; i++)
    {
        /* A word contains two adjacent black pixels, but is 20cm off the route. */
        uint8_t candidate = LineGuard_Candidate(0x18U, 1U, 20.0f, 0.0f, LINE_GUARD_APPROACH_DEG);
        c = Avoidance_UpdateControl(0x18U, 9.0f, 27.0f, candidate);
        assert(avoidance_state == AVOIDANCE_STATE_TURN_IN && !c.use_line_pid);
    }
    for (unsigned i = 0; i < AVOIDANCE_LINE_HINT_TICKS - 1U; i++)
    {
        c = Avoidance_UpdateControl(0x18U, 9.0f, 27.0f, 1U);
        assert(avoidance_state == AVOIDANCE_STATE_TURN_IN);
    }
    c = Avoidance_UpdateControl(0U, 9.0f, 27.0f, 0U);
    assert(line_hint_ticks == 0U);
    for (unsigned i = 0; i < AVOIDANCE_LINE_HINT_TICKS; i++)
        c = Avoidance_UpdateControl(0x18U, 9.0f, 27.0f, 1U);
    assert(avoidance_state == AVOIDANCE_STATE_EXTRA_TURN && !c.use_line_pid);
    puts("PASS off-map black text rejected / hint dropout / three-stage motion preserved");

    for (unsigned i = 0; i < AVOIDANCE_EXTRA_TIMEOUT_TICKS; i++)
        c = Avoidance_UpdateControl(0x18U, 23.0f, 13.0f, 1U);
    assert(avoidance_state == AVOIDANCE_STATE_IDLE && c.use_line_pid);
    puts("PASS third-stage completion returns to supervisor (not proof of line validity)");

    TurnIn();
    for (unsigned i = 1U; i < AVOIDANCE_IN_TIMEOUT_TICKS; i++)
    {
        c = Avoidance_UpdateControl(0x18U, 9.0f, 27.0f, 0U);
        assert(avoidance_state == AVOIDANCE_STATE_TURN_IN && !c.use_line_pid);
    }
    c = Avoidance_UpdateControl(0x18U, 9.0f, 27.0f, 0U);
    assert(avoidance_state == AVOIDANCE_STATE_IDLE && c.use_line_pid);
    puts("PASS turn-in cannot spin forever when every black patch is rejected");
    puts("ALL TESTS PASSED");
    return 0;
}
