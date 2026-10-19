/* gcc -std=c11 -Wall -Wextra -Werror -IBSP/Inc tests/odom_host/test_line_guard.c
 *     BSP/Src/line_guard.c -lm -o <temp>/test_line_guard.exe */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "line_guard.h"

int main(void)
{
    assert(LineGuard_Shape(0x18U));
    assert(LineGuard_Shape(0x80U));
    assert(!LineGuard_Shape(0U));
    assert(!LineGuard_Shape(0x55U));
    assert(!LineGuard_Shape(0x81U));
    assert(!LineGuard_Shape(0xFFU));
    assert(!LineGuard_Candidate(0x18U, 1U, 20.0f, 0.0f, 75.0f));
    assert(!LineGuard_Candidate(0x18U, 1U, 4.0f, 100.0f, 75.0f));
    assert(!LineGuard_Candidate(0x18U, 0U, 4.0f, 0.0f, 75.0f));
    assert(!LineGuard_Candidate(0x18U, 1U, NAN, 0.0f, 75.0f));
    assert(LineGuard_Candidate(0x18U, 1U, 4.0f, 0.0f, 75.0f));
    puts("PASS font shapes / map distance / heading / unavailable-map gates");

    LineGuard g;
    LineGuard_Reset(&g);
    for (unsigned i = 0; i < 200U; i++)
        assert(!LineGuard_Update(&g, 1U, 0U, 0.0f, 0.0f));
    for (unsigned i = 0; i < 200U; i++)
        assert(!LineGuard_Update(&g, 1U, 1U, 0.0f, 0.0f));
    assert(g.ticks == 0U);
    for (unsigned i = 0; i < 5U; i++)
        assert(!LineGuard_Update(&g, 1U, 1U, 0.8f, 0.0f)); /* distance alone isn't enough */
    assert(!LineGuard_Update(&g, 0U, 1U, 0.2f, 0.0f));
    for (unsigned i = 0; i < LINE_GUARD_TICKS; i++)
        assert(!LineGuard_Update(&g, 1U, 1U, 0.05f, 0.0f)); /* time alone isn't enough */
    assert(!LineGuard_Update(&g, 1U, 0U, -0.1f, 0.0f));
    assert(g.ticks == 0U && g.forward_cm == 0.0f);
    puts("PASS stationary text / brief crossing / time-only / reverse rejected");

    for (unsigned i = 0; i < LINE_GUARD_TICKS + 5U; i++)
        assert(!LineGuard_Update(&g, 1U, 1U, 0.25f, 60.0f));
    assert(LineGuard_Update(&g, 1U, 1U, 0.25f, 30.0f));
    assert(!LineGuard_Update(&g, 0U, 1U, 0.25f, 30.0f));
    assert(g.ticks == 0U);
    for (unsigned i = 1; i < LINE_GUARD_TICKS; i++)
        assert(!LineGuard_Update(&g, 1U, 1U, 0.25f, 0.0f));
    assert(LineGuard_Update(&g, 1U, 1U, 0.25f, 0.0f));
    puts("PASS continuous rolling and aligned-direction confirmation / dropout reset");
    puts("ALL TESTS PASSED");
    return 0;
}
