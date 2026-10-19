/* Windows/MinGW host tests: exercises the actual firmware source, not a Python reimplementation.
 * gcc -std=c11 -Wall -Wextra -Werror -Itests/odom_host -IBSP/Inc tests/odom_host/test_odom.c BSP/Src/line_guard.c -lm -o test_odom.exe
 * Run the executable outside the source tree, e.g. in the approved temp directory. */
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"

static int writes_left = -1;
HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *er, uint32_t *error)
{
    assert(er->PageAddress == 0x0800F000U || er->PageAddress == 0x0800F800U);
    assert(er->NbPages == 2U);
    memset((void *)(uintptr_t)er->PageAddress, 0xFF, er->NbPages * 1024U);
    *error = 0xFFFFFFFFU;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint64_t value)
{
    assert(type == FLASH_TYPEPROGRAM_HALFWORD);
    assert(addr >= 0x0800F000U && addr + 1U < 0x08010000U && (addr & 1U) == 0U);
    if (writes_left == 0) return HAL_ERROR;
    if (writes_left > 0) writes_left--;
    uint16_t *p = (uint16_t *)(uintptr_t)addr;
    assert((*p & (uint16_t)value) == (uint16_t)value);
    *p = (uint16_t)value;
    return HAL_OK;
}

/* Include permits inspecting internal state for deterministic geometry and fault injection. */
#include "../../BSP/Src/odom.c"

#define TEST_STEP_COUNTS ((int16_t)(ODOM_PULSES_PER_CM + 0.5f))

static void StraightTeach(void)
{
    Odom_StartTeach();
    for (unsigned i = 0; i < 160U; i++)
    {
        Odom_ObserveLine(0x18U);
        Odom_Update(TEST_STEP_COUNTS, TEST_STEP_COUNTS, 1U);
    }
    assert(Odom_TeachValid());
}

static void TestOdometry(void)
{
    Odom_Init();
    assert(!Odom_MapReady());
    Odom_Update(TEST_STEP_COUNTS, TEST_STEP_COUNTS, 0U);
    assert(fabsf(Odom_GetX() - TEST_STEP_COUNTS / ODOM_PULSES_PER_CM) < 0.001f);
    assert(fabsf(Odom_GetY()) < 0.001f);
    Odom_Reset();
    Odom_Update(1040, 1040, 0U); /* 一整圈应为48mm轮子的周长，约15.080cm。 */
    assert(fabsf(Odom_GetX() - 4.8f * PI_F) < 0.001f);
    Odom_Reset();
    Odom_Update(-1040, 1040, 0U); /* 对转各一圈，17cm轮距对应约101.65度。 */
    assert(fabsf(Odom_GetHeadingDeg() - (2.0f * 4.8f * PI_F / 17.0f) * RAD2DEG) < 0.01f);
    Odom_Reset();
    Odom_Update(-10, 10, 0U);
    assert(Odom_GetHeadingDeg() > 0.0f);
    assert(fabsf(Odom_GetX()) < 0.001f);
    puts("PASS odometry straight / pivot sign");
}

static void TestStorage(void)
{
    StraightTeach();
    uint16_t n = Odom_Save();
    assert(n > 20U && Odom_MapReady());
    assert(active_slot == SLOT0);
    Odom_Init();
    assert(Odom_MapReady() && Odom_GetMapCount() == n);

    /* A failed write must preserve and reload the old slot. */
    StraightTeach();
    writes_left = 5;
    assert(Odom_Save() == 0U);
    writes_left = -1;
    Odom_Init();
    assert(Odom_MapReady() && active_slot == SLOT0);

    /* Complete another map; then corrupt it and fall back to the older one. */
    StraightTeach();
    assert(Odom_Save() == n && active_slot == SLOT1);
    ((MapPoint *)(uintptr_t)(SLOT1 + sizeof(MapHeader)))[3].x_mm ^= 1;
    Odom_Init();
    assert(Odom_MapReady() && active_slot == SLOT0);

    /* Stopped/invalid lessons must never erase the good map. */
    Odom_StartTeach();
    assert(Odom_Save() == 0U);
    Odom_CancelTeach();
    StraightTeach();
    for (unsigned i = 0; i < ODOM_TEACH_LOST_TICKS; i++) Odom_ObserveLine(0U);
    assert(!Odom_TeachValid() && Odom_Save() == 0U);
    Odom_CancelTeach();
    assert(Odom_MapReady() && active_slot == SLOT0);
    puts("PASS flash roundtrip / CRC / interrupted save / aborted teaching");
}

static void SetOffLine(float offset)
{
    Odom_Reset();
    x = 50.0f; y = offset; theta = 0.0f;
    Match(0U);
}

static void TestTeachFaults(void)
{
    StraightTeach();
    uint16_t saved_count = Odom_GetMapCount();
    float saved_dist = Odom_GetDistCm();
    for (unsigned i = 1U; i < ODOM_TEACH_LOST_TICKS; i++) Odom_ObserveLine(0U);
    assert(Odom_TeachValid());
    Odom_ObserveLine(0U);
    assert(!Odom_TeachValid() && Odom_GetTeachError() == ODOM_TEACH_LOST_LINE);
    assert(strcmp(Odom_GetTeachErrorName(), "LOST_LINE") == 0);
    assert(Odom_GetMode() == ODOM_MODE_TEACH && Odom_GetMapCount() == saved_count);
    assert(Odom_Save() == 0U); /* invalid RAM is not mislabeled NO TEACH DATA */
    Odom_Update(TEST_STEP_COUNTS, TEST_STEP_COUNTS, 0U);
    Odom_ObserveLine(0x18U);
    assert(Odom_GetDistCm() == saved_dist && Odom_GetTeachError() == ODOM_TEACH_LOST_LINE);
    Odom_CancelTeach();
    assert(Odom_MapReady());

    Odom_StartTeach();
    assert(Odom_TeachValid() && Odom_GetTeachError() == ODOM_TEACH_OK);
    for (unsigned i = 0; i < 1300U; i++)
    {
        Odom_ObserveLine(0x18U);
        Odom_Update(TEST_STEP_COUNTS, TEST_STEP_COUNTS, 1U);
    }
    assert(Odom_GetTeachError() == ODOM_TEACH_MAP_FULL);
    assert(Odom_GetMapCount() == ODOM_MAP_MAX && Odom_GetMode() == ODOM_MODE_TEACH);
    for (unsigned i = 0; i < ODOM_TEACH_LOST_TICKS; i++) Odom_ObserveLine(0U);
    assert(Odom_GetTeachError() == ODOM_TEACH_MAP_FULL); /* first error is latched */
    assert(Odom_Save() == 0U);
    Odom_CancelTeach();
    assert(Odom_MapReady());

    Odom_StartTeach();
    x = 3200.0f;
    for (unsigned i = 0; i < 6U; i++) Odom_Update(TEST_STEP_COUNTS, TEST_STEP_COUNTS, 1U);
    assert(Odom_GetTeachError() == ODOM_TEACH_COORD_RANGE);
    assert(strcmp(Odom_GetTeachErrorName(), "COORD_RANGE") == 0);
    assert(Odom_Save() == 0U);
    Odom_CancelTeach();
    assert(Odom_MapReady());
    puts("PASS teaching faults retain RAM / specific cause / freeze / save refusal / old-map preservation");
}

static void TestRecovery(void)
{
    float base, steer;
    SetOffLine(30.0f);
    assert(fabsf(map_distance - 30.0f) < 0.1f);
    for (unsigned i = 1; i < ODOM_RECOVERY_LOST_TICKS; i++)
        assert(!Odom_RecoveryControl(0U, 1U, 0U, 0U, &base, &steer));
    assert(Odom_RecoveryControl(0U, 1U, 0U, 0U, &base, &steer));
    assert(base == 0.0f && steer > 0.0f); /* line is right, pivot right */
    theta = atan2f(target_y - y, target_x - x);
    assert(Odom_RecoveryControl(0U, 1U, 0U, 0U, &base, &steer));
    assert(base == ODOM_RECOVERY_SPEED);
    for (unsigned i = 0; i < 10U; i++)
    {
        assert(Odom_RecoveryControl(0U, 1U, 1U, 0U, &base, &steer));
        assert(base == 0.0f && steer == 0.0f);
    }
    assert(!Odom_RecoveryControl(1U, 1U, 0U, 0U, &base, &steer)); /* shared guard has verified */
    assert(recovery == ODOM_RECOVERY_IDLE);

    /* No line/no progress: hard timeout, never endless spinning. */
    SetOffLine(30.0f);
    for (unsigned i = 0; i < ODOM_RECOVERY_LOST_TICKS + ODOM_RECOVERY_MAX_TICKS; i++)
        (void)Odom_RecoveryControl(0U, 1U, 0U, 0U, &base, &steer);
    assert(recovery == ODOM_RECOVERY_FAILED && base == 0.0f && steer == 0.0f);
    assert(!Odom_RecoveryControl(0U, 0U, 0U, 0U, &base, &steer)); /* obstacle preemption */
    assert(recovery == ODOM_RECOVERY_IDLE);

    SetOffLine(5.0f);
    for (unsigned i = 0; i < 100U; i++)
        assert(!Odom_RecoveryControl(0U, 1U, 0U, 0U, &base, &steer));
    assert(Odom_RecoveryControl(0U, 1U, 0U, 1U, &base, &steer)); /* unverified avoidance return */
    SetOffLine(120.0f);
    for (unsigned i = 0; i < ODOM_RECOVERY_LOST_TICKS; i++)
        (void)Odom_RecoveryControl(0U, 1U, 0U, 0U, &base, &steer);
    assert(recovery == ODOM_RECOVERY_FAILED);
    SetOffLine(30.0f);
    assert(Odom_RecoveryControl(0U, 1U, 0U, 1U, &base, &steer));
    x = target_x; y = target_y;
    assert(Odom_RecoveryControl(0U, 1U, 1U, 0U, &base, &steer)); /* do not fail mid-verification */
    assert(recovery != ODOM_RECOVERY_FAILED);
    assert(Odom_RecoveryControl(0U, 1U, 0U, 0U, &base, &steer));
    assert(recovery == ODOM_RECOVERY_FAILED); /* arrived with no credible line */
    puts("PASS recovery delay / nearest segment / steering / reacquisition / timeout / preemption / distance gates");
}

static void TestLocalSearch(void)
{
    /* A later parallel branch is geometrically nearer, but outside the progress window. */
    MapPoint nearby[100];
    for (unsigned i = 0; i < 100U; i++)
        nearby[i] = (MapPoint){ (int16_t)(i < 50U ? i * 50U : (i - 50U) * 50U),
                               (int16_t)(i < 50U ? 0 : 280), 0 };
    map = nearby; count = 100U; mode = ODOM_MODE_RUN;
    Odom_Reset(); x = 20.0f; y = 30.0f;
    Match(0U);
    assert(fabsf(map_distance - 30.0f) < 0.1f);
    assert(anchor == 0U);
    Odom_Init();
    puts("PASS local search excludes nearby wrong branch");
}

static void TestClosedLoop(void)
{
    float base = 0.0f, steer = 0.0f;
    uint8_t entered = 0U, returned = 0U;
    LineGuard guard;
    LineGuard_Reset(&guard);
    SetOffLine(30.0f);
    for (unsigned tick = 0; tick < 650U; tick++)
    {
        /* Ideal differential-drive feedback, no slip; black line is y=0. */
        Odom_Update((int16_t)lroundf(base + steer), (int16_t)lroundf(base - steer), 0U);
        float sensor_y = y + ODOM_SENSOR_FORWARD_CM * sinf(theta);
        uint8_t black = 0U;
        static const int weights[8] = {-5, -4, -2, -1, 1, 2, 4, 5};
        int sum = 0, hits = 0;
        for (unsigned ch = 0U; ch < 8U; ch++)
        {
            /* 通道0在左、7在右；车体左侧的世界y分量为正。 */
            float lateral = (3.5f - (float)ch) * (8.0f / 7.0f);
            if (fabsf(sensor_y + lateral * cosf(theta)) < 0.8f)
            { black |= (uint8_t)(1U << ch); sum += weights[ch]; hits++; }
        }
        uint8_t candidate = LineGuard_Candidate(black, 1U, map_distance, heading_error, LINE_GUARD_APPROACH_DEG);
        uint8_t verified = LineGuard_Update(&guard, candidate, (uint8_t)(base > fabsf(steer)), step_cm, heading_error);
        uint8_t active = Odom_RecoveryControl(verified, 1U, candidate && !verified, 0U, &base, &steer);
        if (candidate && !verified)
        {
            base = LINE_GUARD_SPEED;
            steer = 0.07f * (hits ? (float)sum * 1000.0f / (5.0f * hits) : 0.0f)
                    + LINE_GUARD_HEADING_KP * heading_error;
            if (steer > LINE_GUARD_STEER_MAX) steer = LINE_GUARD_STEER_MAX;
            if (steer < -LINE_GUARD_STEER_MAX) steer = -LINE_GUARD_STEER_MAX;
        }
        if (recovery == ODOM_RECOVERY_FAILED)
            printf("recovery failure tick=%u pose=(%.2f,%.2f,%.1f) D=%.2f GS=%02X guard=%u/%.2f target=(%.2f,%.2f)\n",
                   tick, (double)x, (double)y, (double)(theta * RAD2DEG), (double)map_distance,
                   black, guard.ticks, (double)guard.forward_cm, (double)target_x, (double)target_y);
        assert(recovery != ODOM_RECOVERY_FAILED);
        if (active) entered = 1U;
        if (entered && !active) { returned = 1U; break; }
    }
    assert(entered && returned);
    puts("PASS ideal closed-loop approach / rolling verification / real-line handover");
}

int main(void)
{
    void *flash = VirtualAlloc((void *)(uintptr_t)0x08000000U, 65536U,
                               MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
    assert(flash == (void *)(uintptr_t)0x08000000U);
    memset(flash, 0xFF, 65536U);
    TestOdometry();
    TestStorage();
    TestTeachFaults();
    TestRecovery();
    TestLocalSearch();
    TestClosedLoop();
    VirtualFree(flash, 0U, MEM_RELEASE);
    puts("ALL TESTS PASSED");
    return 0;
}
