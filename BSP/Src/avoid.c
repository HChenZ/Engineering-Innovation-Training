#include "avoid.h"
#include "track.h"

/* ============================================================================
 * 避障状态机：前进式 S 形绕行，见 avoid.h 顶部的流程说明
 *
 * FOLLOW ──近障碍确认──> DODGE（向外让，固定时长）
 *   ▲                      │
 *   │                      ▼
 * COOLDOWN <──见黑线── RETURN（回线，直到见黑线）
 * ========================================================================== */

/* 状态 */
#define AVOID_ST_FOLLOW   0U /* 正常循迹（接近确认也在这一步做） */
#define AVOID_ST_DODGE_R  1U /* 障碍偏左：右前弧（第一阶段） */
#define AVOID_ST_DODGE_L  2U /* 障碍偏右：左前弧（第一阶段） */
#define AVOID_ST_RETURN_R 3U /* 右让之后：左前弧回线（第二阶段） */
#define AVOID_ST_RETURN_L 4U /* 左让之后：右前弧回线（第二阶段） */
#define AVOID_ST_COOLDOWN 5U /* 已交还循迹，暂时不再触发 */

static uint8_t  s_state = AVOID_ST_FOLLOW;
static uint32_t s_state_tick = 0U;   /* 进入当前状态的时刻 */
static uint32_t s_sample_tick = 0U;  /* 上次触发采样的时刻 */
static uint8_t  s_sample_next = 0U;  /* 下一路触发左(0)/右(1) */
static uint32_t s_near_tick = 0U;    /* 接近状态起始时刻（触发确认用） */
static uint8_t  s_near_active = 0U;
static uint8_t  s_line_cnt = 0U;     /* 回线阶段"见线"连续确认计数 */

static int16_t  s_drive_l = 0;       /* 绕行时左右轮目标速度（脉冲/10ms） */
static int16_t  s_drive_r = 0;
static AvoidAction s_action = AVOID_FOLLOW;

/* 取距离：-1（无回波）当作很远 */
static int16_t Avoid_DistCm(HCSR04_ID id)
{
    const int16_t cm = HCSR04_LastCm(id);

    return (cm < 0) ? 999 : cm;
}

/* 是否见到黑线。
 * 直接读 8 路位图，不用 Track_IsLost()——那个标志只在 Track_GetPosition()
 * 里刷新，而避障期间位置环不在跑，读到的会是启动避障之前的旧值。
 * 主循环读传感器与 TIM1 中断里的 Track_AllBlack() 共用一个模拟开关，
 * 单次读有可能撞上中断而失真，所以调用方要用连续两次确认。 */
static uint8_t Avoid_LineSeen(void)
{
    return (Track_ReadAll() != 0U) ? 1U : 0U;
}

void Avoid_Init(void)
{
    HCSR04_Init();

    s_state = AVOID_ST_FOLLOW;
    s_state_tick = HAL_GetTick();
    s_sample_tick = HAL_GetTick();
    s_sample_next = 0U;
    s_near_active = 0U;
    s_line_cnt = 0U;
    s_drive_l = 0;
    s_drive_r = 0;
    s_action = AVOID_FOLLOW;
}

/* 轮流触发两路采样（非阻塞）。
 * 只有两路都空闲时才触发下一路：等这一路测完再发下一路，
 * 避免两个探头同时工作、互相收到对方的波（串扰） */
static void Avoid_Sample(void)
{
    if ((HAL_GetTick() - s_sample_tick) < AVOID_SAMPLE_MS)
        return;

    if (HCSR04_IsBusy(HCSR04_LEFT) || HCSR04_IsBusy(HCSR04_RIGHT))
        return;

    s_sample_tick = HAL_GetTick();

    if (s_sample_next == 0U)
    {
        HCSR04_Trigger(HCSR04_LEFT);
        s_sample_next = 1U;
    }
    else
    {
        HCSR04_Trigger(HCSR04_RIGHT);
        s_sample_next = 0U;
    }
}

/* 启动绕行：按距离判断障碍在左还是右，往更空的一侧让 */
static void Avoid_StartDodge(void)
{
    s_state_tick = HAL_GetTick();
    s_near_active = 0U;

    if (Avoid_DistCm(HCSR04_LEFT) <= Avoid_DistCm(HCSR04_RIGHT))
    {
        /* 左侧更近 → 障碍偏左 → 先向右前方绕 */
        s_state = AVOID_ST_DODGE_R;
        s_drive_l = AVOID_DRIVE_SPEED + AVOID_ARC_DIFF;
        s_drive_r = AVOID_DRIVE_SPEED - AVOID_ARC_DIFF;
    }
    else
    {
        /* 右侧更近 → 障碍偏右 → 先向左前方绕 */
        s_state = AVOID_ST_DODGE_L;
        s_drive_l = AVOID_DRIVE_SPEED - AVOID_ARC_DIFF;
        s_drive_r = AVOID_DRIVE_SPEED + AVOID_ARC_DIFF;
    }

    s_action = AVOID_AVOIDING;
}

AvoidAction Avoid_Update(void)
{
    const uint32_t now = HAL_GetTick();
    int16_t fl, fr, fmin;

    Avoid_Sample();

    fl = Avoid_DistCm(HCSR04_LEFT);
    fr = Avoid_DistCm(HCSR04_RIGHT);
    fmin = (fl < fr) ? fl : fr;

    switch (s_state)
    {
    case AVOID_ST_DODGE_R: /* 第一阶段：右前弧，固定时长，不理会黑线 */
        s_drive_l = AVOID_DRIVE_SPEED + AVOID_ARC_DIFF;
        s_drive_r = AVOID_DRIVE_SPEED - AVOID_ARC_DIFF;
        s_action = AVOID_AVOIDING;

        if ((now - s_state_tick) >= AVOID_DODGE_MS)
        {
            /* 进入第二阶段：向左前方回线 */
            s_state = AVOID_ST_RETURN_R;
            s_state_tick = now;
            s_line_cnt = 0U;
        }
        break;

    case AVOID_ST_DODGE_L: /* 第一阶段镜像 */
        s_drive_l = AVOID_DRIVE_SPEED - AVOID_ARC_DIFF;
        s_drive_r = AVOID_DRIVE_SPEED + AVOID_ARC_DIFF;
        s_action = AVOID_AVOIDING;

        if ((now - s_state_tick) >= AVOID_DODGE_MS)
        {
            s_state = AVOID_ST_RETURN_L;
            s_state_tick = now;
            s_line_cnt = 0U;
        }
        break;

    case AVOID_ST_RETURN_R: /* 第二阶段：左前弧回线 */
    case AVOID_ST_RETURN_L:
        if (s_state == AVOID_ST_RETURN_R)
        {
            s_drive_l = AVOID_DRIVE_SPEED - AVOID_ARC_DIFF;
            s_drive_r = AVOID_DRIVE_SPEED + AVOID_ARC_DIFF;
        }
        else
        {
            s_drive_l = AVOID_DRIVE_SPEED + AVOID_ARC_DIFF;
            s_drive_r = AVOID_DRIVE_SPEED - AVOID_ARC_DIFF;
        }
        s_action = AVOID_AVOIDING;

        /* 交还条件：见到黑线（连续 2 次确认，滤掉与中断抢读的偶发失真），
         * 且线已回到阵列中段（|偏差| <= AVOID_REJOIN_POS）——保证真正走完
         * 一段回线弧、车头基本摆正，而不是刚碰到线沿就把控制权撒手 */
        if (Avoid_LineSeen())
        {
            const int16_t pos = Track_GetPosition();

            if ((pos >= -AVOID_REJOIN_POS) && (pos <= AVOID_REJOIN_POS))
            {
                if (++s_line_cnt >= 2U)
                {
                    s_line_cnt = 0U;
                    s_state = AVOID_ST_COOLDOWN;
                    s_state_tick = now;
                    s_action = AVOID_FOLLOW; /* 本轮就交还，主循环随即清掉避障接管 */
                }
            }
            else
            {
                s_line_cnt = 0U;
            }
        }
        else
        {
            s_line_cnt = 0U;
        }

        /* 兜底：回线最长执行 AVOID_RETURN_MAX_MS，超时就交还循迹自己处理 */
        if (((s_state == AVOID_ST_RETURN_R) || (s_state == AVOID_ST_RETURN_L)) &&
            ((now - s_state_tick) >= AVOID_RETURN_MAX_MS))
        {
            s_line_cnt = 0U;
            s_state = AVOID_ST_COOLDOWN;
            s_state_tick = now;
            s_action = AVOID_FOLLOW;
        }
        break;

    case AVOID_ST_COOLDOWN: /* 已交还循迹：等障碍离开或超时再重新武装 */
        s_action = AVOID_FOLLOW;

        if (((fl > AVOID_CLEAR_CM) && (fr > AVOID_CLEAR_CM)) ||
            ((now - s_state_tick) >= AVOID_COOLDOWN_MS))
        {
            s_state = AVOID_ST_FOLLOW;
            s_near_active = 0U;
        }
        break;

    case AVOID_ST_FOLLOW:
    default:
        /* 触发确认：接近状态持续 AVOID_CONFIRM_MS 才启动绕行 */
        if (fmin < AVOID_STOP_CM)
        {
            if (!s_near_active)
            {
                s_near_active = 1U;
                s_near_tick = now;
            }
            else if ((now - s_near_tick) >= AVOID_CONFIRM_MS)
            {
                Avoid_StartDodge();
                break;
            }
        }
        else
        {
            s_near_active = 0U;
        }

        s_action = (fmin < AVOID_SLOW_CM) ? AVOID_SLOW : AVOID_FOLLOW;
        s_drive_l = 0;
        s_drive_r = 0;
        break;
    }

    return s_action;
}

AvoidAction Avoid_GetAction(void)
{
    return s_action;
}

uint8_t Avoid_GetState(void)
{
    return s_state;
}

int16_t Avoid_GetDriveL(void)
{
    return s_drive_l;
}

int16_t Avoid_GetDriveR(void)
{
    return s_drive_r;
}

int16_t Avoid_GetLeftCm(void)
{
    return HCSR04_LastCm(HCSR04_LEFT);
}

int16_t Avoid_GetRightCm(void)
{
    return HCSR04_LastCm(HCSR04_RIGHT);
}

int16_t Avoid_GetFrontCm(void)
{
    const int16_t l = HCSR04_LastCm(HCSR04_LEFT);
    const int16_t r = HCSR04_LastCm(HCSR04_RIGHT);

    if ((l < 0) && (r < 0))
        return -1;
    if (l < 0)
        return r;
    if (r < 0)
        return l;

    return (l < r) ? l : r;
}
