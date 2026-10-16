#include "avoid.h"
#include "track.h"
#include "encoder.h"

/* ==================== 避障状态机 ====================
 * 针对"不规则波浪线 + 随机障碍物"设计。
 *
 * 关键事实（来自比赛规则 + 实车）：
 *   障碍物都在循迹线【任意一侧】、不在线上 -> 线始终可见，不会被挡住；
 *   转弯慢、是弧线（内外轮差速），弧线的弧长前进分量足够越过障碍物，
 *   所以【不要直行段】：转出 + 摆正两个弧线直接闭合，成一个"S"形绕行。
 *   转出弧线横向位移 ≈ R(1-cosθ) > 车半宽 7.5cm（错开障碍物），
 *   纵向位移 ≈ R·sinθ ≈ 30cm（越过最大障碍物 15cm），
 *   摆正弧线对称转回同样角度，车头回正、回到线附近。
 *
 * 动作序列（障碍在某一侧时，朝它【反方向】绕）：
 *   ① TURN_AWAY 转出   差速弧线，侧向超声波闭环（障碍离开侧方才停），
 *                       最小45°/最大60°兜底，结束时记录实际转角 away_pulses。
 *   ② TURN_BACK  摆正   差速弧线，反方向转回【转出同样角度】车头回正（不看线）。
 *   ③ SEARCH     找回   朝【线最后消失的那一侧】转+扫，压线立即交回。
 *   ④ RESUME     恢复   低速循迹过渡，让位置环把车头扳正再提速。
 */
typedef enum
{
    AVOID_STATE_FOLLOW = 0,  /* 正常循迹 */
    AVOID_STATE_TURN_AWAY,   /* ① 转出弧线（侧向闭环） */
    AVOID_STATE_TURN_BACK,   /* ② 摆正弧线（转回同角） */
    AVOID_STATE_SEARCH,      /* ③ 找回（朝线消失方向） */
    AVOID_STATE_RESUME       /* ④ 低速恢复 */
} AvoidState;

static AvoidState state = AVOID_STATE_FOLLOW;
static AvoidAction action = AVOID_FOLLOW;
static int8_t away_dir = 0;         /* 绕行方向：-1=往左, +1=往右 */
static int8_t line_last_side = 0;   /* 线最后在车哪侧：-1左 +1右 0未知 */
static uint32_t state_tick = 0;     /* 状态切换时刻 */
static uint32_t sample_tick = 0;    /* 上次采样时刻 */
static uint32_t cooldown_until = 0; /* 绕行后的冷却截止时刻 */
static int32_t turn_base = 0;       /* 转向起始时 (右+左) 的累计脉冲 */

/* 转 deg 度需要左右轮累计的脉冲差(绝对值)：
 *   每脉冲行进 = π·D / CPR
 *   脉冲差 = 转角(rad) × 轮距 / 每脉冲行进 */
static int32_t Avoid_TurnTargetPulses(float deg)
{
    const float per_pulse_cm = 3.14159265f * AVOID_WHEEL_D_CM / (float)ENCODER_CPR;
    const float rad = deg * 3.14159265f / 180.0f;

    return (int32_t)(rad * AVOID_WHEELBASE_CM / per_pulse_cm + 0.5f);
}

/* 自 turn_base 起，左右轮【前进距离】之差(带符号，脉冲)。
 *
 * 注意编码器方向：左轮编码器是反的(main.c 的 ENC_LEFT_INVERT=1)，
 * 前进时 encoder_left.total_count 往【负】走，右轮往正走。
 * 所以"右轮前进 - 左轮前进" = right - (-left) = right + left，
 * 不是直觉上的 right - left。用错符号会把"差"算成"和"，转角判据整个失效。 */
static int32_t Avoid_TurnDiff(void)
{
    return (encoder_right.total_count + encoder_left.total_count) - turn_base;
}

/* 转角脉冲差的绝对值 */
static int32_t Avoid_TurnMag(void)
{
    const int32_t turned = Avoid_TurnDiff();

    return (turned < 0) ? -turned : turned;
}

/* 记录转向起点（每次开始一段转向时调用） */
static void Avoid_TurnReset(void)
{
    turn_base = encoder_right.total_count + encoder_left.total_count;
}

/* 取距离，-1（无回波）当作很远 */
static int16_t Avoid_Cm(HCSR04_ID id)
{
    const int16_t cm = HCSR04_LastCm(id);

    return (cm < 0) ? 999 : cm;
}

/* 障碍所在的探头：往右绕(障碍在左)用左探头，往左绕(障碍在右)用右探头 */
static HCSR04_ID Avoid_ObstacleId(void)
{
    return (away_dir > 0) ? HCSR04_LEFT : HCSR04_RIGHT;
}

/* 找回方向：朝线最后消失的那一侧转；线未知则退化朝绕行反方向转 */
static AvoidAction Avoid_SearchDir(void)
{
    int8_t dir = line_last_side;

    if (dir == 0)
        dir = (int8_t)(-away_dir);   /* 未知：朝绕行反方向（即线的大致方向） */

    return (dir > 0) ? AVOID_TURN_RIGHT : AVOID_TURN_LEFT;
}

/* 正前方距离（左/右两路取最小；-1 表示两路都没有回波） */
static int16_t Avoid_FrontRawCm(void)
{
    int16_t front = -1;
    int16_t cm;

    cm = HCSR04_LastCm(HCSR04_LEFT);
    if (cm > 0 && (front < 0 || cm < front))
        front = cm;

    cm = HCSR04_LastCm(HCSR04_RIGHT);
    if (cm > 0 && (front < 0 || cm < front))
        front = cm;

#if AVOID_USE_CENTER
    cm = HCSR04_LastCm(HCSR04_CENTER);
    if (cm > 0 && (front < 0 || cm < front))
        front = cm;
#endif

    return front;
}

/* 正前方距离（-1 换成 999，便于比较） */
static int16_t Avoid_FrontCm(void)
{
    const int16_t cm = Avoid_FrontRawCm();

    return (cm < 0) ? 999 : cm;
}

/* 障碍偏哪一侧：-1=偏左, +1=偏右, 0=判不出（两侧都远）
 * 判据：距离小的一侧就是障碍所在侧。 */
int8_t Avoid_ObstacleSide(void)
{
    const int16_t left = Avoid_Cm(HCSR04_LEFT);
    const int16_t right = Avoid_Cm(HCSR04_RIGHT);

    if (left > AVOID_SLOW_CM && right > AVOID_SLOW_CM)
        return 0;   /* 两侧都够远，判不出也无所谓 */

    if (left < right)
        return -1;  /* 左边更近 -> 障碍在左 */
    if (right < left)
        return 1;   /* 右边更近 -> 障碍在右 */

    return 1;       /* 一样近（含两侧都无回波）：默认当障碍在右，往左绕 */
}

void Avoid_Init(void)
{
    HCSR04_Init();

    state = AVOID_STATE_FOLLOW;
    action = AVOID_FOLLOW;
    away_dir = 0;
    line_last_side = 0;
    state_tick = HAL_GetTick();
    sample_tick = HAL_GetTick();
    cooldown_until = 0;
}

/* 采样：每个周期把左右两个探头都读一遍（反应更快） */
static void Avoid_Sample(void)
{
    if ((HAL_GetTick() - sample_tick) < AVOID_SAMPLE_MS)
        return;

    sample_tick = HAL_GetTick();

    HCSR04_ReadCm(HCSR04_LEFT);
    HCSR04_ReadCm(HCSR04_RIGHT);

#if AVOID_USE_CENTER
    HCSR04_ReadCm(HCSR04_CENTER);
#endif
}

/* 开始绕行：按障碍所在侧决定往哪边绕（在左->往右，在右->往左），
 * 并记录线最后在车哪一侧，供找回阶段使用。 */
static AvoidAction Avoid_StartAway(void)
{
    int8_t side;

    /* 【关键】决定方向前强制重读两个探头。
     * 否则用的是上一轮采样留下的旧读数（可能还没刷新到"谁近"），
     * 会导致方向判断经常落到同一个分支（表现为"永远往右绕"）。 */
    HCSR04_ReadCm(HCSR04_LEFT);
    HCSR04_ReadCm(HCSR04_RIGHT);
    side = Avoid_ObstacleSide();

    away_dir = (side == -1) ? 1 : -1;   /* 障碍在左 -> 往右绕；障碍在右 -> 往左绕 */
    line_last_side = Track_GetLastSide(); /* 线最后在车哪侧，找回时朝它转 */

    Avoid_TurnReset();   /* 记录转向起点，后面按编码器算转过的角度 */

    state = AVOID_STATE_TURN_AWAY;
    state_tick = HAL_GetTick();
    action = (away_dir > 0) ? AVOID_TURN_RIGHT : AVOID_TURN_LEFT;

    return action;
}

/* 正常状态下的判断 */
static AvoidAction Avoid_Decide(void)
{
    const int16_t front = Avoid_FrontCm();
    const uint32_t now = HAL_GetTick();

    switch (state)
    {
    /* ---- ① 转出：朝障碍反方向差速弧线，侧向超声波闭环 ----
     * 转够最小角前不因"探头离开"而提前结束（防误判）；
     * 之后障碍物离开侧方探头视野就结束 —— 障碍大自动多绕、小自动少绕。 */
    case AVOID_STATE_TURN_AWAY:
    {
        const int32_t mag = Avoid_TurnMag();
        const int16_t obs = Avoid_Cm(Avoid_ObstacleId());

        if (mag < Avoid_TurnTargetPulses(AVOID_AWAY_MIN_DEG))
        {
            /* 没转够最小角：只管转 */
            if ((now - state_tick) < AVOID_AWAY_MAX_MS)
                return action;
        }
        else if (obs <= AVOID_CLEAR_CM &&
                 mag < Avoid_TurnTargetPulses(AVOID_AWAY_MAX_DEG) && (now - state_tick) < AVOID_AWAY_MAX_MS)
        {
            /* 障碍还在侧方、且没到最大角：继续转 */
            return action;
        }

        /* 转出结束：直接摆正（两个弧线闭合，不直行） */
        Avoid_TurnReset();
        state = AVOID_STATE_TURN_BACK;
        state_tick = now;
        action = (away_dir > 0) ? AVOID_TURN_LEFT : AVOID_TURN_RIGHT;
        return action;
    }

    /* ---- ② 摆正：反方向差速弧线，转固定角度，车头回正 ----
     * 【前几步不看线】此段不查循迹传感器，先转够角度把车头摆正。
     * 转角固定为 AVOID_BACK_DEG，配合转出角自己调弧线。 */
    case AVOID_STATE_TURN_BACK:
        if (Avoid_TurnMag() < Avoid_TurnTargetPulses(AVOID_BACK_DEG) &&
            (now - state_tick) < AVOID_BACK_MAX_MS)
            return action;

        /* 摆正完：进入找回，朝线最后消失的那一侧转 */
        Avoid_TurnReset();
        state = AVOID_STATE_SEARCH;
        state_tick = now;
        action = Avoid_SearchDir();
        return action;

    /* ---- ③ 找回：朝线最后消失的那一侧转+扫，压到线就进入低速恢复 ---- */
    case AVOID_STATE_SEARCH:
        if (!Track_IsLost())
        {
            state = AVOID_STATE_RESUME;
            state_tick = now;
            return AVOID_SLOW;
        }

        /* 转够设定角还没压到线也交回，交给循迹丢线逻辑继续找 */
        if (Avoid_TurnMag() < Avoid_TurnTargetPulses(AVOID_SEARCH_DEG) &&
            (now - state_tick) < AVOID_SEARCH_MAX_MS)
            return action;

        state = AVOID_STATE_RESUME;
        state_tick = now;
        return AVOID_SLOW;

    /* ---- ④ 低速恢复：先慢速循迹一段时间，让位置环把车头扳正再提速 ---- */
    case AVOID_STATE_RESUME:
        if ((now - state_tick) < AVOID_RESUME_MS)
            return AVOID_SLOW;

        state = AVOID_STATE_FOLLOW;
        state_tick = now;
        cooldown_until = now + AVOID_COOLDOWN_MS;
        return AVOID_FOLLOW;

    default:
        break;
    }

    /* 绕行冷却期内：无视障碍读数，直接循迹往前走 */
    if ((int32_t)(now - cooldown_until) < 0)
        return AVOID_FOLLOW;

    /* 前方安全：继续循迹，靠近了就先减速 */
    if (front > AVOID_STOP_CM)
    {
        if (front <= AVOID_SLOW_CM)
            return AVOID_SLOW;

        return AVOID_FOLLOW;
    }

    /* 进入危险距离：立即开始绕行 */
    return Avoid_StartAway();
}

AvoidAction Avoid_Update(void)
{
    Avoid_Sample();
    action = Avoid_Decide();

    return action;
}

AvoidAction Avoid_GetAction(void)
{
    return action;
}

uint8_t Avoid_GetState(void)
{
    return (uint8_t)state;
}

int8_t Avoid_GetAwayDir(void)
{
    return away_dir;
}

int16_t Avoid_GetFrontCm(void)
{
    return Avoid_FrontRawCm();
}

int16_t Avoid_GetLeftCm(void)
{
    return HCSR04_LastCm(HCSR04_LEFT);
}

int16_t Avoid_GetRightCm(void)
{
    return HCSR04_LastCm(HCSR04_RIGHT);
}
