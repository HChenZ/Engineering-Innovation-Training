#include "avoid.h"

/* 避障状态
 *   FOLLOW -> 正常循迹
 *   BRAKE  -> 先停稳，停稳后再决定绕行方向
 *   TURN   -> 原地转向保持中
 *   BACK   -> 后退保持中（默认不启用，见 avoid.h 的 AVOID_ALLOW_BACK） */
typedef enum
{
    AVOID_STATE_FOLLOW = 0,
    AVOID_STATE_BRAKE,
    AVOID_STATE_TURN,
    AVOID_STATE_BACK
} AvoidState;

static AvoidState state = AVOID_STATE_FOLLOW;
static AvoidAction action = AVOID_FOLLOW;

static uint32_t state_tick = 0;  /* 状态切换时刻 */
static uint32_t sample_tick = 0; /* 上次采样时刻 */
static uint8_t sample_next = 0;  /* 下一次测哪一路 */

/* 取距离，-1（无回波）当作很远 */
static int16_t Avoid_Cm(HCSR04_ID id)
{
    const int16_t cm = HCSR04_LastCm(id);

    return (cm < 0) ? 999 : cm;
}

/* 正前方距离（原始值，-1 表示相关探头都没有回波）
 *
 * 左右两路都算进来，再叠加可选的中间探头，取最小值：
 * 这样即使中间探头忘了插（读数恒为 -1），逻辑依然由左右两路兜底，
 * 不会出现"正前方永远判定通畅"的哑火。 */
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

void Avoid_Init(void)
{
    HCSR04_Init();

    state = AVOID_STATE_FOLLOW;
    action = AVOID_FOLLOW;
    state_tick = HAL_GetTick();
    sample_tick = HAL_GetTick();
    sample_next = 0;
}

/* 轮流采样（对外公开：停车/待机时由主循环直接调用，保持距离数据新鲜）。
 * 每 AVOID_SAMPLE_MS 只测一路，单个探头最坏阻塞 8ms，
 * 主循环不会被长时间占住。 */
void Avoid_Sample(void)
{
    if ((HAL_GetTick() - sample_tick) < AVOID_SAMPLE_MS)
        return;

    sample_tick = HAL_GetTick();

#if AVOID_USE_CENTER
    switch (sample_next)
    {
    case 0:
        HCSR04_ReadCm(HCSR04_CENTER);
        sample_next = 1;
        break;

    case 1:
        HCSR04_ReadCm(HCSR04_LEFT);
        sample_next = 2;
        break;

    default:
        HCSR04_ReadCm(HCSR04_RIGHT);
        sample_next = 0;
        break;
    }
#else
    if (sample_next == 0)
    {
        HCSR04_ReadCm(HCSR04_LEFT);
        sample_next = 1;
    }
    else
    {
        HCSR04_ReadCm(HCSR04_RIGHT);
        sample_next = 0;
    }
#endif
}

/* 往更空的一侧原地转向（原地转不会触发"逆行"判罚） */
static AvoidAction Avoid_StartTurn(void)
{
    const int16_t left = Avoid_Cm(HCSR04_LEFT);
    const int16_t right = Avoid_Cm(HCSR04_RIGHT);

    state = AVOID_STATE_TURN;
    state_tick = HAL_GetTick();
    action = (left > right) ? AVOID_TURN_LEFT : AVOID_TURN_RIGHT;

    return action;
}

/* 前方已经进入危险距离，选一个"逃出去"的动作 */
static AvoidAction Avoid_Escape(void)
{
#if AVOID_ALLOW_BACK
    const int16_t left = Avoid_Cm(HCSR04_LEFT);
    const int16_t right = Avoid_Cm(HCSR04_RIGHT);

    /* 只有允许后退时，两侧都堵才倒车 */
    if (left <= AVOID_STOP_CM && right <= AVOID_STOP_CM)
    {
        state = AVOID_STATE_BACK;
        state_tick = HAL_GetTick();
        action = AVOID_BACK;
        return action;
    }
#endif

    /* 默认策略：后退有被判"逆行"的风险，所以一律改成原地转向 */
    return Avoid_StartTurn();
}

/* 刹车结束后决定下一步：恢复循迹还是绕行 */
static AvoidAction Avoid_AfterBrake(void)
{
    if (Avoid_FrontCm() > AVOID_STOP_CM)
    {
        state = AVOID_STATE_FOLLOW;
        state_tick = HAL_GetTick();
        action = AVOID_FOLLOW;
        return action;
    }

    return Avoid_Escape();
}

/* 正常状态的判断 */
static AvoidAction Avoid_Decide(void)
{
    const int16_t front = Avoid_FrontCm();

    switch (state)
    {
    case AVOID_STATE_TURN:
        if ((HAL_GetTick() - state_tick) < AVOID_TURN_MS)
            return action;

        state = AVOID_STATE_FOLLOW;
        state_tick = HAL_GetTick();
        break;

    case AVOID_STATE_BACK:
#if AVOID_ALLOW_BACK
        if ((HAL_GetTick() - state_tick) < AVOID_BACK_MS)
            return AVOID_BACK;

        /* 退完先停一下再看，避免刚退完就贴上去 */
        state = AVOID_STATE_BRAKE;
        state_tick = HAL_GetTick();
        action = AVOID_BRAKE;
        return action;
#else
        state = AVOID_STATE_FOLLOW;
        state_tick = HAL_GetTick();
        break;
#endif

    default:
        break;
    }

    /* 前方安全：继续循迹，靠近了就先减速 */
    if (front > AVOID_STOP_CM)
    {
        if (front <= AVOID_SLOW_CM)
            return AVOID_SLOW;

        return AVOID_FOLLOW;
    }

    /* 前方进入危险距离：先刹车停稳，停稳后再决定往哪边绕 */
    state = AVOID_STATE_BRAKE;
    state_tick = HAL_GetTick();
    action = AVOID_BRAKE;
    return action;
}

AvoidAction Avoid_Update(void)
{
    Avoid_Sample();

    if (state == AVOID_STATE_BRAKE)
    {
        /* 刹车时间到：用最新读数决定是恢复循迹还是转向 */
        if ((HAL_GetTick() - state_tick) < AVOID_BRAKE_MS)
            action = AVOID_BRAKE;
        else
            action = Avoid_AfterBrake();
    }
    else
    {
        action = Avoid_Decide();
    }

    return action;
}

AvoidAction Avoid_GetAction(void)
{
    return action;
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
