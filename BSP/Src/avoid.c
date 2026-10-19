#include "avoid.h"

#include <stdlib.h>   /* labs */

#include "encoder.h"  /* encoder_left / encoder_right */
#include "motor.h"    /* Motor_SetSpeed / Motor_BrakeAll */

/* ==================== 状态机 ==================== */
typedef enum
{
    ST_FOLLOW = 0,
    ST_BRAKE,
    ST_DODGE,        /* 第一段：转出 */
    ST_CIRCLE,       /* 第二段：画弧绕障 + 找线 */
    ST_BACKTURN,     /* 第三段：反向回正 + 找线 */
    ST_CAPTURE,      /* 压到线：刹停验证 */
    ST_APPROACH,     /* 缓弧爬线 */
    ST_SWEEP         /* 兜底扫线 */
} AvoidState;

typedef struct
{
    AvoidState state;
    uint32_t   tick;         /* 进入本阶段的时刻（超时用；CAPTURE 期间暂停） */
    int32_t    ref_l;        /* 进入本阶段时的编码器累计值（CAPTURE 不覆盖） */
    int32_t    ref_r;
    uint8_t    side;         /* 障碍方向：1 = 左，2 = 右 */
    AvoidState capture_from; /* 从哪一段进的 CAPTURE（验证失败要退回去） */
    uint32_t   capture_tick; /* 进入 CAPTURE 的时刻 */
    uint32_t   rearm_tick;   /* 上次交回循迹的时刻（冷却） */
} AvoidCtx;

static AvoidCtx    ctx;
static AvoidAction action = AVOID_FOLLOW;

volatile int16_t  g_avoid_out_l  = 0;
volatile int16_t  g_avoid_out_r  = 0;
volatile uint16_t g_max_dodge    = 0;
volatile uint16_t g_max_circle   = 0;
volatile uint16_t g_max_backturn = 0;

static uint32_t sample_tick = 0;
static uint8_t  sample_next = 0;
static uint8_t  hit_streak[HCSR04_ID_COUNT];
static uint8_t  hit_near[HCSR04_ID_COUNT];   /* 最近一次读数落在近区（立刻触发） */
static uint32_t maneuver_cnt = 0;

/* ==================== 测距 ==================== */

static void ClearHits(void)
{
    uint8_t i;

    for (i = 0; i < HCSR04_ID_COUNT; i++)
    {
        hit_streak[i] = 0;
        hit_near[i]   = 0;
    }
}

static void NoteHit(HCSR04_ID id)
{
    const int16_t cm  = HCSR04_LastCm(id);
    const int16_t lim = (id == HCSR04_CENTER) ? AVOID_CENTER_TRIGGER_CM : AVOID_TRIGGER_CM;

    if (cm >= AVOID_MIN_VALID_CM && cm <= lim)
    {
        if (hit_streak[id] < 250U)
            hit_streak[id]++;
    }
    else
    {
        hit_streak[id] = 0;
    }

    hit_near[id] = (cm >= AVOID_MIN_VALID_CM && cm <= AVOID_NEAR_CM) ? 1U : 0U;
}

/* 近区一次命中 或 远区连续确认 */
static uint8_t Hit(HCSR04_ID id)
{
    return (hit_near[id] || hit_streak[id] >= AVOID_CONFIRM) ? 1U : 0U;
}

/* -1（无回波）折算成 999（很远） */
static int16_t Cm(HCSR04_ID id)
{
    const int16_t cm = HCSR04_LastCm(id);

    return (cm < 0) ? 999 : cm;
}

/* ==================== 线上吸引项 ====================
 * 把"线偏在哪边"变成开环 PWM 上的一个差速修正，见 avoid.h 的说明。
 *   返回正数：线偏右 -> 要把车头往右拧 -> 左轮加快、右轮减慢
 *   返回 0  ：没压线 / 线基本居中 / 还没稳（不算）
 * 注意这里只读 g_pos_copy / g_online_cnt 副本，不碰 CD4051 地址线。 */
static int16_t LineCorr(void)
{
    int16_t pos;
    int32_t c;

    if (g_online_cnt < AVOID_LINE_CORR_TICKS)
        return 0;

    pos = g_pos_copy;
    if ((pos > -AVOID_LINE_CORR_DEAD) && (pos < AVOID_LINE_CORR_DEAD))
        return 0;                       /* 线在车头正中：不用修正，免得来回抖 */

    c = ((int32_t)pos * AVOID_LINE_CORR_MAX) / 1000;   /* ±1000 -> ±MAX */

    if (c >  (int32_t)AVOID_LINE_CORR_MAX)
        c =  (int32_t)AVOID_LINE_CORR_MAX;
    if (c < -(int32_t)AVOID_LINE_CORR_MAX)
        c = -(int32_t)AVOID_LINE_CORR_MAX;

    return (int16_t)c;
}

/* 轮流测，每 AVOID_SAMPLE_MS 只测一路（单次最坏阻塞 8ms） */
static void Avoid_Sample(void)
{
#if AVOID_USE_CENTER
    static const HCSR04_ID order[] = {HCSR04_LEFT, HCSR04_CENTER, HCSR04_RIGHT};
#else
    static const HCSR04_ID order[] = {HCSR04_LEFT, HCSR04_RIGHT};
#endif
    HCSR04_ID id;

    if ((HAL_GetTick() - sample_tick) < AVOID_SAMPLE_MS)
        return;

    sample_tick = HAL_GetTick();

    id = order[sample_next];
    if (++sample_next >= (uint8_t)(sizeof(order) / sizeof(order[0])))
        sample_next = 0;

    HCSR04_ReadCm(id);
    NoteHit(id);
}

/* 判断往哪边绕（side：1 = 障碍在左 -> 向右绕；2 = 障碍在右 -> 向左绕）
 *   中间命中：弯道上，按 8 路循迹的线位置决定（线偏右 -> 向右绕）；
 *   只有一侧命中：就是那一侧；
 *   其余：左右谁近就是谁；一样近 -> 默认方向 */
static uint8_t PickSide(uint8_t l_hit, uint8_t r_hit, uint8_t c_hit)
{
    int16_t l, r;

    if (c_hit && g_track_active > 0)
    {
        const int16_t pos = g_pos_copy;

        if (pos >= AVOID_CENTER_POS_DEADBAND)
            return AVOID_CENTER_TURN_TO_LINE ? 1 : 2;
        if (pos <= -AVOID_CENTER_POS_DEADBAND)
            return AVOID_CENTER_TURN_TO_LINE ? 2 : 1;
    }

    if (l_hit && !r_hit)
        return 1;
    if (r_hit && !l_hit)
        return 2;

    l = Cm(HCSR04_LEFT);
    r = Cm(HCSR04_RIGHT);
    if (l < r)
        return 1;
    if (r < l)
        return 2;
    return AVOID_DEFAULT_SIDE;
}

/* ==================== 编码器量取 ==================== */

/* 转角：两轮行程差（左轮计数方向是反的，所以取"和"）。直行时 ≈ 0 */
static int32_t EncRot(void)
{
    const int32_t dl = encoder_left.total_count  - ctx.ref_l;
    const int32_t dr = encoder_right.total_count - ctx.ref_r;

    return labs(dl + dr);
}

/* 里程：两轮行程平均 */
static int32_t EncDist(void)
{
    const int32_t dl = labs(encoder_left.total_count  - ctx.ref_l);
    const int32_t dr = labs(encoder_right.total_count - ctx.ref_r);

    return (dl + dr) / 2;
}

/* ==================== 状态切换 ==================== */

static void Enter(AvoidState s)
{
    ctx.state        = s;
    ctx.tick         = HAL_GetTick();
    ctx.ref_l        = encoder_left.total_count;
    ctx.ref_r        = encoder_right.total_count;
}

/* 进入刹停验证：不动 tick / ref，验证失败时原阶段可以接着做 */
static void EnterCapture(void)
{
    ctx.capture_from = ctx.state;
    ctx.capture_tick = HAL_GetTick();
    ctx.state        = ST_CAPTURE;
}

/* 验证失败：退回原阶段，超时计时扣掉刹停验证花掉的时间 */
static void LeaveCapture(void)
{
    ctx.tick += HAL_GetTick() - ctx.capture_tick;
    ctx.state = ctx.capture_from;
}

static void HandBack(void)
{
    ctx.rearm_tick = HAL_GetTick();
    ClearHits();
    Enter(ST_FOLLOW);
}

/* 连续压线 need 拍（10ms/拍）。
 * ⚠ 必须用 TIM1 中断里数的 g_online_cnt，不能在主循环里自己数：
 *   主循环一圈要刷 OLED（软件 I2C）+ 测超声波，间隔可能几十毫秒，
 *   两次"看到线"之间丢没丢线根本看不见，擦边扫过也会被凑成"连续压线"。 */
static uint8_t LineSeen(uint8_t need)
{
    return (g_online_cnt >= need) ? 1U : 0U;
}

static AvoidAction StateAction(AvoidState s)
{
    switch (s)
    {
    case ST_BRAKE:    return AVOID_BRAKE;
    case ST_DODGE:    return AVOID_DODGE;
    case ST_CIRCLE:   return AVOID_CIRCLE;
    case ST_BACKTURN: return AVOID_BACKTURN;
    case ST_CAPTURE:  return AVOID_CAPTURE;
    case ST_APPROACH: return AVOID_APPROACH;
    case ST_SWEEP:    return AVOID_SWEEP;
    default:          return AVOID_FOLLOW;
    }
}

/* ==================== 初始化 ==================== */

void Avoid_Init(void)
{
    HCSR04_Init();

    Enter(ST_FOLLOW);
    ctx.side       = 0;
    ctx.rearm_tick = HAL_GetTick() - AVOID_REARM_MS;  /* 上电即可触发 */

    ClearHits();
    maneuver_cnt = 0;
    action       = AVOID_FOLLOW;
    sample_tick  = HAL_GetTick();
    sample_next  = 0;
}

/* ==================== 状态机主体 ==================== */

AvoidAction Avoid_Update(void)
{
    const uint32_t now = HAL_GetTick();

    Avoid_Sample();

    {
        const uint16_t cur = Avoid_GetStagePulses();

        if (ctx.state == ST_DODGE && cur > g_max_dodge)
            g_max_dodge = cur;
        if (ctx.state == ST_CIRCLE && cur > g_max_circle)
            g_max_circle = cur;
        if (ctx.state == ST_BACKTURN && cur > g_max_backturn)
            g_max_backturn = cur;
    }

    switch (ctx.state)
    {
    case ST_FOLLOW:
    {
        const uint8_t l_hit = Hit(HCSR04_LEFT);
        const uint8_t r_hit = Hit(HCSR04_RIGHT);
#if AVOID_USE_CENTER
        const uint8_t c_hit = Hit(HCSR04_CENTER);
#else
        const uint8_t c_hit = 0;
#endif

        if (AVOID_ENABLE &&
            ((now - ctx.rearm_tick) >= AVOID_REARM_MS) &&
            (l_hit || r_hit || c_hit))
        {
            ctx.side = PickSide(l_hit, r_hit, c_hit);
            ClearHits();
            maneuver_cnt++;
            g_max_dodge = g_max_circle = g_max_backturn = 0;
            Enter(ST_BRAKE);
        }
        break;
    }

    case ST_BRAKE:
        if ((now - ctx.tick) >= AVOID_BRAKE_MS)
            Enter(ST_DODGE);
        break;

    /* 第一段：转出 */
    case ST_DODGE:
        if ((EncRot() >= AVOID_DODGE_PULSES) ||
            ((now - ctx.tick) >= AVOID_DODGE_MAX_MS))
            Enter(ST_CIRCLE);
        break;

    /* 第二段：全程画弧（PWM 上一直叠着线上吸引项，见 Avoid_ApplyMotion）。
     * 画弧走够 AVOID_CIRCLE_GRAB_MIN 之后，只要连续压线 AVOID_CIRCLE_LINE_TICKS 拍
     * 就去刹停验证 —— 这里【没有"循迹窗口"】：原来的 10~20cm 窗口到 20cm 就把
     * 控制权强行收回开环画弧，等于"车已经压在线上、循迹也跟上了，还要把它揪下来"，
     * 之后 30cm 回正 + 35cm 爬线全是盲走，这就是第二阶段永远交不回去的原因。 */
    case ST_CIRCLE:
    {
        const int32_t dist = EncDist();

        /* ① 抓线优先（走够起步距离之后，且线不是横着切过来的） */
        if ((dist >= AVOID_CIRCLE_GRAB_MIN) &&
            (g_track_active <= AVOID_GRAB_MAX_ACTIVE) &&
            LineSeen(AVOID_CIRCLE_LINE_TICKS))
        {
            EnterCapture();
            break;
        }

        /* ② 画弧走满 30cm / 超时：此刻还压着线就直接验证，别去转回正 ——
         *    回正是朝反方向拧，会把刚抓到的线甩掉 */
        if ((dist >= AVOID_CIRCLE_PULSES) || ((now - ctx.tick) >= AVOID_CIRCLE_MAX_MS))
        {
            if (LineSeen(AVOID_CAPTURE_CONFIRM))
            {
                EnterCapture();
            }
            else
            {
                /* 刹停消惯性，进入回正后 ApplyMotion 里还会再刹一次，
                 * 两段加起来把画弧的惯性彻底消掉，回正才转得动 */
                Motor_BrakeAll();
                HAL_Delay(AVOID_BACKTURN_BRAKE_MS);
                Enter(ST_BACKTURN);
            }
        }
        break;
    }

    /* 第三段：反向回正，转过最小角度后也在找线 */
    case ST_BACKTURN:
    {
        const int32_t rotated = EncRot();

        if ((rotated >= (AVOID_CAPTURE_MIN_DEG * AVOID_PULSES_PER_DEG)) &&
            LineSeen(AVOID_CAPTURE_CONFIRM))
            EnterCapture();
        else if ((rotated >= AVOID_BACKTURN_PULSES) ||
                 ((now - ctx.tick) >= AVOID_BACKTURN_MAX_MS))
            Enter(ST_APPROACH);
        break;
    }

    case ST_APPROACH:
        if (LineSeen(AVOID_CAPTURE_CONFIRM))
            EnterCapture();
        else if (EncDist() >= AVOID_APPROACH_MAX_PULSES)
            Enter(ST_SWEEP);
        break;

    case ST_SWEEP:
        if (LineSeen(AVOID_CAPTURE_CONFIRM))
            EnterCapture();
        break;

    /* 刹停验证：停稳后线还在 -> 交回循迹；否则退回原阶段继续 */
    case ST_CAPTURE:
    {
        const uint32_t held = now - ctx.capture_tick;
        const uint8_t  seen = LineSeen(AVOID_LINE_CONFIRM);

        /* 停稳后发现线是横着切过传感器排的（压了一大片）：
         * 这个角度交回循迹，一给电就是"横穿线 -> 立刻丢线"。
         * 退回原阶段（画弧继续走），等线和车身夹角小了自然会再抓一次。
         * ⚠ 只在"从第二段（画弧）进来的"时候退回：画弧继续走是往前走，
         *   退回一定是有进展的；从兜底扫线/回正退回则是原地接着转，
         *   会变成"抓了又放"的死循环，那还不如交回去让它自己找。 */
        if ((held >= AVOID_CAPTURE_MS) &&
            (ctx.capture_from == ST_CIRCLE) &&
            (g_track_active > AVOID_GRAB_MAX_ACTIVE))
        {
            LeaveCapture();
        }
        else if (held >= AVOID_CAPTURE_MS && seen)
            HandBack();
        else if (held >= (AVOID_CAPTURE_MS * 2U))
            LeaveCapture();
        break;
    }

    default:
        HandBack();
        break;
    }

    action = StateAction(ctx.state);
    return action;
}

/* ==================== 电机输出 ==================== */
/* 必须由 main.c 在 __disable_irq()/__enable_irq() 之间调用 */
void Avoid_ApplyMotion(void)
{
    /* 按 side = 1（障碍在左）写的左右轮 PWM；side = 2 时左右互换 */
    int16_t l1 = 0;
    int16_t r1 = 0;

    switch (ctx.state)
    {
    case ST_BRAKE:
    case ST_CAPTURE:
        g_avoid_out_l = 0;
        g_avoid_out_r = 0;
        Motor_BrakeAll();
        return;

    /* 第一段：坦克转，远离障碍 */
    case ST_DODGE:
        l1 =  AVOID_TURN_PWM;
        r1 = -AVOID_TURN_PWM;
        break;

    /* 第二段：画弧，朝障碍一侧弯回去（以障碍为圆心） */
    case ST_CIRCLE:
        l1 = AVOID_CIRCLE_SLOW;
        r1 = AVOID_CIRCLE_FAST;
        break;

    /* 第三段：先刹停消惯性，再反向坦克转 */
    case ST_BACKTURN:
        if ((HAL_GetTick() - ctx.tick) < AVOID_BACKTURN_BRAKE_MS)
        {
            g_avoid_out_l = 0;
            g_avoid_out_r = 0;
            Motor_BrakeAll();
            return;
        }
        l1 = -AVOID_TURN_PWM;
        r1 =  AVOID_TURN_PWM;
        break;

    case ST_APPROACH:
        l1 = AVOID_APPROACH_PWM - AVOID_APPROACH_DIFF;
        r1 = AVOID_APPROACH_PWM + AVOID_APPROACH_DIFF;
        break;

    case ST_SWEEP:
    {
        const uint8_t urgent = ((HAL_GetTick() - ctx.tick) >= AVOID_SWEEP_MS);

        l1 = urgent ? AVOID_SWEEP_SLOW2 : AVOID_SWEEP_SLOW;
        r1 = urgent ? AVOID_SWEEP_FAST2 : AVOID_SWEEP_FAST;
        break;
    }

    default:
        g_avoid_out_l = 0;
        g_avoid_out_r = 0;
        return;
    }

    if (ctx.side == 1)
    {
        g_avoid_out_l = l1;
        g_avoid_out_r = r1;
    }
    else
    {
        g_avoid_out_l = r1;
        g_avoid_out_r = l1;
    }

    /* ==================== 叠加"线上吸引项" ====================
     * 找线的每一段（画弧 / 爬线 / 扫线）只要传感器排压着线，就按偏差补差速，
     * 把轨迹往线上拉。这样"压到线"不是一次模式切换，而是同一条命令里连续的一段
     * —— 没有切换，也就没有空档。
     *   不加的两段：
     *     DODGE    要坚决转出去，不能被线牵着回来；
     *     BACKTURN 是原地反向转（本身就是"传感器排横扫"的动作），
     *              加差速只会把转身搅乱，它转完由 APPROACH 接着找线。 */
    if ((ctx.state == ST_CIRCLE) || (ctx.state == ST_APPROACH) ||
        (ctx.state == ST_SWEEP))
    {
        const int16_t c = LineCorr();

        g_avoid_out_l += c;
        g_avoid_out_r -= c;

        /* 修正不许把内侧轮顶成反转：反转 = 原地打转，车头 20cm 长的传感器排
         * 会横向甩出去，线瞬间离开探头（"刚抓到线就丢线"就是这么来的）。
         * 留一点前进分量，线是"顺着"滑回中心的。 */
        if (g_avoid_out_l < 0)
            g_avoid_out_l = 0;
        if (g_avoid_out_r < 0)
            g_avoid_out_r = 0;
    }

    Motor_SetSpeed(MOTOR_LEFT,  MOTOR_START, g_avoid_out_l);
    Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, g_avoid_out_r);
}

/* ==================== 对外查询 ==================== */

AvoidAction Avoid_GetAction(void)   { return action; }
int16_t Avoid_GetLeftCm(void)       { return HCSR04_LastCm(HCSR04_LEFT); }
int16_t Avoid_GetCenterCm(void)     { return HCSR04_LastCm(HCSR04_CENTER); }
int16_t Avoid_GetRightCm(void)      { return HCSR04_LastCm(HCSR04_RIGHT); }
uint32_t Avoid_GetManeuverCount(void) { return maneuver_cnt; }

uint16_t Avoid_GetStagePulses(void)
{
    const int32_t d = (ctx.state == ST_DODGE || ctx.state == ST_BACKTURN) ? EncRot() : EncDist();

    return (d > 9999) ? (uint16_t)9999 : (uint16_t)d;
}

uint8_t Avoid_IsActive(void)
{
    return (ctx.state != ST_FOLLOW) ? 1U : 0U;
}

uint8_t Avoid_GetLastSide(void)
{
    if (ctx.state != ST_FOLLOW || (HAL_GetTick() - ctx.rearm_tick) < AVOID_REARM_MS)
        return ctx.side;

    return 0;
}
