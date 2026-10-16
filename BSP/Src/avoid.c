#include "avoid.h"

#include <stdlib.h>   /* labs */

#include "encoder.h"  /* encoder_left / encoder_right */
#include "motor.h"    /* Motor_SetSpeed / Motor_BrakeAll */

/* ==================== 状态机 ==================== */
typedef enum
{
    ST_FOLLOW = 0,   /* 循迹，只测距 */
    ST_BRAKE,        /* 刹停 */
    ST_DODGE,        /* 转出 45°（远离障碍） */
    ST_PASS,         /* 直行绕过（越过障碍物） */
    ST_BACKTURN,     /* 反向转 65°：车头以 20° 斜对着线 */
    ST_CAPTURE,      /* 回正途中扫到线：刹停抓住 */
    ST_APPROACH,     /* 沿 20° 斜角直线爬回线上 */
    ST_SWEEP         /* 兜底：一直画弧扫线，绝不停车 */
} AvoidState;

/* 状态机上下文：跨状态必须记住的东西全装在这里。
 * side（障碍在哪一侧）同时决定"绕行方向"和"回线方向"，两者必须一致。 */
typedef struct
{
    AvoidState state;
    uint32_t   tick;       /* 进入本状态的时刻（超时用） */
    uint8_t    side;       /* 障碍在哪一侧：0 = 未知，1 = 左，2 = 右 */
    int32_t    ref_l;      /* 进入本状态时，左轮累计脉冲 */
    int32_t    ref_r;      /* 进入本状态时，右轮累计脉冲 */
    AvoidState capture_from; /* 是从哪个阶段进 ST_CAPTURE 的（抓空之后要退回去） */
    uint8_t    slowing;    /* 减速状态的滞回标志（仅 AVOID_USE_SLOW = 1 时用） */
    uint32_t   rearm_tick; /* 上次"交回循迹"的时刻，用于触发冷却 */
    uint8_t    line_cnt;   /* 循迹传感器连续压线的拍数（确认"真的回来了"） */
    uint32_t   line_tick;  /* 上一次统计压线拍数时的传感器采样时刻（防重复计数） */
} AvoidCtx;

static AvoidCtx    ctx;
static AvoidAction action = AVOID_FOLLOW;

/* 避障实际发给电机的指令（屏幕显示用，见 avoid.h） */
volatile int16_t g_avoid_out_l = 0;
volatile int16_t g_avoid_out_r = 0;

/* 各阶段实际走到的最大脉冲数（峰值记录，屏幕显示用） */
volatile uint16_t g_max_dodge    = 0;
volatile uint16_t g_max_pass     = 0;
volatile uint16_t g_max_backturn = 0;

/* 采样限频 */
static uint32_t sample_tick = 0;
static uint8_t  sample_next = 0;

/* 每个探头"连续读到障碍"的次数（用于 AVOID_CONFIRM 确认） */
static uint8_t  hit_streak[HCSR04_ID_COUNT];

/* 已经触发过多少次避障动作（屏幕显示用） */
static uint32_t maneuver_cnt = 0;

/* ==================== 小工具 ==================== */

/* 清空连续命中计数：强制下一次触发重新走一遍"连续确认" */
static void ClearHits(void)
{
    uint8_t i;

    for (i = 0; i < HCSR04_ID_COUNT; i++)
        hit_streak[i] = 0;
}

/* 记录一次测量结果：命中就累加，没命中就清零。
 * AVOID_MIN_VALID_CM 用来滤掉盲区毛刺（驱动把短回波统一报成 2cm）。 */
static void NoteHit(HCSR04_ID id)
{
    const int16_t cm = HCSR04_LastCm(id);

    if (cm >= AVOID_MIN_VALID_CM && cm <= AVOID_TRIGGER_CM)
    {
        if (hit_streak[id] < 250U)
            hit_streak[id]++;
    }
    else
    {
        hit_streak[id] = 0;
    }
}

/* 取距离，-1（无回波 / 超过约 1.37m）折算成 999（很远） */
static int16_t Cm(HCSR04_ID id)
{
    const int16_t cm = HCSR04_LastCm(id);

    return (cm < 0) ? 999 : cm;
}

/* 正前方距离：取所有可用探头里的最小值（-1 折算成 999）。
 * 中间探头没装也不影响，左右两路照常兜底。 */
static int16_t FrontCm(void)
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

    return (front < 0) ? 999 : front;
}

/* 算里程/转角用的编码器累计值。
 *
 * 转出和回正这两个阶段是"一个轮停、另一个轮转"，必须读【在转的那个轮】；
 * 直行的阶段两轮都转，读左轮就够了。
 *
 * 注意：累计值只用绝对值 —— 左轮编码器装了反向修正（ENC_LEFT_INVERT），
 * 正向行驶时它的 total_count 是在减小的，我们只关心"转了多少 / 走了多远"。 */
/* ==================== 里程与转角的量取 ====================
 *
 * ⚠ 转角【必须】用两轮行程差来算，绝对不能只看某一个轮子。
 *
 *   以前的写法（只看正转那个轮）有一个致命后果：
 *   直行阶段两轮都在 +30 正转、累计脉冲一直在涨；切换到回正时那个轮子
 *   只是继续正转，于是"转角"立刻就凑够了 —— 状态机以为回正做完了，
 *   其实车头一点没转。实测现象就是：拐出去之后一路直行、完全没有回正。
 *
 *   两轮行程差在直行时 ≈ 0（两轮同向、走得一样远），
 *   只有车头真的转起来时差值才会涨，这才是转角的正确度量。
 *
 * 标定：车身转 θ 弧度 -> 两轮行程差 = 轮距 × θ
 *       15cm × 1rad = 15cm 行程差 = 15 × 69 = 1035 脉冲/弧度
 *       -> 每度 ≈ 18 脉冲（与转弯方式无关） */
static int32_t EncRot(void)
{
    /* 左轮编码器装了反向修正（ENC_LEFT_INVERT），所以"前进"在左轮原始计数里
     * 是递减、在右轮里是递增 —— 对转时两个增量同号，同向行驶时一正一负，
     * 因此取两者的【和】的绝对值就是转角脉冲（直行时约等于 0）。 */
    const int32_t dl = encoder_left.total_count  - ctx.ref_l;
    const int32_t dr = encoder_right.total_count - ctx.ref_r;

    return labs(dl + dr);
}

/* 直行距离脉冲：两轮行程的平均值（单轮打滑时比只看一个轮子稳）。
 * 69 脉冲/cm：轮径 48mm -> 周长 15.08cm，1040 脉冲/圈。 */
static int32_t EncDist(void)
{
    const int32_t dl = labs(encoder_left.total_count  - ctx.ref_l);
    const int32_t dr = labs(encoder_right.total_count - ctx.ref_r);

    return (dl + dr) / 2;
}

/* 进入一个新状态（记录时间戳和编码器起点） */
static void Enter(AvoidState s)
{
    ctx.state = s;
    ctx.tick  = HAL_GetTick();

    ctx.ref_l     = encoder_left.total_count;
    ctx.ref_r     = encoder_right.total_count;
    ctx.line_cnt  = 0;
    ctx.line_tick = 0;
}

/* 交回循迹：开冷却、清确认计数（下一次触发要重新连续确认） */
static void HandBack(void)
{
    ctx.rearm_tick = HAL_GetTick();
    ClearHits();
    Enter(ST_FOLLOW);
}

/* 循迹传感器连续压线的拍数。连续压够 AVOID_LINE_CONFIRM 拍才算"真的回到线上"。
 * 只擦一下不算 —— 否则交回循迹时车头还歪着，循迹立刻又丢线。 */
static uint8_t LineConfirm(void)
{
    if (g_track_active > 0)
    {
        /* ⚠ 只在"传感器更新过一拍"时才计数：g_track_active 是 TIM1 中断
         * 每 10ms 更新的，而本函数被主循环每 5ms 调一次 —— 不加这个判断，
         * 同一次采样会被算两遍，一个毛刺就能凑够"连续确认"，
         * 把回正提前中止（实测回正凭空消失，就是这里放行的）。 */
        const uint32_t now = HAL_GetTick();

        if ((now - ctx.line_tick) >= 10U)
        {
            ctx.line_tick = now;
            if (ctx.line_cnt < 250U)
                ctx.line_cnt++;
        }
    }
    else
    {
        ctx.line_cnt  = 0;
        ctx.line_tick = 0;
    }

    return (ctx.line_cnt >= AVOID_LINE_CONFIRM) ? 1 : 0;
}

/* 回正途中"抓线"用：比 LineConfirm 松（AVOID_CAPTURE_CONFIRM 拍），
 * 因为转头时线在传感器下面停留时间很短，要求太严就会错过。
 * 抓到之后 ST_CAPTURE 会刹停再验一次，所以这里松一点不会误判。 */
static uint8_t LineCaptureCheck(void)
{
    if (g_track_active > 0)
    {
        const uint32_t now = HAL_GetTick();

        if ((now - ctx.line_tick) >= 10U)
        {
            ctx.line_tick = now;
            if (ctx.line_cnt < 250U)
                ctx.line_cnt++;
        }
    }
    else
    {
        ctx.line_cnt  = 0;
        ctx.line_tick = 0;
    }

    return (ctx.line_cnt >= AVOID_CAPTURE_CONFIRM) ? 1 : 0;
}

/* 轮流采样：每 AVOID_SAMPLE_MS 只测一路，单个探头最坏阻塞 8ms，
 * 主循环不会被长时间占住。这个函数【只能】在主循环里调用。 */
static void Avoid_Sample(void)
{
    HCSR04_ID id;

    if ((HAL_GetTick() - sample_tick) < AVOID_SAMPLE_MS)
        return;

    sample_tick = HAL_GetTick();

#if AVOID_USE_CENTER
    if (sample_next == 0)
    {
        id = HCSR04_CENTER;
        sample_next = 1;
    }
    else if (sample_next == 1)
    {
        id = HCSR04_LEFT;
        sample_next = 2;
    }
    else
    {
        id = HCSR04_RIGHT;
        sample_next = 0;
    }
#else
    if (sample_next == 0)
    {
        id = HCSR04_LEFT;
        sample_next = 1;
    }
    else
    {
        id = HCSR04_RIGHT;
        sample_next = 0;
    }
#endif

    HCSR04_ReadCm(id);
    NoteHit(id);
}

/* ==================== 初始化 ==================== */

void Avoid_Init(void)
{
    HCSR04_Init();

    ctx.state      = ST_FOLLOW;
    ctx.tick       = HAL_GetTick();
    ctx.side       = 0;
    ctx.ref_l      = encoder_left.total_count;
    ctx.ref_r      = encoder_right.total_count;
    ctx.slowing    = 0;
    ctx.rearm_tick = HAL_GetTick() - AVOID_REARM_MS;  /* 上电即可触发，不冷却 */
    ctx.line_cnt   = 0;

    ClearHits();
    maneuver_cnt = 0;

    action      = AVOID_FOLLOW;
    sample_tick = HAL_GetTick();
    sample_next = 0;
}

/* ==================== 状态机主体 ==================== */

AvoidAction Avoid_Update(void)
{
    Avoid_Sample();

    /* 峰值记录：每个阶段推进到过的最大脉冲数。
     * 这样跑完不用抓瞬时，看屏幕就知道哪个阶段没走完。 */
    {
        const uint16_t cur = Avoid_GetStagePulses();

        if (ctx.state == ST_DODGE && cur > g_max_dodge)
            g_max_dodge = cur;
        if (ctx.state == ST_PASS && cur > g_max_pass)
            g_max_pass = cur;
        if (ctx.state == ST_BACKTURN && cur > g_max_backturn)
            g_max_backturn = cur;
    }

    switch (ctx.state)
    {
    /* ---------------- 循迹：只测距，不碰电机 ---------------- */
    case ST_FOLLOW:
    {
        /* 冷却期内不再触发：刚绕完障碍回到线上时车还贴着障碍物，
         * 不复位的话会"刚回线就刹车 -> 又转出去"，走一段停一下。 */
        const uint8_t rearmed =
            ((HAL_GetTick() - ctx.rearm_tick) >= AVOID_REARM_MS);

        /* 用"连续确认过"的命中，而不是单次读数：
         * 超声波偶尔蹦一个假回波，不能让车因此拐出去。 */
        const uint8_t l_hit = (hit_streak[HCSR04_LEFT]  >= AVOID_CONFIRM);
        const uint8_t r_hit = (hit_streak[HCSR04_RIGHT] >= AVOID_CONFIRM);

        if (AVOID_ENABLE && rearmed && (l_hit || r_hit))
        {
            /* 两侧都确认到 -> 取更近的一侧；只有一侧 -> 就是那一侧 */
            if (l_hit && r_hit)
                ctx.side = (Cm(HCSR04_LEFT) <= Cm(HCSR04_RIGHT)) ? 1 : 2;
            else
                ctx.side = l_hit ? 1 : 2;

            ClearHits();     /* 下一次触发要重新连续确认 */
            maneuver_cnt++;  /* 记一笔，屏幕上看得到（N） */

            Enter(ST_BRAKE);
            action = AVOID_BRAKE;
        }
#if AVOID_USE_SLOW
        /* 带滞回：进门槛低、退出门槛高，防止在门槛附近来回跳 */
        else if (ctx.slowing ? (FrontCm() <= AVOID_SLOW_OUT_CM)
                             : (FrontCm() <= AVOID_SLOW_IN_CM))
        {
            ctx.slowing = 1;
            action = AVOID_SLOW;
        }
        else
        {
            ctx.slowing = 0;
            action = AVOID_FOLLOW;
        }
#else
        else
        {
            action = AVOID_FOLLOW;  /* 默认：除了避障，一律正常循迹 */
        }
#endif
        break;
    }

    /* ---------------- 刹停：先停稳，防惯性蹭到 ---------------- */
    case ST_BRAKE:
        if ((HAL_GetTick() - ctx.tick) >= AVOID_BRAKE_MS)
            Enter(ST_DODGE);

        action = AVOID_BRAKE;
        break;

    /* ---------------- 转出：转到偏离原方向 AVOID_DODGE_DEG 度 ---------------- */
    case ST_DODGE:
        if ((EncRot() >= AVOID_DODGE_PULSES) ||
            ((HAL_GetTick() - ctx.tick) >= AVOID_DODGE_MAX_MS))
            Enter(ST_PASS);

        action = AVOID_DODGE;
        break;

    /* ---------------- 直行绕过：走够 AVOID_PASS_PULSES，让车尾也越过障碍 ---------------- */
    case ST_PASS:
        if ((EncDist() >= AVOID_PASS_PULSES) ||
            ((HAL_GetTick() - ctx.tick) >= AVOID_PASS_MAX_MS))
            Enter(ST_BACKTURN);

        action = AVOID_PASS;
        break;

    /* ---------------- 回正：反向转 (转出角 + 回线角)，车头斜对着线 ----------------
     * 这一步是整个回线能不能成的关键：先把车头转正，再交给"斜爬回线"去找线。
     *
     * ⚠ 这里【一定转满固定角度】，不再"扫到线就提前刹停"（详见 avoid.h 的
     *   AVOID_BACKTURN_CAPTURE）。理由：提前停会让回正的实际角度随"线在哪儿
     *   出现"乱变，转角不可复现；而找线这件事交给后面的缓弧爬线更合适。 */
    case ST_BACKTURN:
    {
        const int32_t rotated = EncRot();

#if AVOID_BACKTURN_CAPTURE
        /* 必须先转够 AVOID_CAPTURE_MIN_DEG 才允许抓线：
         * 否则线还挂在传感器排上的时候，回正一开始就被"抓住"刹停，
         * 车头根本没转回来（详见 avoid.h 里这个宏的说明）。 */
        if ((rotated >= (AVOID_CAPTURE_MIN_DEG * AVOID_PULSES_PER_DEG)) &&
            LineCaptureCheck())
        {
            ctx.capture_from = ST_BACKTURN;
            Enter(ST_CAPTURE);
            action = AVOID_CAPTURE;
        }
        else
#endif
        if ((rotated >= AVOID_BACKTURN_PULSES) ||
                 ((HAL_GetTick() - ctx.tick) >= AVOID_BACKTURN_MAX_MS))
        {
            Enter(ST_APPROACH);
            action = AVOID_APPROACH;
        }
        else
        {
            action = AVOID_BACKTURN;
        }
        break;
    }

    /* ---------------- 抓住：刹停确认 ----------------
     * 停稳之后线还在 -> 刚才是真扫到线，而且车基本就停在线上了，
     *   这时交回循迹几乎没有误差（比"边转边交"稳得多）；
     * 停稳之后线不在 -> 刚才是毛刺，按当前斜角继续爬回去。 */
    case ST_CAPTURE:
        if (((HAL_GetTick() - ctx.tick) >= AVOID_CAPTURE_MS) && LineConfirm())
        {
            action = AVOID_FOLLOW;
            HandBack();
        }
        else if ((HAL_GetTick() - ctx.tick) >= (AVOID_CAPTURE_MS * 2U))
        {
            /* 虚惊一场（刚才那次是毛刺）：回到【进来时的那个阶段】继续做完。
             * 以前这里固定进 ST_APPROACH（两轮同速直行），等于把剩下的
             * 回正动作整个吞掉（实测"完全没有回正"）。现在从哪进来就回哪去，
             * 而且直接改 state、保留 ref_l/ref_r，接着上次的角度/距离继续。 */
            ctx.state     = ctx.capture_from;
            ctx.tick      = HAL_GetTick();
            ctx.line_cnt  = 0;
            ctx.line_tick = 0;
            action = (ctx.state == ST_BACKTURN) ? AVOID_BACKTURN : AVOID_APPROACH;
        }
        else
        {
            action = AVOID_CAPTURE;
        }
        break;

    /* ---------------- 沿斜角直线爬回线上 ---------------- */
    case ST_APPROACH:
        /* 斜着爬到线时也先"抓住刹停"再交回，而不是带着 30° 斜向冲力直接交接：
         * 前者是停稳在线上了再交（循迹从静止对正，最稳），
         * 后者是把还在斜冲的车直接丢给循迹，很容易一交就冲过头又丢线。 */
        if (LineCaptureCheck())
        {
            ctx.capture_from = ST_APPROACH;
            Enter(ST_CAPTURE);
            action = AVOID_CAPTURE;
        }
        else if (EncDist() >= AVOID_APPROACH_MAX_PULSES)
        {
            /* 爬了 87cm 还没看到线：可能已经越过线了，退化成画弧扫线接着找 */
            Enter(ST_SWEEP);
            action = AVOID_SWEEP;
        }
        else
        {
            action = AVOID_APPROACH;
        }
        break;

    /* ---------------- 兜底扫线：一直扫，绝不停车 ---------------- */
    case ST_SWEEP:
        if (LineConfirm())
        {
            action = AVOID_FOLLOW;
            HandBack();
        }
        else
        {
            action = AVOID_SWEEP;
        }
        break;

    default:
        action = AVOID_FOLLOW;
        break;
    }

    return action;
}

/* ==================== 电机输出 ==================== */
/*
 * 本函数【必须】由 main.c 在 __disable_irq()/__enable_irq() 之间调用，
 * 否则左右两条 Motor_SetSpeed 之间可能被 TIM1 中断插入，
 * 出现"左轮避障值 + 右轮循迹值"的混合状态，车会猛抽一下。
 */
void Avoid_ApplyMotion(void)
{
    int16_t l = 0;
    int16_t r = 0;
    uint8_t drive = 1;

    switch (ctx.state)
    {
    /* 刹停 / 抓住：直接短接制动 */
    case ST_BRAKE:
    case ST_CAPTURE:
        g_avoid_out_l = 0;
        g_avoid_out_r = 0;
        Motor_BrakeAll();
        return;

    /* 转出：障碍侧轮停(慢)，另一侧轮快
     *   障碍在右(side=2) -> 往左转 -> 右轮快、左轮停
     *   障碍在左(side=1) -> 往右转 -> 左轮快、右轮停 */
    case ST_DODGE:
        if (ctx.side == 1)
        {
            l = AVOID_DODGE_FAST;
            r = AVOID_DODGE_SLOW;
        }
        else
        {
            l = AVOID_DODGE_SLOW;
            r = AVOID_DODGE_FAST;
        }
        break;

    /* 回正：方向与转出【相反】，同样是两轮一正一反的坦克转弯
     * （内侧轮是负值 = 反向驱动，不是停住） */
    case ST_BACKTURN:
        /* ⚠ 先短接刹停 AVOID_BACKTURN_BRAKE_MS 毫秒，把直行留下的惯性消掉。
         * 直行结束时两轮都在正转、车身带惯性，直接让一个轮反转，
         * 它顶着惯性根本倒不过来 —— 前 0.2 秒车头几乎不转，
         * 而那 0.2 秒里"转角"脉冲已经被凑够了（见 EncRot 的说明）。 */
        if ((HAL_GetTick() - ctx.tick) < AVOID_BACKTURN_BRAKE_MS)
        {
            g_avoid_out_l = 0;
            g_avoid_out_r = 0;
            Motor_BrakeAll();
            return;
        }

        if (ctx.side == 1)
        {
            l = AVOID_DODGE_SLOW;
            r = AVOID_BACKTURN_PWM;
        }
        else
        {
            l = AVOID_BACKTURN_PWM;
            r = AVOID_DODGE_SLOW;
        }
        break;

    /* 直行绕过 / 斜着爬回线：两轮同速 */
    case ST_PASS:
        l = AVOID_PASS_PWM;
        r = AVOID_PASS_PWM;
        break;

    case ST_APPROACH:
        /* 很缓的弧（见 AVOID_APPROACH_DIFF 的说明）：弧的横摆范围远大于
         * 可能的几何误差，保证能扫过黑线；方向与回正一致（继续朝线那边转）。 */
        if (ctx.side == 1)
        {
            l = AVOID_APPROACH_PWM - AVOID_APPROACH_DIFF;
            r = AVOID_APPROACH_PWM + AVOID_APPROACH_DIFF;
        }
        else
        {
            l = AVOID_APPROACH_PWM + AVOID_APPROACH_DIFF;
            r = AVOID_APPROACH_PWM - AVOID_APPROACH_DIFF;
        }
        break;

    /* 兜底扫线：画弧朝线的方向扫，两轮都不停（时间到了只是转得更急） */
    case ST_SWEEP:
    {
        const uint8_t urgent = ((HAL_GetTick() - ctx.tick) >= AVOID_SWEEP_MS);
        const int16_t fast = urgent ? AVOID_SWEEP_FAST2 : AVOID_SWEEP_FAST;
        const int16_t slow = urgent ? AVOID_SWEEP_SLOW2 : AVOID_SWEEP_SLOW;

        if (ctx.side == 1)
        {
            l = slow;
            r = fast;
        }
        else
        {
            l = fast;
            r = slow;
        }
        break;
    }

    /* 循迹状态：本函数不该被调用 */
    default:
        drive = 0;
        break;
    }

    if (!drive)
    {
        g_avoid_out_l = 0;
        g_avoid_out_r = 0;
        return;
    }

    g_avoid_out_l = l;
    g_avoid_out_r = r;
    Motor_SetSpeed(MOTOR_LEFT,  MOTOR_START, l);
    Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, r);
}

/* ==================== 对外查询 ==================== */

AvoidAction Avoid_GetAction(void)
{
    return action;
}

int16_t Avoid_GetFrontCm(void)
{
    return FrontCm();
}

int16_t Avoid_GetLeftCm(void)
{
    return HCSR04_LastCm(HCSR04_LEFT);
}

int16_t Avoid_GetRightCm(void)
{
    return HCSR04_LastCm(HCSR04_RIGHT);
}

uint32_t Avoid_GetManeuverCount(void)
{
    return maneuver_cnt;
}

uint16_t Avoid_GetStagePulses(void)
{
    int32_t d;

    /* 转角阶段报转角脉冲，直行/爬线阶段报距离脉冲（屏幕 G 用） */
    if (ctx.state == ST_DODGE || ctx.state == ST_BACKTURN)
        d = EncRot();
    else
        d = EncDist();

    return (d > 9999) ? (uint16_t)9999 : (uint16_t)d;
}
