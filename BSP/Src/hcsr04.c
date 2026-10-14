#include "hcsr04.h"
#include "tim.h"

/* ============================================================================
 * HC-SR04 测距（TIM1 CH3/CH4 输入捕获）
 *
 * 每路 Echo 一个独立状态机：
 *   IDLE ──触发──> WAIT_RISE ──捕获到上升沿，记下时刻并切下降沿──> WAIT_FALL
 *        ──捕获到下降沿，算宽度、恢复上升沿捕获──> IDLE
 *
 * 为什么这样设计：
 *   1) 全程非阻塞：发 12µs 触发脉冲后立刻返回，其余由硬件打点、
 *      中断算结果（分辨率 1µs ≈ 0.17mm），不会像旧版那样在循环里等 8ms。
 *   2) TIM1 周期是 10ms（与循迹 PID 共用），而回波最长约 66ms（超量程
 *      脉冲），所以要跨周期测量：每个 10ms 更新中断给正在 WAIT_FALL 的
 *      通道累加 wraps，宽度 = CCR降沿 − CCR升沿 + wraps×10000µs；
 *      结果为负再补一个周期（回波边沿撞上周期切换点时的修正）。
 *   3) 捕获中断优先级高于更新中断：边沿时间戳第一时间记录，PID 不会引入抖动。
 * ========================================================================== */

/* ---------------------------- 内部状态 ---------------------------- */

#define HCSR04_ST_IDLE      0U /* 空闲，可以触发下一次 */
#define HCSR04_ST_WAIT_RISE 1U /* 已触发，等 Echo 上升沿 */
#define HCSR04_ST_WAIT_FALL 2U /* 已捕获上升沿（极性已切下降沿），等下降沿 */

typedef struct
{
    uint32_t tim_channel;    /* 捕获通道：TIM_CHANNEL_3 / TIM_CHANNEL_4 */
    GPIO_TypeDef *trig_port; /* TRIG 引脚 */
    uint16_t trig_pin;

    volatile uint8_t state;      /* HCSR04_ST_x */
    volatile uint16_t t_rise;    /* 上升沿捕获值（µs） */
    volatile uint16_t wraps;     /* WAIT_FALL 期间经过的 10ms 周期数 */
    volatile uint8_t tick_cnt;   /* WAIT_RISE 期间经过的更新中断次数 */
    volatile uint32_t seq;       /* 每完成一次测量 +1，供阻塞读数判断新结果 */

    volatile int16_t last_cm;         /* 最近结果，-1 = 无目标 */
    volatile uint16_t last_width_us;  /* 最近回波宽度（调试） */
    volatile uint32_t last_result_ms; /* 结果时刻（陈旧保护用） */

    uint32_t last_trig_ms;            /* 上次触发时刻（最小间隔用） */
} Hcsr04Channel;

/* 探头映射表：ECHO 通道与 TRIG 引脚。
 * 如果左右探头接线与这里相反，把下面两行整体对调即可。 */
static Hcsr04Channel s_ch[HCSR04_ID_COUNT] = {
    [HCSR04_LEFT] = {
        .tim_channel = TIM_CHANNEL_3,
        .trig_port   = HCSR04_TRIG_LEFT_PORT,
        .trig_pin    = HCSR04_TRIG_LEFT_PIN,
    },
    [HCSR04_RIGHT] = {
        .tim_channel = TIM_CHANNEL_4,
        .trig_port   = HCSR04_TRIG_RIGHT_PORT,
        .trig_pin    = HCSR04_TRIG_RIGHT_PIN,
    },
};

static uint8_t s_started = 0U;

/* ---------------------------- 微秒延时（DWT） ---------------------------- */
/* 只用于给 TRIG 发 12µs 脉冲，不需要额外外设 */

static uint32_t s_cycles_per_us = 72U;

static void Hcsr04_TimeBaseInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    s_cycles_per_us = SystemCoreClock / 1000000U;
    if (s_cycles_per_us == 0U)
        s_cycles_per_us = 1U;
}

static void Hcsr04_DelayUs(uint32_t us)
{
    const uint32_t start = DWT->CYCCNT;
    const uint32_t wait = us * s_cycles_per_us;

    while ((DWT->CYCCNT - start) < wait)
    {
    }
}

/* ---------------------------- 结果处理 ---------------------------- */

/* 测量失败（等不到边沿/回波异常拉长）：复位状态并按无目标上报。
 * 注意：失败时极性可能还停在下降沿，必须切回上升沿，否则下一次测量
 * 捕获不到回波上升沿 */
static void Hcsr04_Fail(Hcsr04Channel *ch)
{
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, ch->tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);
    ch->state = HCSR04_ST_IDLE;
    ch->last_cm = -1;
    ch->last_width_us = 0U;
    ch->seq++;
    ch->last_result_ms = HAL_GetTick();
}

/* 把回波宽度（µs）换算成厘米并保存 */
static void Hcsr04_SaveWidth(Hcsr04Channel *ch, int32_t w_us)
{
    int16_t cm;

    if (w_us < 0)
        w_us = 0;
    else if (w_us > 0xFFFF)
        w_us = 0xFFFF;

    if (w_us < (int32_t)HCSR04_BLIND_ZONE_US)
    {
        /* 盲区：读数随机，按 2cm 上报，宁可误报不要漏报 */
        cm = 2;
    }
    else if (w_us > (int32_t)HCSR04_TIMEOUT_US)
    {
        /* 超过有效量程（含 66ms 超量程脉冲）：按无目标处理 */
        cm = -1;
    }
    else
    {
        /* 距离(cm) = 往返时间(µs) ÷ 58。
         * 手册按 340m/s 精确值是 58.8µs/cm，差约 1.5%，沿用 58 保持与旧数据一致 */
        cm = (int16_t)((w_us + 29) / 58);
    }

    /* 跳变过滤：周期切换点上的相位巧合可能让结果差一个 TIM1 周期（~172cm），
     * 与上次有效读数相差超过阈值就丢弃本次、保持旧值 */
    if ((cm > 0) && (ch->last_cm > 0))
    {
        int16_t d = (int16_t)(cm - ch->last_cm);

        if (d < 0)
            d = (int16_t)(-d);

        if (d > HCSR04_MAX_JUMP_CM)
        {
            ch->seq++; /* 本次已完成（虽然被丢弃），阻塞读数不用再等 */
            ch->last_result_ms = HAL_GetTick();
            return;
        }
    }

    ch->last_cm = cm;
    ch->last_width_us = (uint16_t)w_us;
    ch->seq++;
    ch->last_result_ms = HAL_GetTick();
}

/* ---------------------------- 初始化 ---------------------------- */

void HCSR04_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint8_t i;

    Hcsr04_TimeBaseInit();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* TRIG：推挽输出，默认拉低 */
    GPIO_InitStruct.Pin = HCSR04_TRIG_LEFT_PIN | HCSR04_TRIG_RIGHT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, HCSR04_TRIG_LEFT_PIN | HCSR04_TRIG_RIGHT_PIN, GPIO_PIN_RESET);

    /* ECHO：PA10/PA11 输入（TIM1 CH3/CH4 捕获脚）。
     * 下拉：探头没插时线是确定低电平，不会检到悬空噪声。
     * 注：PA10/PA11 都是 5V 容忍脚，ECHO 直连安全 */
    GPIO_InitStruct.Pin = HCSR04_ECHO_LEFT_PIN | HCSR04_ECHO_RIGHT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(HCSR04_ECHO_PORT, &GPIO_InitStruct);

    /* 状态复位 */
    for (i = 0; i < HCSR04_ID_COUNT; i++)
    {
        s_ch[i].state = HCSR04_ST_IDLE;
        s_ch[i].last_cm = -1;
        s_ch[i].last_width_us = 0U;
        s_ch[i].last_result_ms = 0U;
        s_ch[i].last_trig_ms = 0U;
    }
}

void HCSR04_Start(void)
{
    TIM_IC_InitTypeDef sConfigIC = {0};

    if (htim1.Instance != TIM1)
        return; /* TIM1 还没初始化（需要先调用 MX_TIM1_Init） */

    if (s_started)
        return; /* 已经启动过 */

    /* 通道配置：直连 TI3/TI4，从上升沿开始捕，无预分频；
     * 滤波器 0x4 = 连续 6 次采样一致才认，滤掉约 0.17µs 以内的毛刺 */
    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter    = 0x4U;

    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
        Error_Handler();

    /* 捕获中断优先级要高于 TIM1 更新中断（1）：边沿时间戳第一时间处理 */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

    if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4) != HAL_OK)
        Error_Handler();

    s_started = 1U;
}

/* ---------------------------- 触发与读取 ---------------------------- */

uint8_t HCSR04_Trigger(HCSR04_ID id)
{
    Hcsr04Channel *ch;

    if ((uint32_t)id >= (uint32_t)HCSR04_ID_COUNT)
        return 0U;

    if (!s_started)
    {
        if (htim1.Instance != TIM1)
            return 0U; /* TIM1 未初始化，无法测距 */
        HCSR04_Start(); /* 首次使用时自动补启动 */
    }

    ch = &s_ch[id];

    /* 同一路同一时间只允许一次测量在途；两路请轮流触发 */
    if (ch->state != HCSR04_ST_IDLE)
        return 0U;

    /* 同一路的最小触发间隔（首次触发不受限制） */
    if ((ch->last_trig_ms != 0U) &&
        ((int32_t)(HAL_GetTick() - ch->last_trig_ms) < (int32_t)HCSR04_INTERVAL_MS))
        return 0U;

    ch->last_trig_ms = HAL_GetTick();
    ch->wraps = 0U;
    ch->tick_cnt = 0U;
    ch->state = HCSR04_ST_WAIT_RISE;

    /* 触发：>10µs 高电平（手册要求），这里给 12µs */
    HAL_GPIO_WritePin(ch->trig_port, ch->trig_pin, GPIO_PIN_SET);
    Hcsr04_DelayUs(HCSR04_TRIG_US);
    HAL_GPIO_WritePin(ch->trig_port, ch->trig_pin, GPIO_PIN_RESET);

    return 1U;
}

int16_t HCSR04_LastCm(HCSR04_ID id)
{
    if ((uint32_t)id >= (uint32_t)HCSR04_ID_COUNT)
        return -1;

    /* 结果陈旧保护：太久没有新结果按无目标处理 */
    if ((HAL_GetTick() - s_ch[id].last_result_ms) > HCSR04_STALE_MS)
        return -1;

    return s_ch[id].last_cm;
}

uint8_t HCSR04_IsBusy(HCSR04_ID id)
{
    if ((uint32_t)id >= (uint32_t)HCSR04_ID_COUNT)
        return 1U;

    return (s_ch[id].state != HCSR04_ST_IDLE) ? 1U : 0U;
}

uint16_t HCSR04_LastWidthUs(HCSR04_ID id)
{
    if ((uint32_t)id >= (uint32_t)HCSR04_ID_COUNT)
        return 0U;

    return s_ch[id].last_width_us;
}

int16_t HCSR04_ReadCmTimeout(HCSR04_ID id, uint32_t timeout_ms)
{
    uint32_t seq0;
    uint32_t start;

    if ((uint32_t)id >= (uint32_t)HCSR04_ID_COUNT)
        return -1;

    seq0 = s_ch[id].seq;

    if (!HCSR04_Trigger(id))
    {
        /* 触发不了（正忙/间隔未到）：直接给出当前读数 */
        return HCSR04_LastCm(id);
    }

    start = HAL_GetTick();
    while (s_ch[id].seq == seq0)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
            return -1;
        HAL_Delay(1);
    }

    return HCSR04_LastCm(id);
}

int16_t HCSR04_ReadCm(HCSR04_ID id)
{
    return HCSR04_ReadCmTimeout(id, HCSR04_READ_TIMEOUT_MS);
}

int16_t HCSR04_ReadCmMin(HCSR04_ID id, uint8_t times)
{
    int16_t best = -1;
    int16_t cm;
    uint8_t i;

    if (times == 0U)
        return -1;

    for (i = 0; i < times; i++)
    {
        cm = HCSR04_ReadCm(id);
        if ((cm > 0) && ((best < 0) || (cm < best)))
            best = cm;

        if ((uint8_t)(i + 1U) < times)
            HAL_Delay(HCSR04_INTERVAL_MS);
    }

    return best;
}

/* ---------------------------- 中断钩子 ---------------------------- */

/* 每 10ms 调用一次（stm32f1xx_it.c 的 TIM1_UP_IRQHandler 里已接好）：
 * 给正在等待下降沿的通道累计周期数，并做超时检查 */
void HCSR04_OnUpdate(void)
{
    uint8_t i;

    for (i = 0; i < HCSR04_ID_COUNT; i++)
    {
        Hcsr04Channel *ch = &s_ch[i];

        if (ch->state == HCSR04_ST_WAIT_FALL)
        {
            /* 又经过一个 10ms 周期，回波仍为高 → 记一次溢出（66ms 脉冲约 6~7 次）。
             * 超过上限说明回波卡住或异常，复位防死等 */
            if (++ch->wraps >= HCSR04_MAX_WRAPS)
                Hcsr04_Fail(ch);
        }
        else if (ch->state == HCSR04_ST_WAIT_RISE)
        {
            /* 触发后一直没等到上升沿（探头掉线/没插）→ 判失败复位 */
            if (++ch->tick_cnt >= HCSR04_RISE_TIMEOUT_TICKS)
                Hcsr04_Fail(ch);
        }
    }
}

/* TIM1 捕获中断回调（TIM1_CC_IRQHandler → HAL_TIM_IRQHandler 分发到这里）。
 * 只做记录 + 切换极性 + 算宽度，计算量很小 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    Hcsr04Channel *ch = NULL;
    uint32_t chsel;
    uint8_t i;

    if (htim->Instance != TIM1)
        return;

    /* 定位本次捕获对应的通道。
     * 注意 htim->Channel 是 HAL_TIM_ACTIVE_CHANNEL_x，不能直接和 TIM_CHANNEL_x 比较 */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        chsel = TIM_CHANNEL_3;
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        chsel = TIM_CHANNEL_4;
    else
        return;

    for (i = 0; i < HCSR04_ID_COUNT; i++)
    {
        if (s_ch[i].tim_channel == chsel)
        {
            ch = &s_ch[i];
            break;
        }
    }

    if (ch == NULL)
        return;

    if (ch->state == HCSR04_ST_WAIT_RISE)
    {
        /* 上升沿：记录起始时刻，把捕获极性切成下降沿 */
        ch->t_rise = (uint16_t)HAL_TIM_ReadCapturedValue(htim, ch->tim_channel);
        ch->state = HCSR04_ST_WAIT_FALL;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, ch->tim_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else if (ch->state == HCSR04_ST_WAIT_FALL)
    {
        /* 下降沿：宽度 = 差值 + 溢出周期数。
         * 结果为负说明恰好有一个周期在更新中断里被"抢先"计数
         * （回波边沿撞上周期切换点），补回一个周期即可 */
        int32_t w = (int32_t)(uint16_t)HAL_TIM_ReadCapturedValue(htim, ch->tim_channel)
                    - (int32_t)ch->t_rise
                    + (int32_t)ch->wraps * (int32_t)HCSR04_TIM1_WRAP_US;

        __HAL_TIM_SET_CAPTUREPOLARITY(htim, ch->tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);
        ch->state = HCSR04_ST_IDLE;

        if (w < 0)
            w += (int32_t)HCSR04_TIM1_WRAP_US;

        Hcsr04_SaveWidth(ch, w);
    }
    /* IDLE 状态下的杂散沿（如探头刚插上）直接忽略 */
}
