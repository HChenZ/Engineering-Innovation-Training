#include "hcsr04.h"

/* 微秒计时用的每微秒周期数（72MHz 时等于 72） */
static uint32_t cycles_per_us = 72;

/* 最近一次测量结果 */
static int16_t last_cm[HCSR04_ID_COUNT] = {-1, -1, -1};

/* 超声波硬件映射结构体 */
typedef struct
{
    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;

    GPIO_TypeDef *echo_port;
    uint16_t echo_pin;
} HCSR04_PinConfig;

/* 全局超声波引脚映射表
 * 如果发现左右装反了，只需要交换下面两行的顺序即可 */
static const HCSR04_PinConfig hcsr04Config[] = {
    {/* HCSR04_LEFT  -> U1 插座：PA8 / PA9 */
     HC_Trig1_GPIO_Port, HC_Trig1_Pin,
     HC_Echo1_GPIO_Port, HC_Echo1_Pin},
    {/* HCSR04_CENTER -> U2 插座：PA10 / PA11 */
     HC_Trig2_GPIO_Port, HC_Trig2_Pin,
     HC_Echo2_GPIO_Port, HC_Echo2_Pin},
    {/* HCSR04_RIGHT -> U4 插座：PA4 / PA5 */
     HC_Trig3_GPIO_Port, HC_Trig3_Pin,
     HC_Echo3_GPIO_Port, HC_Echo3_Pin}};

/* TIM2/TIM3/TIM4 已经被电机和编码器占用，
 * 这里用 Cortex-M3 自带的 DWT 周期计数器做微秒计时，不需要额外外设。 */
static void HCSR04_TimeBaseInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0)
        cycles_per_us = 1;
}

/* 微秒级延时 */
static void HCSR04_DelayUs(uint32_t us)
{
    const uint32_t start = DWT->CYCCNT;
    const uint32_t wait = us * cycles_per_us;

    while ((DWT->CYCCNT - start) < wait)
    {
    }
}

void HCSR04_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint8_t i;

    HCSR04_TimeBaseInit();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* TRIG：推挽输出，初始拉低 */
    GPIO_InitStruct.Pin = HC_Trig1_Pin | HC_Trig2_Pin | HC_Trig3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, HC_Trig1_Pin | HC_Trig2_Pin | HC_Trig3_Pin, GPIO_PIN_RESET);

    /* ECHO：输入
     * 注意 PA9 是 5V 容忍脚，PA5 不是（TTa，只能 3.3V）。
     * U4 那路请让模块用 3.3V 供电，或在 ECHO 与 PA5 之间加分压。
     * PA11 也是 5V 容忍脚，U2 那路不受影响。 */
    GPIO_InitStruct.Pin = HC_Echo1_Pin | HC_Echo2_Pin | HC_Echo3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    for (i = 0; i < HCSR04_ID_COUNT; i++)
        last_cm[i] = -1;
}

int16_t HCSR04_ReadCm(HCSR04_ID id)
{
    const HCSR04_PinConfig *cfg;
    uint32_t start;
    uint32_t t_rise;
    uint32_t timeout;
    uint32_t width_us;
    int16_t cm;

    if (id >= HCSR04_ID_COUNT)
        return -1;

    cfg = &hcsr04Config[id];

    /* 1) 触发：给 >10us 的高电平 */
    HAL_GPIO_WritePin(cfg->trig_port, cfg->trig_pin, GPIO_PIN_SET);
    HCSR04_DelayUs(12);
    HAL_GPIO_WritePin(cfg->trig_port, cfg->trig_pin, GPIO_PIN_RESET);

    timeout = HCSR04_TIMEOUT_US * cycles_per_us;

    /* 2) 等回波上升沿
     * 注意：等上升沿和量脉宽共用同一个超时预算，
     * 这样单次测量的最坏阻塞时间才真的等于 HCSR04_TIMEOUT_US。 */
    start = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(cfg->echo_port, cfg->echo_pin) == GPIO_PIN_RESET)
    {
        if ((DWT->CYCCNT - start) > timeout)
        {
            last_cm[id] = -1;
            return -1;
        }
    }

    /* 3) 量高电平宽度（仍受同一个总超时约束） */
    t_rise = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(cfg->echo_port, cfg->echo_pin) == GPIO_PIN_SET)
    {
        if ((DWT->CYCCNT - start) > timeout)
        {
            last_cm[id] = -1;
            return -1;
        }
    }
    width_us = (DWT->CYCCNT - t_rise) / cycles_per_us;

    /* 4) 距离(cm) = 往返时间(us) / 58
     *    推导：距离 = 时间 × 声速 / 2，343m/s 时 1cm 对应约 58us 往返 */
    if (width_us < 116U)
    {
        /* 回波特别短：目标已经贴到盲区边缘了。
         * 这里当作 2cm 上报，宁可误报也不要漏报。 */
        cm = 2;
    }
    else
    {
        cm = (int16_t)((width_us + 29U) / 58U);
        if (cm > (int16_t)(HCSR04_TIMEOUT_US / 58U))
            cm = -1;
    }

    last_cm[id] = cm;
    return cm;
}

int16_t HCSR04_ReadCmMin(HCSR04_ID id, uint8_t times)
{
    int16_t best = -1;
    int16_t cm;
    uint8_t i;

    if (times == 0)
        return -1;

    for (i = 0; i < times; i++)
    {
        cm = HCSR04_ReadCm(id);
        if (cm > 0 && (best < 0 || cm < best))
            best = cm;

        if (i + 1 < times)
            HAL_Delay(HCSR04_INTERVAL_MS);
    }

    return best;
}

int16_t HCSR04_LastCm(HCSR04_ID id)
{
    if (id >= HCSR04_ID_COUNT)
        return -1;

    return last_cm[id];
}
