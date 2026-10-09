#include "track.h"

/* 上一次有效的偏差值（丢线时保持，用来"按记忆方向继续走"） */
static int16_t last_position = 0;

/* 丢线标记 */
static uint8_t lost = 0;

/* CD4051 地址建立延时：写完地址后输出要几百 ns 才稳定，
 * 72MHz 下这段空循环约 1us，足够覆盖。 */
static void Track_SettleDelay(void)
{
    volatile uint32_t i;
    for (i = 0; i < 50; i++)
    {
        __NOP();
    }
}

/* 选通一个通道（写 3 根地址线 A/B/C） */
static void Track_SelectChannel(uint8_t channel)
{
    HAL_GPIO_WritePin(Track_AD0_GPIO_Port, Track_AD0_Pin,
                      (channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Track_AD1_GPIO_Port, Track_AD1_Pin,
                      (channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Track_AD2_GPIO_Port, Track_AD2_Pin,
                      (channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    Track_SettleDelay();
}

void Track_Init(void)
{
    last_position = 0;
    lost = 0;

    /* 地址线初始选通 0 号通道 */
    Track_SelectChannel(0);
}

uint8_t Track_Read(uint8_t channel)
{
    GPIO_PinState out;

    if (channel >= TRACK_CHANNEL_COUNT)
        return 0;

    Track_SelectChannel(channel);
    out = HAL_GPIO_ReadPin(Track_Out_GPIO_Port, Track_Out_Pin);

    /* 把"压线"统一成 1：Out 上的电平等于有效电平时视为压线 */
    return (out == (GPIO_PinState)TRACK_ACTIVE_LEVEL) ? 1 : 0;
}

uint8_t Track_ReadAll(void)
{
    uint8_t bitmap = 0;
    uint8_t ch;

    for (ch = 0; ch < TRACK_CHANNEL_COUNT; ch++)
    {
        if (Track_Read(ch))
            bitmap |= (uint8_t)(1u << ch);
    }

    return bitmap;
}

uint8_t Track_CountActive(void)
{
    uint8_t bitmap = Track_ReadAll();
    uint8_t count = 0;
    uint8_t i;

    for (i = 0; i < TRACK_CHANNEL_COUNT; i++)
    {
        if (bitmap & (1u << i))
            count++;
    }

    return count;
}

int16_t Track_GetPosition(void)
{
    uint8_t bitmap = Track_ReadAll();
    int32_t sum_w = 0;
    int32_t sum_a = 0;
    int16_t pos;
    uint8_t i;

    if (bitmap == 0)
    {
        /* 丢线：保持上一次偏差，让小车按记忆方向继续走，直到重新压线 */
        lost = 1;
        return last_position;
    }

    /* 加权平均求"压线质心"：
     * 通道 i 的权重从 -(N-1) 到 +(N-1)（最左到最右），
     * 质心 = sum(weight_i) / sum(active_i)，再归一化到 ±TRACK_POS_MAX。
     * 例：8 路时通道 0 单独压线 -> -1000，通道 7 -> +1000，通道 3/4 之间 -> 0。 */
    for (i = 0; i < TRACK_CHANNEL_COUNT; i++)
    {
        if (bitmap & (1u << i))
        {
            int32_t w = (int32_t)i * 2 - (TRACK_CHANNEL_COUNT - 1);
            sum_w += w;
            sum_a++;
        }
    }

    lost = 0;

    /* bitmap != 0 已保证 sum_a > 0，这里只是防御 */
    if (sum_a == 0)
        return last_position;

    pos = (int16_t)((sum_w * TRACK_POS_MAX) / ((TRACK_CHANNEL_COUNT - 1) * sum_a));

#if TRACK_INVERT
    pos = (int16_t)(-pos);
#endif

    last_position = pos;
    return pos;
}

uint8_t Track_IsLost(void)
{
    return lost;
}

TrackState Track_Update(void)
{
    int16_t pos = Track_GetPosition();

    if (Track_IsLost())
        return TRACK_LOST;

    /* 死区：偏差小于满量程的 1/4 就当居中，避免在小偏差下频繁左右摆 */
    if (pos > TRACK_POS_MAX / 4)
        return TRACK_RIGHT;
    if (pos < -TRACK_POS_MAX / 4)
        return TRACK_LEFT;

    return TRACK_CENTER;
}
