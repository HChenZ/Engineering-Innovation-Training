#include "track.h"

/* 上一次有效的偏差值（丢线时保持，用来"按记忆方向继续走"） */
static int16_t last_position = 0;

/* 丢线标记 */
static uint8_t lost = 0;

/* ==================== 大角度切入检测（避障后垂直入线判据）==================== */
/* 检测逻辑：避障结束后，车头与线接近垂直（90°），快速横扫过线。
 * 场景：避障回正后，车以很大的角度（接近90°）切入线，传感器快速从一侧扫到另一侧。
 *
 * 判据：
 * 1. 传感器快速从一侧切换到另一侧（左→右 或 右→左）
 * 2. 偏差快速反向且幅度大（从-800跳到+800，或反之）
 * 3. 短时间内（100ms）完成这个过程
 */
#define LARGE_ANGLE_DETECT_WINDOW_MS  100   /* 检测时间窗口100ms */
#define LARGE_ANGLE_FLIP_THRESH       600   /* 偏差反向阈值（从一侧跳到另一侧） */

static int16_t  prev_pos_for_flip = 0;      /* 上一拍的偏差 */
static uint32_t flip_detect_tick = 0;       /* 检测到反向的时刻 */
static uint8_t  large_angle_detected = 0;   /* 是否已检测到 */
static uint8_t  flip_detected = 0;          /* 是否检测到偏差快速反向 */

/* 位置权重表（左右对称 8 路）
 * 中间两路权重 ±1，越往外跳得越快：±3、±6、±8。
 * 这样"中间钝、边上陡"：直线时小偏差不敏感（不容易抖），
 * 大偏差（快丢线、正在回正）时转向更果断。
 * ⚠ 表里最大的那个数（8）必须和下面 Track_GetPosition() 里归一化的除数一致，
 *   否则所有偏差值会被整体放大或缩小（等于顺手改了转向力度）。
 * 注意：只针对 8 路，若改 TRACK_CHANNEL_COUNT 需同步调整。 */
/* 通道屏蔽掩码：bit n = 1 表示第 (n+1) 路启用。
 * 0x7E = 第 1 路和第 8 路【屏蔽】—— 它们是最外侧、权重最大的两路，
 * 灵敏度调不好时误报会造成满量程假偏差（±1000）。
 * 想恢复：把掩码改回 0xFF。 */
#define TRACK_CHANNEL_MASK 0xFF   /* 0xFF = 全部启用（已回退屏蔽）；
                                  * 之前用 0x7E 临时屏蔽第 1、8 路，现在取消。 */

static const int8_t track_weight[8] = {-8, -6, -3, -1, 1, 3, 6, 8};

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

    /* 大角度切入检测初始化 */
    prev_pos_for_flip = 0;
    flip_detect_tick = 0;
    large_angle_detected = 0;
    flip_detected = 0;

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
        if (!(TRACK_CHANNEL_MASK & (1u << ch)))
            continue;                /* 被屏蔽的通道：永远当作"没压线" */
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
        /* 丢线：往反方向转，往回找线。
         * 之前线在右（last_position>0）说明右转冲过了线，现在左转往回扫。
         * 注意：这只对"转向过冲"丢线有效；若是"急弯线在前方"丢线，
         * 反方向会找反，届时改回记忆方向或加限时停车。 */
        lost = 1;
        /* ⚠ 方向必须和位置环一致：pos > 0 = 线偏右、需右转。
         *   last_position > 0 说明脱线前线在车头右侧、车没转够、
         *   线从右边滑出去了 —— 要【继续往右】猛转把它钩回来（返回正值）。
         *   （原来返回的是负值，正好朝反方向打，把车推向赛道外侧。） */
        if (last_position > 0)
            return (int16_t)TRACK_LOST_STEER;
        if (last_position < 0)
            return (int16_t)(-TRACK_LOST_STEER);
        /* ⚠ 这里原来是 return 0，是个坑：如果丢线前最后一个有效偏差正好是 0
         * （车头斜着压线、线刚好落在传感器排中间），位置环拿到的就是
         * "完全居中"的假象 -> 转向算出来是 0 -> 车笔直往前开，再也不回头找线。
         * 实测现象就是：丢线之后 SL≈SR、PL≈PR，车直着冲出赛道。
         * 改成固定朝一个方向扫（车会画小圈往回找），总比直着开出去强。 */
        return (int16_t)(-TRACK_LOST_STEER);
    }

    /* 加权平均求"压线质心"：
     * 用 track_weight 表（官方 1,2,4,5）加权，再归一化到 ±TRACK_POS_MAX。
     * 例：8 路时通道 0 单独压线 -> -1000，通道 7 -> +1000，
     *     通道 3 -> -200，通道 4 -> +200（中间分辨率比线性权重更高）。 */
    for (i = 0; i < TRACK_CHANNEL_COUNT; i++)
    {
        if (bitmap & (1u << i))
        {
            sum_w += track_weight[i];
            sum_a++;
        }
    }

    lost = 0;

    /* bitmap != 0 已保证 sum_a > 0，这里只是防御 */
    if (sum_a == 0)
        return last_position;

    pos = (int16_t)((sum_w * TRACK_POS_MAX) / (8 * sum_a));  /* 8 = 最边缘权重，单路压边 -> 满偏 */

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

uint8_t Track_AllBlack(void)
{
    uint8_t bitmap = Track_ReadAll();

    /* 8 路全压线 -> bitmap 全 1 */
    /* 只比【启用】的通道是否全压线（被屏蔽的不参与） */
    return (bitmap == (uint8_t)(TRACK_CHANNEL_MASK & ((1u << TRACK_CHANNEL_COUNT) - 1))) ? 1 : 0;
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

/* 大角度切入检测：避障后垂直入线，偏差快速反向 */
uint8_t Track_DetectFastSweep(void)
{
    int16_t pos = last_position;  /* 当前偏差 */
    uint32_t now = HAL_GetTick();

    /* 如果已经检测到，保持标志直到Reset */
    if (large_angle_detected)
        return 1;

    /* 检测偏差快速反向（从一侧跳到另一侧）
     * 场景：车以90°角切入线，传感器从左侧快速扫到右侧（或反之）
     * 偏差会从 -800 快速变到 +800，或从 +800 快速变到 -800 */

    /* 检测到偏差反向且幅度够大 */
    if (!flip_detected)
    {
        /* 从左侧（负值）跳到右侧（正值） */
        if (prev_pos_for_flip < -LARGE_ANGLE_FLIP_THRESH &&
            pos > LARGE_ANGLE_FLIP_THRESH)
        {
            flip_detected = 1;
            flip_detect_tick = now;
        }
        /* 从右侧（正值）跳到左侧（负值） */
        else if (prev_pos_for_flip > LARGE_ANGLE_FLIP_THRESH &&
                 pos < -LARGE_ANGLE_FLIP_THRESH)
        {
            flip_detected = 1;
            flip_detect_tick = now;
        }
    }

    /* 如果检测到反向，并且在时间窗口内（说明反向很快） */
    if (flip_detected)
    {
        if ((now - flip_detect_tick) <= LARGE_ANGLE_DETECT_WINDOW_MS)
        {
            /* 确认是大角度切入 */
            large_angle_detected = 1;
            return 1;
        }
        else
        {
            /* 超时，可能是误判，重置 */
            flip_detected = 0;
        }
    }

    /* 更新上一拍偏差 */
    prev_pos_for_flip = pos;

    return 0;
}

/* 重置大角度切入检测器 */
void Track_ResetFastSweep(void)
{
    prev_pos_for_flip = 0;
    flip_detect_tick = 0;
    large_angle_detected = 0;
    flip_detected = 0;
}
