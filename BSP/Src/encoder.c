#include "encoder.h"

/* 左轮：TIM4 (PB6=CH1, PB7=CH2) */
Encoder_t encoder_left = {
    .htim = &htim4,
    .last_count = 0,
    .total_count = 0,
    .speed = 0,
    .direction = ENCODER_DIR_CW
};

/* 右轮：TIM3 (PA6=CH1, PA7=CH2) */
Encoder_t encoder_right = {
    .htim = &htim3,
    .last_count = 0,
    .total_count = 0,
    .speed = 0,
    .direction = ENCODER_DIR_CW
};

/* ==================== 初始化 ==================== */
void Encoder_Init(void)
{
    /* 启动 TIM3 编码器接口（右轮） */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    /* 启动 TIM4 编码器接口（左轮） */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    /* 清零状态 */
    encoder_left.last_count = 0;
    encoder_left.total_count = 0;
    encoder_right.last_count = 0;
    encoder_right.total_count = 0;
}

/* ==================== 读取当前计数值 ==================== */
int32_t Encoder_GetCount(Encoder_t *enc)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(enc->htim);
}

/* ==================== 读取方向 ==================== */
uint8_t Encoder_GetDirection(Encoder_t *enc)
{
    /* __HAL_TIM_IS_TIM_COUNTING_DOWN 返回 1 表示向下计数（反转） */
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(enc->htim))
        enc->direction = ENCODER_DIR_CCW;
    else
        enc->direction = ENCODER_DIR_CW;

    return enc->direction;
}

/* ==================== 读取速度（脉冲增量法） ==================== */
/*
 * 返回值：本次采样周期内输出轴转过的脉冲增量（带符号）。
 * 正数表示正转，负数表示反转。
 * 速度单位换算：转速(RPM) = speed / ENCODER_CPR * (60 / 采样周期秒)
 * 例如 10ms 采样：RPM = speed / 1040 * 6000
 */
int16_t Encoder_GetSpeed(Encoder_t *enc)
{
    int32_t current = (int32_t)__HAL_TIM_GET_COUNTER(enc->htim);

    /* 16位计数器环形差值处理，自动处理 0 <-> 65535 溢出 */
    int32_t diff = current - enc->last_count;

    /* 将差值转换为有符号16位范围 */
    if (diff > 32767)
        diff -= 65536;
    else if (diff < -32768)
        diff += 65536;

    enc->last_count = current;
    enc->speed = (int16_t)diff;

    /* 同时更新方向和累计计数 */
    if (diff > 0)
        enc->direction = ENCODER_DIR_CW;
    else if (diff < 0)
        enc->direction = ENCODER_DIR_CCW;

    enc->total_count += diff;

    return enc->speed;
}

/* ==================== 重置编码器 ==================== */
void Encoder_Reset(Encoder_t *enc)
{
    __HAL_TIM_SET_COUNTER(enc->htim, 0);
    enc->last_count = 0;
    enc->total_count = 0;
    enc->speed = 0;
    enc->direction = ENCODER_DIR_CW;
}

/* ==================== 获取输出轴转过的圈数 ==================== */
float Encoder_GetRevolutions(Encoder_t *enc)
{
    return (float)enc->total_count / (float)ENCODER_CPR;
}
