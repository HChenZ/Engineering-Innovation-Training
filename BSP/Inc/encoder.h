#ifndef ENCODER_H
#define ENCODER_H

// PB6 - LEFT_OUTA
// PB7 - LEFT_OUTB
// PA6 - RIGHT_OUTA
// PA7 - RIGHT_OUTB

#include "main.h"
#include "tim.h"

/* ==================== 编码器参数 ==================== */
#define ENCODER_PPR 13             /* 编码器线数（每转脉冲数） */
#define ENCODER_GEAR_RATIO 20      /* 减速比 1:20 */
#define ENCODER_MULTIPLIER 4       /* TI1 and TI2 模式为4倍频 */

/* 输出轴每转一圈的计数值 = 13 × 4 × 20 = 1040 */
#define ENCODER_CPR (ENCODER_PPR * ENCODER_MULTIPLIER * ENCODER_GEAR_RATIO)

/* ==================== 方向定义 ==================== */
#define ENCODER_DIR_CW 0           /* 顺时针 / 正转 */
#define ENCODER_DIR_CCW 1          /* 逆时针 / 反转 */

/* ==================== 编码器句柄结构体 ==================== */
typedef struct {
    TIM_HandleTypeDef *htim;        /* 定时器句柄指针 */
    int32_t last_count;             /* 上次采样时的计数值 */
    int32_t total_count;            /* 累计脉冲数（带溢出处理） */
    int16_t speed;                  /* 转速（单位：脉冲/采样周期） */
    uint8_t direction;              /* 方向：0=正转，1=反转 */
} Encoder_t;

/* ==================== 外部变量 ==================== */
extern Encoder_t encoder_left;      /* 左轮编码器（TIM4, PB6/PB7） */
extern Encoder_t encoder_right;     /* 右轮编码器（TIM3, PA6/PA7） */

/* ==================== 函数声明 ==================== */
void Encoder_Init(void);
int32_t Encoder_GetCount(Encoder_t *enc);
int16_t Encoder_GetSpeed(Encoder_t *enc);
uint8_t Encoder_GetDirection(Encoder_t *enc);
void Encoder_Reset(Encoder_t *enc);
float Encoder_GetRevolutions(Encoder_t *enc);

#endif
