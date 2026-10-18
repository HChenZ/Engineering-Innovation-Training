#ifndef TRACKING_H
#define TRACKING_H

#include "main.h"

/*
 * 灰度位置：负值表示黑线偏左，正值表示黑线偏右。
 * bitmap 为 0 时位置返回 0，此时必须通过 bitmap 区分“丢线”和“居中”。
 */
int16_t Track_GetPositionFromBitmap(uint8_t bitmap);
int16_t Track_GetPosition(void);

/* 返回 bitmap 中最长的一段连续黑点数量。 */
uint8_t Track_CountConsecutiveBlackFromBitmap(uint8_t bitmap);
uint8_t Track_CountConsecutiveBlack(void);

#endif
