#ifndef TRACK_H
#define TRACK_H

// 循迹模块：多路数字灰度传感器，经 CD4051 模拟多路开关分时选通
//
// 引脚（已由 CubeMX 配好，见 main.h 里的宏定义）：
//   PB14 = Track_AD0  -> CD4051 地址位 A（bit0，最低位）
//   PB13 = Track_AD1  -> CD4051 地址位 B（bit1）
//   PB12 = Track_AD2  -> CD4051 地址位 C（bit2，最高位）
//   PB15 = Track_Out  -> CD4051 公共输出端，接 STM32 数字输入
//
// 工作原理：
//   3 根地址线选中 8 路传感器中的一路，该路信号从 Out 输出。
//   单片机先写地址、再读 Out，即可分时读回 8 路状态——4 根 IO 顶 8 路。
//
// 方向约定（面向车头看）：
//   通道 0 在最左，通道 N-1 在最右。
//   Track_GetPosition() > 0 表示线偏右（需右转），< 0 表示线偏左（需左转）。
//
// 用法：
//   setup 里调用 Track_Init()
//   主循环里用 Track_GetPosition() 拿连续偏差做差速转向（PID / 比例），
//   或改用 Track_Update() 拿离散建议动作（和 avoid 模块一个风格）

#include "main.h"

/* ==================== 通道配置 ==================== */

/* 实际使用的传感器路数（2 ~ 8，对应 CD4051 的 8 个通道）
 * 只装了 5 路就填 5，位置计算会自动按 5 路归一化 */
#define TRACK_CHANNEL_COUNT 8

#if TRACK_CHANNEL_COUNT < 2
#error "TRACK_CHANNEL_COUNT 至少为 2"
#endif

/* 压线时 CD4051 Out 脚的有效电平，按传感器模块极性二选一：
 *   低电平有效型模块填 0，高电平有效型模块（压线灯亮=高电平）填 1。
 * 已按实车确认：压线灯灭 = 0，没压线灯亮 = 1（日常常亮），故填 0 */
#define TRACK_ACTIVE_LEVEL 0

/* 位置方向反转：
 *   0 = 正常（通道 0 在车头最左，通道 N-1 在最右）
 *   1 = 传感器左右反装，偏差取反
 * 现象同样是"线在左车往右跑"：若改 TRACK_ACTIVE_LEVEL 无效，改这里 */
#define TRACK_INVERT 0

/* 位置偏差满量程：Track_GetPosition() 返回 -TRACK_POS_MAX ~ +TRACK_POS_MAX */
#define TRACK_POS_MAX 1000

/* ==================== 循迹建议动作 ==================== */

typedef enum
{
    TRACK_CENTER = 0, /* 线居中或接近居中，直行微调 */
    TRACK_LEFT,       /* 线偏左，需左转 */
    TRACK_RIGHT,      /* 线偏右，需右转 */
    TRACK_LOST        /* 丢线：所有通道都没压线 */
} TrackState;

/* ==================== 函数 ==================== */

/* 初始化（GPIO 已由 MX_GPIO_Init 配好，这里只清内部状态并选通 0 号通道） */
void Track_Init(void);

/* 读单个通道（0 ~ TRACK_CHANNEL_COUNT-1），返回 1=压线 / 0=没压线 */
uint8_t Track_Read(uint8_t channel);

/* 读全部通道，返回位图：bit0=通道0，bitN=1 表示该路压线 */
uint8_t Track_ReadAll(void);

/* 压线通道数（0 ~ TRACK_CHANNEL_COUNT） */
uint8_t Track_CountActive(void);

/* 位置偏差：-TRACK_POS_MAX ~ +TRACK_POS_MAX，0 表示居中
 *   正 = 线偏右（需右转），负 = 线偏左（需左转）
 * 丢线时返回"记忆中的最后一个有效值"，配合 Track_IsLost() 判断 */
int16_t Track_GetPosition(void);

/* 是否丢线（所有通道都没压线） */
uint8_t Track_IsLost(void);

/* 每个主循环调用一次：刷新采样并返回离散建议动作。
 * 只想拿连续偏差自己做 PID，就只用 Track_GetPosition()，不必管这个 */
TrackState Track_Update(void);

#endif
