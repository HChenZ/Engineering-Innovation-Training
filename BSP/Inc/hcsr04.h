#ifndef HCSR04_H
#define HCSR04_H

// 超声波模块：HC-SR04（2020款，宽电压 3-5.5V，默认 GPIO 模式）
// 底板预留 3 路插座，驱动全部支持，用哪几路由 avoid.c 决定：
//   U1（车头右侧）: TRIG = PA8    ECHO = PA9
//   U2（车头正中）: TRIG = PA10   ECHO = PA11
//   U4（车头左侧）: TRIG = PA4    ECHO = PA5
//
// 模块参数（厂家手册）
//   供电 3-5.5V   电流 2.2mA   量程 2-450cm   盲区 2-4cm
//   精度 ±2%      分辨率 1mm   波束角 ±15°(典型) / ±20°(最大)
//   触发：TRIG 给 >10us 高电平，ECHO 回一个等比例高电平脉冲
//   距离 = 往返时间 × 声速 / 2
//
// 引脚定义说明：这里自带一份，用 #ifndef 保护。
// 如果以后在 CubeMX 里正式配置了这几个脚，main.h 里的定义会自动生效。

#include "main.h"

/* 超声波编号 */
typedef enum
{
    HCSR04_LEFT = 0, /* U4：车头左侧，探头向外斜 */
    HCSR04_CENTER,   /* U2：车头正中（开关见 avoid.h 的 AVOID_USE_CENTER） */
    HCSR04_RIGHT,    /* U1：车头右侧，探头向外斜 */
    HCSR04_ID_COUNT
} HCSR04_ID;

/* ---------------- 引脚定义 ---------------- */
#ifndef HC_Trig1_Pin
#define HC_Trig1_Pin GPIO_PIN_8
#define HC_Trig1_GPIO_Port GPIOA
#endif

#ifndef HC_Echo1_Pin
#define HC_Echo1_Pin GPIO_PIN_9
#define HC_Echo1_GPIO_Port GPIOA
#endif

#ifndef HC_Trig2_Pin
#define HC_Trig2_Pin GPIO_PIN_10
#define HC_Trig2_GPIO_Port GPIOA
#endif

#ifndef HC_Echo2_Pin
#define HC_Echo2_Pin GPIO_PIN_11
#define HC_Echo2_GPIO_Port GPIOA
#endif

#ifndef HC_Trig3_Pin
#define HC_Trig3_Pin GPIO_PIN_4
#define HC_Trig3_GPIO_Port GPIOA
#endif

#ifndef HC_Echo3_Pin
#define HC_Echo3_Pin GPIO_PIN_5
#define HC_Echo3_GPIO_Port GPIOA
#endif

/* ---------------- 参数 ---------------- */

/* 单次测距超时（微秒）。8000us ≈ 1.37m，
 * 意思是超过 1.37m 就当作"前方无障碍"。
 * 等回波上升沿和量脉宽共用这一个预算，所以单次测量最坏就阻塞这么久。 */
#define HCSR04_TIMEOUT_US 8000

/* 两次测量之间的最小间隔（毫秒）。
 * 手册建议 >=200ms 以保证远距离精度，厂家示例程序用 30+100ms，
 * 避障场景取 60ms 兼顾响应速度。 */
#define HCSR04_INTERVAL_MS 60

/* ---------------- 函数 ---------------- */

/* 初始化（配置 GPIO、启动微秒计时） */
void HCSR04_Init(void);

/* 单次测距：返回厘米；-1 表示无回波（超出量程或落在盲区）
 * 会阻塞，最坏 HCSR04_TIMEOUT_US 微秒（默认 8ms） */
int16_t HCSR04_ReadCm(HCSR04_ID id);

/* 连续测量 times 次，返回其中的最小值（只统计有效值）
 * 两次之间间隔 HCSR04_INTERVAL_MS，注意会阻塞 */
int16_t HCSR04_ReadCmMin(HCSR04_ID id, uint8_t times);

/* 取最近一次测量结果，不触发新的测量（用于显示/调试） */
int16_t HCSR04_LastCm(HCSR04_ID id);

#endif
