#ifndef HCSR04_H
#define HCSR04_H

// 超声波测距驱动：HC-SR04（CS100A 芯片，宽电压 3-5.5V 版）
//
// 测距方式：TIM1 输入捕获（CH3 / CH4）测量 Echo 高电平宽度，非阻塞。
//   左探头：TRIG = PA4   ECHO = PA10（TIM1_CH3）
//   右探头：TRIG = PA8   ECHO = PA11（TIM1_CH4）
// ECHO 必须接 PA10/PA11（只有这两个脚连在 TIM1 捕获通道上），TRIG 可改。
// 若左右探头接反，交换 hcsr04.c 里 s_ch 表中两行即可（见该处注释）。
//
// 模块原理（厂家手册）：
//   TRIG 给 >10µs 高电平 → 模块发 8 个 40kHz 脉冲 → ECHO 输出高电平，
//   宽度 = 超声波往返时间；距离 = 宽度 × 340m/s ÷ 2（≈58.8µs/cm）。
//   超出量程时 ECHO 仍会输出一个约 66ms 的高电平脉冲，按"无目标"处理。
//
// 与循迹 PID 共用 TIM1 的说明：
//   TIM1 原本承担 10ms 周期的 PID 控制中断（main.c），
//   tim.c 里已把计数频率改为 1MHz（PSC=71）、周期保持 10ms（ARR=9999），
//   并利用每个 10ms 更新中断给宽回波做溢出计数，两者互不影响。
//
// 用法：
//   MX_TIM1_Init();  HAL_TIM_Base_Start_IT(&htim1);   // main.c 里已有
//   HCSR04_Init();                  // 任意时机（不依赖 TIM1）
//   HCSR04_Start();                 // 在 MX_TIM1_Init() 之后调用一次
//                                   // （没调也没关系，第一次 Trigger 会自动补）
//   // 循环里轮流触发两路，不要同时触发（避免左右探头互相串扰）：
//   HCSR04_Trigger(HCSR04_LEFT);   /   HCSR04_Trigger(HCSR04_RIGHT);
//   // 取最近结果（-1 = 无目标/无效，不触发新测量）：
//   int16_t d = HCSR04_LastCm(HCSR04_LEFT);
//
// 调试：HCSR04_ReadCm() 是阻塞版（等本次测完再返回，简单测试用）；
//       HCSR04_LastWidthUs() 可看原始回波宽度（µs）。

#include "main.h"

/* 超声波编号 */
typedef enum
{
    HCSR04_LEFT = 0, /* 左探头：TRIG=PA4， ECHO=PA10（TIM1_CH3） */
    HCSR04_RIGHT,    /* 右探头：TRIG=PA8， ECHO=PA11（TIM1_CH4） */
    HCSR04_ID_COUNT
} HCSR04_ID;

/* ---------------- 引脚定义（TRIG 可改；ECHO 固定在 PA10/PA11） ---------------- */
#define HCSR04_TRIG_LEFT_PORT   GPIOA   /* PA4，沿用底板 U4 插座的 TRIG */
#define HCSR04_TRIG_LEFT_PIN    GPIO_PIN_4
#define HCSR04_TRIG_RIGHT_PORT  GPIOA   /* PA8，沿用底板 U1 插座的 TRIG */
#define HCSR04_TRIG_RIGHT_PIN   GPIO_PIN_8

#define HCSR04_ECHO_PORT        GPIOA
#define HCSR04_ECHO_LEFT_PIN    GPIO_PIN_10   /* PA10 = TIM1_CH3，不可改 */
#define HCSR04_ECHO_RIGHT_PIN   GPIO_PIN_11   /* PA11 = TIM1_CH4，不可改 */

/* ---------------- 参数 ---------------- */

/* TRIG 触发脉冲宽度（µs）。手册要求 >10µs，12µs 留余量 */
#define HCSR04_TRIG_US 12U

/* 有效量程上限（µs）：回波宽于此时按"无目标"(-1)上报。
 * 8000µs ≈ 1.37m。捕获方式下这个值只影响判决，不产生阻塞；
 * 想测更远（最大约 4~6m）可以直接调大，比如 30000。 */
#define HCSR04_TIMEOUT_US 8000U

/* 盲区（µs）：回波短于 116µs（约 2cm）时按 2cm 上报。
 * 盲区内的读数是随机值，若当成"没测到"会把贴脸障碍判成通畅，
 * 所以宁可误报为 2cm。 */
#define HCSR04_BLIND_ZONE_US 116U

/* 同一探头两次触发的最小间隔（ms）。
 * 手册：最坏情况（超量程 66ms 脉冲）的测量周期约 66ms，取 60ms 做下限保护。 */
#define HCSR04_INTERVAL_MS 60U

/* 结果陈旧保护（ms）：超过这么久没有新的测量结果，
 * HCSR04_LastCm 返回 -1，防止探头掉线时一直沿用旧数据。 */
#define HCSR04_STALE_MS 300U

/* 跳变过滤阈值（cm）：TIM1 周期为 10ms，回波边沿恰好落在周期切换点上时，
 * 极端相位下结果可能差一个周期（约 172cm）。与上次有效读数相差超过
 * 该阈值就丢弃本次（有效量程内正常变化不可能超过 137cm）。 */
#define HCSR04_MAX_JUMP_CM 150

/* 阻塞读数接口的最长等待（ms）：正常测量（含 66ms 超量程脉冲）
 * 都能在此时限内完成 */
#define HCSR04_READ_TIMEOUT_MS 80U

/* 等 Echo 上升沿的超时（以 10ms 更新中断计）。
 * TRIG 后回波上升沿通常几百 µs 内到达，2 个周期（20ms）还没来按无效处理 */
#define HCSR04_RISE_TIMEOUT_TICKS 2U

/* 等 Echo 下降沿的下限超时（以 10ms 更新中断计）。
 * 66ms 超量程脉冲约跨 7 个周期，超过 8 个（80ms）还没有下降沿就按异常复位 */
#define HCSR04_MAX_WRAPS 8U

/* TIM1 计数周期（µs）：ARR+1 = 10000 个 1µs tick */
#define HCSR04_TIM1_WRAP_US 10000U

/* ---------------- 函数 ---------------- */

/* 初始化 GPIO（TRIG 输出、ECHO 输入）和内部状态；不碰 TIM1，随时可调用 */
void HCSR04_Init(void);

/* 配置并使能 TIM1 CH3/CH4 输入捕获（含 NVIC）。
 * 必须在 MX_TIM1_Init() 之后调用；重复调用只在第一次生效。 */
void HCSR04_Start(void);

/* 非阻塞触发一路测距。
 * 返回 1 = 已发出触发脉冲；0 = 该路正忙或未到最小间隔（本次未触发）。
 * 注意：两路请轮流触发，避免两个超声波互相串扰。 */
uint8_t HCSR04_Trigger(HCSR04_ID id);

/* 该路是否正在测量中（非空闲即算忙）。
 * 上层轮流触发两路时用它检查另一路是否空闲，等测完再触发下一路 */
uint8_t HCSR04_IsBusy(HCSR04_ID id);

/* 最近一次测距结果（cm）。-1 = 无目标/测量失败/结果过期。不触发新测量 */
int16_t HCSR04_LastCm(HCSR04_ID id);

/* 最近一次回波宽度（µs），调试/标定用；0 = 尚无有效数据 */
uint16_t HCSR04_LastWidthUs(HCSR04_ID id);

/* 阻塞式单次测距（简单测试用）：触发并在时限内等待完成，返回厘米。
 * 不要放进高频循环，会阻塞主循环 */
int16_t HCSR04_ReadCm(HCSR04_ID id);
int16_t HCSR04_ReadCmTimeout(HCSR04_ID id, uint32_t timeout_ms);

/* 连续测量 times 次取最小值（只统计有效值），两次之间间隔
 * HCSR04_INTERVAL_MS；会阻塞。没有任何有效值时返回 -1 */
int16_t HCSR04_ReadCmMin(HCSR04_ID id, uint8_t times);

/* TIM1 更新中断（每 10ms 一次）钩子：回波溢出计数与超时处理。
 * 已在 stm32f1xx_it.c 的 TIM1_UP_IRQHandler 里调用，不要重复调用 */
void HCSR04_OnUpdate(void);

#endif
