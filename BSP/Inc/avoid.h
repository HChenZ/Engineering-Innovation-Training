#ifndef AVOID_H
#define AVOID_H

// 超声波避障逻辑（配合 HC-SR04 + TIM1 输入捕获驱动，全程非阻塞）
//
// 策略：前进式 S 形绕行（不倒车、不原地转，符合"禁止逆行"的判罚规则）：
//
//   FOLLOW（循迹）
//     │ 前方（左右探头较小值）低于 AVOID_STOP_CM，且持续 AVOID_CONFIRM_MS
//     ▼
//   DODGE（向外让）：障碍偏左→右前弧；障碍偏右→左前弧。
//                    固定 AVOID_DODGE_MS，不看黑线（起步时车还压着线）
//     ▼
//   RETURN（回线）：向反方向走前弧，重新见到黑线即刻交还循迹
//     ▼
//   COOLDOWN：交还后等障碍离开视野（> AVOID_CLEAR_CM）或超时，
//             防止让过障碍后被残余读数二次触发
//
// 动作执行：绕行动作表示为"左右轮目标速度"（脉冲/10ms），由 main.c 里
// TIM1 中断中的速度环闭环执行——弧线半径不受电池电压影响，调一次到处管用。
//
// 用法（main.c 已按此接入）：
//   Avoid_Init();
//   while (1) {
//       switch (Avoid_Update()) {
//       case AVOID_AVOIDING:
//           g_track_enable = 1;
//           g_avoid_vl = Avoid_GetDriveL();
//           g_avoid_vr = Avoid_GetDriveR();
//           g_avoid_drive = 1;
//           break;
//       ...
//       }
//   }

#include "main.h"
#include "hcsr04.h"

/* ==================== 距离参数（厘米） ==================== */

#define AVOID_STOP_CM    25   /* 小于此距离（且持续确认）触发绕行 */
#define AVOID_SLOW_CM    50   /* 小于此距离先减速循迹 */
#define AVOID_CLEAR_CM   40   /* 两侧都回到该距离以上，冷却解除 */

/* 交还循迹的"回线到位"门槛：线回到 |偏差| <= AVOID_REJOIN_POS 才算接上线，
 * 否则继续弧线把车头摆正（TRACK_POS_MAX = 1000，400 约为阵列中段附近） */
#define AVOID_REJOIN_POS 400

/* ==================== 时间参数（毫秒） ==================== */

/* 接近状态至少持续这么久才触发绕行。
 * 左右轮流采样、单路约 80ms 更新一次，150ms ≈ 要求连续 2 次采样都
 * 在阈值内，滤掉单次异常读数（比如旁边一闪而过的反光面）。
 * 嫌反应慢就调小（最小 0），或把 AVOID_STOP_CM 调大 */
#define AVOID_CONFIRM_MS 150U

/* 采样周期：每 AVOID_SAMPLE_MS 触发下一路（两路轮流，且等上一路测完） */
#define AVOID_SAMPLE_MS  40U

/* 第一阶段（向外让）时长：要同时让开车身宽度 + 纵向越过障碍。
 * 先按 400ms 起步，用 60mm 宽的最小障碍物实测标定 */
#define AVOID_DODGE_MS   400U

/* 冷却上限：交还循迹后最迟多久重新武装避障判断 */
#define AVOID_COOLDOWN_MS 600U

/* 回线阶段最长时长：超时就交还循迹，由循迹自己找线（兜底防死绕） */
#define AVOID_RETURN_MAX_MS 2500U

/* ==================== 弧线参数（脉冲/10ms，实车标定） ==================== */

/* 弧线基础速度：比正常循迹慢，动作短、好控制 */
#define AVOID_DRIVE_SPEED 14

/* 弧线差速：越大转得越急，但必须小于 AVOID_DRIVE_SPEED，
 * 否则内轮目标速度变成负数——那就不再是弧线，而是原地急旋，
 * 急旋会把车身横到黑线上，触发误停车保护。
 * 向右让：左=base+diff、右=base-diff；向左让：左=base-diff、右=base+diff */
#define AVOID_ARC_DIFF    8

/* ==================== 接口 ==================== */

typedef enum
{
    AVOID_FOLLOW = 0, /* 正常循迹（含刚回到线上的冷却期） */
    AVOID_SLOW,       /* 循迹但减速（距离中等） */
    AVOID_AVOIDING    /* 绕行中：用 Avoid_GetDriveL/R() 的左右轮目标速度执行 */
} AvoidAction;

/* 初始化（内部会调用 HCSR04_Init） */
void Avoid_Init(void);

/* 每个主循环调用一次（非阻塞，内部按毫秒/采样周期自己计时） */
AvoidAction Avoid_Update(void);

/* 取当前动作，不刷新状态（调试用） */
AvoidAction Avoid_GetAction(void);

/* 取当前内部状态号（调试用）：
 * 0=循迹 1=右让 2=左让 3=右让后回线 4=左让后回线 5=冷却 */
uint8_t Avoid_GetState(void);

/* AVOID_AVOIDING 状态下的左右轮目标速度（脉冲/10ms），交速度环执行 */
int16_t Avoid_GetDriveL(void);
int16_t Avoid_GetDriveR(void);

/* 最近距离读数（厘米，-1 = 无回波），调试/显示用 */
int16_t Avoid_GetFrontCm(void);   /* 左右中较小者 */
int16_t Avoid_GetLeftCm(void);
int16_t Avoid_GetRightCm(void);

#endif
