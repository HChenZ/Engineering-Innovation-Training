#ifndef AVOID_H
#define AVOID_H

// 超声波避障状态机
//
// ==================== 动作序列 ====================
//   FOLLOW    循迹中轮流测距（左 / 中 / 右）
//     |  任一探头【连续 AVOID_CONFIRM 次】读到 <= 触发距离
//     v
//   BRAKE     刹停（防惯性蹭上去）
//     v
//   DODGE     第一段：原地坦克转，车头偏离 AVOID_DODGE_DEG
//     v
//   CIRCLE    第二段：画弧绕障，【全程被线吸引】
//     |         画弧的差速 PWM 上一直叠加"线上吸引项"：只要传感器排压着线，
//     |         就朝"把线拉回车头中心"的方向补差速 —— 轨迹自己弯向线，
//     |         不是直着切过去。（避障和循迹不再是两套独立的东西）
//     |         走够 AVOID_CIRCLE_GRAB_MIN(6cm) 之后，只要连续压线 3 拍 -> 刹停验证
//     |         走满 30cm（或超时）还没压到线 -> 刹停 -> 第三段
//     v
//   BACKTURN  第三段：反向原地转，车头斜对着线（转过 AVOID_CAPTURE_MIN_DEG 后也在找线）
//     v
//   APPROACH  缓弧朝线爬（一直找线），爬够 AVOID_APPROACH_MAX_PULSES 还没有 -> SWEEP
//     v
//   SWEEP     兜底画弧扫线，绝不停车
//
//   找线：BACKTURN / APPROACH / SWEEP
//   任何一段压到线 -> CAPTURE 刹停验证：
//     线还在 -> 交回循迹（大角度入线由 main.c 的对正逻辑处理）；
//     线不在 -> 回到原来那一段，接着之前的里程 / 转角继续做完。
//
// ==================== 使用约定 ====================
//   1. 本模块只在主循环调用，不进中断；不调用任何 Track_xxx（会和 TIM1 抢 CD4051 地址线），
//      循迹状态只读 g_track_active 副本。
//   2. Avoid_ApplyMotion() 必须由 main.c 在 __disable_irq()/__enable_irq() 之间调用。

#include "main.h"
#include "hcsr04.h"

/* ==================== 开关 ==================== */
#define AVOID_ENABLE      1   /* 0 = 只循迹，完全不避障（对照实验用） */
#define AVOID_USE_CENTER  1   /* 1 = 使用车头正中的 U2 探头 */

/* ==================== 触发 ==================== */
#define AVOID_TRIGGER_CM        23   /* 左 / 右探头触发距离（不要超过 ~35，远处是地面/墙回波） */
#define AVOID_CENTER_TRIGGER_CM 23   /* 中间探头触发距离（正对障碍，可单独调） */
#define AVOID_CONFIRM            2   /* 远区（NEAR ~ TRIGGER）：连续命中几次才算真障碍 */
#define AVOID_NEAR_CM           15   /* 近区：读到 <= 此距离【一次】就立刻触发。
                                      * 细障碍回波弱、往往很近才第一次被看到，
                                      * 这时再等第二次确认就已经撞上了 */
#define AVOID_MIN_VALID_CM       4   /* 小于此值当盲区毛刺丢掉（驱动把超短回波报成 2cm） */

/* 中间探头命中（= 必然在弯道上）时，按 8 路循迹判断往哪边绕：
 *   线在中间偏右 -> 往右转；偏左 -> 往左转。
 *   偏差绝对值 < DEADBAND（线基本居中）或丢线 -> 退回用左右探头距离判断。
 * AVOID_CENTER_TURN_TO_LINE 设 0 可整体反向（朝线的另一侧绕）。 */
#define AVOID_CENTER_TURN_TO_LINE  1
#define AVOID_CENTER_POS_DEADBAND 100   /* 偏差 -1000~1000；单压中间一路约 ±125 */

/* 以上都判断不出时往哪边绕：1 = 向右绕；2 = 向左绕 */
#define AVOID_DEFAULT_SIDE       2

#define AVOID_REARM_MS        1000   /* 交回循迹后的触发冷却（防刚回线又触发） */
#define AVOID_SAMPLE_MS         27   /* 每这么久测一路，顺序 左-中-右：
                                      * 每路约 80ms 刷新一次（和原来只有左右两路、40ms 时一样） */

/* ==================== 找线确认（按 TIM1 的 10ms 拍，必须连续，断一拍就清零） ==================== */
#define AVOID_CIRCLE_LINE_TICKS 3    /* 画弧中：连续压线 30ms 就刹停验证（擦边扫过不算） */
#define AVOID_CAPTURE_CONFIRM  2     /* 第三段及以后：运动中连续压线几拍就刹停去验证 */
#define AVOID_LINE_CONFIRM     3     /* 刹停后连续压线几拍才算真的回到线上 */
#define AVOID_CAPTURE_MS     100     /* 刹停验证：停稳这么久之后再看线 */

/* 压线角度判据：压在传感器排上的"路数"就是线相对车身的角度。
 *   车头顺着线走：线只从 1~2 路上穿过；
 *   车头和线夹角越大，线在排上"斜切"得越长，压的路数越多；接近垂直时 8 路全压。
 * 所以压了太多路的时候不许交回循迹 —— 那是"横着切过线"，交回去也是一进去就丢。 */
#define AVOID_GRAB_MAX_ACTIVE    4   /* 允许抓线时最多压几路（8 路制，>4 判为斜穿） */

/* ==================== 线上吸引项（循迹-避障耦合的核心）====================
 * 画弧 / 回正 / 爬线 / 扫线这几段本来是开环 PWM：车不知道线在哪，
 * 只能在固定距离上"猜"线该从哪儿出现。压到线的那一瞬间就变成一次
 * 模式切换 —— 切换前后两套逻辑互不相干，中间必然有一段"空档"。
 *
 * 这里把线的偏差直接叠到开环 PWM 上：只要传感器排压着线，就朝
 * "把线拉回车头中心"的方向补差速。于是画弧的轨迹自己会弯向线并顺着线走，
 * 压线不再是"切换时刻"，而是轨迹的一部分；等到连续压线 3 拍再去刹停验证，
 * 车已经是"压着线停住"的状态，交回循迹时不用再猜角度。 */
#define AVOID_LINE_CORR_TICKS    2    /* 连续压线几拍才让吸引项参与（防擦一下就走偏） */
#define AVOID_LINE_CORR_MAX     10    /* 吸引项最大差速（PWM）：左轮 +10、右轮 -10 */
#define AVOID_LINE_CORR_DEAD    80    /* 偏差绝对值小于此值算居中（不做修正，防抖） */

/* ==================== 角度 / 里程 ====================
 * 69 脉冲/cm（轮径 48mm、1040 脉冲/圈）；两轮行程差 18 脉冲 = 车身转 1°（轮距 15cm） */
#define AVOID_PULSES_PER_DEG   18
#define AVOID_PULSES_PER_CM    69

#define AVOID_DODGE_DEG        40    /* 第一段：转出角度 */
#define AVOID_BACKTURN_DEG     40    /* 第三段：回正角度（最大值，途中压线会提前收尾） */
#define AVOID_CAPTURE_MIN_DEG  30    /* 第三段转过这么多度才允许抓线（防线还挂在传感器上就停） */

#define AVOID_DODGE_PULSES     (AVOID_DODGE_DEG    * AVOID_PULSES_PER_DEG)
#define AVOID_BACKTURN_PULSES  (AVOID_BACKTURN_DEG * AVOID_PULSES_PER_DEG)

#define AVOID_CIRCLE_GRAB_MIN      (6 * AVOID_PULSES_PER_CM)   /* 画弧至少走 6cm 才允许抓线（先躲开障碍） */
#define AVOID_CIRCLE_PULSES        (30 * AVOID_PULSES_PER_CM)  /* 画弧最长 30cm，还没线就转回正 */
#define AVOID_APPROACH_MAX_PULSES  (35 * AVOID_PULSES_PER_CM)  /* 爬线 35cm 还没线 -> 兜底扫线 */

/* ==================== 电机 PWM（-100~100） ==================== */
/* 坦克转弯（两轮一正一反），别调快：主循环检查有滞后，太快会转过头 */
#define AVOID_TURN_PWM       18

#define AVOID_CIRCLE_FAST    28      /* 画弧外侧轮 */
#define AVOID_CIRCLE_SLOW    18      /* 画弧内侧轮 */

#define AVOID_APPROACH_PWM   18      /* 爬线基础速度 */
#define AVOID_APPROACH_DIFF   2      /* 爬线缓弧差速（0 = 直线，扫不到线） */

#define AVOID_SWEEP_FAST     45      /* 兜底扫线第一段 */
#define AVOID_SWEEP_SLOW     20
#define AVOID_SWEEP_FAST2    55      /* AVOID_SWEEP_MS 后仍没线，转得更急 */
#define AVOID_SWEEP_SLOW2     8
#define AVOID_SWEEP_MS     1500

/* ==================== 时间（毫秒） ==================== */
#define AVOID_BRAKE_MS           60  /* 触发后刹停 */
#define AVOID_BACKTURN_BRAKE_MS  90  /* 第三段开转前先刹停消惯性（否则反转轮顶不动） */
#define AVOID_DODGE_MAX_MS      800  /* 各段超时兜底（正常靠编码器判据结束） */
#define AVOID_CIRCLE_MAX_MS    3000  /* 画弧超时（画弧现在是"边走边被线吸引"，时间全算） */
#define AVOID_BACKTURN_MAX_MS   900

/* ==================== 动作枚举 ====================
 * 数值就是屏幕上 A 的编号，固定写死，方便看历史。 */
typedef enum
{
    AVOID_FOLLOW   = 0,   /* 交给循迹 */
    AVOID_BRAKE    = 2,   /* 刹停 */
    AVOID_DODGE    = 3,   /* 第一段：转出 */
    AVOID_CIRCLE   = 4,   /* 第二段：画弧绕障 + 找线 */
    AVOID_BACKTURN = 5,   /* 第三段：回正 */
    AVOID_CAPTURE  = 6,   /* 压到线，刹停验证 */
    AVOID_APPROACH = 7,   /* 缓弧爬线 */
    AVOID_SWEEP    = 8    /* 兜底扫线 */
} AvoidAction;

/* ==================== 函数 ==================== */
void        Avoid_Init(void);
AvoidAction Avoid_Update(void);       /* 每个主循环调用一次 */
void        Avoid_ApplyMotion(void);  /* 必须在关中断区间里调用 */
AvoidAction Avoid_GetAction(void);

/* 最近一次距离读数（cm，-1 = 无回波） */
int16_t Avoid_GetLeftCm(void);
int16_t Avoid_GetCenterCm(void);
int16_t Avoid_GetRightCm(void);

uint32_t Avoid_GetManeuverCount(void);  /* 已触发避障次数（屏幕 N） */
uint16_t Avoid_GetStagePulses(void);    /* 当前阶段已走脉冲（转角或里程，屏幕 G） */

/* 1 = 避障流程进行中（包括第二段里临时交给循迹的那一截） */
uint8_t Avoid_IsActive(void);

/* 最近一次避障的障碍方向：0 = 无 / 已冷却，1 = 障碍在左，2 = 障碍在右 */
uint8_t Avoid_GetLastSide(void);

/* main.c 的 TIM1 中断维护（避障期间也照常更新）：
 *   g_track_active 当前压线通道数（0 = 丢线）
 *   g_online_cnt   连续压线拍数（10ms/拍，丢线一拍就清零，上限 250）
 *   g_pos_copy     循迹偏差 -1000~1000（> 0 = 线偏右） */
extern volatile uint8_t g_track_active;
extern volatile uint8_t g_online_cnt;
extern volatile int16_t g_pos_copy;

/* 屏幕调试：避障实际输出的 PWM、各段走到的最大脉冲（保留到下一次避障） */
extern volatile int16_t  g_avoid_out_l;
extern volatile int16_t  g_avoid_out_r;
extern volatile uint16_t g_max_dodge;
extern volatile uint16_t g_max_circle;
extern volatile uint16_t g_max_backturn;

#endif
