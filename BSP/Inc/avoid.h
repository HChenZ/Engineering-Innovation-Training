#ifndef AVOID_H
#define AVOID_H

/* ==================== 超声波避障 ====================
 * 针对"不规则波浪线 + 随机障碍物"设计（障碍物都在循迹线任意一侧、不在线上）。
 *
 * 转弯慢、是弧线（内外轮差速），弧线的弧长前进分量足够越过障碍物，
 * 所以【没有直行段】：转出 + 摆正两个弧线大体闭合，成一个"S"形绕行：
 *   ① TURN_AWAY 转出   差速弧线，侧向超声波闭环（障碍离开侧方才停），
 *                       最小45°/最大90°兜底；实际结束角 45°~90° 浮动。
 *   ② TURN_BACK  摆正   差速弧线，反方向转【固定 45°】(AVOID_BACK_DEG)；
 *                       转出角超过 45° 时两弧不严格闭合，残余偏角靠 ③ 补。
 *   ③ SEARCH     找回   朝"线最后消失的那一侧"转+扫，压到线就交回循迹。
 *   ④ RESUME     恢复   低速循迹过渡，让位置环把车头扳正再提速。
 *
 * 动作的落地（电机 PWM / 循迹控制权切换）也由本模块完成（见 avoid.c 的
 * Avoid_Apply）：主循环只调 Avoid_Update()，参数见下面"动作执行参数"。
 *
 * 为什么转出用侧向闭环、找回朝线消失方向：
 *   障碍物大小不定(6~15cm)、线是波浪线，固定角度绕行要么绕不全要么绕过头；
 *   线转弯是连续的、不会瞬间跳到另一侧，朝它最后消失的方向扫命中率最高。
 */

#include "main.h"
#include "hcsr04.h"

/* ==================== 探头配置 ==================== */

/* 只用左右两个探头（中间探头暂时关闭） */
#define AVOID_USE_CENTER   0
/* 默认不允许后退（后退有被判"逆行"的风险） */
#define AVOID_ALLOW_BACK   0

/* ==================== 距离阈值（厘米） ==================== */

/* 小于该距离立即开始绕行。障碍物贴线放时车半宽 7.5cm 才够错开，要提前动作 */
#define AVOID_STOP_CM      18
/* 小于该距离减速但继续循迹 */
#define AVOID_SLOW_CM      30

/* 采样周期(ms)：每个周期左右两个探头都读一遍 */
#define AVOID_SAMPLE_MS    15

/* ==================== ① 转出（侧向闭环） ==================== */

/* 转出最小转角：没转够前不因"探头离开"而提前结束，防止误判 */
#define AVOID_AWAY_MIN_DEG 45.0f
/* 转出最大转角：兜底，防止卡死在转向 */
#define AVOID_AWAY_MAX_DEG 90.0f
/* 转出时间上限(ms)：转弯慢，留足余量 */
#define AVOID_AWAY_MAX_MS  600
/* 障碍侧探头 > 此值视为障碍已离开侧方 */
#define AVOID_CLEAR_CM     40

/* ==================== ② 摆正（固定角度） ==================== */

#define AVOID_BACK_DEG     45.0f   /* 摆正转角(度)：车头回正，配合转出角自己调弧线 */
#define AVOID_BACK_MAX_MS  3000    /* 摆正时间上限(ms)：转弯慢，留足余量 */

/* ==================== ③ 找回（朝线消失方向） ==================== */

#define AVOID_SEARCH_DEG   60.0f   /* 找回转角(度)：转+扫，途中压线立即交回 */
#define AVOID_SEARCH_MAX_MS 3000   /* 找回时间上限(ms) */

/* ==================== ④ 恢复 ==================== */

#define AVOID_RESUME_MS    600     /* 压到线后先低速循迹这么久，让位置环扳正车头 */
#define AVOID_COOLDOWN_MS  400     /* 绕完后忽略障碍读数的时间(ms) */

/* ==================== 车体几何（按编码器算转角） ==================== */
/* 差速转向时: 转角(rad) = (右轮行进距离 - 左轮行进距离) / 轮距
 * 轮径 48mm；编码器 1040 脉冲/圈(输出轴) */
#define AVOID_WHEEL_D_CM    4.8f    /* 轮子直径(cm) */
#define AVOID_WHEELBASE_CM  17.0f   /* 轮距(cm，左右轮中心距) */

/* ==================== 动作执行参数 ====================
 * "动作怎么落地"的参数：以前散在 main.c 主循环的避障 switch 里，
 * 和本文件的角度/超时参数是一套东西（改 PWM 会直接影响实际角速度，
 * 要同步复核 AVOID_AWAY_MAX_MS / AVOID_BACK_MAX_MS），统一放这里。 */

/* 交回循迹时的目标直行速度（脉冲/10ms，需实车标定） */
#define AVOID_TRACK_SPEED_NORMAL 30   /* 正常循迹 */
#define AVOID_TRACK_SPEED_SLOW   15   /* 减速循迹（AVOID_SLOW 动作） */

/* 绕障开环 PWM 占空比（0~100）：外轮快、内轮慢，差速越大弧线越急。
 * 当前 20/6（≈1:3.3），和 AVOID_AWAY_MAX_MS 一起决定转出/摆正的节奏。 */
#define AVOID_TURN_OUTER 20
#define AVOID_TURN_INNER 6

/* 避障动作（avoid.c 内部按此驱动电机，见 Avoid_Update） */
typedef enum
{
    AVOID_FOLLOW = 0,   /* 正常循迹 */
    AVOID_SLOW,         /* 循迹但减速 */
    AVOID_TURN_LEFT,    /* 差速往左转（外轮=右轮快，内轮=左轮慢） */
    AVOID_TURN_RIGHT    /* 差速往右转（外轮=左轮快，内轮=右轮慢） */
} AvoidAction;

/* 初始化（含超声波初始化） */
void Avoid_Init(void);

/* 每个主循环调用一次：内部采样 + 决策 + 执行（驱动电机 / 交回循迹）。
 * 返回值 = 当前动作，仅供调试显示，主循环无需再按动作做任何处理。 */
AvoidAction Avoid_Update(void);

/* 最近一次的动作（调试显示用） */
AvoidAction Avoid_GetAction(void);

/* 调试用：正前方距离（cm，-1=无回波） / 左右探头原始距离 */
int16_t Avoid_GetFrontCm(void);
int16_t Avoid_GetLeftCm(void);
int16_t Avoid_GetRightCm(void);

/* 障碍偏哪一侧：-1=偏左, +1=偏右, 0=判不出（两侧都远） */
int8_t Avoid_ObstacleSide(void);

/* 当前避障状态（调试显示用）：0循迹 1转出 2摆正 3找回 4恢复 */
uint8_t Avoid_GetState(void);

/* 本次绕行方向（调试显示用）：-1=往左, +1=往右, 0=还没定 */
int8_t Avoid_GetAwayDir(void);

#endif
