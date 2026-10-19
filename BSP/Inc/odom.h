#ifndef ODOM_H
#define ODOM_H

// 轮式里程计（odometry）+ 航位推算（dead reckoning）+ 固定赛道的"航向地图"
//
// ==================== 思路 ====================
//   1. 里程计：每 10ms 用左右轮编码器增量推算车的位姿 (x, y, θ) 和累计里程 s。
//   2. 地图：赛道是固定的，所以"沿线走了 s 厘米时，车头应该朝哪"是固定的。
//      示教跑一趟（只循迹、不避障），每 ODOM_MAP_STEP_CM 记一次 θ，存进 Flash
//      最后一页 —— 这就是整张赛道的"航向曲线" θref(s)。
//   3. 比赛跑：一边推算 θ，一边按"沿赛道的进度"查 θref，得到航向误差
//          e = θ - θref(进度)
//      |e| 大 = 车头和赛道方向差了一个大角度（冲出去 / 斜着切线 / 避障后歪着）。
//   4. 进度：用 ds·cos(e) 累加 —— 避障绕出去时，只有"顺着赛道方向"的分量算进度，
//      绕行多走的横向路程不会让进度跑到前面去。
//   5. 纠漂：编码器推算的 θ 会慢慢漂；车稳稳压在线中间时（偏差小、连续在线），
//      把 θ 往 θref 慢慢拉（互补滤波），所以全程不会越漂越远。
//
// ==================== 怎么用 ====================
//   Flash 里没有地图（第一次烧录） -> 自动进入【示教】：把车按比赛起点的位置和朝向
//     摆好，上电，车只循迹不避障，跑满 ODOM_ROUTE_CM 后自动停车并存地图。
//     示教期间赛道上【不要放障碍物】。
//   Flash 里有地图 -> 正常比赛模式，每次上电都从 s = 0、θ = 0 开始，
//     所以【每次都要按示教时一样的位置和朝向摆车】。
//   想重新示教：ODOM_FORCE_TEACH 设 1 烧一次，示教完再改回 0 烧一次。
//   ⚠ 重新烧录程序时，烧录工具若设成"整片擦除"会把地图一起擦掉（屏幕上 M 变 0），
//     请改成"只擦除用到的扇区"。
//
// ==================== 使用约定 ====================
//   Odom_Update() 只在 TIM1 中断里调用；Odom_Save() 只在主循环里、车停稳后调用。

#include "main.h"

#define ODOM_ENABLE            1     /* 0 = 整个模块不工作（不示教、不给误差） */

/* ==================== 需要实车标定的参数 ==================== */
/* 每厘米脉冲数 = 编码器每圈脉冲 / 轮周长 = 1040 / (π × 4.8cm) ≈ 68.97
 * 标定：推车直走 100cm，看屏幕 K（里程 cm），不是 100 就按比例改这里 */
#define ODOM_PULSES_PER_CM     68.97f

/* 有效轮距（cm）。图纸 15cm，但轮胎有宽度、会打滑，实际值要标定：
 * 原地转 5 整圈（1800°），看屏幕 H（航向，度）；
 * H 读数偏大 -> 把这里调大；偏小 -> 调小（按 读数/1800 的比例乘上去） */
#define ODOM_TRACK_WIDTH_CM    15.0f

/* ==================== 赛道 / 地图 ==================== */
#define ODOM_ROUTE_CM        600     /* 赛道全长（cm），示教跑满这么长就停车存图。
                                      * 量一下实际赛道沿线长度填进来（稍微多填一点没关系） */
#define ODOM_MAP_STEP_CM       2     /* 每隔多少 cm 记一个航向点 */
#define ODOM_MAP_MAX         500     /* 最多存多少点（Flash 一页 1KB 的上限） */
#define ODOM_FORCE_TEACH       0     /* 1 = 不管 Flash 里有没有图，都重新示教 */

#if (ODOM_ROUTE_CM / ODOM_MAP_STEP_CM) + 1 > ODOM_MAP_MAX
#error "赛道太长：调大 ODOM_MAP_STEP_CM"
#endif

/* ==================== 纠漂（互补滤波） ==================== */
#define ODOM_FUSE_GAIN       0.02f   /* 每拍把 θ 往 θref 拉多少（0.02 = 约 0.5s 收敛一半） */
#define ODOM_FUSE_POS_MAX     250    /* 循迹偏差小于这个才算"压在线中间" */
#define ODOM_FUSE_ONLINE       20    /* 连续在线这么多拍（200ms）才纠 */
#define ODOM_FUSE_ERR_MAX_DEG 20.0f  /* 误差太大时不纠（可能是进度对错了，别把 θ 拉歪） */

/* ==================== 大角度偏移 ==================== */
#define ODOM_LARGE_DEG        35.0f  /* |航向误差| 超过这个就算大角度偏移 */

/* 丢线时用地图找线：转向 = HOLD_KP × (e + 偏置)，偏置朝"最后看到线的那一侧" */
#define ODOM_HOLD_KP          0.4f   /* 每度误差给多少转向（脉冲/10ms） */
#define ODOM_HOLD_TURN_MAX     12    /* 转向限幅 */
#define ODOM_SEARCH_BIAS_DEG  15.0f  /* 在赛道方向上再往线那一侧多偏多少度（横向靠回去） */

/* ==================== 函数 ==================== */
typedef enum
{
    ODOM_MODE_NONE = 0,   /* 没有地图：只做里程计，不给误差 */
    ODOM_MODE_TEACH,      /* 示教中：边跑边记 */
    ODOM_MODE_RUN         /* 有地图：给航向误差 */
} OdomMode;

void     Odom_Init(void);   /* 上电时调用：读 Flash 决定模式，清零位姿 */

/* TIM1 中断每 10ms 调一次。vL / vR 是方向修正后的轮速（脉冲/10ms，前进为正）。
 * on_line_stable = 1 表示"此刻车稳稳压在线中间"，用来纠漂 */
void     Odom_Update(int16_t vL, int16_t vR, uint8_t on_line_stable);

OdomMode Odom_GetMode(void);
uint8_t  Odom_MapReady(void);        /* 1 = 有地图，误差可用 */
uint8_t  Odom_TeachDone(void);       /* 示教跑满全长了（主循环看到就停车 + Odom_Save） */
uint8_t  Odom_Save(void);            /* 写 Flash，1 = 成功；成功后自动切到 RUN */

float    Odom_GetHeadingErrDeg(void);   /* 航向误差 e（度，-180~180；> 0 = 车头偏左了） */
uint8_t  Odom_IsLargeDeviation(void);   /* |e| > ODOM_LARGE_DEG */

/* 显示 / 调试 */
float    Odom_GetX(void);            /* cm，起点为原点，起点朝向为 +x */
float    Odom_GetY(void);            /* cm，左为 +y */
float    Odom_GetHeadingDeg(void);   /* 累计航向（度，不回绕；左转为正） */
float    Odom_GetDistCm(void);       /* 实际累计里程 */
float    Odom_GetProgressCm(void);   /* 沿赛道的进度（查地图用） */
uint16_t Odom_GetMapCount(void);     /* 地图点数（屏幕 M） */

#endif
