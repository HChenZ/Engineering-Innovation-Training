/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "led.h"
#include "motor.h"
#include "oled_software_i2c.h"
#include "avoid.h"
#include "track.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 目标直行速度（单位：脉冲/10ms，取决于车速，需实车标定）
 * 例：车轮输出轴 1 圈/秒 ≈ 1040 脉冲/秒 ≈ 10.4 脉冲/10ms */
#define TRACK_SPEED_NORMAL 25   /* 正常循迹目标速度 */
#define TRACK_SPEED_SLOW   10   /* 避障减速目标速度 */

/* 电机一致性测试模式：
 *   1 = 不循迹、不避障，左右轮速度环各自闭环到相同目标速度，
 *       用于排查电机硬件差异（看左右 PWM 是否一致）；测完改回 0 */
#define MOTOR_TEST_MODE 0
/* 测试目标速度（脉冲/10ms，约 2 圈/秒） */
#define MOTOR_TEST_SPEED 20

/* 编码器方向修正：前进时该轮速度应为正。
 * 若某轮编码器方向反了（速度闭环下该轮会飞转），把对应项设为 1 取反 */
#define ENC_LEFT_INVERT  1   /* 左轮已确认反了 */
#define ENC_RIGHT_INVERT 0

/* ==================== 抓线软启动（治"碰到线抖一下就丢"）====================
 * 现象：避障结束交回循迹、或者丢线很久之后重新压到线时，
 *       车会"猛地一抖"把线甩掉，然后又丢线去转圈找。
 *
 * 原因：位置环是纯 P。车重新压到线时，线往往在传感器排的最边上，
 *       偏差值直接顶到满量程 -> turn 立刻顶到限幅（±30）
 *       -> 内侧轮目标速度变成 0 甚至反向 -> 车一把猛打，线就被甩掉了。
 *
 * 办法：这些时刻把 turn 限幅压小一段时间，让车"顺着线慢慢拧过来"。
 *       两个条件都会触发软启动：
 *         a) 避障刚把控制权交回来；
 *         b) 连续丢线超过 TRACK_SOFT_LOST_MIN_MS 之后重新抓到线
 *            （故意留一个下限，避免过弯时闪一下丢线也触发，那样弯道会变钝）。 */
/* ⚠ 现在设成 0（关闭）：软启动的作用已经被下面的"偏差降速"取代了。
 *   软启动只是把转向压小，车的速度还是全速，所以它治的是"猛打"、治不了
 *   "高速甩线"；而偏差降速是把速度降下来（转弯半径跟着变小），
 *   根子上解决了"来不及对正线就滑出传感器"。两者叠加反而会让
 *   交回循迹后的对正变得太肉 —— 实测就是这样。
 *   想再打开就把 350 填回去（配合 TRACK_SOFT_TURN_MAX 使用）。 */
#define TRACK_SOFT_START_MS      0  /* 软启动持续多久（毫秒）；0 = 关闭这个功能 */
#define TRACK_SOFT_TURN_MAX     18  /* 软启动期间转向限幅（脉冲/10ms，正常是 30） */
#define TRACK_SOFT_LOST_MIN_MS 200  /* 丢线连续超过这么久才武装软启动 */

/* ==================== 偏差降速（治"斜着放上去跟不上去"）====================
 * 现象：把车斜着（车头与线有夹角）放在线上再上电，它跟不上去。
 *
 * 原因：斜着的时候，线会在传感器排下面横向滑走，速度是 v×sin(夹角)。
 *       车速 36cm/s、夹角 30° 时线每秒滑走 18cm，而传感器排只有 ±4cm 余量
 *       —— 也就是 0.2 秒左右线就掉出去了；而把车头拧正需要 0.2~0.35 秒。
 *       赛跑输掉 -> 丢线 -> 转圈找线。
 *
 * 办法：偏差越大，目标速度越低，给位置环留出对正的时间。
 *       偏差小（直线上）仍是全速，不影响单圈成绩。 */
#define TRACK_SLOW_POS_START  400  /* 偏差小于这个值 -> 全速（0~1000） */
#define TRACK_SLOW_POS_FULL   800  /* 偏差大于这个值 -> 降到 TRACK_SLOW_SPEED */
#define TRACK_SLOW_SPEED       12  /* 大偏差时的目标速度（脉冲/10ms，正常 25） */

/* 丢线时的速度上限。
 * 丢线的一瞬间车速还是 25（36cm/s），一秒钟就是 36cm —— 而赛道可能只有
 * 几十厘米宽，冲出去就再也找不回来了。限到 16（约 23cm/s）之后，
 * 丢线时车会"贴着线附近慢慢画圈找"。
 * ⚠ 这个值不能太小：太小的话内侧轮的目标速度会被转向吃掉变成 0，
 *   车又变成原地打转 —— 原地转是扫不到线的（车头不前进，
 *   传感器只能绕后轴画一圈，扫过线只有一两拍）。16 配上 TRACK_LOST_STEER
 *   带来的转向，大致是"外侧 30、内侧 2"，边走边转。 */
#define TRACK_LOST_SPEED       16

/* ==================== 丢线搜索：宽弧 + 定时换向 ====================
 * 原来的丢线搜索是"按最后一次看到线的方向【反着】扫"—— 那是给"过弯冲过线"
 * 准备的逻辑。但如果车只是斜着离开了线、线一直在那一侧，反着扫就是往反方向
 * 跑，越跑越远（"拐完直行、从来没回到黑线"就是这个）。
 * 而且原来的搜索转弯很急（半径不到 10cm），一旦车离线超过 25cm 就只在原地
 * 兜小圈，永远碰不到线。
 *
 * 现在改成：用一条【较宽的弧】一直往前扫，并且定时换方向 ——
 * 不管线在哪一侧，最多一个来回就能扫到；弧够宽，离车 50cm 内的线都够得着。 */
#define TRACK_SEARCH_FLIP_MS  900  /* 每这么久换一次搜索方向 */

/* ⚠ 关键分层：只有"丢线足够久"才允许切换到上面的宽弧搜索。
 *   刚脱线那一下必须保留位置环自己的输出（朝最后看到线的方向强转钩线），
 *   否则急弯稍微偏出 1~2mm 就会：满舵 ±30 瞬间泄成 ±7（转弯半径从几厘米
 *   变成约 17cm，等于直行），而且搜索方向是按时间翻转的、有一半概率反打 —— 
 *   直接冲出赛道。 */
#define TRACK_LOST_STRONG_MS 300   /* 丢线短于这个时间：保留位置环输出（强转钩线） */

#define TRACK_SEARCH_TURN      7   /* 搜索时的转向（脉冲/10ms）。
                                    * 配 TRACK_LOST_SPEED 16 时：
                                    * 外侧轮 23、内侧轮 9 -> 转弯半径约 17cm，
                                    * 加车头前伸的 20cm，横扫范围足够大。 */

/* ==================== 交回循迹时的"对正辅助" ====================
 * 位置环只有 P 的时候有个死穴：车头歪着压到线时，线可能正好落在传感器排
 * 中间（偏差 ≈ 0），位置环以为"对得正" -> 直着开 -> 斜着冲过线 -> 立刻丢线。
 * 这就是"避障做完回不到线上"的直接原因（历史里能看到 0 后面紧跟 8）。
 *
 * 辅助项看的是"偏差往哪个方向持续变化"：车头歪着时，即使偏差很小，
 * 它也会持续单向变化；车头正了就不变。漂移率经过低通滤波（抗传感器量化
 * 带来的跳变），只在辅助窗口内生效，正常循迹时这一项完全不参与。 */
#define TRACK_ASSIST_MS    700   /* 交回循迹 / 重新抓到线之后，辅助项生效多久 */
#define TRACK_ASSIST_GAIN  0.9f  /* 漂移率 -> 转向的增益；拧得太猛就降到 0.5 */
#define TRACK_ASSIST_MAX      8  /* 辅助项本身的限幅（脉冲/10ms），防一把猛打 */

/* ==================== 大角度进入循迹对正（治"斜着切入跟不上去"）====================
 * 现象：避障结束后车大角度斜着切入线，传感器快速扫过线，循迹来不及对正就丢线。
 *
 * 检测：短时间内（200ms）有4-6个不同的传感器亮过 -> 说明传感器快速横扫过线。
 *
 * 对正：检测到后先停车，小角度转弯对正（朝偏差减小的方向转），
 *       然后接入循迹并临时增大Kp，让回正更果断。 */
#define TRACK_ALIGN_ENABLE        1    /* 1 = 启用大角度对正；0 = 关闭 */
#define TRACK_ALIGN_TURN_DEG     12    /* 对正转向角度（度），10-15度范围 */
#define TRACK_ALIGN_KP_BOOST    1.4f   /* 对正后临时Kp倍率（持续TRACK_ALIGN_KP_MS） */
#define TRACK_ALIGN_KP_MS       500    /* 临时增大Kp的持续时间（毫秒） */


/* ==================== 终点停车（8 路全黑）====================
 * ⚠⚠ 这是"车莫名其妙停死、轮子完全不动、必须重新上电"的唯一来源：
 *    g_stop 一旦置 1 就永久刹车、永不清零。
 *
 * 为什么"8 路全黑"这个判据本身不可靠 ——
 *   在线上【原地转头】的时候，传感器排会转到和线平行，
 *   这时 8 路传感器全压在线上，读数和"车头正对着冲过终点线"完全一样。
 *   所以避障摆头、掉头、斜着切过线，都会被误判成"过终点"。
 *   这是这个方案的先天缺陷（两条线都是黑线，只能靠"车在干什么"来区分）。
 *
 * 四道闸，必须同时成立才停车：
 *   a) 必须先离开起点线才解锁 —— 上电时车正压在起点线上，
 *      不然第一拍中断就会判定"过终点"而永久停车；
 *   b) 全黑之前，车必须【一直在线上正常跑】TRACK_ONLINE_BEFORE 拍；
 *   c) 全黑必须【连续 TRACK_ALLBLACK_SAMPLES 拍】才确认，中途有一路没压线就清零；
 *   d) 【车必须是"沿着线直着快跑"】—— 这一条才是真正区分上面两种情况的关键：
 *      冲过终点线时两轮速度接近、都在快跑；
 *      原地摆头/掉头时两轮速度差极大（一个在转一个几乎不动），直接判掉。
 *
 * ⚠ 现在默认【关闭】。先把"跑完一圈不莫名其妙死"这件事做稳，
 *   等避障流程彻底顺了、确认不再有误判，再把它改回 1 去拿终点停止分。
 *   调试阶段强烈建议保持 0 —— 误停一次整轮就没分了。 */
#define TRACK_STOP_ON_ALLBLACK    0  /* 1 = 启用终点停车；0 = 关闭（调试默认） */
#define TRACK_ALLBLACK_SAMPLES    3  /* 连续全黑几拍才算终点（1 拍 = 10ms） */
#define TRACK_ONLINE_BEFORE      10  /* 全黑之前必须连续在线上几拍（10 拍 = 100ms） */
#define TRACK_STOP_MIN_RUN_MS  1000  /* 离开起点后至少跑多久才允许判终点（毫秒） */
#define TRACK_STOP_MAX_DIFF      10  /* 左右轮速度差超过这个值就算"在转向"，不算冲线 */
#define TRACK_STOP_MIN_SPEED      8  /* 两轮自身都要转这么快（脉冲/10ms ≈ 11cm/s） */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* 循迹状态副本（TIM1 中断里维护，主循环只读）
 * 目的：让主循环（屏幕显示等）不必去碰循迹的 CD4051 地址线，
 *       避免和中断抢 GPIO，把循迹读数搞乱。 */
volatile uint8_t g_track_active = 0;   /* 当前压线的通道数（0 = 丢线） */
volatile int16_t g_pos_copy = 0;       /* 当前循迹偏差（-1000 ~ +1000） */
/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ==================== PID 循迹控制（TIM1 中断里跑） ==================== */
static PID_t pid_pos;      /* 位置环：循迹偏差 -> 转向速度差 */
static PID_t pid_speed_l;  /* 左轮速度环 */
static PID_t pid_speed_r;  /* 右轮速度环 */

/* 控制状态（主循环写，TIM1 中断读，volatile 保证可见性） */
volatile uint8_t g_track_enable = 0;   /* 1=中断循迹，0=避障接管 */
volatile uint8_t g_stop = 0;          /* 终止停车标志：全黑确认后置 1，永久停止 */
volatile int16_t g_target_speed = 20;  /* 目标直行速度（脉冲/10ms） */

/* 大角度对正状态（主循环写，TIM1 中断读） */
volatile uint8_t  g_align_active = 0;   /* 1=正在执行对正，0=正常 */
volatile uint32_t g_align_kp_until = 0; /* 临时增大Kp的截止时刻 */

/* 调试观测值（中断写，主循环读，OLED 显示用） */
volatile int16_t g_speed_l = 0;  /* 左轮实测速度（脉冲/10ms） */
volatile int16_t g_speed_r = 0;  /* 右轮实测速度（脉冲/10ms） */
volatile int16_t g_pwm_l = 0;    /* 左轮 PWM 输出（-100~100） */
volatile int16_t g_pwm_r = 0;    /* 右轮 PWM 输出（-100~100） */

/* 终点判定的中间量（中断写，屏幕读）——用来判断"到底是谁让它停的" */
volatile uint8_t g_black_cnt = 0;   /* 连续全黑次数（到 TRACK_ALLBLACK_SAMPLES 就停车） */
volatile uint8_t g_online_cnt = 0;  /* 连续压在循迹线上的次数（0 = 刚离线/丢线） */

/* ==================== 避障动作 A 的变化历史 ====================
 * 每变一次 A 就在屏幕上多记一位数字，滚动保留最近 ACT_HIST_MAX 次。
 * 用途：避障全程只有一两秒，肉眼根本看不清 A 走过哪些状态、
 *       有没有卡在某一格。跑完停下车拍一张照片，就能把整条路径读出来
 *       （例如 "3456034560" = 转出->直行->回正->抓住->交回，重复了两轮）。
 * 只在 A 发生变化时记，所以同一状态的持续时长不会把它撑满。 */
#define ACT_HIST_MAX 20
static char    g_act_hist[ACT_HIST_MAX + 1];  /* 数字字符，末尾 '\0' */
static uint8_t g_act_hist_len = 0;

/* 往历史里压一个字符（满了就整体左移，滚动保留最近的 20 个） */
static void ActHist_Push(uint8_t code)
{
    uint8_t i;

    if (g_act_hist_len >= ACT_HIST_MAX)
    {
        for (i = 1; i < ACT_HIST_MAX; i++)
            g_act_hist[i - 1] = g_act_hist[i];
        g_act_hist_len = ACT_HIST_MAX - 1;
    }

    g_act_hist[g_act_hist_len++] = (char)('0' + (code % 10U));
    g_act_hist[g_act_hist_len]   = '\0';
}

/* 记录一次动作变化（0/2/3/4/5/6/7）。
 * ⚠ 动作和下面的事件必须各自记住"上一次的值" —— 共用一个会串味：
 *   事件记完之后，下一步动作值虽然没变也会被当成"变化"记一遍，
 *   历史里就会冒出一串莫名其妙的 0。 */
static void ActHist_AddAction(uint8_t a)
{
    static uint8_t last = 0xFF;

    if (a == last)
        return;

    last = a;
    ActHist_Push(a);
}

/* 记录一次"丢线(8) / 重新抓到线(9)"事件 */
static void ActHist_AddEvent(uint8_t e)
{
    static uint8_t last = 0xFF;

    if (e == last)
        return;

    last = e;
    ActHist_Push(e);
}

/* ==================== 屏幕调试显示 ==================== */
/* 每 200ms 刷新一次（内部限频）。
 *
 * ⚠ 这套 OLED 驱动只实现了 6x8 一种字号，sizey 必须传 8；
 *   x = 像素坐标（0~122，每个字符 6 像素宽），y = 页号（0~7，每页 8 像素高）
 *
 * ⚠ 这里只读【副本变量】(g_track_active / g_pos_copy / g_speed_* / g_pwm_*)，
 *   绝不在这里调用 Track_GetPosition() 之类的函数 —— 那些函数会操作
 *   CD4051 的地址线，和 TIM1 中断抢 GPIO，会把循迹读数搞乱。
 *
 * 显示内容：
 *   第0行：L/R = 左右超声波距离(cm，-1 = 无回波)  A = 避障动作  S = 停车标志
 *   第1行：P = 循迹偏差(-1000~1000)  T = 压线路数(8 = 全黑)
 *          O = 连续在线上几拍（越大说明"正常沿线跑"越久）  B = 连续全黑几拍
 *   第2行：SL/SR = 左右轮实测速度（脉冲/10ms，看两个轮子一致不一致）
 *   第3行：PL/PR = 左右轮 PWM 输出（-100~100）  N = 已触发避障次数
 *   第4行：A + 一串数字 = A 的变化历史（每变一次记一位，滚动保留最近 20 次）
 *          例：3456034560 = 转出->直行->回正->抓住->交回，跑了两轮
 *          另外 8 = 丢线、9 = 重新抓到线（也记在同一行）
 *
 * 动作 A：0=循迹 1=减速 2=刹停 3=转出45° 4=直行绕过 5=回正(反向转)
 *         6=抓住(回正途中扫到线刹停) 7=斜着爬回线 8=兜底画弧扫线
 * 停车 S：0=正常  1=已触发终点停车（只能重新上电复位） */
static void Debug_Show(void)
{
    static uint32_t dbg_tick = 0;

    if (HAL_GetTick() - dbg_tick < 200)
        return;

    dbg_tick = HAL_GetTick();

    /* ---- 第 0 行：左右超声波 + 避障动作 + 停车标志 ---- */
    OLED_ShowString(0, 0, (uint8_t *)"L", 8);
    OLED_ShowSignedNum(6, 0, Avoid_GetLeftCm(), 3, 8);

    OLED_ShowString(30, 0, (uint8_t *)"R", 8);
    OLED_ShowSignedNum(36, 0, Avoid_GetRightCm(), 3, 8);

    OLED_ShowString(60, 0, (uint8_t *)"A", 8);
    OLED_ShowNum(66, 0, (uint32_t)Avoid_GetAction(), 1, 8);

    OLED_ShowString(78, 0, (uint8_t *)"S", 8);
    OLED_ShowNum(84, 0, (uint32_t)g_stop, 1, 8);

    /* ---- 第 1 行：循迹偏差 + 压线路数（读副本，安全） ---- */
    OLED_ShowString(0, 1, (uint8_t *)"P", 8);
    OLED_ShowSignedNum(6, 1, g_pos_copy, 4, 8);

    OLED_ShowString(36, 1, (uint8_t *)"T", 8);
    OLED_ShowNum(42, 1, (uint32_t)g_track_active, 1, 8);

    OLED_ShowString(54, 1, (uint8_t *)"O", 8);
    OLED_ShowNum(60, 1, (uint32_t)g_online_cnt, 3, 8);

    OLED_ShowString(84, 1, (uint8_t *)"B", 8);
    OLED_ShowNum(90, 1, (uint32_t)g_black_cnt, 2, 8);

    /* ---- 第 2 行：左右轮实测速度（脉冲/10ms） ---- */
    OLED_ShowString(0, 2, (uint8_t *)"SL", 8);
    OLED_ShowSignedNum(12, 2, g_speed_l, 4, 8);

    OLED_ShowString(42, 2, (uint8_t *)"SR", 8);
    OLED_ShowSignedNum(54, 2, g_speed_r, 4, 8);

    /* ---- 第 3 行：左右轮 PWM 输出 ---- */
    OLED_ShowString(0, 3, (uint8_t *)"PL", 8);
    OLED_ShowSignedNum(12, 3, g_pwm_l, 4, 8);

    OLED_ShowString(42, 3, (uint8_t *)"PR", 8);
    OLED_ShowSignedNum(54, 3, g_pwm_r, 4, 8);

    /* ---- 第 3 行右侧：已经触发过多少次避障动作 ----
     * 数字一直往上涨 -> 车在反复触发（贴着障碍物来回蹭）；
     * 数字不动但车停住 -> 停的原因不在避障上。 */
    OLED_ShowString(84, 3, (uint8_t *)"N", 8);
    OLED_ShowNum(90, 3, Avoid_GetManeuverCount(), 3, 8);

    /* ---- 第 4 行：A 的变化历史（跑完拍这张照片就能回看整个避障过程） ---- */
    OLED_ShowString(0, 4, (uint8_t *)"A", 8);
    OLED_ShowString(6, 4, (uint8_t *)g_act_hist, 8);

    /* ---- 第 5 行：当前阶段已经走了多少脉冲（看阶段判据有没有在推进）----
     * 转出/回正数的是转角（每度 18 脉冲），直行/爬线数的是距离（每厘米 69 脉冲）。
     *   G 一直不涨 -> 这一阶段的判据失效，只能靠超时退出 -> 会走过头。 */
    OLED_ShowString(0, 5, (uint8_t *)"G", 8);
    OLED_ShowNum(6, 5, (uint32_t)Avoid_GetStagePulses(), 4, 8);

    /* ---- 第 6 行：避障【实际】发给电机的指令（PL/PR 是循迹的旧值，看不出避障）
     * 0/0 且 A=2 或 6 -> 正在刹停；某一阶段这里一直是 0/0 就是没驱动电机。 ---- */
    OLED_ShowString(0, 6, (uint8_t *)"ML", 8);
    OLED_ShowSignedNum(12, 6, g_avoid_out_l, 4, 8);
    OLED_ShowString(48, 6, (uint8_t *)"MR", 8);
    OLED_ShowSignedNum(60, 6, g_avoid_out_r, 4, 8);

    /* ---- 第 7 行：三个关键阶段【实际走到】的最大脉冲数（峰值，跑完拍照就行）
     * D = 转出（目标 540 = 30°）、S = 直行（目标 1380 = 20cm）、
     * B = 回正（目标 1080 = 60°）
     * 某个值没到目标 -> 那个阶段没走完（被提前退出 or 没驱动电机）。
     * ⚠ B 是四位数（1080），显示位数必须给 4，否则千位会被截掉。 ---- */
    OLED_ShowString(0, 7, (uint8_t *)"D", 8);
    OLED_ShowNum(6, 7, (uint32_t)g_max_dodge, 4, 8);
    OLED_ShowString(36, 7, (uint8_t *)"S", 8);
    OLED_ShowNum(42, 7, (uint32_t)g_max_pass, 4, 8);
    OLED_ShowString(72, 7, (uint8_t *)"B", 8);
    OLED_ShowNum(78, 7, (uint32_t)g_max_backturn, 4, 8);
}

/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  Encoder_Init();
  OLED_Init();
  Avoid_Init();
  Track_Init();

  /* TIM1 定时中断：10ms PID 控制周期 */
  MX_TIM1_Init();
  HAL_TIM_Base_Start_IT(&htim1);

  /* PID 参数初始化（需实车调，见各环注释） */
  PID_Init(&pid_pos,     0.07f, 0.005f, 0.0f, 500.0f, -40.0f, 40.0f);   /* 位置环：纯 P（0.07，回正更果断），限幅防边缘掉头 */
  PID_Init(&pid_speed_l, 3.0f,  0.05f, 0.0f, 300.0f, -100.0f, 100.0f); /* 左速度环 */
  PID_Init(&pid_speed_r, 3.0f,  0.05f, 0.0f, 300.0f, -100.0f, 100.0f); /* 右速度环 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (g_stop)
    {
        /* 终止停车已触发：电机保持刹车，不再做任何控制。
         * ⚠ 这里仍然刷新屏幕：不然 S 会永远停在 0，
         *   看起来像"不是终点逻辑停的"，会把人带到错的方向去查。 */
        Debug_Show();
        HAL_Delay(10);
        continue;
    }

#if MOTOR_TEST_MODE
    /* ----- 电机一致性测试：固定目标速度，左右轮速度环各自闭环 ----- */
    g_track_enable = 1;
    g_target_speed = MOTOR_TEST_SPEED;
#else
    /* ----- 超声波避障决策（循迹 PID 控制在 TIM1 中断里跑） ----- */
    {
    const AvoidAction act = Avoid_Update();

    /* 记一次 A 的变化历史（屏幕第 4 行），方便停下来拍照回看 */
    ActHist_AddAction((uint8_t)act);

    /* 顺便把"丢线 / 重新抓到线"也记进同一行：
     *   1 = 丢线（传感器排下面没有线）    9 = 重新抓到线
     * ⚠ 丢线事件用 1 而不是 8：8 是"兜底画弧扫线"这个动作的编号，
     *   两者撞车的话，事后看历史根本分不清那个 8 是"丢线"还是"进了扫线状态"。
     * 跑完拍一张照片就能判断：交回循迹之后车是稳稳跟着线走，
     * 还是立刻就丢线了（那样历史里会看到一串 1 9 1 9）。 */
    {
        static uint8_t last_online = 0xFF;
        const uint8_t  online = (g_track_active > 0U) ? 1U : 0U;

        if (last_online == 0xFF)
        {
            last_online = online;     /* 上电第一拍不记 */
        }
        else if (online != last_online)
        {
            ActHist_AddEvent(online ? 9U : 1U);
            last_online = online;
        }
    }

    switch (act)
    {
    case AVOID_FOLLOW: /* 正常循迹 */
        /* 刚从避障交回循迹：清掉避障期间攒下的 PID 残留，
         * 否则交回的那一拍输出会突然跳一下（车会顿一下/抽一下） */
        if (!g_track_enable)
        {
            PID_Reset(&pid_pos);
            PID_Reset(&pid_speed_l);
            PID_Reset(&pid_speed_r);
            Track_ResetFastSweep();  /* 重置快速扫线检测 */
        }

        g_target_speed = TRACK_SPEED_NORMAL;
        g_track_enable = 1;
        break;

    case AVOID_SLOW: /* 循迹但减速（AVOID_USE_SLOW = 0 时不会出现） */
        g_target_speed = TRACK_SPEED_SLOW;
        g_track_enable = 1;
        break;

    /* ---- 避障接管：中断停手，主循环按状态机给电机 ----
     * 关中断是为了让"g_track_enable=0 + 左右轮两条指令"整组原子生效，
     * 否则中断可能插在中间，出现"左轮避障值 + 右轮循迹值"的混合状态。 */
    default:
        __disable_irq();
        g_track_enable = 0;
        Avoid_ApplyMotion();
        __enable_irq();
        break;
    }
    }
#endif  /* MOTOR_TEST_MODE */

#if TRACK_ALIGN_ENABLE && !MOTOR_TEST_MODE
    /* ==================== 大角度进入循迹检测与对正（独立检测，每次主循环都执行）==================== */
    /* 只在循迹模式下检测，避障期间不检测 */
    if (g_track_enable && !g_align_active && Track_DetectFastSweep())
    {
        /* 检测到快速扫线：停车，根据避障方向对正 */
        const uint8_t avoid_side = Avoid_GetLastSide();

        /* 关闭循迹中断，接管控制 */
        __disable_irq();
        g_track_enable = 0;
        __enable_irq();

        /* 刹车停稳 */
        Motor_BrakeAll();
        HAL_Delay(100);

        /* 根据避障方向决定转向方向
         * 关键逻辑：避障从哪一侧绕过，回线时就需要朝反方向转回来
         * - 避障从左侧绕过（障碍在左，side=1）：车头朝右偏，需要【右转】对正
         * - 避障从右侧绕过（障碍在右，side=2）：车头朝左偏，需要【左转】对正 */
        if (avoid_side == 1 || avoid_side == 2)
        {
            /* 计算需要转的脉冲数（每度18脉冲），加大到20度 */
            const int32_t align_pulses = 20 * 18;
            const int32_t ref_l = encoder_left.total_count;
            const int32_t ref_r = encoder_right.total_count;

            /* 坦克转弯对正 */
            if (avoid_side == 1)
            {
                /* 障碍在左，从左侧绕过 → 车头朝右 → 需要右转 */
                Motor_SetSpeed(MOTOR_LEFT,  MOTOR_START, 18);   // 左轮正转
                Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, -18);  // 右轮反转
            }
            else
            {
                /* 障碍在右，从右侧绕过 → 车头朝左 → 需要左转 */
                Motor_SetSpeed(MOTOR_LEFT,  MOTOR_START, -18);  // 左轮反转
                Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, 18);   // 右轮正转
            }

            /* 等待转够角度 */
            while (1)
            {
                const int32_t dl = encoder_left.total_count - ref_l;
                const int32_t dr = encoder_right.total_count - ref_r;
                const int32_t rot = (dl > dr) ? (dl - dr) : (dr - dl);

                if (rot >= align_pulses)
                    break;

                HAL_Delay(10);
            }

            /* 停车 */
            Motor_BrakeAll();
            HAL_Delay(50);
        }

        /* 标记对正完成，接下来临时增大Kp，重新启用循迹 */
        g_align_active = 1;
        g_align_kp_until = HAL_GetTick() + TRACK_ALIGN_KP_MS;
        Track_ResetFastSweep();  /* 清除检测标志 */

        /* 清空PID残留 */
        PID_Reset(&pid_pos);
        PID_Reset(&pid_speed_l);
        PID_Reset(&pid_speed_r);

        /* 重新启用循迹 */
        __disable_irq();
        g_track_enable = 1;
        __enable_irq();
    }

    /* 检查临时Kp是否到期 */
    if (g_align_active && (int32_t)(g_align_kp_until - HAL_GetTick()) <= 0)
    {
        g_align_active = 0;
    }
#endif

    /* 屏幕调试显示（L/R 距离、避障动作、偏差、转速、PWM、停车标志） */
    Debug_Show();

    HAL_Delay(5);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* ==================== 循迹 PID 控制（TIM1 每 10ms 中断一次） ==================== */
/* 双环结构：
 *   外环（位置环）：循迹偏差 pos -> 转向速度差 turn
 *   内环（速度环）：左右轮目标速度 vs 编码器实测速度 -> PWM
 *
 * 注意编码器方向：前进时 encoder_left / encoder_right 必须都返回正值。
 * 如果某个轮子前进时速度为负（安装方向反了），把对应编码器 A/B 相对调，
 * 或在 Encoder_GetSpeed 后取反。方向错了速度环会发散（飞转/堵转）。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
#if TRACK_STOP_ON_ALLBLACK
    /* 终点判定状态（只在本中断里读写，不需要 volatile） */
    static uint8_t  start_line_cleared = 0;  /* 0 = 还压在起点线上，终点判定上锁 */
    static uint32_t leave_tick = 0;          /* 离开起点线的时刻（HAL_GetTick） */
#endif

    /* 软启动状态（见 TRACK_SOFT_START_MS 的说明） */
    static uint8_t  lost_cnt     = 0;  /* 连续丢线的拍数 */
    static uint8_t  soft_armed   = 0;  /* 丢线够久了 -> 下次抓到线要软启动 */
    static uint32_t soft_until   = 0;  /* 软启动的截止时刻 */
    static uint32_t assist_until = 0;  /* 对正辅助的截止时刻（见 TRACK_ASSIST_MS） */
    static uint8_t  assist_restart = 0;/* 辅助窗口刚开启：低通滤波器要用真实偏差重置
                                        * （丢线期间偏差是假的 ±TRACK_LOST_STEER，
                                        *   不清掉会在刚压上线时产生反向冲量） */
    static uint8_t  prev_enable  = 0;  /* 上一拍是否在循迹（用来发现"刚交回来"） */
    static uint8_t  started      = 0;  /* 是否已经循迹过一次（上电第一次不算"交回来"） */

    int16_t vL, vR;

    if (htim->Instance != TIM1)
        return;

    if (g_stop)
        return;  /* 已触发终止停车，不再控制 */

    /* ---- 1. 读编码器（必须在 g_track_enable 判断【之前】）
     * Encoder_GetSpeed() 是唯一更新累计脉冲 total_count 的地方；
     * 避障期间也照读，交回循迹时第一次读才不会算出"整个避障期间的巨大增量"，
     * 速度环也就不会在交回那一拍猛抽一下（车会顿一下）。 ---- */
    vL = Encoder_GetSpeed(&encoder_left);
    vR = Encoder_GetSpeed(&encoder_right);

#if ENC_LEFT_INVERT
    vL = -vL;  /* 左轮编码器方向修正 */
#endif
#if ENC_RIGHT_INVERT
    vR = -vR;  /* 右轮编码器方向修正 */
#endif

    /* 屏幕显示用（避障期间也刷新，方便看两个轮子一致不一致） */
    g_speed_l = vL;
    g_speed_r = vR;

    /* ---- 2. 给屏幕留一份循迹状态副本（主循环读副本，不碰 CD4051 地址线） ---- */
    g_track_active = Track_CountActive();

    /* ---- 2.5 记录"连续压在线上"的拍数（终点判定的前提，见第 4 步）
     * 避障期间也照记：绕完障碍回到线上那一瞬间它是 0，
     * 于是"斜着压线"不会被当成冲过终点线。 ---- */
    if (g_track_active > 0)
    {
        if (g_online_cnt < 250U)
            g_online_cnt++;
    }
    else
    {
        g_online_cnt = 0;
    }

    /* ---- 2.6 软启动计时 ----
     * 刚交回循迹、或者丢线很久之后重新抓到线时，先把转向限幅压小一段时间。
     *
     * 为什么必须压：位置环是纯 P，偏差一大 turn 立刻顶到限幅（±30），
     * 内侧轮目标直接变成 0 甚至反向 —— 车会"猛地一抖"把线甩掉，
     * 甩掉之后又丢线、又转圈找，就是实测看到的
     * "碰到线抖一下就继续转圈"。把刚抓到线那一下的转向压小，
     * 车就会顺着线慢慢拧过来，而不是一把打过去。 */
    if (g_track_active == 0)
    {
        if (lost_cnt < 250U)
            lost_cnt++;
        if (lost_cnt >= (TRACK_SOFT_LOST_MIN_MS / 10U))
            soft_armed = 1;          /* 丢线够久了，抓到线要软启动 */
    }
    else
    {
        if (soft_armed)
        {
            soft_until   = HAL_GetTick() + TRACK_SOFT_START_MS;
            assist_until = HAL_GetTick() + TRACK_ASSIST_MS;
            assist_restart = 1;
            soft_armed   = 0;
        }
        lost_cnt = 0;
    }

    if (!prev_enable && g_track_enable)
    {
        /* 避障刚把控制权交回来：这之前肯定离线很久了，直接软启动。
         * 但【上电第一次】不软启动 —— 上电时常常是把车斜着放在线上，
         * 那一下正需要果断对正，这时压小转向反而会跟不上去。 */
        if (started)
        {
            soft_until = HAL_GetTick() + TRACK_SOFT_START_MS;
            assist_until = HAL_GetTick() + TRACK_ASSIST_MS;
            assist_restart = 1;
        }

        started    = 1;
        lost_cnt   = 0;
        soft_armed = 0;
    }
    prev_enable = g_track_enable;

    /* ---- 3. 避障接管时，中断不再控电机 ---- */
    /* ---- 2.7 每拍都读一次循迹位置（放在"避障就 return"之前）----
     * 这样避障期间 track.c 内部的 last_position / lost 也保持新鲜 ——
     * 冻结的旧值会让恢复循迹时的丢线方向判断用错数据；
     * 顺便屏幕上的 P 也能一直显示真实偏差。 */
    {
        const int16_t pos_now = Track_GetPosition();

        g_pos_copy = pos_now;
    }

    if (!g_track_enable)
        return;

#if TRACK_STOP_ON_ALLBLACK
    /* ---- 4. 终点判定：四个条件同时成立才刹车停车（见文件开头的说明） ---- */
    if (!start_line_cleared)
    {
        /* 起点线解锁：上电时车就压在起点线上，先等它离开 */
        if (!Track_AllBlack())
        {
            start_line_cleared = 1;
            leave_tick = HAL_GetTick();
        }
    }
    else
    {
        /* "沿着线直着快跑"判据：这一条专门把"在线上原地转头"排除掉。
         * 原地转头时一个轮子快、一个几乎不动，速度差很大；
         * 冲过终点线时两个轮子都接近目标速度、差值很小。 */
        const int16_t spd_diff = (vL > vR) ? (int16_t)(vL - vR)
                                           : (int16_t)(vR - vL);
        const uint8_t straight = (spd_diff <= TRACK_STOP_MAX_DIFF) &&
                                 (vL >= TRACK_STOP_MIN_SPEED) &&
                                 (vR >= TRACK_STOP_MIN_SPEED);

        if (Track_AllBlack() && straight &&
            (g_online_cnt >= TRACK_ONLINE_BEFORE) &&
            ((HAL_GetTick() - leave_tick) >= TRACK_STOP_MIN_RUN_MS))
        {
            if (g_black_cnt < 250U)
                g_black_cnt++;

            if (g_black_cnt >= TRACK_ALLBLACK_SAMPLES)
            {
                g_stop = 1;
                g_track_enable = 0;
                Motor_BrakeAll();
                return;
            }
        }
        else
        {
            g_black_cnt = 0;  /* 条件不齐，重新计数 */
        }
    }
#endif  /* TRACK_STOP_ON_ALLBLACK */

#if MOTOR_TEST_MODE
    /* 测试模式：不循迹，位置环输出置 0，只跑左右速度环 */
    float turn = 0.0f;
    const int16_t spd = g_target_speed;
#else
    /* 5. 循迹位置偏差（在第 2.7 步已经读过，这里直接取副本） */
    const int16_t pos = g_pos_copy;

    /* 6. 位置环：偏差 -> 转向速度差（脉冲/10ms）
     * 大角度对正后临时增大Kp，让回正更果断。 */
#if TRACK_ALIGN_ENABLE
    if (g_align_active)
    {
        pid_pos.Kp *= TRACK_ALIGN_KP_BOOST;
    }
#endif

    float turn = PID_Update(&pid_pos, (float)pos);

#if TRACK_ALIGN_ENABLE
    if (g_align_active)
    {
        pid_pos.Kp /= TRACK_ALIGN_KP_BOOST;  /* 恢复原值 */
    }
#endif

    /* 6.05 大角度回正增强：偏差很大时额外补一个转向，加快回正速度。
     * 只在压线时生效（丢线时不加，避免干扰搜索逻辑）。
     * 偏差 >= 700 时补偿 +10，让大角度切入时回正更果断。 */
    if (!Track_IsLost())
    {
        const int16_t apos = (pos < 0) ? (int16_t)(-pos) : pos;
        if (apos >= 700)
        {
            /* 补偿方向和偏差方向一致：pos > 0 补正值，pos < 0 补负值 */
            const float boost = (pos > 0) ? 10.0f : -10.0f;
            turn += boost;
        }
    }

    /* 6.1 丢线时：上面的 P 输出没有意义（偏差是个固定值），
     *     改用"宽弧 + 定时换向"的搜索方式（见 TRACK_SEARCH_* 的说明）。 */
    if (Track_IsLost())
    {
        static uint32_t flip_tick  = 0;
        static uint8_t  search_dir = 0;

        /* 刚脱线（< TRACK_LOST_STRONG_MS）：保留位置环输出不动。
         * 此时 pos 是 ±TRACK_LOST_STEER，位置环会朝"最后看到线的方向"给一个
         * 强转向，急弯脱线还能自己钩回来。 */
        if (lost_cnt < (TRACK_LOST_STRONG_MS / 10U))
        {
            /* 不动 turn */
        }
        else
        {
            /* 丢线久了：宽弧 + 定时换向搜索（方向靠不住，就两边都扫） */
            if ((HAL_GetTick() - flip_tick) >= TRACK_SEARCH_FLIP_MS)
            {
                flip_tick  = HAL_GetTick();
                search_dir ^= 1U;
            }

            turn = search_dir ? (float)TRACK_SEARCH_TURN : -(float)TRACK_SEARCH_TURN;
        }
    }

    /* 软启动期间把转向压小：刚抓到线的那一下先顺着线慢慢拧过来，
     * 不要一把猛打（猛打会把线甩掉，然后又回去转圈） */
    if ((int32_t)(soft_until - HAL_GetTick()) > 0)
    {
        if (turn >  (float)TRACK_SOFT_TURN_MAX)
            turn =  (float)TRACK_SOFT_TURN_MAX;
        if (turn < -(float)TRACK_SOFT_TURN_MAX)
            turn = -(float)TRACK_SOFT_TURN_MAX;
    }

    /* 6.4 对正辅助（只在刚交回循迹/刚重新抓到线的一段时间内生效）：
     * "偏差在持续往一个方向变化"本身就说明车头和线不平行 ——
     * 即使此刻偏差很小（线正好在传感器排中间），车头其实是歪的。
     * 这一项按漂移方向补一点转向，把车头拧回来；正常循迹时它不参与。 */
    {
        static float assist_lp      = 0.0f;  /* 低通后的偏差（抗量化跳变） */
        static float assist_lp_prev = 0.0f;

        if ((int32_t)(assist_until - HAL_GetTick()) > 0)
        {
            float d;

            /* 窗口刚开的那一拍：用当前真实偏差重置滤波器。
             * 丢线期间 pos 是人为的 ±TRACK_LOST_STEER（±200），如果不清掉，
             * 刚压上线时滤波器会以为"偏差从 -200 猛涨到 +1000"，
             * 于是给出一个【反方向】的冲量，正好把线又甩掉。 */
            if (assist_restart)
            {
                assist_lp      = (float)pos;
                assist_lp_prev = assist_lp;
                assist_restart = 0;
            }

            assist_lp += ((float)pos - assist_lp) * 0.25f;
            d = TRACK_ASSIST_GAIN * (assist_lp - assist_lp_prev);
            assist_lp_prev = assist_lp;

            if (d >  (float)TRACK_ASSIST_MAX)
                d =  (float)TRACK_ASSIST_MAX;
            if (d < -(float)TRACK_ASSIST_MAX)
                d = -(float)TRACK_ASSIST_MAX;

            turn += d;
        }
        else if (Track_IsLost())
        {
            /* 丢线期间不更新滤波器（那时的偏差是假的），
             * 保留最后一次真实值，等 assist_restart 再重置。 */
            assist_restart = 1;
        }
        else
        {
            /* 不在辅助窗口内：把滤波器复位到当前偏差，避免窗口一开就爆一下 */
            assist_lp      = (float)pos;
            assist_lp_prev = assist_lp;
        }
    }

    /* 6.5 偏差降速：偏差越大目标速度越低，给位置环留出对正的时间。
     * 斜着放上车、过急弯、刚回线的时候偏差都很大，这时如果还全速冲，
     * 线会在对正完成之前就从传感器排下面溜走（现象就是"跟不上去"）。
     * 注意：丢线时【不降速】—— 那时需要冲出去把线找回来。
     *       （丢线时 Track_GetPosition() 返回的是固定值 ±TRACK_LOST_STEER，
     *         拿它当偏差去降速会把找线的螺旋压得很小，反而更难找到线。） */
    int16_t spd = g_target_speed;
    if (Track_IsLost())
    {
        /* 丢线：限速，别让车高速冲出赛道（丢线时 Track_GetPosition() 返回的是
         * 固定值 ±TRACK_LOST_STEER，拿它去插值没有意义）。 */
        if (spd > TRACK_LOST_SPEED)
            spd = TRACK_LOST_SPEED;
    }
    else
    {
        const int16_t apos = (pos < 0) ? (int16_t)(-pos) : pos;

        if (apos >= TRACK_SLOW_POS_FULL)
        {
            spd = TRACK_SLOW_SPEED;
        }
        else if (apos > TRACK_SLOW_POS_START)
        {
            /* ⚠ 插值方向必须是从"全速"降到"最低速"：
             *   偏差 = TRACK_SLOW_POS_START 时保持 g_target_speed（全速），
             *   偏差 = TRACK_SLOW_POS_FULL  时降到 TRACK_SLOW_SPEED。
             *   （之前写成 TRACK_SLOW_SPEED + ...，方向反了：小偏差降到最低速、
             *    大偏差反而升到全速 —— 直线变肉、快丢线时反而冲得更快。） */
            spd = (int16_t)(g_target_speed -
                  (int16_t)(((int32_t)(g_target_speed - TRACK_SLOW_SPEED) *
                             (apos - TRACK_SLOW_POS_START)) /
                            (TRACK_SLOW_POS_FULL - TRACK_SLOW_POS_START)));
        }
    }
#endif

    /* 7. 目标速度 = 直行速度 ± 转向（线偏右则左快右慢，向右纠偏） */
    float vL_target = (float)spd + turn;
    float vR_target = (float)spd - turn;

    /* 8. 速度环：目标 vs 实测 -> PWM 占空比 */
    float pwm_l = PID_Update(&pid_speed_l, vL_target - (float)vL);
    float pwm_r = PID_Update(&pid_speed_r, vR_target - (float)vR);

    /* 记录观测值（供 OLED 显示 / 调试观察） */
    g_pwm_l = (int16_t)pwm_l;
    g_pwm_r = (int16_t)pwm_r;

    /* 9. 更新电机 */
    Motor_SetSpeed(MOTOR_LEFT,  MOTOR_START, (int16_t)pwm_l);
    Motor_SetSpeed(MOTOR_RIGHT, MOTOR_START, (int16_t)pwm_r);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the source file name and source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
