#include "odom.h"

#include <math.h>

/* ==================== Flash 存图 ====================
 * STM32F103C8 共 64KB，每页 1KB；用最后一页（链接脚本里已把 FLASH 缩成 63K，程序不会占到这里）
 * 布局（全部按半字写）：
 *   +0  magic(32 位)  +4 点数  +6 步长(cm)  +8 校验和(32 位)  +16 起：航向点（int16，单位 0.1°） */
#define ODOM_FLASH_ADDR   0x0800FC00UL
#define ODOM_FLASH_MAGIC  0x4F444D31UL   /* "ODM1" */
#define ODOM_DATA_OFFSET  16U

#define DEG2RAD  0.017453293f
#define RAD2DEG  57.29577951f
#define TWO_PI   6.283185307f
#define PI_F     3.141592654f

typedef struct
{
    uint32_t magic;
    uint16_t count;
    uint16_t step_cm;
    uint32_t checksum;
    uint32_t reserved;
} OdomHeader;

/* 位姿（中断写，主循环读；32 位读写是原子的） */
static volatile float x_cm, y_cm, theta;   /* theta：弧度，不回绕，左转为正 */
static volatile float dist_cm;             /* 实际累计里程 */
static volatile float progress_cm;         /* 沿赛道进度 */
static volatile float err_deg;             /* 航向误差 */

static volatile OdomMode mode = ODOM_MODE_NONE;
static volatile uint8_t  teach_done = 0;

static int16_t        map_ram[ODOM_MAP_MAX];   /* 示教时的缓存 */
static const int16_t *map = map_ram;           /* 当前使用的地图（RUN 时指向 Flash） */
static volatile uint16_t map_count = 0;

/* ==================== 小工具 ==================== */

static float WrapPi(float a)
{
    while (a >  PI_F) a -= TWO_PI;
    while (a < -PI_F) a += TWO_PI;
    return a;
}

static int16_t ToTenthDeg(float rad)
{
    float d = rad * RAD2DEG * 10.0f;

    if (d >  32000.0f) d =  32000.0f;
    if (d < -32000.0f) d = -32000.0f;
    return (int16_t)((d >= 0.0f) ? (d + 0.5f) : (d - 0.5f));
}

/* 地图在进度 p 处的航向（弧度），相邻两点线性插值 */
static float RefAt(float p)
{
    float idx;
    uint16_t i;

    if (map_count == 0)
        return 0.0f;

    idx = p / (float)ODOM_MAP_STEP_CM;
    if (idx <= 0.0f)
        return (float)map[0] * (DEG2RAD / 10.0f);

    i = (uint16_t)idx;
    if (i >= map_count - 1U)
        return (float)map[map_count - 1U] * (DEG2RAD / 10.0f);

    {
        const float f  = idx - (float)i;
        const float a  = (float)map[i];
        const float b  = (float)map[i + 1U];

        return (a + (b - a) * f) * (DEG2RAD / 10.0f);
    }
}

static uint32_t Checksum(const int16_t *data, uint16_t count, uint16_t step)
{
    uint32_t sum = 0x12345678UL ^ ((uint32_t)count << 16) ^ step;
    uint16_t i;

    for (i = 0; i < count; i++)
        sum = (sum << 5) + (sum >> 27) + (uint16_t)data[i];

    return sum;
}

static uint8_t FlashMapValid(void)
{
    const OdomHeader *h = (const OdomHeader *)ODOM_FLASH_ADDR;
    const int16_t    *d = (const int16_t *)(ODOM_FLASH_ADDR + ODOM_DATA_OFFSET);

    if (h->magic != ODOM_FLASH_MAGIC)
        return 0;
    if (h->count < 2U || h->count > ODOM_MAP_MAX || h->step_cm != ODOM_MAP_STEP_CM)
        return 0;
    return (Checksum(d, h->count, h->step_cm) == h->checksum) ? 1U : 0U;
}

/* ==================== 初始化 ==================== */

void Odom_Init(void)
{
    x_cm = y_cm = theta = 0.0f;
    dist_cm = progress_cm = err_deg = 0.0f;
    teach_done = 0;

#if ODOM_ENABLE
    if (!ODOM_FORCE_TEACH && FlashMapValid())
    {
        map       = (const int16_t *)(ODOM_FLASH_ADDR + ODOM_DATA_OFFSET);
        map_count = ((const OdomHeader *)ODOM_FLASH_ADDR)->count;
        mode      = ODOM_MODE_RUN;
    }
    else
    {
        map       = map_ram;
        map_count = 0;
        mode      = ODOM_MODE_TEACH;
    }
#else
    map_count = 0;
    mode      = ODOM_MODE_NONE;
#endif
}

/* ==================== 每 10ms 更新（TIM1 中断里） ==================== */

void Odom_Update(int16_t vL, int16_t vR, uint8_t on_line_stable)
{
    const float dl  = (float)vL / ODOM_PULSES_PER_CM;
    const float dr  = (float)vR / ODOM_PULSES_PER_CM;
    const float ds  = 0.5f * (dl + dr);
    const float dth = (dr - dl) / ODOM_TRACK_WIDTH_CM;
    float th = theta;

    /* 中点法积分：用这一拍转到一半时的朝向去分解位移，比用起点朝向准 */
    {
        const float mid = th + 0.5f * dth;

        x_cm += ds * cosf(mid);
        y_cm += ds * sinf(mid);
    }
    th      += dth;
    dist_cm += ds;

    if (mode == ODOM_MODE_TEACH)
    {
        /* 示教时车一直在线上，实际里程就是沿赛道进度 */
        const float p = (dist_cm > 0.0f) ? dist_cm : 0.0f;

        progress_cm = p;
        while (map_count < ODOM_MAP_MAX &&
               p >= (float)map_count * (float)ODOM_MAP_STEP_CM)
        {
            map_ram[map_count] = ToTenthDeg(th);
            map_count++;
        }

        if (p >= (float)ODOM_ROUTE_CM || map_count >= ODOM_MAP_MAX)
            teach_done = 1;
    }
    else if (mode == ODOM_MODE_RUN)
    {
        const float max_p = (float)(map_count - 1U) * (float)ODOM_MAP_STEP_CM;
        float e = WrapPi(th - RefAt(progress_cm));
        float c = cosf(e);
        float p;

        /* 只把"顺着赛道方向"的分量算进度：避障横着绕出去时进度几乎不涨，
         * 掉头往回走时进度也不往回退（cos < 0 当 0） */
        if (c < 0.0f)
            c = 0.0f;
        p = progress_cm + ds * c;
        if (p < 0.0f)  p = 0.0f;
        if (p > max_p) p = max_p;
        progress_cm = p;

        /* 纠漂：车稳稳压在线中间时，它的朝向就是赛道方向 */
        if (on_line_stable && fabsf(e) < (ODOM_FUSE_ERR_MAX_DEG * DEG2RAD))
        {
            th -= ODOM_FUSE_GAIN * e;
            e  -= ODOM_FUSE_GAIN * e;
        }

        err_deg = e * RAD2DEG;
    }

    theta = th;
}

/* ==================== 存图 ==================== */

uint8_t Odom_Save(void)
{
    FLASH_EraseInitTypeDef er;
    uint32_t page_err = 0;
    OdomHeader h;
    const uint16_t *hw;
    uint32_t addr;
    uint16_t i;
    uint8_t ok = 1;

    if (mode != ODOM_MODE_TEACH || map_count < 2U)
        return 0;

    h.magic    = ODOM_FLASH_MAGIC;
    h.count    = map_count;
    h.step_cm  = ODOM_MAP_STEP_CM;
    h.checksum = Checksum(map_ram, map_count, ODOM_MAP_STEP_CM);
    h.reserved = 0xFFFFFFFFUL;

    HAL_FLASH_Unlock();

    er.TypeErase   = FLASH_TYPEERASE_PAGES;
    er.PageAddress = ODOM_FLASH_ADDR;
    er.NbPages     = 1;
    if (HAL_FLASHEx_Erase(&er, &page_err) != HAL_OK)
        ok = 0;

    hw   = (const uint16_t *)&h;
    addr = ODOM_FLASH_ADDR;
    for (i = 0; ok && i < sizeof(h) / 2U; i++, addr += 2U)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, hw[i]) != HAL_OK)
            ok = 0;

    addr = ODOM_FLASH_ADDR + ODOM_DATA_OFFSET;
    for (i = 0; ok && i < map_count; i++, addr += 2U)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, (uint16_t)map_ram[i]) != HAL_OK)
            ok = 0;

    HAL_FLASH_Lock();

    if (ok && FlashMapValid())
    {
        map  = (const int16_t *)(ODOM_FLASH_ADDR + ODOM_DATA_OFFSET);
        mode = ODOM_MODE_RUN;
        return 1;
    }
    return 0;
}

/* ==================== 查询 ==================== */

OdomMode Odom_GetMode(void)        { return mode; }
uint8_t  Odom_MapReady(void)       { return (mode == ODOM_MODE_RUN) ? 1U : 0U; }
uint8_t  Odom_TeachDone(void)      { return teach_done; }
float    Odom_GetHeadingErrDeg(void) { return Odom_MapReady() ? err_deg : 0.0f; }

uint8_t Odom_IsLargeDeviation(void)
{
    return (Odom_MapReady() && fabsf(err_deg) > ODOM_LARGE_DEG) ? 1U : 0U;
}

float    Odom_GetX(void)           { return x_cm; }
float    Odom_GetY(void)           { return y_cm; }
float    Odom_GetHeadingDeg(void)  { return theta * RAD2DEG; }
float    Odom_GetDistCm(void)      { return dist_cm; }
float    Odom_GetProgressCm(void)  { return progress_cm; }
uint16_t Odom_GetMapCount(void)    { return map_count; }
