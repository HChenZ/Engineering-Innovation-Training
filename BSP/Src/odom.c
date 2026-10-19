#include "odom.h"
#include "line_guard.h"
#include <math.h>
#include <string.h>

/* 64KB器件最后4KB，两个2KB槽。链接脚本必须限制程序为60KB。
 * 不兼容旧版航向图；旧格式被忽略，需重新学习。 */
#define SLOT0 0x0800F000UL
#define SLOT1 0x0800F800UL
#define SLOT_BYTES 2048U
#define MAGIC 0x4F444D33UL
#define DEG2RAD 0.01745329252f
#define RAD2DEG 57.29577951f
#define PI_F 3.141592654f

typedef struct { int16_t x_mm, y_mm, heading; } MapPoint;
typedef struct
{
    uint32_t magic, generation, checksum;
    uint16_t count, step;
    /* 标定变化后不再默默使用旧图。 */
    float pulses, width, forward, left_scale, right_scale;
} MapHeader;
_Static_assert(sizeof(MapHeader) + ODOM_MAP_MAX * sizeof(MapPoint) <= SLOT_BYTES, "map exceeds slot");

static volatile OdomMode mode;
static volatile float x, y, theta, dist, travel, progress, map_distance, heading_error;
static MapPoint teach[ODOM_MAP_MAX];
static const MapPoint *map;
static volatile uint16_t count;
static uint16_t anchor;
static uint16_t teach_lost, lost_ticks, recovery_ticks;
static float step_cm;
static volatile OdomTeachError teach_error = ODOM_TEACH_OK;
static volatile OdomRecoveryState recovery;
static float target_x, target_y, recovery_travel;
static uint32_t generation, active_slot;

static float Wrap(float a)
{
    while (a > PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

/* CRC-32：不包含提交字和checksum本身，包含所有标定值及有效点。 */
static uint32_t CrcBytes(uint32_t crc, const void *ptr, uint32_t size)
{
    const uint8_t *p = ptr;
    while (size--)
    {
        crc ^= *p++;
        for (unsigned bit = 0; bit < 8U; bit++)
            crc = (crc >> 1) ^ (0xEDB88320UL & (0U - (crc & 1U)));
    }
    return crc;
}

static uint32_t MapCrc(const MapHeader *h, const MapPoint *points)
{
    uint32_t crc = CrcBytes(0xFFFFFFFFU, &h->generation, sizeof(h->generation));
    crc = CrcBytes(crc, &h->count, sizeof(*h) - 12U);
    return ~CrcBytes(crc, points, h->count * sizeof(*points));
}

static uint8_t ValidSlot(uint32_t addr)
{
    const MapHeader *h = (const MapHeader *)(uintptr_t)addr;
    if (h->magic != MAGIC || h->count < 2U || h->count > ODOM_MAP_MAX ||
        h->step != ODOM_MAP_STEP_CM || h->pulses != ODOM_PULSES_PER_CM ||
        h->width != ODOM_TRACK_WIDTH_CM || h->forward != ODOM_SENSOR_FORWARD_CM ||
        h->left_scale != ODOM_LEFT_SCALE || h->right_scale != ODOM_RIGHT_SCALE)
        return 0U;
    return h->checksum == MapCrc(h, (const MapPoint *)(uintptr_t)(addr + sizeof(*h)));
}

static void UseFlash(void)
{
    uint8_t v0 = ValidSlot(SLOT0), v1 = ValidSlot(SLOT1);
    mode = ODOM_MODE_NONE;
    count = 0U;
    active_slot = 0U;
    generation = 0U;
    if (ODOM_ENABLE && (v0 || v1))
    {
        active_slot = (!v0 || (v1 && (int32_t)(((const MapHeader *)SLOT1)->generation -
                                             ((const MapHeader *)SLOT0)->generation) > 0)) ? SLOT1 : SLOT0;
        const MapHeader *h = (const MapHeader *)(uintptr_t)active_slot;
        count = h->count;
        generation = h->generation;
        map = (const MapPoint *)(uintptr_t)(active_slot + sizeof(*h));
        mode = ODOM_MODE_RUN;
    }
}

void Odom_Reset(void)
{
    x = y = theta = dist = travel = progress = heading_error = 0.0f;
    map_distance = 0.0f;
    anchor = teach_lost = lost_ticks = recovery_ticks = 0U;
    step_cm = 0.0f;
    recovery = ODOM_RECOVERY_IDLE;
    if (mode == ODOM_MODE_TEACH)
    {
        count = 1U;
        teach_error = ODOM_TEACH_OK;
        teach[0] = (MapPoint){ (int16_t)lroundf(ODOM_SENSOR_FORWARD_CM * 10.0f), 0, 0 };
    }
}

void Odom_Init(void) { UseFlash(); Odom_Reset(); }
void Odom_StartTeach(void)
{
    mode = ODOM_MODE_TEACH;
    map = teach;
    Odom_Reset();
}
void Odom_CancelTeach(void) { if (mode == ODOM_MODE_TEACH) UseFlash(); }
uint8_t Odom_TeachValid(void) { return mode == ODOM_MODE_TEACH && teach_error == ODOM_TEACH_OK; }
OdomTeachError Odom_GetTeachError(void) { return teach_error; }
const char *Odom_GetTeachErrorName(void)
{
    switch (teach_error)
    {
    case ODOM_TEACH_LOST_LINE: return "LOST_LINE";
    case ODOM_TEACH_MAP_FULL: return "MAP_FULL";
    case ODOM_TEACH_COORD_RANGE: return "COORD_RANGE";
    default: return "OK";
    }
}

static void TeachFault(OdomTeachError error)
{
    if (teach_error == ODOM_TEACH_OK) teach_error = error;
}

void Odom_ObserveLine(uint8_t bitmap)
{
    if (mode != ODOM_MODE_TEACH) return;
    if (bitmap) teach_lost = 0U;
    else if (teach_lost < ODOM_TEACH_LOST_TICKS) teach_lost++;
    if (teach_lost >= ODOM_TEACH_LOST_TICKS) TeachFault(ODOM_TEACH_LOST_LINE);
}

/* 在上次在线点附近找最近的线段投影，不对全图盲目跳段。坐标是传感器前端点。
 * 正常在线时更新anchor；丢线时冻结搜索区，救援选定的目标也不随车移动而跳变。 */
static void Match(uint8_t online)
{
    float sx = x + ODOM_SENSOR_FORWARD_CM * cosf(theta);
    float sy = y + ODOM_SENSOR_FORWARD_CM * sinf(theta);
    float best = 1.0e30f, best_t = 0.0f;
    uint16_t best_i = anchor;
    uint16_t lo = (anchor > ODOM_MATCH_WINDOW) ? anchor - ODOM_MATCH_WINDOW : 0U;
    uint16_t hi = anchor + ODOM_MATCH_WINDOW;
    if (hi > count - 2U) hi = count - 2U;
    for (uint16_t i = lo; i <= hi; i++)
    {
        float ax = map[i].x_mm * 0.1f, ay = map[i].y_mm * 0.1f;
        float vx = (map[i + 1U].x_mm - map[i].x_mm) * 0.1f;
        float vy = (map[i + 1U].y_mm - map[i].y_mm) * 0.1f;
        float len2 = vx * vx + vy * vy;
        float t = len2 > 0.0001f ? ((sx - ax) * vx + (sy - ay) * vy) / len2 : 0.0f;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        float dx = sx - ax - t * vx, dy = sy - ay - t * vy;
        float d2 = dx * dx + dy * dy;
        if (d2 < best) { best = d2; best_i = i; best_t = t; }
    }
    map_distance = sqrtf(best);
    progress = (best_i + best_t) * ODOM_MAP_STEP_CM;
    float h0 = map[best_i].heading * 0.1f * DEG2RAD;
    float h1 = map[best_i + 1U].heading * 0.1f * DEG2RAD;
    float h = h0 + best_t * Wrap(h1 - h0);
    heading_error = Wrap(theta - h) * RAD2DEG;
    /* 返回最近线段前方一点，减小垂直扎线的概率；再把传感器坐标换成轮轴坐标。
     * 只在进入救援前更新，进入后锁定。 */
    if (recovery == ODOM_RECOVERY_IDLE)
    {
        float ahead = best_i + best_t + ODOM_RECOVERY_LOOKAHEAD_CM / ODOM_MAP_STEP_CM;
        if (ahead > count - 1U) ahead = count - 1U;
        uint16_t i = (uint16_t)ahead;
        if (i >= count - 1U) i = count - 2U;
        float t = ahead - i;
        float ah = map[i].heading * 0.1f * DEG2RAD;
        float bh = map[i + 1U].heading * 0.1f * DEG2RAD;
        float th = ah + t * Wrap(bh - ah);
        target_x = (map[i].x_mm + t * (map[i + 1U].x_mm - map[i].x_mm)) * 0.1f
                   - ODOM_SENSOR_FORWARD_CM * cosf(th);
        target_y = (map[i].y_mm + t * (map[i + 1U].y_mm - map[i].y_mm)) * 0.1f
                   - ODOM_SENSOR_FORWARD_CM * sinf(th);
    }
    /* 救援中或明显不合地图的黑块，不能借raw bitmap重定位进度。 */
    if (online && recovery == ODOM_RECOVERY_IDLE && map_distance <= LINE_GUARD_DISTANCE_CM &&
        fabsf(heading_error) <= LINE_GUARD_CONFIRMED_DEG) anchor = best_i;
}

void Odom_Update(int16_t vL, int16_t vR, uint8_t line_centered)
{
    /* 故障后冻结这一趟的位姿和缓存，直到明确取消或开始下一次学习。 */
    step_cm = 0.0f;
    if (mode == ODOM_MODE_TEACH && teach_error != ODOM_TEACH_OK) return;
    float dl = vL * ODOM_LEFT_SCALE / ODOM_PULSES_PER_CM;
    float dr = vR * ODOM_RIGHT_SCALE / ODOM_PULSES_PER_CM;
    float ds = (dl + dr) * 0.5f;
    step_cm = ds;
    float dth = (dr - dl) / ODOM_TRACK_WIDTH_CM;
    float old_dist = dist, old_x = x, old_y = y, old_th = theta;
    x += ds * cosf(theta + dth * 0.5f);
    y += ds * sinf(theta + dth * 0.5f);
    theta += dth;
    dist += ds;
    travel += fabsf(ds);

    if (mode == ODOM_MODE_TEACH && Odom_TeachValid())
    {
        float next = count * (float)ODOM_MAP_STEP_CM;
        while (dist >= next && ds > 0.0f)
        {
            if (count >= ODOM_MAP_MAX) { TeachFault(ODOM_TEACH_MAP_FULL); break; }
            float t = (next - old_dist) / ds;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            float h = old_th + t * dth;
            float px = old_x + t * (x - old_x) + ODOM_SENSOR_FORWARD_CM * cosf(h);
            float py = old_y + t * (y - old_y) + ODOM_SENSOR_FORWARD_CM * sinf(h);
            if (fabsf(px) > 3200.0f || fabsf(py) > 3200.0f) { TeachFault(ODOM_TEACH_COORD_RANGE); break; }
            teach[count++] = (MapPoint){ (int16_t)lroundf(px * 10.0f), (int16_t)lroundf(py * 10.0f),
                                        (int16_t)lroundf(Wrap(h) * RAD2DEG * 10.0f) };
            next = count * (float)ODOM_MAP_STEP_CM;
        }
    }
    else if (Odom_MapReady()) Match(line_centered);
}

uint8_t Odom_RecoveryControl(uint8_t trusted_line, uint8_t allowed, uint8_t verifying,
                            uint8_t force_return, float *base, float *steer)
{
    *base = *steer = 0.0f;
    if (!allowed || !Odom_MapReady())
    {
        lost_ticks = recovery_ticks = 0U;
        recovery = ODOM_RECOVERY_IDLE;
        return 0U;
    }
    if (recovery == ODOM_RECOVERY_FAILED) return 1U;
    if (trusted_line)
    {
        lost_ticks = recovery_ticks = 0U;
        recovery = ODOM_RECOVERY_IDLE;
        Match(1U);
        return 0U;
    }
    if (verifying && recovery == ODOM_RECOVERY_IDLE)
    {
        lost_ticks = 0U;
        return 0U;
    }
    if (lost_ticks < ODOM_RECOVERY_LOST_TICKS) lost_ticks++;
    if (recovery == ODOM_RECOVERY_IDLE)
    {
        if (!force_return && (lost_ticks < ODOM_RECOVERY_LOST_TICKS || map_distance < ODOM_RECOVERY_TRIGGER_CM)) return 0U;
        recovery_travel = travel;
        recovery_ticks = 0U;
        recovery = map_distance > ODOM_RECOVERY_MAX_ERROR_CM ? ODOM_RECOVERY_FAILED : ODOM_RECOVERY_TURN;
    }
    if (recovery == ODOM_RECOVERY_FAILED) return 1U;
    float dx = target_x - x, dy = target_y - y;
    float distance = sqrtf(dx * dx + dy * dy);
    if (++recovery_ticks >= ODOM_RECOVERY_MAX_TICKS || travel - recovery_travel >= ODOM_RECOVERY_MAX_TRAVEL_CM ||
        (!verifying && distance <= ODOM_RECOVERY_ARRIVAL_CM))
    {
        /* 已到估计目标却没看到黑线：不继续盲开。 */
        recovery = ODOM_RECOVERY_FAILED;
        return 1U;
    }
    if (verifying) return 1U; /* 主控制器用低速灰度+地图方向验线，预算不会暂停 */
    float error = Wrap(atan2f(dy, dx) - theta) * RAD2DEG;
    if (recovery == ODOM_RECOVERY_TURN && fabsf(error) <= ODOM_RECOVERY_ALIGN_DEG) recovery = ODOM_RECOVERY_DRIVE;
    if (recovery == ODOM_RECOVERY_DRIVE && fabsf(error) > ODOM_RECOVERY_TURN_DEG) recovery = ODOM_RECOVERY_TURN;
    *steer = -ODOM_RECOVERY_KP * error;
    if (*steer > ODOM_RECOVERY_STEER_MAX) *steer = ODOM_RECOVERY_STEER_MAX;
    if (*steer < -ODOM_RECOVERY_STEER_MAX) *steer = -ODOM_RECOVERY_STEER_MAX;
    if (recovery == ODOM_RECOVERY_DRIVE) *base = ODOM_RECOVERY_SPEED;
    return 1U;
}

uint16_t Odom_Save(void)
{
    if (!Odom_TeachValid() || count < 2U || dist < ODOM_TEACH_MIN_CM) return 0U;
    uint32_t addr = active_slot == SLOT0 ? SLOT1 : SLOT0;
    MapHeader h = {0};
    h.magic = MAGIC; h.generation = generation + 1U; h.count = count; h.step = ODOM_MAP_STEP_CM;
    h.pulses = ODOM_PULSES_PER_CM; h.width = ODOM_TRACK_WIDTH_CM; h.forward = ODOM_SENSOR_FORWARD_CM;
    h.left_scale = ODOM_LEFT_SCALE; h.right_scale = ODOM_RIGHT_SCALE;
    h.checksum = MapCrc(&h, teach);
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error;
    uint8_t ok = HAL_FLASH_Unlock() == HAL_OK;
    erase.TypeErase = FLASH_TYPEERASE_PAGES; erase.PageAddress = addr; erase.NbPages = 2U;
    if (ok && HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) ok = 0U;
    /* magic最后写：掉电或写失败的新槽不会被误认为完整地图。 */
    for (uint32_t i = 4U; ok && i < sizeof(h); i += 2U)
    {
        uint16_t half;
        memcpy(&half, (const uint8_t *)&h + i, sizeof(half));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, half) != HAL_OK) ok = 0U;
    }
    for (uint32_t i = 0U; ok && i < count * sizeof(MapPoint); i += 2U)
    {
        uint16_t half;
        memcpy(&half, (const uint8_t *)teach + i, sizeof(half));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + sizeof(h) + i, half) != HAL_OK) ok = 0U;
    }
    if (ok && HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + 2U, MAGIC >> 16) != HAL_OK) ok = 0U;
    if (ok && HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, MAGIC & 0xFFFFU) != HAL_OK) ok = 0U;
    HAL_FLASH_Lock();
    if (ok) ok = ValidSlot(addr);
    UseFlash();
    return ok ? count : 0U;
}

OdomMode Odom_GetMode(void) { return mode; }
uint8_t Odom_MapReady(void) { return ODOM_ENABLE && mode == ODOM_MODE_RUN && count >= 2U; }
OdomRecoveryState Odom_GetRecoveryState(void) { return recovery; }
float Odom_GetHeadingErrDeg(void) { return heading_error; }
float Odom_GetHeadingDeg(void) { return theta * RAD2DEG; }
float Odom_GetDistCm(void) { return dist; }
float Odom_GetX(void) { return x; }
float Odom_GetY(void) { return y; }
float Odom_GetMapDistanceCm(void) { return map_distance; }
float Odom_GetProgressCm(void) { return progress; }
float Odom_GetStepCm(void) { return step_cm; }
float Odom_GetStoredLengthCm(void)
{
    if (active_slot == 0U) return 0.0f;
    const MapHeader *h = (const MapHeader *)(uintptr_t)active_slot;
    return (h->count - 1U) * (float)h->step;
}
uint16_t Odom_GetMapCount(void) { return count; }
uint16_t Odom_GetMapStepCm(void) { return ODOM_MAP_STEP_CM; }
