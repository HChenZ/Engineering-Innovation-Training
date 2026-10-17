#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 三路 HC-SR04 的逻辑编号。具体引脚映射见 ultrasonic.c 中的硬件表。 */
typedef enum
{
    ULTRASONIC_LEFT = 0,
    ULTRASONIC_CENTER = 1,
    ULTRASONIC_RIGHT = 2
} Ultrasonic_Sensor;

#define ULTRASONIC_SENSOR_COUNT 3U
#define ULTRASONIC_INVALID_MM 0xFFFFU

/* 模块资料标称量程为 2 cm ~ 600 cm。 */
#define ULTRASONIC_MIN_RANGE_MM 20U
#define ULTRASONIC_MAX_RANGE_MM 6000U

/* 超过这个时间没有完成一次新的有效采样，就认为数据已过期。 */
#define ULTRASONIC_STALE_MS 500U

typedef struct
{
    uint16_t filtered_mm; /* 最近三次有效采样的中值，INVALID 表示当前不可用 */
    uint16_t latest_mm;   /* 最近一次有效原始距离，INVALID 表示最近一次无回波 */
    uint16_t echo_us;     /* 最近一次有效回波的高电平宽度，单位 us */
    uint32_t age_ms;      /* 距离最近一次有效采样的时间 */
    uint8_t valid;        /* 1: filtered_mm 可用于控制；0: 无有效数据 */
    uint8_t missed_cycles;/* 连续超时/超量程次数 */
} Ultrasonic_Reading;

/*
 * 初始化三路超声波模块。
 */
void Ultrasonic_Init(void);

/*
 * 驱动调度函数，必须在主循环中尽可能频繁地调用。
 *
 * 函数本身不做长延时，也不会等待 Echo 脉冲结束。三路模块会依次触发，
 * 避免同时发射造成相互串扰。Echo 边沿由 EXTI 在后台记录。
 */
void Ultrasonic_Process(void);

/*
 * 读取指定模块的中值滤波距离。
 *
 * 返回 true 时 distance_mm 写入有效距离；返回 false 表示尚未采到数据、
 * 连续无回波、超量程，或者数据超过 ULTRASONIC_STALE_MS。
 */
bool Ultrasonic_GetDistance(Ultrasonic_Sensor sensor, uint16_t *distance_mm);

/* 读取完整状态，适合调试、屏幕显示或做更上层的数据融合。 */
bool Ultrasonic_GetReading(Ultrasonic_Sensor sensor, Ultrasonic_Reading *reading);

/* 一次读取三路状态，数组顺序为 LEFT、CENTER、RIGHT。 */
void Ultrasonic_GetAll(Ultrasonic_Reading readings[ULTRASONIC_SENSOR_COUNT]);

/* 清空三路滤波历史；下一次成功回波会立即成为新的有效值。 */
void Ultrasonic_ResetFilters(void);

#ifdef __cplusplus
}
#endif

#endif
