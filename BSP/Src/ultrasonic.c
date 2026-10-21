#include "ultrasonic.h"
#include "delay_us.h"

/*
 * 新款 HC-SR04（CS-100A）资料要点：
 * - VCC: 3.0 V ~ 5.5 V，探测范围 2 cm ~ 600 cm；
 * - TRIG 高电平至少 10 us；
 * - ECHO 高电平宽度就是超声波往返时间，距离 = 时间 * 340 m/s / 2；
 * - 超量程时 ECHO 仍可能保持高电平约 66 ms，因此必须有完整超时处理。
 *
 * 左右两路不能同时触发：同一空间内的 40 kHz 回波会互相串扰。这里采用
 * LEFT -> RIGHT 的顺序触发，并在两次触发之间留保护间隔。
 */

#define ULTRASONIC_TRIGGER_PULSE_US 20U
#define ULTRASONIC_START_TIMEOUT_US 10000U
#define ULTRASONIC_ECHO_TIMEOUT_US 70000U
#define ULTRASONIC_RELEASE_TIMEOUT_US 100000U
#define ULTRASONIC_GUARD_US 20000U

#define ULTRASONIC_FILTER_DEPTH 3U
#define ULTRASONIC_MISS_LIMIT 2U
#define ULTRASONIC_ECHO_US_MAX 36000U /* 对应约 612 cm，用于排除异常长脉冲 */

typedef struct
{
    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t echo_pin;
} Ultrasonic_Hardware;

typedef struct
{
    uint16_t history_mm[ULTRASONIC_FILTER_DEPTH];
    uint16_t filtered_mm;
    uint16_t latest_mm;
    uint16_t echo_us;
    uint32_t last_valid_tick;
    uint8_t history_pos;
    uint8_t history_count;
    uint8_t missed_cycles;
    uint8_t valid;
} Ultrasonic_Filter;

typedef enum
{
    ULTRASONIC_PHASE_IDLE = 0,
    ULTRASONIC_PHASE_WAIT_RISE,
    ULTRASONIC_PHASE_WAIT_FALL,
    ULTRASONIC_PHASE_WAIT_LOW,
    ULTRASONIC_PHASE_RESULT,
    ULTRASONIC_PHASE_BUSY
} Ultrasonic_Phase;

/* LEFT -> TRIG3/PA4, ECHO3/PA5；RIGHT -> TRIG1/PA8, ECHO1/PA9。 */
static const Ultrasonic_Hardware ultrasonic_hw[ULTRASONIC_SENSOR_COUNT] = {
    [ULTRASONIC_LEFT] = {TRIG3_GPIO_Port, TRIG3_Pin, ECHO3_GPIO_Port, ECHO3_Pin},
    [ULTRASONIC_RIGHT] = {TRIG1_GPIO_Port, TRIG1_Pin, ECHO1_GPIO_Port, ECHO1_Pin},
};

/* 左右两路轮流触发。 */
static const uint8_t ultrasonic_scan_order[ULTRASONIC_SENSOR_COUNT] = {
    ULTRASONIC_LEFT,
    ULTRASONIC_RIGHT,
};

static Ultrasonic_Filter ultrasonic_filter[ULTRASONIC_SENSOR_COUNT];

static volatile uint8_t ultrasonic_phase = ULTRASONIC_PHASE_IDLE;
static volatile uint8_t ultrasonic_active_sensor = 0;
static volatile uint32_t ultrasonic_window_start_cycles = 0;
static volatile uint32_t ultrasonic_rise_cycles = 0;
static volatile uint32_t ultrasonic_pulse_cycles = 0;

static uint8_t ultrasonic_next_scan_index = 0U;
static uint32_t ultrasonic_next_start_cycles = 0;

static uint32_t Ultrasonic_CyclesPerUs(void)
{
    uint32_t cycles = SystemCoreClock / 1000000U;

    return (cycles == 0U) ? 1U : cycles;
}

static uint32_t Ultrasonic_UsToCycles(uint32_t us)
{
    return us * Ultrasonic_CyclesPerUs();
}

static bool Ultrasonic_Elapsed(uint32_t start_cycles, uint32_t timeout_us)
{
    return (uint32_t)(DWT->CYCCNT - start_cycles) >= Ultrasonic_UsToCycles(timeout_us);
}

static bool Ultrasonic_TimeReached(uint32_t deadline_cycles)
{
    /* 只用于几十毫秒以内的调度，无符号回绕安全。 */
    return (int32_t)(DWT->CYCCNT - deadline_cycles) >= 0;
}

static bool Ultrasonic_EchoHigh(uint8_t sensor)
{
    const Ultrasonic_Hardware *hw = &ultrasonic_hw[sensor];

    return HAL_GPIO_ReadPin(hw->echo_port, hw->echo_pin) == GPIO_PIN_SET;
}

static uint16_t Ultrasonic_Median3(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t temp;

    if (a > b)
    {
        temp = a;
        a = b;
        b = temp;
    }
    if (b > c)
    {
        b = c;
    }
    if (a > b)
    {
        b = a;
    }

    return b;
}

static void Ultrasonic_ClearHistory(Ultrasonic_Filter *filter)
{
    filter->history_mm[0] = ULTRASONIC_INVALID_MM;
    filter->history_mm[1] = ULTRASONIC_INVALID_MM;
    filter->history_mm[2] = ULTRASONIC_INVALID_MM;
    filter->history_pos = 0U;
    filter->history_count = 0U;
    filter->filtered_mm = ULTRASONIC_INVALID_MM;
    filter->latest_mm = ULTRASONIC_INVALID_MM;
    filter->echo_us = 0U;
    filter->valid = 0U;
    filter->missed_cycles = 0U;
}

static void Ultrasonic_RecordSuccess(uint8_t sensor, uint16_t distance_mm, uint16_t echo_us)
{
    Ultrasonic_Filter *filter = &ultrasonic_filter[sensor];
    uint16_t median = distance_mm;

    filter->history_mm[filter->history_pos] = distance_mm;
    filter->history_pos++;
    if (filter->history_pos >= ULTRASONIC_FILTER_DEPTH)
    {
        filter->history_pos = 0U;
    }
    if (filter->history_count < ULTRASONIC_FILTER_DEPTH)
    {
        filter->history_count++;
    }

    if (filter->history_count >= 3U)
    {
        median = Ultrasonic_Median3(filter->history_mm[0], filter->history_mm[1], filter->history_mm[2]);
    }
    else if (filter->history_count == 2U)
    {
        uint32_t sum = (uint32_t)filter->history_mm[0] + (uint32_t)filter->history_mm[1];
        median = (uint16_t)((sum + 1U) / 2U);
    }

    filter->filtered_mm = median;
    filter->latest_mm = distance_mm;
    filter->echo_us = echo_us;
    filter->last_valid_tick = HAL_GetTick();
    filter->valid = 1U;
    filter->missed_cycles = 0U;
}

static void Ultrasonic_RecordMiss(uint8_t sensor)
{
    Ultrasonic_Filter *filter = &ultrasonic_filter[sensor];

    filter->latest_mm = ULTRASONIC_INVALID_MM;
    filter->echo_us = 0U;
    if (filter->missed_cycles < 0xFFU)
    {
        filter->missed_cycles++;
    }

    /* 保留一次中值结果以跨越偶发漏回波；连续失败则立即失效。 */
    if (filter->missed_cycles >= ULTRASONIC_MISS_LIMIT)
    {
        Ultrasonic_ClearHistory(filter);
        filter->missed_cycles = ULTRASONIC_MISS_LIMIT;
    }
}

static void Ultrasonic_FinishMeasurement(bool success, uint32_t pulse_cycles)
{
    uint32_t now = DWT->CYCCNT;
    uint8_t sensor = ultrasonic_active_sensor;

    if (success)
    {
        uint32_t echo_us = pulse_cycles / Ultrasonic_CyclesPerUs();
        uint32_t distance_mm = (echo_us * 17U + 50U) / 100U;

        if ((echo_us >= 1U) && (echo_us <= ULTRASONIC_ECHO_US_MAX) && (distance_mm >= ULTRASONIC_MIN_RANGE_MM) && (distance_mm <= ULTRASONIC_MAX_RANGE_MM))
        {
            Ultrasonic_RecordSuccess(sensor, (uint16_t)distance_mm, (uint16_t)echo_us);
        }
        else
        {
            Ultrasonic_RecordMiss(sensor);
        }
    }
    else
    {
        Ultrasonic_RecordMiss(sensor);
    }

    ultrasonic_next_start_cycles = now + Ultrasonic_UsToCycles(ULTRASONIC_GUARD_US);
    ultrasonic_phase = ULTRASONIC_PHASE_IDLE;
}

static void Ultrasonic_TriggerNext(void)
{
    uint32_t primask;
    uint8_t sensor = ultrasonic_scan_order[ultrasonic_next_scan_index];
    const Ultrasonic_Hardware *hw = &ultrasonic_hw[sensor];

    if (Ultrasonic_EchoHigh(sensor))
    {
        /*
         * ECHO 一直为高时不能把整组扫描卡死：记一次失败并跳到下一路。
         * 正常模块在 20 ms 保护间隔后应已释放 ECHO。
         */
        Ultrasonic_RecordMiss(sensor);
        ultrasonic_next_scan_index++;
        if (ultrasonic_next_scan_index >= ULTRASONIC_SENSOR_COUNT)
        {
            ultrasonic_next_scan_index = 0U;
        }
        ultrasonic_next_start_cycles = DWT->CYCCNT + Ultrasonic_UsToCycles(ULTRASONIC_GUARD_US);
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    ultrasonic_active_sensor = sensor;
    ultrasonic_window_start_cycles = DWT->CYCCNT;
    __HAL_GPIO_EXTI_CLEAR_IT(hw->echo_pin);
    ultrasonic_phase = ULTRASONIC_PHASE_WAIT_RISE;
    __set_PRIMASK(primask);

    HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_SET);
    Delay_us(ULTRASONIC_TRIGGER_PULSE_US);
    HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_RESET);

    ultrasonic_next_scan_index++;
    if (ultrasonic_next_scan_index >= ULTRASONIC_SENSOR_COUNT)
    {
        ultrasonic_next_scan_index = 0U;
    }
}

void Ultrasonic_Init(void)
{
    /*
     * GPIO 模式、上下拉和 EXTI 边沿属性由 CubeMX 生成的 MX_GPIO_Init() 配置。本函数必须在 MX_GPIO_Init() 之后调用。
     */
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);

    ultrasonic_phase = ULTRASONIC_PHASE_IDLE;
    ultrasonic_active_sensor = 0U;
    ultrasonic_window_start_cycles = 0U;
    ultrasonic_rise_cycles = 0U;
    ultrasonic_pulse_cycles = 0U;
    ultrasonic_next_scan_index = 0U;
    ultrasonic_next_start_cycles = DWT->CYCCNT;
    Ultrasonic_ResetFilters();
}

void Ultrasonic_Process(void)
{
    bool measurement_done = false;
    bool measurement_valid = false;
    uint8_t sensor;
    uint32_t primask;
    uint32_t pulse_cycles = 0U;

    switch (ultrasonic_phase)
    {
    case ULTRASONIC_PHASE_IDLE:
        if (Ultrasonic_TimeReached(ultrasonic_next_start_cycles))
        {
            Ultrasonic_TriggerNext();
        }
        break;

    case ULTRASONIC_PHASE_WAIT_RISE:
        sensor = ultrasonic_active_sensor;
        if (Ultrasonic_EchoHigh(sensor))
        {
            /* EXTI 漏边沿时的软件兜底。 */
            primask = __get_PRIMASK();
            __disable_irq();
            if (ultrasonic_phase == ULTRASONIC_PHASE_WAIT_RISE)
            {
                ultrasonic_rise_cycles = DWT->CYCCNT;
                ultrasonic_phase = ULTRASONIC_PHASE_WAIT_FALL;
            }
            __set_PRIMASK(primask);
        }
        else if (Ultrasonic_Elapsed(ultrasonic_window_start_cycles, ULTRASONIC_START_TIMEOUT_US))
        {
            primask = __get_PRIMASK();
            __disable_irq();
            if (ultrasonic_phase == ULTRASONIC_PHASE_WAIT_RISE)
            {
                ultrasonic_phase = ULTRASONIC_PHASE_BUSY;
                measurement_done = true;
            }
            __set_PRIMASK(primask);
        }
        break;

    case ULTRASONIC_PHASE_WAIT_FALL:
        sensor = ultrasonic_active_sensor;
        if (!Ultrasonic_EchoHigh(sensor))
        {
            primask = __get_PRIMASK();
            __disable_irq();
            if (ultrasonic_phase == ULTRASONIC_PHASE_WAIT_FALL)
            {
                pulse_cycles = (uint32_t)DWT->CYCCNT - ultrasonic_rise_cycles;
                ultrasonic_phase = ULTRASONIC_PHASE_BUSY;
                measurement_done = true;
                measurement_valid = true;
            }
            __set_PRIMASK(primask);
        }
        else if (Ultrasonic_Elapsed(ultrasonic_rise_cycles, ULTRASONIC_ECHO_TIMEOUT_US))
        {
            primask = __get_PRIMASK();
            __disable_irq();
            if (ultrasonic_phase == ULTRASONIC_PHASE_WAIT_FALL)
            {
                ultrasonic_phase = ULTRASONIC_PHASE_WAIT_LOW;
            }
            __set_PRIMASK(primask);
        }
        break;

    case ULTRASONIC_PHASE_WAIT_LOW:
        sensor = ultrasonic_active_sensor;
        if (!Ultrasonic_EchoHigh(sensor) ||
            Ultrasonic_Elapsed(ultrasonic_rise_cycles, ULTRASONIC_RELEASE_TIMEOUT_US))
        {
            primask = __get_PRIMASK();
            __disable_irq();
            if (ultrasonic_phase == ULTRASONIC_PHASE_WAIT_LOW)
            {
                ultrasonic_phase = ULTRASONIC_PHASE_BUSY;
                measurement_done = true;
            }
            __set_PRIMASK(primask);
        }
        break;

    case ULTRASONIC_PHASE_RESULT:
        primask = __get_PRIMASK();
        __disable_irq();
        if (ultrasonic_phase == ULTRASONIC_PHASE_RESULT)
        {
            pulse_cycles = ultrasonic_pulse_cycles;
            ultrasonic_phase = ULTRASONIC_PHASE_BUSY;
            measurement_done = true;
            measurement_valid = true;
        }
        __set_PRIMASK(primask);
        break;

    case ULTRASONIC_PHASE_BUSY:
    default:
        break;
    }

    if (measurement_done)
    {
        Ultrasonic_FinishMeasurement(measurement_valid, pulse_cycles);
    }
}

bool Ultrasonic_GetReading(Ultrasonic_Sensor sensor, Ultrasonic_Reading *reading)
{
    const Ultrasonic_Filter *filter;
    uint32_t age_ms;

    if ((reading == NULL) ||
        ((uint32_t)sensor >= ULTRASONIC_SENSOR_COUNT))
    {
        return false;
    }

    filter = &ultrasonic_filter[sensor];
    age_ms = HAL_GetTick() - filter->last_valid_tick;

    reading->filtered_mm = filter->filtered_mm;
    reading->latest_mm = filter->latest_mm;
    reading->echo_us = filter->echo_us;
    reading->age_ms = age_ms;
    reading->missed_cycles = filter->missed_cycles;
    reading->valid = (uint8_t)((filter->valid != 0U) && (age_ms <= ULTRASONIC_STALE_MS));

    if (reading->valid == 0U)
    {
        reading->filtered_mm = ULTRASONIC_INVALID_MM;
    }

    return reading->valid != 0U;
}

bool Ultrasonic_GetDistance(Ultrasonic_Sensor sensor, uint16_t *distance_mm)
{
    Ultrasonic_Reading reading;

    if ((distance_mm == NULL) || !Ultrasonic_GetReading(sensor, &reading))
    {
        return false;
    }

    *distance_mm = reading.filtered_mm;
    return true;
}

void Ultrasonic_GetAll(Ultrasonic_Reading readings[ULTRASONIC_SENSOR_COUNT])
{
    uint32_t i;

    if (readings == NULL)
    {
        return;
    }

    for (i = 0U; i < ULTRASONIC_SENSOR_COUNT; i++)
    {
        (void)Ultrasonic_GetReading((Ultrasonic_Sensor)i, &readings[i]);
    }
}

void Ultrasonic_ResetFilters(void)
{
    uint32_t i;
    uint32_t tick = HAL_GetTick();

    for (i = 0U; i < ULTRASONIC_SENSOR_COUNT; i++)
    {
        Ultrasonic_ClearHistory(&ultrasonic_filter[i]);
        ultrasonic_filter[i].last_valid_tick = tick;
    }
}

/*
 * 三个 ECHO 引脚对应的 EXTI 回调。只接受当前正在测量的那一路边沿，
 * 其它通道即使因为干扰产生边沿也不会污染本次结果。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t sensor = ultrasonic_active_sensor;
    uint32_t now;

    if ((sensor >= ULTRASONIC_SENSOR_COUNT) || (GPIO_Pin != ultrasonic_hw[sensor].echo_pin))
    {
        return;
    }

    now = DWT->CYCCNT;

    if ((ultrasonic_phase == ULTRASONIC_PHASE_WAIT_RISE) && (HAL_GPIO_ReadPin(ultrasonic_hw[sensor].echo_port, ultrasonic_hw[sensor].echo_pin) == GPIO_PIN_SET))
    {
        ultrasonic_rise_cycles = now;
        ultrasonic_phase = ULTRASONIC_PHASE_WAIT_FALL;
    }
    else if ((ultrasonic_phase == ULTRASONIC_PHASE_WAIT_FALL) && (HAL_GPIO_ReadPin(ultrasonic_hw[sensor].echo_port, ultrasonic_hw[sensor].echo_pin) == GPIO_PIN_RESET))
    {
        ultrasonic_pulse_cycles = now - ultrasonic_rise_cycles;
        ultrasonic_phase = ULTRASONIC_PHASE_RESULT;
    }
}
