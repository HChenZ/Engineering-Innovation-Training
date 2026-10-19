#include "avoidance.h"
#include "tracking.h"

#define SENSOR_LEFT_BIT (1U << ULTRASONIC_LEFT)
#define SENSOR_CENTER_BIT (1U << ULTRASONIC_CENTER)
#define SENSOR_RIGHT_BIT (1U << ULTRASONIC_RIGHT)

static volatile Avoidance_State avoidance_state = AVOIDANCE_STATE_IDLE;
static volatile Avoidance_Direction pending_direction = AVOIDANCE_DIRECTION_NONE;
static volatile Avoidance_Direction active_direction = AVOIDANCE_DIRECTION_NONE;
static volatile uint8_t waiting_for_line_direction = 0U;

/* 只有主循环修改；Avoidance_Reset() 在中断中清零。 */
static volatile uint8_t locked_sensor_mask = 0U;
static int16_t last_nonzero_position = 0;

/* 只有 TIM1 中断修改。 */
static uint16_t turn_out_ticks = 0U;
static float turn_out_differential = 0.0f;
static uint16_t extra_turn_ticks = 0U;
static float extra_turn_differential = 0.0f;
static uint8_t extra_turn_skip_accumulation = 0U;
static uint16_t turn_in_ticks = 0U;
static uint8_t line_hint_ticks = 0U;

static uint8_t SensorReadingFresh(const Ultrasonic_Reading *reading)
{
    return (uint8_t)((reading->valid != 0U) && (reading->age_ms <= AVOIDANCE_SENSOR_MAX_AGE_MS) && (reading->filtered_mm != ULTRASONIC_INVALID_MM));
}

static uint8_t SensorNear(const Ultrasonic_Reading *reading, uint16_t trigger_mm)
{
    return (uint8_t)(SensorReadingFresh(reading) && (reading->filtered_mm <= trigger_mm));
}

static uint8_t SensorCleared(const Ultrasonic_Reading *reading, uint16_t trigger_mm)
{
    return (uint8_t)(SensorReadingFresh(reading) && (reading->filtered_mm >= (uint16_t)(trigger_mm + AVOIDANCE_RELEASE_EXTRA_MM)));
}

static float DirectionSign(Avoidance_Direction direction)
{
    return (direction == AVOIDANCE_DIRECTION_RIGHT) ? 1.0f : -1.0f;
}

static void RequestDirection(Avoidance_Direction direction)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if ((avoidance_state == AVOIDANCE_STATE_IDLE) && (pending_direction == AVOIDANCE_DIRECTION_NONE))
    {
        pending_direction = direction;
        waiting_for_line_direction = 0U;
    }
    __set_PRIMASK(primask);
}

void Avoidance_Reset(void)
{
    avoidance_state = AVOIDANCE_STATE_IDLE;
    pending_direction = AVOIDANCE_DIRECTION_NONE;
    active_direction = AVOIDANCE_DIRECTION_NONE;
    waiting_for_line_direction = 0U;
    locked_sensor_mask = 0U;
    last_nonzero_position = 0;
    turn_out_ticks = 0U;
    turn_out_differential = 0.0f;
    extra_turn_ticks = 0U;
    extra_turn_differential = 0.0f;
    extra_turn_skip_accumulation = 0U;
    turn_in_ticks = 0U;
    line_hint_ticks = 0U;
}

void Avoidance_Init(void)
{
    Avoidance_Reset();
}

void Avoidance_ProcessSensors(const Ultrasonic_Reading readings[ULTRASONIC_SENSOR_COUNT], uint8_t bitmap, int16_t line_position)
{
    uint8_t near_mask = 0U;

    if (readings == NULL)
    {
        return;
    }

    if ((bitmap != 0U) && (line_position != 0))
    {
        last_nonzero_position = line_position;
    }

    if (SensorNear(&readings[ULTRASONIC_LEFT], AVOIDANCE_SIDE_TRIGGER_MM))
    {
        near_mask |= SENSOR_LEFT_BIT;
    }
    if (SensorNear(&readings[ULTRASONIC_CENTER], AVOIDANCE_CENTER_TRIGGER_MM))
    {
        near_mask |= SENSOR_CENTER_BIT;
    }
    if (SensorNear(&readings[ULTRASONIC_RIGHT], AVOIDANCE_SIDE_TRIGGER_MM))
    {
        near_mask |= SENSOR_RIGHT_BIT;
    }

    /* 障碍物离开并超过回滞距离后，才重新允许对应探头触发。 */
    if (((locked_sensor_mask & SENSOR_LEFT_BIT) != 0U) && SensorCleared(&readings[ULTRASONIC_LEFT], AVOIDANCE_SIDE_TRIGGER_MM))
    {
        locked_sensor_mask &= (uint8_t)~SENSOR_LEFT_BIT;
    }
    if (((locked_sensor_mask & SENSOR_CENTER_BIT) != 0U) && SensorCleared(&readings[ULTRASONIC_CENTER], AVOIDANCE_CENTER_TRIGGER_MM))
    {
        locked_sensor_mask &= (uint8_t)~SENSOR_CENTER_BIT;
    }
    if (((locked_sensor_mask & SENSOR_RIGHT_BIT) != 0U) && SensorCleared(&readings[ULTRASONIC_RIGHT], AVOIDANCE_SIDE_TRIGGER_MM))
    {
        locked_sensor_mask &= (uint8_t)~SENSOR_RIGHT_BIT;
    }

    if ((avoidance_state != AVOIDANCE_STATE_IDLE) || (pending_direction != AVOIDANCE_DIRECTION_NONE))
    {
        return;
    }

    /*
     * 障碍在左则向右避，障碍在右则向左避。
     * 按约定两侧不会同时进入阈值，因此左侧优先即可。
     */
    if (((locked_sensor_mask & SENSOR_LEFT_BIT) == 0U) && ((near_mask & SENSOR_LEFT_BIT) != 0U))
    {
        locked_sensor_mask |= near_mask;
        RequestDirection(AVOIDANCE_DIRECTION_RIGHT);
        return;
    }

    if (((locked_sensor_mask & SENSOR_RIGHT_BIT) == 0U) && ((near_mask & SENSOR_RIGHT_BIT) != 0U))
    {
        locked_sensor_mask |= near_mask;
        RequestDirection(AVOIDANCE_DIRECTION_LEFT);
        return;
    }

    if (((near_mask & SENSOR_CENTER_BIT) != 0U) && (((locked_sensor_mask & SENSOR_CENTER_BIT) == 0U) || (waiting_for_line_direction != 0U)))
    {
        int16_t direction_position = (line_position != 0) ? line_position : last_nonzero_position;

        locked_sensor_mask |= near_mask;

        if (direction_position > 0)
        {
            RequestDirection(AVOIDANCE_DIRECTION_RIGHT);
        }
        else if (direction_position < 0)
        {
            RequestDirection(AVOIDANCE_DIRECTION_LEFT);
        }
        else
        {
            /* 本次启动后还没有出现过非零位置：先低速直行等待灰度方向。 */
            waiting_for_line_direction = 1U;
        }
        return;
    }

    waiting_for_line_direction = 0U;
}

Avoidance_Control Avoidance_UpdateControl(uint8_t bitmap, float left_speed, float right_speed,
                                          uint8_t line_candidate)
{
    Avoidance_Control control;
    float steer_sign;

    control.use_line_pid = 1U;
    control.base_speed = 0.0f;
    control.steer = 0.0f;

    if (avoidance_state == AVOIDANCE_STATE_IDLE)
    {
        if (pending_direction == AVOIDANCE_DIRECTION_NONE)
        {
            if (waiting_for_line_direction != 0U)
            {
                control.use_line_pid = 0U;
                control.base_speed = AVOIDANCE_WAIT_SPEED;
            }
            return control;
        }

        active_direction = pending_direction;
        pending_direction = AVOIDANCE_DIRECTION_NONE;
        avoidance_state = AVOIDANCE_STATE_TURN_OUT;
        turn_out_ticks = 0U;
        turn_out_differential = 0.0f;
    }

    if (avoidance_state == AVOIDANCE_STATE_TURN_OUT)
    {
        steer_sign = DirectionSign(active_direction);

        turn_out_differential += left_speed - right_speed;
        if (turn_out_ticks < 0xFFFFU)
        {
            turn_out_ticks++;
        }

        if (((steer_sign * turn_out_differential) >= AVOIDANCE_OUT_TURN_DIFF_COUNTS) ||
            (turn_out_ticks >= AVOIDANCE_OUT_TIMEOUT_TICKS))
        {
            avoidance_state = AVOIDANCE_STATE_TURN_IN;
            turn_in_ticks = 0U;
            line_hint_ticks = 0U;
            steer_sign = -steer_sign;
        }

        control.use_line_pid = 0U;
        control.base_speed = AVOIDANCE_BASE_SPEED;
        control.steer = steer_sign * AVOIDANCE_SPEED_DIFF;
        return control;
    }

    if (avoidance_state == AVOIDANCE_STATE_TURN_IN)
    {
        if (turn_in_ticks < AVOIDANCE_IN_TIMEOUT_TICKS) turn_in_ticks++;
        if (line_candidate && Track_CountConsecutiveBlackFromBitmap(bitmap) >= 2U)
        {
            if (line_hint_ticks < AVOIDANCE_LINE_HINT_TICKS) line_hint_ticks++;
        }
        else line_hint_ticks = 0U;
        if (line_hint_ticks >= AVOIDANCE_LINE_HINT_TICKS)
        {
            /*
             * 保留原避障方向，立即切到第三段小弧线。
             * 当前速度反馈属于第二段，因此本周期不参与第三段差速累计。
             */
            avoidance_state = AVOIDANCE_STATE_EXTRA_TURN;
            extra_turn_ticks = 0U;
            extra_turn_differential = 0.0f;
            extra_turn_skip_accumulation = 1U;
        }
        else if (turn_in_ticks >= AVOIDANCE_IN_TIMEOUT_TICKS)
        {
            /* 不是回线成功：主控制器保留return_pending，低速验证/地图救援接手。 */
            avoidance_state = AVOIDANCE_STATE_IDLE;
            active_direction = AVOIDANCE_DIRECTION_NONE;
            return control;
        }
        else
        {
            steer_sign = -DirectionSign(active_direction);
            control.use_line_pid = 0U;
            control.base_speed = AVOIDANCE_BASE_SPEED;
            control.steer = steer_sign * AVOIDANCE_SPEED_DIFF;
            return control;
        }
    }

    if (avoidance_state == AVOIDANCE_STATE_EXTRA_TURN)
    {
        steer_sign = DirectionSign(active_direction);

        if (extra_turn_skip_accumulation != 0U)
        {
            extra_turn_skip_accumulation = 0U;
        }
        else
        {
            extra_turn_differential += left_speed - right_speed;
        }

        if (extra_turn_ticks < 0xFFFFU)
        {
            extra_turn_ticks++;
        }

        if (((steer_sign * extra_turn_differential) >= AVOIDANCE_EXTRA_TURN_DIFF_COUNTS) ||
            (extra_turn_ticks >= AVOIDANCE_EXTRA_TIMEOUT_TICKS))
        {
            avoidance_state = AVOIDANCE_STATE_IDLE;
            active_direction = AVOIDANCE_DIRECTION_NONE;
            pending_direction = AVOIDANCE_DIRECTION_NONE;
            waiting_for_line_direction = 0U;
            extra_turn_skip_accumulation = 0U;
            return control;
        }

        control.use_line_pid = 0U;
        control.base_speed = AVOIDANCE_BASE_SPEED;
        control.steer = steer_sign * AVOIDANCE_EXTRA_SPEED_DIFF;
        return control;
    }

    return control;
}

Avoidance_State Avoidance_GetState(void)
{
    return avoidance_state;
}

Avoidance_Direction Avoidance_GetDirection(void)
{
    return active_direction;
}
