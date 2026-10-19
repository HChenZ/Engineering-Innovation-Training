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
static uint16_t turn_in_ticks = 0U;
static float turn_out_differential = 0.0f;
static float extra_turn_travel = 0.0f;
static uint16_t extra_turn_check_ticks = 0U;
static uint8_t extra_turn_skip_travel = 0U;
static float extra_turn_speed_diff = AVOIDANCE_EXTRA_SPEED_DIFF_2;

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
    turn_in_ticks = 0U;
    turn_out_differential = 0.0f;
    extra_turn_travel = 0.0f;
    extra_turn_check_ticks = 0U;
    extra_turn_skip_travel = 0U;
    extra_turn_speed_diff = AVOIDANCE_EXTRA_SPEED_DIFF_2;
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

Avoidance_Control Avoidance_UpdateControl(uint8_t bitmap, float left_speed, float right_speed)
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
            turn_in_ticks = 0U;      /* 保底转圈时间从进入转入段这一拍重新数 */
            steer_sign = -steer_sign;
        }

        control.use_line_pid = 0U;
        control.base_speed = AVOIDANCE_BASE_SPEED;
        control.steer = steer_sign * AVOIDANCE_SPEED_DIFF;
        return control;
    }

    if (avoidance_state == AVOIDANCE_STATE_TURN_IN)
    {
        uint8_t black_count = Track_CountBlackFromBitmap(bitmap);
        uint8_t consecutive_black = Track_CountConsecutiveBlackFromBitmap(bitmap);
        uint8_t line_detected;

        /* 记"已经在转入段转了多少拍"（保底转圈时间用，每拍 10ms）。 */
        if (turn_in_ticks < 0xFFFFU)
        {
            turn_in_ticks++;
        }

        /* 黑线必须是一整段连续区域（≥2 个黑点），才算真的抓到了线。 */
        line_detected = (uint8_t)((black_count >= 2U) && (black_count == consecutive_black));

        /*
         * 保底转圈时间没到之前【只转、不判线】（见 avoid.h 里
         * AVOIDANCE_TURN_IN_MIN_TICKS 的说明）：
         * 弯道旁的黑色印刷字/深色块同样会被读成"连续黑"，如果不设这道闸，
         * 车会把它当黑线、提前结束避障、还没绕开障碍就交回循迹，然后彻底丢线。
         *
         * AVOIDANCE_TURN_IN_MIN_TICKS = 0 时，本判断恒成立，行为与改动前完全一致。
         */
        if ((turn_in_ticks >= AVOIDANCE_TURN_IN_MIN_TICKS) && (line_detected != 0U))
        {
            /* 黑线必须是一整段连续区域，才切到第三段小弧线。 */
            avoidance_state = AVOIDANCE_STATE_EXTRA_TURN;
            extra_turn_travel = 0.0f;
            extra_turn_check_ticks = 0U;
            extra_turn_skip_travel = 1U;

            if (consecutive_black >= 4U)
            {
                extra_turn_speed_diff = AVOIDANCE_EXTRA_SPEED_DIFF_4_PLUS;
            }
            else if (consecutive_black == 3U)
            {
                extra_turn_speed_diff = AVOIDANCE_EXTRA_SPEED_DIFF_3;
            }
            else
            {
                extra_turn_speed_diff = AVOIDANCE_EXTRA_SPEED_DIFF_2;
            }
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
        float average_delta;

        /*
         * 进入第三段的首个速度反馈来自第二段，不能计入第三段行驶距离。
         */
        if (extra_turn_skip_travel != 0U)
        {
            extra_turn_skip_travel = 0U;
        }
        else
        {
            average_delta = (left_speed + right_speed) * 0.5f;
            if (average_delta > 0.0f)
            {
                extra_turn_travel += average_delta;
            }
        }

        if (extra_turn_travel >= AVOIDANCE_EXTRA_MIN_TRAVEL_COUNTS)
        {
            if (extra_turn_check_ticks < 0xFFFFU)
            {
                extra_turn_check_ticks++;
            }

            /* 达到最小距离后，才允许由黑线与超时结束第三段。 */
            if (((bitmap & AVOIDANCE_MIDDLE_FOUR_MASK) != 0U) ||
                (extra_turn_check_ticks >= AVOIDANCE_EXTRA_TIMEOUT_TICKS))
            {
                avoidance_state = AVOIDANCE_STATE_IDLE;
                active_direction = AVOIDANCE_DIRECTION_NONE;
                pending_direction = AVOIDANCE_DIRECTION_NONE;
                waiting_for_line_direction = 0U;
                return control;
            }
        }

        steer_sign = DirectionSign(active_direction);
        control.use_line_pid = 0U;
        control.base_speed = AVOIDANCE_BASE_SPEED;
        control.steer = steer_sign * extra_turn_speed_diff;
        return control;
    }

    return control;
}

Avoidance_State Avoidance_GetState(void)
{
    return avoidance_state;
}
