#include "tracking.h"
#include "greyscale_sensor.h"

#define TRACK_POS_MAX 1000

static const int8_t track_weight[8] = {-5, -4, -2, -1, 1, 2, 4, 5};

int16_t Track_GetPositionFromBitmap(uint8_t bitmap)
{
    int32_t sum_weight = 0;
    int32_t sum_amount = 0;
    uint8_t i;

    if (bitmap == 0)
        return 0; /* 丢线：无有效位置，调用者需结合 bitmap 判断 */

    for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        if (bitmap & (1u << i))
        {
            sum_weight += track_weight[i];
            sum_amount++;
        }
    }

    return (int16_t)((sum_weight * TRACK_POS_MAX) / (5 * sum_amount));
}

int16_t Track_GetPosition(void)
{
    return Track_GetPositionFromBitmap(Grayscale_Sensor_Read_All());
}

uint8_t Track_CountConsecutiveBlackFromBitmap(uint8_t bitmap)
{
    uint8_t current_count = 0;
    uint8_t max_count = 0;
    uint8_t i;

    for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        if ((bitmap & (1u << i)) != 0)
        {
            current_count++;
            if (current_count > max_count)
                max_count = current_count;
        }
        else
            current_count = 0;
    }

    return max_count;
}

uint8_t Track_CountConsecutiveBlack(void)
{
    return Track_CountConsecutiveBlackFromBitmap(Grayscale_Sensor_Read_All());
}
