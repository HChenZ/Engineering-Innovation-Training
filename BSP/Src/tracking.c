#include "tracking.h"
#include "greyscale_sensor.h"

#define TRACK_POS_MAX 1000

static const int8_t track_weight[8] = {-5, -4, -2, -1, 1, 2, 4, 5};

int16_t Track_GetPosition(void)
{
    uint8_t bitmap = Grayscale_Sensor_Read_All();
    int32_t sum_weight = 0;
    int32_t sum_amount = 0;
    uint8_t i;

    if (bitmap == 0)
        return 0; /* 丢线：无有效位置，直接给 0 */

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

uint8_t Track_CountConsecutiveBlack(void)
{
    uint8_t bitmap = Grayscale_Sensor_Read_All();
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
