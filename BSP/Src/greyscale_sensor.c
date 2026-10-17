#include "greyscale_sensor.h"
#include "delay_us.h"

// 选择传感器通道
static void Select_Channel(uint8_t channel)
{
    HAL_GPIO_WritePin(Track_AD0_GPIO_Port, Track_AD0_Pin, ((channel >> 0) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET); // bit0 -> AD0
    HAL_GPIO_WritePin(Track_AD1_GPIO_Port, Track_AD1_Pin, ((channel >> 1) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET); // bit1 -> AD1
    HAL_GPIO_WritePin(Track_AD2_GPIO_Port, Track_AD2_Pin, ((channel >> 2) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET); // bit2 -> AD2
}

// 读取 OUT 引脚的值
static uint16_t Read_OUT_Value(void)
{
    return HAL_GPIO_ReadPin(Track_Out_GPIO_Port, Track_Out_Pin) == GPIO_PIN_SET ? 0 : 1;
}

// 读取所有 8 个通道的灰度值
void Grayscale_Sensor_Read_All(uint16_t* sensor_values)
{
    uint8_t i;
    for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        Select_Channel(i);
        Delay_us(50);
        sensor_values[i] = Read_OUT_Value();
    }
}

// 读取单个指定通道的灰度值
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel)
{
    if (channel >= GRAYSCALE_SENSOR_CHANNELS)
    {
        return 0; // 无效通道
    }
    Select_Channel(channel);
    Delay_us(50);
    return Read_OUT_Value();
}
