#ifndef GREYSCALE_SENSOR_H
#define GREYSCALE_SENSOR_H

#include "main.h"
#include "delay_us.h"

#define GRAYSCALE_SENSOR_CHANNELS 8

void Grayscale_Sensor_Read_All(uint16_t* sensor_values);
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel);

#endif
