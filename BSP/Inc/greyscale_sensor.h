#ifndef GREYSCALE_SENSOR_H
#define GREYSCALE_SENSOR_H

#include "main.h"

#define GRAYSCALE_SENSOR_CHANNELS 8

uint8_t Grayscale_Sensor_Read_All(void);
uint8_t Grayscale_Sensor_Read_Single(uint8_t channel);

#endif
