#ifndef OLED_SOFTWARE_I2C_H
#define OLED_SOFTWARE_I2C_H

// SSD1306

// PA12 - OLED_SCL
// PB5 - OLED_SDA

#include "main.h"
#include "oled_font.h"

#define OLED_CMD 0
#define OLED_DATA 1

#define OLED_SCL_Set() (HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET))
#define OLED_SCL_Clear() (HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET))

#define OLED_SDA_Set() (HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET))
#define OLED_SDA_Clear() (HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_RESET))

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ClearCharArea(uint8_t x, uint8_t y, uint8_t num_chars);
void OLED_ClearArea(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_WaitAck(void);
void Send_Byte(uint8_t dat);
void OLED_WR_Byte(uint8_t dat, uint8_t mode);
void OLED_Set_Position(uint8_t x, uint8_t y);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey);
void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t sizey);
void OLED_ShowFloatNum(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len, uint8_t sizey);
void OLED_ShowFloatNumCompact(uint8_t x, uint8_t y, float num, uint8_t dec_len, uint8_t sizey);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t sizey);

#endif
