#ifndef BLUETOOTH_TEST_USART_H
#define BLUETOOTH_TEST_USART_H
#include <stdint.h>
typedef struct { void *Instance; uint32_t ErrorCode; } UART_HandleTypeDef;
#define USART2 ((void *)(uintptr_t)2U)
#define HAL_UART_ERROR_ORE 8U
#define __HAL_UART_CLEAR_OREFLAG(h) ((h)->ErrorCode = 0U)
extern UART_HandleTypeDef huart2;
int HAL_UART_Receive_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
int HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);
#endif
