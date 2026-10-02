#ifndef LED_H
#define LED_H

// PC13

#include "main.h"
#include "gpio.h"

void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
void LED_Blink(void);

#endif
