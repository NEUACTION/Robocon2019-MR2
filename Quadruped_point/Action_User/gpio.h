#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx_gpio.h"


void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);

void KeyInit(void);
void LEDInit(void);
void BeepInit(void);
void PhotoelectricityInit(void);

#endif
