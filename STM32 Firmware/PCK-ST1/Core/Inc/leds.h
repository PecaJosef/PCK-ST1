/*
 * leds.h
 *
 *  Created on: Aug 5, 2025
 *      Author: pecka
 */

#ifndef INC_LEDS_H_
#define INC_LEDS_H_

#include "stm32l4xx_hal.h"

#define LED_TIMER TIM4

extern TIM_HandleTypeDef htim4;

void LED_IT_Handeler();

void LED_Tim_Enable();

#endif /* INC_LEDS_H_ */
