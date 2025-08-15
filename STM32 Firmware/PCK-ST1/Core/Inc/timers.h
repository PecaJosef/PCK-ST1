/*
 * timers.h
 *
 *  Created on: Aug 5, 2025
 *      Author: pecka
 */

#ifndef INC_TIMERS_H_
#define INC_TIMERS_H_

#include "leds.h"
#include "stepper.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_TIMERS_H_ */
