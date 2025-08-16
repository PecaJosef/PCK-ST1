/*
 * exti.h
 *
 *  Created on: Aug 16, 2025
 *      Author: pecka
 */

#ifndef INC_EXTI_H_
#define INC_EXTI_H_

#include "main.h"
#include "stepper.h"
#include "stm32l4xx_hal.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* INC_EXTI_H_ */
