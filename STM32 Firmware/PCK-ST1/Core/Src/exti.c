/*
 * exti.c
 *
 *  Created on: Aug 16, 2025
 *      Author: pecka
 */

#include "exti.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == EL_STOP_Pin) {

    	//Disable interrupts
        HAL_NVIC_DisableIRQ(EXTI2_IRQn);
        Stepper_Stop(&EL_Axis_motor);
    }
}

