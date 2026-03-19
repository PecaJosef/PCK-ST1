/*
 * exti.c
 *
 *  Created on: Aug 16, 2025
 *      Author: pecka
 */

#include "exti.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ALT_LIM_Pin)
    {
    	//Disable interrupts
		HAL_NVIC_DisableIRQ(ALT_AxisMotor.EXTI_IRQn);

    	//Call Endstop_reached function
        endstopReached(&ALT_AxisMotor, ALT_COARSE_SPEED);
    }
    else if (GPIO_Pin == RA_LIM_Pin)
    {
    	//Disable interrupts
		HAL_NVIC_DisableIRQ(RA_AxisMotor.EXTI_IRQn);

    	//Call Endstop_reached function
		endstopReached(&RA_AxisMotor, RA_COARSE_SPEED);
    }
    else if (GPIO_Pin == DEC_LIM_Pin)
    {
    	//Disable interrupts
		HAL_NVIC_DisableIRQ(DEC_AxisMotor.EXTI_IRQn);

    	//Call Endstop_reached function
		endstopReached(&DEC_AxisMotor, DEC_COARSE_SPEED);
    }
}

