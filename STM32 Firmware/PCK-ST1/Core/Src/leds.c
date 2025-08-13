/*
 * leds.c
 *
 *  Created on: Aug 5, 2025
 *      Author: pecka
 */
#include "leds.h"
#include "stm32l4xx_hal.h"
#include "main.h"


void LED_IT_Handeler()
{
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void LED_Tim_Enable()
{
	HAL_TIM_Base_Start_IT(&htim4);
}
