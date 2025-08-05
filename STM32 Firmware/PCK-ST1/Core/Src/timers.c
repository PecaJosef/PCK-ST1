/*
 * timers.c
 *
 *  Created on: Aug 5, 2025
 *      Author: pecka
 */
#include "timers.h"
#include "leds.h"
#include "stepper.h"
#include "main.h"
#include "stm32l4xx_hal.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
	  //LEDs timer
	  LED_IT_Handeler();
  }
  else if(htim->Instance == TIM3)
  {
	  //Stepper timer
	  Stepper_IT_Handeler();
  }

}
