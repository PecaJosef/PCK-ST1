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

#include "usbd_cdc_if.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
	  //LEDs timer
	  LED_IT_Handeler();
  }
  else if(htim->Instance == TIM3)
  {
	  //Stepper timer AZ,EL
	  Stepper_IT_Handeler();
  }
  else if (htim->Instance == TIM5)
  {
	  //Stepper timer DEC
	  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
	  HAL_TIM_Base_Stop_IT(&htim5);
  }
  else if(htim->Instance == TIM2)
  {
	  //Stepper timer RA
	  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	  HAL_TIM_Base_Stop_IT(&htim2);
	  //HAL_GPIO_TogglePin(PWR_BTN_LED_GPIO_Port, PWR_BTN_LED_Pin);
  }

}
