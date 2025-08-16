/*
 * timers.c
 *
 *  Created on: Aug 5, 2025
 *      Author: pecka
 */
#include "timers.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == LED_TIMER)
  {
	  //LEDs timer
	  LED_IT_Handeler();
  }
  else if(htim->Instance == STEPPER_TIMER)
  {
	  //Stepper timer AZ,EL
	  Stepper_IT_Handeler();
  }
  else if (htim->Instance == DEC_TIM)
  {
	  //Stepper timer DEC
	  Stepper_Stop(&DEC_Axis_motor);
  }
  else if(htim->Instance == RA_TIM)
  {
	  //Stepper timer RA
	  Stepper_Stop(&RA_Axis_motor);
  }

}
