/*
 * timers.c
 *
 *  Created on: Aug 5, 2025
 *      Author: pecka
 */
#include "timers.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == STEPPER_TIMER)
  {
	  //Stepper timer AZ,EL
	  stepperInterruptHandler();
  }
  else if (htim->Instance == DEC_TIM)
  {
	  //Stepper timer DEC
	  stepperStop(&DEC_AxisMotor);
  }
  else if(htim->Instance == RA_TIM)
  {
	  //Stepper timer RA
	  stepperStop(&RA_AxisMotor);
  }
  else if(htim->Instance == CAMERA_TIM)
  {
	  //Camera control timer - 5ms interrupts
	  cameraInterruptHandler();
  }

}
