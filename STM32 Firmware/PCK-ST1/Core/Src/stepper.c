/*
 * stepper.c
 *
 *  Created on: Aug 5, 2025
 *      Author: pecka
 */
#include "stepper.h"
#include "usbd_cdc_if.h"
#include "math.h"
#include "telemetry.h"

StepperMotor_t ALT_AxisMotor = {
	.STEP_Port = ALT_STEP_GPIO_Port,
	.STEP_Pin = ALT_STEP_Pin,
	.EN_Port = ALT_EN_GPIO_Port,
	.EN_Pin = ALT_EN_Pin,
	.DIR_Port = ALT_DIR_GPIO_Port,
	.DIR_Pin = ALT_DIR_Pin,
	.ENDSTOP_Port = ALT_LIM_GPIO_Port,
	.ENDSTOP_Pin = ALT_LIM_Pin,
	.EXTI_IRQn = ALT_LIM_EXTI_IRQn,
	.stepsPerArcmin = ALT_STEPS_PER_ARCMIN,
	.enabled = false,
	.busy = false,
	.homing = false,
	.highPrecisionAxis = false,
	.homing_state = AXIS_HOMING_IDLE,
	.Positive_dir = !ALT_POS_DIR,
	.Position = {
				.direction = true,
				.lastStepsRead = 0,
				.stepPosition = 0,
				.angularPosition = 0,
				},

	//Low precision stepper motors
	.StepsCounter = 0,
	.StepsTarget = 0,
	.TicksPerStep = 0,
	.TickCounter = 0,
};

StepperMotor_t AZ_AxisMotor = {
	.STEP_Port = AZ_STEP_GPIO_Port,
	.STEP_Pin = AZ_STEP_Pin,
	.EN_Port = AZ_EN_GPIO_Port,
	.EN_Pin = AZ_EN_Pin,
	.DIR_Port = AZ_DIR_GPIO_Port,
	.DIR_Pin = AZ_DIR_Pin,
	//.ENDSTOP_Port = ,
	//.ENDSTOP_Pin = ,
	//.EXTI_IRQn = ,
	.stepsPerArcmin = AZ_STEPS_PER_ARCMIN,
	.enabled = false,
	.busy = false,
	.homing = false,
	.highPrecisionAxis = false,
	.homing_state = AXIS_HOMING_IDLE,
	.Positive_dir = !AZ_POS_DIR,
	.Position = {
				.direction = true,
				.lastStepsRead = 0,
				.stepPosition = 0,
				.angularPosition = 0,
				},

	//Low precision stepper motors
	.StepsCounter = 0,
	.StepsTarget = 0,
	.TicksPerStep = 0,
	.TickCounter = 0,
};

StepperMotor_t RA_AxisMotor = {
	.STEP_Port = RA_STEP_GPIO_Port,
	.STEP_Pin = RA_STEP_Pin,
	.EN_Port = RA_EN_GPIO_Port,
	.EN_Pin = RA_EN_Pin,
	.DIR_Port = RA_DIR_GPIO_Port,
	.DIR_Pin = RA_DIR_Pin,
	.ENDSTOP_Port = RA_LIM_GPIO_Port,
	.ENDSTOP_Pin = RA_LIM_Pin,
	.EXTI_IRQn = RA_LIM_EXTI_IRQn,
	.stepsPerArcmin = RA_STEPS_PER_ARCMIN,
	.enabled = false,
	.busy = false,
	.homing = false,
	.highPrecisionAxis = true,
	.homing_state = AXIS_HOMING_IDLE,
	.Positive_dir = !RA_POS_DIR,
	.Position = {
				.direction = true,
				.lastStepsRead = 0,
				.stepPosition = 0,
				.angularPosition = 0,
				},

	//High precision stepper motors
	.PWM_Timer = RA_PWM_TIM,
	.PWM_Channel = RA_PWM_CH,
	.Step_Counter_Timer = RA_STEP_COUNTER_TIM,
	.PWM_Type = PWM_OUT_N,

};

StepperMotor_t DEC_AxisMotor = {
	.STEP_Port = DEC_STEP_GPIO_Port,
	.STEP_Pin = DEC_STEP_Pin,
	.EN_Port = DEC_EN_GPIO_Port,
	.EN_Pin = DEC_EN_Pin,
	.DIR_Port = DEC_DIR_GPIO_Port,
	.DIR_Pin = DEC_DIR_Pin,
	.ENDSTOP_Port = DEC_LIM_GPIO_Port,
	.ENDSTOP_Pin = DEC_LIM_Pin,
	.EXTI_IRQn = DEC_LIM_EXTI_IRQn,
	.stepsPerArcmin = DEC_STEPS_PER_ARCMIN,
	.enabled = false,
	.busy = false,
	.homing = false,
	.highPrecisionAxis = true,
	.homing_state = AXIS_HOMING_IDLE,
	.Positive_dir = !DEC_POS_DIR,
	.Position = {
				.direction = true,
				.lastStepsRead = 0,
				.stepPosition = 0,
				.angularPosition = 0,
				},

	//High precision stepper motors
	.PWM_Timer = DEC_PWM_TIM,
	.PWM_Channel = DEC_PWM_CH,
	.Step_Counter_Timer = DEC_STEP_COUNTER_TIM,
	.PWM_Type = PWM_OUT_P,
};


void stepperInit()
{
	//Low precision timer init
	__HAL_TIM_SET_PRESCALER(STEPPER_TIMER_PTR, STEPPER_TIMER_PRESCALE-1); //1MHz base timer clock
	uint16_t lowPrecisionArr = ((CORE_FREQ/STEPPER_TIMER_PRESCALE)/STEPPER_TIMER_FREQ)-1;
	__HAL_TIM_SET_AUTORELOAD(STEPPER_TIMER_PTR, lowPrecisionArr);

	//High precision timer init
	__HAL_TIM_SET_PRESCALER(RA_PWM_TIM, (CORE_FREQ/STEPPER_TIMER_HI_FREQ)-1); //High precision gearbox RA axis base timer frequency
	__HAL_TIM_SET_PRESCALER(DEC_PWM_TIM, (CORE_FREQ/STEPPER_TIMER_HI_FREQ)-1); //High precision gearbox DEC axis base timer frequency

}

void stepperInterruptHandler()
{
    if (ALT_AxisMotor.enabled)
    {
    	stepsGenerating(&ALT_AxisMotor);
    }

    if (AZ_AxisMotor.enabled)
    {
        stepsGenerating(&AZ_AxisMotor);
    }
}

void Stepper_IT_Enable()
{
	HAL_TIM_Base_Start_IT(&htim3);
}

void Stepper_IT_Disable()
{
	HAL_TIM_Base_Stop_IT(&htim3);
}

void stepperEnable(StepperMotor_t *Axis)
{
	HAL_GPIO_WritePin(Axis->EN_Port,Axis->EN_Pin, GPIO_PIN_SET);
	if(!Axis->highPrecisionAxis)
	{
		Stepper_IT_Enable();
	}
	Axis->enabled = true;
}

void stepperDisable(StepperMotor_t *Axis)
{
	HAL_GPIO_WritePin(Axis->EN_Port,Axis->EN_Pin, GPIO_PIN_RESET);
	if(!Axis->highPrecisionAxis)
	{
		Stepper_IT_Disable();
	}
	Axis->enabled = false;
}


void stepperMove(StepperMotor_t *Axis, float angle, float speed) //angle is in arcmin, speed is in deg/s!
{
	if (!Axis || speed <= 0.0f || angle == 0.0f)
	        return;
	else
	{
		if(!Axis->enabled)
		{
			stepperEnable(Axis);
		}
		//Set direction
		if(angle > 0.0f)
		{
			Axis->Position.direction = Axis->Positive_dir; //Sets direction for position tracking
			HAL_GPIO_WritePin(Axis->DIR_Port, Axis->DIR_Pin, Axis->Positive_dir);
		}
		else
		{
			Axis->Position.direction = !Axis->Positive_dir;
			HAL_GPIO_WritePin(Axis->DIR_Port, Axis->DIR_Pin, !Axis->Positive_dir);
		}

		angle = fabsf(angle);


		if (!Axis->highPrecisionAxis) //Low precision gearbox axes - AZ and ALT
		{
			//Steps calculation
			Axis->StepsTarget = (uint32_t)ceilf(2*angle*Axis->stepsPerArcmin); //Times two because of toggle STEP pin -> twice lower steps

			//Steps per second calculation
			Axis->TicksPerStep = (uint32_t)ceilf(STEPPER_TIMER_FREQ/(2*speed*Axis->stepsPerArcmin*DEG));
			Axis->TickCounter = 0;
		}
		else if (Axis->highPrecisionAxis)
		{
			uint32_t steps = (uint32_t)ceilf(angle*Axis->stepsPerArcmin);
			uint32_t arr = (uint32_t)ceilf(STEPPER_TIMER_HI_FREQ / (speed*Axis->stepsPerArcmin*DEG));
			uint32_t ccr = arr / 2;

			//Set PWM (STEP) timer frequency
			__HAL_TIM_SET_AUTORELOAD(Axis->PWM_Timer, arr-1);
			__HAL_TIM_SET_COMPARE(Axis->PWM_Timer, Axis->PWM_Channel, ccr);

			//Set Counting timer targeted steps
			__HAL_TIM_SET_AUTORELOAD(Axis->Step_Counter_Timer, steps - 1);
			__HAL_TIM_SET_COUNTER(Axis->Step_Counter_Timer, 0);
			__HAL_TIM_CLEAR_IT(Axis->Step_Counter_Timer, TIM_IT_UPDATE);

			// Start Step counting timer (slave counter) and PWM timer (step pulse generator)
			HAL_TIM_Base_Start_IT(Axis->Step_Counter_Timer);

			if (Axis->PWM_Type == PWM_OUT_P)
			{
				HAL_TIM_PWM_Start(Axis->PWM_Timer, Axis->PWM_Channel);
			}
			else if (Axis->PWM_Type == PWM_OUT_N)
			{
				HAL_TIMEx_PWMN_Start(Axis->PWM_Timer, Axis->PWM_Channel);
			}
		}
		//Set flags
		Axis->enabled = true;
		Axis->busy = true;
	}


}

void stepperHome(StepperMotor_t *Axis, float speed, bool dir)
{
	if (!Axis || speed <= 0.0f)
		        return;
		else
		{
			if (!Axis->enabled)
			{
				stepperEnable(Axis);
			}

			if (!Axis->highPrecisionAxis)
			{
				//Steps calculation
				Axis->StepsTarget = (uint32_t)(2*90.0f*Axis->stepsPerArcmin); //Times two because of toggle STEP pin -> twice lower steps
				//Steps per second calculation
				Axis->TicksPerStep = (uint32_t)(STEPPER_TIMER_FREQ/(2*speed*Axis->stepsPerArcmin));
				Axis->TickCounter = 0;
			}

			else
			{
				return;
			}

			Axis->homing = true;
			Axis->busy = true;
			HAL_GPIO_WritePin(Axis->DIR_Port, Axis->DIR_Pin, dir);
		}
}

void stepperStop(StepperMotor_t *Axis)
{

	if (Axis->highPrecisionAxis)
	{
		//Stop PWM timer
		if (Axis->PWM_Type == PWM_OUT_P)
		{
			HAL_TIM_PWM_Stop(Axis->PWM_Timer, Axis->PWM_Channel);
		}
		else if (Axis->PWM_Type == PWM_OUT_N)
		{
			HAL_TIMEx_PWMN_Stop(Axis->PWM_Timer, Axis->PWM_Channel);
		}
		//Stop STEP counting timer
		HAL_TIM_Base_Stop_IT(Axis->Step_Counter_Timer);

		Axis->Position.lastStepsRead = 0;
		}
	else if (!Axis->highPrecisionAxis)
	{
		Axis->Position.lastStepsRead = 0;

		//Reset StepsCounter and StepsTarget
		Axis->StepsTarget = 0;
		Axis->StepsCounter = 0;
	}
	//Update axis position based on the StepsTarget
	updatePosition(Axis);

	//stepperDisable(Axis);
	Axis->busy = false;

}

void stepsGenerating(StepperMotor_t *Axis)
{
	if (Axis->StepsCounter >= Axis->StepsTarget)
	        {
				Axis->busy= false;
				//Lower the motor current
				//TBD

				//Update axis position based on the StepsTarget
				updatePosition(Axis);
				Axis->Position.lastStepsRead = 0;

				//reset StepsCounter and StepsTarget
				Axis->StepsCounter = 0;
				Axis->StepsTarget = 0;
	            return;
	        }
	        if (Axis->TickCounter == 0)
	        {
	            // Generate one step pulse
	            HAL_GPIO_TogglePin(Axis->STEP_Port, Axis->STEP_Pin);

	            Axis->StepsCounter++;
	            Axis->TickCounter = Axis->TicksPerStep;
	        }
	        else
	        {
	        	Axis->TickCounter--;
	        }
}


void Stepper_nSleep(bool n_sleep)
{
	HAL_GPIO_WritePin(STEP_SLEEP_n_GPIO_Port,STEP_SLEEP_n_Pin, n_sleep); //1 = enabled
}

