/*
 * stepper.c
 *
 *  Created on: Aug 5, 2025
 *      Author: pecka
 */
#include "stepper.h"
#include "usbd_cdc_if.h"
#include "math.h"


StepperMotor_t ALT_AxisMotor = {
	.STEP_Port = ALT_STEP_GPIO_Port,
	.STEP_Pin = ALT_STEP_Pin,
	.EN_Port = ALT_EN_GPIO_Port,
	.EN_Pin = ALT_EN_Pin,
	.DIR_Port = ALT_DIR_GPIO_Port,
	.DIR_Pin = ALT_DIR_Pin,
	.ENDSTOP_Port = ALT_LIM_GPIO_Port,
	.ENDSTOP_Pin = ALT_LIM_Pin,
	.EXTI_IRQn = EXTI2_IRQn,
	.Steps_per_deg = ALT_STEP_PER_DEG,
	.enabled = false,
	.busy = false,
	.homing = false,
	.High_precision = false,
	.homing_state = AXIS_HOMING_IDLE,
	.Position = -1,
	.Positive_dir = !ALT_POS_DIR,

	//Low precision stepper motors
	.Steps_remaining = 0,
	.Step_interval_ticks = 0,
	.Tick_counter = 0,
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
	.Steps_per_deg = AZ_STEP_PER_DEG,
	.enabled = false,
	.busy = false,
	.homing = false,
	.High_precision = false,
	.homing_state = AXIS_HOMING_IDLE,
	.Position = -1,
	.Positive_dir = !AZ_POS_DIR,

	//Low precision stepper motors
	.Steps_remaining = 0,
	.Step_interval_ticks = 0,
	.Tick_counter = 0,
};

StepperMotor_t RA_AxisMotor = {
	.STEP_Port = RA_STEP_GPIO_Port,
	.STEP_Pin = RA_STEP_Pin,
	.EN_Port = RA_EN_GPIO_Port,
	.EN_Pin = RA_EN_Pin,
	.DIR_Port = RA_DIR_GPIO_Port,
	.DIR_Pin = RA_DIR_Pin,
	//.ENDSTOP_Port = ,
	//.ENDSTOP_Pin = ,
	//.EXTI_IRQn = ,
	.Steps_per_deg = RA_STEP_PER_DEG,
	.enabled = false,
	.busy = false,
	.homing = false,
	.High_precision = true,
	.homing_state = AXIS_HOMING_IDLE,
	.Position = -1,
	.Positive_dir = !RA_POS_DIR,

	//High precision stepper motors
	.PWM_Timer = RA_PWM_TIM,
	.PWM_Channel = RA_PWM_CH,
	.Step_Counter_Timer = RA_STEP_TIM,
	.PWM_Type = PWM_OUT_N,

};

StepperMotor_t DEC_AxisMotor = {
	.STEP_Port = DEC_STEP_GPIO_Port,
	.STEP_Pin = DEC_STEP_Pin,
	.EN_Port = DEC_EN_GPIO_Port,
	.EN_Pin = DEC_EN_Pin,
	.DIR_Port = DEC_DIR_GPIO_Port,
	.DIR_Pin = DEC_DIR_Pin,
	//.ENDSTOP_Port = ,
	//.ENDSTOP_Pin = ,
	//.EXTI_IRQn = ,
	.Steps_per_deg = DEC_STEP_PER_DEG,
	.enabled = false,
	.busy = false,
	.homing = false,
	.High_precision = true,
	.homing_state = AXIS_HOMING_IDLE,
	.Position = -1,
	.Positive_dir = !DEC_POS_DIR,

	//High precision stepper motors
	.PWM_Timer = DEC_PWM_TIM,
	.PWM_Channel = DEC_PWM_CH,
	.Step_Counter_Timer = DEC_STEP_TIM,
	.PWM_Type = PWM_OUT_P,
};


void Stepper_IT_Handeler()
{
    if (ALT_AxisMotor.enabled)
    {
    	STEP_Generating(&ALT_AxisMotor);
    }

    if (AZ_AxisMotor.enabled)
    {
        STEP_Generating(&AZ_AxisMotor);
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
	if(!Axis->High_precision)
	{
		Stepper_IT_Enable();
	}
	Axis->enabled = true;
}

void stepperDisable(StepperMotor_t *Axis)
{
	HAL_GPIO_WritePin(Axis->EN_Port,Axis->EN_Pin, GPIO_PIN_RESET);
	if(!Axis->High_precision)
	{
		Stepper_IT_Disable();
	}
	Axis->enabled = false;
}


////!!!!!!!Remove the bool dir from the function. Direction is set based on the angle (negative/positive)
void stepperMove(StepperMotor_t *Axis, float angle, float speed) //Speed is in deg/s
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
			HAL_GPIO_WritePin(Axis->DIR_Port, Axis->DIR_Pin, Axis->Positive_dir);
		}
		else
		{
			HAL_GPIO_WritePin(Axis->DIR_Port, Axis->DIR_Pin, !Axis->Positive_dir);
		}

		angle = fabsf(angle);


		if (!Axis->High_precision)
		{
			//Steps calculation
			Axis->Steps_remaining = (uint32_t)(2*angle*Axis->Steps_per_deg); //Times two because of toggle STEP pin -> twice lower steps
			//Steps per second calculation
			Axis->Step_interval_ticks = (uint32_t)(STEPPER_TIMER_FREQ/(2*speed*Axis->Steps_per_deg));
			Axis->Tick_counter = 0;
		}
		else if (Axis->High_precision)
		{
			uint32_t steps = (uint32_t)(angle*Axis->Steps_per_deg);
			uint32_t arr = (uint32_t)(STEPPER_TIMER_HI_FREQ / (speed*Axis->Steps_per_deg));
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

			if (!Axis->High_precision)
			{
				//Steps calculation
				Axis->Steps_remaining = (uint32_t)(2*90.0f*Axis->Steps_per_deg); //Times two because of toggle STEP pin -> twice lower steps
				//Steps per second calculation
				Axis->Step_interval_ticks = (uint32_t)(STEPPER_TIMER_FREQ/(2*speed*Axis->Steps_per_deg));
				Axis->Tick_counter = 0;
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

	if (Axis->High_precision)
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
		}
	else if (!Axis->High_precision)
	{
		Axis->Steps_remaining = 0;
	}

	//stepperDisable(Axis);
	Axis->busy = false;

}

void STEP_Generating(StepperMotor_t *Axis)
{
	if (Axis->Steps_remaining == 0)
	        {
				Axis->busy= false;
				//stepperDisable(Axis);
	            return;
	        }
	        if (Axis->Tick_counter == 0)
	        {
	            // Generate one step pulse
	            HAL_GPIO_TogglePin(Axis->STEP_Port, Axis->STEP_Pin);

	            Axis->Steps_remaining--;
	            Axis->Tick_counter = Axis->Step_interval_ticks;
	        }
	        else
	        {
	        	Axis->Tick_counter--;
	        }
}


void Stepper_nSleep(bool n_sleep)
{
	HAL_GPIO_WritePin(STEP_SLEEP_n_GPIO_Port,STEP_SLEEP_n_Pin, n_sleep); //1 = enabled
}




/*
void Stepper_Move_RA(float angle, float speed, bool dir)
{
	uint32_t steps = (uint32_t)(angle*RA_STEP_PER_DEG);

	uint32_t arr = (uint32_t)(STEPPER_TIMER_HI_FREQ / (speed*RA_STEP_PER_DEG));
    uint32_t ccr = arr / 2;

    // DIR
    HAL_GPIO_WritePin(RA_DIR_GPIO_Port, RA_DIR_Pin, dir);

    // Set TIM8 frequency
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr-1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);

    // Set TIM5 target steps
    __HAL_TIM_SET_AUTORELOAD(&htim2, steps - 1);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

    // Start TIM5 (slave counter) and TIM8 (pulse generator)
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}
*/

