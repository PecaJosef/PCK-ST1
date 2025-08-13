/*
 * stepper.c
 *
 *  Created on: Aug 5, 2025
 *      Author: pecka
 */
#include "stepper.h"
#include "stm32l4xx_hal.h"
#include "main.h"

#include "usbd_cdc_if.h"

Stepper_motor EL_Axis_motor = {
	.STEP_Port = EL_STEP_GPIO_Port,
	.STEP_Pin = EL_STEP_Pin,
	.EN_Port = EL_EN_GPIO_Port,
	.EN_Pin = EL_EN_Pin,
	.DIR_Port = EL_DIR_GPIO_Port,
	.DIR_Pin = EL_DIR_Pin,
	.Steps_remaining = 0,
	.Step_interval_ticks = 0,
	.Tick_counter = 0,
	.Steps_per_deg = EL_STEP_PER_DEG,
	.enabled = false,
};

Stepper_motor AZ_Axis_motor = {
	.STEP_Port = AZ_STEP_GPIO_Port,
	.STEP_Pin = AZ_STEP_Pin,
	.EN_Port = AZ_EN_GPIO_Port,
	.EN_Pin = AZ_EN_Pin,
	.DIR_Port = AZ_DIR_GPIO_Port,
	.DIR_Pin = AZ_DIR_Pin,
	.Steps_remaining = 0,
	.Step_interval_ticks = 0,
	.Tick_counter = 0,
	.Steps_per_deg = AZ_STEP_PER_DEG,
	.enabled = false,
};

Stepper_motor RA_Axis_motor = {
	.STEP_Port = RA_STEP_GPIO_Port,
	.STEP_Pin = RA_STEP_Pin,
	.EN_Port = RA_EN_GPIO_Port,
	.EN_Pin = RA_EN_Pin,
	.DIR_Port = RA_DIR_GPIO_Port,
	.DIR_Pin = RA_DIR_Pin,
	.Steps_remaining = 0,
	.Step_interval_ticks = 0,
	.Tick_counter = 0,
	.Steps_per_deg = RA_STEP_PER_DEG,
	.enabled = false,
};

Stepper_motor DEC_Axis_motor = {
	.STEP_Port = DEC_STEP_GPIO_Port,
	.STEP_Pin = DEC_STEP_Pin,
	.EN_Port = DEC_EN_GPIO_Port,
	.EN_Pin = DEC_EN_Pin,
	.DIR_Port = DEC_DIR_GPIO_Port,
	.DIR_Pin = DEC_DIR_Pin,
	.Steps_remaining = 0,
	.Step_interval_ticks = 0,
	.Tick_counter = 0,
	.Steps_per_deg = DEC_STEP_PER_DEG,
	.enabled = false,
};


void Stepper_IT_Handeler()
{
    if (EL_Axis_motor.enabled)
    {
    	STEP_Generating(&EL_Axis_motor);
    }


    if (AZ_Axis_motor.enabled)
    {
        STEP_Generating(&AZ_Axis_motor);
    }
/*
    if (RA_Axis_motor.enabled)
	{
		STEP_Generating(&RA_Axis_motor);
	}

    if (DEC_Axis_motor.enabled)
    {
    	STEP_Generating(&DEC_Axis_motor);
    }
*/
}

void Stepper_IT_Enable()
{
	HAL_TIM_Base_Start_IT(&htim3);
}

void Stepper_Enable(Stepper_motor *Axis)
{
	HAL_GPIO_WritePin(Axis->EN_Port,Axis->EN_Pin, GPIO_PIN_SET);
}

void Stepper_Move(Stepper_motor *Axis, float angle, float speed, bool dir) //Speed is in deg/s
{
	if (!Axis || speed <= 0.0f)
	        return;

	//Steps calculation
	Axis->Steps_remaining = (uint32_t)(2*angle*Axis->Steps_per_deg); //Times two because of toggle STEP pin -> twice lower steps
	//Steps per second calculation
	Axis->Step_interval_ticks = (uint32_t)(STEPPER_TIMER_FREQ/(2*speed*Axis->Steps_per_deg));
	Axis->Tick_counter = 0;
	Axis->enabled = true;
	HAL_GPIO_WritePin(Axis->DIR_Port, Axis->DIR_Pin, dir);
}

void Stepper_Move_DEC(float angle, float speed, bool dir)
{
    uint32_t steps = (uint32_t)(angle*DEC_STEP_PER_DEG);

	uint32_t arr = (uint32_t)(STEPPER_TIMER_HI_FREQ / (speed*DEC_STEP_PER_DEG));
    uint32_t ccr = arr / 2;

    // DIR
    HAL_GPIO_WritePin(DEC_DIR_GPIO_Port, DEC_DIR_Pin, dir);

    // Set TIM8 frequency
    __HAL_TIM_SET_AUTORELOAD(&htim8, arr-1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, ccr);

    // Set TIM5 target steps
    __HAL_TIM_SET_AUTORELOAD(&htim5, steps - 1);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);

    // Start TIM5 (slave counter) and TIM8 (pulse generator)
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}

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

void STEP_Generating(Stepper_motor *Axis)
{
	if (Axis->Steps_remaining == 0)
	        {
				Axis->enabled= false;
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



