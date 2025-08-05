/*
 * stepper.c
 *
 *  Created on: Aug 5, 2025
 *      Author: pecka
 */
#include "stepper.h"
#include "stm32l4xx_hal.h"
#include "main.h"

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


void Stepper_IT_Handeler()
{
    if (EL_Axis_motor.enabled)
    {
        if (EL_Axis_motor.Steps_remaining == 0)
        {
            EL_Axis_motor.enabled= false;
            return;
        }
        if (EL_Axis_motor.Tick_counter == 0)
        {
            // Generate one step pulse
            HAL_GPIO_TogglePin(EL_Axis_motor.STEP_Port, EL_Axis_motor.STEP_Pin);

            EL_Axis_motor.Steps_remaining--;
            EL_Axis_motor.Tick_counter = EL_Axis_motor.Step_interval_ticks;
        }
        else
        {
            EL_Axis_motor.Tick_counter--;
        }
    }
	//HAL_GPIO_TogglePin(EL_STEP_GPIO_Port, EL_STEP_Pin);

}

void Stepper_IT_Enable()
{
	HAL_TIM_Base_Start_IT(&htim3);
}

void Stepper_Enable(Stepper_motor *Axis)
{
	HAL_GPIO_WritePin(Axis->EN_Port,Axis->EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_SLEEP_n_GPIO_Port,STEP_SLEEP_n_Pin, GPIO_PIN_SET);
}

void Stepper_Move(Stepper_motor *Axis, float angle, float speed, bool direction) //Speed is in deg/s
{
	if (!Axis || speed <= 0.0f)
	        return;

	//Steps calculation
	Axis->Steps_remaining = (uint32_t)(2*angle*Axis->Steps_per_deg); //Times two because of toggle STEP pin -> twice lower steps
	//Steps per second calculation
	Axis->Step_interval_ticks = (uint32_t)(STEPPER_TIMER_FREQ/(2*speed*Axis->Steps_per_deg));
	Axis->Tick_counter = 0;
	Axis->enabled = true;
	HAL_GPIO_WritePin(Axis->DIR_Port, Axis->DIR_Pin, direction);
}




