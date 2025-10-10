/*
 * homing.c
 *
 *  Created on: Sep 5, 2025
 *      Author: pecka
 */

#include "homing.h"


void handleHoming()
{
	static bool homed = false;

	    if (!homed) {
	        // Kick off homing for all axes
	        Axis_Home_Start(&EL_Axis_motor, 10.0f, 5.0f);
	        //Axis_Home_Start(&AZ_Axis_motor, 200.0f, 50.0f);
	        //Axis_Home_Start(&RA_Axis_motor, 200.0f, 50.0f);
	        //Axis_Home_Start(&DEC_Axis_motor, 200.0f, 50.0f);

	        homed = true;
	    }

    // Update all axes in parallel
    Axis_Homing_Update(&EL_Axis_motor, 10.0f, 5.0f);
    Axis_Homing_Update(&AZ_Axis_motor, 10.0f, 5.0f);
    Axis_Homing_Update(&RA_Axis_motor, 5.0f, 1.0f);
    Axis_Homing_Update(&DEC_Axis_motor, 5.0f, 1.0f);

    // Check if all done
    if (!EL_Axis_motor.homing && !AZ_Axis_motor.homing && !RA_Axis_motor.homing && !DEC_Axis_motor.homing)
    {
    	homed = false;
    	controlState = WARMING_UP;
    }
}

void Axis_Home_Start(Stepper_motor *Axis, float coarseSpeed, float fineSpeed)
{
    if (!Axis) return;
    if (!Axis->enabled) Stepper_Enable(Axis);

    Axis->homing = true;
    __HAL_GPIO_EXTI_CLEAR_IT(Axis->ENDSTOP_Pin);
    HAL_NVIC_EnableIRQ(Axis->EXTI_IRQn);

    if (HAL_GPIO_ReadPin(Axis->ENDSTOP_Port, Axis->ENDSTOP_Pin) == GPIO_PIN_RESET)
    {
        // Endstop already pressed → skip coarse, go to backoff
        Axis->homing_state = AXIS_HOMING_BACKOFF;
        Stepper_Move(Axis, 5.0f, fineSpeed, !Axis->Homing_dir);
    }
    else
    {
        // Normal coarse move towards switch
        Axis->homing_state = AXIS_HOMING_COARSE;
        Stepper_Move(Axis, 180.0f, coarseSpeed, Axis->Homing_dir); // 180° is just "long enough"
    }
}

void Axis_Homing_Update(Stepper_motor *Axis, float coarseSpeed, float fineSpeed)
{
    if (!Axis->homing) return;

    if (!Axis->busy) {  // Move finished without trigger
        switch (Axis->homing_state)
        {
        case AXIS_HOMING_BACKOFF:
            // Backoff done → now fine approach
        	__HAL_GPIO_EXTI_CLEAR_IT(Axis->ENDSTOP_Pin);
        	HAL_NVIC_EnableIRQ(Axis->EXTI_IRQn);

            Axis->homing_state = AXIS_HOMING_FINE;
            Stepper_Move(Axis, 10.0f, fineSpeed, Axis->Homing_dir);
            break;

        case AXIS_HOMING_FINE:
            // Should have been triggered by interrupt. If not, treat as done anyway
            Axis->homing_state = AXIS_HOMING_DONE;
            Axis->homing = false;
            Axis->Position = 0;
            break;

        default:
            break;
        }
    }
}

void Endstop_Reached(Stepper_motor *Axis)
{
	Stepper_Stop(Axis);

	if (!Axis->homing) return;

	switch (Axis->homing_state) {
	case AXIS_HOMING_COARSE:
		// First hit → back off
		Axis->homing_state = AXIS_HOMING_BACKOFF;
		Stepper_Move(Axis, 5.0f, 5.0f, !Axis->Homing_dir);
		break;

	case AXIS_HOMING_FINE:
		// Final hit → success
		Axis->homing_state = AXIS_HOMING_DONE;
		Axis->homing = false;
		Axis->Position = 0;
		break;

	default:
		break;
	}
}

