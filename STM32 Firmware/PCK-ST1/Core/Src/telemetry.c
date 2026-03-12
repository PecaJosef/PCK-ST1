/*
 * telemetry.c
 *
 *  Created on: Mar 11, 2026
 *      Author: pecka
 */
#include "telemetry.h"
#include "stepper.h"


float getVoltage(void)
{
    uint32_t adcRaw = 0;
    float adcVoltage = 0.0f;
    float inputVoltage = 0.0f;

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {

    	adcRaw = HAL_ADC_GetValue(&hadc1);

    	adcVoltage = (adcRaw * VREF) / ADC_MAX;

    	inputVoltage = adcVoltage * ((DIVIDER_R_UP + DIVIDER_R_DOWN) / DIVIDER_R_DOWN);
    }

    HAL_ADC_Stop(&hadc1);
    return inputVoltage;
}


//Updates the step position of selected axis
void updatePosition(StepperMotor_t *Axis)
{
	if (!Axis->highPrecisionAxis)
	{

		//HANDLETHE RETURN TO ZERO AFTER A MOVE
		int32_t deltaSteps = Axis->StepsCounter - Axis->Position.lastStepsRead;

		if (Axis->Position.direction == true) //If the direction of rotation is positive (true) increase the stepPostition
		{
			Axis->Position.stepPosition += deltaSteps;
		}
		else
		{
			Axis->Position.stepPosition -= deltaSteps;
		}

	}
	else if (Axis->highPrecisionAxis)
	{

	}
}

