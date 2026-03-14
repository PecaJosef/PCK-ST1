/*
 * telemetry.c
 *
 *  Created on: Mar 11, 2026
 *      Author: pecka
 */
#include "telemetry.h"
#include "stepper.h"


void telemetryHandler()
{





}





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
	int32_t deltaSteps = 0;
	if (!Axis->highPrecisionAxis)
	{

		deltaSteps = Axis->StepsCounter - Axis->Position.lastStepsRead;

	}
	else if (Axis->highPrecisionAxis)
	{
		deltaSteps = __HAL_TIM_GET_COUNTER(Axis->Step_Counter_Timer);
	}

	if (Axis->Position.direction == true) //If the direction of rotation is positive (true) increase the stepPostition
	{
		Axis->Position.stepPosition += deltaSteps;
	}
	else
	{
		Axis->Position.stepPosition -= deltaSteps;
	}
}

