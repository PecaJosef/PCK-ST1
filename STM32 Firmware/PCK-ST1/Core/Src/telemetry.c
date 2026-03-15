/*
 * telemetry.c
 *
 *  Created on: Mar 11, 2026
 *      Author: pecka
 */
#include "telemetry.h"
#include "stepper.h"
#include "cmd_handler.h"

#define TELEMETRY_LED_PERIOD 250

typedef enum {
	POWER_INPUT_DC,
	POWER_INPUT_USB,
	POWER_INPUT_DC_LOW,
	POWER_INPUT_USB_LOW,
} PowerInput_t;


static void checkPowerInput();

static volatile uint32_t adc_raw = 0;

void telemetryHandler()
{
	static uint32_t telemetryLastTicks = 0;

	if (HAL_GetTick() - telemetryLastTicks < TELEMETRY_PERIOD)
	{
		return;
	}
	telemetryLastTicks = HAL_GetTick();

	float voltage = getVoltage();

	char dbgbuff[32];
	sprintf(dbgbuff,"Voltage: %f\r\n", voltage);
	uartSend(PC_UART_SRC, dbgbuff);

	checkPowerInput();
}


void adcVoltageInit()
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_raw, 1);
}

float getVoltage(void)
{
    float adcVoltage    = (adc_raw * VREF) / ADC_MAX;
    float inputVoltage  = adcVoltage * ((DIVIDER_R_UP + DIVIDER_R_DOWN) / DIVIDER_R_DOWN);
    return inputVoltage;
}

static void checkPowerInput()
{
	static PowerInput_t pwrInput;
	static uint32_t lastBlinkTick = 0;

	if (HAL_GPIO_ReadPin(PWR_STATE_GPIO_Port, PWR_STATE_Pin))
	{
		if(getVoltage() < 9.0f)
		{
			pwrInput = POWER_INPUT_USB_LOW;
		}
		else
		{
			pwrInput = POWER_INPUT_USB;
		}
	}
	else
	{
		if(getVoltage() < 9.0f)
		{
			pwrInput = POWER_INPUT_DC_LOW;
		}
		else
		{
			pwrInput = POWER_INPUT_DC;
		}
	}

	switch(pwrInput)
	{
	case POWER_INPUT_DC:
		HAL_GPIO_WritePin(LED_DC_GPIO_Port, LED_DC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_USB_GPIO_Port, LED_USB_Pin, GPIO_PIN_RESET);
		break;
	case POWER_INPUT_USB:
		HAL_GPIO_WritePin(LED_DC_GPIO_Port, LED_DC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_USB_GPIO_Port, LED_USB_Pin, GPIO_PIN_SET);
		break;
	case POWER_INPUT_DC_LOW:
		HAL_GPIO_WritePin(LED_USB_GPIO_Port, LED_USB_Pin, GPIO_PIN_RESET);
		if ((HAL_GetTick() - lastBlinkTick) >= TELEMETRY_LED_PERIOD)
		{
			HAL_GPIO_TogglePin(LED_DC_GPIO_Port, LED_DC_Pin);
			lastBlinkTick = HAL_GetTick();
		}
		break;
	case POWER_INPUT_USB_LOW:
		HAL_GPIO_WritePin(LED_DC_GPIO_Port, LED_DC_Pin, GPIO_PIN_RESET);
		if ((HAL_GetTick() - lastBlinkTick) >= TELEMETRY_LED_PERIOD)
		{
			HAL_GPIO_TogglePin(LED_USB_GPIO_Port, LED_USB_Pin);
			lastBlinkTick = HAL_GetTick();
		}
		break;
	}


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

