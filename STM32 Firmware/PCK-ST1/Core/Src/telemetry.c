/*
 * telemetry.c
 *
 *  Created on: Mar 11, 2026
 *      Author: pecka
 */
#include "telemetry.h"
#include "stepper.h"
#include "cmd_handler.h"
#include "control_loop.h"
#include "astro.h"

#define TELEMETRY_LED_PERIOD 200

typedef enum {
	POWER_INPUT_DC,
	POWER_INPUT_USB,
	POWER_INPUT_DC_LOW,
	POWER_INPUT_USB_LOW,
} PowerInput_t;


static void checkPowerInput();

//DMA adc_buffer
static volatile uint16_t adc_buffer[2]; //[0] = VREFINT, [1] = CH4 ADC data


//Main telemetry handler
void telemetryHandler()
{
	static uint32_t telemetryLastTicks = 0;

	if (HAL_GetTick() - telemetryLastTicks < TELEMETRY_PERIOD)
	{
		return;
	}
	telemetryLastTicks = HAL_GetTick();

	updatePosition(&AZ_AxisMotor);
	updatePosition(&ALT_AxisMotor);
	updatePosition(&RA_AxisMotor);
	updatePosition(&DEC_AxisMotor);

	checkPowerInput();

}

void getTelemetry()
{
	float voltage = getVoltage();

	char dbgbuff[128];
	sprintf(dbgbuff,"AZ Pos:  %.3f\r\nALT Pos: %.3f\r\nRA Pos:  %.3f\r\nDEC Pos: %.3f\r\nVoltage: %.2f\r\n", AZ_AxisMotor.Position.angularPosition, ALT_AxisMotor.Position.angularPosition, RA_AxisMotor.Position.angularPosition, DEC_AxisMotor.Position.angularPosition, voltage);
	uartSend(PC_UART_SRC, dbgbuff);
}

void getFullTelemetry()
{
	float voltage = getVoltage();

	char dbgbuff[128];
	//Full telemetry in format
	//$TEL:AZpos:ALTpos:RApos:DECpos:Voltage:RAcoordinates:DECcoordinates:Homed:Calibrated:gpsOK:gpsFixed:rpiReady:polarAligned:tracking:moveEnabled:fault
	sprintf(dbgbuff,"#TEL:%.3f:%.3f:%.3f:%.3f:%.3f:%.5f:%.5f:%d:%d:%d:%d:%d:%d:%d:%d:%d\r\n",
			AZ_AxisMotor.Position.angularPosition,
			ALT_AxisMotor.Position.angularPosition,
			RA_AxisMotor.Position.angularPosition,
			DEC_AxisMotor.Position.angularPosition,
			voltage,
			coordinatesRaDec.raCoordinatesAngle,
			coordinatesRaDec.decCoordinatesAngle,
			statusFlags.homed,
			statusFlags.calibrated,
			statusFlags.gpsOK,
			statusFlags.gpsFixed,
			statusFlags.rpiRDY,
			statusFlags.polarAligned,
			statusFlags.tracking,
			statusFlags.moveEnabled,
			statusFlags.fault
			);

	uartSend(PC_UART_SRC, dbgbuff);
}


void adcVoltageInit()
{
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buffer, 2);
}

float getVoltage(void)
{
    uint16_t vref_raw = adc_buffer[0];
    uint16_t adc_raw = adc_buffer[1];

    //Get real vref voltage value
    float vref_real = 3.0f * ((float)*VREFINT_CAL_ADDR / (float)vref_raw);
	float adcVoltage = (adc_raw * vref_real) / ADC_MAX;

    float inputVoltage = adcVoltage * ((DIVIDER_R_UP + DIVIDER_R_DOWN) / DIVIDER_R_DOWN);
    return inputVoltage;
}

static void checkPowerInput()
{
	static PowerInput_t pwrInput;
	static uint32_t lastBlinkTick = 0;

	if (HAL_GPIO_ReadPin(PWR_STATE_GPIO_Port, PWR_STATE_Pin))
	{
		if(getVoltage() < LOW_VOLTAGE)
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

//Updates the step position of selected axis and RA DEC coordinates if the PCK-ST1 is polar aligned
void updatePosition(StepperMotor_t *Axis)
{
	int32_t deltaSteps = 0;
	uint32_t currentStepCount = 0;
	if (!Axis->highPrecisionAxis)
	{
		currentStepCount = Axis->StepsCounter;
		deltaSteps = currentStepCount - Axis->Position.lastStepsRead;
		Axis->Position.lastStepsRead = currentStepCount;

	}
	else if (Axis->highPrecisionAxis)
	{
		currentStepCount = __HAL_TIM_GET_COUNTER(Axis->Step_Counter_Timer);
		deltaSteps = currentStepCount - Axis->Position.lastStepsRead;
		Axis->Position.lastStepsRead = currentStepCount;
	}

	if (Axis->Position.direction != Axis->Positive_dir) //If the direction of rotation is in the negative direction make the deltaSteps negative
	{
		deltaSteps = -deltaSteps;
	}

	Axis->Position.stepPosition += deltaSteps;

	//The low precision axes do double the steps for the same angle as the high precision axes
	if (!Axis->highPrecisionAxis)
	{
		//Anglular position divided by 2
		Axis->Position.angularPosition = ((float)Axis->Position.stepPosition / (Axis->stepsPerArcmin*DEG))/2.0f;
	}
	else
	{
		Axis->Position.angularPosition = (float)Axis->Position.stepPosition / (Axis->stepsPerArcmin*DEG);
	}

	//Update the RA and DEC celestial coordinates
	if (Axis->AxisID == RA || Axis->AxisID == DEC)
	{
		updateRaDec(Axis, deltaSteps);
	}

}

