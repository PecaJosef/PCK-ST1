/*
 * telemetry.h
 *
 *  Created on: Mar 11, 2026
 *      Author: pecka
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "stepper.h"

#define DIVIDER_R_UP 100000.0f
#define DIVIDER_R_DOWN 22000.0f
#define ADC_MAX 4095.0f

#define TELEMETRY_PERIOD 250

extern ADC_HandleTypeDef hadc1;

void telemetryHandler();

void adcVoltageInit();

float getVoltage(void);

void updatePosition(StepperMotor_t *Axis);

void getTelemetry();

void getFullTelemetry();

#endif /* INC_TELEMETRY_H_ */
