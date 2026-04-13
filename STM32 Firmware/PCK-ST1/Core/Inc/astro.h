/*
 * astro.h
 *
 *  Created on: Apr 6, 2026
 *      Author: pecka
 */

#ifndef INC_ASTRO_H_
#define INC_ASTRO_H_

#define TRACKING_TIMER_FREQ 3200000
#define TRACKING_PRESCALER (CORE_FREQ/TRACKING_TIMER_FREQ)
#define TRACKING_DIR 1

#define SECONDS_IN_DAY 86164
#define TRACKING_FREQ ((STEPS_PER_REV*RA_GEAR_RATIO*RA_MICROSTEPPING)/SECONDS_IN_DAY)
#define SECONDS_PER_DEGREE (SECONDS_IN_DAY/360.0f)
#define DEG_TO_HOURS 15.041f

#define GOTO_RA_MIN -45.0f
#define GOTO_RA_MAX 135.0f
#define RA_GOTO_SPEED 2.0f
#define DEC_GOTO_SPEED 5.0f

#include "control_loop.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include "gps.h"
#include "button.h"
#include "stepper.h"
#include "cmd_handler.h"
#include "telemetry.h"

typedef struct{
	float raAngle;
	float decAngle;
	bool gotoRequested;
}gotoRequest_t;

typedef struct{
	float raCoordinatesHours; //Regular RA coordinates in hours
	float raCoordinatesAngle; //RA coordinates in degrees
	float decCoordinatesAngle; //Regular DEC coordinates in degrees
	uint32_t lastRaUpdate;
	bool trackingPosition;
}CoordinatesRaDec_t;


extern gotoRequest_t gotoRequest;
extern CoordinatesRaDec_t coordinatesRaDec;

void trackingStart(StepperMotor_t *Axis);

void trackingStop(StepperMotor_t *Axis);

void updateRaDec(StepperMotor_t *Axis, int32_t deltaSteps);

void gotoRaDec(float RAcoord, float DECcoord);

#endif /* INC_ASTRO_H_ */
