/*
 * control_loop.h
 *
 *  Created on: Aug 25, 2025
 *      Author: pecka
 */

#ifndef INC_CONTROL_LOOP_H_
#define INC_CONTROL_LOOP_H_

#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "gps.h"
#include "button.h"
#include "wmm.h"
#include "stepper.h"

typedef enum {
	LOW_POWER_IDLE, //Waiting for button press to power up, initial state
	HOMING, //Axis homing
	WARMING_UP, //Powering up RPi, waiting for "OK" from RPi, waiting for GPS fix, calculating magnetic declination
	ALIGNING, //Pointing to NCP, communication with RPi - taking photo
	IDLE, //Not moving, waiting for action (commands), can be powered off while in idle
	AXIS_MOVING, //Moving with any axis - GOTO
	GUIDING, //Moving RA axis to counter Earth's rotation
	FAULT, //Error state - waiting for reset
}ControlState_t;

typedef struct {
	float elevation_angle;
	float declination;
}Align_t;

typedef struct {
	bool rpiStarting;
	bool rpiRunning;
}Status_t;

typedef struct{
	StepperMotor_t *Axis;
	float angle;
	float speed;
	bool moveRequested;
}MoveRequest_t;


extern ControlState_t controlState;
extern ControlState_t prevControlState;

extern Align_t Align_data;

extern MoveRequest_t AZ_MoveRequest;
extern MoveRequest_t EL_MoveRequest;
extern MoveRequest_t DEC_MoveRequest;
extern MoveRequest_t RA_MoveRequest;

void Control_loop();

void handleMoving();

#endif /* INC_CONTROL_LOOP_H_ */
