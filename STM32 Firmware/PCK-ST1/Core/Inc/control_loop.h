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

typedef enum {
	LOW_POWER_IDLE, //Waiting for button press to power up, initial state
	WARMING_UP, //Powering up RPi, waiting for "OK" from RPi, waiting for GPS fix, calculating magnetic declination
	HOMING, //Axis homing
	ALIGNING, //Pointing to NCP, communication with RPi - taking photo
	AXIS_MOVING, //Moving with any axis - GOTO
	GUIDING, //Moving RA axis to counter Earth's rotation
	IDLE, //Not moving, waiting for action (commands), can be powered off while in idle
	FAULT, //Error state - waiting for reset
}ControlState_t;

typedef struct {
	float elevation_angle;
	float declination;
}Align_t;

extern ControlState_t controlState;

extern Align_t Align_data;

void Control_loop();

#endif /* INC_CONTROL_LOOP_H_ */
