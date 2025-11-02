/*
 * control_loop.c
 *
 *  Created on: Aug 25, 2025
 *      Author: pecka
 */
/*
 * Control states should be as follows during regular run
 * LOW_POWER_IDLE
* 		- waits for power button power up sequence (short press, long press)
 * 		- after powering sequence LEDs start to blink
 * 		- another button press initiates homing -> HOMING state, powers up RPi
 *
 * HOMING
 * 		- homes all axis
 * 		- after homing is performed software jumps to WARMING_UP state
 *
 * WARMING_UP
 * 		- checks if GPS and magnetometer are operational (if not -> ERROR state)
 * 			- if magnetometer is calibrated PCK-ST1 waits for RPi to bootup (or times out ~TBDs -> ERROR state)
 * 			- if magnetometer calibration data are not present in flash calibration is initiated
 * 		- if RPi boots up correctly and sends OK message LEDs signal to press button for NCP aligning -> ALIGNING
 *
 * ALIGNING
 * 		- drives AZ and EL axis based on GPS and magnetometer data
 * 		- sends "take star picture" to RPi
 * 			- if only Polaris found - point to the Polaris (RPi waits for another "take star picture" command)
 * 			- if NCP found - point to NCP estimated position (repeat taking picture for better accuracy)
 * 			- if no stars are found -> ERROR state
 *		- turns off EL and AZ motor after successful aligning - reduces power (AZ may stay powered to prevent accidental movement - TBD)
 *		- jumps into IDLE state
 *
 * IDLE
 * 		- waits for commands from PC/Controller or RPi (BTE app)
 */
#include "low_power_idle.h"
#include "warming_up.h"
#include "control_loop.h"
#include "homing.h"

ControlState_t controlState;
ControlState_t prevControlState;

Align_t Align_data = {
	.elevation_angle = 0,
	.declination = 0,
};

void Control_loop()
{
	switch (controlState)
	    {
	        case LOW_POWER_IDLE:
	            handleLowPowerIdle();   // checks button sequence
	            break;

	        case HOMING:
				handleHoming();
				break;

	        case WARMING_UP:
	            handleWarmingUp();      // your warming up logic
	            break;

	        case AXIS_MOVING:
	        	handleMoving();
	        	break;

	        // add more states as needed
	        default:
	            break;
	    }
}

/*
 * MOVING STATE
 *
 * Whenever a move command comes it should fill at least one of the move request structures with sufficient info to move the axis
 *
 */
MoveRequest_t AZ_MoveRequest = {
		.angle = 0,
		.speed = 0,
		.moveRequested = false,
};

MoveRequest_t EL_MoveRequest = {
		.angle = 0,
		.speed = 0,
		.moveRequested = false,
};

MoveRequest_t DEC_MoveRequest = {
		.angle = 0,
		.speed = 0,
		.moveRequested = false,
};

MoveRequest_t RA_MoveRequest = {
		.angle = 0,
		.speed = 0,
		.moveRequested = false,
};


void handleMoving()
{




}



