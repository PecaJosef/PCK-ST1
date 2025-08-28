/*
 * control_loop.c
 *
 *  Created on: Aug 25, 2025
 *      Author: pecka
 */
#include <low_power_idle.h>
#include <warming_up.h>
#include "control_loop.h"


ControlState_t controlState;
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

	        case WARMING_UP:
	            handleWarmingUp();      // your warming up logic
	            break;

	        // add more states as needed
	        default:
	            break;
	    }

}



