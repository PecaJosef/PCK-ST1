/*
 * warming_up_state.c
 *
 *  Created on: Aug 25, 2025
 *      Author: pecka
 */

//#include "control_loop.h"
#include "warming_up.h"

typedef enum {
	WAITING_FOR_GPS_FIX,
	WAITING_FOR_BUTTON_PRESS,
	WAITING_FOR_BUTTON_RELEASE,
}Warm_up_state_t;

static bool gpsConfigured = false;
static uint32_t gpsLastCheck = 0;
static GPS_Data_t GPS_Data;
static Warm_up_state_t WarmUpState = WAITING_FOR_BUTTON_PRESS;//WAITING_FOR_GPS_FIX;

void handleWarmingUp()
{

    uint8_t btn = readButtonDebounced();

    // 1. Check if GPS is alive (sending data)
    if (!GPS_IsAlive()) {
    	controlState = FAULT;
        return;
    }

    // 2. Configure GPS once
    if (!gpsConfigured) {
        GPS_Config();
        gpsConfigured = true;
    }

    //Wait for valid GPS fix
    switch (WarmUpState)
    {
		case WAITING_FOR_GPS_FIX:
			if ((HAL_GetTick()-gpsLastCheck)>1000)
			{
				gpsLastCheck = HAL_GetTick();
				GPS_Data = Get_GPS_Data();
				//Calculate altitude angle from GPS longitude
				alignmentData.altAngle = 90.0f-GPS_Data.longitude;
				// Compute magnetic declination from GPS fix
				float wmm_date = wmm_get_date(GPS_Data.year, GPS_Data.month, GPS_Data.day);
				float declination;
				E0000(GPS_Data.latitude, GPS_Data.longitude, wmm_date, &declination);
				// Move on to next state
			}
			break;
		case WAITING_FOR_BUTTON_PRESS:
			if (btn == PRESSED) // pressed
			{
				WarmUpState = WAITING_FOR_BUTTON_RELEASE;
			}
			break;
		case WAITING_FOR_BUTTON_RELEASE:
			if (btn == RELEASED) // released
			{
				controlState = LOW_POWER_IDLE; //Change state to ALIGN
			}
			break;
	}

}
