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
 * 		- another button press initiates homing -> HOMING state, powers up RPi - button press is part of the homing state
 *
 * HOMING
 * 		- homes all axis
 * 		- after homing is performed software jumps to WARMING_UP state
 *
 * WARMING_UP
 * 		- checks if GPS and magnetometer are operational (if not -> ERROR state)
 * 			- if magnetometer is calibrated PCK-ST1 waits for RPi to bootup (or times out ~(TBD)s -> ERROR state)
 * 			- if magnetometer calibration data are not present in flash calibration is initiated
 * 		- if RPi boots up correctly and sends OK message LEDs signal to press button for NCP aligning -> ALIGNING
 *
 * ALIGNING
 * 		- drives AZ and ALT axis based on GPS and magnetometer data
 * 		- sends "take star picture" to RPi
 * 			- if only Polaris found - point to the Polaris (RPi waits for another "take star picture" command)
 * 			- if NCP found - point to NCP estimated position (repeat taking picture for better accuracy)
 * 			- if no stars are found -> ERROR state
 *		- reduces ALT and AZ motor current after successful aligning
 *		- jumps into IDLE state
 *
 * IDLE
 * 		- waits for commands from PC/Controller or RPi (?BTE app)
 *
 * SHUTDOWN
 * 		- the SHUTDOWN state is initiate after a button is held for 5s (TBD)
 * 		- All LEDs light up (LED1-4), the the button is released and user should confirm the shutdown by press-holding the button again - LEDs turn off in seqence (similar to power on)
 * 		- if not confirmed, PCK-ST1 return to its previous state (after 10s - TBD)
 * 		- after a shutdown is initiated the main board stays "alive", to safely turn off RPi
 *
 *
 *
 */


//#include "low_power_idle.h"
//#include "warming_up.h"
#include "control_loop.h"
//#include "homing.h"
#include "cmd_handler.h"
#include "mag.h"

ControlState_t controlState;
ControlState_t prevControlState;

StatusFlags_t statusFlags;

/*
 *	LOW POWER IDLE variables and defines
 */

#define LED_DELAY 500
#define HOLD_TIME 4*LED_DELAY
#define RESET_TIME 1000

#define ROUGH_ALIGNMENT_THRESHOLD 2.0f
#define ALIGNMENT_TRIES 3
#define PRECISE_ALIGNMENT_THRESHOLD 0.5f

#define CALIB_STEP 2.5f
#define CALIB_MINMAX_SAMPLES 5

//---Low Power Idle internal states---
typedef enum {
	BTN_SEQ_WAIT_FIRST_PRESS,
	BTN_SEQ_WAIT_FIRST_RELEASE,
	BTN_SEQ_WAIT_SECOND_PRESS,
	BTN_SEQ_HOLDING,
	BTN_SEQ_DONE
} LowPowerIdleState_t;

//---Axis homing internal states---
typedef enum {
	HOMING_WAITING_FOR_BUTTON_PRESS,
	HOMING_WAITING_FOR_BUTTON_RELEASE,
	HOMING_PROCESS,
}HomingState_t;

//---Warming up internal states---
typedef enum {
	CHECK_AND_CONFIG_GPS,
	WAITING_FOR_GPS_ACK,
	WAITING_FOR_GPS_FIX,
}WarmingUpState_t;

//---Polar alignment internal states---
typedef enum {
	ALIGN_WAITING_FOR_BUTTON_PRESS,
	ALIGN_WAITING_FOR_BUTTON_RELEASE,
	ROUGH_ALIGNMENT_AZ,
	ROUGH_ALIGNMENT_AZ_CHECK,
	ROUGH_ALIGNMENT_ALT,
	ROUGH_ALIGNMENT_ALT_CHECK,
	PRECISE_ALIGN_CMD,
	PRECISE_ALIGN_WAIT,
	PRECISE_ALIGNMENT,
	ALIGNMENT_DONE,
}AlignState_t;

typedef enum {
    CAL_IDLE,
    CAL_ROTATION_CHECK,   // Moving to -180
    CAL_SAMPLING,     // Moving 0 to 360 and taking samples
    CAL_POST_ROTATION_CHECK,  // Returning to 0
    CAL_COMPUTE       // Calculating final matrix
} CalibState_t;


typedef struct {
	uint32_t timeoutDeadline;
	bool timeoutActive;
}Timeout_t;

static LowPowerIdleState_t btnSeqState = BTN_SEQ_WAIT_FIRST_PRESS;
static HomingState_t homingState = HOMING_WAITING_FOR_BUTTON_PRESS;
static WarmingUpState_t WarmUpState = CHECK_AND_CONFIG_GPS;
static AlignState_t alignmentState = ALIGN_WAITING_FOR_BUTTON_PRESS;
static CalibState_t calState = CAL_IDLE;

static uint16_t currentCalSample = 0;
static uint16_t totalCalSamples = 360/CALIB_STEP;
static int32_t sumX, sumY;
static int64_t sumXX, sumYY, sumXY;

static uint32_t secondPressStart = 0;
static uint32_t lastLedTime = 0;
static int ledStep = 0;

static GPS_Data_t gpsData;
Timeout_t timeoutControlLoop = {
		.timeoutActive = false,
		.timeoutDeadline = 0,
};

MagCalib_t magCalib;


MoveRequest_t AZ_MoveRequest = {
		.angle = 0,
		.speed = 0,
		.moveRequested = false,
};
MoveRequest_t ALT_MoveRequest = {
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

Align_t alignmentData = {
	.altAngle = 0,
	.declination = 0,
	.alignmentDataUpdated = false,
	.polarisFound = 0,
	.ncpFound = 0,
	.azError = 0,
	.altError = 0,
	.alignmentTriesCounter = 0,
};


static void axisHomingStart(StepperMotor_t *Axis, float coarseSpeed, float fineSpeed);
static void axisHomingUpdate(StepperMotor_t *Axis, float coarseSpeed, float fineSpeed);

static void blinkLeds();
void timeoutStart(Timeout_t *timeout, uint32_t seconds);
void timeoutReset(Timeout_t *timeout);
bool timeoutReached(Timeout_t *timeout);

void controlLoop()
{
	switch (controlState)
	{
		case LOW_POWER_IDLE:
			handleLowPowerIdle();
			break;

		case HOMING:
			handleHoming();
			break;

		case CALIBRATION:
			handleCalibration();
			break;

		case WARMING_UP:
			handleWarmingUp();
			break;

		case ALIGNMENT:
			handleAligning();
			break;
		case IDLE:
			//handleIdle();
			break;

		case AXIS_MOVING:
			handleMoving();
			break;

		case SHUTDOWN:
			//handleShutdown();
			break;
		case FAULT:
			//uartSend(PC_UART_SRC, "FAULT\r\n");
			break;

		default:
			break;
	}
}

/*
 * LOW POWER IDLE STATE
 */
void handleLowPowerIdle() {
	uint8_t btn = readButtonDebounced();

	switch (btnSeqState) {
		case BTN_SEQ_WAIT_FIRST_PRESS:
			if (btn == PRESSED) // pressed
			{
				btnSeqState = BTN_SEQ_WAIT_FIRST_RELEASE;
				// turn on LEDs initially
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
			}
			break;

		case BTN_SEQ_WAIT_FIRST_RELEASE:
			if (btn == RELEASED) // released
			{
				btnSeqState = BTN_SEQ_WAIT_SECOND_PRESS;
				secondPressStart = HAL_GetTick(); // start 2s window
			}
			break;

		case BTN_SEQ_WAIT_SECOND_PRESS:
			if ((HAL_GetTick() - secondPressStart) > RESET_TIME)
			{
				// Timeout, reset sequence
				btnSeqState = BTN_SEQ_WAIT_FIRST_PRESS;
				ledStep = 0;
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			} else if (btn == PRESSED) { // pressed again
				btnSeqState = BTN_SEQ_HOLDING;
				secondPressStart = HAL_GetTick();
				lastLedTime = secondPressStart;
				ledStep = 0;
				/*
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
				*/
			}
			break;

		case BTN_SEQ_HOLDING:
			if (btn == RELEASED) // released too early → reset
			{
				btnSeqState = BTN_SEQ_WAIT_FIRST_PRESS;
				ledStep = 0;
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
				break;
			}
			// Check how long held
			uint32_t heldTime = HAL_GetTick() - secondPressStart;

			// light LEDs progressively
			if ((heldTime / LED_DELAY) > ledStep && ledStep < 4)
			{
				ledStep++;
				switch (ledStep) {
					case 1: HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); break;
					case 2: HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); break;
					case 3: HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); break;
					case 4: HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET); break;
				}
			}

			if (heldTime >= HOLD_TIME) {
				btnSeqState = BTN_SEQ_DONE;
				HAL_GPIO_WritePin(PWR_BTN_LED_GPIO_Port, PWR_BTN_LED_Pin, GPIO_PIN_SET);
			}
			break;

		case BTN_SEQ_DONE:
			if (btn == RELEASED)
			{
				controlState = HOMING;
				btnSeqState = BTN_SEQ_WAIT_FIRST_PRESS; //reset for next time

				uartSend(PC_UART_SRC, "HOMING\r\n");
			}
			break;

	}
}

/*
 * HOMING STATE
 */

void handleHoming()
{
	static bool homing = false;
	uint8_t btn = readButtonDebounced();

	switch(homingState)
	{
		case HOMING_WAITING_FOR_BUTTON_PRESS:
			blinkLeds();

			if (btn == PRESSED) // pressed
			{
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
				homingState = HOMING_WAITING_FOR_BUTTON_RELEASE;
			}
			break;

		case HOMING_WAITING_FOR_BUTTON_RELEASE:
			if (btn == RELEASED)
			{
				ALT_AxisMotor.homing_state = AXIS_HOMING_IDLE;
				AZ_AxisMotor.homing_state = AXIS_HOMING_IDLE;
				RA_AxisMotor.homing_state = AXIS_HOMING_IDLE;
				DEC_AxisMotor.homing_state = AXIS_HOMING_IDLE;
				homingState = HOMING_PROCESS;
			}
			break;

		case HOMING_PROCESS:
			if (!homing)
			{
				// Start homing for all axes
				axisHomingStart(&ALT_AxisMotor, 10.0f, 5.0f);
				//Axis_Home_Start(&AZ_Axis_motor, 200.0f, 50.0f);
				//Axis_Home_Start(&RA_Axis_motor, 200.0f, 50.0f);
				//Axis_Home_Start(&DEC_Axis_motor, 200.0f, 50.0f);

				homing = true;
			}

			// Update all axes in parallel
			axisHomingUpdate(&ALT_AxisMotor, 10.0f, 7.5f);
			axisHomingUpdate(&AZ_AxisMotor, 10.0f, 7.5f);
			axisHomingUpdate(&RA_AxisMotor, 1.5f, 1.0f);
			axisHomingUpdate(&DEC_AxisMotor, 1.5f, 1.0f);

			// Check if all done
			if (!ALT_AxisMotor.homing && !AZ_AxisMotor.homing && !RA_AxisMotor.homing && !DEC_AxisMotor.homing)
			{
				homing = false;
				stepperDisable(&ALT_AxisMotor);

				ALT_AxisMotor.homing_state = AXIS_HOMING_IDLE;
				AZ_AxisMotor.homing_state = AXIS_HOMING_IDLE;
				RA_AxisMotor.homing_state = AXIS_HOMING_IDLE;
				DEC_AxisMotor.homing_state = AXIS_HOMING_IDLE;

				homingState = HOMING_WAITING_FOR_BUTTON_PRESS;
				controlState = CALIBRATION;
				uartSend(PC_UART_SRC, "CALIB\r\n");
			}
			break;
	}
}

static void axisHomingStart(StepperMotor_t *Axis, float coarseSpeed, float fineSpeed)
{
    if (!Axis) return;
    if (!Axis->enabled)
	{
    	stepperEnable(Axis);
	}

    Axis->homing = true;
    __HAL_GPIO_EXTI_CLEAR_IT(Axis->ENDSTOP_Pin);
    HAL_NVIC_EnableIRQ(Axis->EXTI_IRQn);

    if (HAL_GPIO_ReadPin(Axis->ENDSTOP_Port, Axis->ENDSTOP_Pin) == GPIO_PIN_RESET)
    {
        // Endstop already pressed → skip coarse, go to backoff
        Axis->homing_state = AXIS_HOMING_BACKOFF;
        stepperMove(Axis, 5.0f*DEG, fineSpeed);
    }
    else
    {
        // Normal coarse move towards switch
        Axis->homing_state = AXIS_HOMING_COARSE;
        stepperMove(Axis, -360.0f*DEG, coarseSpeed); // 180° is just "long enough"
    }
}

static void axisHomingUpdate(StepperMotor_t *Axis, float coarseSpeed, float fineSpeed)
{
    if (!Axis->homing) return;

    if (!Axis->busy) {  // Move finished without trigger
        switch (Axis->homing_state)
        {
        case AXIS_HOMING_BACKOFF:
            // Backoff done → now fine approach
        	__HAL_GPIO_EXTI_CLEAR_IT(Axis->ENDSTOP_Pin);
        	HAL_NVIC_EnableIRQ(Axis->EXTI_IRQn);

            Axis->homing_state = AXIS_HOMING_FINE;
            stepperMove(Axis, -10.0f*DEG, fineSpeed);
            break;

        case AXIS_HOMING_FINE:
            // Should have been triggered by interrupt. If not, treat as done anyway
            Axis->homing_state = AXIS_HOMING_DONE;
            Axis->homing = false;
            Axis->Position.angularPosition = 0;
            break;

        default:
            break;
        }
    }
}

void endstopReached(StepperMotor_t *Axis)
{
	stepperStop(Axis);

	if (!Axis->homing) return;

	switch (Axis->homing_state) {
	case AXIS_HOMING_COARSE:
		// First hit → back off
		Axis->homing_state = AXIS_HOMING_BACKOFF;
		stepperMove(Axis, 5.0f*DEG, 5.0f);
		break;

	case AXIS_HOMING_FINE:
		// Final hit → success
		Axis->homing_state = AXIS_HOMING_DONE;
		Axis->homing = false;
		Axis->Position.angularPosition = 0;
		break;

	default:
		break;
	}
}

/*
 * CALIBRATION STATE
 */
//#define CALIB_DIS

void handleCalibration()
{
	#ifdef CALIB_DIS
	controlState = WARMING_UP;
	return;

	#else
	if (LoadCalibrationFromFlash(&magCalib))
	{
		controlState = WARMING_UP;
		//printf("M00: %.9f, M01: %.9f, M10: %.9f, M11: %.9f\r\n",magCalib.softiron[0][0],magCalib.softiron[0][1],magCalib.softiron[1][0],magCalib.softiron[1][1]);

		return;
	}

	switch (calState)
	{
		case CAL_IDLE:
			//Initialize calibration accumulators
			sumX = sumY = sumXX = sumYY = sumXY = 0;
			currentCalSample = 0;

			//Rotate the star tracker (-180°)
			stepperMove(&AZ_AxisMotor, -180.0f*DEG, 15.0f);
			calState = CAL_ROTATION_CHECK;
			break;

		case CAL_ROTATION_CHECK:
			if (AZ_AxisMotor.busy)
			{
				break;
			}
			else
			{
				calState = CAL_SAMPLING;
			}
			break;

		case CAL_SAMPLING:
			//Check if the AZ motor is still running
			if (AZ_AxisMotor.busy)
			{
				break;
			}

			MagRawData_t calibData;
			if (magReadRaw(&calibData) == HAL_OK)
			{
				int16_t x = calibData.x;
				int16_t y = calibData.y;

				sumX += x;
				sumY += y;
				sumXX += x*x;
				sumYY += y*y;
				sumXY += x*y;

				//Move to next measurement point
				stepperMove(&AZ_AxisMotor, CALIB_STEP*DEG, 10.0f);

				currentCalSample++;
			}

			//Jump to next state if the full 360° measurement finished
			if (currentCalSample >= totalCalSamples)
			{
				//Measurement sweep finished - initiate rotation back to initial position (-180°)
				stepperMove(&AZ_AxisMotor, -180.0f*DEG, 15.0f);
				calState = CAL_POST_ROTATION_CHECK;
			}
			break;

		case CAL_POST_ROTATION_CHECK:
			if (AZ_AxisMotor.busy)
			{
				break;
			}
			else
			{
				calState = CAL_COMPUTE;
			}
			break;

		case CAL_COMPUTE:
			//Compute the calibration matrix

			magCalib = calibrationMatrix(sumX, sumY, sumXX, sumYY, sumXY, totalCalSamples);// offsetX, offsetY);
			SaveCalibrationToFlash(&magCalib);

			//Reset calibration state and jump to the warming up state
			controlState = WARMING_UP;
			uartSend(PC_UART_SRC, "WARMUP\r\n");
			calState = CAL_IDLE; // Reset local state
			break;
	}

	/*
	//#define CALIB_FROM_FLASH

	#ifdef CALIB_FROM_FLASH
	  if (!LoadCalibrationFromFlash(&magCalib))
	  {
		magCalib = CalibrateMagnetometer(&AZ_AxisMotor, 1.0f, 7.5f);
		SaveCalibrationToFlash(&magCalib);
	  }
	#else
	  magCalib = CalibrateMagnetometer(&AZ_AxisMotor, 1.0f, 7.5f);
	  SaveCalibrationToFlash(&magCalib);
	#endif

	controlState = WARMING_UP;
	uartSend(PC_UART_SRC, "WARMUP\r\n");
	return;
	*/
#endif

}




/*
 * WARMING UP STATE
 *
 *
 *
 *
 */

void handleWarmingUp()
{
	static bool gpsConfigured = false;
	static uint32_t gpsLastCheck = 0;
	static uint16_t configRetries = 0;
    //uint8_t btn = readButtonDebounced();

    switch (WarmUpState)
    {
    	case CHECK_AND_CONFIG_GPS:
    		//Check if GPS is alive (sending data)
			if (!gpsIsAlive()) {
				//controlState = FAULT;
				uartSend(PC_UART_SRC, "ERROR:GPS Fault\r\n");
				break;
			}

			//Configure GPS once
			if (!gpsConfigured)
			{
				uartSend(PC_UART_SRC, "GPS_CONF\r\n");
				gpsConfig();
				timeoutStart(&timeoutControlLoop, 2); // 2s ACK timeout
				WarmUpState = WAITING_FOR_GPS_ACK;
			}
			else
			{
				timeoutStart(&timeoutControlLoop, 60);
				WarmUpState = WAITING_FOR_GPS_FIX;
			}
			break;

    	case WAITING_FOR_GPS_ACK:
    		GPS_Ack_t ack = gpsCheckAck();

			if (ack == GPS_ACK_OK)
			{
				uartSend(PC_UART_SRC, "GPS_ACK_OK\r\n");
				gpsConfigured = true;
				configRetries = 0;
				timeoutStart(&timeoutControlLoop, 90);
				WarmUpState = WAITING_FOR_GPS_FIX;
			}
			else if (ack == GPS_ACK_NAK || timeoutReached(&timeoutControlLoop))
			{
			    if (ack == GPS_ACK_NAK)
			    {
			    	uartSend(PC_UART_SRC, "GPS_ACK_NAK\r\n");
			    }
			    else
				{
			    	uartSend(PC_UART_SRC, "GPS_ACK_TIMEOUT\r\n");
				}

			    if (++configRetries >= 3) {
			        uartSend(PC_UART_SRC, "ERROR:GPS Config Failed\r\n");
			        controlState = FAULT;
			    } else {
			        uartSend(PC_UART_SRC, "GPS_CONF_RETRY\r\n");
			        gpsConfig();
			        timeoutStart(&timeoutControlLoop, 2);
			    }
			}
			break;

    	case WAITING_FOR_GPS_FIX:
    		//Wait for valid GPS fix
    		if (timeoutReached(&timeoutControlLoop))
    		{
				timeoutReset(&timeoutControlLoop);
				uartSend(PC_UART_SRC, "ERROR: GPS Timeout\r\n");
				controlState = FAULT; // Jump to FAULT state
				break;
			}

			if ((HAL_GetTick()-gpsLastCheck)>1000)
			{
				gpsLastCheck = HAL_GetTick();

				uartSend(PC_UART_SRC, "GPS\r\n");

				gpsData = getGPSData();

				if (gpsData.fix == true)
				{
					timeoutReset(&timeoutControlLoop);
					//Calculate altitude angle from GPS longitude
					alignmentData.altAngle = 90.0f-gpsData.latitude;

					//printf("altAngle %f \r\n", alignmentData.altAngle);

					// Compute magnetic declination from GPS fix
					float wmm_date = wmm_get_date(gpsData.year, gpsData.month, gpsData.day);
					float declination;
					E0000(gpsData.latitude, gpsData.longitude, wmm_date, &declination);
					alignmentData.declination = declination;

					//Reset the internal state machine
					WarmUpState = WAITING_FOR_GPS_FIX;
					// Move on to next state
					controlState = ALIGNMENT;

					uartSend(PC_UART_SRC, "ALIGNMENT\r\n");
					break;
				}
				else
				{
					break;
				}
			}

	}
}


/*
 * ALIGNING STATE
 * Commands the RPi to start the alignment process (sends $ALIGN command)
 * Waits for the RPi response
 * Moves certain arcmin towards the NCP position
 */

void handleAligning()
{
	uint8_t btn = readButtonDebounced();
	static bool roughCorrection = false;

	switch(alignmentState)
	{
	case ALIGN_WAITING_FOR_BUTTON_PRESS:
		//Turns the LEDs on and off at around 2Hz - signaling the alignment is ready to begin
		blinkLeds();

		if (btn == PRESSED) // pressed
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			alignmentState = ALIGN_WAITING_FOR_BUTTON_RELEASE;
		}
		break;
	case ALIGN_WAITING_FOR_BUTTON_RELEASE:
		if (btn == RELEASED)
		{
			roughCorrection = false;
			alignmentState = ROUGH_ALIGNMENT_AZ;
		}
		break;

	case ROUGH_ALIGNMENT_AZ:
		float azAngle = getCalibratedHeading(&magCalib, alignmentData.declination);

		char dbgbuff[32];
		sprintf(dbgbuff,"Heading: %f\r\n", azAngle);
		uartSend(PC_UART_SRC, dbgbuff);

		//Move by the magnetic heading angle in the opposite direction
		stepperMove(&AZ_AxisMotor, -azAngle*DEG, 5.0f);

		alignmentState = ROUGH_ALIGNMENT_AZ_CHECK;

		break;

	case ROUGH_ALIGNMENT_AZ_CHECK:

		//Waits for the rough alignment move to complete
		if (AZ_AxisMotor.busy == true)
		{
			break;
		}

		float azAngleError = getCalibratedHeading(&magCalib, alignmentData.declination);

		if (azAngleError <= ROUGH_ALIGNMENT_THRESHOLD && azAngleError >= -ROUGH_ALIGNMENT_THRESHOLD  && roughCorrection == false)
		{
			roughCorrection = true;
			alignmentState = ROUGH_ALIGNMENT_AZ;
		}
		else
		{
			alignmentState = ROUGH_ALIGNMENT_ALT;
		}

		break;

	case ROUGH_ALIGNMENT_ALT:

		stepperMove(&ALT_AxisMotor, alignmentData.altAngle*DEG, 5.0f);
		alignmentState = ROUGH_ALIGNMENT_ALT_CHECK;
		break;

	case ROUGH_ALIGNMENT_ALT_CHECK:
		if (ALT_AxisMotor.busy == true)
		{
			break;
		}
		else
		{
			stepperMove(&RA_AxisMotor, 60.0*DEG, 2.0);

			alignmentData.alignmentTriesCounter = 0;
			alignmentState = PRECISE_ALIGN_CMD;
		}
		break;

	case PRECISE_ALIGN_CMD:
		if (RA_AxisMotor.busy == true)
		{
			break;
		}

		uartSend(RPI_UART_SRC, "$ALIGN\r\n");
		alignmentData.alignmentDataUpdated = false;
		alignmentState = PRECISE_ALIGN_WAIT;
		break;

	case PRECISE_ALIGN_WAIT:
		if (alignmentData.alignmentDataUpdated == false)
		{
			break;
		}
		else
		{
			alignmentState = PRECISE_ALIGNMENT;
		}
		break;

	case PRECISE_ALIGNMENT:
		if ((fabsf(alignmentData.azError) <= PRECISE_ALIGNMENT_THRESHOLD && fabsf(alignmentData.altAngle) <= PRECISE_ALIGNMENT_THRESHOLD) || alignmentData.alignmentTriesCounter >= ALIGNMENT_TRIES)
		{
			//If the alignment error is less than PRECISE_ALIGNMENT_THRESHOLD in both axes or a ALIGNMENT_TRIES are exceeded, the system proceeds to ALIGNMENT_DONE state
			stepperMove(&RA_AxisMotor, -60.0*DEG, 2.0);
			alignmentData.alignmentTriesCounter = 0;
			alignmentState = ALIGNMENT_DONE;
			break;
		}
		stepperMove(&AZ_AxisMotor, alignmentData.azError/cos(gpsData.latitude*(M_PI/180)), 5.0f);
		stepperMove(&ALT_AxisMotor, alignmentData.altError, 5.0f);
		alignmentData.alignmentTriesCounter++;

		alignmentState = PRECISE_ALIGN_CMD;
		break;

	case ALIGNMENT_DONE:
		if (ALT_AxisMotor.busy == true || AZ_AxisMotor.busy == true || RA_AxisMotor.busy == true)
		{
			break;
		}
		else
		{
			//Reset the internal state machine
			alignmentState = ALIGN_WAITING_FOR_BUTTON_PRESS;
			alignmentData.alignmentTriesCounter = 0;
			alignmentData.alignmentDataUpdated = false;

			controlState = LOW_POWER_IDLE;
		}
		break;

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

void handleMoving()
{
	//If all axis are not busy and there is no move requested the control loop shall return to the previous state
	if(!AZ_AxisMotor.busy && !ALT_AxisMotor.busy && !RA_AxisMotor.busy && !DEC_AxisMotor.busy)
	{
		if(!AZ_MoveRequest.moveRequested && !ALT_MoveRequest.moveRequested && !RA_MoveRequest.moveRequested && !DEC_MoveRequest.moveRequested)
		{
			controlState = prevControlState;
			return;
		}
	}

	if(AZ_MoveRequest.moveRequested && !AZ_AxisMotor.busy)
	{
		stepperMove(&AZ_AxisMotor, AZ_MoveRequest.angle, AZ_MoveRequest.speed);
		AZ_MoveRequest.moveRequested = false;
	}
	if(ALT_MoveRequest.moveRequested && !ALT_AxisMotor.busy)
	{
		stepperMove(&ALT_AxisMotor, ALT_MoveRequest.angle, ALT_MoveRequest.speed);
		ALT_MoveRequest.moveRequested = false;
	}
	if(RA_MoveRequest.moveRequested && !RA_AxisMotor.busy)
	{
		stepperMove(&RA_AxisMotor, RA_MoveRequest.angle, RA_MoveRequest.speed);
		RA_MoveRequest.moveRequested = false;
	}
	if(DEC_MoveRequest.moveRequested && !DEC_AxisMotor.busy)
	{
		stepperMove(&DEC_AxisMotor, DEC_MoveRequest.angle, DEC_MoveRequest.speed);
		DEC_MoveRequest.moveRequested = false;
	}
}



static void blinkLeds()
{
	static uint32_t ledLastTicks = 0;

	if ((HAL_GetTick() - ledLastTicks) >= 250)
	{
		ledLastTicks = HAL_GetTick();

		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	}

}

void timeoutStart(Timeout_t *timeout, uint32_t seconds)
{
    timeout->timeoutDeadline = HAL_GetTick() + (seconds * 1000);
    timeout->timeoutActive = true;
}

void timeoutReset(Timeout_t *timeout)
{
    timeout->timeoutActive = false;
}

bool timeoutReached(Timeout_t *timeout)
{
    if (timeout->timeoutActive && (HAL_GetTick() >= timeout->timeoutDeadline))
    {
        return true;
    }
    return false;
}
