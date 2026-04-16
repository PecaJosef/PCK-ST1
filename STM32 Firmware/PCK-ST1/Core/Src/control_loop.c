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
 * 		- after homing is performed, control loop checks the calibration
 *
 *CALIBRATION
 *		- checks if there are calibration data in the memory
 *		- if not, it starts calibrating the magnetometer by rotating around the AZ axis
 *		- after loading the calib data or calibrating check GPS and RPI in thw WARM_UP state
 *
 * WARMING_UP
 * 		- checks if GPS and magnetometer are operational (if not -> ERROR state)
 * 			- if magnetometer is calibrated PCK-ST1 waits for RPi to bootup (or times out ~(TBD)s -> ERROR state)
 * 			- if magnetometer calibration data are not present in flash calibration is initiated
 * 		- if RPi boots up correctly and sends RDY message LEDs signal to press button for NCP aligning -> ALIGNING
 *
 * ALIGNING
 * 		- drives AZ and ALT axis based on GPS and magnetometer data
 * 		- sends "take star picture" to RPi
 * 		- checks initial polar alignment
 * 			- if only Polaris found - point to the Polaris (sends "get center of rotation" command)
 * 			- if NCP found - point to NCP estimated position (sends "get center of rotation" command)
 * 			- if no stars are found -> ERROR state
 * 		- rotates the RA axis to take second CoR image
 * 		- after getting the CoR, proceed to polar alignment
 * 		- check the PA error up to 3 times and perform corrections
 * 			- sends "get PA error" command
 * 			- moves AZ ALT axes based on the error
 * 			- if the error is <= Threshold -> ALIGNMENT Done
 * 			- if the error is > Threshold - repeat
 *		- reduces ALT and AZ motor current after successful aligning
 *		- jumps into IDLE state
 *
 * IDLE
 * 		- waits for commands from PC/Controller or RPi (?BTE app)
 * 		- by pressing the button starts/ends the tracking
 *
 * SHUTDOWN
 * 		- the SHUTDOWN state is initiate after a button is held for a certain time
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
#include "telemetry.h"
#include "cmd_handler.h"
#include "mag.h"
#include "astro.h"


/*
 *	LOW POWER IDLE variables and defines
 */

#define POWER_OFF_TIME 2500
#define POWER_OFF_CANCLE_TIME 10000

#define LED_DELAY 500
#define HOLD_TIME 4*LED_DELAY
#define RESET_TIME 1000
#define LED_BLINK_PERIOD_SHORT 500
#define LED_BLINK_PERIOD_LONG 1000
#define TRACKING_RAMP_UP_PERIOD 1000

#define RPI_SHUTDOWN_TIME 5 //5s

//Duration in which the GPS should get fix and RPi should boot up into Python software
#define WARMING_UP_TIMEOUT 90
#define PA_TIMEOUT 60
#define COR_TIMEOUT 60

#define GPS_CONFIG_RETRIES 5

#define ROUGH_ALIGNMENT_THRESHOLD 2.0f
#define ALIGNMENT_TRIES 3
#define PRECISE_ALIGNMENT_THRESHOLD 0.5f

#define CALIB_STEP 2.5f
#define CALIB_MINMAX_SAMPLES 5

#define BACKOFF_DIST 10.0f*DEG
#define OVERSHOOT_DIST 20.0f*DEG
#define HOMING_DIST 135.0f*DEG
#define FINE_DIST 12.5f*DEG //Fine dist needs to be slightly larger than backoff

#define COR_ANGLE 60.0f

//---Power off button states---
typedef enum {
	PWR_OFF_BUTTON_RELEASED,
	PWR_OFF_BUTTON_PRESSED,
	PWR_OFF_POWERING_OFF,
}PowerOffButtonState_t;

//---Shutdown states---
typedef enum {
	SHUTDOWN_CHECK_BUTTON_RELEASE,
	SHUTDOWN_WAIT_FOR_LONG_PRESS,
	SHUTDOWN_LONG_PRESS,
	SHUTDOWN_PROCESS,
	SHUTDOWN_FINISHED,
}ShutdownState_t;

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
	TIMEOUT_START,
	WAITING_FOR_GPS_AND_RPI,
	WARMUP_FINISHED,
}WarmingUpState_t;

//---Warming up GPS states---
typedef enum {
	CHECK_AND_CONFIG_GPS,
	WAITING_FOR_GPS_ACK,
	WAITING_FOR_GPS_FIX,
	GPS_FIXED,
}gpsWarmingUpState_t;

//---Warming up RPi states---
typedef enum {
	WAITING_FOR_RPI_BOOT,
	RPI_READY,
}rpiWarmingUpState_t;

//---Polar alignment internal states---
typedef enum {
	ALIGN_WAITING_FOR_BUTTON_PRESS,
	ALIGN_WAITING_FOR_BUTTON_RELEASE,
	ROUGH_ALIGNMENT_AZ,
	ROUGH_ALIGNMENT_AZ_CHECK,
	ROUGH_ALIGNMENT_ALT,
	PRECISE_INITIAL_ALIGN_CMD,
	PRECISE_INITIAL_ALIGN_WAIT,
	PRECISE_INITIAL_ALIGNMENT,
	PRECISE_COR_CMD,
	PRECISE_COR_WAIT,
	PRECISE_COR_CHECK,
	PRECISE_ALIGN_CMD,
	PRECISE_ALIGN_WAIT,
	PRECISE_ALIGNMENT,
	ALIGNMENT_DONE,
}AlignState_t;

//---Calibration internal states---
typedef enum {
    CAL_IDLE,
    CAL_ROTATION_CHECK,   // Moving to -180
    CAL_SAMPLING,     // Moving 0 to 360 and taking samples
    CAL_POST_ROTATION_CHECK,  // Returning to 0
    CAL_COMPUTE       // Calculating final matrix
} CalibState_t;

//---Idle internal states---
typedef enum {
	IDLE_BTN_RELEASED,
	IDLE_BTN_PRESSED,
	IDLE_TRACKING_START,
}IdleState_t;

//--Tracking internal states---
typedef enum {
	TRACKING_BTN_RELEASED,
	TRACKING_BTN_PRESSED,
	TRACKING_STOP,
}TrackingState_t;

typedef struct {
	uint32_t timeoutDeadline;
	bool timeoutActive;
}Timeout_t;

static ShutdownState_t shutdownState = SHUTDOWN_CHECK_BUTTON_RELEASE;
static PowerOffButtonState_t pwrOffButtonState = PWR_OFF_BUTTON_RELEASED;
static LowPowerIdleState_t lowPowerIdleButtonState = BTN_SEQ_WAIT_FIRST_PRESS;
static HomingState_t homingState = HOMING_WAITING_FOR_BUTTON_PRESS;
static WarmingUpState_t WarmUpState = TIMEOUT_START;
static gpsWarmingUpState_t gpsWarmUpState = CHECK_AND_CONFIG_GPS;
static rpiWarmingUpState_t rpiWarmUpState = WAITING_FOR_RPI_BOOT;
static AlignState_t alignmentState = ALIGN_WAITING_FOR_BUTTON_PRESS;
static CalibState_t calState = CAL_IDLE;
static IdleState_t idleState = IDLE_BTN_RELEASED;
static TrackingState_t trackingState = TRACKING_BTN_RELEASED;

errorCode_t errorCode = NONE;

static uint16_t currentCalSample = 0;
static uint16_t totalCalSamples = 360/CALIB_STEP;
static int32_t sumX, sumY;
static int64_t sumXX, sumYY, sumXY;

static GPS_Data_t gpsData;

ControlState_t controlState = LOW_POWER_IDLE;
ControlState_t prevControlState;

StatusFlags_t statusFlags = {
		.calibForceEnable = false,
		.gpsOK = false,
		.gpsFixed = false,
		.homed = false,
		.calibrated = false,
		.polarAligned = false,
		.magOK = false,
		.moveEnabled = true,
		.rpiRDY = false,
		.fault = false,
};

Timeout_t timeoutControlLoop = {
		.timeoutActive = false,
		.timeoutDeadline = 0,
};

Timeout_t gpsAckTimeout = {
		.timeoutActive = false,
		.timeoutDeadline = 0,
};

Timeout_t rpiShutdownTimeout = {
		.timeoutActive = false,
		.timeoutDeadline = 0,
};

Timeout_t paCommandTimeout = {
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
	.raAngle = 0,
	.alignmentTriesCounter = 0,
	.corFound = 0,
	.imageCaptured = false
};


static void axisHomingStart(StepperMotor_t *Axis, float coarseSpeed, float fineSpeed);
static void axisHomingUpdate(StepperMotor_t *Axis, float coarseSpeed, float fineSpeed);

static void checkRPi();
static void checkGPS();

static void ledsBlink(uint32_t blinkPeriod);
static void ledsRampUp(uint32_t ledPeriod);
static void pwrButtonBlink(uint32_t ledPeriod);

static void resetStates();

void timeoutStart(Timeout_t *timeout, uint32_t seconds);
void timeoutReset(Timeout_t *timeout);
bool timeoutReached(Timeout_t *timeout);

/*
 * MAIN CONTROL LOOP
 */
void controlLoop()
{
	static uint32_t shutdownButtonPressStart = 0;

	if (controlState != LOW_POWER_IDLE && controlState != SHUTDOWN)
	{
		uint8_t btn = readButtonDebounced();

		switch(pwrOffButtonState)
		{
		case PWR_OFF_BUTTON_RELEASED:
			if (btn == PRESSED)
			{
				//If the power button is pressed, capture the press time and jump to next state
				pwrOffButtonState = PWR_OFF_BUTTON_PRESSED;
				shutdownButtonPressStart = HAL_GetTick();
			}
			break;

		case PWR_OFF_BUTTON_PRESSED:
			//Check if the button is still pressed, if not -> reset
			if (btn == RELEASED)
			{
				pwrOffButtonState = PWR_OFF_BUTTON_RELEASED;
				break;
			}
			//Check how long is the button held pressed
			uint32_t heldTime = HAL_GetTick() - shutdownButtonPressStart;

			//
			if (heldTime >= POWER_OFF_TIME)
			{
				pwrOffButtonState = PWR_OFF_POWERING_OFF;
			}
			break;

		case PWR_OFF_POWERING_OFF:
			pwrButtonSet(true);
			ledsSet(true);

			if (btn == RELEASED)
			{
				//Save last control loop state in case of returning back
				prevControlState = controlState;

				//Reset all states to the default
				resetStates();

				//Stop all motors
				if (AZ_AxisMotor.busy) stepperStop(&AZ_AxisMotor);
				if (ALT_AxisMotor.busy) stepperStop(&ALT_AxisMotor);
				if (RA_AxisMotor.busy) stepperStop(&RA_AxisMotor);
				if (DEC_AxisMotor.busy) stepperStop(&DEC_AxisMotor);


				WarmUpState = TIMEOUT_START;
				gpsWarmUpState = CHECK_AND_CONFIG_GPS;
				rpiWarmUpState = WAITING_FOR_RPI_BOOT;
				alignmentState = ALIGN_WAITING_FOR_BUTTON_PRESS;
				calState = CAL_IDLE;

				pwrOffButtonState = PWR_OFF_BUTTON_RELEASED;
				controlState = SHUTDOWN;
			}
			break;
		}
	}


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
		handleIdle();
		break;

	case AXIS_MOVING:
		handleMoving();
		break;

	case GOTO:
		handleGoto();
		break;

	case SHUTDOWN:
		handleShutdown();
		break;
	case FAULT:
		handleFault();
		break;

	case TRACKING:
		handleTracking();
		break;

	default:
		controlState = FAULT;
		break;
	}
}

/*
 * LOW POWER IDLE STATE
 */
void handleLowPowerIdle()
{
	uint8_t btn = readButtonDebounced();
	static uint32_t secondPressStart = 0;
	//static uint32_t lastLedTime = 0;
	static int ledStep = 0;

	switch (lowPowerIdleButtonState) {
		case BTN_SEQ_WAIT_FIRST_PRESS:
			if (btn == PRESSED) // pressed
			{
				lowPowerIdleButtonState = BTN_SEQ_WAIT_FIRST_RELEASE;
				//Turn LEDs ON on button press
				ledsSet(true);
				pwrButtonSet(true);
			}
			break;

		case BTN_SEQ_WAIT_FIRST_RELEASE:
			if (btn == RELEASED) //Released
			{
				lowPowerIdleButtonState = BTN_SEQ_WAIT_SECOND_PRESS;
				secondPressStart = HAL_GetTick(); //Start 2s window
			}
			break;

		case BTN_SEQ_WAIT_SECOND_PRESS:
			if ((HAL_GetTick() - secondPressStart) > RESET_TIME)
			{
				// Timeout, reset sequence
				lowPowerIdleButtonState = BTN_SEQ_WAIT_FIRST_PRESS;
				ledStep = 0;
				//Reset LEDs to ON state
				ledsSet(false);
			}
			else if (btn == PRESSED) //Pressed again
			{
				lowPowerIdleButtonState = BTN_SEQ_HOLDING;
				secondPressStart = HAL_GetTick();
				//lastLedTime = secondPressStart;
				ledStep = 0;
			}
			break;

		case BTN_SEQ_HOLDING:
			if (btn == RELEASED) //released too early -> reset
			{
				ledStep = 0;
				ledsSet(false);
				lowPowerIdleButtonState = BTN_SEQ_WAIT_FIRST_PRESS;
				break;
			}
			// Check how long held
			uint32_t heldTime = HAL_GetTick() - secondPressStart;

			// light LEDs progressively
			if ((heldTime / LED_DELAY) > ledStep && ledStep < 4)
			{
				ledStep++;
				switch (ledStep)
				{
					case 1:
						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
						break;
					case 2:
						HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
						break;
					case 3:
						HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
						break;
					case 4:
						HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
						break;
				}
			}

			if (heldTime >= HOLD_TIME) {
				lowPowerIdleButtonState = BTN_SEQ_DONE;
			}
			break;

		case BTN_SEQ_DONE:
			if (btn == RELEASED)
			{
				controlState = HOMING;
				lowPowerIdleButtonState = BTN_SEQ_WAIT_FIRST_PRESS; //Reset for next time

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
    static bool homingActive = false;
    uint8_t btn = readButtonDebounced();

    switch(homingState)
    {
        case HOMING_WAITING_FOR_BUTTON_PRESS:
            ledsBlink(LED_BLINK_PERIOD_SHORT);
            if (btn == PRESSED)
            {
                ledsSet(false);
                homingState = HOMING_WAITING_FOR_BUTTON_RELEASE;
            }
            break;

        case HOMING_WAITING_FOR_BUTTON_RELEASE:
            if (btn == RELEASED)
            {
                //Reset all internal states before starting homing
                ALT_AxisMotor.homing_state = AXIS_HOMING_IDLE;
                AZ_AxisMotor.homing_state = AXIS_HOMING_IDLE;
                RA_AxisMotor.homing_state = AXIS_HOMING_IDLE;
                DEC_AxisMotor.homing_state = AXIS_HOMING_IDLE;
                homingState = HOMING_PROCESS;
                homingActive = false;
            }
            break;

        case HOMING_PROCESS:
            if (!homingActive)
            {
                //Star homing all axes
                axisHomingStart(&ALT_AxisMotor, ALT_COARSE_SPEED, ALT_FINE_SPEED);
                axisHomingStart(&RA_AxisMotor, RA_COARSE_SPEED, RA_FINE_SPEED);
                axisHomingStart(&DEC_AxisMotor, DEC_COARSE_SPEED, DEC_FINE_SPEED);
                homingActive = true;
            }

            // Update logic for all axes in parallel
            axisHomingUpdate(&ALT_AxisMotor, ALT_COARSE_SPEED, ALT_FINE_SPEED);
            axisHomingUpdate(&RA_AxisMotor, RA_COARSE_SPEED, RA_FINE_SPEED);
            axisHomingUpdate(&DEC_AxisMotor, DEC_COARSE_SPEED, DEC_FINE_SPEED);

            // Check if all axes have finished their specific state machines
            if (!ALT_AxisMotor.homing && !RA_AxisMotor.homing && !DEC_AxisMotor.homing)
            {
                homingActive = false;
                statusFlags.homed = true;
                //Reset the homing state
                homingState = HOMING_WAITING_FOR_BUTTON_PRESS;
                controlState = CALIBRATION;
                uartSend(PC_UART_SRC, "HOMING DONE\r\n");
            }
            break;
    }
}

static void axisHomingStart(StepperMotor_t *Axis, float coarseSpeed, float fineSpeed)
{
    if (!Axis)
	{
    	return;
	}

    if (!Axis->enabled)
	{
		stepperEnable(Axis);
	}

    Axis->homing = true;
    __HAL_GPIO_EXTI_CLEAR_IT(Axis->ENDSTOP_Pin);
    HAL_NVIC_EnableIRQ(Axis->EXTI_IRQn);

    if (HAL_GPIO_ReadPin(Axis->ENDSTOP_Port, Axis->ENDSTOP_Pin) == GPIO_PIN_RESET)
    {
        //Already on endstop -> back off from the endstop (positive dir)
        Axis->homing_state = AXIS_HOMING_BACKOFF;
        stepperMove(Axis, BACKOFF_DIST, fineSpeed);
    }
    else
    {
        //Normal Start
        Axis->homing_state = AXIS_HOMING_SEARCH_COARSE; //Move 180°towards endstop (negative dir)
        stepperMove(Axis, -HOMING_DIST, coarseSpeed);
    }
}

static void axisHomingUpdate(StepperMotor_t *Axis, float coarseSpeed, float fineSpeed)
{
    if (!Axis->homing || Axis->busy)
	{
		return; //Return from the update function if the axis is not homing or is busy (still moving)
	}

    switch (Axis->homing_state)
    {
        case AXIS_HOMING_SEARCH_COARSE:
            //This code is not executed unless the endstop is not found in the initial search
        	//Move towards positive direction
            stepperMove(Axis, 2*HOMING_DIST, coarseSpeed);
            break;

        case AXIS_HOMING_BACKOFF:
			//Backoff done -> jump to fine search
			__HAL_GPIO_EXTI_CLEAR_IT(Axis->ENDSTOP_Pin);
			HAL_NVIC_EnableIRQ(Axis->EXTI_IRQn);

			Axis->homing_state = AXIS_HOMING_SEARCH_FINE;

			if (Axis->Position.direction == Axis->Positive_dir) //The Axis->Position.direction stores last motion direction (backoff dir) -> move the opposite way
			{
				stepperMove(Axis, -FINE_DIST, fineSpeed);
			}
			else
			{
				stepperMove(Axis, FINE_DIST, fineSpeed);
			}
			break;

        case AXIS_HOMING_OVERSHOOT:
        	//Only for highPrecision axes
            //Found edgeA and performed overshoot past the endstop
        	//Clear the interrupt and enable it
        	__HAL_GPIO_EXTI_CLEAR_IT(Axis->ENDSTOP_Pin);
			HAL_NVIC_EnableIRQ(Axis->EXTI_IRQn);

            Axis->homing_state = AXIS_HOMING_REVERSE_SEARCH;
            //Reverse of current direction
            if (Axis->Position.direction == Axis->Positive_dir) //The Axis->Position.direction stores last motion direction -> the axis shall move the opposite way
            {
            	stepperMove(Axis, -OVERSHOOT_DIST, fineSpeed);
            }
            else
            {
            	stepperMove(Axis, OVERSHOOT_DIST, fineSpeed);
            }
            break;

        case AXIS_HOMING_GO_TO_CENTER:
            // Final movement finished
            Axis->homing_state = AXIS_HOMING_DONE;
            break;

        case AXIS_HOMING_DONE:
            Axis->homing = false;

            Axis->Position.stepPosition = 0;
			Axis->Position.lastStepsRead = 0;
			Axis->Position.angularPosition = 0;

            //If the axis is not highPrecision (ALT) disable it (the axis is not backdrivable, no need to keep it enabled)
            if(!Axis->highPrecisionAxis)
            {
            	stepperDisable(Axis);
            }
            break;

        default: break;
    }
}

void endstopReached(StepperMotor_t *Axis, float fineSpeed)
{
    if (!Axis->homing)
	{
    	return;
	}

    stepperStop(Axis); //The stepperStop functions updates the Position
    float currentPos = Axis->Position.angularPosition;

    switch(Axis->homing_state) {
        case AXIS_HOMING_SEARCH_COARSE:
            //Hit the endstop during coarse search -> backoff
            Axis->homing_state = AXIS_HOMING_BACKOFF;
            if (Axis->Position.direction == Axis->Positive_dir)
            {
            	stepperMove(Axis, -BACKOFF_DIST, fineSpeed);
            }
            else
            {
            	stepperMove(Axis, BACKOFF_DIST, fineSpeed);
            }
            break;

        case AXIS_HOMING_SEARCH_FINE:
            //Hit the endstop during fine search -> DONE for !highPrecision, save endgeA position for highPrecision
            if (Axis->highPrecisionAxis)
            {
            	Axis->edgeA = currentPos;
                //Move past the endstop to perform reverse search
                Axis->homing_state = AXIS_HOMING_OVERSHOOT;
                if (Axis->Position.direction == Axis->Positive_dir)
                {
                	stepperMove(Axis, OVERSHOOT_DIST, fineSpeed);
                }
                else
                {
                	stepperMove(Axis, -OVERSHOOT_DIST, fineSpeed);
                }

            }
            else
            {
                //Homing done for !highPrecision axis
                Axis->homing_state = AXIS_HOMING_DONE;
            }
            break;

        case AXIS_HOMING_REVERSE_SEARCH:
            //Hit the endstop during reverse search -> save the edgeB position -> move towards center of endstop active area
            Axis->edgeB = currentPos;
            Axis->homing_state = AXIS_HOMING_GO_TO_CENTER;
            float center = (Axis->edgeA - Axis->edgeB) / 2.0f;
            //Move to the endstop center position

            stepperMove(Axis, center*DEG, fineSpeed);
            break;

        default: break;
    }
}

/*
 * CALIBRATION STATE
 */

void handleCalibration()
{
	static bool magChecked = false;
	static uint8_t magCheckTries = 0;

	if(magChecked == false)
	{
		if (magIsAlive() == false)
		{
			if (magCheckTries >= 3)
			{
				error(MAG_ERROR);
				magCheckTries = 0;
			}
			magCheckTries++;
			return;
		}
		else
		{
			uartSend(PC_UART_SRC, "MAG OK\r\n");
			magChecked = true;
			magCheckTries = 0;
		}
	}

	if (LoadCalibrationFromFlash(&magCalib) && statusFlags.calibForceEnable == false)
	{
		controlState = WARMING_UP;
		statusFlags.calibrated = true;

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

			statusFlags.calibrated = true;

			//Reset calibration state and jump to the warming up state
			controlState = WARMING_UP;
			uartSend(PC_UART_SRC, "WARMUP\r\n");
			calState = CAL_IDLE; // Reset local state
			break;
	}
}

/*
 * WARMING UP STATE
 */

void handleWarmingUp()
{
	ledsRampUp(LED_BLINK_PERIOD_SHORT);

	switch(WarmUpState)
	{
		case TIMEOUT_START:
			//Start booting the RPi
			rpiPowerOn();
			//Start the warm up timeout
			timeoutStart(&timeoutControlLoop, WARMING_UP_TIMEOUT);
			WarmUpState = WAITING_FOR_GPS_AND_RPI;
			break;

		case WAITING_FOR_GPS_AND_RPI:
			if (timeoutReached(&timeoutControlLoop))
			{
				timeoutReset(&timeoutControlLoop);
				uartSend(PC_UART_SRC, "ERROR: Warming up timeout\r\n");
				ledsSet(false);
				//Set error to WARMUP TIMEOUT
				error(WARMUP_TIMEOUT);
				break;
			}

			checkGPS();
			checkRPi();
			if(statusFlags.rpiRDY && statusFlags.gpsFixed)
			{
				timeoutReset(&timeoutControlLoop);
				WarmUpState = WARMUP_FINISHED;
				gpsWarmUpState = CHECK_AND_CONFIG_GPS;
				rpiWarmUpState = WAITING_FOR_RPI_BOOT;
			}
			break;

		case WARMUP_FINISHED:
			//Reset warming up state
			WarmUpState = TIMEOUT_START;
			// Move on to next state
			controlState = ALIGNMENT;
			//Reset LEDs
			ledsSet(false);

			uartSend(PC_UART_SRC, "ALIGNMENT\r\n");
			break;
	}
}

static void checkGPS()
{
	static bool gpsConfigured = false;
	static uint32_t gpsLastCheck = 0;
	static uint16_t configRetries = 0;

	switch(gpsWarmUpState)
    {
	case CHECK_AND_CONFIG_GPS:
		//Check if GPS is alive (sending any data)
		if (!gpsIsAlive()) {
			statusFlags.gpsOK = false;
			uartSend(PC_UART_SRC, "ERROR:GPS Fault\r\n");

			error(GPS_ERROR);
			break;
		}
		else
		{
			statusFlags.gpsOK = true;
		}

		//Configure GPS once
		if (!gpsConfigured)
		{
			uartSend(PC_UART_SRC, "GPS Configuration...\r\n");
			gpsConfig();
			timeoutStart(&gpsAckTimeout, 2); // 2s ACK timeout
			gpsWarmUpState = WAITING_FOR_GPS_ACK;
		}
		else
		{
			gpsWarmUpState = WAITING_FOR_GPS_FIX;
		}
		break;

	case WAITING_FOR_GPS_ACK:
		GPS_Ack_t ack = gpsCheckAck();

		if (ack == GPS_ACK_OK)
		{
			uartSend(PC_UART_SRC, "GPS Configured\r\n");
			gpsConfigured = true;
			configRetries = 0;
			gpsWarmUpState = WAITING_FOR_GPS_FIX;
		}
		else if (ack == GPS_ACK_NAK || timeoutReached(&gpsAckTimeout))
		{
			if (ack == GPS_ACK_NAK)
			{
				uartSend(PC_UART_SRC, "GPS ACK:NAK\r\n");
			}
			else
			{
				uartSend(PC_UART_SRC, "GPS Configuration timeout\r\n");
			}

			if (++configRetries >= GPS_CONFIG_RETRIES)
			{
				uartSend(PC_UART_SRC, "ERROR:GPS Configuration failed!\r\n");
				controlState = FAULT;
			}
			else
			{
				uartSend(PC_UART_SRC, "GPS Configuration retry\r\n");
				gpsConfig();
				timeoutStart(&gpsAckTimeout, 2);
			}
		}
		break;

	case WAITING_FOR_GPS_FIX:
		//Wait for valid GPS fix
		if ((HAL_GetTick()-gpsLastCheck)>1000)
		{
			gpsLastCheck = HAL_GetTick();
			gpsData = getGPSData();

			if (gpsData.fix == true)
			{
				uartSend(PC_UART_SRC, "GPS Fixed\r\n");

				//Calculate altitude angle from GPS longitude
				alignmentData.altAngle = 90.0f-gpsData.latitude;

				//Calculate magnetic declination from GPS data
				float wmm_date = wmm_get_date(gpsData.year, gpsData.month, gpsData.day);
				float declination;
				E0000(gpsData.latitude, gpsData.longitude, wmm_date, &declination);
				alignmentData.declination = declination;

				statusFlags.gpsFixed = true;
				//Reset the internal state machine
				gpsWarmUpState = GPS_FIXED;
				break;
			}
			else
			{
				break;
			}
		}
		break;

	case GPS_FIXED:
			//Do nothing if the GPS is already fixed and the system is still waiting for RPi to boot up
		break;
	}
}

static void checkRPi()
{
	switch(rpiWarmUpState)
	{
		case WAITING_FOR_RPI_BOOT:
			if(statusFlags.rpiRDY == false)
			{
				break;
			}
			else
			{
				uartSend(PC_UART_SRC, "RPi is ready\r\n");
				rpiWarmUpState = RPI_READY;
			}
		break;

		case RPI_READY:
			//Do nothing if the RPi is already booted and the system is still waiting for GPS to fix
		break;
	}
}

/*
 * --- ALIGNING STATE ---
 */

void handleAligning()
{
	uint8_t btn = readButtonDebounced();
	static bool roughCorrection = false;
	static bool corFirstImageCaptured = false;
	static bool corSecondImageCaptured = false;

	switch(alignmentState)
	{
	case ALIGN_WAITING_FOR_BUTTON_PRESS:
		//Turns the LEDs on and off at around 2Hz - signaling the alignment is ready to begin
		ledsBlink(LED_BLINK_PERIOD_SHORT);

		if (btn == PRESSED) // pressed
		{
			ledsSet(false);
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
		alignmentState = PRECISE_INITIAL_ALIGN_CMD;
		break;

	case PRECISE_INITIAL_ALIGN_CMD:
		//Wait for the ALT motor to finish the move
		if (ALT_AxisMotor.busy == true)
		{
			break;
		}

		uartSend(RPI_UART_SRC, "$PA:ALIGN\r\n");
		timeoutStart(&paCommandTimeout, PA_TIMEOUT);

		alignmentData.alignmentDataUpdated = false;
		alignmentData.imageCaptured = false;
		alignmentState = PRECISE_INITIAL_ALIGN_WAIT;
		break;

	case PRECISE_INITIAL_ALIGN_WAIT:
		//Wait for the PA image to be captured
		if (alignmentData.imageCaptured == false)
		{
			if (timeoutReached(&paCommandTimeout) == true)
			{
				timeoutReset(&paCommandTimeout);
				error(PA_ERROR);
			}
			break;
		}
		//Image captured - move RA 45° to capture first CoR image while the PA image gets processed
		alignmentData.imageCaptured = false;
		stepperMove(&RA_AxisMotor, COR_ANGLE*DEG, RA_COARSE_SPEED);
		alignmentState = PRECISE_INITIAL_ALIGNMENT;
		break;

	case PRECISE_INITIAL_ALIGNMENT:
		//Wait for the alignment data to be updated
		if (alignmentData.alignmentDataUpdated == false)
		{
			if (timeoutReached(&paCommandTimeout) == true)
			{
				timeoutReset(&paCommandTimeout);
				error(PA_ERROR);
			}
			break;
		}
		//Image captured and alignment data updated - reset the polar alignment timeout
		timeoutReset(&paCommandTimeout);

		//Polaris not found - ERROR
		if (alignmentData.polarisFound == false)
		{
			error(POLARIS_NOT_FOUND);
			break;
		}
		//For the initial polar alignment if the NCP is not found, the alignment is done on Polaris, to center the image closer to NCP
		//Alignment data updated and NCP or Polaris found
		if(alignmentData.azError != 0.0f)
		{
			stepperMove(&AZ_AxisMotor, alignmentData.azError/cosf(gpsData.latitude*(M_PI/180)), 5.0f);
		}
		stepperMove(&ALT_AxisMotor, alignmentData.altError, 5.0f);

		alignmentState = PRECISE_COR_CMD;
		break;

	case PRECISE_COR_CMD:
		//Wait for the initial alignment to finish and for the RA axis to stop moving
		if (AZ_AxisMotor.busy == true || ALT_AxisMotor.busy == true || RA_AxisMotor.busy == true)
		{
			break;
		}
		//Send $PA:COR command
		uartSend(RPI_UART_SRC, "$PA:COR\r\n");
		timeoutStart(&paCommandTimeout, COR_TIMEOUT);

		alignmentState = PRECISE_COR_WAIT;
		break;

	case PRECISE_COR_WAIT:
		//Wait for image to be captured
		if (alignmentData.imageCaptured == false)
		{
			if (timeoutReached(&paCommandTimeout) == true)
			{
				timeoutReset(&paCommandTimeout);
				error(COR_ERROR);
			}
			break;
		}
		//Reset the timeout
		timeoutReset(&paCommandTimeout);

		if (corFirstImageCaptured == false)
		{
			corFirstImageCaptured = true;
			alignmentData.imageCaptured = false;
			//Captured first CoR image, move the RA axis back to 0 and send second CoR command
			stepperMove(&RA_AxisMotor, -COR_ANGLE*DEG, RA_COARSE_SPEED);
			alignmentState = PRECISE_COR_CMD;
			break;
		}
		if (corSecondImageCaptured == false)
		{
			//Captured second CoR image
			corSecondImageCaptured = true;
			alignmentData.imageCaptured = false;

			//Start timer for the CoR calculation
			timeoutStart(&paCommandTimeout, COR_TIMEOUT);
			//Both images for CoR captured proceed to proper polar alignment
			alignmentState = PRECISE_COR_CHECK;
			break;
		}
		break;

	case PRECISE_COR_CHECK:
		//Wait until CoR calculation is complete and the CoR is found
		if (alignmentData.corFound == 0)
		{
			if (timeoutReached(&paCommandTimeout) == true)
			{
				timeoutReset(&paCommandTimeout);
				error(COR_ERROR);
			}
			break;
		}
		//Reset the timeout if the CoR command is received
		timeoutReset(&paCommandTimeout);

		//CoR FAILED - ERROR
		if(alignmentData.corFound == -1)
		{
			error(COR_ERROR);
			break;
		}
		//CoR DONE
		alignmentData.alignmentTriesCounter = 0;
		alignmentState = PRECISE_ALIGN_CMD;
		break;

	case PRECISE_ALIGN_CMD:
		if (AZ_AxisMotor.busy || ALT_AxisMotor.busy)
		{
			break;
		}
		uartSend(RPI_UART_SRC, "$PA:ALIGN\r\n");
		//Start the PA timeout
		timeoutStart(&paCommandTimeout, PA_TIMEOUT);
		alignmentData.alignmentDataUpdated = false;
		alignmentState = PRECISE_ALIGN_WAIT;
		break;

	case PRECISE_ALIGN_WAIT:
		//Wait for the image to be captured
		if (alignmentData.imageCaptured == false)
		{
			if (timeoutReached(&paCommandTimeout) == true)
			{
				timeoutReset(&paCommandTimeout);
				error(PA_ERROR);
			}
			break;
		}

		//Reset the image captured flag and proceed to the alignment
		alignmentData.imageCaptured = false;
		alignmentState = PRECISE_ALIGNMENT;
		break;

	case PRECISE_ALIGNMENT:
		//Wait for the alignment data to be updated
		if (alignmentData.alignmentDataUpdated == false)
		{
			if (timeoutReached(&paCommandTimeout) == true)
			{
				timeoutReset(&paCommandTimeout);
				error(PA_ERROR);
			}
			break;
		}
		//Reset the PA timeout
		timeoutReset(&paCommandTimeout);

		//NCP not found - ERROR
		if (alignmentData.ncpFound == false)
		{
			error(NCP_NOT_FOUND);
			break;
		}

		//Check whether the alignment error is below threshold or alignment tries reached the thershold
		if ((fabsf(alignmentData.azError) <= PRECISE_ALIGNMENT_THRESHOLD && fabsf(alignmentData.altError) <= PRECISE_ALIGNMENT_THRESHOLD) || alignmentData.alignmentTriesCounter >= ALIGNMENT_TRIES)
		{
			//If the alignment error is less than PRECISE_ALIGNMENT_THRESHOLD in both axes or a ALIGNMENT_TRIES are exceeded, the system proceeds to ALIGNMENT_DONE state
			alignmentData.alignmentTriesCounter = 0;
			alignmentState = ALIGNMENT_DONE;
			break;
		}

		if(alignmentData.azError != 0.0f)
		{
			stepperMove(&AZ_AxisMotor, alignmentData.azError/cosf(gpsData.latitude*(M_PI/180)), 5.0f);
		}
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
			//Set statusFlag
			statusFlags.polarAligned = true;

			//Reset the internal state machine
			alignmentState = ALIGN_WAITING_FOR_BUTTON_PRESS;
			alignmentData.alignmentTriesCounter = 0;
			alignmentData.alignmentDataUpdated = false;
			alignmentData.imageCaptured = false;
			alignmentData.corFound = false;
			corFirstImageCaptured = false;
			corSecondImageCaptured = false;

			stepperDisable(&ALT_AxisMotor);

			controlState = IDLE;
		}
		break;

	default:
		break;
	}
}


/*
 * --- IDLE STATE ---
 */
void handleIdle()
{
	uint8_t btn = readButtonDebounced();

	switch(idleState)
	{
	case IDLE_BTN_RELEASED:
		ledsBlink(LED_BLINK_PERIOD_SHORT);
		//If the button is pressed, jump to next state to wait for it to be released
		if (btn == PRESSED)
		{
			idleState = IDLE_BTN_PRESSED;
		}
		break;

	case IDLE_BTN_PRESSED:
		//If the button is released go to next state to switch the tracking ON
		if (btn == RELEASED)
		{
			idleState = IDLE_TRACKING_START;
		}
		break;

	case IDLE_TRACKING_START:
		ledsSet(false);
		trackingStart(&RA_AxisMotor);
		//Switch control state to TRACKING
		controlState = TRACKING;
		//Reset internal state
		idleState = IDLE_BTN_RELEASED;
		break;
	}
}

void handleTracking()
{
	uint8_t btn = readButtonDebounced();
	ledsRampUp(TRACKING_RAMP_UP_PERIOD);

	switch (trackingState)
	{
	case TRACKING_BTN_RELEASED:
		if (btn == PRESSED)
		{
			trackingState = TRACKING_BTN_PRESSED;
		}
		break;

	case TRACKING_BTN_PRESSED:
		if (btn == RELEASED)
		{
			trackingState = TRACKING_STOP;
		}
		break;

	case TRACKING_STOP:
		ledsSet(false);
		trackingStop(&RA_AxisMotor);
		//Switch control state to IDLE
		controlState = IDLE;
		//Reset internal state
		trackingState = TRACKING_BTN_RELEASED;
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
			//Disable the RA DEC position tracking
			coordinatesRaDec.trackingPosition = false;
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


/*
 * --- GOTO MOVEMENT STATE
 */

void handleGoto()
{
	//If both RA and DEC axes are not moving and there is no goto requested the control loop shall return to the previous state
	if(RA_AxisMotor.busy == false && DEC_AxisMotor.busy == false)
	{
		if(gotoRequest.gotoRequested == false)
		{
			coordinatesRaDec.trackingPosition = false;
			controlState = prevControlState;
			return;
		}
	}

	if(gotoRequest.gotoRequested == true && RA_AxisMotor.busy == false && DEC_AxisMotor.busy == false)
	{
		gotoRaDec(gotoRequest.raAngle, gotoRequest.decAngle);
		gotoRequest.gotoRequested = false;
		coordinatesRaDec.trackingPosition = true;
	}

}


/*
 * SHUTDOWN STATE
 */

void handleShutdown()
{
	uint8_t btn = readButtonDebounced();
	static uint32_t longPressStart = 0;
	static uint8_t shutdownLedStep = 0;

	switch (shutdownState)
	{

		case SHUTDOWN_CHECK_BUTTON_RELEASE:
			if (btn == RELEASED) //Released
			{
				shutdownState = SHUTDOWN_WAIT_FOR_LONG_PRESS;
				longPressStart = HAL_GetTick();
			}
			break;

		case SHUTDOWN_WAIT_FOR_LONG_PRESS:
			if ((HAL_GetTick() - longPressStart) > POWER_OFF_CANCLE_TIME)
			{
				//Timeout, shutdown canceled -> back to prevState
				controlState = prevControlState;
				//Reset shutdown state
				shutdownState = SHUTDOWN_CHECK_BUTTON_RELEASE;
				shutdownLedStep = 0;
				//Reset LEDs to OFF state
				ledsSet(false);
			}
			else if (btn == PRESSED) //Pressed again
			{
				shutdownState = SHUTDOWN_LONG_PRESS;
				longPressStart = HAL_GetTick();
				shutdownLedStep = 0;
			}
			break;

		case SHUTDOWN_LONG_PRESS:
			if (btn == RELEASED) //released too early -> reset
			{
				shutdownLedStep = 0;
				ledsSet(true);
				shutdownState = SHUTDOWN_CHECK_BUTTON_RELEASE;
				break;
			}
			//Check how long is the button pressed
			uint32_t heldTime = HAL_GetTick() - longPressStart;

			//Turns off LEDs one by one
			if ((heldTime / LED_DELAY) > shutdownLedStep && shutdownLedStep < 4)
			{
				shutdownLedStep++;
				switch (shutdownLedStep)
				{
					case 1:
						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
						break;
					case 2:
						HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
						break;
					case 3:
						HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
						break;
					case 4:
						HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
						break;
					default:
						ledsSet(false);
						break;
				}
			}

			if (heldTime >= HOLD_TIME)
			{
				shutdownState = SHUTDOWN_PROCESS;
			}
			break;

		case SHUTDOWN_PROCESS:
			if (btn == RELEASED)
			{
				//Send $SHUTDOWN command to RPi
				rpiShutdown();
				timeoutStart(&rpiShutdownTimeout, RPI_SHUTDOWN_TIME);

				//Move all axes to zero position
				stepperMove(&AZ_AxisMotor, -(AZ_AxisMotor.Position.angularPosition)*DEG, AZ_COARSE_SPEED);
				stepperMove(&ALT_AxisMotor, -(ALT_AxisMotor.Position.angularPosition)*DEG, ALT_COARSE_SPEED);
				stepperMove(&RA_AxisMotor, -(RA_AxisMotor.Position.angularPosition)*DEG, RA_COARSE_SPEED);
				stepperMove(&DEC_AxisMotor, -(DEC_AxisMotor.Position.angularPosition)*DEG, DEC_COARSE_SPEED);

				shutdownState = SHUTDOWN_FINISHED;
			}
			break;

		case SHUTDOWN_FINISHED:
			//Wait for all motors to home and for the RPi to shut down
			if (AZ_AxisMotor.busy || ALT_AxisMotor.busy || RA_AxisMotor.busy || DEC_AxisMotor.busy)
			{
				break;
			}
			if(!timeoutReached(&rpiShutdownTimeout))
			{
				break;
			}

			pwrButtonSet(false);
			ledsSet(false);

			//Cut off RPi power
			rpiPowerOff();
			//Disable all motors
			stepperDisable(&AZ_AxisMotor);
			stepperDisable(&ALT_AxisMotor);
			stepperDisable(&RA_AxisMotor);
			stepperDisable(&DEC_AxisMotor);

			uartSend(PC_UART_SRC, "Shutdown Successful!\r\n");
			controlState = LOW_POWER_IDLE;
			shutdownState = SHUTDOWN_CHECK_BUTTON_RELEASE;
			break;
	}
}

/*
 * --- FAULT STATE ---
 */

void handleFault()
{
	static GPIO_PinState faultLedState = GPIO_PIN_SET;
	static uint32_t faultLastLedTicks = 0;

	pwrButtonBlink(LED_BLINK_PERIOD_SHORT);

	if((HAL_GetTick() - faultLastLedTicks) >= LED_BLINK_PERIOD_SHORT)
	{
		faultLastLedTicks = HAL_GetTick();
		//Blink the 4 LEDs to show binary error code
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (errorCode & 0x08) ? faultLedState : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (errorCode & 0x04) ? faultLedState : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (errorCode & 0x02) ? faultLedState : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (errorCode & 0x01) ? faultLedState : GPIO_PIN_RESET);

		faultLedState = (faultLedState == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
	}
}

void error(errorCode_t err)
{
	errorCode = err;
	controlState = FAULT;
}

static void ledsBlink(uint32_t blinkPeriod)
{
	static uint32_t ledLastTicks = 0;

	if ((HAL_GetTick() - ledLastTicks) >= blinkPeriod)
	{
		ledLastTicks = HAL_GetTick();

		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	}
}

static void ledsRampUp(uint32_t ledPeriod)
{
	static uint8_t  currentLED   = 0;
	static uint32_t lastSwitchTicks = 0;

	if (HAL_GetTick() - lastSwitchTicks < ledPeriod)
	{
		return;
	}
	lastSwitchTicks = HAL_GetTick();

	//Reset all LEDs
	ledsSet(false);

	// Light up current one
	switch (currentLED)
	{
		case 0:
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			break;
		case 1:
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
			break;
	}

	currentLED = (currentLED + 1) % 4;
}


static void pwrButtonBlink(uint32_t ledPeriod)
{
	static uint32_t ledLastTicks = 0;

		if ((HAL_GetTick() - ledLastTicks) >= ledPeriod)
		{
			ledLastTicks = HAL_GetTick();

			HAL_GPIO_TogglePin(PWR_BTN_LED_GPIO_Port, PWR_BTN_LED_Pin);
		}
}


void ledsSet(bool state)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, state);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, state);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, state);
}

void pwrButtonSet(bool state)
{
	HAL_GPIO_WritePin(PWR_BTN_LED_GPIO_Port, PWR_BTN_LED_Pin, state);
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

//---State change/jump commands handling---

void homeCommand(UART_Source_t src)
{
	ledsSet(false);
	controlState = HOMING;
	//Check the button just in case, if its not pressed, proceed to HOMING
	homingState = HOMING_WAITING_FOR_BUTTON_RELEASE;
}

void calibCommand(UART_Source_t src)
{
	if (statusFlags.homed == false)
	{
		uartSend(src, "ERR: Not homed!\r\n");
		return;
	}
	ledsSet(false);
	controlState = CALIBRATION;
}

void alignCommand(UART_Source_t src)
{
	if (statusFlags.homed == false)
	{
		uartSend(src, "ERR: Not homed!\r\n");
		return;
	}
	if (statusFlags.calibrated == false)
	{
		uartSend(src, "ERR: Not calibrated!\r\n");
		return;
	}
	if (statusFlags.gpsFixed == false)
	{
		uartSend(src, "ERR: GPS not fixed!\r\n");
		return;
	}
	if (statusFlags.rpiRDY == false)
	{
		uartSend(src, "ERR: RPi not ready!\r\n");
		return;
	}

	ledsSet(false);
	controlState = ALIGNMENT;
	alignmentState = ALIGN_WAITING_FOR_BUTTON_RELEASE;

}

void patestCommand(UART_Source_t src)
{
	alignmentData.altAngle = 41.0f;
	statusFlags.gpsFixed = true;


	if (statusFlags.homed == false)
	{
		uartSend(src, "ERR: Not homed!\r\n");
		return;
	}
	if (statusFlags.calibrated == false)
	{
		uartSend(src, "ERR: Not calibrated!\r\n");
		return;
	}
	if (statusFlags.gpsFixed == false)
	{
		uartSend(src, "ERR: GPS not fixed!\r\n");
		return;
	}
	if (statusFlags.rpiRDY == false)
	{
		uartSend(src, "ERR: RPi not ready!\r\n");
		return;
	}

	ledsSet(false);
	controlState = ALIGNMENT;
	alignmentState = ROUGH_ALIGNMENT_ALT;

}

void trackingCommand(UART_Source_t src, bool trackingStart)
{
	if (trackingStart == true)
	{
		if(controlState != IDLE)
		{
			uartSend(src, "ERR:NOT IN IDLE\r\n");
			return;
		}
		//Switch idle state to IDLE TRACKING START -> handles tracking start
		idleState = IDLE_TRACKING_START;
		uartSend(src, "TRACKING ON\r\n");
	}
	else
	{
		if(controlState != TRACKING)
		{
			uartSend(src, "ERR:NOT TRACKING\r\n");
			return;
		}
		//Switch tracking state to TRACKING STOP -> handles tracking stop
		trackingState = TRACKING_STOP;
		uartSend(src, "TRACKING OFF\r\n");
	}

}

void idleCommand(UART_Source_t src)
{
	ledsSet(false);
	controlState = IDLE;
	idleState = IDLE_BTN_RELEASED;
}

static void resetStates()
{
	//States reset
	lowPowerIdleButtonState = BTN_SEQ_WAIT_FIRST_PRESS;
	WarmUpState = TIMEOUT_START;
	gpsWarmUpState = CHECK_AND_CONFIG_GPS;
	rpiWarmUpState = WAITING_FOR_RPI_BOOT;
	alignmentState = ALIGN_WAITING_FOR_BUTTON_PRESS;
	calState = CAL_IDLE;

	homingState = HOMING_WAITING_FOR_BUTTON_PRESS;
	ALT_AxisMotor.homing_state = AXIS_HOMING_IDLE;
	ALT_AxisMotor.homing = false;
	AZ_AxisMotor.homing_state = AXIS_HOMING_IDLE;
	AZ_AxisMotor.homing = false;
	RA_AxisMotor.homing_state = AXIS_HOMING_IDLE;
	RA_AxisMotor.homing = false;
	DEC_AxisMotor.homing_state = AXIS_HOMING_IDLE;
	DEC_AxisMotor.homing = false;

	statusFlags.homed = false;
	statusFlags.calibrated = false;
	statusFlags.gpsFixed = false;
	statusFlags.gpsOK = false;
	statusFlags.magOK = false;
	statusFlags.rpiRDY = false;
	statusFlags.polarAligned = false;

	alignmentData.alignmentDataUpdated = false;
	alignmentData.corFound = 0;
	alignmentData.imageCaptured = false;
	alignmentData.ncpFound = false;
	alignmentData.polarisFound = false;
}





