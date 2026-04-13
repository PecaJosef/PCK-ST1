/*
 * control_loop.h
 *
 *  Created on: Aug 25, 2025
 *      Author: pecka
 */

#ifndef INC_CONTROL_LOOP_H_
#define INC_CONTROL_LOOP_H_

#define RA_FINE_SPEED 2.0f
#define RA_COARSE_SPEED 2.0f
#define DEC_FINE_SPEED 5.0f
#define DEC_COARSE_SPEED 10.0f
#define ALT_FINE_SPEED 7.5f
#define ALT_COARSE_SPEED 10.0f
#define AZ_COARSE_SPEED 10.0f

#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "gps.h"
#include "button.h"
#include "wmm.h"
#include "stepper.h"
#include "cmd_handler.h"

//Status flags mostly for control via commands
typedef struct {
	bool homed;
	bool calibrated;
	bool gpsOK;
	bool gpsFixed;
	bool magOK;
	bool rpiRDY;
	bool moveEnabled;
	bool calibForceEnable;
	bool tracking;
	bool polarAligned;
	bool fault;

}StatusFlags_t;

typedef enum {
	LOW_POWER_IDLE, //Waiting for button press to power up, initial state
	WARMING_UP, //Powering up RPi, waiting for "OK" from RPi, waiting for GPS fix, calculating magnetic declination
	HOMING, //Axis homing
	CALIBRATION, //Calibrates the magnetometer or loads calibration data
	ALIGNMENT, //Pointing to NCP, communication with RPi - taking photo
	IDLE, //Not moving, waiting for action (commands), can be powered off while in idle
	AXIS_MOVING, //Moving with any axis
	GOTO, //GOTO movement
	TRACKING, //Moving RA axis to counter Earth's rotation
	FAULT, //Error state - waiting for reset
	SHUTDOWN, //Shutting down state - takes care of safe shut down of RPi and the whole system
}ControlState_t;

typedef enum {
	NONE = 0,
	POLARIS_NOT_FOUND = 1,
	NCP_NOT_FOUND = 2,
	COR_ERROR = 3,
	PA_ERROR = 4,
	RPI_ERROR = 5,
	GPS_ERROR = 6,
	MAG_ERROR = 7,
	WARMUP_TIMEOUT = 8,


}errorCode_t;

typedef struct {
	float altAngle;
	float declination;
	bool alignmentDataUpdated;
	bool polarisFound;
	bool ncpFound;
	int8_t corFound;
	bool imageCaptured;
	float azError;
	float altError;
	float raAngle;
	uint8_t alignmentTriesCounter;
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

extern Align_t alignmentData;

extern errorCode_t errorCode;

extern MoveRequest_t AZ_MoveRequest;
extern MoveRequest_t ALT_MoveRequest;
extern MoveRequest_t DEC_MoveRequest;
extern MoveRequest_t RA_MoveRequest;

extern StatusFlags_t statusFlags;

void controlLoop();

void handleGoto();

void handleMoving();

void handleAligning();

void handleLowPowerIdle();

void handleWarmingUp();

void handleHoming();

void handleCalibration();

void handleShutdown();

void handleIdle();

void handleFault();

void error(errorCode_t err);

void endstopReached(StepperMotor_t *Axis, float fineSpeed);

void ledsSet(bool state);

void pwrButtonSet(bool state);

//---Command handling---

void homeCommand(char **saveptr, UART_Source_t src);
void calibCommand(char **saveptr, UART_Source_t src);
void alignCommand(char **saveptr, UART_Source_t src);

void patestCommand(char **saveptr, UART_Source_t src);

#endif /* INC_CONTROL_LOOP_H_ */
