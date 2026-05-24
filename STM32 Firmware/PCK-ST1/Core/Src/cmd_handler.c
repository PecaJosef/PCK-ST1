/*
 * cmd_handler.c
 *
 *  Created on: Oct 7, 2025
 *      Author: pecka
 */

#include "cmd_handler.h"
#include "string.h"
#include "stdio.h"
#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include "control_loop.h"
#include "telemetry.h"
#include "astro.h"
#include "camera.h"


// Buffers for PC and RPi commands
static uint8_t pc_buffer[CMD_BUFFER_SIZE];
static uint8_t rpi_buffer[CMD_BUFFER_SIZE];
static uint16_t pc_index = 0;
static uint16_t rpi_index = 0;

static uint8_t pc_cmd_ready = 0;
static uint8_t rpi_cmd_ready = 0;

static void executeCommand(char *cmd, UART_Source_t src);

static void moveParsing(char **saveptr, UART_Source_t src);

static void gotoParsing(char **saveptr, UART_Source_t src);

static void rpiParsing(char **saveptr, UART_Source_t src);

static void exposureParsing(char **saveptr, UART_Source_t src);

static void polarAlignmentParsing (char **saveptr, UART_Source_t src);

static void trackingParsing(char **saveptr, UART_Source_t src);

static void calibParsing(char **saveptr, UART_Source_t src);

static void isetParsing(char **saveptr, UART_Source_t src);

static void telemetryParsing(char **saveptr, UART_Source_t src);

void commandHandler(void)
{
    if (pc_cmd_ready) {
        executeCommand((char *)pc_buffer, PC_UART_SRC);
        pc_cmd_ready = 0;

    }
    if (rpi_cmd_ready) {
        executeCommand((char *)rpi_buffer, RPI_UART_SRC);
        rpi_cmd_ready = 0;
    }
}

void commandHandler_Init(void)
{
    pc_index = rpi_index = 0;
    pc_cmd_ready = rpi_cmd_ready = 0;
}

void CMD_UART_StoreByte(UART_Source_t src, uint8_t byte)
{
    uint8_t *buf;
    uint16_t *idx;
    uint8_t *ready;

    if (src == PC_UART_SRC)
    {
    	buf = pc_buffer;
    	idx = &pc_index;
    	ready = &pc_cmd_ready;
    }
    else
    {
    	buf = rpi_buffer;
    	idx = &rpi_index;
    	ready = &rpi_cmd_ready;
    }

    if (*ready) return; //previous command not processed yet

    buf[*idx] = byte;

    if (byte == '\r' || byte == '\n' || *idx >= CMD_BUFFER_SIZE)
    {
        buf[*idx] = '\0';
        *ready = 1;
        *idx = 0;
    }
    else
    {
    	(*idx)++;
    }
}

void uartSend(UART_Source_t src, const char *msg)
{
    if (src == PC_UART_SRC)
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    else if (src == RPI_UART_SRC)
        HAL_UART_Transmit(&huart5, (uint8_t*)msg, strlen(msg), 100);
    else
	{
		return;
	}
}

static void executeCommand(char *cmd, UART_Source_t src)
{
	char *token;
	char *saveptr;

	//Forward info messages from RPi starting with # directly to PC/controller UART
	if (cmd[0] == '#')
	{
		//Forward only the info messages from RPi
		if (src == RPI_UART_SRC)
		{

			uartSend(PC_UART_SRC, "RPI:");
			uartSend(PC_UART_SRC, cmd);
			uartSend(PC_UART_SRC, "\r\n");
		}
		return;
	}

	if (cmd[0] != '$')
	{
		uartSend(src, "ERR:FORMAT\r\n");
		return;
	}

	token = strtok_r(cmd, ":", &saveptr);
	if (!token)
	{
		return;
	}

	if (strcmp(token, "$ECHO")==0)
	{
		uartSend(src, "#ECHO\r\n");
	}
	else if(strcmp(token, "$MOVE")==0)
	{
		//Move commands parsing
		moveParsing(&saveptr, src);
	}
	else if(strcmp(token, "$GOTO")==0)
	{
		//GOTO command parsing
		gotoParsing(&saveptr, src);
	}
	else if(strcmp(token, "$IMG")==0)
	{
		//Parsing the image capture command
		exposureParsing(&saveptr, src);
	}
	else if(strcmp(token, "$PA")==0)
	{
		//Polar alignment commands processing
		polarAlignmentParsing(&saveptr, src);
	}
	else if (strcmp(token, "$RPI")==0)
	{
		//RPI commands processing and forwarding
		rpiParsing(&saveptr, src);
	}
	else if(strcmp(token, "$TRACKING")==0)
	{
		trackingParsing(&saveptr, src);
	}
	else if(strcmp(token, "$CALIB")==0)
	{
		calibParsing(&saveptr, src);
	}
	else if(strcmp(token, "$RDY")==0 && src == RPI_UART_SRC)
	{
		//Set status flag rpiRDY
		statusFlags.rpiRDY = true;
		//sends response from RPi to PC
		uartSend(PC_UART_SRC, "RPi has started\r\n");
	}
	else if (strcmp(token, "$PWROFF")==0 && src == RPI_UART_SRC)
	{
		//The powering off command from RPi -> reset the rpiRDY flag
		statusFlags.rpiRDY = false;
		//sends response from RPi to PC
		uartSend(PC_UART_SRC, "RPI Powered off\r\n");
	}
	else if(strcmp(token, "$TEL")==0)
	{
		telemetryParsing(&saveptr, src);
	}
	else if(strcmp(token, "$HOME")==0)
	{
		homeCommand(src);
	}
	else if(strcmp(token, "$CALIBRATE")==0)
	{
		calibCommand(src);
	}
	else if(strcmp(token, "$ALIGN")==0)
	{
		alignCommand(src);
	}
	else if(strcmp(token, "$IDLE")==0)
	{
		idleCommand(src);
	}
	else if(strcmp(token, "$PATEST")==0)
	{
		patestCommand(src);
	}
	else if(strcmp(token, "$ISET")==0)
	{
		isetParsing(&saveptr, src);
	}
	else if(strcmp(token, "$DIS")==0)
	{
		stepperDisable(&ALT_AxisMotor);
		stepperDisable(&AZ_AxisMotor);
		stepperDisable(&RA_AxisMotor);
		stepperDisable(&DEC_AxisMotor);
	}
	else if(strcmp(token, "$EN")==0)
	{
		stepperEnable(&ALT_AxisMotor);
		stepperEnable(&AZ_AxisMotor);
		stepperEnable(&RA_AxisMotor);
		stepperEnable(&DEC_AxisMotor);
	}
	else if(strcmp(token, "$ERR")==0)
	{
		uartSend(PC_UART_SRC, "RPI:ERR\r\n");
	}
	else
	{
		if (src != PC_UART_SRC)
		{
			uartSend(PC_UART_SRC, "RPI:ERR:UNKNOWN\r\n");
			return;
		}
		uartSend(src, "ERR:UNKNOWN\r\n");
	}

}

static void moveParsing(char **saveptr, UART_Source_t src)
{
	//If some control state disables move, return early
	if(statusFlags.moveEnabled == false)
	{
		uartSend(src, "ERR: MOVEMENT DISABLED!\r\n");
		return;
	}

	char *command;
	char *saveptr_cmd;

	command  = strtok_r(NULL, "$",saveptr);

	if(command == NULL)
	{
		uartSend(src, "#ERR:FORMAT\r\n");
		return;
	}

	if(strcmp(command, "PARK")==0)
	{
		//Move the RA and DEX axes to their reference position using move requests
		DEC_MoveRequest.angle = -DEC_AxisMotor.Position.angularPosition*DEG;
		DEC_MoveRequest.speed = DEC_COARSE_SPEED;
		DEC_MoveRequest.moveRequested = true;

		RA_MoveRequest.angle = -RA_AxisMotor.Position.angularPosition*DEG;
		RA_MoveRequest.speed = RA_COARSE_SPEED;
		RA_MoveRequest.moveRequested = true;

		//RA DEC coordinates must be updated during during parking - enable position tracking
		coordinatesRaDec.trackingPosition = true;

		//Changes state to MOVE and prevState to current state
		if(controlState != AXIS_MOVING)
		{
			prevControlState = controlState; //Keep the previous control state intact if there are multiple $MOVE commands while in AXIS_MOVING state
			controlState = AXIS_MOVING;
		}

		return;
	}

	while(command != NULL)
	{

		char *axis = strtok_r(command, ":", &saveptr_cmd);
		char *angle = strtok_r(NULL, ":",&saveptr_cmd);
		char *speed = strtok_r(NULL, ":",&saveptr_cmd);

		if(axis == NULL || angle == NULL || speed == NULL)
		{
			uartSend(src, "#ERR:FORMAT\r\n");
			return;
		}

		if(strcmp(axis, "AZ")==0)
		{
			if(AZ_MoveRequest.moveRequested==false)
			{
				AZ_MoveRequest.angle = atof(angle)*DEG;
				AZ_MoveRequest.speed = atof(speed);
				AZ_MoveRequest.moveRequested = true;
			}
			else
			{
				uartSend(src, "ERR:AXIS_BUSY\r\n");
			}
		}
		else if(strcmp(axis, "ALT")==0)
		{
			if(ALT_MoveRequest.moveRequested==false)
			{
				ALT_MoveRequest.angle = atof(angle)*DEG;
				ALT_MoveRequest.speed = atof(speed);
				ALT_MoveRequest.moveRequested = true;
			}
			else
			{
				uartSend(src, "ERR:AXIS_BUSY\r\n");
			}
		}
		else if(strcmp(axis, "DEC")==0)
		{
			if(DEC_MoveRequest.moveRequested==false)
			{
				DEC_MoveRequest.angle = atof(angle)*DEG;
				DEC_MoveRequest.speed = atof(speed);
				DEC_MoveRequest.moveRequested = true;
			}
			else
			{
				uartSend(src, "ERR:AXIS_BUSY\r\n");
			}
		}
		else if(strcmp(axis, "RA")==0)
		{
			if(RA_MoveRequest.moveRequested==false)
			{
				RA_MoveRequest.angle = atof(angle)*DEG;
				RA_MoveRequest.speed = atof(speed);
				RA_MoveRequest.moveRequested = true;
			}
			else
			{
				uartSend(src, "ERR:AXIS_BUSY\r\n");
			}
		}
		else
		{
			//Wrong Axis name -> Error
			uartSend(src, "#ERR:AXIS_NAME\r\n");
			return;
		}
		command  = strtok_r(NULL, "$",saveptr);
	}
	//Changes state to MOVE and prevState to current state
	if(controlState != AXIS_MOVING)
	{
		prevControlState = controlState; //Keep the previous control state intact if there are multiple $MOVE commands while in AXIS_MOVING state
		controlState = AXIS_MOVING;
	}
}

// GOTO Command Parsing

static void gotoParsing(char **saveptr, UART_Source_t src)
{
	//If some control state disables move, return early
	if(statusFlags.moveEnabled == false)
	{
		uartSend(src, "ERR: MOVEMENT DISABLED!\r\n");
		return;
	}
	//Return if the PCK-ST1 is not polar aligned
	if(statusFlags.polarAligned == false)
	{
		uartSend(src, "ERR:NOT ALIGNED\r\n");
		return;
	}

	//The command has the following format
	//$GOTO:hh:mm:ss:dd:mm:ss

	char *raHours_c = strtok_r(NULL, ":", saveptr);
	char *raMinutes_c = strtok_r(NULL, ":", saveptr);
	char *raSeconds_c = strtok_r(NULL, ":", saveptr);
	char *decDegrees_c = strtok_r(NULL, ":", saveptr);
	char *decArcmin_c = strtok_r(NULL, ":", saveptr);
	char *decArcsec_c = strtok_r(NULL, ":", saveptr);

	if (raHours_c == NULL || raMinutes_c == NULL || raSeconds_c == NULL || decDegrees_c == NULL || decArcmin_c == NULL || decArcsec_c == NULL)
	{
		uartSend(src, "ERR:FORMAT\r\n");
		return;
	}

	int16_t raHours = atoi(raHours_c);
	int16_t raMinutes = atoi(raMinutes_c);
	int16_t raSeconds = atoi(raSeconds_c);

	int16_t decDegrees = atoi(decDegrees_c);
	int16_t decArcmin = atoi(decArcmin_c);
	int16_t decArcsec = atoi(decArcsec_c);

	if(raHours >= 24 || raHours < 0 || decDegrees > 90 || decDegrees < -90)
	{
		uartSend(src, "ERR:INVALID DATA\r\n");
		return;
	}

	gotoRequest.raAngle = raHours*15.0f + raMinutes*15.0f/60.0f + raSeconds*15.0f/3600.0f;
	gotoRequest.decAngle = decDegrees + decArcmin/60.0f + decArcsec/3600.0f;

	gotoRequest.gotoRequested = true;

	if (controlState != GOTO)
	{
		prevControlState = controlState;
		controlState = GOTO;
	}

}


static void rpiParsing(char **saveptr, UART_Source_t src)
{
	char *rpiCommand;

	rpiCommand = strtok_r(NULL,":",saveptr); //Gets the RPI command type
	if(strcmp(rpiCommand, "ECHO")==0)
	{
		uartSend(RPI_UART_SRC, "$ECHO\r\n");
		uartSend(PC_UART_SRC, "Sent ECHO to RPi\r\n");
	}
	else if(strcmp(rpiCommand, "SHUTDOWN")==0)
	{
		rpiShutdown();
	}
	else if(strcmp(rpiCommand, "ON")==0)
	{
		rpiPowerOn();
	}
	else if(strcmp(rpiCommand, "OFF")==0)
	{
		if(statusFlags.rpiRDY == true)
		{
			uartSend(src, "RPI STILL RUNNING\r\n");
			return;
		}
		rpiPowerOff();
	}
	else if(strcmp(rpiCommand, "ALIGN")==0)
	{
		uartSend(RPI_UART_SRC, "$PA:ALIGN\r\n");
		uartSend(PC_UART_SRC, "ALIGN command sent\r\n");
	}
	else if(strcmp(rpiCommand, "CAPTURE")==0)
	{
		char *exposure = strtok_r(NULL, ":", saveptr);
		char *filename = strtok_r(NULL, ":", saveptr);

		if (exposure == NULL || filename == NULL)
		{
			uartSend(src, "ERR:FORMAT\r\n");
			return;
		}

		char buffer[64];

		int len = snprintf(buffer, sizeof(buffer), "$CAPTURE:%s:%s\r\n", exposure, filename);

		if (len > 0 && len < (int)sizeof(buffer))
		{
			uartSend(RPI_UART_SRC, buffer);
			uartSend(PC_UART_SRC, "#CAPTURE CMD Sent\r\n");
		}
		else
		{
			uartSend(src, "ERR:DATA SIZE\r\n");
		}
	}
	else
	{
		uartSend(src, "$ERR:UNKNOWN\r\n");
	}

}

//Image command parsing

static void exposureParsing(char **saveptr, UART_Source_t src)
{
	char *imgExposure_c;
	char *imgCount_c;

	imgExposure_c = strtok_r(NULL, ":", saveptr);
	imgCount_c = strtok_r(NULL, ":", saveptr);

	if(imgExposure_c == NULL || imgCount_c == NULL)
	{
		uartSend(src, "ERR:FORMAT\r\n");
		return;
	}

	float imgExposure = atof(imgExposure_c);
	uint16_t imgCount = atoi(imgCount_c);

	if(imgExposure <= 0 || imgCount <= 0)
	{
		uartSend(src, "ERR:FORMAT\r\n");
		return;
	}
	//Capture images based on the exposure value and the number of images
	captureImage(imgExposure, imgCount);
}

static void polarAlignmentParsing (char **saveptr, UART_Source_t src)
{
	char *command;

	command = strtok_r(NULL, ":", saveptr);

	if(strcmp(command, "DEV")==0)
	{
		//Gets the PA data from RPi - $PA:DEV:A:B:X.xxxxx:Y.yyyyy,
		//where A marks if the NCP was found successfully, B marks if the Polaris was found, X value marks deviation in AZ axis and Y value in ALT axis
		char *ncpFound_c = strtok_r(NULL, ":", saveptr);
		char *polarisFound_c = strtok_r(NULL, ":", saveptr);
		char *azError_c = strtok_r(NULL, ":", saveptr);
		char *altError_c = strtok_r(NULL, ":", saveptr);
		char *raAngle_c = strtok_r(NULL, ":", saveptr);

		//Return if part of the command is missing
		if (polarisFound_c == NULL || ncpFound_c == NULL || azError_c == NULL || altError_c == NULL)
		{
			uartSend(src, "ERR:FORMAT\n\r");
			return;
		}
		//Convert the ncpFound_c to bool while checking the formating
		if (ncpFound_c[1] == '\0')
		{
			alignmentData.ncpFound = (ncpFound_c[0] == '1');
		}
		else
		{
			uartSend(src, "ERR:FORMAT\n\r");
			return;
		}
		//Convert the polarisFound_c to bool while checking the formating
		if (polarisFound_c[1] == '\0')
		{
			alignmentData.polarisFound = (polarisFound_c[0] == '1');
		}
		else
		{
			uartSend(src, "ERR:FORMAT\n\r");
			return;
		}
		//Convert the error values to floats
		if (altError_c != NULL)
		{
			alignmentData.altError = atof(altError_c);
		}
		else
		{
			uartSend(src, "ERR:FORMAT\n\r");
			return;
		}
		if (azError_c != NULL)
		{
			alignmentData.azError = atof(azError_c);
		}
		else
		{
			uartSend(src, "ERR:FORMAT\n\r");
			return;
		}
		if (raAngle_c != NULL)
		{
			alignmentData.raAngle = atof(raAngle_c);
		}
		else
		{
			uartSend(src, "ERR:FORMAT\n\r");
			return;
		}

		alignmentData.alignmentDataUpdated = true;

		//Send the output over UART to PC/controller
		char paDataBuffer[128];
		sprintf(paDataBuffer,	"NCP found:	%d\r\nPoalris found:	%d\r\nAZ error:	%.5f\r\nALT error:	%.5f\r\nRA angle: %.2f\r\n",
								alignmentData.ncpFound, alignmentData.polarisFound, alignmentData.azError, alignmentData.altError, alignmentData.raAngle);

		uartSend(PC_UART_SRC, paDataBuffer);


		return;
	}
	else if(strcmp(command, "CAPTURED")==0)
	{
		alignmentData.imageCaptured = true;
		uartSend(PC_UART_SRC, "CAPTURED\r\n");
	}
	else if(strcmp(command, "COR")==0)
	{
		char *cor_cmd = strtok_r(NULL, ":", saveptr);

		if(strcmp(cor_cmd, "DONE")==0)
		{
			alignmentData.corFound = 1;
		}
		else if(strcmp(cor_cmd, "FAIL")==0)
		{
			alignmentData.corFound = -1;
		}
	}
	else
	{
		uartSend(src, "ERR:UNKNOWN\n\r");
	}
}


static void trackingParsing(char **saveptr, UART_Source_t src)
{
	char *command;

	command = strtok_r(NULL, ":", saveptr);

	if(strcmp(command, "START")==0)
	{
		trackingCommand(src, true);
	}
	else if (strcmp(command, "STOP")==0)
	{
		trackingCommand(src, false);
	}
	else
	{
		uartSend(src, "ERROR:FORMAT\r\n");
	}
}


static void calibParsing(char **saveptr, UART_Source_t src)
{
	char *command;

	command = strtok_r(NULL, ":", saveptr);

	if(strcmp(command, "ON")==0)
	{
		statusFlags.calibForceEnable = true;
		uartSend(src, "CALIBRATION ENABLED\r\n");
	}
	else if (strcmp(command, "OFF")==0)
	{
		statusFlags.calibForceEnable = false;
		uartSend(src, "CALIBRATION DISABLED\r\n");
	}
	else
	{
		uartSend(src, "ERROR:FORMAT\r\n");
	}

}

static void telemetryParsing(char **saveptr, UART_Source_t src)
{
	char *telemetryCmd;
	telemetryCmd = strtok_r(NULL, ":",saveptr);

	if(telemetryCmd == NULL)
	{
		getTelemetry();
	}
	else if(strcmp(telemetryCmd, "FULL")==0)
	{
		getFullTelemetry();
	}
	else
	{
		uartSend(src, "ERR:FORMAT\r\n");
	}
}

static void isetParsing(char **saveptr, UART_Source_t src)
{
	char *val;
	val = strtok_r(NULL, ":",saveptr);

	if(val == NULL)
	{
		uartSend(src, "ERR:FORMAT\r\n");
		return;
	}
	float percentage = atof(val);
	setCurrent(percentage);
}


void rpiShutdown()
{
	uartSend(RPI_UART_SRC, "$SHUTDOWN");
}

void rpiPowerOn()
{
	HAL_GPIO_WritePin(RPI_PWR_EN_GPIO_Port, RPI_PWR_EN_Pin, 1);
}

void rpiPowerOff()
{
	HAL_GPIO_WritePin(RPI_PWR_EN_GPIO_Port, RPI_PWR_EN_Pin, 0);
}

