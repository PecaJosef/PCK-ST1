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


// Buffers for PC and RPi commands
static uint8_t pc_buffer[CMD_BUFFER_SIZE];
static uint8_t rpi_buffer[CMD_BUFFER_SIZE];
static uint16_t pc_index = 0;
static uint16_t rpi_index = 0;

static uint8_t pc_cmd_ready = 0;
static uint8_t rpi_cmd_ready = 0;

static void executeCommand(char *cmd, UART_Source_t src);

static void moveParsing(char **saveptr, UART_Source_t src);

static void rpiParsing(char **saveptr, UART_Source_t src);

static void polarAlignmentParsing (char **saveptr, UART_Source_t src);

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
    	buf = pc_buffer; idx = &pc_index; ready = &pc_cmd_ready;
    }
    else
    {
    	buf = rpi_buffer; idx = &rpi_index; ready = &rpi_cmd_ready;
    }

    if (*ready) return; // previous command not yet processed

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

static void executeCommand(char *cmd, UART_Source_t src)
{
	char *token;
	char *saveptr;
	//uartSend(PC_UART_SRC, "H\r\n");

	token = strtok_r(cmd, ":", &saveptr);
	if (!token) return;

	if (strcmp(token, "$ECHO")==0)
	{
		uartSend(src, "ECHO\r\n");
	}
	else if(strcmp(token, "$MOVE")==0)
	{
		//Move commands parsing
		moveParsing(&saveptr, src);
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


	else if(strcmp(token, "#ECHO")==0 && src == RPI_UART_SRC)
	{
		uartSend(PC_UART_SRC, "#RPI:ECHO\r\n"); //sends response from RPi (ECHO) to PC
	}
	else if(strcmp(token, "#RDY")==0 && src == RPI_UART_SRC)
	{
		uartSend(PC_UART_SRC, "#RPI:RDY\r\n"); //sends response from RPi (ECHO) to PC
	}

	else if(strcmp(token, "$DIS")==0)
	{
		stepperDisable(&ALT_AxisMotor);
		stepperDisable(&AZ_AxisMotor);
		stepperDisable(&RA_AxisMotor);
	}
	else if(strcmp(token, "$EN")==0)
		{
			stepperEnable(&ALT_AxisMotor);
			stepperEnable(&AZ_AxisMotor);
			stepperEnable(&RA_AxisMotor);
		}
	else
	{
		uartSend(src, "ERR:UNKNOWN\r\n");
		uartSend(PC_UART_SRC, "ERR:UNKNOWN\r\n");

		//For debugging only - comment out for normal use
		//#define CMD_DEBUG

		#ifdef CMD_DEBUG
		for(uint8_t i=0;i<10;i++)
		{
			printf("%d -> %d\r\n",i,*(token+i));
			HAL_Delay(100);
			if(*(token+i)=='\0')
			{
				break;
			}
		}
		#endif
	}

}


static void moveParsing(char **saveptr, UART_Source_t src)
{
	char *command;
	char *saveptr_cmd;

	command  = strtok_r(NULL, "$",saveptr);

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
				//CMD_Send(UART_SRC_PC, "AZ\r\n");
				//printf("Move AZ angle:%.5f speed:%.5f\r\n",AZ_MoveRequest.angle, AZ_MoveRequest.speed);
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
				//CMD_Send(UART_SRC_PC, "ALT\r\n");
				//printf("Move ALT angle:%.5f speed:%.5f\r\n",ALT_MoveRequest.angle, ALT_MoveRequest.speed);
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


static void rpiParsing(char **saveptr, UART_Source_t src)
{
	char *command;

	command = strtok_r(NULL,":",saveptr); //Gets the RPI command type
	if(strcmp(command, "ECHO")==0)
	{
		uartSend(RPI_UART_SRC, "$ECHO\r\n");
		uartSend(PC_UART_SRC, "Sent ECHO to RPi\r\n");
	}
	else if(strcmp(command, "SHUTDOWN")==0)
	{
		rpiShutdown();
	}
	else if(strcmp(command, "START")==0)
	{
		rpiPowerOn();
	}
	else if(strcmp(command, "ALIGN")==0)
	{
		uartSend(RPI_UART_SRC, "$ALIGN\r\n");
		uartSend(PC_UART_SRC, "#ALIGN CMD Sent\r\n");
	}
	else
	{
		uartSend(src, "$ERR:UNKNOWN\r\n");
	}

}

void polarAlignmentParsing (char **saveptr, UART_Source_t src)
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
		alignmentData.altError = atof(altError_c);
		alignmentData.azError = atof(azError_c);
		alignmentData.alignmentDataUpdated = true;
		return;
	}

	else if(strcmp(command, "START")==0)
	{
		//Start Polar Alignment process manually
	}
	else if(strcmp(command, "ABORT")==0)
	{

	}
	else
	{
		uartSend(src, "ERR:UNKNOWN\n\r");
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

