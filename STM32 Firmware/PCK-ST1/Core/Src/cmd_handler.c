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

static void CMD_Execute(char *cmd, UART_Source_t src);

static void moveParsing(char **saveptr, UART_Source_t src);

static void rpiParsing(char **saveptr, UART_Source_t src);

void CMD_Handler_Process(void)
{
    if (pc_cmd_ready) {
        CMD_Execute((char *)pc_buffer, UART_SRC_PC);
        pc_cmd_ready = 0;
    }
    if (rpi_cmd_ready) {
        CMD_Execute((char *)rpi_buffer, UART_SRC_RPI);
        rpi_cmd_ready = 0;
    }
}

void CMD_Handler_Init(void)
{
    pc_index = rpi_index = 0;
    pc_cmd_ready = rpi_cmd_ready = 0;
}

void CMD_UART_StoreByte(UART_Source_t src, uint8_t byte)
{
    uint8_t *buf;
    uint16_t *idx;
    uint8_t *ready;

    if (src == UART_SRC_PC)
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

static void CMD_Execute(char *cmd, UART_Source_t src)
{
	char *token;
	char *saveptr;

	token = strtok_r(cmd, ":", &saveptr);
	if (!token) return;

	if (strcmp(token, "$ECHO")==0)
	{
		CMD_Send(src, "ECHO\r\n");
	}
	else if(strcmp(token, "$MOVE")==0)
	{
		moveParsing(&saveptr, src);
	}


	else if(strcmp(token, "ECHO")==0 && src == UART_SRC_RPI)
	{
		CMD_Send(UART_SRC_PC, "#RPI:ECHO\r\n"); //sends response from RPi (ECHO) to PC
	}
	else if (strcmp(token, "$RPI")==0)
	{
		//Jump to RPI command sub processing
		rpiParsing(&saveptr, src);
	}
	else
	{
		CMD_Send(src, "ERR:Unknown command\r\n");

		//For debugging only - comment out for normal use
		#define CMD_DEBUG

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
			CMD_Send(src, "#ERR:Wrong format\r\n");
			return;
		}

		if(strcmp(axis, "AZ")==0)
		{
			if(AZ_MoveRequest.moveRequested==false)
			{
				AZ_MoveRequest.angle = atof(angle);
				AZ_MoveRequest.speed = atof(speed);
				//CMD_Send(UART_SRC_PC, "AZ\r\n");
				//printf("Move AZ angle:%.5f speed:%.5f\r\n",AZ_MoveRequest.angle, AZ_MoveRequest.speed);
				AZ_MoveRequest.moveRequested = true;
			}
			else
			{
				CMD_Send(src, "ERR:Axis busy\r\n");
			}
		}
		else if(strcmp(axis, "EL")==0)
		{
			if(EL_MoveRequest.moveRequested==false)
			{
				EL_MoveRequest.angle = atof(angle);
				EL_MoveRequest.speed = atof(speed);
				//CMD_Send(UART_SRC_PC, "EL\r\n");
				//printf("Move EL angle:%.5f speed:%.5f\r\n",EL_MoveRequest.angle, EL_MoveRequest.speed);
				EL_MoveRequest.moveRequested = true;
			}
			else
			{
				CMD_Send(src, "ERR:Axis busy\r\n");
			}
		}
		else if(strcmp(axis, "DEC")==0)
		{
			if(DEC_MoveRequest.moveRequested==false)
			{
				DEC_MoveRequest.angle = atof(angle);
				DEC_MoveRequest.speed = atof(speed);
				DEC_MoveRequest.moveRequested = true;
			}
			else
			{
				CMD_Send(src, "ERR:Axis busy\r\n");
			}
		}
		else if(strcmp(axis, "RA")==0)
		{
			if(RA_MoveRequest.moveRequested==false)
			{
				RA_MoveRequest.angle = atof(angle);
				RA_MoveRequest.speed = atof(speed);
				RA_MoveRequest.moveRequested = true;
			}
			else
			{
				CMD_Send(src, "ERR:Axis busy\r\n");
			}
		}
		else
		{
			//Wrong Axis name -> Error
			CMD_Send(src, "#ERR:Wrong axis name\r\n");

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

	command = strtok_r(NULL,":",saveptr); //Gets the RPI command name
	if(strcmp(command, "ECHO")==0)
	{
		CMD_Send(UART_SRC_RPI, "$ECHO\r\n");
		CMD_Send(UART_SRC_PC, "Sent ECHO to RPi\r\n");
	}
	else if(strcmp(command, "SHUTDOWN")==0)
	{
		rpiShutdown();
	}
	else if(strcmp(command, "START")==0)
	{
		rpiPowerOn();
	}

}

void CMD_Send(UART_Source_t src, const char *msg)
{
    if (src == UART_SRC_PC)
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    else
        HAL_UART_Transmit(&huart5, (uint8_t*)msg, strlen(msg), 100);
}

void rpiShutdown()
{
	CMD_Send(UART_SRC_RPI, "$SHUTDOWN");
}

void rpiPowerOn()
{
	HAL_GPIO_WritePin(RPI_PWR_EN_GPIO_Port, RPI_PWR_EN_Pin, 1);
}

void rpiPowerOff()
{
	HAL_GPIO_WritePin(RPI_PWR_EN_GPIO_Port, RPI_PWR_EN_Pin, 0);
}

