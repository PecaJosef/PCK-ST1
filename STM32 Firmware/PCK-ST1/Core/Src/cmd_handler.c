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

    buf[(*idx)++] = byte;

    if (byte == '\n' || byte == '\r' || *idx >= CMD_BUFFER_SIZE)
    {
        buf[*idx] = '\0';
        *ready = 1;
        *idx = 0;
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
	else if (strcmp(token, "RPI")==0)
	{
		//Jump to RPI command sub processing
	}
	else
	{
		CMD_Send(src, "ERR:Unknown command\r\n");
	}


/*
//Out of date code - may be useful later
    if (strncmp((char*)cmd, "ECHO", 4) == 0)
    {
        printf("here\r\n");
    	CMD_Send(src, "ECHO\r\n");
    }
    else if (strncmp((char*)cmd, "$RPI:ON", 7) == 0)
    {
    	rpiStart();
    	CMD_Send(src, "RPI Started\r\n");
    }
    else if (strncmp((char*)cmd, "$RPI:OFF", 8) == 0)
    {
    	//CMD_Send(UART_SRC_RPI, "$RPI:OFF\r\n");
    	rpiShutdown();
    	CMD_Send(src, "RPI Shutdown\r\n");
    }
    else
    {
        CMD_Send(src, "ERR:UNKNOWN_CMD\r\n");
    }
*/
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
	HAL_GPIO_WritePin(RPI_PWR_EN_GPIO_Port, RPI_PWR_EN_Pin, 0);
}

void rpiStart()
{
	HAL_GPIO_WritePin(RPI_PWR_EN_GPIO_Port, RPI_PWR_EN_Pin, 1);
}

