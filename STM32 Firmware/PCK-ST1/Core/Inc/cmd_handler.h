/*
 * cmd_handler.h
 *
 *  Created on: Oct 7, 2025
 *      Author: pecka
 */




#ifndef INC_CMD_HANDLER_H_
#define INC_CMD_HANDLER_H_

#include "main.h"
#include "stm32l4xx_hal.h"

#define CMD_BUFFER_SIZE 128

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;


typedef enum {
    UART_SRC_PC,
    UART_SRC_RPI
} UART_Source_t;

void commandHandler_Init(void);

void commandHandler(void);

void CMD_UART_StoreByte(UART_Source_t src, uint8_t byte);

void CMD_Send(UART_Source_t src, const char *msg);

void rpiPowerOn();

void rpiPowerOff();

void rpiShutdown();

#endif /* INC_CMD_HANDLER_H_ */
