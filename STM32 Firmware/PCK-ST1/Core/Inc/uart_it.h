/*
 * uart_it.h
 *
 *  Created on: Oct 7, 2025
 *      Author: pecka
 */

#ifndef INC_UART_IT_H_
#define INC_UART_IT_H_

#include "main.h"
#include "stm32l4xx_hal.h"

#define UART_BUFFER_SIZE 128

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;

void UART_IT_Init(void);

#endif /* INC_UART_IT_H_ */
