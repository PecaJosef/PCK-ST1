/*
 * uart_it.c
 *
 *  Created on: Oct 7, 2025
 *      Author: pecka
 */

/*
 * UART1 - Controller/PC
 * UART5 - RPI
 */

#include "uart_it.h"
#include "cmd_handler.h"
#include "stm32l4xx_hal.h"
//#include "usbd_cdc_if.h"

uint8_t uart1_rx_byte;
uint8_t uart5_rx_byte;

volatile uint8_t uart1_rx_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart5_rx_buffer[UART_BUFFER_SIZE];

volatile uint16_t uart1_rx_index = 0;
volatile uint16_t uart5_rx_index = 0;

void UART_IT_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    HAL_UART_Receive_IT(&huart5, &uart5_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) //Controller/PC uart
    {
    	CMD_UART_StoreByte(PC_UART_SRC, uart1_rx_byte);
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    }
    else if (huart->Instance == UART5) //RPi uart
    {
        CMD_UART_StoreByte(RPI_UART_SRC, uart5_rx_byte);
        HAL_UART_Receive_IT(&huart5, &uart5_rx_byte, 1);
    }
}
