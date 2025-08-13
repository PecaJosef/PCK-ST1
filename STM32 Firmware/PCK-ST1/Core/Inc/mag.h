/*
 * mag.h
 *
 *  Created on: Aug 8, 2025
 *      Author: pecka
 */

#ifndef INC_MAG_H_
#define INC_MAG_H_

#include "stm32l4xx_hal.h"  // Change if using a different STM32 family

extern I2C_HandleTypeDef hi2c3;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MagRawData_t;

// Initialize HMC5883
HAL_StatusTypeDef Mag_Init(I2C_HandleTypeDef *hi2c);

// Read raw magnetometer data
HAL_StatusTypeDef Mag_ReadRaw(MagRawData_t *data);

// Compute heading in degrees (0–360)
float ReadMagHeading(void);

#endif /* INC_MAG_H_ */
