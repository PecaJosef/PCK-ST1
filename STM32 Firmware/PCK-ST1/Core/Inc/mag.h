/*
 * mag.h
 *
 *  Created on: Aug 8, 2025
 *      Author: pecka
 */

#ifndef INC_MAG_H_
#define INC_MAG_H_

#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include "stepper.h"

#define QMC5883_ADDR         (0x0D << 1) // 7-bit address shifted for HAL
#define QMC5883_REG_X_LSB    0x00
#define QMC5883_REG_CTRL1    0x09
#define QMC5883_REG_SETRESET 0x0B

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MagRawData_t;

typedef struct {
    float x_offset;
    float y_offset;
    float x_scale;
    float y_scale;
} MagCalib_t;

// Initialize HMC5883
HAL_StatusTypeDef Mag_Init(I2C_HandleTypeDef *hi2c);

// Read raw magnetometer data
HAL_StatusTypeDef Mag_ReadRaw(MagRawData_t *data);

// Compute heading in degrees (0–360)
float ReadMagHeading(void);

MagCalib_t CalibrateMagnetometer(void *AZ_motor, float step_angle, float speed);

float GetCalibratedHeading(MagCalib_t *calib);

#endif /* INC_MAG_H_ */
