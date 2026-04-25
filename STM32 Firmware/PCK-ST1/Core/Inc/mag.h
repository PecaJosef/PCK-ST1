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
#define QMC5883_REG_ID      0x0D


#define CALIB_FLASH_ADDR 0x0808F800
#define CALIB_FLASH_PAGE 31


typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MagRawData_t;


typedef struct {
    float x_offset;
    float y_offset;
    float softiron[2][2]; // 2x2 correction matrix
} MagCalib_t;

typedef struct {
    uint32_t magic;
    uint32_t padding; // pad to 8-byte multiple
    MagCalib_t calib;
} __attribute__((aligned(8))) StoredCalib_t;



// Initialize HMC5883
HAL_StatusTypeDef Mag_Init(I2C_HandleTypeDef *hi2c);

// Read raw magnetometer data
HAL_StatusTypeDef magReadRaw(MagRawData_t *data);

MagCalib_t CalibrateMagnetometer(StepperMotor_t *AZ_motor, float step_angle, float speed);

MagCalib_t calibrationMatrix(int32_t sumX, int32_t sumY, int64_t sumXX, int64_t sumYY, int64_t sumXY, int16_t totalCalSamples);//, float offsetX, float offsetY);

float getCalibratedHeading(MagCalib_t *calib, float declination);

void SaveCalibrationToFlash(MagCalib_t *calib);

bool LoadCalibrationFromFlash(MagCalib_t *calib);

bool magIsAlive();

#endif /* INC_MAG_H_ */
