/*
 * mag.c
 *
 *  Created on: Aug 8, 2025
 *      Author: pecka
 */

#include "mag.h"
#include <math.h>

#define HMC5883_ADDR         (0x1E << 1) // 7-bit address shifted for HAL
#define HMC5883_REG_CONFIG_A 0x00
#define HMC5883_REG_CONFIG_B 0x01
#define HMC5883_REG_MODE     0x02
#define HMC5883_REG_DATA_X_MSB 0x03

// Store I2C handle
static I2C_HandleTypeDef *mag_hi2c;

HAL_StatusTypeDef Mag_Init(I2C_HandleTypeDef *hi2c) {
    mag_hi2c = hi2c;

    uint8_t configA[2] = {HMC5883_REG_CONFIG_A, 0x70}; // 8-average, 15 Hz, normal
    uint8_t configB[2] = {HMC5883_REG_CONFIG_B, 0x20}; // Gain = 1.3 Ga
    uint8_t mode[2]    = {HMC5883_REG_MODE, 0x00};     // Continuous mode

    if (HAL_I2C_Master_Transmit(mag_hi2c, HMC5883_ADDR, configA, 2, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Master_Transmit(mag_hi2c, HMC5883_ADDR, configB, 2, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Master_Transmit(mag_hi2c, HMC5883_ADDR, mode, 2, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef Mag_ReadRaw(MagRawData_t *data) {
    uint8_t reg = HMC5883_REG_DATA_X_MSB;
    uint8_t buffer[6];

    if (HAL_I2C_Master_Transmit(mag_hi2c, HMC5883_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Master_Receive(mag_hi2c, HMC5883_ADDR, buffer, 6, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;

    data->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->z = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->y = (int16_t)((buffer[4] << 8) | buffer[5]);

    return HAL_OK;
}

float ReadMagHeading(void) {
    MagRawData_t raw;
    if (Mag_ReadRaw(&raw) != HAL_OK) return -1.0f; // Error

    // Convert to heading
    float heading = atan2f((float)raw.y, (float)raw.x) * (180.0f / M_PI);

    if (heading < 0) heading += 360.0f;

    return heading;
}

