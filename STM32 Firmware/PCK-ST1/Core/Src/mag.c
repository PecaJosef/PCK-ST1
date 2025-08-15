/*
 * mag.c
 *
 *  Created on: Aug 8, 2025
 *      Author: pecka
 */

#include "mag.h"
#include <math.h>

static I2C_HandleTypeDef *mag_hi2c;

extern I2C_HandleTypeDef hi2c3;

HAL_StatusTypeDef Mag_Init(I2C_HandleTypeDef *hi2c) {
    mag_hi2c = hi2c;

    uint8_t setreset[2] = {QMC5883_REG_SETRESET, 0x01};
    uint8_t ctrl1[2]    = {QMC5883_REG_CTRL1, 0x1D};
    // 0x1D = OSR=512, RNG=8G, ODR=200Hz, Continuous

    if (HAL_I2C_Master_Transmit(mag_hi2c, QMC5883_ADDR, setreset, 2, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Master_Transmit(mag_hi2c, QMC5883_ADDR, ctrl1, 2, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef Mag_ReadRaw(MagRawData_t *data) {
    uint8_t reg = QMC5883_REG_X_LSB;
    uint8_t buffer[6];

    if (HAL_I2C_Master_Transmit(mag_hi2c, QMC5883_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Master_Receive(mag_hi2c, QMC5883_ADDR, buffer, 6, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;

    data->x = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->y = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return HAL_OK;
}

float ReadMagHeading(void) {
    MagRawData_t raw;
    if (Mag_ReadRaw(&raw) != HAL_OK) return -1.0f;

    float heading = atan2f((float)raw.y, (float)raw.x) * (180.0f / M_PI);
    if (heading < 0) heading += 360.0f;

    return heading;
}


MagCalib_t CalibrateMagnetometer(void *AZ_motor, float step_angle, float speed) {
    MagRawData_t data;
    int sample_count = (int)(360.0f / step_angle) + 1;

    int16_t *x_samples = malloc(sizeof(int16_t) * sample_count);
    int16_t *y_samples = malloc(sizeof(int16_t) * sample_count);

    int16_t x_min = INT16_MAX, x_max = INT16_MIN;
    int16_t y_min = INT16_MAX, y_max = INT16_MIN;

    // Rotate full 360° and collect samples
    for (int i = 0; i < sample_count; i++) {
        Stepper_Move(AZ_motor, step_angle, speed, true); // CW rotation

        HAL_Delay((uint32_t)(((step_angle*1000)/speed)+25)); // small delay for magnetometer stabilization

        if (Mag_ReadRaw(&data) != HAL_OK) continue;

        x_samples[i] = data.x;
        y_samples[i] = data.y;

        if (data.x < x_min) x_min = data.x;
        if (data.x > x_max) x_max = data.x;
        if (data.y < y_min) y_min = data.y;
        if (data.y > y_max) y_max = data.y;
    }

    MagCalib_t calib;

    // Hard-iron offsets
    calib.x_offset = (x_max + x_min) / 2.0f;
    calib.y_offset = (y_max + y_min) / 2.0f;

    // Optional soft-iron scaling
    float x_range = x_max - x_min;
    float y_range = y_max - y_min;
    float avg_range = (x_range + y_range) / 2.0f;

    calib.x_scale = avg_range / x_range;
    calib.y_scale = avg_range / y_range;

    free(x_samples);
    free(y_samples);

    return calib;
}

// Compute calibrated heading
float GetCalibratedHeading(MagCalib_t *calib) {
    MagRawData_t data;
    if (Mag_ReadRaw(&data) != HAL_OK) return -1.0f;

    float x = (data.x - calib->x_offset) * calib->x_scale;
    float y = (data.y - calib->y_offset) * calib->y_scale;

    float heading = -(atan2f(y, x) * 180.0f / M_PI);
    heading += 5.0f;
    if (heading < 0) heading += 360.0f;

    return heading;
}


