/*
 * mag.c
 *
 *  Created on: Aug 8, 2025
 *      Author: pecka
 */

#include "mag.h"
#include <math.h>


static I2C_HandleTypeDef *mag_hi2c;
//extern I2C_HandleTypeDef hi2c3;


HAL_StatusTypeDef Mag_Init(I2C_HandleTypeDef *hi2c)
{
    mag_hi2c = hi2c;

    uint8_t setreset[2] = {QMC5883_REG_SETRESET, 0x01};
    uint8_t ctrl1[2]    = {QMC5883_REG_CTRL1, 0x0D};
    // 0x1D = OSR=512, RNG=2G, ODR=200Hz, Continuous

    if (HAL_I2C_Master_Transmit(mag_hi2c, QMC5883_ADDR, setreset, 2, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Master_Transmit(mag_hi2c, QMC5883_ADDR, ctrl1, 2, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

bool magIsAlive()
{
	uint8_t reg = QMC5883_REG_ID;
	uint8_t id_buffer[1];

	if (HAL_I2C_Master_Transmit(mag_hi2c, QMC5883_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	if (HAL_I2C_Master_Receive(mag_hi2c, QMC5883_ADDR, id_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}

	if (id_buffer[0] == 0xFF)
	{
		return true;
	}
	else
	{
		return false;
	}
}

HAL_StatusTypeDef magReadRaw(MagRawData_t *data)
{
    uint8_t reg = QMC5883_REG_X_LSB;
    uint8_t buffer[6];

    if (HAL_I2C_Master_Transmit(mag_hi2c, QMC5883_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Master_Receive(mag_hi2c, QMC5883_ADDR, buffer, 6, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;

    data->x = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->y = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->z = (int16_t)((buffer[5] << 8) | buffer[4]);

    printf("%d;%d\r\n", data->x, data->y);

    return HAL_OK;
}

MagCalib_t calibrationMatrix(int32_t sumX, int32_t sumY, int64_t sumXX, int64_t sumYY, int64_t sumXY, int16_t totalCalSamples)
{
	MagCalib_t calib;

	float n = (float)totalCalSamples;
	double meanX = (double)sumX / n;
	double meanY = (double)sumY / n;

	printf("Xmean: %f, Ymean %f\r\n",meanX, meanY);

	//Covariance values
	double varX  = (double)sumXX / n - (meanX * meanX);
	double varY  = (double)sumYY / n - (meanY * meanY);
	double covXY = (double)sumXY / n - (meanX * meanY);

	//Eigenvalues
	double trace = varX + varY;
	double det   = varX * varY - covXY * covXY;
	double disc  = sqrtf(trace * trace - 4.0f * det);
	double l1    = (trace + disc) / 2.0f;
	double l2    = (trace - disc) / 2.0f;

	//Eigenvectors (Normalized)
	double v1x = covXY;
	double v1y = l1 - varX;
	double mag1 = sqrtf(v1x*v1x + v1y*v1y);
	v1x /= mag1; v1y /= mag1;

	double v2x = covXY;
	double v2y = l2 - varX;
	double mag2 = sqrtf(v2x*v2x + v2y*v2y);
	v2x /= mag2; v2y /= mag2;

	//Whitening Matrix M = s1*v1*v1^T + s2*v2*v2^T
	float s1 = 1.0f / sqrtf(fmaxf(l1, 1e-6f));
	float s2 = 1.0f / sqrtf(fmaxf(l2, 1e-6f));

	calib.x_offset = meanX;
	calib.y_offset = meanY;
	calib.softiron[0][0] = s1*v1x*v1x + s2*v2x*v2x;
	calib.softiron[0][1] = s1*v1x*v1y + s2*v2x*v2y;
	calib.softiron[1][0] = s1*v1y*v1x + s2*v2y*v2x;
	calib.softiron[1][1] = s1*v1y*v1y + s2*v2y*v2y;


	printf("M00: %.9f, M01: %.9f, M10: %.9f, M11: %.9f\r\n",calib.softiron[0][0],calib.softiron[0][1],calib.softiron[1][0],calib.softiron[1][1]);

	return calib;
}

float getCalibratedHeading(MagCalib_t *calib, float declination)
{
    MagRawData_t data;
    if (magReadRaw(&data) != HAL_OK) return -1.0f;

    // Subtract offsets (hard-iron correction)
    float dx = data.x - calib->x_offset;
    float dy = data.y - calib->y_offset;

    // Apply soft-iron correction matrix
    float x = calib->softiron[0][0] * dx + calib->softiron[0][1] * dy;
    float y = calib->softiron[1][0] * dx + calib->softiron[1][1] * dy;

    // Compute heading
    //Positive values - heading EAST from NORTH, negative values - heading WEST from NORTH
    float heading = -(atan2f(y, x) * 180.0f / M_PI);

    // Apply magnetic declination
    heading += declination;

    return heading;
}

void SaveCalibrationToFlash(MagCalib_t *calib)
{
    HAL_FLASH_Unlock();

    // Erase the page
    FLASH_EraseInitTypeDef eraseInit = {0};
    uint32_t pageError;
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.Page = CALIB_FLASH_PAGE;
    eraseInit.NbPages = 1;
    eraseInit.Banks = FLASH_BANK_2;

    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return; // erase failed
    }
    // Prepare struct
    StoredCalib_t data = {0xDEADBEEF, *calib, 0};

    // Program as 64-bit doublewords
    uint64_t *p = (uint64_t*)&data;
    for (size_t i = 0; i < sizeof(data) / 8; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CALIB_FLASH_ADDR + i*8, p[i]) != HAL_OK)
        {
            break; // stop on error
        }
    }
    HAL_FLASH_Lock();
}

bool LoadCalibrationFromFlash(MagCalib_t *calib)
{
    //Cast the Flash address directly
    StoredCalib_t *flashData = (StoredCalib_t*)CALIB_FLASH_ADDR;

    //Check the magic number
    if(flashData->magic != 0xDEADBEEF)
    {
        return false;
    }

    *calib = flashData->calib;

    return true;
}

