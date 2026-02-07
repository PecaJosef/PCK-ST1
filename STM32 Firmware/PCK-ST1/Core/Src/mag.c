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

HAL_StatusTypeDef Mag_ReadRaw(MagRawData_t *data)
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

float ReadMagHeading(void)
{
    MagRawData_t raw;
    if (Mag_ReadRaw(&raw) != HAL_OK) return -1.0f;

    float heading = atan2f((float)raw.y, (float)raw.x) * (180.0f / M_PI);
    if (heading < 0) heading += 360.0f;

    return heading;
}

static void eigen2x2(float a, float b, float c, float *l1, float *l2,float *vx1, float *vy1, float *vx2, float *vy2)
{
    // Symmetric matrix [[a, b],[b, c]]
    float tr = a + c;
    float det = a*c - b*b;
    float disc = tr*tr*0.25f - det;
    if (disc < 0) disc = 0;
    float s = sqrtf(disc);

    *l1 = tr*0.5f + s;
    *l2 = tr*0.5f - s;

    // Eigenvectors; handle near-zero b specially
    if (fabsf(b) > 1e-9f) {
        *vx1 = *l1 - c; *vy1 = b;
        *vx2 = *l2 - c; *vy2 = b;
    } else {
        // matrix is (almost) diagonal
        *vx1 = 1.0f; *vy1 = 0.0f;
        *vx2 = 0.0f; *vy2 = 1.0f;
    }
    // Normalize
    float n1 = sqrtf((*vx1)*(*vx1) + (*vy1)*(*vy1)); if (n1 > 0) { *vx1 /= n1; *vy1 /= n1; }
    float n2 = sqrtf((*vx2)*(*vx2) + (*vy2)*(*vy2)); if (n2 > 0) { *vx2 /= n2; *vy2 /= n2; }
}

MagCalib_t CalibrateMagnetometer(StepperMotor_t *AZ_motor, float step_deg, float speed)
{
    const uint16_t N = (uint16_t)(360.0f / step_deg);
    // Accumulators
    float sx=0, sy=0;
    float sxx=0, sxy=0, syy=0;

    MagRawData_t d;

    // 1) First pass: compute mean (hard-iron)
    for (int i=0;i<N;i++) {
        stepperMove(AZ_motor, step_deg, speed);
        while (AZ_motor->busy);

        if (Mag_ReadRaw(&d) != HAL_OK)
        {
        i--;
        continue;
        }

        sx += (float)d.x;
        sy += (float)d.y;
    }
    float mx = sx / N;
    float my = sy / N;

    // 2) Second pass: covariance of centered data
    // (Run the sweep again to avoid storing samples; if you prefer one sweep, store into a small buffer.)
    // If re-sweeping isn’t practical, store (x,y) into arrays on first pass, then compute cov below.
    // Here we re-sweep for simplicity:
    // Return to start angle if needed (optional)
    for (int i=0;i<N;i++) {
        stepperMove(AZ_motor, -step_deg, speed);
        while (AZ_motor->busy);
        //HAL_Delay(200);

        if (Mag_ReadRaw(&d) != HAL_OK) { i--; continue; }

        float cx = (float)d.x - mx;
        float cy = (float)d.y - my;

        sxx += cx*cx;
        sxy += cx*cy;
        syy += cy*cy;
    }
    sxx /= N; sxy /= N; syy /= N;

    // 3) Eigendecompose covariance -> eigenvalues λ1, λ2 and unit eigenvectors v1, v2
    float l1, l2, vx1, vy1, vx2, vy2;
    eigen2x2(sxx, sxy, syy, &l1, &l2, &vx1, &vy1, &vx2, &vy2);

    // Guard against degenerate/negative values (add tiny floor)
    const float eps = 1e-6f;
    if (l1 < eps) l1 = eps;
    if (l2 < eps) l2 = eps;

    // 4) Whitening matrix M = V * diag(1/sqrt(l)) * V^T
    float s1 = 1.0f / sqrtf(l1);
    float s2 = 1.0f / sqrtf(l2);

    // V = [v1 v2], with v1=(vx1,vy1), v2=(vx2,vy2)
    // M = s1*v1*v1^T + s2*v2*v2^T
    float M00 = s1*vx1*vx1 + s2*vx2*vx2;
    float M01 = s1*vx1*vy1 + s2*vx2*vy2;
    float M10 = s1*vy1*vx1 + s2*vy2*vx2;
    float M11 = s1*vy1*vy1 + s2*vy2*vy2;

    MagCalib_t calib;
    calib.x_offset = mx;
    calib.y_offset = my;
    calib.softiron[0][0] = M00;
    calib.softiron[0][1] = M01;
    calib.softiron[1][0] = M10;
    calib.softiron[1][1] = M11;

    return calib;
}

static inline float wrap360(float a)
{
    while (a < 0.0f)   a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

float getCalibratedHeading(MagCalib_t *calib, float declination)
{
    MagRawData_t data;
    if (Mag_ReadRaw(&data) != HAL_OK) return -1.0f;

    // Subtract offsets (hard-iron correction)
    float dx = data.x - calib->x_offset;
    float dy = data.y - calib->y_offset;

    // Apply soft-iron correction matrix
    float x = calib->softiron[0][0] * dx + calib->softiron[0][1] * dy;
    float y = calib->softiron[1][0] * dx + calib->softiron[1][1] * dy;

    // Compute heading
    float heading = -(atan2f(y, x) * 180.0f / M_PI);
    if (heading < 0) heading += 360.0f;

    // Apply magnetic declination
    heading += declination;
    if (heading >= 360.0f) heading -= 360.0f;

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
/*
bool LoadCalibrationFromFlash(MagCalib_t *calib)
{
    uint32_t *addr = (uint32_t*)CALIB_FLASH_ADDR;
    if(addr[0] != 0xDEADBEEF) return false;

    calib->x_offset = *((float*)&addr[1]);
    calib->y_offset = *((float*)&addr[2]);

    for(int i=0;i<2;i++)
        for(int j=0;j<2;j++)
            calib->softiron[i][j] = *((float*)&addr[3 + i*2 + j]);

    return true;
}
*/
bool LoadCalibrationFromFlash(MagCalib_t *calib)
{
    // Cast the Flash address directly to your wrapper struct type
    StoredCalib_t *flashData = (StoredCalib_t*)CALIB_FLASH_ADDR;

    // 1. Check the magic number
    if(flashData->magic != 0xDEADBEEF) {
        return false;
    }

    // 2. Use memcpy or direct assignment to copy the nested struct
    // This handles all internal padding and alignment automatically
    *calib = flashData->calib;

    return true;
}
