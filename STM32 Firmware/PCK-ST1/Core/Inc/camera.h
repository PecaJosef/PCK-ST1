/*
 * camera.h
 *
 *  Created on: Apr 8, 2026
 *      Author: pecka
 */

#ifndef INC_CAMERA_H_
#define INC_CAMERA_H_


#include "main.h"
#include "control_loop.h"
#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include "cmd_handler.h"

#define CAMERA_TICK_DURATION 0.005f //seconds
#define CAMERA_INTER_IMG_DELAY 1000 //ms

#define CAMERA_TIM TIM16
#define CAMERA_TIM_CH TIM_CHANNEL_1
#define CAM_TIM_PTR &htim16

#define CAM_TIM_FREQ 2000 //2kHz
#define CAMERA_PRESCALE (CORE_FREQ/CAM_TIM_FREQ)
#define CAM_ARR ((CAMERA_TICK_DURATION)*CAM_TIM_FREQ)


typedef enum {
    CAM_IDLE,
    CAM_SHUTTER_OPEN,
    CAM_WAIT_DELAY,
    CAM_BUSY,
} CameraState_t;

typedef struct {
	volatile uint32_t exposureLengthTicks;
	volatile uint32_t exposureCurrentTicks;
	uint16_t imagesRemaining;
	CameraState_t currentState;
	uint32_t delayLastTicks;
}Camera_t;

extern TIM_HandleTypeDef htim16;
extern Camera_t camera;

void cameraInit();

void cameraHandler();

void cameraInterruptHandler();

void captureImage(float exposure, uint16_t imageCount);

void captureStop();


#endif /* INC_CAMERA_H_ */
