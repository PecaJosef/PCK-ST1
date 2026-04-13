/*
 * camera.c
 *
 *  Created on: Apr 8, 2026
 *      Author: pecka
 */
#include "camera.h"

Camera_t camera = {
		.exposureCurrentTicks = 0,
		.exposureLengthTicks = 0,
		.imagesRemaining = 0,
		.currentState = CAM_IDLE,
		.delayLastTicks = 0,
};

void cameraInit()
{
	//Camera timer initiation
	__HAL_TIM_SET_PRESCALER(CAM_TIM_PTR, CAMERA_PRESCALE-1); //10kHz timer freq
	__HAL_TIM_SET_AUTORELOAD(CAM_TIM_PTR, (uint32_t)(CAM_ARR-1));
}

void cameraHandler()
{
    switch (camera.currentState)
    {
    	case CAM_IDLE:
    		//Do nothing
    		break;

    	case CAM_SHUTTER_OPEN:
    		//Do nothing
    		break;

        case CAM_BUSY:
            if (camera.imagesRemaining > 0)
            {
            	camera.exposureCurrentTicks = 0;

            	//Reset the camera timer
				__HAL_TIM_SET_COUNTER(CAM_TIM_PTR, 0);
				__HAL_TIM_CLEAR_IT(CAM_TIM_PTR, TIM_IT_UPDATE);

            	//Start capturing the image
                HAL_GPIO_WritePin(CAM_SHUTTER_GPIO_Port, CAM_SHUTTER_Pin, GPIO_PIN_SET);

                //Start the camera timer IT
                HAL_TIM_Base_Start_IT(CAM_TIM_PTR);
                camera.currentState = CAM_SHUTTER_OPEN;
            }
            else
            {
                camera.currentState = CAM_IDLE;
            }
            break;

        case CAM_WAIT_DELAY:
            if (HAL_GetTick() - camera.delayLastTicks >= CAMERA_INTER_IMG_DELAY)
            {
                camera.currentState = CAM_BUSY;
            }
            break;

        default: break;
    }
}

void captureImage(float exposure, uint16_t imageCount)
{
    if (camera.currentState != CAM_IDLE || imageCount == 0)
	{
		return;
	}

    //Calculate how many camera ticks are needed
    camera.exposureLengthTicks = (uint32_t)(exposure / CAMERA_TICK_DURATION);
    if (camera.exposureLengthTicks == 0)
    {
    	camera.exposureLengthTicks = 1;
    }

    camera.imagesRemaining = imageCount;
    camera.currentState = CAM_BUSY;
}

void cameraInterruptHandler()
{
	camera.exposureCurrentTicks++;

	if (camera.exposureCurrentTicks >= camera.exposureLengthTicks)
	{
		//End of exposure - reset the shutter pin
		HAL_GPIO_WritePin(CAM_SHUTTER_GPIO_Port, CAM_SHUTTER_Pin, GPIO_PIN_RESET);

		//Stop the camera timer and reset the IT flag
		HAL_TIM_Base_Stop_IT(CAM_TIM_PTR);
		__HAL_TIM_CLEAR_IT(CAM_TIM_PTR, TIM_IT_UPDATE);

		camera.imagesRemaining--;
		camera.delayLastTicks = HAL_GetTick();
		camera.currentState = CAM_WAIT_DELAY;
	}
}
