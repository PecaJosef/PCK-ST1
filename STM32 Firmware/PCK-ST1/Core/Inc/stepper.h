/*
 * stepper.h
 *
 *  Created on: Aug 5, 2025
 *      Author: pecka
 */
#include "stm32l4xx_hal.h"
#include "main.h"
#include "stdbool.h"

#define STEPS_PER_REV 200
#define STEPPER_TIMER_FREQ 100000

//Define microstepping for each axis
#define EL_MICROSTEPPING 2
#define AZ_MICROSTEPPING 4
#define DEC_MICROSTEPPING 128
#define RA_MICROSTEPPING 128

//Define Gear ratios for each axis
#define EL_GEAR_RATIO 1.0f //Should be 90
#define AZ_GEAR_RATIO 48.5625f
#define DEC_GEAR_RATIO 100.0f
#define RA_GEAR_RATIO 1000.0f

#define EL_STEP_PER_DEG ((STEPS_PER_REV*EL_MICROSTEPPING*EL_GEAR_RATIO)/(360))
#define AZ_STEP_PER_DEG ((STEPS_PER_REV*AZ_MICROSTEPPING*AZ_GEAR_RATIO)/(360))
#define DEC_STEP_PER_DEG ((STEPS_PER_REV*DEC_MICROSTEPPING*DEC_GEAR_RATIO)/(360))
#define RA_STEP_PER_DEG ((STEPS_PER_REV*RA_MICROSTEPPING*RA_GEAR_RATIO)/(360))

typedef struct {
	GPIO_TypeDef *STEP_Port;
	uint16_t STEP_Pin;
	GPIO_TypeDef *EN_Port;
	uint16_t EN_Pin;
	GPIO_TypeDef *DIR_Port;
	uint16_t DIR_Pin;
	uint32_t Steps_remaining;
	uint32_t Step_interval_ticks;
	uint32_t Tick_counter;
	const float Steps_per_deg;
	bool enabled;
}Stepper_motor;

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_


extern TIM_HandleTypeDef htim3;
extern Stepper_motor EL_Axis_motor;

void Stepper_IT_Handeler();

void Stepper_IT_Enable();

void Stepper_Enable(Stepper_motor *Axis);

void Stepper_Move(Stepper_motor *Axis, float angle, float speed, bool direction);

#endif /* INC_STEPPER_H_ */
