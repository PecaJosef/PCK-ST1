/*
 * stepper.h
 *
 *  Created on: Aug 5, 2025
 *      Author: pecka
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_


#include "stm32l4xx_hal.h"
#include "main.h"
#include "stdbool.h"

#define STEPS_PER_REV 200.0f
#define STEPPER_TIMER_FREQ 10000
#define STEPPER_TIMER_HI_FREQ 2000000

//Define microstepping for each axis
#define EL_MICROSTEPPING 2
#define AZ_MICROSTEPPING 4
#define DEC_MICROSTEPPING 128
#define RA_MICROSTEPPING 128

//Define Gear ratios for each axis
#define EL_GEAR_RATIO 90.0f //Should be 90
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

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;


extern Stepper_motor EL_Axis_motor;
extern Stepper_motor AZ_Axis_motor;
extern Stepper_motor RA_Axis_motor;
extern Stepper_motor DEC_Axis_motor;

extern

void Stepper_IT_Handeler();

void Stepper_IT_Enable();

void Stepper_Enable(Stepper_motor *Axis);

void Stepper_Move(Stepper_motor *Axis, float angle, float speed, bool direction);

void Stepper_Move_DEC(float angle, float speed, bool dir);

void Stepper_Move_RA(float angle, float speed, bool dir);

void Stepper_nSleep(bool n_sleep);

void STEP_Generating(Stepper_motor *Axis);

#endif /* INC_STEPPER_H_ */
