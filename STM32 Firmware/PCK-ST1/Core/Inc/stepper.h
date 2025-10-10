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

#define AZ_HOMING_DIR 1
#define EL_HOMING_DIR 1
#define DEC_HOMING_DIR 1
#define RA_HOMING_DIR 1

#define RA_TIM TIM2
#define RA_PWM_TIM &htim1
#define RA_PWM_CH TIM_CHANNEL_2
#define RA_STEP_TIM &htim2

#define DEC_TIM TIM5
#define DEC_PWM_TIM &htim8
#define DEC_PWM_CH TIM_CHANNEL_3
#define DEC_STEP_TIM &htim5

#define STEPPER_TIMER TIM3

typedef enum {
    PWM_OUT_P,
    PWM_OUT_N
} PWM_OutputType;

typedef enum {
    AXIS_HOMING_IDLE,
    AXIS_HOMING_COARSE,
    AXIS_HOMING_BACKOFF,
    AXIS_HOMING_FINE,
    AXIS_HOMING_DONE
} AxisHomingState;

typedef struct {
	GPIO_TypeDef *STEP_Port;
	uint16_t STEP_Pin;
	GPIO_TypeDef *EN_Port;
	uint16_t EN_Pin;
	GPIO_TypeDef *DIR_Port;
	uint16_t DIR_Pin;
	GPIO_TypeDef *ENDSTOP_Port;
	uint16_t ENDSTOP_Pin;
	IRQn_Type EXTI_IRQn;
	const float Steps_per_deg;
	bool enabled;
	bool busy;
	bool High_precision;
	bool homing;
	bool Homing_dir;
	AxisHomingState homing_state;
	float Position;

	//Low precision stepper motors
	uint32_t Steps_remaining; //Number of steps
	uint32_t Step_interval_ticks;
	uint32_t Tick_counter;

	//High precision stepper motors
	TIM_HandleTypeDef *PWM_Timer; //PWM (STEP) signal timer
	uint32_t PWM_Channel; //PWM (STEP) signal channel
	TIM_HandleTypeDef *Step_Counter_Timer; //Timer for counting steps
	PWM_OutputType PWM_Type; //PWM Channel polarity

}Stepper_motor;


extern Stepper_motor EL_Axis_motor;
extern Stepper_motor AZ_Axis_motor;
extern Stepper_motor RA_Axis_motor;
extern Stepper_motor DEC_Axis_motor;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

void Stepper_IT_Handeler();

void Stepper_IT_Enable();

void Stepper_IT_Disable();

void Stepper_Enable(Stepper_motor *Axis);

void Stepper_Disable(Stepper_motor *Axis);

void Stepper_Move(Stepper_motor *Axis, float angle, float speed, bool direction);

void Stepper_Home(Stepper_motor *Axis, float speed, bool direction);

void Stepper_nSleep(bool n_sleep);

void STEP_Generating(Stepper_motor *Axis);

void Stepper_Stop(Stepper_motor *Axis);

#endif /* INC_STEPPER_H_ */
