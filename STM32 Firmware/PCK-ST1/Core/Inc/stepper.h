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
#define STEPPER_TIMER_FREQ 7500
#define STEPPER_TIMER_HI_FREQ 1000000
#define STEPPER_TIMER_PRESCALE (CORE_FREQ/STEPPER_TIMER_HI_FREQ)
#define CORE_FREQ 64000000

#define DEG 60.0f

//Define microstepping for each axis
#define ALT_MICROSTEPPING 2
#define AZ_MICROSTEPPING 4
#define DEC_MICROSTEPPING 32
#define RA_MICROSTEPPING 32

//Define Gear ratios for each axis
#define ALT_GEAR_RATIO 90.0f
#define AZ_GEAR_RATIO 48.57954f //(5 + 2/11) * (150/16)
#define DEC_GEAR_RATIO 100.0f
#define RA_GEAR_RATIO 1000.0f

#define ARCMIN_FULL_ROT (360.0*60)

#define ALT_STEPS_PER_ARCMIN ((STEPS_PER_REV*ALT_MICROSTEPPING*ALT_GEAR_RATIO)/ARCMIN_FULL_ROT)
#define AZ_STEPS_PER_ARCMIN ((STEPS_PER_REV*AZ_MICROSTEPPING*AZ_GEAR_RATIO)/ARCMIN_FULL_ROT)
#define DEC_STEPS_PER_ARCMIN ((STEPS_PER_REV*DEC_MICROSTEPPING*DEC_GEAR_RATIO)/ARCMIN_FULL_ROT)
#define RA_STEPS_PER_ARCMIN ((STEPS_PER_REV*RA_MICROSTEPPING*RA_GEAR_RATIO)/ARCMIN_FULL_ROT)

//Positive direction is opposite to Homing direction -> while homing the stepper moves in negative direction towards endstop
#define AZ_POS_DIR 1
#define ALT_POS_DIR 0
#define DEC_POS_DIR 1
#define RA_POS_DIR 0

#define RA_PWM_TIM &htim1 //Main timer - generates step signal
#define RA_PWM_CH TIM_CHANNEL_2
#define RA_TIM TIM2
#define RA_STEP_COUNTER_TIM &htim2 //Slave timer - interrupts when targeted steps reached

#define DEC_PWM_TIM &htim8 //Main timer - generates step signal
#define DEC_PWM_CH TIM_CHANNEL_3
#define DEC_TIM TIM5
#define DEC_STEP_COUNTER_TIM &htim5 //Slave timer - interrupts when targeted steps reached

#define STEPPER_TIMER TIM3 //Timer for ALT/AZ axes - generates periodic interrupts if enabled -> to drive the axes
#define STEPPER_TIMER_PTR &htim3

#define IREG_TIMER TIM17
#define IREG_TIMER_PTR &htim17
#define IREG_TIM_CHANNEL TIM_CHANNEL_1
#define IREG_ARR 1000


typedef enum {
    PWM_OUT_P,
    PWM_OUT_N
} PWM_OutputType_t;

/*
typedef enum {
    AXIS_HOMING_IDLE,
    AXIS_HOMING_COARSE,
    AXIS_HOMING_BACKOFF,
    AXIS_HOMING_FINE,
    AXIS_HOMING_DONE
} AxisHomingState_t;
*/

typedef enum {
    AXIS_HOMING_IDLE,
    AXIS_HOMING_SEARCH_COARSE, //Fast search
    AXIS_HOMING_BACKOFF,       //Move back from the end-stop
    AXIS_HOMING_SEARCH_FINE,   //Slow approach for Edge A
    AXIS_HOMING_OVERSHOOT,     //Move past Edge A
    AXIS_HOMING_REVERSE_SEARCH, //Slow approach for Edge B from other side
    AXIS_HOMING_GO_TO_CENTER,  //Move to (edgeA+edgeB)/2
    AXIS_HOMING_DONE,
} AxisHomingState_t;

typedef struct {
	bool direction;
	int32_t lastStepsRead;
	int32_t stepPosition;
	float angularPosition;
} axisPosition_t;

typedef enum {
	AZ,
	ALT,
	RA,
	DEC,
} AxisID_t;

typedef struct {
	AxisID_t AxisID;
	GPIO_TypeDef *STEP_Port;
	uint16_t STEP_Pin;
	GPIO_TypeDef *EN_Port;
	uint16_t EN_Pin;
	GPIO_TypeDef *DIR_Port;
	uint16_t DIR_Pin;
	GPIO_TypeDef *ENDSTOP_Port;
	uint16_t ENDSTOP_Pin;
	IRQn_Type EXTI_IRQn;
	const float stepsPerArcmin;
	bool enabled;
	bool busy;
	bool highPrecisionAxis;
	bool homing;
	bool Positive_dir;
	AxisHomingState_t homing_state;
	axisPosition_t Position;

	//Low-precision speed axis
	uint32_t StepsCounter; //Number of steps already made in single move
	uint32_t StepsTarget; //Number of steps to make
	uint32_t TicksPerStep; //Number of interrupt ticks between each step
	uint32_t TickCounter;	//Interrupt ticks counter

	//High-precision speed axis
	TIM_HandleTypeDef *PWM_Timer; //PWM (STEP) signal timer
	uint32_t PWM_Channel; //PWM (STEP) signal channel
	TIM_HandleTypeDef *Step_Counter_Timer; //Timer for counting steps
	PWM_OutputType_t PWM_Type; //PWM Channel polarity
	float edgeA; //Position variable for precise homing in the middle of end-stop active zone
	float edgeB; // -//-
}StepperMotor_t;


extern StepperMotor_t ALT_AxisMotor;
extern StepperMotor_t AZ_AxisMotor;
extern StepperMotor_t RA_AxisMotor;
extern StepperMotor_t DEC_AxisMotor;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim17;

void stepperInit();

void stepperInterruptHandler();

void Stepper_IT_Enable();

void Stepper_IT_Disable();

void stepperEnable(StepperMotor_t *Axis);

void stepperDisable(StepperMotor_t *Axis);

void stepperMove(StepperMotor_t *Axis, float angle, float speed);

void stepperHome(StepperMotor_t *Axis, float speed, bool direction);

void Stepper_nSleep(bool n_sleep);

void stepsGenerating(StepperMotor_t *Axis);

void stepperStop(StepperMotor_t *Axis);

void setCurrent(float percentage);

#endif /* INC_STEPPER_H_ */
