/*
 * homing.h
 *
 *  Created on: Sep 5, 2025
 *      Author: pecka
 */

#ifndef INC_HOMING_H_
#define INC_HOMING_H_

#include "control_loop.h"

void handleHoming();

void Axis_Home_Start(StepperMotor_t *Axis, float coarseSpeed, float fineSpeed);

void Axis_Homing_Update(StepperMotor_t *Axis, float coarseSpeed, float fineSpeed);

void Endstop_Reached(StepperMotor_t *Axis);

#endif /* INC_HOMING_H_ */
