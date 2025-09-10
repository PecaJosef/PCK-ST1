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

void Axis_Home_Start(Stepper_motor *Axis, float coarseSpeed, float fineSpeed);

void Axis_Homing_Update(Stepper_motor *Axis, float coarseSpeed, float fineSpeed);

void Endstop_Reached(Stepper_motor *Axis);

#endif /* INC_HOMING_H_ */
