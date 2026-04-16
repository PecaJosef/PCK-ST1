/*
 * astro.c
 *
 *  Created on: Apr 6, 2026
 *      Author: pecka
 */

#include "astro.h"

gotoRequest_t gotoRequest = {
	.raAngle = 0,
	.decAngle = 0,
	.gotoRequested = false,
};

CoordinatesRaDec_t coordinatesRaDec = {
		.raCoordinatesHours = 6.0f,
		.raCoordinatesAngle = 90.0f,
		.decCoordinatesAngle = 90.0f,
		.lastRaUpdate = 0,
		.trackingPosition = false,
};

void trackingStart(StepperMotor_t *Axis)
{
	if(!Axis->enabled)
	{
		stepperEnable(Axis);
	}

	uint32_t ccr = (TRACKING_TIMER_FREQ/TRACKING_FREQ) / 2;
	HAL_GPIO_WritePin(Axis->DIR_Port, Axis->DIR_Pin, TRACKING_DIR);
	Axis->Position.direction = TRACKING_DIR;

	__HAL_TIM_SET_PRESCALER(Axis->PWM_Timer, TRACKING_PRESCALER-1); //RA axis base timer frequency 3.2MHz
	__HAL_TIM_SET_AUTORELOAD(Axis->PWM_Timer, (TRACKING_TIMER_FREQ/TRACKING_FREQ)-1); //ARR based on tracking frequency
	__HAL_TIM_SET_COMPARE(Axis->PWM_Timer, Axis->PWM_Channel, ccr); //PWM duty cycle ~50%
	__HAL_TIM_SET_AUTORELOAD(Axis->Step_Counter_Timer, 0xFFFFFFFF - 1); //Max steps ceiling - runs indefinitely
	__HAL_TIM_SET_COUNTER(Axis->Step_Counter_Timer, 0); //Reset counter
	__HAL_TIM_CLEAR_IT(Axis->Step_Counter_Timer, TIM_IT_UPDATE);

	//Start Step counting timer (slave counter) and PWM timer (step pulse generator)
	HAL_TIM_Base_Start_IT(Axis->Step_Counter_Timer);

	if (Axis->PWM_Type == PWM_OUT_P)
	{
		HAL_TIM_PWM_Start(Axis->PWM_Timer, Axis->PWM_Channel);
	}
	else if (Axis->PWM_Type == PWM_OUT_N)
	{
		HAL_TIMEx_PWMN_Start(Axis->PWM_Timer, Axis->PWM_Channel);
	}
	//Set flags
	Axis->enabled = true;
	Axis->busy = true;

	statusFlags.tracking = true;
	statusFlags.moveEnabled = false;

}

void trackingStop(StepperMotor_t *Axis)
{
	//Update the axis position
	updatePosition(Axis);

	//Stop PWM timer
	if (Axis->PWM_Type == PWM_OUT_P)
	{
		HAL_TIM_PWM_Stop(Axis->PWM_Timer, Axis->PWM_Channel);
	}
	else if (Axis->PWM_Type == PWM_OUT_N)
	{
		HAL_TIMEx_PWMN_Stop(Axis->PWM_Timer, Axis->PWM_Channel);
	}
	//Stop STEP counting timer
	HAL_TIM_Base_Stop_IT(Axis->Step_Counter_Timer);

	Axis->Position.lastStepsRead = 0;
	__HAL_TIM_SET_COUNTER(Axis->Step_Counter_Timer, 0);

	Axis->busy = false;

	__HAL_TIM_SET_PRESCALER(RA_PWM_TIM, (CORE_FREQ/STEPPER_TIMER_HI_FREQ)-1); //Reset RA timer to initial values


	//Set flags
	Axis->busy = false;

	statusFlags.tracking = false;
	statusFlags.moveEnabled = true;

}

//Setting RA and DEC coordinates are set directly in degrees, even for RA
//Coordinates are then converted to hours for the RA
void setRaDec(float raAngle, float decAngle)
{
	coordinatesRaDec.raCoordinatesHours = raAngle / DEG_TO_HOURS; //Convert RA from degrees to hours
	coordinatesRaDec.raCoordinatesAngle = raAngle;
	coordinatesRaDec.decCoordinatesAngle = decAngle;
}

void updateRaDec(StepperMotor_t *Axis, int32_t deltaSteps)
{
	//deltaSteps is the distance traveled since last position update

	static uint32_t deltaTimeTicks = 0; //ms

	if (statusFlags.polarAligned == false || !Axis)
	{
		return;
	}

	if (Axis->AxisID == RA)
	{
		//Update the delta time for the RA coordinates update
		deltaTimeTicks = (HAL_GetTick() - coordinatesRaDec.lastRaUpdate);
		coordinatesRaDec.lastRaUpdate = HAL_GetTick();
		if (statusFlags.tracking == false)
		{
			//Update the RA position with the sidereal rate if the tracking is off
			coordinatesRaDec.raCoordinatesAngle += (float)(deltaTimeTicks/1000.0f)/SECONDS_PER_DEGREE;
			coordinatesRaDec.raCoordinatesHours = coordinatesRaDec.raCoordinatesAngle / DEG_TO_HOURS;
		}
	}

	if(deltaSteps == 0)
	{
		//If there is no movement, return
		return;
	}

	if (coordinatesRaDec.trackingPosition == true)
	{
		static bool wasFlipped = false;

		bool isFlipped = (DEC_AxisMotor.Position.angularPosition < 0.0f);

		if (isFlipped != wasFlipped)
		{
			if (isFlipped) {
				coordinatesRaDec.raCoordinatesAngle += 180.0f;
			} else {
				coordinatesRaDec.raCoordinatesAngle -= 180.0f;
			}
			wasFlipped = isFlipped;
		}

		if (Axis->AxisID == RA)
		{
			coordinatesRaDec.raCoordinatesAngle += (float)deltaSteps / (Axis->stepsPerArcmin*DEG);

			// Wrap around 360°
			if(coordinatesRaDec.raCoordinatesAngle > 360.0f) coordinatesRaDec.raCoordinatesAngle -= 360.0f;
			if(coordinatesRaDec.raCoordinatesAngle < 0.0f) coordinatesRaDec.raCoordinatesAngle += 360.0f;

			coordinatesRaDec.raCoordinatesHours = coordinatesRaDec.raCoordinatesAngle / 15.0f;
		}
		else if (Axis->AxisID == DEC)
		{
			//Update the DEC coordinates based on DEC mechanical position
			float decMechanicalPosition = DEC_AxisMotor.Position.angularPosition;

			//At mechanical position 0 the DEC celestial coordinates are +90°
			if (decMechanicalPosition < 0.0f)
			{
				coordinatesRaDec.decCoordinatesAngle = 90.0f + decMechanicalPosition;
			}
			else
			{
				coordinatesRaDec.decCoordinatesAngle = 90.0f - decMechanicalPosition;
			}
		}
	}
}



void gotoRaDec(float targetRA, float targetDec)
{
    if (statusFlags.polarAligned == false)
    {
        return;
    }

    float deltaRA = targetRA - coordinatesRaDec.raCoordinatesAngle;

    // Wrap to for the shortest path to move
    if (deltaRA > 180.0f)  deltaRA -= 360.0f;
    if (deltaRA < -180.0f) deltaRA += 360.0f;

    float targetMechRA = RA_AxisMotor.Position.angularPosition + deltaRA;

    //90degree offset (DEC mechanical position 0 is DEC celestial 90°)
    float targetMechDec = 90.0f - targetDec;

    float finalMechRA;
    float finalMechDec;

    //If the target is outside the GOTO_RA_MIN - GOTO_RA_MAX mechanical position, do meridian flip to keep some space for tracking
    if (targetMechRA < GOTO_RA_MIN || targetMechRA > GOTO_RA_MAX)
	{
		//Meridian flip
		finalMechRA = (targetMechRA > 0) ? (targetMechRA - 180.0f) : (targetMechRA + 180.0f);
		finalMechDec = -targetMechDec; //The DEC axis moves backwards (negative direction)
	}
	else
	{
		//Normal
		finalMechRA = targetMechRA;
		finalMechDec = targetMechDec; //The DEC axis moves forward (positive direction)
	}

    float raDeltaDeg = finalMechRA - RA_AxisMotor.Position.angularPosition;
    float decDeltaDeg = finalMechDec - DEC_AxisMotor.Position.angularPosition;

    stepperMove(&RA_AxisMotor, raDeltaDeg * DEG, RA_GOTO_SPEED);
    stepperMove(&DEC_AxisMotor, decDeltaDeg * DEG, DEC_GOTO_SPEED);
}

