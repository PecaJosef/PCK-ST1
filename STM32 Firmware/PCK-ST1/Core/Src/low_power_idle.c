/*
 * low_power_idle_state.c
 *
 *  Created on: Aug 25, 2025
 *      Author: pecka
 */

#include "low_power_idle.h"

#define LED_DELAY 500
#define HOLD_TIME 4*LED_DELAY
#define RESET_TIME 1000

// --- Sequence state ---
typedef enum {
	BTN_SEQ_WAIT_FIRST_PRESS,
	BTN_SEQ_WAIT_FIRST_RELEASE,
	BTN_SEQ_WAIT_SECOND_PRESS,
	BTN_SEQ_HOLDING,
	BTN_SEQ_DONE
} BtnSeqState_t;


static BtnSeqState_t btnSeqState = BTN_SEQ_WAIT_FIRST_PRESS;
static uint32_t secondPressStart = 0;
static uint32_t lastLedTime = 0;
static int ledStep = 0;

void handleLowPowerIdle() {
	uint8_t btn = readButtonDebounced();

	switch (btnSeqState) {
		case BTN_SEQ_WAIT_FIRST_PRESS:
			if (btn == PRESSED) // pressed
			{
				btnSeqState = BTN_SEQ_WAIT_FIRST_RELEASE;
				// turn on LEDs initially
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
			}
			break;

		case BTN_SEQ_WAIT_FIRST_RELEASE:
			if (btn == RELEASED) // released
			{
				btnSeqState = BTN_SEQ_WAIT_SECOND_PRESS;
				secondPressStart = HAL_GetTick(); // start 2s window
			}
			break;

		case BTN_SEQ_WAIT_SECOND_PRESS:
			if ((HAL_GetTick() - secondPressStart) > RESET_TIME)
			{
				// Timeout, reset sequence
				btnSeqState = BTN_SEQ_WAIT_FIRST_PRESS;
				ledStep = 0;
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			} else if (btn == PRESSED) { // pressed again
				btnSeqState = BTN_SEQ_HOLDING;
				secondPressStart = HAL_GetTick();
				lastLedTime = secondPressStart;
				ledStep = 0;
				/*
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
				*/
			}
			break;

		case BTN_SEQ_HOLDING:
			if (btn == RELEASED) // released too early → reset
			{
				btnSeqState = BTN_SEQ_WAIT_FIRST_PRESS;
				ledStep = 0;
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
				break;
			}
			// Check how long held
			uint32_t heldTime = HAL_GetTick() - secondPressStart;

			// light LEDs progressively
			if ((heldTime / LED_DELAY) > ledStep && ledStep < 4)
			{
				ledStep++;
				switch (ledStep) {
					case 1: HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); break;
					case 2: HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); break;
					case 3: HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); break;
					case 4: HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET); break;
				}
			}

			if (heldTime >= HOLD_TIME) {
				btnSeqState = BTN_SEQ_DONE;
				HAL_GPIO_WritePin(PWR_BTN_LED_GPIO_Port, PWR_BTN_LED_Pin, GPIO_PIN_SET);
			}
			break;

		case BTN_SEQ_DONE:
			controlState = HOMING;
			btnSeqState = BTN_SEQ_WAIT_FIRST_PRESS; // reset for next time
			break;
	}
}
