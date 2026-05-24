/*
 * button.c
 *
 *  Created on: Aug 26, 2025
 *      Author: pecka
 */
#include "button.h"

static uint32_t btnLastChangeTime = 0;
static uint8_t btnStableState = 1;
static uint8_t btnLastRead = 1;


uint8_t readButtonDebounced() {
	uint8_t rawState = HAL_GPIO_ReadPin(PWR_BTN_GPIO_Port, PWR_BTN_Pin);

	if (rawState != btnLastRead) {
		btnLastChangeTime = HAL_GetTick();
		btnLastRead = rawState;
	}

	if ((HAL_GetTick() - btnLastChangeTime) > 30) { //30ms debounce
		btnStableState = rawState;
	}

	return btnStableState;
}
