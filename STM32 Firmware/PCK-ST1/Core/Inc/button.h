/*
 * button.h
 *
 *  Created on: Aug 26, 2025
 *      Author: pecka
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "control_loop.h"

#define PRESSED 1
#define RELEASED 0

uint8_t readButtonDebounced();

#endif /* INC_BUTTON_H_ */
