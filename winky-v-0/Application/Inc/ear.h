/*
 * ear.h
 *
 *  Created on: 3 oct. 2019
 *      Author: clement
 */

#ifndef EAR_H_
#define EAR_H_

// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "stm32wbxx_hal.h"
// --------------------------------------------------------------------------------------------------------------------
// ----- public constant macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------

typedef enum {
	LEFT_EAR,
	RIGHT_EAR
} EarType;

typedef struct {
	bool isActive;
	uint8_t pulseCounter;
	uint8_t angle;
	GPIO_TypeDef* gpioPort;
	uint16_t gpioPin;
} WinkyEar;

typedef struct {
	EarType currentEar;
	WinkyEar rightEar;
	WinkyEar leftEar;
	uint8_t DCCounter;
} WinkyEars;

// --------------------------------------------------------------------------------------------------------------------
// ----- public variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public function prototypes
// --------------------------------------------------------------------------------------------------------------------

void WinkyEars_init();
void WinkyEar_rotate(EarType side, uint8_t degree);
void WinkyEar_PWMDCCallback();

#endif /* EAR_H_ */