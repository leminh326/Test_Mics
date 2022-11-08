/*
 * winky_touch.h
 *
 *  Created on: 17 nov. 2019
 *      Author: clement
 */

#ifndef WINKY_TOUCH_H_
#define WINKY_TOUCH_H_

// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include "IQS5xx.h"
#include "IQS333.h"
// --------------------------------------------------------------------------------------------------------------------
// ----- public constant macros
// --------------------------------------------------------------------------------------------------------------------

#define WINKY_ONOFF_BUTTON_X 	640
#define WINKY_ONOFF_BUTTON_Y 	0

#define WINKY_HEAD_FRONT_TRACKPAD_X_MIN 	160
#define WINKY_HEAD_FRONT_TRACKPAD_X_MAX 	1280
#define WINKY_HEAD_FRONT_TRACKPAD_Y_MIN 	1067
#define WINKY_HEAD_FRONT_TRACKPAD_Y_MAX 	1280

#define WINKY_HEAD_BACK_TRACKPAD_X_MIN	0
#define WINKY_HEAD_BACK_TRACKPAD_X_MAX	1280
#define WINKY_HEAD_BACK_TRACKPAD_Y_MIN	426
#define WINKY_HEAD_BACK_TRACKPAD_Y_MAX	640

#define HOLD_TIME				100
#define LONG_HOLD_TIME			1000
#define VERY_LONG_HOLD_TIME		7000
#define SINGLE_TAP_TIME			200
#define LONG_TAP_TIME			2000
#define VERY_LONG_TAP_TIME		10000

#define FLAGS_MSK_IQS5XX_RDY 		0x0002U
// --------------------------------------------------------------------------------------------------------------------
// ----- public function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------
typedef enum {released, pressed, hold, longHold, veryLongHold, singleTap, longTap, veryLongTap} buttonEvent;
typedef enum {
	headTouch = 0x01,
	bodyTouch = 0x02
} touchType;
// --------------------------------------------------------------------------------------------------------------------
// ----- public variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public function prototypes
// --------------------------------------------------------------------------------------------------------------------

void winky_touch_init(touchType touchType);
void winky_touch_deinit(touchType touchType);
void winky_touch_enableLowPower();
void winky_touch_disableLowPower();

void winky_touch_onOffButtonCallback(buttonEvent event);
void winky_touch_headTrackpadFrontCallback(uint8_t x, uint8_t y);
void winky_touch_headTrackpadBackCallback(uint8_t x, uint8_t y);
#endif /* WINKY_TOUCH_H_ */
