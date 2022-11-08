/*
 * winky_touch.c
 *
 *  Created on: 17 nov. 2019
 *      Author: clement
 */


// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include "winky_touch.h"
#include "main.h"
// --------------------------------------------------------------------------------------------------------------------
// ----- local constant macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------
osThreadId_t iqs5xxThreadId;
osThreadId_t iqs333ThreadId;
extern osEventFlagsId_t interruptFlagId;



typedef struct {
	buttonState lastButtonState;
	buttonEvent lastButtonGesture;
	uint32_t lastButtonStateChangeTime;
	IQS5xx_ButtonTypeDef iqsButton;
}Winky_TouchOnOffButtonTypeDef;



Winky_TouchOnOffButtonTypeDef onOffButton;
IQS5xx_TrackpadTypeDef headTrackpadFront;
IQS5xx_TrackpadTypeDef headTrackpadBack;

// --------------------------------------------------------------------------------------------------------------------
// ----- local function prototypes
// --------------------------------------------------------------------------------------------------------------------
static void onOffButtonCallback(buttonState state);
static void headTrackpadFrontCallback(uint8_t x, uint8_t y);
static void headTrackpadBackCallback(uint8_t x, uint8_t y);
static void winky_head_touch_process(void *argument);
static void winky_body_touch_process(void *argument);

// --------------------------------------------------------------------------------------------------------------------
// ----- local functions
// --------------------------------------------------------------------------------------------------------------------
static void onOffButtonCallback(buttonState state)
{
	uint32_t changeTime = HAL_GetTick();
	if (state == buttonPressed)
	{
		if (onOffButton.lastButtonState != buttonPressed)
		{
			onOffButton.lastButtonStateChangeTime = changeTime;
			winky_touch_onOffButtonCallback(pressed);
		}
		else // Button Hold
		{
			uint32_t changeTimeDiff = HAL_GetTick() - onOffButton.lastButtonStateChangeTime;
			if ((changeTimeDiff > VERY_LONG_HOLD_TIME) && (onOffButton.lastButtonGesture != veryLongHold))
			{
				onOffButton.lastButtonGesture = veryLongHold;
				winky_touch_onOffButtonCallback(veryLongHold);
			}
			else if ((changeTimeDiff > LONG_HOLD_TIME) && (changeTimeDiff < VERY_LONG_HOLD_TIME) && (onOffButton.lastButtonGesture != longHold))
			{
				onOffButton.lastButtonGesture = longHold;
				winky_touch_onOffButtonCallback(longHold);

			}
			else if ((changeTimeDiff > HOLD_TIME) && (changeTimeDiff < LONG_HOLD_TIME) &&(onOffButton.lastButtonGesture != hold))
			{
				onOffButton.lastButtonGesture = hold;
				winky_touch_onOffButtonCallback(hold);
			}
		}

	}
	else // state == buttonReleased
	{
		winky_touch_onOffButtonCallback(released);
		uint32_t changeTimeDiff = HAL_GetTick() - onOffButton.lastButtonStateChangeTime;
		if ((changeTimeDiff > VERY_LONG_TAP_TIME) && (onOffButton.lastButtonGesture != veryLongTap))
		{
			onOffButton.lastButtonGesture = veryLongTap;
			winky_touch_onOffButtonCallback(veryLongTap);
		}
		else if ((changeTimeDiff > LONG_TAP_TIME) && (onOffButton.lastButtonGesture != longTap))
		{
			onOffButton.lastButtonGesture = longTap;
			winky_touch_onOffButtonCallback(longTap);
		}
		else if ((changeTimeDiff > SINGLE_TAP_TIME) && (onOffButton.lastButtonGesture != singleTap))
		{
			onOffButton.lastButtonGesture = singleTap;
			winky_touch_onOffButtonCallback(singleTap);
		}
		onOffButton.lastButtonStateChangeTime = changeTime;

	}
	onOffButton.lastButtonState = state;
}

static void headTrackpadFrontCallback(uint8_t x, uint8_t y)
{
	printf("Front Head SLider touched\r\n");
	winky_touch_headTrackpadFrontCallback(x,y);
}

static void headTrackpadBackCallback(uint8_t x, uint8_t y)
{
	printf("Back Head SLider touched\r\n");
	winky_touch_headTrackpadBackCallback(x,y);
}

static void winky_head_touch_process(void *argument)
{
	UNUSED(argument);

	uint32_t flags;
	while(1)
	{
		flags = osEventFlagsWait(interruptFlagId, FLAGS_MSK_IQS5XX_RDY,osFlagsWaitAny,osWaitForever);

		if ((flags & 0x80000000)  == 0x80000000)
		{
			ERROR_HANDLER();
			return;
		}
//		while(HAL_GPIO_ReadPin(RDY_IQS572_WAKEUP2_GPIO_Port, RDY_IQS572_WAKEUP2_Pin) != GPIO_PIN_SET)	{
//			osDelay(1);
//		}
		//IQS5xx_CheckVersion();
		IQS5xx_Refresh_Data();
		IQS5xx_Process_New_Data();
		IQS5xx_EndCommWindows();
		osThreadYield();
	}
}

static void winky_body_touch_process(void *argument)
{
	UNUSED(argument);
	while(1)
	{
		IQS333_Process();
		osThreadYield();
	}
}
// --------------------------------------------------------------------------------------------------------------------
// ----- public functions
// --------------------------------------------------------------------------------------------------------------------

//Can only be call from winky_touch callback as IQSxxWindows should still be open
void winky_touch_enableLowPower()
{
	IQS5xx_EnableALP();
}

//Can only be call from winky_touch callback as IQSxxWindows should still be open
void winky_touch_disableLowPower()
{
	IQS5xx_DisableALP();
}

void winky_touch_init(touchType touchType)
{
	if ((touchType & headTouch) == headTouch)
	{
		IQS5xx_ButtonTypeDef iqsButton;
		iqsButton.Xpos = WINKY_ONOFF_BUTTON_X;
		iqsButton.Ypos = WINKY_ONOFF_BUTTON_Y;
		iqsButton.callback = &onOffButtonCallback;
		onOffButton.iqsButton = iqsButton;
		IQS5xx_RegisterButton(&onOffButton.iqsButton);

		headTrackpadFront.Xmin = WINKY_HEAD_FRONT_TRACKPAD_X_MIN;
		headTrackpadFront.Xmax = WINKY_HEAD_FRONT_TRACKPAD_X_MAX;
		headTrackpadFront.Ymin = WINKY_HEAD_FRONT_TRACKPAD_Y_MIN;
		headTrackpadFront.Ymax = WINKY_HEAD_FRONT_TRACKPAD_Y_MAX;
		headTrackpadFront.callback = &headTrackpadFrontCallback;
		IQS5xx_RegisterTrackpad(&headTrackpadFront);

		headTrackpadBack.Xmin = WINKY_HEAD_BACK_TRACKPAD_X_MIN;
		headTrackpadBack.Xmax = WINKY_HEAD_BACK_TRACKPAD_X_MAX;
		headTrackpadBack.Ymin = WINKY_HEAD_BACK_TRACKPAD_Y_MIN;
		headTrackpadBack.Ymax = WINKY_HEAD_BACK_TRACKPAD_Y_MAX;
		headTrackpadBack.callback = &headTrackpadBackCallback;
		IQS5xx_RegisterTrackpad(&headTrackpadBack);

		/* definition and creation of myTask */
		const osThreadAttr_t winkyHeadTouchThread_attributes = {
				.name = "headTouchTask",
				.priority = (osPriority_t) osPriorityNormal,
				.stack_size = 128 * 6
		};
		iqs5xxThreadId = osThreadNew(winky_head_touch_process, NULL, &winkyHeadTouchThread_attributes);

		HAL_GPIO_WritePin(RST_IQS572_AUDIO_GPIO1_GPIO_Port, RST_IQS572_AUDIO_GPIO1_Pin, GPIO_PIN_RESET);
		osDelay(50);
		HAL_GPIO_WritePin(RST_IQS572_AUDIO_GPIO1_GPIO_Port, RST_IQS572_AUDIO_GPIO1_Pin, GPIO_PIN_SET);
		osDelay(50);
	}
	if ((touchType & bodyTouch) == bodyTouch)
	{
		const osThreadAttr_t winkyBodyTouchThread_attributes = {
				.name = "bodyTouchTask",
				.priority = (osPriority_t) osPriorityLow,
				.stack_size = 128 * 4
		};
		iqs333ThreadId = osThreadNew(winky_body_touch_process, NULL, &winkyBodyTouchThread_attributes);
	}
}

void winky_touch_deinit(touchType touchType)
{
	if ((touchType & headTouch) == headTouch)
	{
		IQS5xx_UnregisterAll();
		osThreadTerminate(iqs5xxThreadId);
	}
	if ((touchType & bodyTouch) == bodyTouch)
	{
		osThreadTerminate(iqs333ThreadId);

	}
}

//TODO @MAINBOT redefine non __weak callback function
__weak void winky_touch_onOffButtonCallback(buttonEvent event){};
__weak void winky_touch_headTrackpadFrontCallback(uint8_t x, uint8_t y){};
__weak void winky_touch_headTrackpadBackCallback(uint8_t x, uint8_t y){};
