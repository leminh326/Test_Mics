/*
 * motorhead.c
 *
 *  Created on: 17 juil. 2019
 *      Author: Arnaud
 */

#include "motorhead.h"

#include "main.h"
#include "peripheral_config.h"
#include "stm32wbxx_hal_tim.h"
#include "stm32wbxx_hal_gpio.h"

void MotorHead_init(MotorHead * motorhead, bool useEncoder) {

	drv883x_init(&motorhead->motorController,
			&htim2, TIM_CHANNEL_2,
			PH_M_GPIO_Port, PH_M_Pin,
			nSLEEP_M_GPIO_Port, nSLEEP_M_Pin);

	motorhead->encoderValue = 0;
	motorhead->useEncoder = useEncoder;
	if(useEncoder)
	{
		REncoder_Init(&motorhead->encoder, &htim1);
		REncoder_StartEncoding(&motorhead->encoder);
	}
}

void MotorHead_forward(MotorHead * motorhead, float speed) {
	  drv883x_setMode(&motorhead->motorController, FORWARD);
	  drv883x_setSpeed(&motorhead->motorController, speed);
}

void MotorHead_backward(MotorHead * motorhead, float speed) {
	  drv883x_setMode(&motorhead->motorController, BACKWARD);
	  drv883x_setSpeed(&motorhead->motorController, speed);
}

void MotorHead_stop(MotorHead * motorhead) {
	drv883x_setMode(&motorhead->motorController, STOP);
}

void MotorHead_sleep(MotorHead * motorhead) {
	drv883x_setMode(&motorhead->motorController, SLEEP);
}

int32_t MotorHead_getEncoderValue(MotorHead * motorhead) {
	if(motorhead->useEncoder)
	{
		motorhead->encoderValue += REncoder_GetDiff(&motorhead->encoder);
	}
	return motorhead->encoderValue;
}

void MotorHead_resetEncoderValue(MotorHead * motorhead) {
	motorhead->encoderValue = 0;
	if(motorhead->useEncoder)
	{
		REncoder_Reset(&motorhead->encoder);
	}
}

