/*
 * rencoder.c
 *
 *  Created on: 11 juil. 2019
 *      Author: Arnaud Guyon
 */

#include "rencoder.h"
#include "stm32wbxx_hal_tim.h"

void REncoder_Init(REncoder * encoder, TIM_HandleTypeDef * timer) {
	encoder->timer = timer;
	encoder->maxValue = timer->Init.Period;
	encoder->previousValue = 0;
	encoder->timer->Instance->CNT = 0;
}

void REncoder_Reset(REncoder * encoder) {
	encoder->previousValue = 0;
	encoder->timer->Instance->CNT = 0;
}

void REncoder_StartEncoding(REncoder * encoder) {
	HAL_TIM_Encoder_Start(encoder->timer, TIM_CHANNEL_ALL);
}

void REncoder_StopEncoding(REncoder * encoder) {
	HAL_TIM_Encoder_Stop(encoder->timer, TIM_CHANNEL_ALL);
}

uint32_t REncoder_GetRawValue(REncoder * encoder) {
	return encoder->timer->Instance->CNT;
}

int REncoder_GetDiff(REncoder * encoder) {
	int64_t raw64 = encoder->timer->Instance->CNT;
	int64_t previous = encoder->previousValue;
	int64_t diff = (raw64 - previous);
	int64_t max = encoder->maxValue + 1;
	encoder->previousValue = raw64;

	if (diff >= 0) {
		if (diff < max/2) {
			return diff;
		} else {
			return -(previous + (max - raw64));
		}
	} else {
		if (diff > -max/2) {
			return diff;
		} else {
			return (raw64 + (max - previous));
		}
	}
}

