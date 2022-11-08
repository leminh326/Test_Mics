/*
 * rencoder.h
 *
 *  Created on: 11 juil. 2019
 *      Author: Arnaud
 */

#ifndef ROTARY_ENCODER_RENCODER_H_
#define ROTARY_ENCODER_RENCODER_H_

#include "stm32wbxx_hal.h"

typedef struct {
	TIM_HandleTypeDef * timer;
	uint32_t maxValue;				// corresponding to "Counter Period" of the timer
	int64_t previousValue;
} REncoder;

/**
  * @brief  Initialize the Encoder. Must be called before any other function
  * @param  encoder: user structure which is used internally to store values.
  *         This structure is initialized using the other parameters
  * @param  timer: Handle to the timer (ie &htim1)
  */
void REncoder_Init(REncoder * encoder, TIM_HandleTypeDef * timer);


/**
  * @brief  Reset the encoder counter (set to 0)
  * @param  encoder: user structure previously initialized using REncoder_Init().
  */
void REncoder_Reset(REncoder * encoder);

/**
  * @brief  Starts the encoder. Must be called before calling REncoder_GetRawValue or REncoder_GetDiff
  * @param  encoder: user structure previously initialized using REncoder_Init().
  */
void REncoder_StartEncoding(REncoder * encoder);


/**
  * @brief  Stops the encoder.
  * @param  encoder: user structure previously initialized using REncoder_Init().
  */
void REncoder_StopEncoding(REncoder * encoder);

/**
  * @brief  Returns the raw value of the encoder.
  * @param  encoder: user structure previously initialized using REncoder_Init().
  * @retval Raw value (between 0 and "Counter Period" of the timer)
  */
uint32_t REncoder_GetRawValue(REncoder * encoder);

/**
  * @brief  Returns variation of the encoder since the last call to that function
  * @param  encoder: user structure previously initialized using REncoder_Init().
  * @retval Variation (positive or negative) of the encoder
  */
int REncoder_GetDiff(REncoder * encoder);

#endif /* ROTARY_ENCODER_RENCODER_H_ */
