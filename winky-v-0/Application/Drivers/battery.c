/*
 * battery.c
 *
 *  Created on: 16 ao�t 2019
 *      Author: Arnaud
 */

#include "battery.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;

const float VBAT_TO_VOLTAGE = 0.0011259f;
const float VBUS_TO_VOLTAGE = 0.0015215f;

// private
float Battery_getVoltage(uint32_t channel, float ratio_to_voltage);


void Battery_iSetChange(bool active) {
	HAL_GPIO_WritePin(ISET_CHANGE_GPIO_Port, ISET_CHANGE_Pin, active ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool Battery_getPowerGood() {
	GPIO_PinState state = HAL_GPIO_ReadPin(POWER_GOOD_IRQ_GPIO_Port, POWER_GOOD_IRQ_Pin);
	return (state == GPIO_PIN_RESET);
}

bool Battery_getChargeStatus() {
	GPIO_PinState state = HAL_GPIO_ReadPin(CHARGE_STATUS_IRQ_GPIO_Port, CHARGE_STATUS_IRQ_Pin);
	return (state == GPIO_PIN_RESET);
}

float Battery_getVBat() {
	return Battery_getVoltage(ADC_CHANNEL_1, VBAT_TO_VOLTAGE);
}

float Battery_getVBus() {
	return Battery_getVoltage(ADC_CHANNEL_2, VBUS_TO_VOLTAGE);
}

float Battery_getVoltage(uint32_t channel, float ratio_to_voltage) {

	// Update Channel
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		ERROR_HANDLER();
		return 0;
	}

	const int NUM_READS = 8;
	float result = 0;
	for(int i=0; i<NUM_READS; ++i) {
        if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        	ERROR_HANDLER();
        	return 0;
        }
        if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
        	ERROR_HANDLER();
        	return 0;
        } else {
        	result += HAL_ADC_GetValue(&hadc1);
        }
	}
	result /= NUM_READS;
	result *= ratio_to_voltage;
	return result;
}
