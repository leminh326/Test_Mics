/*
 * ht16k33.h
 *
 *  Created on: 28 mai 2019
 *      Author: clement
 */

#ifndef HT16K33_HT16K33_H_
#define HT16K33_HT16K33_H_

#include "stm32wbxx_hal.h"

enum
{
	HT16K33_COMMAND_LED_DATA	= 0x00,
	HT16K33_COMMAND_SYSTEM_SETUP	= 0x20,
	HT16K33_COMMAND_DISPLAY_SETUP	= 0x80,
	HT16K33_COMMAND_DIMMING	= 0xE0,
};

enum
{
	HT16K33_SETTING_OSCCILATOR_OFF	= 0x00,
	HT16K33_SETTING_OSCCILATOR_ON	= 0x01,
};

typedef enum
{
	HT16K33_DIMMING_1_16    = 0x00,
	HT16K33_DIMMING_2_16    = 0x01,
	HT16K33_DIMMING_3_16    = 0x02,
	HT16K33_DIMMING_4_16  	= 0x03,
	HT16K33_DIMMING_5_16   	= 0x04,
	HT16K33_DIMMING_6_16   	= 0x05,
	HT16K33_DIMMING_7_16   	= 0x06,
	HT16K33_DIMMING_8_16   	= 0x07,
	HT16K33_DIMMING_9_16   	= 0x08,
	HT16K33_DIMMING_10_16 	= 0x09,
	HT16K33_DIMMING_11_16	= 0x0A,
	HT16K33_DIMMING_12_16	= 0x0B,
	HT16K33_DIMMING_13_16	= 0x0C,
	HT16K33_DIMMING_14_16	= 0x0D,
	HT16K33_DIMMING_15_16	= 0x0E,
	HT16K33_DIMMING_16_16	= 0x0F
} ht16k33Dimming_t;

enum
{
	HT16K33_SETTING_DISPLAY_OFF	= 0x00,
	HT16K33_SETTING_DISPLAY_ON	= 0x01,
};


typedef enum
{
	HT16K33_BLINKRATE_OFF       = 0x00,
	HT16K33_BLINKRATE_2HZ       = 0x02,
	HT16K33_BLINKRATE_1HZ       = 0x04,
	HT16K33_BLINKRATE_HALFHZ    = 0x06
} ht16k33BlinkRate_t;


typedef struct {
	uint16_t addr;

	I2C_HandleTypeDef* i2c;

	uint8_t  id;        /* Chip ID */
	uint16_t displayBuffer[8];
} ledMatrix_HandleTypedef;

typedef enum
{
	ROTATION90      = 0x00,
	ROTATION180     = 0x01,
	ROTATION270     = 0x02,
	VMIRROR    		= 0x03,
	HMIRROR 		= 0x04,
	NOTRANSFO		= 0x05
}ledMatrixTransformation_t;


HAL_StatusTypeDef ledMatrixInit(ledMatrix_HandleTypedef* handle);
HAL_StatusTypeDef ledMatrixSetBrigthness(ledMatrix_HandleTypedef* handle, ht16k33Dimming_t brightness);
HAL_StatusTypeDef ledMatrixSetBlinkRate(ledMatrix_HandleTypedef* handle, ht16k33BlinkRate_t blinkRate);
void ledMatrixDrawPixel(ledMatrix_HandleTypedef* handle, int8_t x, int8_t y);
void ledMatrixDrawBitmap(ledMatrix_HandleTypedef* handle, const uint8_t bitmap[], ledMatrixTransformation_t transfo);
HAL_StatusTypeDef ledMatrixRender(ledMatrix_HandleTypedef* handle);
void ledMatrixClear(ledMatrix_HandleTypedef* handle);
void ledMatrixPrintChar(ledMatrix_HandleTypedef* handle, char character);
#endif /* HT16K33_HT16K33_H_ */
