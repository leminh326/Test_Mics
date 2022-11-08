/*
 * ht16k33.c
 *
 *  Created on: 28 mai 2019
 *      Author: clement
 */

#include "ht16k33.h"
#include "font8.h"



static HAL_StatusTypeDef ht16k33WriteRegister(I2C_HandleTypeDef* i2c, uint16_t devAddress, uint16_t reg);
static HAL_StatusTypeDef ht16k33Init(I2C_HandleTypeDef* i2c, uint16_t addr);
static HAL_StatusTypeDef ht16k33SetBrigthness(I2C_HandleTypeDef* i2c, uint16_t addr, ht16k33Dimming_t brightness);
static HAL_StatusTypeDef ht16k33SetBlinkRate(I2C_HandleTypeDef* i2c, uint16_t addr, ht16k33BlinkRate_t blinkRate);
static HAL_StatusTypeDef ht16k33WriteDisplay(I2C_HandleTypeDef* i2c, uint16_t addr, uint16_t* buffer);


//#include "i2c.h"

/**
  * @brief  Write to a ht16k33 register .
  * @param  i2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  reg the register to write
  * @retval HAL status
  */
static HAL_StatusTypeDef ht16k33WriteRegister(I2C_HandleTypeDef* i2c, uint16_t devAddress, uint16_t reg) {
	return HAL_I2C_Master_Transmit(i2c, devAddress, &reg, 1, 100);
}

/**
  * @brief  Initialize ht16k33.
  * @param  i2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @retval HAL status
  */
static HAL_StatusTypeDef ht16k33Init(I2C_HandleTypeDef* i2c, uint16_t addr){
	// check that i2c exists at this address
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(i2c,addr,1,20000);
	if( status != HAL_OK){
		return HAL_ERROR;
	}
	//Turn oscillator on
	ht16k33WriteRegister(i2c, addr, HT16K33_COMMAND_SYSTEM_SETUP | HT16K33_SETTING_OSCCILATOR_ON);
	//Turn blink off
	ht16k33SetBlinkRate(i2c,addr,HT16K33_BLINKRATE_OFF);
	//Set max brightness
	ht16k33SetBrigthness(i2c,addr,HT16K33_DIMMING_16_16);

	return HAL_OK;
};

/**
  * @brief  Set ht16k33 brightness.
  * @param  i2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param brightness level to set
  * @retval HAL status
  */
static HAL_StatusTypeDef ht16k33SetBrigthness(I2C_HandleTypeDef* i2c, uint16_t addr, ht16k33Dimming_t brightness){
	return ht16k33WriteRegister(i2c, addr, HT16K33_COMMAND_DIMMING | brightness);
};

/**
  * @brief  Set ht16k33 blink rate.
  * @param  i2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param blinkRate value
  * @retval HAL status
  */
static HAL_StatusTypeDef ht16k33SetBlinkRate(I2C_HandleTypeDef* i2c, uint16_t addr, ht16k33BlinkRate_t blinkRate){
	return ht16k33WriteRegister(i2c, addr, HT16K33_COMMAND_DISPLAY_SETUP | blinkRate | HT16K33_SETTING_DISPLAY_ON);
};

/**
  * @brief  Write ht16k33 display values.
  * @param  i2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param buffer value to wrte to the display
  * @retval HAL status
  */
static HAL_StatusTypeDef ht16k33WriteDisplay(I2C_HandleTypeDef* i2c, uint16_t addr, uint16_t *buffer)
{
	int32_t i;
	uint8_t ht16k33Buffer[16];
	for (i = 0; i < 8; i++)
	{
		ht16k33Buffer[(i*2)] = buffer[i] & 0xFF;
		ht16k33Buffer[(i*2)+1] = buffer[i] >> 8;
	}
	/* Check if we got an ACK or TIMEOUT error */
	return HAL_I2C_Mem_Write(i2c, addr, HT16K33_COMMAND_LED_DATA, 1, ht16k33Buffer, 16, 100);
}


/**
  * @brief  Initialize an 8x8 led matrix.
  * @param  handle Pointer to a ledMatrix_HandleTypedef structure that contains
  *                the configuration information for the specified matrix.
  * @retval HAL status
  */
HAL_StatusTypeDef ledMatrixInit(ledMatrix_HandleTypedef* handle){
	ledMatrixClear(handle);
	return ht16k33Init(handle->i2c, handle->addr);
};

/**
  * @brief  set led matrix brightness.
  * @param  handle Pointer to a ledMatrix_HandleTypedef structure that contains
  *                the configuration information for the specified matrix.
  * @param  brightness level
  * @retval HAL status
  */
HAL_StatusTypeDef ledMatrixSetBrigthness(ledMatrix_HandleTypedef* handle, ht16k33Dimming_t brightness)
{
	return ht16k33SetBrigthness(handle->i2c, handle->addr, brightness);
};

/**
  * @brief  set led matrix blinkrate.
  * @param  handle Pointer to a ledMatrix_HandleTypedef structure that contains
  *                the configuration information for the specified matrix.
  * @param  blinkRate value
  * @retval HAL status
  */
HAL_StatusTypeDef ledMatrixSetBlinkRate(ledMatrix_HandleTypedef* handle, ht16k33BlinkRate_t blinkRate)
{
	return ht16k33SetBlinkRate(handle->i2c, handle->addr,blinkRate);
};

/**
  * @brief  Draw a pixel.
  * @param  handle Pointer to a ledMatrix_HandleTypedef structure that contains
  *                the configuration information for the specified matrix.
  * @param  x value starting from 0
  * @param  y value starting from 0
  * @warning this function do not actualy render the pixel on the matrix you need to use ledMatrixRender afterward
  * @see ledMatrixRender
  */
void ledMatrixDrawPixel(ledMatrix_HandleTypedef* handle, int8_t x, int8_t y)
{
	if ((y < 0) || (y >= 8)) return;
	if ((x < 0) || (x >= 8)) return;

	// wrap around the x
	x += 7;
	x %= 8;

	handle->displayBuffer[y] |= (1 << x);
};

/**
  * @brief  Draw a bitmap.
  * @param  handle Pointer to a ledMatrix_HandleTypedef structure that contains
  *                the configuration information for the specified matrix.
  * @param  bitmap table
  * @warning this function do not actualy render the pixel on the matrix you need to use ledMatrixRender afterward
  * @see ledMatrixRender
  */
void ledMatrixDrawBitmap(ledMatrix_HandleTypedef* handle, const uint8_t bitmap[], ledMatrixTransformation_t transfo)
{
	for(int h = 0; h<8; h++){
		for(int w = 0; w<8; w++){
			if(bitmap[h] & (1 << w)){
				//ledMatrixDrawPixel(handle,w,h);
				if (transfo == ROTATION90)
				{
					ledMatrixDrawPixel(handle, 7 - w, h);

				}
				else if(transfo == ROTATION180)
				{
					ledMatrixDrawPixel(handle, h, 7 - w);
				}
				else if(transfo == ROTATION270)
				{
					ledMatrixDrawPixel(handle, h, 7 - w);
				}
				else if(transfo == HMIRROR)
				{
					ledMatrixDrawPixel(handle, h, 7 - w);
				}
				else if(transfo == VMIRROR)
				{
					ledMatrixDrawPixel(handle, h, 7 - w);
				}
				else
				{
					ledMatrixDrawPixel(handle, h, 7 - w);
				}
			}
		}
	}
};

/**
  * @brief  Render the matrix display.
  * @param  handle Pointer to a ledMatrix_HandleTypedef structure that contains
  *                the configuration information for the specified matrix.
  * @retval HAL status
  */
HAL_StatusTypeDef ledMatrixRender(ledMatrix_HandleTypedef* handle)
{
	return ht16k33WriteDisplay(handle->i2c, handle->addr, handle->displayBuffer);
};

/**
  * @brief  Clear the matrix.
  * @param  handle Pointer to a ledMatrix_HandleTypedef structure that contains
  *                the configuration information for the specified matrix.
  * @warning this function do not actualy render the pixel on the matrix you need to use ledMatrixRender afterward
  * @see ledMatrixRender
  */
void ledMatrixClear(ledMatrix_HandleTypedef* handle)
{
	for (uint8_t i=0; i<8; i++) {
		handle->displayBuffer[i] = 0;
	}
};

void ledMatrixPrintChar(ledMatrix_HandleTypedef* handle, char character) {
	ledMatrixClear(handle);
	ledMatrixDrawBitmap(handle, font8x8_basic[character & 0x7F], NOTRANSFO);
	ledMatrixRender(handle);
}
