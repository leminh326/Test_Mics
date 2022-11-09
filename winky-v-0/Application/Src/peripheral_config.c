/*
 * peripheral_config.c
 *
 *  Created on: 9 août 2019
 *      Author: clement
 */


// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include "main.h"
#include "peripheral_config.h"
#include "stm32wbxx_hal.h"

ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;


UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

void Init_Peripheral (void)
{

}

void DeInit_Peripheral (void)
{

}
