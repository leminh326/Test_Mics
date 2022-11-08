/*
 * peripheral_config.h
 *
 *  Created on: 9 août 2019
 *      Author: clement
 */

#ifndef WINKY_FW_INC_PERIPHERAL_CONFIG_H_
#define WINKY_FW_INC_PERIPHERAL_CONFIG_H_

// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include "stm32wbxx_hal.h"
#include "hw_conf.h"
// --------------------------------------------------------------------------------------------------------------------
// ----- public constant macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public variables
// --------------------------------------------------------------------------------------------------------------------
extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;

extern RTC_HandleTypeDef hrtc;

extern SAI_HandleTypeDef hsai_BlockA1;
extern SAI_HandleTypeDef hsai_BlockB1;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim16;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

// --------------------------------------------------------------------------------------------------------------------
// ----- public function prototypes
// --------------------------------------------------------------------------------------------------------------------
void gpio_init(void);
void dma_init(void);
void adc1_init(void);
void i2c_init(I2C_HandleTypeDef* i2c);
void rf_init(void);
void rtc_init(void);
void sai1_init(void);
void spi1_init(void);
void tim1_init(void);
void tim2_init(void);
void tim16_init(void);
void tim16_elapsed(void);
void usart1_uart_init(void);

void gpio_deinit(void);
void dma_deinit(void);
void adc1_deinit(void);
void i2c_deinit(void);
void rf_deinit(void);
void rtc_deinit(void);
void sai1_deinit(void);
void spi1_deinit(void);
void tim1_deinit(void);
void tim2_deinit(void);
void tim16_deinit(void);
void usart1_uart_deinit(void);

void Init_Peripheral(void);
void DeInit_Peripheral(void);

#endif /* WINKY_FW_INC_PERIPHERAL_CONFIG_H_ */
