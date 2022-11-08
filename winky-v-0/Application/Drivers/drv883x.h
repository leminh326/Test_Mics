/*
 * drv883x.h
 *
 *  Created on: 5 juil. 2019
 *      Author: Arnaud
 */

#ifndef POLOLU_DRV883X_DRV883X_H_
#define POLOLU_DRV883X_DRV883X_H_

#include "stm32wbxx_hal.h"

typedef enum {
	FORWARD		= 1,
	BACKWARD	= 2,
	STOP		= 3,
	SLEEP		= 4
} drv883x_Mode;

typedef struct {
	TIM_HandleTypeDef *	pwm_tim_handle;
	uint32_t			pwm_channel;
	TIM_OC_InitTypeDef	pwm_update_config;
	GPIO_TypeDef *		gpio_phase;
	uint16_t			gpio_phase_pin;
	GPIO_TypeDef *		gpio_sleep;
	uint16_t			gpio_sleep_pin;
	drv883x_Mode		mode;
	float				speed;
} drv883x_MotorController;

typedef enum {
	DRV883X_NO_ERROR		= 0,
	DRV883X_WRONG_VALUE		= 1
} drv883x_ErrorCode;


/**
  * @brief  Initialize DRV883x controller. Must be called before any other function
  * @param  controller: user structure which is then used internally to store values.
  *         This structure is initialized using the other parameters
  * @param  pwm_tim_handle: Handle to the PWM timer (ie &htim2)
  * @param	pwm_channel: PWM channel (ie TIM_CHANNEL_2)
  * @param	gpio_phase: GPIO linked to the head direction (ie GPIOB)
  * @param	gpio_phase_pin: pin of the head direction GPIO (ie GPIO_PIN_2)
  * @param	gpio_sleep: GPIO linked to the sleep mode
  * @param	gpio_sleep_pin: pin of the sleep GPIO
  */
void drv883x_init(drv883x_MotorController * controller,
		TIM_HandleTypeDef * pwm_tim_handle, uint32_t pwm_channel,
		GPIO_TypeDef * gpio_phase, uint16_t gpio_phase_pin,
		GPIO_TypeDef * gpio_sleep, uint16_t gpio_sleep_pin);

/**
  * @brief  Selects the mode & direction of the motor
  * @param  controller: user structure previously initialized using init().
  * @param  mode: FORWARD, BACKWARD, SLEEP_MODE (changing speed has no effect in SLEEP_MODE)
  */
void drv883x_setMode(drv883x_MotorController * controller, drv883x_Mode mode);

/**
  * @brief  Sets the speed of the motor (no matter the direction)
  * @param  controller: user structure previously initialized using init().
  * @param  speed: accepts [0-1] values. The Timer period must be previously set to 100.
  *         When speed is set to 0, the motor is stopped.
  *         Speed is ignored in SLEEP_MODE
  * @retval drv883x_ErrorCode potential error code or DRV883X_NO_ERROR if no error
  */
drv883x_ErrorCode drv883x_setSpeed(drv883x_MotorController * controller, float speed);

#endif /* POLOLU_DRV883X_DRV883X_H_ */
