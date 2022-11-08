/*
 * drv883x.c
 *
 *  Created on: 5 juil. 2019
 *      Author: Arnaud Guyon
 *
 *  Pololu Motor controllers like:
 *  DRV8835 https://www.pololu.com/product/2135
 *  DRV8838 https://www.pololu.com/product/2990
 *
 *
 */

#include "drv883x.h"
#include "stm32wbxx_hal_tim.h"

void drv883x_startMotor(drv883x_MotorController * controller);
void drv883x_stopMotor(drv883x_MotorController * controller);

void drv883x_init(drv883x_MotorController * controller,
		TIM_HandleTypeDef * pwm_tim_handle, uint32_t pwm_channel,
		GPIO_TypeDef * gpio_phase, uint16_t gpio_phase_pin,
		GPIO_TypeDef * gpio_sleep, uint16_t gpio_sleep_pin) {

	controller->pwm_tim_handle = pwm_tim_handle;
	controller->pwm_channel = pwm_channel;

	controller->pwm_update_config.OCMode = TIM_OCMODE_PWM1;
	controller->pwm_update_config.Pulse = 0;
	controller->pwm_update_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	controller->pwm_update_config.OCFastMode = TIM_OCFAST_DISABLE;

	// Direction (phase)
	controller->gpio_phase = gpio_phase;
	controller->gpio_phase_pin = gpio_phase_pin;

	// Sleep mode
	controller->gpio_sleep = gpio_sleep;
	controller->gpio_sleep_pin = gpio_sleep_pin;

	controller->mode = FORWARD;
	HAL_GPIO_WritePin(gpio_phase, gpio_phase_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpio_sleep, gpio_sleep_pin, GPIO_PIN_RESET);

	controller->speed = 0;
	drv883x_stopMotor(controller);
}

void drv883x_startMotor(drv883x_MotorController * controller) {
	HAL_TIM_PWM_Start(controller->pwm_tim_handle, controller->pwm_channel);
}

void drv883x_stopMotor(drv883x_MotorController * controller) {
	HAL_TIM_PWM_Stop(controller->pwm_tim_handle, controller->pwm_channel);
}

void drv883x_setMode(drv883x_MotorController * controller, drv883x_Mode mode) {
	switch(mode) {
	case FORWARD:
		if (controller->mode == STOP) {
			drv883x_startMotor(controller);
		}
		HAL_GPIO_WritePin(controller->gpio_sleep, controller->gpio_sleep_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(controller->gpio_phase, controller->gpio_phase_pin, GPIO_PIN_SET);
		break;
	case BACKWARD:
		if (controller->mode == STOP) {
			drv883x_startMotor(controller);
		}
		HAL_GPIO_WritePin(controller->gpio_sleep, controller->gpio_sleep_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(controller->gpio_phase, controller->gpio_phase_pin, GPIO_PIN_RESET);
		break;
	case SLEEP:
		HAL_GPIO_WritePin(controller->gpio_sleep, controller->gpio_sleep_pin, GPIO_PIN_RESET);
		break;
	case STOP:
		drv883x_stopMotor(controller);
		break;
	};
	controller->mode = mode;
}

drv883x_ErrorCode drv883x_setSpeed(drv883x_MotorController * controller, float speed) {
	if ((speed < 0) || (speed > 1)) {
		return DRV883X_WRONG_VALUE;
	}
	if (speed == controller->speed) {
		return DRV883X_NO_ERROR;
	}
	controller->speed = speed;
	uint32_t value = (uint32_t)(speed * 100);
	controller->pwm_update_config.Pulse = value;
	HAL_TIM_PWM_ConfigChannel(controller->pwm_tim_handle, &controller->pwm_update_config, controller->pwm_channel);
	if (speed == 0) {
		HAL_TIM_PWM_Stop(controller->pwm_tim_handle, controller->pwm_channel);
	} else {
		HAL_TIM_PWM_Start(controller->pwm_tim_handle, controller->pwm_channel);
	}
	return DRV883X_NO_ERROR;
}

