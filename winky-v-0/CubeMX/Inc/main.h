/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RDY_IQS572_WAKEUP2_Pin GPIO_PIN_13
#define RDY_IQS572_WAKEUP2_GPIO_Port GPIOC
#define BOOT0_Pin GPIO_PIN_3
#define BOOT0_GPIO_Port GPIOH
#define VBAT_MEAS_Pin GPIO_PIN_0
#define VBAT_MEAS_GPIO_Port GPIOC
#define VBUS_MEAS_Pin GPIO_PIN_1
#define VBUS_MEAS_GPIO_Port GPIOC
#define RDY_I2C3_Pin GPIO_PIN_2
#define RDY_I2C3_GPIO_Port GPIOC
#define TIM2_CH1_PWMVOL_Pin GPIO_PIN_0
#define TIM2_CH1_PWMVOL_GPIO_Port GPIOA
#define TIM2_CH2_PWMMOT_Pin GPIO_PIN_1
#define TIM2_CH2_PWMMOT_GPIO_Port GPIOA
#define ToF_IRQ_WAKEUP4_Pin GPIO_PIN_2
#define ToF_IRQ_WAKEUP4_GPIO_Port GPIOA
#define ToF_IRQ_WAKEUP4_EXTI_IRQn EXTI2_IRQn
#define FLASH_CS_Pin GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOC
#define Gesture_IRQ_WAKEUP5_Pin GPIO_PIN_5
#define Gesture_IRQ_WAKEUP5_GPIO_Port GPIOC
#define Gesture_IRQ_WAKEUP5_EXTI_IRQn EXTI9_5_IRQn
#define PH_M_Pin GPIO_PIN_2
#define PH_M_GPIO_Port GPIOB
#define nSLEEP_M_Pin GPIO_PIN_10
#define nSLEEP_M_GPIO_Port GPIOB
#define ISET_CHANGE_Pin GPIO_PIN_11
#define ISET_CHANGE_GPIO_Port GPIOB
#define EN_5VBO_Pin GPIO_PIN_0
#define EN_5VBO_GPIO_Port GPIOB
#define EN_1V8DIG_Pin GPIO_PIN_1
#define EN_1V8DIG_GPIO_Port GPIOB
#define EN_3V0ANA_Pin GPIO_PIN_4
#define EN_3V0ANA_GPIO_Port GPIOE
#define POWER_GOOD_IRQ_Pin GPIO_PIN_12
#define POWER_GOOD_IRQ_GPIO_Port GPIOB
#define EAR_L_Pin GPIO_PIN_15
#define EAR_L_GPIO_Port GPIOB
#define POS_EN_Pin GPIO_PIN_6
#define POS_EN_GPIO_Port GPIOC
#define POS_EN_EXTI_IRQn EXTI9_5_IRQn
#define EAR_R_Pin GPIO_PIN_11
#define EAR_R_GPIO_Port GPIOA
#define LBO_5VBOOST_Pin GPIO_PIN_12
#define LBO_5VBOOST_GPIO_Port GPIOA
#define RST_IQS572_AUDIO_GPIO1_Pin GPIO_PIN_15
#define RST_IQS572_AUDIO_GPIO1_GPIO_Port GPIOA
#define ToF_xSHUT_Pin GPIO_PIN_10
#define ToF_xSHUT_GPIO_Port GPIOC
#define EEPROM_CS_Pin GPIO_PIN_11
#define EEPROM_CS_GPIO_Port GPIOC
#define IMU_IRQ_WAKEUP3_Pin GPIO_PIN_12
#define IMU_IRQ_WAKEUP3_GPIO_Port GPIOC
#define IMU_IRQ_WAKEUP3_EXTI_IRQn EXTI15_10_IRQn
#define AUDIO_RST_Pin GPIO_PIN_0
#define AUDIO_RST_GPIO_Port GPIOD
#define CHARGE_STATUS_IRQ_Pin GPIO_PIN_1
#define CHARGE_STATUS_IRQ_GPIO_Port GPIOD
#define CHARGE_STATUS_IRQ_EXTI_IRQn EXTI1_IRQn
#define LOG_TX_Pin GPIO_PIN_6
#define LOG_TX_GPIO_Port GPIOB
#define LOG_RX_Pin GPIO_PIN_7
#define LOG_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
