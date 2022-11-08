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

#include "app_examples_conf.h" //TODO remove it from final version

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_conf.h"
#include "hw_conf.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ble_common.h"

#if (IMU_EXAMPLE != 0)
#include "lsm6dsl.h"
#endif

#if (EAR_EXAMPLE != 0)
#include "ear.h"
#endif

#if (BATTERY_EXAMPLE != 0)
	#include "ht16k33.h"
	#include "battery.h"
	#include "stdbool.h"
#endif

#if ( HEAD_EXAMPLE != 0)
#include "motorhead.h"
#endif

#if ( GEST_EXAMPLE != 0)
#include "apds9960.h"
#endif

#if ( TOF_EXAMPLE != 0)
osThreadId_t tofTaskHandle;
void StartToFTask(void *argument);
#endif

#if (	MATRIX_EXAMPLE != 0)
#include "ht16k33.h"
#endif

#if ( SOUND_EXAMPLE != 0)
#include "audio_codec_tlv300dac31.h"
#endif
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct{
#if (IMU_EXAMPLE != 0)
	LSM6DSL_Axes_t IMUChar;
	uint8_t IMUNotification_Status;
#endif
	uint8_t validationValue;
	uint8_t unusedWinky_context;
} WINKYAPP_Context_t;

/**
 * START of Section BLE_DRIVER_CONTEXT
 */
extern PLACE_IN_SECTION("BLE_APP_CONTEXT") WINKYAPP_Context_t WINKYAPP_Context;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */
//osEventFlagsId_t interruptFlagId;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(const char *file, const uint16_t line);

#define ERROR_HANDLER() Error_Handler(__FILE__, __LINE__)

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    signed char *pcTaskName );

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
