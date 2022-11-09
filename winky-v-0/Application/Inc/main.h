#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "hw_conf.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ble_common.h"

typedef struct{
	uint8_t validationValue;
	uint8_t unusedWinky_context;
} WINKYAPP_Context_t;

extern PLACE_IN_SECTION("BLE_APP_CONTEXT") WINKYAPP_Context_t WINKYAPP_Context;

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
