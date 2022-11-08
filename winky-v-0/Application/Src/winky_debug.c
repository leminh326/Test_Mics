/*
 * winky_debug.c
 *
 *  Created on: 21 janv. 2020
 *      Author: clement
 */


// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "winky_debug.h"
#include "peripheral_config.h"

extern UART_HandleTypeDef huart1;
// --------------------------------------------------------------------------------------------------------------------
// ----- local constant macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local function prototypes
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local functions
// --------------------------------------------------------------------------------------------------------------------
#if (CFG_WINKY_TRACE != 0)
void uart_log_implementation_write(uint8_t * buffer, uint16_t size)
{
	HAL_UART_Transmit(&huart1, buffer, size,100);
}
#else
void uart_log_implementation_write(uint8_t * buffer, uint16_t size){};
#endif

size_t _write(int handle, const unsigned char * buf, size_t bufSize)
{
	uart_log_implementation_write((uint8_t *)buf,bufSize);
	return bufSize;
}

size_t __write(int handle, const unsigned char * buf, size_t bufSize)
{
	uart_log_implementation_write((uint8_t *)buf,bufSize);
	return bufSize;
}

/**
 * @brief  DbgTraceGetFileName: Return filename string extracted from full path information
 * @param  *fullPath Fullpath string (path + filename)
 * @retval char* Pointer on filename string
 */

const char *DbgTraceGetFileName(const char *fullpath)
{
  const char *ret = fullpath;

  if (strrchr(fullpath, '\\') != NULL)
  {
    ret = strrchr(fullpath, '\\') + 1;
  }
  else if (strrchr(fullpath, '/') != NULL)
  {
    ret = strrchr(fullpath, '/') + 1;
  }

  return ret;
}

// --------------------------------------------------------------------------------------------------------------------
// ----- public functions
// --------------------------------------------------------------------------------------------------------------------

void winky_debug_init()
{
#if (CFG_WINKY_TRACE != 0)
	usart1_uart_init();
#else
	usart1_uart_deinit();
#endif


#if (CFG_WINKY_DEBUG != 0)
	/**
	 * Keep debugger enabled while in any low power mode
	 */
	HAL_DBGMCU_EnableDBGSleepMode();
	HAL_DBGMCU_EnableDBGStopMode();
	HAL_DBGMCU_EnableDBGStandbyMode();

	/***************** ENABLE DEBUGGER *************************************/
	LL_EXTI_EnableIT_32_63(LL_EXTI_LINE_48);
	LL_C2_EXTI_EnableIT_32_63(LL_EXTI_LINE_48);
#else
	//disable debugger pin for less consumption
	GPIO_InitTypeDef gpio_config = {0};

	gpio_config.Pull = GPIO_NOPULL;
	gpio_config.Mode = GPIO_MODE_ANALOG;

	gpio_config.Pin = GPIO_PIN_14 | GPIO_PIN_13;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	HAL_GPIO_Init(GPIOA, &gpio_config);

	HAL_DBGMCU_DisableDBGSleepMode();
	HAL_DBGMCU_DisableDBGStopMode();
	HAL_DBGMCU_DisableDBGStandbyMode();
#endif
}

