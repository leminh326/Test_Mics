/*
 * winky_pwr.c
 *
 *  Created on: 25 nov. 2019
 *      Author: clement
 */


// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include "winky_pwr.h"
#include "hw_conf.h"
#include "stm32wbxx_hal.h"
#include "main.h"
// --------------------------------------------------------------------------------------------------------------------
// ----- local constant macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------

bool exitingStopMode = false, enteringStopMode = false;
// --------------------------------------------------------------------------------------------------------------------
// ----- local function prototypes
// --------------------------------------------------------------------------------------------------------------------

void winky_pwr_enterStopMode(winkyPwrWakeupSrc wakeUpSource);
void winky_pwr_exitStopMode();
// --------------------------------------------------------------------------------------------------------------------
// ----- local functions
// --------------------------------------------------------------------------------------------------------------------
void winky_pwr_enterStopMode(winkyPwrWakeupSrc wakeUpSource)
{
	enteringStopMode = true;

	//TODO disable all IRQ


	//TODO Re-enable IRQ that correspond to wakeUpSource
	if ((wakeUpSource & IMUIrqWakeUp) == IMUIrqWakeUp)
	{
		// TODO
	}
	if ((wakeUpSource & GestureIrqWakeUp) == GestureIrqWakeUp)
	{
		// TODO
	}
	if ((wakeUpSource & BLEWakeUp) == BLEWakeUp)
	{
		// TODO
	}
	if ((wakeUpSource & RTCWakeUp) == RTCWakeUp)
	{
		// TODO
	}
	if ((wakeUpSource & OnOffButtonWakeUp) == OnOffButtonWakeUp)
	{
		// Disable Falling Edge Interrupt
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = RDY_IQS572_WAKEUP2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(RDY_IQS572_WAKEUP2_GPIO_Port, &GPIO_InitStruct);
	}
	  aci_gap_set_non_discoverable();

	  osDelay(200);

	  LL_C1_IPCC_DisableIT_RXO( IPCC );
	  LL_C1_IPCC_DisableIT_TXF( IPCC );


	  LL_EXTI_DisableRisingTrig_0_31(RTC_EXTI_LINE_WAKEUPTIMER_EVENT);
	  LL_EXTI_DisableIT_0_31(RTC_EXTI_LINE_WAKEUPTIMER_EVENT);

	  LL_EXTI_DisableIT_32_63(LL_EXTI_LINE_48);
	  LL_C2_EXTI_DisableIT_32_63(LL_EXTI_LINE_48);

	  HAL_NVIC_DisableIRQ(IPCC_C1_RX_IRQn);
	  HAL_NVIC_DisableIRQ(IPCC_C1_TX_IRQn);

	  LL_C2_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN); //TODO check if we want to wakeup from BLE

	  /* Check and Clear the Wakeup flag */
	  if (LL_PWR_IsActiveFlag_WU2() != 0)
	  {
	    LL_PWR_ClearFlag_WU2();
	  }

	  DeInit_Peripheral();

	  // /* Ensure that MSI is wake-up system clock */
	  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

	  PWR_PVDTypeDef pwr_confg;

	  pwr_confg.Mode      = PWR_PVD_MODE_IT_RISING;
	  pwr_confg.PVDLevel  = PWR_PVDLEVEL_7;

	  SysTick->CTRL  = 0; //Suspend systick Freertos
	  HAL_SuspendTick(); //Suspend Hal Tick  TIM17

	  HAL_PWR_ConfigPVD(&pwr_confg);

	  HAL_PWR_EnablePVD();

	  __HAL_PWR_PVD_EXTI_CLEAR_FLAG();

	  enteringStopMode = false;
	  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

}

void winky_pwr_exitStopMode()
{
	  exitingStopMode = true;
	  SystemClock_Config();

	  /**Configure the Systick interrupt time
	  */
	  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	  /**Configure the Systick
	  */
	  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	  HAL_ResumeTick();

	  Init_Peripheral();



	  LL_EXTI_EnableRisingTrig_0_31(RTC_EXTI_LINE_WAKEUPTIMER_EVENT);
	  LL_EXTI_EnableIT_0_31(RTC_EXTI_LINE_WAKEUPTIMER_EVENT);

	  HAL_NVIC_EnableIRQ(IPCC_C1_RX_IRQn);
	  HAL_NVIC_EnableIRQ(IPCC_C1_TX_IRQn);

	  LL_EXTI_EnableIT_32_63(LL_EXTI_LINE_48);
	  LL_C2_EXTI_EnableIT_32_63(LL_EXTI_LINE_48);

	  LL_C1_IPCC_EnableIT_RXO( IPCC );
	  LL_C1_IPCC_EnableIT_TXF( IPCC );

	  //ReStartADV();
	  LL_PWR_EnableBootC2();
}

// --------------------------------------------------------------------------------------------------------------------
// ----- public functions
// --------------------------------------------------------------------------------------------------------------------
bool winky_pwr_isEnteringStopMode()
{
	return enteringStopMode;
}

bool winky_pwr_isExitingStopMode()
{
	return exitingStopMode;
}

void winky_pwr_enterLowPower()
{
	//TODO add sleepMode for idleHookTask and Standby Mode for even less consumption
	winky_pwr_enterStopMode(OnOffButtonWakeUp | RTCWakeUp);
	winky_pwr_exitStopMode();

}

void winky_pwr_exitLowPower()
{
	exitingStopMode = false;
	ReStartADV();
}
