/*
 * ear.c
 *
 *  Created on: 3 oct. 2019
 *      Author: clement
 */


// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include "ear.h"
#include "main.h"
#include "peripheral_config.h"
#include "app_conf.h"
#include "app_common.h"
// --------------------------------------------------------------------------------------------------------------------
// ----- local constant macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------
osTimerId_t earPeriodTimer;
static WinkyEars ears;
// --------------------------------------------------------------------------------------------------------------------
// ----- local function prototypes
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local functions
// --------------------------------------------------------------------------------------------------------------------
static void earPWMPeriodCallback (void *argument) {
	//Change CUrrent ear;
	ears.currentEar = (ears.currentEar == RIGHT_EAR) ? LEFT_EAR : RIGHT_EAR;
	WinkyEar *ear;
	if (ears.currentEar == RIGHT_EAR)
	{
		ear = &ears.rightEar;
	}
	else
	{
		ear = &ears.leftEar;
	}
	if (ear->isActive)
	{
		if (ear->pulseCounter < 70)
		{
			htim16.Instance->ARR = 50+(ear->angle/2);

			HAL_GPIO_WritePin(ear->gpioPort, ear->gpioPin, GPIO_PIN_SET);

			__HAL_TIM_SET_COUNTER(&htim16, 0);
			HAL_TIM_Base_Start_IT(&htim16);
			ear->pulseCounter += 1;
		}
		else
		{
			ear->isActive = false;
		}
	}
}

void earPWMDCCallback() //TODO Delete this function
{
	APP_DBG_MSG("/!\ DEPRECATED /!\ Please use WinkyEar_PWMDCCallback instead\r\n");
	// Set GPIO Low
	if (ears.DCCounter >= 1)
	{
		if (ears.currentEar == RIGHT_EAR)
		{

			HAL_GPIO_WritePin(ears.rightEar.gpioPort, ears.rightEar.gpioPin, GPIO_PIN_RESET);
		}
		else
		{

			HAL_GPIO_WritePin(ears.leftEar.gpioPort, ears.leftEar.gpioPin, GPIO_PIN_RESET);
		}
		HAL_TIM_Base_Stop_IT(&htim16);
		ears.DCCounter = 0;
	}
	else
	{
		ears.DCCounter +=1;
	}
}
// --------------------------------------------------------------------------------------------------------------------
// ----- public functions
// --------------------------------------------------------------------------------------------------------------------


/*##-PWM- EAR Servomoteur #######################################
 * Ear Servomotor are controlled by a PWM. Unfortunately, tim1 which was dedicated to both encoder
 * and ear pwm on his 4 channels cannot be used as encoder mode and PWM at the same time.
 * There is only on other timer available with only one channel. Thus to generate two PWM, we used
 * on cmsis-os timer for the Period and tim16 for the DutyCycle (pulse). As we have two independent ear
 * the cmisi-os timer will have to be set to half of the period and will alternatively launch tim16 for
 *
 *
 *             __                                         __
 *            |  |                                       |  |
 * EAR1 ______|  |_______________________________________|  |_______________
 *
 *                                 ___                                        ___
 *                                |   |                                      |   |
 * EAR2 __________________________|   |______________________________________|   |_______________
 *
 *             __                  ___                    __                  ___
 *            |  |                |   |                  |  |                |   |
 * PWM  ______|  |________________|   |__________________|  |________________|   |_______________
 *             <------ 10ms -----><----------10ms------->
 *                CMSIS-OS TImer
 *                                <--->                   <->
 *                             Duty Cycle EAR 2          Duty Cycle EAR1
 *                                TIM16                   TIM16
 *
 *
 *  CMSIS-OS Timer: set to 10ms
 *  TIM16 : 800us (0°), 1500us (90°), 2300us (180°) => 1° ~= 7,7us => 128570 Hz.
 *  		TIM16 Prescaler = (uint32_t)((SystemCoreClock) / 128570) - 1; (theory)
 *  		TIM16 ARR (Period) = 50 + (degree/2) (from oscilloscope) (from theory it should be 104 + ~degree, there might be a factor 2 on the timer clock?!)
 *
 *
 *
 */


void WinkyEars_init()
{
	APP_DBG_MSG("WinkyEars_init");

	// TIM16 INIT
	/* Prescaler value declartion*/
	uint32_t uwPrescalerValue = 0;

	/*##1 Compute the prescaler value, to have TIM2Freq = 128570 Hz

	 * TIM2CLK = SystemCoreClock
	 *
	 * Prescaler = (TIM2CLK /TIM2 counter clock) - 1
	 *
	 * The prescaler value is computed in order to have TIM2 counter clock
	 * set at 128570 Hz. => 1/7.7us which is the 1°degree step for the servomotor
	 */

	uwPrescalerValue = (uint32_t)((SystemCoreClock) / 128570) - 1;


	htim16.Instance = TIM16;
	htim16.Init.Prescaler = uwPrescalerValue;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 50; // For 0° (theory said it should be 104)
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
	{
		ERROR_HANDLER();
	}


	memset(&ears, 0x0, sizeof(WinkyEars));

	ears.leftEar.isActive = false;
	ears.leftEar.gpioPort = EAR_L_GPIO_Port;
	ears.leftEar.gpioPin = EAR_L_Pin;
	ears.rightEar.isActive = false;
	ears.rightEar.gpioPort = EAR_R_GPIO_Port;
	ears.rightEar.gpioPin = EAR_R_Pin;
	ears.DCCounter = 0;

	earPeriodTimer = osTimerNew(earPWMPeriodCallback, osTimerPeriodic, NULL, NULL);
	osTimerStart(earPeriodTimer, 10U);
}

void WinkyEar_rotate(EarType side,uint8_t degree)
{
	WinkyEar *ear;
	if (side == RIGHT_EAR)
	{
		ear = &ears.rightEar;

	}
	else
	{
		ear = &ears.leftEar;
		degree = 180 - degree;
	}
	ear->angle = degree;
	ear->isActive = true;
	ear->pulseCounter = 0;
}

void WinkyEar_PWMDCCallback()
{
	// Set GPIO Low
	if (ears.DCCounter >= 1)
	{
		if (ears.currentEar == RIGHT_EAR)
		{

			HAL_GPIO_WritePin(ears.rightEar.gpioPort, ears.rightEar.gpioPin, GPIO_PIN_RESET);
		}
		else
		{

			HAL_GPIO_WritePin(ears.leftEar.gpioPort, ears.leftEar.gpioPin, GPIO_PIN_RESET);
		}
		HAL_TIM_Base_Stop_IT(&htim16);
		ears.DCCounter = 0;
	}
	else
	{
		ears.DCCounter +=1;
	}
}
