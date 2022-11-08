/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "peripheral_config.h"
#include "app_entry.h"
#include "otp.h"
#include "app_common.h"

#include "winky_touch.h"
#include "winky_pwr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_HSE_TUNNING_CAPACITOR (18)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

osThreadId_t bluetoothTaskHandle;
osThreadId_t loggerTaskHandle;
osThreadId_t myTaskHandle;
/* USER CODE BEGIN PV */
/**
 * START of Section BLE_DRIVER_CONTEXT
 */
PLACE_IN_SECTION("BLE_APP_CONTEXT") WINKYAPP_Context_t WINKYAPP_Context;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartBluetoothTask(void *argument);
void StartLoggerTask(void *argument);
void StartMyTask(void *argument);

/* USER CODE BEGIN PFP */
static void Config_HSE(void);
static void Reset_Device( void );
static void Reset_IPCC( void );
static void Reset_BackupDomain( void );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void wakeUpTimeoutCallback()
{
	printf("go back to sleep\r\n");
	winky_touch_enableLowPower();
	winky_pwr_enterLowPower();
}

osTimerId_t wakeUpTimeoutTimer;

void winky_touch_onOffButtonCallback(buttonEvent event)
{
	if (winky_pwr_isEnteringStopMode())
	{
		//Nothing //TODO ignore touch
		return;
	}
	if (winky_pwr_isExitingStopMode())
	{
		if (osTimerIsRunning(wakeUpTimeoutTimer))
		{
			osTimerStop(wakeUpTimeoutTimer);
		}

		if(event == longTap)
		{
			winky_touch_disableLowPower();
			winky_pwr_exitLowPower();
			return;
		}
		else if( event == released)
		{
			return;
		}
		else
		{
			wakeUpTimeoutTimer = osTimerNew(wakeUpTimeoutCallback, osTimerOnce, NULL, NULL);
			osTimerStart(wakeUpTimeoutTimer, 3* 1000U);
			return;
		}
	}

	switch (event)
	{
	case pressed:
		printf("winky OnOffButton pressed\r\n");
		break;
	case released:
		printf("winky OnOffButton released\r\n");
		break;
	case singleTap:
		printf("winky OnOffButton singleTap\r\n");
		break;
	case longTap:
		printf("winky OnOffButton longTap\r\n");
		winky_touch_enableLowPower();
		winky_pwr_enterLowPower();
		printf("WakeUpFromStopMode\r\n");
		break;
	case veryLongTap:
		printf("winky OnOffButton veryLongTap\r\n");
		break;
	case hold:
		printf("winky OnOffButton hold\r\n");
		break;
	case longHold:
		printf("winky OnOffButton longHold\r\n");
		break;
	case veryLongHold:
		printf("winky OnOffButton veryLongHold\r\n");
		break;
	default:
		break;
	}
}

void winky_touch_headTrackpadFrontCallback(uint8_t x, uint8_t y)
{
	if (winky_pwr_isEnteringStopMode())
	{
		//Nothing //TODO ignore
		return;
	}
	if (winky_pwr_isExitingStopMode())
	{
		if (osTimerIsRunning(wakeUpTimeoutTimer))
		{
			osTimerStop(wakeUpTimeoutTimer);
		}
		//Nothing //TODO ignore
		wakeUpTimeoutTimer = osTimerNew(wakeUpTimeoutCallback, osTimerOnce, NULL, NULL);
		osTimerStart(wakeUpTimeoutTimer, 2* 1000U);
		return;
	}
	printf("winky head front trackpad %u, %u\r\n", x,y);
	//osThreadFlagsSet(validationTaskHandle,TOUCH_FRONT_SLIDER);
}
void winky_touch_headTrackpadBackCallback(uint8_t x, uint8_t y)
{
	if (winky_pwr_isEnteringStopMode())
	{
		//Nothing //TODO ignore
		return;
	}
	if (winky_pwr_isExitingStopMode())
	{
		if (osTimerIsRunning(wakeUpTimeoutTimer))
		{
			osTimerStop(wakeUpTimeoutTimer);
		}
		//Nothing //TODO ignore
		wakeUpTimeoutTimer = osTimerNew(wakeUpTimeoutCallback, osTimerOnce, NULL, NULL);
		osTimerStart(wakeUpTimeoutTimer, 2* 1000U);
		return;
	}
	printf("winky head back trackpad %u, %u\r\n", x,y);

	//osThreadFlagsSet(validationTaskHandle,TOUCH_BACK_SLIDER);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	LL_RCC_HSE_SetCapacitorTuning(DEFAULT_HSE_TUNNING_CAPACITOR);
	Reset_Device();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	gpio_init();
	dma_init();
	adc1_init();
	i2c_init(&hi2c1);
	i2c_init(&hi2c3);
	rf_init();
	rtc_init();
	sai1_init();
	spi1_init();
	tim1_init();
	tim2_init();
	//tim16_init();

	DbgTraceInit();
	printf("\n\n\n\n ====================== Hello Winky =================\r\n\n");

  /* USER CODE END 2 */

  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of loggerQueue */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of bluetoothTask */
  const osThreadAttr_t bluetoothTask_attributes = {
    .name = "bluetoothTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128*3
  };
  bluetoothTaskHandle = osThreadNew(StartBluetoothTask, NULL, &bluetoothTask_attributes);

  /* definition and creation of loggerTask */
  const osThreadAttr_t loggerTask_attributes = {
    .name = "loggerTask",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 128*3
  };
  loggerTaskHandle = osThreadNew(StartLoggerTask, NULL, &loggerTask_attributes);

  /* definition and creation of myTask */
  const osThreadAttr_t myTask_attributes = {
    .name = "myTask",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 128*3
  };
  myTaskHandle = osThreadNew(StartMyTask, NULL, &myTask_attributes);

	interruptFlagId = osEventFlagsNew(NULL);
	if (interruptFlagId == NULL) {
		ERROR_HANDLER();
	}

  winky_touch_init(headTouch);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Init code for STM32_WPAN */
  APPE_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Macro to configure the PLL multiplication factor
	 */
	__HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV4);
	/** Macro to configure the PLL clock source
	 */
	__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
	/** Configure LSE Drive Capability
	 */
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);
	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
			|RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		ERROR_HANDLER();
	}
	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
			|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		ERROR_HANDLER();
	}
	/** Initializes the peripherals clocks
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP
			|RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
			|RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_I2C1
			|RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_ADC;
	PeriphClkInitStruct.PLLSAI1.PLLN = 24;
	PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV17;
	PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
	PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV3;
	PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_ADCCLK;
	PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
	PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
	PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		ERROR_HANDLER();
	}
}


/* USER CODE BEGIN 4 */
static void Config_HSE(void)
{
    OTP_ID0_t * p_otp;

  /**
   * Read HSE_Tuning from OTP
   */
  p_otp = (OTP_ID0_t *) OTP_Read(0);
  if (p_otp)
  {
    LL_RCC_HSE_SetCapacitorTuning(p_otp->hse_tuning);
  }

  return;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBluetoothTask */
/**
  * @brief  Function implementing the bluetoothTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartBluetoothTask */
void StartBluetoothTask(void *argument)
{
    
    
    
    
    

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartLoggerTask */
/**
* @brief Function implementing the loggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLoggerTask */
void StartLoggerTask(void *argument)
{
  /* USER CODE BEGIN StartLoggerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartLoggerTask */
}

/* USER CODE BEGIN Header_StartMyTask */
/**
* @brief Function implementing the myTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMyTask */
void StartMyTask(void *argument)
{
  /* USER CODE BEGIN StartMyTask */
	printf("MyTask\r\n");
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
    printf("MyTask\r\n");
  }
  /* USER CODE END StartMyTask */
}

void tim16_elapsed()
{
//	Ear2_update(&leftEar);
//	Ear2_update(&rightEar);
}

static void Reset_Device( void )
{
#if ( CFG_HW_RESET_BY_FW == 1 )
	Reset_BackupDomain();

	Reset_IPCC();
#endif

	return;
}

static void Reset_IPCC( void )
{
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_IPCC);

	LL_C1_IPCC_ClearFlag_CHx(
			IPCC,
			LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
			| LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

	LL_C2_IPCC_ClearFlag_CHx(
			IPCC,
			LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
			| LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

	LL_C1_IPCC_DisableTransmitChannel(
			IPCC,
			LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
			| LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

	LL_C2_IPCC_DisableTransmitChannel(
			IPCC,
			LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
			| LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

	LL_C1_IPCC_DisableReceiveChannel(
			IPCC,
			LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
			| LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

	LL_C2_IPCC_DisableReceiveChannel(
			IPCC,
			LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
			| LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

	return;
}

static void Reset_BackupDomain( void )
{
	if ((LL_RCC_IsActiveFlag_PINRST() != FALSE) && (LL_RCC_IsActiveFlag_SFTRST() == FALSE))
	{
		HAL_PWR_EnableBkUpAccess(); /**< Enable access to the RTC registers */

		/**
		 *  Write twice the value to flush the APB-AHB bridge
		 *  This bit shall be written in the register before writing the next one
		 */
		HAL_PWR_EnableBkUpAccess();

		__HAL_RCC_BACKUPRESET_FORCE();
		__HAL_RCC_BACKUPRESET_RELEASE();
	}

	return;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t now;

	switch (GPIO_Pin)
	{
	case RDY_IQS572_WAKEUP2_Pin :
		 //printf("on est wakeup\r\n");

		if (HAL_GPIO_ReadPin(RDY_IQS572_WAKEUP2_GPIO_Port, RDY_IQS572_WAKEUP2_Pin) == GPIO_PIN_SET)
		{
			osEventFlagsSet(interruptFlagId,FLAGS_MSK_IQS5XX_RDY);

		}
		else
		{
			osEventFlagsClear(interruptFlagId,FLAGS_MSK_IQS5XX_RDY);
		}
		break;
	default:
		break;

	}
}

void Error_Handler(const char *file, const uint16_t line)
{
//    logger_block();
//    LOG(LOGGER_LEVEL_ERROR, "Error @ %s : %i", file, line);
    __asm("BKPT #0\n") ; // Break into the debugger

    HAL_NVIC_SystemReset();
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    signed char *pcTaskName )
{
	ERROR_HANDLER();
    __asm("BKPT #0\n") ; // Break into the debugger
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
