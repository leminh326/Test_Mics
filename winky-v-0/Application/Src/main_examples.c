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
#include <stdbool.h>

#include "peripheral_config.h"
#include "app_entry.h"
#include "otp.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "gatt_service.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define DEFAULT_HSE_TUNNING_CAPACITOR (20)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_DRIVER_CONTEXT
 */
PLACE_IN_SECTION("BLE_APP_CONTEXT") WINKYAPP_Context_t WINKYAPP_Context;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */

#if (IMU_EXAMPLE != 0)
osThreadId_t imuTaskHandle;
void StartIMUTask(void *argument);
#endif

#if (EAR_EXAMPLE != 0)
osThreadId_t earTaskHandle;
void StartEarTask(void *argument);
#endif

#if (BATTERY_EXAMPLE != 0)
	osThreadId_t batteryTaskHandle;
	void StartBatteryTask(void *argument);
#endif

#if ( HEAD_EXAMPLE != 0)
osThreadId_t headTaskHandle;
void StartHeadTask(void *argument);
#endif

#if ( GEST_EXAMPLE != 0)
osThreadId_t gestTaskHandle;
void StartGestTask(void *argument);
#endif

#if ( TOF_EXAMPLE != 0)
osThreadId_t tofTaskHandle;
void StartToFTask(void *argument);
#endif

#if (	MATRIX_EXAMPLE != 0)
osThreadId_t ledTaskHandle;
void StartLedTask(void *argument);
#endif

#if ( SOUND_EXAMPLE != 0)
osThreadId_t SoundTaskHandle;
void StartSoundTask(void *argument);
#endif

osThreadId_t bluetoothTaskHandle;
osThreadId_t loggerTaskHandle;
osThreadId_t myTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartBluetoothTask(void *argument);
void StartLoggerTask(void *argument);
void StartMyTask(void *argument);


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  LL_RCC_HSE_SetCapacitorTuning(DEFAULT_HSE_TUNNING_CAPACITOR);

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

  /* USER CODE BEGIN 2 */
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
    .stack_size = 128*4
  };
  bluetoothTaskHandle = osThreadNew(StartBluetoothTask, NULL, &bluetoothTask_attributes);

  /* definition and creation of loggerTask */
  const osThreadAttr_t loggerTask_attributes = {
    .name = "loggerTask",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 128*4
  };
  loggerTaskHandle = osThreadNew(StartLoggerTask, NULL, &loggerTask_attributes);

  /* definition and creation of myTask */
  const osThreadAttr_t myTask_attributes = {
    .name = "myTask",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 128
  };
  myTaskHandle = osThreadNew(StartMyTask, NULL, &myTask_attributes);

#if (IMU_EXAMPLE != 0)
  /* definition and creation of imuTask */
  const osThreadAttr_t imuTask_attributes = {
		  .name = "imuTask",
		  .priority = (osPriority_t) osPriorityLow,
		  .stack_size = 128*10
  };
  imuTaskHandle = osThreadNew(StartIMUTask, NULL, &imuTask_attributes);
#endif

#if (EAR_EXAMPLE != 0)
  /* definition and creation of earTask */
  const osThreadAttr_t earTask_attributes = {
		  .name = "earTask",
		  .priority = (osPriority_t) osPriorityLow,
		  .stack_size = 128*6
  };
  earTaskHandle = osThreadNew(StartEarTask, NULL, &earTask_attributes);
#endif

#if (BATTERY_EXAMPLE != 0)
  /* definition and creation of ledTask */
  const osThreadAttr_t batteryTask_attributes = {
		  .name = "batteryTask",
		  .priority = (osPriority_t) osPriorityLow,
		  .stack_size = 128*4
  };
  batteryTaskHandle = osThreadNew(StartBatteryTask, NULL, &batteryTask_attributes);
#endif

#if ( HEAD_EXAMPLE != 0)
  /* definition and creation of headTask */
  const osThreadAttr_t headTask_attributes = {
		  .name = "headTask",
		  .priority = (osPriority_t) osPriorityLow,
		  .stack_size = 128*4
  };
  headTaskHandle = osThreadNew(StartHeadTask, NULL, &headTask_attributes);
#endif

#if ( GEST_EXAMPLE != 0)
  /* definition and creation of gestTask */
  const osThreadAttr_t gestTask_attributes = {
		  .name = "gestTask",
		  .priority = (osPriority_t) osPriorityLow,
		  .stack_size = 128*4
  };
  gestTaskHandle = osThreadNew(StartGestTask, NULL, &gestTask_attributes);
#endif

#if ( TOF_EXAMPLE != 0)
  /* definition and creation of tofTask */
  const osThreadAttr_t tofTask_attributes = {
		  .name = "tofTask",
		  .priority = (osPriority_t) osPriorityLow,
		  .stack_size = 128*4
  };
  tofTaskHandle = osThreadNew(StartToFTask, NULL, &tofTask_attributes);
#endif

#if (	MATRIX_EXAMPLE != 0)
  /* definition and creation of ledTask */
  const osThreadAttr_t ledTask_attributes = {
		  .name = "ledTask",
		  .priority = (osPriority_t) osPriorityLow,
		  .stack_size = 128*4
  };
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);
#endif

#if ( SOUND_EXAMPLE != 0)
  /* definition and creation of ledTask */
  const osThreadAttr_t SoundTask_attributes = {
      .name = "SoundTask",
      .priority = (osPriority_t) osPriorityLow,
      .stack_size = 1024
  };
  SoundTaskHandle = osThreadNew(StartSoundTask, NULL, &SoundTask_attributes);
#endif

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
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  // RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV8;

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
	  printf("StartBluetoothTask\r\n");

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
	APP_DBG_MSG("StartLoggerTask started\n");
	/* Infinite loop */
	for(;;)
	{
		//APP_DBG_MSG("delay 1s\n");
		osDelay(1000);
	}
	/* USER CODE END StartLoggerTask */
}

/**
* @brief Function implementing the myTask thread.
* @param argument: Not used
* @retval None
*/
void StartMyTask(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
}

#if (IMU_EXAMPLE != 0)

static int32_t func_ok(void)
{
    return 0;
}

static int32_t get_tick(void)
{
    return HAL_GetTick();
}

static int32_t i2c_write_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
	return HAL_I2C_Mem_Write(&IMU_I2C,addr,reg,1,data,len,100);
}

static int32_t i2c_read_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
	return HAL_I2C_Mem_Read(&IMU_I2C, addr, reg, 1, data, len, 100);
}

void StartIMUTask(void *argument)
{
	WINKYAPP_Context.IMUNotification_Status = 0;
	//LOGLOGGER_LEVEL_INFO, "IMU TASK STARTED");
	RTC_DateTypeDef date;
	date.Date=7;
	date.Month=RTC_MONTH_AUGUST;
	date.Year=19;
	date.WeekDay=RTC_WEEKDAY_WEDNESDAY;
	RTC_TimeTypeDef time;
	time.Hours=16;
	time.Minutes=00;
	time.TimeFormat=RTC_HOURFORMAT12_PM;
    HAL_RTC_SetDate(&hrtc,&date,RTC_FORMAT_BIN);
	HAL_RTC_SetTime(&hrtc,&time,RTC_FORMAT_BIN);
	LSM6DSL_Object_t imu;
	LSM6DSL_IO_t  imu_io;
	uint8_t id;
	imu_io.BusType = LSM6DSL_I2C_BUS; //I2C
	imu_io.Address = IMU_I2C_ADDRESS<<1;
	imu_io.Init        = func_ok;
	imu_io.DeInit      = func_ok;
	imu_io.ReadReg     = i2c_read_reg;
	imu_io.WriteReg    = i2c_write_reg;
	imu_io.GetTick = get_tick;
	if (LSM6DSL_RegisterBusIO(&imu, &imu_io) != LSM6DSL_OK)
	{
		APP_DBG_MSG("LSM6DSL_RegisterBusIO failed");
	}else if (LSM6DSL_ReadID(&imu, &id) != LSM6DSL_OK)
	{
		APP_DBG_MSG("LSM6DSL_ReadID failed");
	}
	if (LSM6DSL_Init(&imu) != LSM6DSL_OK)
	{
		APP_DBG_MSG("LSM6DSL_ReadID failed");
	}
	APP_DBG_MSG("LSM6DSL init ok");

	LSM6DSL_ACC_Enable(&imu);

	LSM6DSL_ACC_Set_INT1_DRDY(&imu,1);
//	LOG(LOGGER_LEVEL_INFO, "LSM6DSL Enable INACTIVITY INT1: ..");
//	if ( LSM6DSL_ACC_Enable_Inactivity_Detection(&imu,LSM6DSL_INT1_PIN) != LSM6DSL_OK){
//		LOG(LOGGER_LEVEL_ERROR, "Enable INACTIVITY INT1 Failed");
//		Error_Handler();
//	}
//
//	LOG(LOGGER_LEVEL_INFO, "LSM6DSL Enable WKUP INT1: ..");
//	if ( LSM6DSL_ACC_Enable_Wake_Up_Detection(&imu,LSM6DSL_INT1_PIN) != LSM6DSL_OK){
//		LOG(LOGGER_LEVEL_ERROR, "Enable WKUP INT1 Failed");
//		Error_Handler();
//	}

	LSM6DSL_Axes_t acc;
	for(;;)
	{
		osDelay(1000);
		if(WINKYAPP_Context.IMUNotification_Status){
			LSM6DSL_ACC_GetAxes(&imu,&acc);
			WINKYAPP_Context.IMUChar=acc;
			APP_DBG_MSG("LSM6DSL x:%d, y:%d, z:%d", acc.x, acc.y, acc.z);
			tBleStatus ret =	WinkySensorWriteCharacteristic_Update(WINKY_NOTIFY_CHARACTERISTIC_UUID, WINKY_NOTIFY_CHARACTERISTIC_VALUE_LENGTH, (uint8_t *)(&WINKYAPP_Context.IMUChar));
			APP_DBG_MSG("Ret update char:%i", ret);
			HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc,&date,RTC_FORMAT_BIN);
			APP_DBG_MSG("Time h:%d, m:%d, s:%d", time.Hours, time.Minutes, time.Seconds);

		} else {
			APP_DBG_MSG("-- Winky APP IMU : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
		}

	}
}
#endif

#if (EAR_EXAMPLE != 0)
void StartEarTask(void *argument)
{
	APP_DBG_MSG("StartEarTask STARTED!!!\r\n");
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	WinkyEars_init();

	WinkyEar_rotate(LEFT_EAR,60);

	WinkyEar_rotate(LEFT_EAR,0);
	osDelay(3000);
	WinkyEar_rotate(LEFT_EAR,90);
	osDelay(3000);
	WinkyEar_rotate(LEFT_EAR,180);
	osDelay(3000);

	uint8_t leftAngle = 0;
	uint8_t rightAngle = 0;

	while (1) {
		osDelay(1000);

		leftAngle += 45;
		if (leftAngle > 180) {
			leftAngle = 0;
		}
		WinkyEar_rotate(LEFT_EAR,leftAngle);


	}
}

#endif

void tim16_elapsed()
{
#if (EAR_EXAMPLE != 0)
	earPWMDCCallback();
#endif
}

#if (HEAD_EXAMPLE != 0)
void StartHeadTask(void *argument)
{
	APP_DBG_MSG("StartHeadTask STARTED!!!\r\n");

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	MotorHead motorhead;
	MotorHead_init(&motorhead, true);

	bool forward = true;
	MotorHead_forward(&motorhead, 0.3f);
	osDelay(1000);
	MotorHead_backward(&motorhead, 0.8f);

	while (1) {
		osDelay(100);

//			  osDelay(1000);
//			  MotorHead_forward(&motorhead, 0.9f);
//			  osDelay(1000);
//			  MotorHead_forward(&motorhead, 0.5f);

			  int32_t encoder = MotorHead_getEncoderValue(&motorhead);
			  //APP_DBG_MSG("Encoder value : %i\r\n",encoder);
			  if ((encoder < -30) || (encoder > 30)) {
				  MotorHead_stop(&motorhead);
				  MotorHead_resetEncoderValue(&motorhead);
				  osDelay(3000);
				  forward = !forward;
				  if (forward) {
					  MotorHead_forward(&motorhead, 0.3f);
				  } else {
					  MotorHead_backward(&motorhead, 0.3f);
				  }
			  }


		}
}
#endif

#if (GEST_EXAMPLE != 0)
void StartGestTask(void *argument)
{
	apds9960_HandleTypedef apds9960Handle;
	uint8_t gesture = 0;
	uint8_t prox = 0;
	apds9960Handle.i2c = &hi2c1;
	apds9960Handle.addr = APDS9960_ADDRESS<<1;
	apds9960Init(&apds9960Handle);
	apds9960EnableProximity(&apds9960Handle,true);
	apds9960EnableGesture(&apds9960Handle,true);
	for(;;)
	{
		if(apds9960GestureValid(&apds9960Handle) == true ){
			gesture = apds9960ReadGesture(&apds9960Handle);
			HAL_UART_Transmit(&huart1, &gesture,1, 0xFFF);
		}
		prox = apds9960ReadProximity(&apds9960Handle);
		HAL_UART_Transmit(&huart1, &prox,1, 0xFFF);
		prox = -1;
		gesture = -1;
	    osDelay(400);
	}
}
#endif

#if (TOF_EXAMPLE != 0)
void StartToFTask(void *argument)
{
}
#endif

#if (MATRIX_EXAMPLE != 0)
void StartLedTask(void *argument)
{
	//LOLOGGER_LEVEL_INFO, "LED TASK SUCCESSFULLY INITIALIZED");

	ledMatrix_HandleTypedef ledMatrixRight, ledMatrixLeft;
	ledMatrixRight.addr = 0x70<<1;
	ledMatrixRight.i2c = &LEDMATRIX_I2C;
	ledMatrixInit(&ledMatrixRight);
	ledMatrixLeft.addr = 0x71<<1;
	ledMatrixLeft.i2c = &LEDMATRIX_I2C;
	ledMatrixInit(&ledMatrixLeft);


	ledMatrixPrintChar(&ledMatrixRight, 'O');
	ledMatrixPrintChar(&ledMatrixLeft, 'K');
	osDelay(2000);
	ledMatrixPrintChar(&ledMatrixRight, 'G');
	ledMatrixPrintChar(&ledMatrixLeft, 'O');
	osDelay(2000);

	for(;;)
	{
		ledMatrixClear(&ledMatrixLeft);
		ledMatrixDrawBitmap(&ledMatrixLeft,heart_01_left_bmp);
		ledMatrixRender(&ledMatrixLeft);
		ledMatrixClear(&ledMatrixRight);
		ledMatrixDrawBitmap(&ledMatrixRight,heart_01_right_bmp);
		ledMatrixRender(&ledMatrixRight);
		osDelay(100);
		ledMatrixClear(&ledMatrixLeft);
		ledMatrixDrawBitmap(&ledMatrixLeft,heart_02_left_bmp);
		ledMatrixRender(&ledMatrixLeft);
		ledMatrixClear(&ledMatrixRight);
		ledMatrixDrawBitmap(&ledMatrixRight,heart_02_right_bmp);
		ledMatrixRender(&ledMatrixRight);
		osDelay(100);
		ledMatrixClear(&ledMatrixLeft);
		ledMatrixDrawBitmap(&ledMatrixLeft,heart_03_left_bmp);
		ledMatrixRender(&ledMatrixLeft);
		ledMatrixClear(&ledMatrixRight);
		ledMatrixDrawBitmap(&ledMatrixRight,heart_03_right_bmp);
		ledMatrixRender(&ledMatrixRight);
		osDelay(100);
		ledMatrixClear(&ledMatrixLeft);
		ledMatrixDrawBitmap(&ledMatrixLeft,heart_04_left_bmp);
		ledMatrixRender(&ledMatrixLeft);
		ledMatrixClear(&ledMatrixRight);
		ledMatrixDrawBitmap(&ledMatrixRight,heart_04_right_bmp);
		ledMatrixRender(&ledMatrixRight);
		osDelay(100);
	}
}
#endif

#if (SOUND_EXAMPLE != 0)

#pragma pack(8)

typedef struct wav_header_t
{
  uint8_t   FileTypeBlocID[4];
  uint32_t  FileSize;
  char    FileFormatID[4];
  uint8_t   FormatBlocID[4];
  uint32_t  BlocSize;
  uint16_t  AudioFormat;
  uint16_t  NbrCanaux;
  uint32_t  Frequence;
  uint32_t  BytePerSec;
  uint16_t  BytePerBloc;
  uint16_t  BitsPerSample;
  uint32_t  DataBlocID;
  uint32_t  DataSize;
}wav_header_t;

#pragma pack(1)

#define PLAY_HEADER (sizeof(wav_header_t))
#define PLAY_BUFF_SIZE       4096
#define AUDIO_VOL           (0)

uint16_t                      PlayBuff[PLAY_BUFF_SIZE];
__IO int16_t                 UpdatePointer = -1;

extern const unsigned char audio_test_wav[]  ;
extern const unsigned int audio_test_wav_len ;


static void codec_player_cb(audio_codec_result_t audio_codec_result);

static void codec_player_cb(audio_codec_result_t audio_codec_result)
{
  switch(audio_codec_result)
  {
    case AUDIO_CODEC_RESULT_HALF_BUFFER:
      UpdatePointer = 0;
      break;
    case AUDIO_CODEC_RESULT_END_BUFFER:
      UpdatePointer = PLAY_BUFF_SIZE/2;
      break;
    case AUDIO_CODEC_RESULT_ERROR:
      APP_DBG_MSG("ERROR While playing audio Sound !");
      break;
    default :
      // Nothing TODO
      break;
  }
}


void StartSoundTask(void *argument)
{
  wav_header_t * wav_header = (wav_header_t *) audio_test_wav;

  APP_DBG_MSG("Audio File information, Sampling Frequency : %d, Number of Channel : %d, File Sisze : %d", wav_header->Frequency, wav_header->NbrCanaux, wav_header->FileSize);

  if(wav_header->NbrCanaux != AUDIO_STEREO)
  {
    APP_DBG_MSG("Audio File Should be in Stereo Mode !");
  }

  audio_codec_tlv300dac31_init(wav_header->Frequency);

  for(int i=0; i < PLAY_BUFF_SIZE; i+=2)
  {
    PlayBuff[i]=*((__IO uint16_t *)(audio_test_wav + PLAY_HEADER + i));
  }

  audio_codec_tlv300dac31_play(PlayBuff, PLAY_BUFF_SIZE, codec_player_cb);

  audio_codec_tlv300dac31_unmute();

  uint32_t PlaybackPosition   = PLAY_BUFF_SIZE + PLAY_HEADER;


  while(1)
  {
    /* Wait a callback event */
    while(UpdatePointer==-1)
    {
      // osDelay(1);
    }

    int position = UpdatePointer;
    UpdatePointer = -1;

    /* Upate the first or the second part of the buffer */
    for(int i = 0; i < PLAY_BUFF_SIZE/2; i++)
    {
      PlayBuff[i+position] = *(uint16_t *)(audio_test_wav + PlaybackPosition);
      PlaybackPosition+=2; 
    }

    /* check the end of the file */
    if((PlaybackPosition+PLAY_BUFF_SIZE/2) > audio_test_wav_len)
    {
      PlaybackPosition = PLAY_HEADER;
    }

    if(UpdatePointer != -1)
    {
      /* Buffer update time is too long compare to the data transfer time */
      ERROR_HANDLER();
    }
  }
}

#endif

#if (BATTERY_EXAMPLE != 0)

void StartBatteryTask(void *argument) {
	ledMatrix_HandleTypedef ledMatrixRight, ledMatrixLeft;
	ledMatrixRight.addr = 0x70<<1;
	ledMatrixRight.i2c = &LEDMATRIX_I2C;
	ledMatrixInit(&ledMatrixRight);
	ledMatrixLeft.addr = 0x71<<1;
	ledMatrixLeft.i2c = &LEDMATRIX_I2C;
	ledMatrixInit(&ledMatrixLeft);

	ledMatrixPrintChar(&ledMatrixRight, 'O');
	ledMatrixPrintChar(&ledMatrixLeft, 'K');

	bool blink = false;
	while(1) {
		osDelay(500);
		float voltage = Battery_getVBat();
		int unit = (int)voltage;
		int dec = (int)(voltage * 10) - (unit * 10);
		char left = '0' + unit;
		char right = '0' + dec;
		ledMatrixPrintChar(&ledMatrixRight, left);
		ledMatrixPrintChar(&ledMatrixLeft, right);
		blink = !blink;
		if (blink) {
			if (Battery_getPowerGood()) {
				ledMatrixDrawPixel(&ledMatrixRight, 0, 0);
				ledMatrixRender(&ledMatrixRight);
			}
			if (Battery_getChargeStatus()) {
				ledMatrixDrawPixel(&ledMatrixLeft, 0, 0);
				ledMatrixRender(&ledMatrixLeft);
			}
		}
	}

}

#endif

void Error_Handler(const char *file, const uint16_t line)
{
//    logger_block();
//    LOG(LOGGER_LEVEL_ERROR, "Error @ %s : %i", file, line);
    __asm("BKPT #0\n") ; // Break into the debugger

    HAL_NVIC_SystemReset();
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
