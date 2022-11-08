/*
 * peripheral_config.c
 *
 *  Created on: 9 août 2019
 *      Author: clement
 */


// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include "main.h"
#include "peripheral_config.h"
#include "stm32wbxx_hal.h"

// --------------------------------------------------------------------------------------------------------------------
// ----- local constant macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;


UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
// --------------------------------------------------------------------------------------------------------------------
// ----- local function prototypes
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local functions
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public functions
// --------------------------------------------------------------------------------------------------------------------
/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void adc1_init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    ERROR_HANDLER();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    ERROR_HANDLER();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void i2c_init(I2C_HandleTypeDef* i2c)
{
	if( i2c == &hi2c3)
	{
		if(HAL_I2C_GetState(&hi2c3) == HAL_I2C_STATE_RESET)
		{
			hi2c3.Instance = I2C3;
			hi2c3.Init.Timing = 0x00707CBB;
			hi2c3.Init.OwnAddress1 = 0;
			hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
			hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
			hi2c3.Init.OwnAddress2 = 0;
			hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
			hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
			hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
			if (HAL_I2C_Init(&hi2c3) != HAL_OK)
			{
				ERROR_HANDLER();
			}
			/** Configure Analogue filter
			 */
			if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
			{
				ERROR_HANDLER();
			}
			/** Configure Digital filter
			 */
			if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
			{
				ERROR_HANDLER();
			}
		}
	}
  else if( i2c == &hi2c1)
  {
    if(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_RESET)
    {
      hi2c1.Instance = I2C1;
      hi2c1.Init.Timing = 0x2000090E;
      hi2c1.Init.OwnAddress1 = 0;
      hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
      hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
      hi2c1.Init.OwnAddress2 = 0;
      hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
      hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
      hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
      if (HAL_I2C_Init(&hi2c1) != HAL_OK)
      {
        ERROR_HANDLER();
      }
      /** Configure Analogue filter
       */
      if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
      {
        ERROR_HANDLER();
      }
      /** Configure Digital filter
       */
      if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
      {
        ERROR_HANDLER();
      }

    }
  }

}


/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
void rf_init(void)
{

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
void rtc_init(void)
{

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    ERROR_HANDLER();
  }

  HAL_NVIC_EnableIRQ(CFG_HW_TS_RTC_WAKEUP_HANDLER_ID);
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
void sai1_init(void)
{
//  hsai_BlockA1.Instance = SAI1_Block_A;
//  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
//  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_RX;
//  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_8;
//  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
//  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
//  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
//  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
//  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
//  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
//  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
//  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
//  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
//  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
//  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
//  hsai_BlockA1.Init.PdmInit.Activation = ENABLE;
//  hsai_BlockA1.Init.PdmInit.MicPairsNbr = 2;
//  hsai_BlockA1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
//  hsai_BlockA1.FrameInit.FrameLength = 8;
//  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
//  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
//  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
//  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
//  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
//  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
//  hsai_BlockA1.SlotInit.SlotNumber = 1;
//  hsai_BlockA1.SlotInit.SlotActive = 0x00000000;
//  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
//  {
//    ERROR_HANDLER();
//  }
  // hsai_BlockB1.Instance = SAI1_Block_B;
  // hsai_BlockB1.Init.AudioMode = SAI_MODEMASTER_TX;
  // hsai_BlockB1.Init.Synchro = SAI_ASYNCHRONOUS;
  // hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  // hsai_BlockB1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  // hsai_BlockB1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  // hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  // hsai_BlockB1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  // hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  // hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  // hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  // hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  // if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  // {
  //   ERROR_HANDLER();
  // }

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void spi1_init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    ERROR_HANDLER();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
void tim1_init(void)
{

	  /* USER CODE BEGIN TIM1_Init 0 */

	  /* USER CODE END TIM1_Init 0 */

	  TIM_Encoder_InitTypeDef sConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM1_Init 1 */

	  /* USER CODE END TIM1_Init 1 */
	  htim1.Instance = TIM1;
	  htim1.Init.Prescaler = 64;
	  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim1.Init.Period = 1000;
	  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim1.Init.RepetitionCounter = 0;
	  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	  sConfig.IC1Filter = 0;
	  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	  sConfig.IC2Filter = 0;
	  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
	  {
	    ERROR_HANDLER();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	  {
	    ERROR_HANDLER();
	  }
	  /* USER CODE BEGIN TIM1_Init 2 */

	  /* USER CODE END TIM1_Init 2 */
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void tim2_init(void)
{


  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    ERROR_HANDLER();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    ERROR_HANDLER();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    ERROR_HANDLER();
  }
  sConfigOC.Pulse = 50;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    ERROR_HANDLER();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
void tim16_init(void)
{
}

/**
  * Enable DMA controller clock
  */
void dma_init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void gpio_init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOE_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOC, ToF_xSHUT_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, FLASH_CS_Pin | EEPROM_CS_Pin, GPIO_PIN_SET);
	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, PH_M_Pin|nSLEEP_M_Pin|ISET_CHANGE_Pin|EN_5VBO_Pin
	                          |EN_1V8DIG_Pin|EAR_L_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(EN_3V0ANA_GPIO_Port, EN_3V0ANA_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, EAR_R_Pin, GPIO_PIN_RESET); //|RST_IQS572_AUDIO_GPIO1_Pin

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(AUDIO_RST_GPIO_Port, AUDIO_RST_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin : RDY_IQS572_WAKEUP2_Pin */
	  GPIO_InitStruct.Pin = RDY_IQS572_WAKEUP2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(RDY_IQS572_WAKEUP2_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : BOOT0_Pin */
	  GPIO_InitStruct.Pin = BOOT0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(BOOT0_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : RDY_I2C3_Pin */
	  GPIO_InitStruct.Pin = RDY_I2C3_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(RDY_I2C3_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : ToF_IRQ_WAKEUP4_Pin */
	  GPIO_InitStruct.Pin = ToF_IRQ_WAKEUP4_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(ToF_IRQ_WAKEUP4_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : FLASH_CS_Pin ToF_xSHUT_Pin EEPROM_CS_Pin */
	  GPIO_InitStruct.Pin = FLASH_CS_Pin|ToF_xSHUT_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = EEPROM_CS_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : Gesture_IRQ_WAKEUP5_Pin POS_EN_Pin IMU_IRQ_WAKEUP3_Pin */
	  GPIO_InitStruct.Pin = Gesture_IRQ_WAKEUP5_Pin|POS_EN_Pin|IMU_IRQ_WAKEUP3_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : PH_M_Pin nSLEEP_M_Pin ISET_CHANGE_Pin EN_5VBO_Pin
	                           EN_1V8DIG_Pin EAR_L_Pin */
	  GPIO_InitStruct.Pin = PH_M_Pin|nSLEEP_M_Pin|ISET_CHANGE_Pin|EN_5VBO_Pin
	                          |EN_1V8DIG_Pin|EAR_L_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin : EN_3V0ANA_Pin */
	  GPIO_InitStruct.Pin = EN_3V0ANA_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(EN_3V0ANA_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : POWER_GOOD_IRQ_Pin */
	  GPIO_InitStruct.Pin = POWER_GOOD_IRQ_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(POWER_GOOD_IRQ_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : EAR_R_Pin RST_IQS572_AUDIO_GPIO1_Pin */
	  GPIO_InitStruct.Pin = EAR_R_Pin|RST_IQS572_AUDIO_GPIO1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : LBO_5VBOOST_Pin */
	  GPIO_InitStruct.Pin = LBO_5VBOOST_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(LBO_5VBOOST_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : AUDIO_RST_Pin */
	  GPIO_InitStruct.Pin = AUDIO_RST_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(AUDIO_RST_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : CHARGE_STATUS_IRQ_Pin */
	  GPIO_InitStruct.Pin = CHARGE_STATUS_IRQ_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(CHARGE_STATUS_IRQ_GPIO_Port, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void usart1_uart_init(void){

	  /* USER CODE BEGIN USART1_Init 0 */

	  /* USER CODE END USART1_Init 0 */

	  /* USER CODE BEGIN USART1_Init 1 */

	  /* USER CODE END USART1_Init 1 */
	  huart1.Instance = USART1;
	  huart1.Init.BaudRate = 2000000;
	  huart1.Init.WordLength = UART_WORDLENGTH_8B;
	  huart1.Init.StopBits = UART_STOPBITS_1;
	  huart1.Init.Parity = UART_PARITY_NONE;
	  huart1.Init.Mode = UART_MODE_TX_RX;
	  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  if (HAL_UART_Init(&huart1) != HAL_OK)
	  {
	    ERROR_HANDLER();
	  }
	  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	  {
	    ERROR_HANDLER();
	  }
	  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	  {
	    ERROR_HANDLER();
	  }
	  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
	  {
	    ERROR_HANDLER();
	  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

}


void gpio_deinit(void)
{

}
void dma_deinit(void)
{
  __HAL_RCC_DMAMUX1_CLK_DISABLE();
  __HAL_RCC_DMA1_CLK_DISABLE();
  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
}
void adc1_deinit(void)
{
  HAL_ADC_DeInit(&hadc1);
}
void i2c_deinit(void)
{
  HAL_I2C_DeInit(&hi2c3);
  //HAL_I2C_DeInit(&hi2c1);
}
void rf_deinit(void)
{

}
void rtc_deinit(void)
{
  HAL_RTC_DeInit(&hrtc);
  HAL_NVIC_DisableIRQ(CFG_HW_TS_RTC_WAKEUP_HANDLER_ID);
}
void sai1_deinit(void)
{
  HAL_SAI_DeInit(&hsai_BlockA1);
}
void spi1_deinit(void)
{
  HAL_SPI_DeInit(&hspi1);
}
void tim1_deinit(void)
{
  HAL_TIM_Base_DeInit(&htim1);
}
void tim2_deinit(void)
{
  HAL_TIM_Base_DeInit(&htim2);
}
void tim16_deinit(void)
{
  HAL_TIM_Base_DeInit(&htim16);
}
void usart1_uart_deinit(void)
{
  HAL_UART_DeInit(&huart1);
}

void Init_Peripheral (void)
{
  gpio_init();
  dma_init();
  adc1_init();
  i2c_init(&hi2c3);
  rf_init();
  rtc_init();
  sai1_init();
  spi1_init();
  tim1_init();
  tim2_init();
  tim16_init(); // TODO: call HAL_TIM_Base_Start_IT(&htim16); so that tim16_elapsed() is called and can call Ear2_update() inside.
  usart1_uart_init();
}

void DeInit_Peripheral (void)
{
  gpio_deinit();
  dma_deinit();
  adc1_deinit();
  i2c_deinit();
  rf_deinit();
  rtc_deinit();
  sai1_deinit();
  spi1_deinit();
  tim1_deinit();
  tim2_deinit();
  tim16_deinit();
}
