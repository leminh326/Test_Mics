#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include "peripheral_config.h"
#include "app_entry.h"
#include "otp.h"
#include "app_common.h"
#include "gatt_service.h"

#include "winky_audio.h"

void SystemClock_Config(void);
void HAL_UART_MspInit2(UART_HandleTypeDef* huart);

/*Number of millisecond of audio at each DMA interrupt*/
#define N_MS_PER_INTERRUPT               (1U)
#define AUDIO_IN_CHANNELS 				 4
#define AUDIO_IN_SAMPLING_FREQUENCY 	 16000
#define MAX_DECIMATION_FACTOR 			 128

#define AUDIO_IN_BUFFER_SIZE             DEFAULT_AUDIO_IN_BUFFER_SIZE
#define AUDIO_VOLUME_INPUT               64U
#define WINKY_AUDIO_INSTANCE 			 0U
#define WINKY_AUDIO_IN_IT_PRIORITY    	 5U

uint16_t PDM_Buffer[((((AUDIO_IN_CHANNELS * AUDIO_IN_SAMPLING_FREQUENCY) / 1000) * MAX_DECIMATION_FACTOR) / 16)* N_MS_PER_INTERRUPT ];
uint16_t PCM_Buffer[((AUDIO_IN_CHANNELS*AUDIO_IN_SAMPLING_FREQUENCY)/1000)  * N_MS_PER_INTERRUPT ];
WINKY_AUDIO_Init_t MicParams;

SAI_HandleTypeDef  hsai_BlockA1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef  hdma_usart1_tx;
DMA_HandleTypeDef  hdma_usart1_rx;
DMA_HandleTypeDef  hdma_sai1_a;

PDM_Filter_Handler_t m_PDM_filter;
PDM_Filter_Config_t m_PDM_Filter_Config;

void AudioProcess(void);
void WINKY_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance);
void WINKY_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance);

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void Audio_to_UART_Blocking_Mode();
static void Audio_to_UART_DMA_Mode();
static void MX_SAI1_Init(void);
uint32_t sai_clk_feq = 0;
int main(void)
{
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_SAI1_Init();

    m_PDM_filter.bit_order           = PDM_FILTER_BIT_ORDER_LSB;
    m_PDM_filter.endianness       = PDM_FILTER_ENDIANNESS_LE;
    m_PDM_filter.high_pass_tap       = 2122358088;
    m_PDM_filter.in_ptr_channels  = 4;
    m_PDM_filter.out_ptr_channels = 4;
    PDM_Filter_Init(&m_PDM_filter);

    m_PDM_Filter_Config.decimation_factor       = PDM_FILTER_DEC_FACTOR_24;
    m_PDM_Filter_Config.output_samples_number = 16;
    m_PDM_Filter_Config.mic_gain              = 24;
    PDM_Filter_setConfig(&m_PDM_filter, &m_PDM_Filter_Config);

	MicParams.BitsPerSample = 16;
	MicParams.ChannelsNbr = AUDIO_IN_CHANNELS;
	MicParams.Device = AUDIO_IN_DIGITAL_MIC;
	MicParams.SampleRate = AUDIO_IN_SAMPLING_FREQUENCY;
	MicParams.Volume = AUDIO_VOLUME_INPUT;

	WINKY_AUDIO_IN_Init(&MicParams);
	WINKY_AUDIO_IN_Record((uint8_t *) PDM_Buffer, AUDIO_IN_BUFFER_SIZE);

	sai_clk_feq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SAI1);

	/* Infinite loop */
	while (1)
	{

	}
}

void AudioProcess(void)
{
	WINKY_AUDIO_IN_PDMToPCM((uint16_t * )PDM_Buffer,PCM_Buffer);
	Audio_to_UART_Blocking_Mode();
}

static void MX_SAI1_Init(void)
{

	/* USER CODE BEGIN SAI1_Init 0 */

	/* USER CODE END SAI1_Init 0 */

	/* USER CODE BEGIN SAI1_Init 1 */

	/* USER CODE END SAI1_Init 1 */
	hsai_BlockA1.Instance = SAI1_Block_A;
	hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
	hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_RX;
	hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
	hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
	hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
	hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
	hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockA1.Init.PdmInit.Activation = ENABLE;
	hsai_BlockA1.Init.PdmInit.MicPairsNbr = 2;
	hsai_BlockA1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
	hsai_BlockA1.FrameInit.FrameLength = 16;
	hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
	hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
	hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
	hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
	hsai_BlockA1.SlotInit.FirstBitOffset = 0;
	hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
	hsai_BlockA1.SlotInit.SlotNumber = 1;
	hsai_BlockA1.SlotInit.SlotActive = 0x0000FFFF;

	hsai_BlockA1.Init.AudioFrequency = 96000;
	hsai_BlockA1.Init.Mckdiv 		 = 0;

	if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SAI1_Init 2 */
	__HAL_SAI_ENABLE(&hsai_BlockA1);
	/* USER CODE END SAI1_Init 2 */

}

static void Audio_to_UART_Blocking_Mode()
{
	// *************** Blocking Mode *************** //
	static uint8_t counter = 0;
	static uint8_t header[] = { 'a', 'b', 'c', 0 };
	header[3] = counter++;
	static uint16_t aPCMBufferOUT[AUDIO_IN_SAMPLING_FREQUENCY/1000];
	uint32_t sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0;

    for(int i = 0; i < AUDIO_IN_SAMPLING_FREQUENCY / 1000; i++)
    {
        aPCMBufferOUT[i] = PCM_Buffer[AUDIO_IN_CHANNELS * i];
        sum1 +=aPCMBufferOUT[i];
    }
	HAL_UART_Transmit(&huart1, header, sizeof(header), 500);
	HAL_UART_Transmit(&huart1, aPCMBufferOUT, (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t), 500);

    for(int i = 0; i < AUDIO_IN_SAMPLING_FREQUENCY / 1000; i++)
    {
        aPCMBufferOUT[i] = PCM_Buffer[AUDIO_IN_CHANNELS * i + 1];
        sum2 +=aPCMBufferOUT[i];
    }

    HAL_UART_Transmit(&huart1, aPCMBufferOUT, (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t), 500);

    for(int i = 0; i < AUDIO_IN_SAMPLING_FREQUENCY / 1000; i++)
    {
        aPCMBufferOUT[i] = PCM_Buffer[AUDIO_IN_CHANNELS * i +2];
        sum3 +=aPCMBufferOUT[i];
    }
    HAL_UART_Transmit(&huart1, aPCMBufferOUT, (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t), 500);

    for(int i = 0; i < AUDIO_IN_SAMPLING_FREQUENCY / 1000; i++)
    {
        aPCMBufferOUT[i] = PCM_Buffer[AUDIO_IN_CHANNELS * i + 3];
        sum4 +=aPCMBufferOUT[i];
    }
//    HAL_UART_Transmit(&huart1, aPCMBufferOUT, (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t), 500);
}

static void Audio_to_UART_DMA_Mode()
{
	// *************** DMA Mode *************** //
	static uint8_t counter = 0;
	static uint8_t header[] = { 'a', 'b', 'c', 0 };
	header[3] = counter++;
	static uint16_t aPCMBufferOUT[AUDIO_IN_SAMPLING_FREQUENCY/1000 * AUDIO_IN_CHANNELS];

	for(int i = 0; i < AUDIO_IN_SAMPLING_FREQUENCY / 1000; i++)
	{
		aPCMBufferOUT[i] = PCM_Buffer[AUDIO_IN_CHANNELS * i];
		aPCMBufferOUT[i+16] = PCM_Buffer[AUDIO_IN_CHANNELS * i + 1];
		aPCMBufferOUT[i+32] = PCM_Buffer[AUDIO_IN_CHANNELS * i + 2];
		aPCMBufferOUT[i+48] = PCM_Buffer[AUDIO_IN_CHANNELS * i + 3];
	}
	uint16_t BufferOUT [66];
	memcpy(BufferOUT, header, 4 * sizeof(uint8_t));
	memcpy(BufferOUT + 2, aPCMBufferOUT, 64 * sizeof(uint16_t));

	HAL_UART_Transmit(&huart1, BufferOUT, 2 + AUDIO_IN_CHANNELS * (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t),100);
//	HAL_UART_Transmit_DMA(&huart1, BufferOUT, 2 + AUDIO_IN_CHANNELS * (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t));
}




void WINKY_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance) 	 { AudioProcess(); }
void WINKY_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance) { AudioProcess(); }

static void MX_USART1_UART_Init(void)
{

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
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}
static void MX_GPIO_Init(void)
{
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOE_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
}
void SystemClock_Config(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	  /** Macro to configure the PLL multiplication factor
	  */
	  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV5);
//	  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV8);


	  /** Macro to configure the PLL clock source
	  */
	  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
	  /** Configure the main internal regulator output voltage
	  */
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
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
	    Error_Handler();
	  }
	  /** Initializes the peripherals clocks
	  */
	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_USART1
	                              |RCC_PERIPHCLK_SAI1;
	  PeriphClkInitStruct.PLLSAI1.PLLN = 86;
	  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV7;
	  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
	  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
	  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
	  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
	  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
	  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN Smps */

	  /* USER CODE END Smps */
}
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}
