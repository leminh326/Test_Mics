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

uint16_t PDM_Buffer[((((AUDIO_IN_CHANNELS * AUDIO_IN_SAMPLING_FREQUENCY) / 1000) * MAX_DECIMATION_FACTOR) / 16)* N_MS_PER_INTERRUPT ];
uint16_t PCM_Buffer[((AUDIO_IN_CHANNELS*AUDIO_IN_SAMPLING_FREQUENCY)/1000)  * N_MS_PER_INTERRUPT ];
WINKY_AUDIO_Init_t MicParams;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

void AudioProcess(void);
void WINKY_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance);
void WINKY_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance);

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	gpio_init();
	dma_init();

	HAL_UART_MspInit2(&huart1);

	winky_debug_init();

	MicParams.BitsPerSample = 16;
	MicParams.ChannelsNbr = AUDIO_IN_CHANNELS;
	MicParams.Device = AUDIO_IN_DIGITAL_MIC;
	MicParams.SampleRate = AUDIO_IN_SAMPLING_FREQUENCY;
	MicParams.Volume = AUDIO_VOLUME_INPUT;

	WINKY_AUDIO_IN_Init(WINKY_AUDIO_INSTANCE, &MicParams);
	WINKY_AUDIO_IN_Record(WINKY_AUDIO_INSTANCE, (uint8_t *) PDM_Buffer, AUDIO_IN_BUFFER_SIZE);

	/* Infinite loop */
	while (1)
	{

	}

}

void AudioProcess(void)
{
	WINKY_AUDIO_IN_PDMToPCM(WINKY_AUDIO_INSTANCE,(uint16_t * )PDM_Buffer,PCM_Buffer);

	static uint8_t counter = 0;
	static uint8_t header[] = { 'a', 'b', 'c', 'd', 0 };
	header[4] = counter++;
	static uint16_t aPCMBufferOUT[AUDIO_IN_SAMPLING_FREQUENCY/1000];
	uint32_t sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0;
    for(int i = 0; i < AUDIO_IN_SAMPLING_FREQUENCY / 1000; i++)
    {
        aPCMBufferOUT[i] = PCM_Buffer[AUDIO_IN_CHANNELS * i];
        sum1 +=aPCMBufferOUT[i];
    }
	HAL_UART_Transmit(&huart1, header, sizeof(header), 500);
	HAL_UART_Transmit(&huart1, aPCMBufferOUT, (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t), 500);
    //HAL_UART_Transmit_DMA(&huart1, header, sizeof(header));
    //HAL_UART_Transmit_DMA(&huart1, aPCMBufferOUT, (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t));

    for(int i = 0; i < AUDIO_IN_SAMPLING_FREQUENCY / 1000; i++)
    {
        aPCMBufferOUT[i] = PCM_Buffer[AUDIO_IN_CHANNELS * i + 1];
        sum2 +=aPCMBufferOUT[i];
    }

    HAL_UART_Transmit(&huart1, aPCMBufferOUT, (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t), 500);
    //HAL_UART_Transmit_DMA(&huart1, aPCMBufferOUT, (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t));

    for(int i = 0; i < AUDIO_IN_SAMPLING_FREQUENCY / 1000; i++)
    {
        aPCMBufferOUT[i] = PCM_Buffer[AUDIO_IN_CHANNELS * i +2];
        sum3 +=aPCMBufferOUT[i];
    }
    HAL_UART_Transmit(&huart1, aPCMBufferOUT, (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t), 500);
    //HAL_UART_Transmit_DMA(&huart1, aPCMBufferOUT, (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t));

    for(int i = 0; i < AUDIO_IN_SAMPLING_FREQUENCY / 1000; i++)
    {
        aPCMBufferOUT[i] = PCM_Buffer[AUDIO_IN_CHANNELS * i + 3];
        sum4 +=aPCMBufferOUT[i];
    }
//    HAL_UART_Transmit(&huart1, aPCMBufferOUT, (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t), 500);
    //HAL_UART_Transmit_DMA(&huart1, aPCMBufferOUT, (AUDIO_IN_SAMPLING_FREQUENCY/1000) * N_MS_PER_INTERRUPT * sizeof(int16_t));

}

void WINKY_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  AudioProcess();
}

void WINKY_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  AudioProcess();
}

void HAL_UART_MspInit2(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel1;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {

    }
    __HAL_LINKDMA(huart,hdmarx,hdma_usart1_rx);
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel2;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
    }
    __HAL_LINKDMA(huart,hdmatx,hdma_usart1_tx);
    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */
  /* USER CODE END USART1_MspInit 1 */
  }
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Macro to configure the PLL multiplication factor
	 */
	__HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV5);
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
// AUDIOTODO
	//	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV8;
//	RCC_OscInitStruct.PLL.PLLN = 32;
//	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
//	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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
			|RCC_PERIPHCLK_I2C1| RCC_PERIPHCLK_SAI1
			|RCC_PERIPHCLK_I2C1
			|RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_ADC;
	PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
	// AUDIOTODO
//	PeriphClkInitStruct.PLLSAI1.PLLN = 86;
//	PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV7;
//	PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
//	PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
	PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
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

void Error_Handler(const char *file, const uint16_t line)
{
	__asm("BKPT #0\n") ; // Break into the debugger

	HAL_NVIC_SystemReset();
}
