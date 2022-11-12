#include "winky_audio.h"
#include "winky_audio_conf.h"
#include "winky_errno.h"
#include "hw_conf.h"

#ifndef USE_STM32L4XX_NUCLEO
#include "arm_math.h"
#endif

/* Recording context */
AUDIO_IN_Ctx_t                         AudioIn;

/* PDM filters params */
static PDM_Filter_Handler_t  PDM_FilterHandler[4];
static PDM_Filter_Config_t   PDM_FilterConfig[4];

extern UART_HandleTypeDef huart1;
extern SAI_HandleTypeDef  hsai_BlockA1;
extern PDM_Filter_Handler_t m_PDM_filter;

#define PDM_INTERNAL_BUFFER_SIZE_SAI          ((MAX_MIC_FREQ / 8) * MAX_AUDIO_IN_CHANNEL_NBR_TOTAL * N_MS_PER_INTERRUPT)
static uint16_t SAI_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_SAI];



__weak int32_t WINKY_AUDIO_IN_Init(WINKY_AUDIO_Init_t* AudioInit)
{
  AudioIn.ChannelsNbr     = AudioInit->ChannelsNbr;

  uint32_t PDM_Clock_Freq;

  PDM_Clock_Freq = 384;
  AudioIn.DecimationFactor = 24;

  /* Double buffer for 1 microphone */
  AudioIn.Size = (PDM_Clock_Freq/8U) * 2U * N_MS_PER_INTERRUPT;

  WINKY_AUDIO_IN_PDMToPCM_Init(16000, AudioIn.ChannelsNbr, AudioIn.ChannelsNbr);

  int32_t ret =  BSP_ERROR_NONE;
  return ret;
}

__weak int32_t WINKY_AUDIO_IN_PDMToPCM_Init(uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut)
{
  int32_t ret =  BSP_ERROR_NONE;

  uint32_t index;
  /* Enable CRC peripheral to unlock the PDM library */
  __HAL_RCC_CRC_CLK_ENABLE();

  for(index = 0; index < ChnlNbrIn; index++)
  {
    volatile uint32_t error = 0;
    /* Init PDM filters */
    PDM_FilterHandler[index].bit_order  = PDM_FILTER_BIT_ORDER_LSB;
    PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
    PDM_FilterHandler[index].high_pass_tap = 2122358088;
    PDM_FilterHandler[index].out_ptr_channels = (uint16_t)ChnlNbrOut;
    PDM_FilterHandler[index].in_ptr_channels  = (uint16_t)ChnlNbrIn;

    /* PDM lib config phase */
    PDM_FilterConfig[index].output_samples_number = (uint16_t) ((AudioFreq/1000U) * N_MS_PER_INTERRUPT);
    PDM_FilterConfig[index].mic_gain = 24;
    PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_24;

    PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));
    PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
  }

  return ret;
}

__weak int32_t WINKY_AUDIO_IN_PDMToPCM(uint16_t *PDMBuf, uint16_t *PCMBuf)
{
  int32_t ret =  BSP_ERROR_NONE;

  uint32_t index;

  for(index = 0; index < AudioIn.ChannelsNbr; index++)
  {
      (void)PDM_Filter(&((uint8_t*)(PDMBuf))[index], (uint16_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);
  }

  return ret;
}

int32_t WINKY_AUDIO_IN_Record(uint8_t* pBuf, uint32_t NbrOfBytes)
{
  int32_t ret = BSP_ERROR_NONE;

  AudioIn.pBuff = (uint16_t*)pBuf;
  HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *)SAI_InternalBuffer, (uint16_t)(AudioIn.Size/2U * AudioIn.ChannelsNbr));

  /* Return BSP status */
  return ret;
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hSai)
{
	UNUSED(hSai);
	uint32_t index;

	uint8_t * DataTempSAI = &(((uint8_t *)SAI_InternalBuffer)[AudioIn.Size  * 2U]) ;
	for(index = 0; index < (AudioIn.Size * 2U) ; index++)
	{
		((uint8_t *)(AudioIn.pBuff))[index] = (DataTempSAI[index]);
	}

	WINKY_AUDIO_IN_TransferComplete_CallBack(0);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hSai)
{
	UNUSED(hSai);
	uint32_t index;

	uint8_t * DataTempSAI = (uint8_t *)SAI_InternalBuffer;
	for(index = 0; index < (AudioIn.Size * 2U); index++)
	{
		((uint8_t *)(AudioIn.pBuff))[index] = (DataTempSAI[index]);
	}
	WINKY_AUDIO_IN_HalfTransfer_CallBack(0);
}

__weak void WINKY_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  UNUSED(Instance);
}

__weak void WINKY_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  UNUSED(Instance);
}
