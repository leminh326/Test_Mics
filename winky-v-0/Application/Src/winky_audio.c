/**
******************************************************************************
* @file    cca02m2_audio.c
* @author  SRA - Central Labs
* @version v1.0.0
* @date    06-Dec-19
* @brief   This file provides the Audio driver for the cca02m2
*          board.
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "winky_audio.h"
#include "winky_audio_conf.h"
#include "winky_errno.h"

#include "hw_conf.h"

#ifndef USE_STM32L4XX_NUCLEO
#include "arm_math.h"
#endif

#define USE_STM32WBXX_NUCLEO

/*### RECORD ###*/

#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

#define DFSDM_OVER_SAMPLING(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (256U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (128U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (64U)  \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (32U) : (32U)

#define DFSDM_CLOCK_DIVIDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (17U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (16U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (16U) : (16U)

#define DFSDM_FILTER_ORDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (DFSDM_FILTER_SINC5_ORDER) \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (DFSDM_FILTER_SINC5_ORDER) : (DFSDM_FILTER_SINC5_ORDER)

#define DFSDM_MIC_BIT_SHIFT(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (8U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (5U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (8U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (8U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (10U) \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (5U) : (5U)

#ifdef USE_STM32WBXX_NUCLEO

#define SAI_DIVIDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (37U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (37U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (16U) \
        : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (16U) : (16U)

#endif

/**
  * @}
  */

/** @defgroup WINKY_AUDIO_Exported_Variables WINKY_AUDIO_ Exported Variables
  * @{
  */
/* Recording context */
AUDIO_IN_Ctx_t                         AudioInCtx[AUDIO_IN_INSTANCES_NBR] = {0};

/**
  * @}
  */

/** @defgroup WINKY_AUDIO_Private_Variables WINKY_AUDIO_ Private Variables
  * @{
  */

#define DECIMATOR_NUM_TAPS (16)
#define DECIMATOR_BLOCK_SIZE (16U*N_MS_PER_INTERRUPT)
#define DECIMATOR_FACTOR 2
#define DECIMATOR_STATE_LENGTH (DECIMATOR_BLOCK_SIZE + (DECIMATOR_NUM_TAPS) -1)

/* PDM filters params */
static PDM_Filter_Handler_t  PDM_FilterHandler[4];
static PDM_Filter_Config_t   PDM_FilterConfig[4];

SAI_HandleTypeDef            hAudioInSai;
DMA_HandleTypeDef 			hAudioInSaiDmaRx;
#define PDM_INTERNAL_BUFFER_SIZE_SAI          ((MAX_MIC_FREQ / 8) * MAX_AUDIO_IN_CHANNEL_NBR_TOTAL * N_MS_PER_INTERRUPT)
static uint16_t SAI_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_SAI];


/* Recording Buffer Trigger */
static __IO uint32_t    RecBuffTrigger          = 0;
static __IO uint32_t    RecBuffHalf             = 0;
static __IO uint32_t    MicBuffIndex[4];

/**
  * @}
  */

/** @defgroup WINKY_AUDIO_Private_Function_Prototypes WINKY_AUDIO_ Private Function Prototypes
  * @{
  */


static void SAI_MspInit(SAI_HandleTypeDef *hsai);



/**
  * @}
  */

/** @defgroup WINKY_AUDIO_IN_Exported_Functions WINKY_AUDIO_IN Exported Functions
  * @{
  */


/**
* @brief  Initialize wave recording.
* @param  Instance  AUDIO IN Instance. It can be:
*       - 0 when I2S is used
*       - 1 if DFSDM is used
*       - 2 if PDM is used
* @param  AudioInit Init structure
* @retval BSP status
*/
__weak int32_t WINKY_AUDIO_IN_Init(uint32_t Instance, WINKY_AUDIO_Init_t* AudioInit)
{
  int32_t ret =  BSP_ERROR_NONE;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Store the audio record context */
    AudioInCtx[Instance].Device          = AudioInit->Device;
    AudioInCtx[Instance].ChannelsNbr     = AudioInit->ChannelsNbr;
    AudioInCtx[Instance].SampleRate      = AudioInit->SampleRate;
    AudioInCtx[Instance].BitsPerSample   = AudioInit->BitsPerSample;
    AudioInCtx[Instance].Volume          = AudioInit->Volume;
    AudioInCtx[Instance].State           = AUDIO_IN_STATE_RESET;

    if(Instance == 0U)
    {
      uint32_t PDM_Clock_Freq;

      switch (AudioInit->SampleRate)
      {
      case AUDIO_FREQUENCY_8K:
        PDM_Clock_Freq = 1280;
        break;

      case AUDIO_FREQUENCY_16K:
        PDM_Clock_Freq = PDM_FREQ_16K;
        break;

      case AUDIO_FREQUENCY_32K:
        PDM_Clock_Freq = 2048;
        break;

      case AUDIO_FREQUENCY_48K:
        PDM_Clock_Freq = 3072;
        break;

      default:
        PDM_Clock_Freq = 1280;
        ret =  BSP_ERROR_WRONG_PARAM;
        break;
      }

      //AudioInCtx[Instance].DecimationFactor = (PDM_Clock_Freq * 1000U)/AudioInit->SampleRate;
      PDM_Clock_Freq = 384;
      AudioInCtx[Instance].DecimationFactor = 24;

      /* Double buffer for 1 microphone */
      AudioInCtx[Instance].Size = (PDM_Clock_Freq/8U) * 2U * N_MS_PER_INTERRUPT;

      if (AudioInCtx[Instance].ChannelsNbr == 1U)
      {
       AudioInCtx[Instance].Size *= 2U;
      }

      /* Initialize SAI */
      __HAL_SAI_RESET_HANDLE_STATE(&hAudioInSai);

      /* PLL clock is set depending by the AudioFreq */
      if(MX_SAI_ClockConfig(&hAudioInSai, PDM_Clock_Freq) != HAL_OK)
      {
        ret =  BSP_ERROR_CLOCK_FAILURE;
      }

      if(HAL_SAI_GetState(&hAudioInSai) == HAL_SAI_STATE_RESET)
      {
        SAI_MspInit(&hAudioInSai);
      }

      hAudioInSai.Instance = AUDIO_IN_SAI_INSTANCE;
      __HAL_SAI_DISABLE(&hAudioInSai);

      hAudioInSai.Init.Protocol = SAI_FREE_PROTOCOL;
      hAudioInSai.Init.AudioMode  = SAI_MODEMASTER_RX;
      hAudioInSai.Init.DataSize = SAI_DATASIZE_16;
      hAudioInSai.Init.FirstBit       = SAI_FIRSTBIT_MSB;
      hAudioInSai.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;
      hAudioInSai.Init.Synchro = SAI_ASYNCHRONOUS;
      hAudioInSai.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
      hAudioInSai.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
      hAudioInSai.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
      hAudioInSai.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
      hAudioInSai.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
      hAudioInSai.Init.MonoStereoMode = SAI_STEREOMODE;
      hAudioInSai.Init.CompandingMode = SAI_NOCOMPANDING;
      hAudioInSai.Init.PdmInit.Activation = ENABLE;
      hAudioInSai.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
      if ( AudioInCtx[Instance].ChannelsNbr <= 2U)
      {
    	  hAudioInSai.FrameInit.FrameLength       = 16;
    	  hAudioInSai.Init.PdmInit.MicPairsNbr = 1;
    	  hAudioInSai.Init.AudioFrequency = ((PDM_Clock_Freq * 1000U) / hAudioInSai.FrameInit.FrameLength ) * 2U;
      }
      else
      {
    	  hAudioInSai.FrameInit.FrameLength       = 16;
    	  hAudioInSai.Init.PdmInit.MicPairsNbr    = 2;
    	  hAudioInSai.Init.AudioFrequency = ((PDM_Clock_Freq * 1000U) / hAudioInSai.FrameInit.FrameLength ) * 4U;
      }

      hAudioInSai.FrameInit.ActiveFrameLength = 1;
      hAudioInSai.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
      hAudioInSai.FrameInit.FSPolarity        = SAI_FS_ACTIVE_HIGH;
      hAudioInSai.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
      hAudioInSai.SlotInit.FirstBitOffset = 0;
      hAudioInSai.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
      hAudioInSai.SlotInit.SlotNumber     = 1;
      hAudioInSai.SlotInit.SlotActive = 0x00000003;

      if (HAL_SAI_Init(&hAudioInSai) != HAL_OK)
      {
        ret =  BSP_ERROR_PERIPH_FAILURE;
      }

      /* Enable SAI to generate clock used by audio driver */
      __HAL_SAI_ENABLE(&hAudioInSai);

      if (WINKY_AUDIO_IN_PDMToPCM_Init(Instance, AudioInCtx[0].SampleRate, AudioInCtx[0].ChannelsNbr, AudioInCtx[0].ChannelsNbr)!= BSP_ERROR_NONE)
      {
        ret =  BSP_ERROR_NO_INIT;
      }
    }
    else if(Instance == 1U)
    {
      ret =  BSP_ERROR_WRONG_PARAM;
    }
    else /* Instance = 2 */
    {

    }

    /* Update BSP AUDIO IN state */
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP;
    /* Return BSP status */
  }
  return ret;
}

/**
* @brief  Deinit the audio IN peripherals.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/

__weak int32_t WINKY_AUDIO_IN_DeInit(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if(Instance == 0U)
    {

    }

    else /* (Instance == 1U) */
    {
      ret =  BSP_ERROR_WRONG_PARAM;
    }
    /* Update BSP AUDIO IN state */
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RESET;
  }
  /* Return BSP status */
  return ret;
}


/**
* @brief  Clock Config.
* @param  hSai: SAI handle if required
* @param  SampleRate: Audio frequency used to play the audio stream.
* @note   This API is called by WINKY_AUDIO_IN_Init()
*         Being __weak it can be overwritten by the application
* @retval HAL_OK if no problem during execution, HAL_ERROR otherwise
*/
__weak HAL_StatusTypeDef MX_SAI_ClockConfig(SAI_HandleTypeDef *hSai, uint32_t PDM_rate)
{
  UNUSED(hSai);

  HAL_StatusTypeDef ret = HAL_OK;
  /*SAI PLL Configuration*/
  RCC_PeriphCLKInitTypeDef rccclkinit;
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);

  if ((PDM_rate % 1280U) == 0U)
  {
    rccclkinit.PLLSAI1.PLLN = 82;
    rccclkinit.PLLSAI1.PLLP = RCC_PLLP_DIV8;
  }
  else
  {
    rccclkinit.PLLSAI1.PLLN = 86;
    rccclkinit.PLLSAI1.PLLP = RCC_PLLP_DIV7;
  }
//  rccclkinit.PLLSAI1.PLLQ 		= RCC_PLLQ_DIV2;
//  rccclkinit.PLLSAI1.PLLR 		= RCC_PLLR_DIV2;
//  rccclkinit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
//  rccclkinit.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
  rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_SAI1;

  if(HAL_RCCEx_PeriphCLKConfig(&rccclkinit) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  return ret;
}




/**
* @brief  Initialize the PDM library.
* @param Instance    AUDIO IN Instance
* @param  AudioFreq  Audio sampling frequency
* @param  ChnlNbrIn  Number of input audio channels in the PDM buffer
* @param  ChnlNbrOut Number of desired output audio channels in the  resulting PCM buffer
* @retval BSP status
*/
__weak int32_t WINKY_AUDIO_IN_PDMToPCM_Init(uint32_t Instance, uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut)
{
  int32_t ret =  BSP_ERROR_NONE;

  if(Instance != 0U)
  {
    ret =  BSP_ERROR_WRONG_PARAM;
  }
  else
  {

    uint32_t index;
    /* Enable CRC peripheral to unlock the PDM library */
    __HAL_RCC_CRC_CLK_ENABLE();

    for(index = 0; index < ChnlNbrIn; index++)
    {
      volatile uint32_t error = 0;
      /* Init PDM filters */
      PDM_FilterHandler[index].bit_order  = PDM_FILTER_BIT_ORDER_LSB;
      if (ChnlNbrIn == 1U)
      {
        PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_BE; /* For WB this should be LE, TODO after bugfix in PDMlib */
      }
      else
      {
        PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
      }
      PDM_FilterHandler[index].high_pass_tap = 2122358088;
      PDM_FilterHandler[index].out_ptr_channels = (uint16_t)ChnlNbrOut;
      PDM_FilterHandler[index].in_ptr_channels  = (uint16_t)ChnlNbrIn;

      /* PDM lib config phase */
      PDM_FilterConfig[index].output_samples_number = (uint16_t) ((AudioFreq/1000U) * N_MS_PER_INTERRUPT);
      PDM_FilterConfig[index].mic_gain = 24;

      switch (AudioInCtx[0].DecimationFactor)
      {
      case 16:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_16;
        break;
      case 24:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_24;
        break;
      case 32:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_32;
        break;
      case 48:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_48;
        break;
      case 64:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_64;
        break;
      case 80:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_80;
        break;
      case 128:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_128;
        break;
      default:
        ret =  BSP_ERROR_WRONG_PARAM;
        break;
      }

      error = PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));
      if (error!=0U)
      {
        ret =  BSP_ERROR_NO_INIT;
      }

      error = PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
      if (error!=0U)
      {
        ret =  BSP_ERROR_NO_INIT;
      }
    }

  }
  return ret;
}



/**
* @brief  Converts audio format from PDM to PCM.
* @param  Instance  AUDIO IN Instance
* @param  PDMBuf    Pointer to PDM buffer data
* @param  PCMBuf    Pointer to PCM buffer data
* @retval BSP status
*/
__weak int32_t WINKY_AUDIO_IN_PDMToPCM(uint32_t Instance, uint16_t *PDMBuf, uint16_t *PCMBuf)
{
  int32_t ret =  BSP_ERROR_NONE;

  if(Instance != 0U)
  {
    ret =  BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    uint32_t index;

    for(index = 0; index < AudioInCtx[Instance].ChannelsNbr; index++)
    {
        (void)PDM_Filter(&((uint8_t*)(PDMBuf))[index], (uint16_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);
    }
  }
  return ret;
}
/**
* @brief  Start audio recording.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  pbuf     Main buffer pointer for the recorded data storing
* @param  NbrOfBytes     Size of the record buffer. Parameter not used when Instance is 0
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_Record(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes)
{
  int32_t ret = BSP_ERROR_NONE;

  if(Instance >= (AUDIO_IN_INSTANCES_NBR - 1U) )
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    AudioInCtx[Instance].pBuff = (uint16_t*)pBuf;

    if(Instance == 0U)
    {
      UNUSED(NbrOfBytes);

      if(HAL_SAI_Receive_DMA(&hAudioInSai, (uint8_t *)SAI_InternalBuffer, (uint16_t)(AudioInCtx[Instance].Size/2U * AudioInCtx[Instance].ChannelsNbr)) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }


      /* Update BSP AUDIO IN state */
      AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;

    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Stop audio recording.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_Stop(uint32_t Instance)
{
  int32_t ret;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if(Instance == 0U)
    {
      ret = BSP_ERROR_NONE;

      if(HAL_SAI_DMAStop(&hAudioInSai) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }

    }
    else /*(Instance == 1U) */
    {
      ret =  BSP_ERROR_WRONG_PARAM;

    }
    /* Update BSP AUDIO IN state */
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP;
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Pause the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_Pause(uint32_t Instance)
{
  int32_t ret;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if(Instance == 0U)
    {
      ret = BSP_ERROR_NONE;

      if(HAL_SAI_DMAPause(&hAudioInSai)!= HAL_OK)
      {
        ret = BSP_ERROR_WRONG_PARAM;
      }
    }
    else /* (Instance == 1U) */
    {
      ret =  BSP_ERROR_WRONG_PARAM;

    }
    /* Update BSP AUDIO IN state */
    AudioInCtx[Instance].State = AUDIO_IN_STATE_PAUSE;
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Resume the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_Resume(uint32_t Instance)
{
  int32_t ret;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if(Instance == 0U)
    {
      ret = BSP_ERROR_NONE;

      if(HAL_SAI_DMAResume(&hAudioInSai)!= HAL_OK)
      {
        ret = BSP_ERROR_WRONG_PARAM;
      }
    }
    else /* (Instance == 1U) */
    {
      ret =  BSP_ERROR_WRONG_PARAM;
    }
    /* Update BSP AUDIO IN state */
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
  }
  /* Return BSP status */
  return ret;
}



/**
* @brief  Set Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Device    The audio input device to be used
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device)
{
  int32_t ret = BSP_ERROR_NONE;
  WINKY_AUDIO_Init_t audio_init;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    if(Instance == 1U)
    {
      ret =  BSP_ERROR_WRONG_PARAM;
    }
    audio_init.Device = Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;
    audio_init.SampleRate    = AudioInCtx[Instance].SampleRate;
    audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume;

    if(WINKY_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_NO_INIT;
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }

  /* Return BSP status */
  return ret;
}

/**
* @brief  Get Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Device    The audio input device used
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device)
{
  int32_t ret = BSP_ERROR_NONE;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Return audio Input Device */
    *Device = AudioInCtx[Instance].Device;
  }
  return ret;
}

/**
* @brief  Set Audio In frequency
* @param  Instance     Audio IN instance
* @param  SampleRate  Input frequency to be set
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t  SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  WINKY_AUDIO_Init_t audio_init;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    if(Instance == 1U)
    {
      ret =  BSP_ERROR_WRONG_PARAM;
    }
    audio_init.Device        = AudioInCtx[Instance].Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;
    audio_init.SampleRate    = SampleRate;
    audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume;
    if(WINKY_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_NO_INIT;
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }

  /* Return BSP status */
  return ret;
}

/**
* @brief  Get Audio In frequency
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  SampleRate  Audio Input frequency to be returned
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Return audio in frequency */
    *SampleRate = AudioInCtx[Instance].SampleRate;
  }

  /* Return BSP status */
  return ret;
}

/**
* @brief  Set Audio In Resolution
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  BitsPerSample  Input resolution to be set
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  WINKY_AUDIO_Init_t audio_init;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    if(Instance == 1U)
    {
      ret =  BSP_ERROR_WRONG_PARAM;
    }
    audio_init.Device        = AudioInCtx[Instance].Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;
    audio_init.SampleRate    = AudioInCtx[Instance].SampleRate;
    audio_init.BitsPerSample = BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume;
    if(WINKY_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_NO_INIT;
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }

  /* Return BSP status */
  return ret;
}

/**
* @brief  Get Audio In Resolution
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  BitsPerSample  Input resolution to be returned
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Return audio in resolution */
    *BitsPerSample = AudioInCtx[Instance].BitsPerSample;
  }
  return ret;
}

/**
* @brief  Set Audio In Channel number
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  ChannelNbr  Channel number to be used
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;

  if((Instance >= AUDIO_IN_INSTANCES_NBR) || (ChannelNbr > 2U))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Update AudioIn Context */
    AudioInCtx[Instance].ChannelsNbr = ChannelNbr;
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Get Audio In Channel number
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  ChannelNbr  Channel number to be used
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Channel number to be returned */
    *ChannelNbr = AudioInCtx[Instance].ChannelsNbr;
  }
  return ret;
}

/**
* @brief  Set the current audio in volume level.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Volume    Volume level to be returnd
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume)
{
  int32_t ret = BSP_ERROR_NONE;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (Instance == 0U)
  {
    uint32_t index;
    static int16_t VolumeGain[] =
    {
      -12,-12,-6,-3,0,2,3,5,6,7,8,9,9,10,11,11,12,12,13,13,14,14,15,15,15,
      16,16,17,17,17,17,18,18,18,19,19,19,19,19,20,20,20,20,21,21,21,21,21,
      22,22,22,22,22,22,23,23,23,23,23,23,23,24,24,24,24,24,24,24,25,25,25,
      25,25,25,25,25,25,26,26,26,26,26,26,26,26,26,27,27,27,27,27,27,27,27,
      27,27,28,28,28,28,28,28,28,28,28,28,28,28,29,29,29,29,29,29,29,29,29,
      29,29,29,29,30,30,30,30,30,30,30,31
    };
    for (index = 0; index < AudioInCtx[Instance].ChannelsNbr; index++)
    {
      if (PDM_FilterConfig[index].mic_gain != VolumeGain[Volume])
      {
        PDM_FilterConfig[index].mic_gain = VolumeGain[Volume];
        (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
      }
    }
  }
  else
  {
    /* Update AudioIn Context */
    AudioInCtx[Instance].Volume = Volume;
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Get the current audio in volume level.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Volume    Volume level to be returnd
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume)
{
  int32_t ret = BSP_ERROR_NONE;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Input Volume to be returned */
    *Volume = AudioInCtx[Instance].Volume;
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Get Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  State     Audio Out state
* @retval BSP status
*/
int32_t WINKY_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State)
{
  int32_t ret = BSP_ERROR_NONE;

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Input State to be returned */
    *State = AudioInCtx[Instance].State;
  }
  return ret;
}


/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into
byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is
written into the buffer that the user indicates when calling the WINKY_AUDIO_IN_Start(...) function.
* @param hSai: SAI handle. Not used
* @retval None
*/
extern UART_HandleTypeDef huart1;
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hSai)
{
    UNUSED(hSai);
  uint32_t index;
//  static uint8_t header[] = { 'a', 'b', 'c', 'd' };
//  HAL_UART_Transmit(&huart1, header, sizeof(header), 500);
//
//  HAL_UART_Transmit(&huart1, SAI_InternalBuffer, (uint16_t)(AudioInCtx[0].Size/2U * AudioInCtx[0].ChannelsNbr) / 2, 500);
  switch(AudioInCtx[0].ChannelsNbr){
  case 1:
    {
      uint8_t * DataTempSAI = &(((uint8_t *)SAI_InternalBuffer)[AudioInCtx[0].Size/2U]) ;
      for(index = 0; index < (AudioInCtx[0].Size/4U) ; index++)
      {
        ((uint8_t *)(AudioInCtx[0].pBuff))[index] = (DataTempSAI[2U*index]);
      }
      /* Remove after bugfix in PDMlib */
      for(index = 0; index < (AudioInCtx[0].Size/8U) ; index++)
      {
     	  ((uint16_t *)(AudioInCtx[0].pBuff))[index] = HTONS(((uint16_t *)(AudioInCtx[0].pBuff))[index]);
      }
      break;
    }
  case 2:
    {
      uint8_t * DataTempSAI = &(((uint8_t *)SAI_InternalBuffer)[AudioInCtx[0].Size]) ;
      for(index = 0; index < (AudioInCtx[0].Size) ; index++)
      {
        ((uint8_t *)(AudioInCtx[0].pBuff))[index] = (DataTempSAI[index]);
      }
      break;
    }
  case 4:
    {
      uint8_t * DataTempSAI = &(((uint8_t *)SAI_InternalBuffer)[AudioInCtx[0].Size  * 2U]) ;
      for(index = 0; index < (AudioInCtx[0].Size * 2U) ; index++)
      {
        ((uint8_t *)(AudioInCtx[0].pBuff))[index] = (DataTempSAI[index]);
      }

      break;
    }
  default:
    {

      break;
    }
  }

  WINKY_AUDIO_IN_TransferComplete_CallBack(0);
}

/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into
byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is
written into the buffer that the user indicates when calling the WINKY_AUDIO_IN_Start(...) function.
* @param hSai: SAI handle. Not used
* @retval None
*/
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hSai)
{
  UNUSED(hSai);
  uint32_t index;

  switch(AudioInCtx[0].ChannelsNbr){
  case 1:
    {
      uint8_t * DataTempSAI = (uint8_t *)SAI_InternalBuffer;
      for(index = 0; index < (AudioInCtx[0].Size/4U) ; index++)
      {
        ((uint8_t *)(AudioInCtx[0].pBuff))[index] = (DataTempSAI[2U*index]);
      }
      /* Remove after bugfix in PDMlib */
      for(index = 0; index < (AudioInCtx[0].Size/8U) ; index++)
           {
         	  ((uint16_t *)(AudioInCtx[0].pBuff))[index] = HTONS(((uint16_t *)(AudioInCtx[0].pBuff))[index]);
           }
      break;
    }
  case 2:
    {
        uint8_t * DataTempSAI = (uint8_t *)SAI_InternalBuffer;
      for(index = 0; index < (AudioInCtx[0].Size); index++)
      {
        ((uint8_t *)(AudioInCtx[0].pBuff))[index] = (DataTempSAI[index]);
      }

      break;
    }
  case 4:
    {
        uint8_t * DataTempSAI = (uint8_t *)SAI_InternalBuffer;
      for(index = 0; index < (AudioInCtx[0].Size * 2U); index++)
      {
        ((uint8_t *)(AudioInCtx[0].pBuff))[index] = (DataTempSAI[index]);
      }

      break;
    }
  default:
    {

      break;
    }

  }



  WINKY_AUDIO_IN_HalfTransfer_CallBack(0);
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hSai)
{
	printf("SAI Error\r\n");
	__asm("BKPT #0\n") ; // Break into the debugger

	while(1)
	{

	}
}

/**
* @brief  User callback when record buffer is filled.
* @retval None
*/
__weak void WINKY_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);

  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}

/**
* @brief  Manages the DMA Half Transfer complete event.
* @retval None
*/
__weak void WINKY_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);

  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}

/**
* @brief  Audio IN Error callback function.
* @retval None
*/
__weak void WINKY_AUDIO_IN_Error_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);

  /* This function is called when an Interrupt due to transfer error on or peripheral
  error occurs. */
}

/**
  * @}
  */

/** @defgroup WINKY_AUDIO_IN_Private_Functions WINKY_AUDIO_IN Private Functions
  * @{
  */

/*******************************************************************************
Static Functions
*******************************************************************************/

/**
* @brief AUDIO IN SAI MSP Init
* @param None
* @retval None
*/
 void SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the SAI peripheral clock */
  AUDIO_IN_SAI_CLK_ENABLE();

  /* Enable SAI GPIO clocks */
  AUDIO_IN_SAI_SCK_GPIO_CLK_ENABLE();
  AUDIO_IN_SAI_SD_GPIO_CLK_ENABLE();
  AUDIO_IN_SAI_SD2_GPIO_CLK_ENABLE();

  /* SAI pins configuration: SCK and SD pins ------------------------------*/
  GPIO_InitStructure.Pin       = AUDIO_IN_CLK_GPIO_Pin;
  GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull      = GPIO_NOPULL;
  GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Alternate = GPIO_AF3_SAI1;
  HAL_GPIO_Init(AUDIO_IN_CLK_GPIO_Port, &GPIO_InitStructure);

  GPIO_InitStructure.Pin       = AUDIO_IN_DATA1_GPIO_Pin;
  GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull      = GPIO_NOPULL;
  GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Alternate = GPIO_AF3_SAI1;
  HAL_GPIO_Init(AUDIO_IN_DATA1_GPIO_Port, &GPIO_InitStructure);

  GPIO_InitStructure.Pin       = AUDIO_IN_DATA2_GPIO_Pin;
  GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull      = GPIO_NOPULL;
  GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Alternate = GPIO_AF3_SAI1;
  HAL_GPIO_Init(AUDIO_IN_DATA2_GPIO_Port, &GPIO_InitStructure);

  /* Enable the DMA clock */
    /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();


    /* Configure the hAudioInSaiDmaRx handle parameters */
  hAudioInSaiDmaRx.Instance = DMA1_Channel3; /*tODO DEFINES */
  hAudioInSaiDmaRx.Init.Request             = DMA_REQUEST_SAI1_A;
  hAudioInSaiDmaRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hAudioInSaiDmaRx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hAudioInSaiDmaRx.Init.MemInc              = DMA_MINC_ENABLE;
  hAudioInSaiDmaRx.Init.PeriphDataAlignment = AUDIO_IN_SAI_DMAx_PERIPH_DATA_SIZE;
  hAudioInSaiDmaRx.Init.MemDataAlignment    = AUDIO_IN_SAI_DMAx_MEM_DATA_SIZE;
  hAudioInSaiDmaRx.Init.Mode                = DMA_CIRCULAR;
  hAudioInSaiDmaRx.Init.Priority            = DMA_PRIORITY_HIGH;




    /* Deinitialize the Stream for new transfer */
    (void)HAL_DMA_DeInit(&hAudioInSaiDmaRx);

    /* Configure the DMA Stream */
    if(HAL_DMA_Init(&hAudioInSaiDmaRx) != HAL_OK)
    {
    	printf("[AUDIO_CODEC] Error while enable SAI DMA \r\n");
    }
    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmarx, hAudioInSaiDmaRx);

  /* I2S DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, WINKY_AUDIO_IN_IT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}


// --------------------------------------------------------------------------------------------------------------------
// ----- local functions
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public functions
// --------------------------------------------------------------------------------------------------------------------