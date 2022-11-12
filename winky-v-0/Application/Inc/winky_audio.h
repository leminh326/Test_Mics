/**
******************************************************************************
* @file    cca02m2_audio.h
* @author  SRA - Central Labs
* @version v1.0.0
* @date    06-Dec-19
* @brief   This file contains the common defines and functions prototypes for
*          the cca02m2_audio.c driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef WINKY_AUDIO_H
#define WINKY_AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "winky_audio_conf.h"
#include <stdlib.h>

/* Include PDM to PCM lib header file */
#include "pdm2pcm_glo.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup CCA02M2
  * @{
  */

/** @defgroup WINKY_AUDIO_ CCA02M2 AUDIO
  * @{
  */

/** @defgroup WINKY_AUDIO_Exported_Variables WINKY_AUDIO_ Exported Variables
  * @{
  */

extern SAI_HandleTypeDef            hAudioInSai;

#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)
#define HTONS(A)  ((((A) & (uint16_t)0xff00) >> 8) | (((A) & (uint16_t)0x00ff) << 8))

/**
 * @}
 */

/** @defgroup WINKY_AUDIO_Exported_Types WINKY_AUDIO_ Exported Types
  * @{
  */

typedef struct {
	int32_t Z;
	int32_t oldOut;
	int32_t oldIn;
}HP_FilterState_TypeDef;

typedef struct
{
  uint32_t                    Device;
  uint32_t                    SampleRate;
  uint32_t                    BitsPerSample;
  uint32_t                    ChannelsNbr;
  uint32_t                    Volume;
}WINKY_AUDIO_Init_t;

typedef struct
{
  uint32_t                    Instance;            /* Audio IN instance              */
  uint32_t                    Device;              /* Audio IN device to be used     */
  uint32_t                    SampleRate;          /* Audio IN Sample rate           */
  uint32_t                    BitsPerSample;       /* Audio IN Sample resolution     */
  uint32_t                    ChannelsNbr;         /* Audio IN number of channel     */
  uint16_t                    *pBuff;              /* Audio IN record buffer         */
  uint8_t                     **pMultiBuff;        /* Audio IN multi-buffer          */
  uint32_t                    Size;                /* Audio IN record buffer size    */
  uint32_t                    Volume;              /* Audio IN volume                */
  uint32_t                    State;               /* Audio IN State                 */
  uint32_t                    IsMultiBuff;         /* Audio IN multi-buffer usage    */
  uint32_t                    IsMspCallbacksValid; /* Is Msp Callbacks registred     */
  HP_FilterState_TypeDef 	  HP_Filters[4];       /*!< HP filter state for each channel*/
  uint32_t DecimationFactor;
}AUDIO_IN_Ctx_t;

typedef struct
{
  uint32_t Mode;
  uint32_t Standard;
  uint32_t DataFormat;
  uint32_t MCLKOutput;
  uint32_t AudioFreq;
  uint32_t CPOL;
  uint32_t ClockSource;
  uint32_t FullDuplexMode;
}MX_I2S_IN_Config;

typedef struct
{
  uint32_t Mode;
  uint32_t Direction;
  uint32_t DataSize;
  uint32_t CLKPolarity;
  uint32_t CLKPhase;
  uint32_t NSS;
  uint32_t BaudRatePrescaler;
  uint32_t FirstBit;
  uint32_t TIMode;
  uint32_t CRCCalculation;
  uint32_t CRCPolynomial;
}MX_SPI_Config;


/**
  * @}
  */

/** @defgroup WINKY_AUDIO_Exported_Constants WINKY_AUDIO_ Exported Constants
  * @{
  */

/* AUDIO FREQUENCY */
#ifndef AUDIO_FREQUENCY_192K
#define AUDIO_FREQUENCY_192K     (uint32_t)192000U
#endif
#ifndef AUDIO_FREQUENCY_176K
#define AUDIO_FREQUENCY_176K     (uint32_t)176400U
#endif
#ifndef AUDIO_FREQUENCY_96K
#define AUDIO_FREQUENCY_96K       (uint32_t)96000U
#endif
#ifndef AUDIO_FREQUENCY_88K
#define AUDIO_FREQUENCY_88K       (uint32_t)88200U
#endif
#ifndef AUDIO_FREQUENCY_48K
#define AUDIO_FREQUENCY_48K       (uint32_t)48000U
#endif
#ifndef AUDIO_FREQUENCY_44K
#define AUDIO_FREQUENCY_44K       (uint32_t)44100U
#endif
#ifndef AUDIO_FREQUENCY_32K
#define AUDIO_FREQUENCY_32K       (uint32_t)32000U
#endif
#ifndef AUDIO_FREQUENCY_22K
#define AUDIO_FREQUENCY_22K       (uint32_t)22050U
#endif
#ifndef AUDIO_FREQUENCY_16K
#define AUDIO_FREQUENCY_16K       (uint32_t)16000U
#endif
#ifndef AUDIO_FREQUENCY_11K
#define AUDIO_FREQUENCY_11K       (uint32_t)11025U
#endif
#ifndef AUDIO_FREQUENCY_8K
#define AUDIO_FREQUENCY_8K         (uint32_t)8000U
#endif


/* AUDIO RESOLUTION */
#ifndef AUDIO_RESOLUTION_16b
#define AUDIO_RESOLUTION_16b                16U
#endif
#ifndef AUDIO_RESOLUTION_24b
#define AUDIO_RESOLUTION_24b                24U
#endif
#ifndef AUDIO_RESOLUTION_32b
#define AUDIO_RESOLUTION_32b                32U
#endif




/*------------------------------------------------------------------------------
                          USER I2S / SPI defines parameters
 -----------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
                        AUDIO IN defines parameters
------------------------------------------------------------------------------*/

  /* I2S Configuration defines */
#define AUDIO_IN_SAI_INSTANCE                                    SAI1_Block_A
#define AUDIO_IN_SAI_CLK_ENABLE()                                __SAI1_CLK_ENABLE()
#define AUDIO_IN_SAI_SCK_PIN                                     GPIO_PIN_3
#define AUDIO_IN_SAI_SCK_GPIO_PORT                               GPIOA
#define AUDIO_IN_SAI_SCK_GPIO_CLK_ENABLE()                       __GPIOA_CLK_ENABLE()
#define AUDIO_IN_SAI_SCK_AF                                      GPIO_AF3_SAI1
#define AUDIO_IN_SAI_SD_PIN                                    GPIO_PIN_3
#define AUDIO_IN_SAI_SD_GPIO_PORT                              GPIOC
#define AUDIO_IN_SAI_SD_GPIO_CLK_ENABLE()                      __GPIOC_CLK_ENABLE();
#define AUDIO_IN_SAI_SD_AF                                     GPIO_AF3_SAI1
#define AUDIO_IN_SAI_SD2_PIN                                    GPIO_PIN_9
#define AUDIO_IN_SAI_SD2_GPIO_PORT                              GPIOB
#define AUDIO_IN_SAI_SD2_GPIO_CLK_ENABLE()                      __GPIOB_CLK_ENABLE();
#define AUDIO_IN_SAI_SD2_AF                                     GPIO_AF3_SAI1

  /* I2S DMA definitions */
#define AUDIO_IN_SAI_DMAx_CLK_ENABLE()                          __DMA1_CLK_ENABLE()
#define AUDIO_IN_SAI_DMAx_STREAM                                DMA1_Stream3
#define AUDIO_IN_SAI_DMAx_CHANNEL                               DMA_CHANNEL_0
#define AUDIO_IN_SAI_DMAx_IRQ                                   DMA1_Stream3_IRQn
#define AUDIO_IN_SAI_DMAx_PERIPH_DATA_SIZE                      DMA_PDATAALIGN_HALFWORD
#define AUDIO_IN_SAI_DMAx_MEM_DATA_SIZE                         DMA_MDATAALIGN_HALFWORD
#define AUDIO_IN_SAI_IRQHandler                                 DMA1_Stream3_IRQHandler


/* AUDIO TIMER definitions */
#define AUDIO_IN_TIMER                                     TIM3
#define AUDIO_IN_TIMER_CLK_ENABLE()                        __TIM3_CLK_ENABLE()
#define AUDIO_IN_TIMER_CHOUT_AF                            GPIO_AF2_TIM3
#define AUDIO_IN_TIMER_CHOUT_PIN                           GPIO_PIN_5
#define AUDIO_IN_TIMER_CHOUT_GPIO_PORT                     GPIOB
#define AUDIO_IN_TIMER_CHOUT_GPIO_PORT_CLK_ENABLE()        __GPIOB_CLK_ENABLE()
#define AUDIO_IN_TIMER_CHIN_AF                             GPIO_AF2_TIM3
#define AUDIO_IN_TIMER_CHIN_PIN                            GPIO_PIN_4
#define AUDIO_IN_TIMER_CHIN_GPIO_PORT                      GPIOB
#define AUDIO_IN_TIMER_CHIN_GPIO_PORT_CLK_ENABLE()         __GPIOB_CLK_ENABLE()

/* Audio In devices */
#ifndef AUDIO_IN_CHANNELS
#define AUDIO_IN_CHANNELS 1
#endif

#ifndef AUDIO_IN_SAMPLING_FREQUENCY
#define AUDIO_IN_SAMPLING_FREQUENCY 16000
#endif

#ifndef AUDIO_VOLUME_INPUT
#define AUDIO_VOLUME_INPUT              64U
#endif

#ifndef WINKY_AUDIO_INSTANCE
#define WINKY_AUDIO_INSTANCE 0U
#endif

#ifndef WINKY_AUDIO_IN_IT_PRIORITY
#define WINKY_AUDIO_IN_IT_PRIORITY    6U
#endif

/* MP34DT01TR digital microphone on PCB top side */
#define AUDIO_IN_DIGITAL_MIC1      0x10U
#define AUDIO_IN_DIGITAL_MIC2      0x20U
#define AUDIO_IN_DIGITAL_MIC3      0x40U
#define AUDIO_IN_DIGITAL_MIC4      0x80U
#define AUDIO_IN_DIGITAL_MIC_LAST  AUDIO_IN_DIGITAL_MIC4
#define AUDIO_IN_DIGITAL_MIC       (AUDIO_IN_DIGITAL_MIC1 | AUDIO_IN_DIGITAL_MIC2 | AUDIO_IN_DIGITAL_MIC3 | AUDIO_IN_DIGITAL_MIC4)
#define DFSDM_MIC_NUMBER           AUDIO_IN_CHANNELS

/* Buffer size defines for F4 and F7*/

#define CHANNEL_DEMUX_MASK                    	0x55U

#ifndef MAX_MIC_FREQ
#define MAX_MIC_FREQ                 	  3072  /*KHz*/
#endif

#ifndef MAX_AUDIO_IN_CHANNEL_NBR_PER_IF
#define MAX_AUDIO_IN_CHANNEL_NBR_PER_IF   2
#endif

#ifndef MAX_AUDIO_IN_CHANNEL_NBR_TOTAL
#define MAX_AUDIO_IN_CHANNEL_NBR_TOTAL    4 /* For WB, this must be minimum equal to 2 */
#endif

#ifndef PDM_FREQ_16K
#define PDM_FREQ_16K 1280
#endif

/*Number of millisecond of audio at each DMA interrupt*/
#ifndef N_MS_PER_INTERRUPT
#define N_MS_PER_INTERRUPT               (1U)
#endif

/* Default Audio IN internal buffer size */
#define DEFAULT_AUDIO_IN_BUFFER_SIZE        (uint32_t)((AUDIO_IN_SAMPLING_FREQUENCY/1000)*2)*N_MS_PER_INTERRUPT

/* Audio In states */
#define AUDIO_IN_STATE_RESET               0U
#define AUDIO_IN_STATE_RECORDING           1U
#define AUDIO_IN_STATE_STOP                2U
#define AUDIO_IN_STATE_PAUSE               3U

/* Audio In instances number:
   Instance 0 is SAI-I2S / SPI path
   Instance 1 is DFSDM path
   Instance 2 is PDM path
 */
#define AUDIO_IN_INSTANCES_NBR             3U
/**
  * @}
  */

/** @defgroup WINKY_AUDIO_Exported_Macros WINKY_AUDIO_ Exported Macros
  * @{
  */
#ifndef POS_VAL
#define POS_VAL(VAL)                  (POSITION_VAL(VAL) - 4U)
#endif
#define VOLUME_IN_CONVERT(Volume)     (((Volume) >= 100)? 239:((uint8_t)(((Volume) * 239) / 100)))

/**
  * @}
  */
/** @addtogroup WINKY_AUDIO_Exported_Variables
  * @{
  */
/* Recording context */
extern AUDIO_IN_Ctx_t                         AudioInCtx[];
/**
  * @}
  */

/** @defgroup WINKY_AUDIO_IN_Exported_Functions WINKY_AUDIO_IN Exported Functions
  * @{
  */
int32_t WINKY_AUDIO_IN_Init(WINKY_AUDIO_Init_t* AudioInit);
int32_t WINKY_AUDIO_IN_DeInit(uint32_t Instance);
int32_t WINKY_AUDIO_IN_Stop(uint32_t Instance);
int32_t WINKY_AUDIO_IN_Pause(uint32_t Instance);
int32_t WINKY_AUDIO_IN_Resume(uint32_t Instance);

int32_t WINKY_AUDIO_IN_RecordChannels(uint32_t Instance, uint8_t **pBuf, uint32_t NbrOfBytes);
int32_t WINKY_AUDIO_IN_StopChannels(uint32_t Instance, uint32_t Device);
int32_t WINKY_AUDIO_IN_PauseChannels(uint32_t Instance, uint32_t Device);
int32_t WINKY_AUDIO_IN_ResumeChannels(uint32_t Instance, uint32_t Device);

int32_t WINKY_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device);
int32_t WINKY_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device);
int32_t WINKY_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t SampleRate);
int32_t WINKY_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate);
int32_t WINKY_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample);
int32_t WINKY_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample);
int32_t WINKY_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr);
int32_t WINKY_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr);
int32_t WINKY_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume);
int32_t WINKY_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume);
int32_t WINKY_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State);

/* Specific PDM recodr APIs */
int32_t WINKY_AUDIO_IN_RecordPDM(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes);

void WINKY_AUDIO_IN_IRQHandler(uint32_t Instance, uint32_t Device);

/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function should be implemented by the user application.
   It is called into this driver when the current buffer is filled to prepare the next
   buffer pointer and its size. */
void WINKY_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance);
void WINKY_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void WINKY_AUDIO_IN_Error_CallBack(uint32_t Instance);

/**
  * @}
  */

/** @defgroup WINKY_AUDIO_IN_Private_Functions WINKY_AUDIO_IN Private Functions
  * @{
  */

HAL_StatusTypeDef MX_SAI_ClockConfig(SAI_HandleTypeDef *hSai, uint32_t PDM_rate);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* WINKY_AUDIO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

// --------------------------------------------------------------------------------------------------------------------
// ----- public variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public function prototypes
// --------------------------------------------------------------------------------------------------------------------
int32_t winky_audio_out_Init();

int32_t winky_audio_out_play();
int32_t winky_audio_out_pause();
int32_t winky_audio_out_resume();
int32_t winky_audio_out_stop();
int32_t winky_audio_out_mute();
int32_t winky_audio_out_unmute();
int32_t winky_audio_out_ismute();
int32_t winky_audio_out_setVolume();
int32_t winky_audio_out_getVolume();

int32_t winky_audio_in_Init();
int32_t winky_audio_in_DeInit();

 /* WINKY_AUDIO_H_ */
