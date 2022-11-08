/*
 * winky_audio_conf.h
 *
 *  Created on: 19 déc. 2019
 *      Author: clement
 */

#ifndef WINKY_AUDIO_CONF_H_
#define WINKY_AUDIO_CONF_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"


/* The default value of the N_MS_PER_INTERRUPT directive in the driver is set to 1,
for backward compatibility: leaving this values as it is allows to avoid any
modification in the application layer developed with the older versions of the driver */

/*Number of millisecond of audio at each DMA interrupt*/
#define N_MS_PER_INTERRUPT               (1U)

#define AUDIO_IN_CHANNELS 4
#define AUDIO_IN_SAMPLING_FREQUENCY 16000

#define AUDIO_IN_BUFFER_SIZE            DEFAULT_AUDIO_IN_BUFFER_SIZE
#define AUDIO_VOLUME_INPUT              64U
#define WINKY_AUDIO_INSTANCE 0U
#define WINKY_AUDIO_IN_IT_PRIORITY    5U

#if (AUDIO_IN_SAMPLING_FREQUENCY == 8000)
#define MAX_DECIMATION_FACTOR 160
#else
#define MAX_DECIMATION_FACTOR 128
#endif

  /*#define USE_SPI3*/
  /*If you wanto to use SPI3 instead of SPI2 for M3 and M4, uncomment this define and
  close SB20 and SB21*/

#ifdef __cplusplus
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


#endif /* WINKY_AUDIO_CONF_H_ */
