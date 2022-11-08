#ifndef AUDO_CODEC_TLV300DAC31
#define AUDO_CODEC_TLV300DAC31

#ifdef __cplusplus
 extern "C" {
#endif

// ------------------------------------------------------------------------------------------------
// includes
// ------------------------------------------------------------------------------------------------

#include <stdint.h>
#include <string.h>

#include "audio_codec_tlv300dac31_registers.h"
// ------------------------------------------------------------------------------------------------
// constant macros
// ------------------------------------------------------------------------------------------------

#define MAX_AUDIO_CODEC_BUFFER_SIZE (4096)


/* AUDIO FREQUENCY */
#define AUDIO_FREQUENCY_192K          ((uint32_t)192000)
#define AUDIO_FREQUENCY_96K           ((uint32_t)96000)
#define AUDIO_FREQUENCY_48K           ((uint32_t)48000)
#define AUDIO_FREQUENCY_44K           ((uint32_t)44100)
#define AUDIO_FREQUENCY_32K           ((uint32_t)32000)
#define AUDIO_FREQUENCY_22K           ((uint32_t)22050)
#define AUDIO_FREQUENCY_16K           ((uint32_t)16000)
#define AUDIO_FREQUENCY_11K           ((uint32_t)11025)
#define AUDIO_FREQUENCY_8K            ((uint32_t)8000)  

/* AUDIO VOLUME */
#define CODEC_OUTPUT_VOLUME_MAX   0
#define CODEC_OUTPUT_VOLUME_MIN   (-78 * 2)

#define AUDIO_STEREO				(2)
// ------------------------------------------------------------------------------------------------
// function macros
// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------
// typedefs, structures, unions and enums
// ------------------------------------------------------------------------------------------------

typedef enum audio_codec_result_t
{
	AUDIO_CODEC_RESULT_FAILED = 0,
	AUDIO_CODEC_RESULT_SUCCESS,
	AUDIO_CODEC_RESULT_HALF_BUFFER,
	AUDIO_CODEC_RESULT_END_BUFFER,
	AUDIO_CODEC_RESULT_ERROR,
	AUDIO_CODEC_RESULT_INVALID_PARAMETER,
}audio_codec_result_t;

typedef void (* audio_codec_end_of_playing_cb_t)(audio_codec_result_t audio_codec_result);

// ------------------------------------------------------------------------------------------------
// public variables
// ------------------------------------------------------------------------------------------------


// ------------------------------------------------------------------------------------------------
// function prototypes
// ------------------------------------------------------------------------------------------------

// /!\ Only stereo sound sampling @ AUDIO_FREQUENCY_44K is currently supported

audio_codec_result_t audio_codec_tlv300dac31_init(uint32_t audio_frequency);
audio_codec_result_t audio_codec_tlv300dac31_play(uint8_t * buffer, 
												  uint16_t buffer_size, 
												  audio_codec_end_of_playing_cb_t audio_codec_end_of_playing_cb);

audio_codec_result_t audio_codec_tlv300dac31_stop(void);
audio_codec_result_t audio_codec_tlv300dac31_mute(void);
audio_codec_result_t audio_codec_tlv300dac31_unmute(void);

// int16_t vol -> should be a negative value between 0 (Max Volume) and 156 (Min Volume)
audio_codec_result_t audio_codec_tlv300dac31_change_volume(int16_t vol);


#ifdef __cplusplus
}
#endif

#endif // AUDO_CODEC_TLV300DAC31
