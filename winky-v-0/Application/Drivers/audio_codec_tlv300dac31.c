// ---------------------------------------------------------------------------------------------------------------------
// includes
// ---------------------------------------------------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include <stddef.h>

#include "audio_codec_tlv300dac31.h"
#include "main.h"

#include "app_conf.h"
#include "peripheral_config.h"
#include "app_entry.h"
#include "otp.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "app_examples_conf.h"

// ------------------------------------------------------------------------------------------------
// private constant macros
// ------------------------------------------------------------------------------------------------

#define AUDIO_CODEC_I2C_ADDR			(0b00110000)

#define AUDIO_CODEC_I2C_TIMING			(0x2000090E)	

#define ENABLE_3V0_ANALOGUE()			(HAL_GPIO_WritePin(EN_3V0ANA_GPIO_Port, EN_3V0ANA_Pin, GPIO_PIN_SET))
#define ENABLE_1V8_DIGITAL()			(HAL_GPIO_WritePin(EN_1V8DIG_GPIO_Port, EN_1V8DIG_Pin, GPIO_PIN_SET))
#define ENABLE_AUDIO_RST()				(HAL_GPIO_WritePin(AUDIO_RST_GPIO_Port, AUDIO_RST_Pin, GPIO_PIN_SET))
#define DISABLE_3V0_ANALOGUE()			(HAL_GPIO_WritePin(EN_3V0ANA_GPIO_Port, EN_3V0ANA_Pin, GPIO_PIN_RESET))
#define DISABLE_1V8_DIGITAL()			(HAL_GPIO_WritePin(EN_1V8DIG_GPIO_Port, EN_1V8DIG_Pin, GPIO_PIN_RESET))
#define DISABLE_AUDIO_RST()				(HAL_GPIO_WritePin(AUDIO_RST_GPIO_Port, AUDIO_RST_Pin, GPIO_PIN_RESET))

#define VOL_ARRAY_SIZE		(11)
// ------------------------------------------------------------------------------------------------
// private function macros
// ------------------------------------------------------------------------------------------------

/*### PLAY ###*/
/* SCK(kHz) = SAI_CK_x/(SAIClockDivider*2*256) */
#define SAIClockDivider(__FREQUENCY__) \
        (__FREQUENCY__ == AUDIO_FREQUENCY_8K)  ? 12 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_11K) ? 2 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_16K) ? 6 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_22K) ? 1 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_32K) ? 3 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_44K) ? 0 \
      : (__FREQUENCY__ == AUDIO_FREQUENCY_48K) ? 2 : 1  \
 
// ------------------------------------------------------------------------------------------------
// private typedefs, structures, unions and enums
// ------------------------------------------------------------------------------------------------


// ------------------------------------------------------------------------------------------------
// private variables
// ------------------------------------------------------------------------------------------------

static audio_codec_end_of_playing_cb_t current_audio_codec_end_of_playing_cb = NULL;

static SAI_HandleTypeDef audio_codec_sai_handle;

extern I2C_HandleTypeDef SPEAKER_I2C;
// ------------------------------------------------------------------------------------------------
// public variables
// ------------------------------------------------------------------------------------------------

DMA_HandleTypeDef audio_codec_sai_dma_tx;

// ------------------------------------------------------------------------------------------------
// private function prototypes
// ------------------------------------------------------------------------------------------------

static void write_i2c_registers(uint8_t reg_addr, uint8_t reg_value);
static uint8_t read_i2c_registers(uint8_t reg_addr);

static audio_codec_result_t audio_codec_init_sai(uint32_t audio_frequency);
static void audio_codec_configure_sai_pll(uint32_t frequency);
static audio_codec_result_t audio_codec_configure(void);
// ------------------------------------------------------------------------------------------------
// private functions
// ------------------------------------------------------------------------------------------------


static void write_i2c_registers(uint8_t reg_addr, uint8_t reg_value)
{
	if(HAL_I2C_Mem_Write(&SPEAKER_I2C, (uint16_t) AUDIO_CODEC_I2C_ADDR, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, 0xFFF) != HAL_OK)
	{
		APP_DBG_MSG("Write I2C ERROR \n");
	}
}

static uint8_t read_i2c_registers(uint8_t reg_addr)
{
	uint8_t reg_value = 0;

	if(HAL_I2C_Mem_Read(&SPEAKER_I2C, (uint16_t) AUDIO_CODEC_I2C_ADDR | 0x00, (uint16_t) reg_addr, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, 0xFFF) != HAL_OK)
	{
		APP_DBG_MSG("Read I2C ERROR \n");
	}

	return reg_value;
}

static audio_codec_result_t audio_codec_init_sai(uint32_t audio_frequency)
{
	audio_codec_result_t audio_codec_result = AUDIO_CODEC_RESULT_SUCCESS;

	uint8_t TxData[2] = {0x00, 0x00};

	audio_codec_sai_handle.Instance = SAI1_Block_B;
	/* Disable SAI peripheral to allow access to SAI internal registers */
	__HAL_SAI_DISABLE(&audio_codec_sai_handle);

	/*******************************/
	/* SAI block used for playback */
	/*******************************/
	/* Configure SAI_Block_B used for transmit
	LSBFirst: Disabled
	DataSize: 16 */
	audio_codec_sai_handle.Init.AudioMode      = SAI_MODEMASTER_TX;
	audio_codec_sai_handle.Init.Synchro        = SAI_ASYNCHRONOUS;
	audio_codec_sai_handle.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
	audio_codec_sai_handle.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
	audio_codec_sai_handle.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
	audio_codec_sai_handle.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
	audio_codec_sai_handle.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;
	audio_codec_sai_handle.Init.Mckdiv         = SAIClockDivider(audio_frequency);
	audio_codec_sai_handle.Init.Mckdiv     	   = audio_codec_sai_handle.Init.Mckdiv  * AUDIO_STEREO;
	audio_codec_sai_handle.Init.MonoStereoMode = SAI_STEREOMODE;
	audio_codec_sai_handle.Init.CompandingMode = SAI_NOCOMPANDING;
	audio_codec_sai_handle.Init.TriState       = SAI_OUTPUT_NOTRELEASED;
	audio_codec_sai_handle.Init.Protocol       = SAI_FREE_PROTOCOL;
	audio_codec_sai_handle.Init.DataSize       = SAI_DATASIZE_16;
	audio_codec_sai_handle.Init.FirstBit       = SAI_FIRSTBIT_MSB;
	audio_codec_sai_handle.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;

	/* Configure SAI_BlocB_x Frame
	Frame Length: 16
	Frame active Length: 16
	FS Definition: Start frame + Channel Side identification
	FS Polarity: FS active Low
	FS Offset: FS asserted one bit before the first bit of slot 0 */
	audio_codec_sai_handle.FrameInit.FrameLength = 32;
	audio_codec_sai_handle.FrameInit.ActiveFrameLength = 16;
	audio_codec_sai_handle.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
	audio_codec_sai_handle.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
	audio_codec_sai_handle.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

	/* Configure SAI Block_B Slot
	Slot First Bit Offset: 0
	Slot Size  : 16
	Slot Number: 1
	Slot Active: Slots 0 and 1 actives */
	audio_codec_sai_handle.SlotInit.FirstBitOffset = 0;
	audio_codec_sai_handle.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
	audio_codec_sai_handle.SlotInit.SlotNumber = 2;
	audio_codec_sai_handle.SlotInit.SlotActive = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;

	/*********************************/
	/* Initializes the SAI peripheral*/
	/*********************************/
	if (HAL_SAI_Init(&audio_codec_sai_handle) != HAL_OK)
	{
		audio_codec_result = AUDIO_CODEC_RESULT_ERROR;

		APP_DBG_MSG("[AUDIO_CODEC] Error SAI init \n");
	}
	else
	{
		/******************************************/
		/* Enable SAI peripheral to generate MCLK */
		/******************************************/
		__HAL_SAI_ENABLE(&audio_codec_sai_handle);
		
		/* Transmit one byte to start FS generation */

		if (HAL_SAI_Transmit(&audio_codec_sai_handle, TxData, 2, 1000) != HAL_OK)
		{
			audio_codec_result = AUDIO_CODEC_RESULT_ERROR;
			APP_DBG_MSG("[AUDIO_CODEC] Error while enable SAI MCLK \n");
		}
		else
		{
		    __HAL_RCC_DMA1_CLK_ENABLE();

		    audio_codec_sai_dma_tx.Init.Request				= DMA_REQUEST_SAI1_B;
		    audio_codec_sai_dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
		    audio_codec_sai_dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
		    audio_codec_sai_dma_tx.Init.MemInc              = DMA_MINC_ENABLE;
		    audio_codec_sai_dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		    audio_codec_sai_dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
		    audio_codec_sai_dma_tx.Init.Mode                = DMA_CIRCULAR;
		    audio_codec_sai_dma_tx.Init.Priority            = DMA_PRIORITY_HIGH;
		    audio_codec_sai_dma_tx.Instance                 = DMA1_Channel2;
		    /* Associate the DMA handle */
		    __HAL_LINKDMA(&audio_codec_sai_handle, hdmatx, audio_codec_sai_dma_tx);
		    /* Deinitialize the Stream for new transfer */
		    HAL_DMA_DeInit(&audio_codec_sai_dma_tx);
		    /* Configure the DMA Stream */
		    if(HAL_DMA_Init(&audio_codec_sai_dma_tx) != HAL_OK)
		    {
		    	APP_DBG_MSG("[AUDIO_CODEC] Error while enable SAI DMA \n");
		    }
		    else
		    {
			    /* SAI DMA IRQ Channel configuration */
			    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
			    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

		    }
		}

	}

	return audio_codec_result;
}


static void audio_codec_configure_sai_pll(uint32_t frequency)
{
	RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;

	/* Retrieve actual RCC configuration */
	HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);

	if ((frequency == AUDIO_FREQUENCY_11K)
	|| (frequency == AUDIO_FREQUENCY_22K)
	|| (frequency == AUDIO_FREQUENCY_44K))
	{
		/* Configure PLLSAI prescalers */
		/* SAI clock config
		PLLSAI1_VCO= (32MHz / PPLM -> 4 = )8 MHz * PLLSAI1N = 8 * 24 = VCO_192M
		SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 192/17 = 11.294 MHz */
		RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
		RCC_ExCLKInitStruct.PLLSAI1.PLLN = 24;
		RCC_ExCLKInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV17;
		RCC_ExCLKInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
		RCC_ExCLKInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV3;
		RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
		RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
	}
	else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K, AUDIO_FREQUENCY_96K */
	{
		/* SAI clock config
		PLLSAI1_VCO= (32MHz / PPLM -> 4 = ) 8MHz * PLLSAI1N = 8 * 43 = VCO_344M
		SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 344/7 = 49.142 MHz */
		RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
		RCC_ExCLKInitStruct.PLLSAI1.PLLN        = 43;
		RCC_ExCLKInitStruct.PLLSAI1.PLLP 		= RCC_PLLP_DIV7;
		RCC_ExCLKInitStruct.PLLSAI1.PLLQ 		= RCC_PLLQ_DIV2;
		RCC_ExCLKInitStruct.PLLSAI1.PLLR 		= RCC_PLLR_DIV3;
		RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
		RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
	}

	if (HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct) != HAL_OK)
	{
		APP_DBG_MSG("[AUDIO_CODEC] Error while configuring audio clock \n");
	}
}

// TODO : Update audio codec configuration for audio sampling frequecy

static audio_codec_result_t audio_codec_configure(void)
{
	audio_codec_result_t audio_codec_result = AUDIO_CODEC_RESULT_SUCCESS;

	// 1.  Define starting point

	// (a) Set register page to 0

	write_i2c_registers(AIC31XX_PAGECTL, 0x00);

	// (b) Initiate SW reset (PLL is powered off as part of reset)

	write_i2c_registers(AIC31XX_RESET, 0x01);

	// 2. Program clock settings 

	//(a) Program PLL clock dividers P, J, D, R (if PLL is used)

	// PLL_clkin = MCLK,codec_clkin = PLL_CLK
	write_i2c_registers(AIC31XX_CLKMUX, 0x03);

	// J = 8
	write_i2c_registers(AIC31XX_PLLJ, 0x08);

	// D = 0000, D(13:8) = 0, D(7:0) = 0
	write_i2c_registers(AIC31XX_PLLDMSB, 0x00);
	write_i2c_registers(AIC31XX_PLLDLSB, 0x00);	

	// (b) Power up PLL (if PLL is used)

	// PLL Power up, P = 1, R = 1
	write_i2c_registers(AIC31XX_PLLPR, 0x91);

	// (c) Program and power up NDAC

	// NDAC is powered up and set to 8
	write_i2c_registers(AIC31XX_NDAC, 0x88);


	// (d) Program and power up MDAC

	// MDAC is powered up and set to 2

	write_i2c_registers(AIC31XX_MDAC, 0x82);

	// (e) Program OSR value

	// DOSR = 128, DOSR(9:8) = 0, DOSR(7:0) = 128
	write_i2c_registers(AIC31XX_DOSRMSB, 0x00);
	write_i2c_registers(AIC31XX_DOSRLSB, 0x80);	

	// (f) Program I2S word length if required (16, 20, 24, 32 bits) and master mode (BCLK and WCLK are outputs)

	// mode is i2s, wordlength is 16, slave mode
	write_i2c_registers(AIC31XX_IFACE1, 0x00);


	// (g) Program the processing block to be used

	// Select Processing Block PRB_P11
	write_i2c_registers(AIC31XX_DACPRB, 0x0B);
	write_i2c_registers(AIC31XX_PAGECTL, 0x08);
	write_i2c_registers(AIC31XX_RESET, 0x04);
	write_i2c_registers(AIC31XX_PAGECTL, 0x00);

	// (h) Miscellaneous page 0 controls

	// DAC => volume control thru pin disable
	write_i2c_registers(AIC31XX_VOLCTRL, 0x00);

	// 3. Program analog blocks

	// (a) Set register page to 1
	write_i2c_registers(AIC31XX_PAGECTL, 0x01);

	// (b) Program common-mode voltage (defalut = 1.35 V)
	write_i2c_registers(AIC31XX_HPDRIVER, 0x04);

	// (c) Program headphone-specific depop settings (in case headphone driver is used)
	// De-pop, Power on = 800 ms, Step time = 4 ms
	// write_i2c_registers(AIC31XX_IFACESEC3, 0x4E);

	// (d) Program routing of DAC output to the output amplifier (headphone/lineout or speaker)

	// LDAC routed to HPL out, RDAC routed to HPR out
	write_i2c_registers(AIC31XX_DACMIXERROUTE, 0x44);

	// (e) Unmute and set gain of output driver

	// Unmute Class-D, set gain = 18 dB
	write_i2c_registers(AIC31XX_SPKGAIN, 0x1C);

	// (f) Power up output drivers

	// Power-up Class-D driver
	write_i2c_registers(AIC31XX_SPKAMP, 0x86);

	// Enable Class-D output analog volume, set = -9 dB
	write_i2c_registers(AIC31XX_ANALOGSPK, 0x00);

	// 4. Apply waiting time determined by the de-pop settings and the soft-stepping settings 
	// of the driver gain or poll page 1 / register 63

	HAL_Delay(1000);

	// 5. Power up DAC
	
	// (a) Set register page to 0
	write_i2c_registers(AIC31XX_PAGECTL, 0x00);

	// (b) Power up DAC channels and set digital gain

	// Powerup DAC left and right channels (soft step enabled)
	write_i2c_registers(AIC31XX_DACSETUP, 0xD4);

	// DAC Left gain = -22 dB
	write_i2c_registers(AIC31XX_LDACVOL, 0xD4);

	// DAC Right gain = -22 dB
	write_i2c_registers(AIC31XX_RDACVOL, 0xD4);


	return audio_codec_result;
}
// ------------------------------------------------------------------------------------------------
// interrupt handlers
// ------------------------------------------------------------------------------------------------

//void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
//{
//	// APP_DBG_MSG("[AUDIO_CODEC] DMA Done");
//
//	if(current_audio_codec_end_of_playing_cb != NULL)
//	{
//		current_audio_codec_end_of_playing_cb(AUDIO_CODEC_RESULT_END_BUFFER);
//	}
//}
//
//void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
//{
//	// APP_DBG_MSG("[AUDIO_CODEC] DMA Half Done");
//
//	if(current_audio_codec_end_of_playing_cb != NULL)
//	{
//		current_audio_codec_end_of_playing_cb(AUDIO_CODEC_RESULT_HALF_BUFFER);
//	}
//}
//
//void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
//{
//	APP_DBG_MSG("[AUDIO_CODEC] DMA ERROR");
//
//	if(current_audio_codec_end_of_playing_cb != NULL)
//	{
//		current_audio_codec_end_of_playing_cb(AUDIO_CODEC_RESULT_ERROR);
//	}
//
//}


// ------------------------------------------------------------------------------------------------
// public functions
// ------------------------------------------------------------------------------------------------

audio_codec_result_t audio_codec_tlv300dac31_init(uint32_t audio_frequency)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	audio_codec_result_t audio_codec_result = AUDIO_CODEC_RESULT_SUCCESS;

	// 1- Power UP the Codec

	/*Configure GPIO pin Output Level */
	// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_11, GPIO_PIN_SET);

	DISABLE_3V0_ANALOGUE();
	DISABLE_1V8_DIGITAL();
	DISABLE_AUDIO_RST();

    GPIO_InitStructure.Pin       = EN_3V0ANA_Pin;
    GPIO_InitStructure.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull      = GPIO_NOPULL;
    GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EN_3V0ANA_GPIO_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin       = EN_1V8DIG_Pin;
    GPIO_InitStructure.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull      = GPIO_NOPULL;
    GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EN_1V8DIG_GPIO_Port, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = AUDIO_RST_Pin;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(AUDIO_RST_GPIO_Port, &GPIO_InitStructure);

	// ENABLE AUDIO DIGITAL THEN ANALOG THEN RST
	ENABLE_1V8_DIGITAL();
	HAL_Delay(100);
	ENABLE_3V0_ANALOGUE();
	HAL_Delay(200);
	ENABLE_AUDIO_RST();
	HAL_Delay(100);

	audio_codec_configure_sai_pll(audio_frequency);
	
	// 2- Enable SAI
	
	if(audio_codec_init_sai(audio_frequency) != AUDIO_CODEC_RESULT_SUCCESS)
	{
		APP_DBG_MSG("[AUDIO_CODEC] Error init SAI\n");
	}
	else
	{

		if(audio_codec_configure() != AUDIO_CODEC_RESULT_SUCCESS)
		{
			APP_DBG_MSG("[AUDIO_CODEC] Error init Codec Configuration\n");
		}
		else
		{
			APP_DBG_MSG("[AUDIO_CODEC] Codec Init Completed !!!\n");
		}

	}

	return audio_codec_result;

}

audio_codec_result_t audio_codec_tlv300dac31_play(uint8_t * buffer, 
												  uint16_t buffer_size, 
												  audio_codec_end_of_playing_cb_t audio_codec_end_of_playing_cb)
{
	audio_codec_result_t audio_codec_result = AUDIO_CODEC_RESULT_SUCCESS;

	if(buffer == NULL || buffer_size == 0 || buffer_size > MAX_AUDIO_CODEC_BUFFER_SIZE)
	{
		APP_DBG_MSG("[AUDIO_CODEC] Invalid Parameter");

		audio_codec_result = AUDIO_CODEC_RESULT_INVALID_PARAMETER;
	}
	else
	{
		current_audio_codec_end_of_playing_cb = audio_codec_end_of_playing_cb;

		if (HAL_SAI_Transmit_DMA(&audio_codec_sai_handle, (uint8_t *)buffer, buffer_size) != HAL_OK)
		{
			APP_DBG_MSG("[AUDIO_CODEC] SAI DMA Transmit Error");
		}
	}

	return audio_codec_result;

}

audio_codec_result_t audio_codec_tlv300dac31_stop(void)
{
	audio_codec_result_t audio_codec_result = AUDIO_CODEC_RESULT_SUCCESS;

	if(HAL_SAI_DMAStop(&audio_codec_sai_handle) != HAL_OK)
	{
		APP_DBG_MSG("[Codec] Impossible to stop SAI DMA");
	}

	return audio_codec_result;

}
audio_codec_result_t audio_codec_tlv300dac31_mute(void)
{
	audio_codec_result_t audio_codec_result = AUDIO_CODEC_RESULT_SUCCESS;

	write_i2c_registers(AIC31XX_DACMUTE, 0x08);

	if(HAL_SAI_DMAPause(&audio_codec_sai_handle) != HAL_OK)
	{
		APP_DBG_MSG("[Codec] Impossible to pause SAI DMA");
	}

	// TODO : SAI_DMA_PAUSE

	return audio_codec_result;

}
audio_codec_result_t audio_codec_tlv300dac31_unmute(void)
{
	audio_codec_result_t audio_codec_result = AUDIO_CODEC_RESULT_SUCCESS;

	// (c) Unmute digital volume control

	// Unmute DAC left and right channels
	write_i2c_registers(AIC31XX_DACMUTE, 0x00);

	return audio_codec_result;

}

audio_codec_result_t audio_codec_tlv300dac31_change_volume(int16_t vol)
{
	uint8_t vol_val;
	uint8_t vol_index;
	uint8_t vol_array[VOL_ARRAY_SIZE] = { 107, 108, 110, 113, 116, 120, 125, 128, 132, 138, 144 };

	audio_codec_result_t audio_codec_result = AUDIO_CODEC_RESULT_SUCCESS;

	if ((vol > CODEC_OUTPUT_VOLUME_MAX) ||
		(vol < CODEC_OUTPUT_VOLUME_MIN)) 
	{
		APP_DBG_MSG("Invalid volume %d.%d dB",vol >> 1, ((uint32_t)vol & 1) ? 5 : 0);
		
		audio_codec_result = AUDIO_CODEC_RESULT_INVALID_PARAMETER;
	}
	else
	{
		/* remove sign */
		vol = -vol;

		/* if volume is near floor, set minimum */
		if (vol > SPK_ANA_VOL_FLOOR) 
		{
			vol_val = SPK_ANA_VOL_FLOOR;
		} 
		else if (vol > SPK_ANA_VOL_LOW_THRESH) 
		{
			/* lookup low volume values */
			for (vol_index = 0; vol_index < VOL_ARRAY_SIZE; vol_index++) 
			{
				if (vol_array[vol_index] >= vol) 
				{
					break;
				}
			}
			
			vol_val = SPK_ANA_VOL_LOW_THRESH + vol_index + 1;
		} 
		else 
		{
			vol_val = (uint8_t)vol;
		}

		write_i2c_registers(AIC31XX_LDACVOL, SPK_ANA_VOL(vol_val));
		write_i2c_registers(AIC31XX_RDACVOL, SPK_ANA_VOL(vol_val));

	}

	return audio_codec_result;
}
