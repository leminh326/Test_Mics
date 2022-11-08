/*
 * apds9960.c
 *
 *  Created on: 18 juin 2019
 *      Author: clement
 *  Port of Adafruit_APDS9960 library
 */
#include "apds9960.h"

#include <stdbool.h>

//TODO#include "i2c.h"
#include "math.h"
#include "cmsis_os.h"

static HAL_StatusTypeDef apds9960WriteRegister(I2C_HandleTypeDef* i2c, uint16_t devAddress, uint16_t reg) {
	return HAL_I2C_Master_Transmit(i2c, devAddress, &reg, 1, 100);
}

static HAL_StatusTypeDef apds9960Write8(I2C_HandleTypeDef* i2c, uint16_t devAddress, uint16_t memAddress, uint8_t value) {
	return HAL_I2C_Mem_Write(i2c,devAddress,memAddress,1,&value,1,100);
}

static HAL_StatusTypeDef apds9960Read8(I2C_HandleTypeDef* i2c, uint16_t devAddress, uint16_t reg, uint8_t *pData) {
	return HAL_I2C_Mem_Read(i2c, devAddress, reg, 1, pData, 1, 100);
}

static HAL_StatusTypeDef apds9960Read(I2C_HandleTypeDef* i2c, uint16_t devAddress, uint16_t reg, uint8_t *pData, uint16_t size) {
	return HAL_I2C_Mem_Read(i2c, devAddress, reg, 1, pData, size, 100);
}

HAL_StatusTypeDef apds9960Init(apds9960_HandleTypedef* handle)
{
	// check that i2c exists at this address
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(handle->i2c,handle->addr,1,20000);
	if( status != HAL_OK){
		return HAL_ERROR;
	}

	/* Make sure we're actually connected */
	uint8_t apds9960Id = 0;
	apds9960Read8(handle->i2c,handle->addr,APDS9960_ID,&apds9960Id);
	if (apds9960Id != 0xAB) {
		return HAL_ERROR;
	}
#if 0
    /* Gesture config register dump */
    uint8_t reg;
    uint8_t val = 0;

    for(reg = 0x80; reg <= 0xAF; reg++) {
        if( (reg != 0x82) && \
            (reg != 0x8A) && \
            (reg != 0x91) && \
            (reg != 0xA8) && \
            (reg != 0xAC) && \
            (reg != 0xAD) )
        {
        	apds9960Read8(handle->i2c,handle->addr,reg, &val);
            printf("reg: %x : %x\r\n", reg,val);
        }
    }

    for(reg = 0xE4; reg <= 0xE7; reg++) {
    	apds9960Read8(handle->i2c,handle->addr,reg, &val);
    	printf("reg: %x : %x\r\n", reg,val);
    }
#endif

    	handle->enable = 0; 	// (GEN << 6) | (PIEN << 5) | (AIEN << 4) | (WEN << 3) | (PEN << 2) | (AEN << 1) | PON
    	handle->pers = 0; 		//(PPERS << 4) | APERS
    	handle->control = 0; 	//(LDRIVE << 6) | (PGAIN << 2) | AGAIN
    	handle->config1 = 1; 	// WLONG << 1
    	handle->config2 = 1 ;	// (PSIEN << 7) | (CPSIEN << 6) | (LED_BOOST << 4) | 1
    	handle->config3 = 0;	// (PCMP << 5) | (SAI << 4) | (PMASK_U << 3) | (PMASK_D << 2) | (PMASK_L << 1) | PMASK_R
    	handle->ppulse = 0; 	//(PPLEN << 6) | PPULSE
    	handle->status = 0;     // CPSAT | PGSAT | PINT | AINT | reserver | GINT | PVALID | AVALID
    	handle->gconf1 = 0; // (GFIFOTH << 6) | (GEXMSK << 2) | GEXPER
    	handle->gconf2 = 0;		// (GGAIN << 5) | (GLDRIVE << 3) | GWTIME
    	handle->gconf3 = 0;		// GDIMS
    	handle->gconf4 = 0;		//(GIEN << 1) | GMODE
    	handle->gpulse = 0;		//(GPLEN << 6) | GPULSE
    	handle->gstatus = 0;    // GFOV << 1 | GVALID

//	handle->enable = (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | 1; 	// (GEN << 6) | (PIEN << 5) | (AIEN << 4) | (WEN << 3) | (PEN << 2) | (AEN << 1) | PON
//	handle->pers = (4 << 4) | 4; 		//(PPERS << 4) | APERS
//	handle->control = (2 << 6) | (2 << 2) | 2; 	//(LDRIVE << 6) | (PGAIN << 2) | AGAIN
//	handle->config1 = (1 << 1); 	// WLONG << 1
//	handle->config2 = (1 << 7) | (1 << 6) | (2 << 4) | 1; 	// (PSIEN << 7) | (CPSIEN << 6) | (LED_BOOST << 4) | 1
//	handle->config3 = (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | 1;	// (PCMP << 5) | (SAI << 4) | (PMASK_U << 3) | (PMASK_D << 2) | (PMASK_L << 1) | PMASK_R
//	handle->ppulse = (2 << 6) | 6; 	//(PPLEN << 6) | PPULSE
//	handle->status = 0xFF;     // CPSAT | PGSAT | PINT | AINT | reserver | GINT | PVALID | AVALID
//	handle->gconf1 = (2 << 6) | (4 << 2) | 2 ; // (GFIFOTH << 6) | (GEXMSK << 2) | GEXPER
//	handle->gconf2 = (2 << 5) | (2 << 3) | 3 ;		// (GGAIN << 5) | (GLDRIVE << 3) | GWTIME
//	handle->gconf3 = 2;		// GDIMS
//	handle->gconf4 = (2 << 1) | 1;		//(GIEN << 1) | GMODE
//	handle->gpulse = (2 << 6) | 6;		//(GPLEN << 6) | GPULSE
//	handle->gstatus = (1 << 1) | 1;    // GFOV << 1 | GVALID


	// disable everything to start
	apds9960EnableGesture(handle, false);
	apds9960EnableProximity(handle, false);
	apds9960EnableColor(handle, false);

	apds9960DisableColorInterrupt(handle);
	apds9960DisableProximityInterrupt(handle);
	apds9960ClearInterrupt(handle);

	/* Note: by default, the device is in power down mode on bootup */
	apds9960Enable(handle, false);
	osDelay(10);
	apds9960Enable(handle, true);
	osDelay(10);

	  // default to all gesture dimensions
	apds9960SetGestureDimensions(handle,APDS9960_DIMENSIONS_ALL);
	apds9960SetGestureFIFOThreshold(handle,APDS9960_GFIFO_4);
	apds9960SetGestureGain(handle,APDS9960_GGAIN_4);
	apds9960SetGestureProximityThreshold(handle,50);
	apds9960ResetCounts(handle);
	apds9960SetADCIntegrationTime(handle, 10);
    apds9960SetADCGain(handle, APDS9960_AGAIN_4X);
    if( apds9960Write8(handle->i2c,handle->addr,APDS9960_GPULSE, DEFAULT_GPULSE) != HAL_OK) {
        return HAL_ERROR;
    }


//    /* Set default values for ambient light and proximity registers */
//	apds9960SetADCIntegrationTime(handle, DEFAULT_ATIME);
//    if( apds9960Write8(handle->i2c,handle->addr,APDS9960_WTIME, DEFAULT_WTIME) != HAL_OK) {
//        return HAL_ERROR;
//    }
//    if( apds9960Write8(handle->i2c,handle->addr,APDS9960_PPULSE, DEFAULT_PROX_PPULSE) != HAL_OK) {
//        return HAL_ERROR;
//    }
//    if( apds9960Write8(handle->i2c,handle->addr,APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR) != HAL_OK) {
//        return HAL_ERROR;
//    }
//    if( apds9960Write8(handle->i2c,handle->addr,APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL) != HAL_OK) {
//        return HAL_ERROR;
//    }
//    if( apds9960Write8(handle->i2c,handle->addr,APDS9960_CONFIG1, DEFAULT_CONFIG1) != HAL_OK) {
//        return HAL_ERROR;
//    }
//    apds9960SetLED(handle,DEFAULT_LDRIVE,APDS9960_LEDBOOST_100PCNT);
//    apds9960SetProxGain(handle, DEFAULT_PGAIN);
//    apds9960SetADCGain(handle, DEFAULT_AGAIN);
//    apds9960SetProximityInterruptThreshold(handle,DEFAULT_PILT,DEFAULT_PIHT,DEFAULT_PERS);
//    apds9960SetIntLimits(handle,DEFAULT_AILT,DEFAULT_AIHT);
//
//
//    if( apds9960Write8(handle->i2c,handle->addr,APDS9960_CONFIG2, DEFAULT_CONFIG2) != HAL_OK) {
//        return HAL_ERROR;
//    }
//    if( apds9960Write8(handle->i2c,handle->addr,APDS9960_CONFIG3, DEFAULT_CONFIG3) != HAL_OK) {
//        return HAL_ERROR;
//    }
//
//    apds9960SetGestureProximityThreshold(handle,DEFAULT_GPENTH);
//    apds9960SetGestureExitThreshold(handle,DEFAULT_GEXTH);
//    apds9960SetGestureFIFOThreshold(handle, APDS9960_GFIFO_4);
//    apds9960SetGestureGain(handle, DEFAULT_GGAIN);
//    apds9960SetGestureLEDDrive(handle, DEFAULT_GLDRIVE);
//    apds9960SetGestureWaitTime(handle, DEFAULT_GWTIME);
//
//    apds9960SetGestureOffset(handle, APDS9960_GOFFSET_U,APDS9960_GOFFSET_D,APDS9960_GOFFSET_L,APDS9960_GOFFSET_R);
//
//
//    if( apds9960Write8(handle->i2c,handle->addr,APDS9960_GPULSE, DEFAULT_GPULSE) != HAL_OK) {
//        return HAL_ERROR;
//    }
//	apds9960SetGestureDimensions(handle, APDS9960_DIMENSIONS_ALL);


#if 0
    /* Gesture config register dump */
    reg;
    val = 0;

    for(reg = 0x80; reg <= 0xAF; reg++) {
        if( (reg != 0x82) && \
            (reg != 0x8A) && \
            (reg != 0x91) && \
            (reg != 0xA8) && \
            (reg != 0xAC) && \
            (reg != 0xAD) )
        {
        	apds9960Read8(handle->i2c,handle->addr,reg, &val);
            printf("reg: %x : %x\r\n", reg,val);
        }
    }

    for(reg = 0xE4; reg <= 0xE7; reg++) {
    	apds9960Read8(handle->i2c,handle->addr,reg, &val);
    	printf("reg: %x : %x\r\n", reg,val);
    }
#endif

	return HAL_OK;
}

/*!
 *  @brief  Sets the integration time for the ADC of the APDS9960, in millis
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *                the configuration information for the APDS9960.
 *  @param  iTimeMS
 *          Integration time
 */
void apds9960SetADCIntegrationTime(apds9960_HandleTypedef* handle, uint16_t iTimeMS){
	float temp;

	// convert ms into 2.78ms increments
	temp = iTimeMS;
	temp /= 2.78;
	temp = 256 - temp;
	if (temp > 255)
		temp = 255;
	if (temp < 0)
		temp = 0;

	/* Update the timing register */
	apds9960Write8(handle->i2c, handle->addr, APDS9960_ATIME, (uint8_t)temp);
}

/*!
 *  @brief  Returns the integration time for the ADC of the APDS9960, in millis
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @return Integration time
 */
float apds9960GetADCIntegrationTime(apds9960_HandleTypedef* handle)
{
	float temp;

	uint8_t value = 0;
	apds9960Read8(handle->i2c,handle->addr,APDS9960_ATIME,&value);
	temp = (float) value;

	// convert to units of 2.78 ms
	temp = 256 - temp;
	temp *= 2.78;
	return temp;
}

/*!
 *  @brief  Adjusts the color/ALS gain on the APDS9960 (adjusts the sensitivity
 *          to light)
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  aGain
 *          Gain
 */
void apds9960SetADCGain(apds9960_HandleTypedef* handle, apds9960AGain_t aGain){

	handle->control = (handle->control & ~(3)) | aGain;
	/* Update the timing register */
	apds9960Write8(handle->i2c, handle->addr, APDS9960_CONTROL, handle->control);
}

/*!
 *  @brief  Returns the ADC gain
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @return ADC gain
 */
apds9960AGain_t apds9960GetADCGain(apds9960_HandleTypedef* handle){
	uint8_t control;
	apds9960Read8(handle->i2c,handle->addr,APDS9960_CONTROL,&control);
	return (apds9960AGain_t) (control & 0x03);
}

/*!
 *  @brief  Set LED brightness for proximity/gesture
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  drive
 *          LED Drive
 *  @param  boost
 *          LED Boost
 */
void apds9960SetLED(apds9960_HandleTypedef* handle, apds9960LedDrive_t drive, apds9960LedBoost_t boost){
	handle->control = (handle->control & ~(3<<6)) | (drive<<6);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_CONTROL, handle->control);

	handle->config2 = (handle->config2 & ~(3<<4)) | (boost<<6) | 1;
	apds9960Write8(handle->i2c, handle->addr, APDS9960_CONFIG2, handle->config2);

}

/*!
 *  @brief  Enable proximity readings on APDS9960
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  en
 *          Enable (false/true)
 */
void apds9960EnableProximity(apds9960_HandleTypedef* handle, bool en){
	handle->enable = (handle->enable & ~(1<<2)) | (en<<2);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_ENABLE, handle->enable);
}

/*!
 *  @brief  Adjusts the Proximity gain on the APDS9960
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  pGain
 *          Gain
 */
void apds9960SetProxGain(apds9960_HandleTypedef* handle, apds9960PGain_t pGain){
	handle->control = (handle->control & ~(3<<2)) | (pGain<<2);
	/* Update the timing register */
	apds9960Write8(handle->i2c, handle->addr, APDS9960_CONTROL, handle->control);
}

/*!
 *  @brief  Returns the Proximity gain on the APDS9960
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @return Proxmity gain
 */
apds9960PGain_t apds9960GetProxGain(apds9960_HandleTypedef* handle){
	uint8_t control;
	apds9960Read8(handle->i2c,handle->addr,APDS9960_CONTROL,&control);
	return (apds9960AGain_t) (control & 0x0C);
}

/*!
 *  @brief  Sets number of proxmity pulses
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  pLen
 *          Pulse Length
 *  @param  pulses
 *          Number of pulses
 */
void apds9960SetProxPulse(apds9960_HandleTypedef* handle, apds9960PPulseLen_t pLen, uint8_t pulses){
	if (pulses < 1)
		pulses = 1;
	if (pulses > 64)
		pulses = 64;
	pulses--;

	handle->ppulse = (handle->ppulse & ~(3<<6)) | (pLen<<6) | pulses;
	apds9960Write8(handle->i2c, handle->addr, APDS9960_PPULSE, handle->ppulse);
}

/*!
 *  @brief  Enable proximity interrupts
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 */
void apds9960EnableProximityInterrupt(apds9960_HandleTypedef* handle){
	handle->enable = (handle->enable & ~(1<<5)) | (1<<5);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_ENABLE, handle->enable);
	apds9960ClearInterrupt(handle);
}

/*!
 *  @brief  Disable proximity interrupts
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 */
void apds9960DisableProximityInterrupt(apds9960_HandleTypedef* handle){
	handle->enable = (handle->enable & ~(1<<5)) | (0<<5);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_ENABLE, handle->enable);
}

/*!
 *  @brief  Read proximity data
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @return Proximity
 */
uint8_t apds9960ReadProximity(apds9960_HandleTypedef* handle){
	uint8_t prox;
	apds9960Read8(handle->i2c,handle->addr,APDS9960_PDATA,&prox);
	return prox;
}

/*!
 *  @brief  Set proxmity interrupt thresholds
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  low
 *          Low threshold
 *  @param  high
 *          High threshold
 *  @param  persistance
 *          Persistance
 */
void apds9960SetProximityInterruptThreshold(apds9960_HandleTypedef* handle, uint8_t low, uint8_t high, uint8_t persistance){
	apds9960Write8(handle->i2c, handle->addr,APDS9960_PILT, low);
	apds9960Write8(handle->i2c, handle->addr,APDS9960_PIHT, high);

	if (persistance > 7)
		persistance = 7;
	handle->pers = (handle->pers & ~(0xF0<<4)) | (persistance<<4); //(PPERS << 4) | APERS
	apds9960Write8(handle->i2c, handle->addr, APDS9960_PERS, handle->pers);
}


/*!
 *  @brief  Returns proxmity interrupt status
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @return True if enabled, false otherwise.
 */
bool apds9960GetProximityInterrupt(apds9960_HandleTypedef* handle){
	uint8_t prox;
	apds9960Read8(handle->i2c,handle->addr,APDS9960_PDATA,&prox);
	return prox;
}

/*!
 *  @brief  Enable gesture readings on APDS9960
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  en
 *          Enable (True/False)
 */
void apds9960EnableGesture(apds9960_HandleTypedef* handle, bool en){
	if (!en) {
		handle->gconf4 = (handle->control & ~(1)) | 0;
		apds9960Write8(handle->i2c, handle->addr, APDS9960_GCONF4, handle->gconf4);
	}
	handle->enable = (handle->enable & ~(1<<6)) | (en<<6);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_ENABLE, handle->enable );
	apds9960ResetCounts(handle);
}

/*!
 *  @brief  Returns validity status of a gesture
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @return Status (false/true)
 */
bool apds9960GestureValid(apds9960_HandleTypedef* handle){
	uint8_t val;

	apds9960Read8(handle->i2c, handle->addr, APDS9960_GSTATUS, &(handle->gstatus));
	val = handle->gstatus & 0b00000001;
	if (val == 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*!
 *  @brief  Sets gesture dimensions
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  dims
 *          Dimensions (APDS9960_DIMENSIONS_ALL, APDS9960_DIMENSIONS_UP_DOWM,
 *          APDS9960_DIMENSIONS_UP_DOWN, APGS9960_DIMENSIONS_LEFT_RIGHT)
 */
void apds9960SetGestureDimensions(apds9960_HandleTypedef* handle, uint8_t dims){
	handle->gconf3 = dims;
	apds9960Write8(handle->i2c, handle->addr,APDS9960_GCONF3, handle->gconf3);
}

/*!
 *  @brief  Sets gesture FIFO Threshold
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  thresh
 *          Threshold (APDS9960_GFIFO_1, APDS9960_GFIFO_4, APDS9960_GFIFO_8,
 *          APDS9960_GFIFO_16)
 */
void apds9960SetGestureFIFOThreshold(apds9960_HandleTypedef* handle, uint8_t thresh){
	handle->gconf1 = (handle->gconf1 & ~(3<<6)) | (thresh<<6);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_GCONF1, handle->gconf1);
}

/*!
 *  @brief  Sets gesture sensor gain
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  thresh
 *  @param  gain
 *          Gain (APDS9960_GAIN_1, APDS9960_GAIN_2, APDS9960_GAIN_4,
 *          APDS9960_GAIN_8)
 */
void apds9960SetGestureGain(apds9960_HandleTypedef* handle, uint8_t gain){
	handle->gconf2 = (handle->gconf2 & ~(3<<5)) | (gain<<5);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_GCONF2, handle->gconf2);
}

void apds9960SetGestureLEDDrive(apds9960_HandleTypedef* handle, uint8_t drive){
	handle->gconf2 = (handle->gconf2 & ~(3<<3)) | (drive<<3);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_GCONF2, handle->gconf2);
}

void apds9960SetGestureWaitTime(apds9960_HandleTypedef* handle, uint8_t time){
	handle->gconf2 = (handle->gconf2 & ~(3)) | (time);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_GCONF2, handle->gconf2);
}

/*!
 *  @brief  Sets gesture sensor threshold
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  thresh
 *          Threshold
 */
void apds9960SetGestureProximityThreshold(apds9960_HandleTypedef* handle, uint8_t thresh){
	apds9960Write8(handle->i2c, handle->addr, APDS9960_GPENTH, thresh);
}


/*!
 *  @brief  Sets gesture sensor exit threshold
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  thresh
 *          Threshold
 */
void apds9960SetGestureExitThreshold(apds9960_HandleTypedef* handle, uint8_t thresh){
	apds9960Write8(handle->i2c, handle->addr, APDS9960_GEXTH, thresh);
}

/*!
 *  @brief  Sets gesture sensor offset
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  offset_up
 *          Up offset
 *  @param  offset_down
 *          Down offset
 *  @param  offset_left
 *          Left offset
 *  @param  offset_right
 *          Right offset
 */
void apds9960SetGestureOffset(apds9960_HandleTypedef* handle, uint8_t offset_up, uint8_t offset_down,
		uint8_t offset_left, uint8_t offset_right){
	apds9960Write8(handle->i2c, handle->addr, APDS9960_GOFFSET_U, offset_up);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_GOFFSET_D, offset_down);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_GOFFSET_L, offset_left);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_GOFFSET_R, offset_right);
}

/*!
 *  @brief  Reads gesture
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @return Received gesture (APDS9960_DOWN APDS9960_UP, APDS9960_LEFT
 *          APDS9960_RIGHT)
 */
uint8_t apds9960ReadGesture(apds9960_HandleTypedef* handle){
	  uint8_t toRead = 0;
	  uint8_t buf[128];
	  unsigned long t = 0;
	  uint8_t gestureReceived = APDS9960_GESTURE_FAR;
	  while (1) {
	    int up_down_diff = 0;
	    int left_right_diff = 0;
	    gestureReceived = 0;
	    if (!apds9960GestureValid(handle))
	      return 0;

	    osDelay(30);
	    apds9960Read8(handle->i2c, handle->addr, APDS9960_GFLVL, &toRead);
	    apds9960Read(handle->i2c, handle->addr, APDS9960_GFIFO_U, buf, (uint16_t) (toRead * 4));

	    for (int i = 0; i < toRead; i++) {
	      if (abs((int)buf[i * 4 + 0] - (int)buf[i * 4 + 1]) > 13) {
	        up_down_diff += (int)buf[i * 4 + 0] - (int)buf[i * 4 + 1];
	      }
	      if (abs((int)buf[i * 4 + 2] - (int)buf[i * 4 + 3]) > 13) {
	        left_right_diff += (int)buf[i * 4 + 2] - (int)buf[i * 4 + 3];
	      }
	    }
	    if (up_down_diff != 0) {
	      if (up_down_diff < 0) {
	        if (handle->DCount > 0) {
	          gestureReceived = APDS9960_GESTURE_UP;
	        } else
	        	handle->UCount++;
	      } else if (up_down_diff > 0) {
	        if (handle->UCount > 0) {
	          gestureReceived = APDS9960_GESTURE_DOWN;
	        } else
	        	handle->DCount++;
	      }
	    }

	    if (left_right_diff != 0) {
	      if (left_right_diff < 0) {
	        if (handle->RCount > 0) {
	          gestureReceived = APDS9960_GESTURE_LEFT;
	        } else
	        	handle->LCount++;
	      } else if (left_right_diff > 0) {
	        if (handle->LCount > 0) {
	          gestureReceived = APDS9960_GESTURE_RIGHT;
	        } else
	        	handle->RCount++;
	      }
	    }

	    if(gestureReceived != APDS9960_GESTURE_FAR)
	    {
	    	apds9960ResetCounts(handle);
	    }

	    if (up_down_diff != 0 || left_right_diff != 0)
	      t = osKernelSysTick();

	    if (osKernelSysTick() - t > 300) {
	    	apds9960ResetCounts(handle);
	    }
    	return gestureReceived;
	}
}

/*!
 *  @brief  Resets gesture counts
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 */
void apds9960ResetCounts(apds9960_HandleTypedef* handle){
	handle->gestCnt = 0;
	handle->UCount = 0;
	handle->DCount = 0;
	handle->LCount = 0;
	handle->RCount = 0;
}

/*!
 *  @brief  Enable proximity readings on APDS9960
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  en
 *          Enable (false/true)
 */
void apds9960EnableColor(apds9960_HandleTypedef* handle, bool en){
	handle->enable = (handle->enable & ~(1<<1)) | (en<<1);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_ENABLE, handle->enable );
}

/*!
 *  @brief  Returns status of color data
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @return True if color data ready, False otherwise
 */
bool apds9960ColorDataReady(apds9960_HandleTypedef* handle){
	apds9960Read8(handle->i2c, handle->addr, APDS9960_STATUS, &(handle->status));
	return (bool) (handle->gstatus & 0x01);
}


/*!
 *  @brief  Reads the raw red, green, blue and clear channel values
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  *r
 *          Red value
 *  @param  *g
 *          Green value
 *  @param  *b
 *          Blue value
 *  @param  *c
 *          Clear channel value
 */
void apds9960GetColorData(apds9960_HandleTypedef* handle, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c){
	  apds9960Read(handle->i2c, handle->addr, APDS9960_CDATAL, c, 2);
	  apds9960Read(handle->i2c, handle->addr, APDS9960_RDATAL, r, 2);
	  apds9960Read(handle->i2c, handle->addr, APDS9960_GDATAL, g, 2);
	  apds9960Read(handle->i2c, handle->addr, APDS9960_BDATAL, b, 2);
}

/*!
 *  @brief  Converts the raw R/G/B values to color temperature in degrees Kelvin
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @return Color temperature
 */
uint16_t apds9960CalculateColorTemperature(uint16_t r, uint16_t g, uint16_t b){
	 float X, Y, Z; /* RGB to XYZ correlation      */
	 float xc, yc;  /* Chromaticity co-ordinates   */
	 float n;       /* McCamy's formula            */
	 float cct;

	  /* 1. Map RGB values to their XYZ counterparts.    */
	  /* Based on 6500K fluorescent, 3000K fluorescent   */
	  /* and 60W incandescent values for a wide range.   */
	  /* Note: Y = Illuminance or lux                    */
	  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
	  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
	  Z = (-0.68202F * r) + (0.77073F * g) + (0.56332F * b);

	  /* 2. Calculate the chromaticity co-ordinates      */
	  xc = (X) / (X + Y + Z);
	  yc = (Y) / (X + Y + Z);

	  /* 3. Use McCamy's formula to determine the CCT    */
	  n = (xc - 0.3320F) / (0.1858F - yc);

	  /* Calculate the final CCT */
	  cct =
	      (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

	  /* Return the results in degrees Kelvin */
	return (uint16_t)cct;
}

/*!
 *  @brief  Calculate ambient light values
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @return LUX value
 */
uint16_t apds9960CalculateLux(uint16_t r, uint16_t g, uint16_t b){
	float illuminance;

	  /* This only uses RGB ... how can we integrate clear or calculate lux */
	  /* based exclusively on clear since this might be more reliable?      */
	  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

	return (uint16_t)illuminance;
}

/*!
 *  @brief  Enables color interrupt
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 */
void apds9960EnableColorInterrupt(apds9960_HandleTypedef* handle){
	handle->enable = (handle->enable & ~(1<<4)) | (1<<4);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_ENABLE, handle->enable );
}

/*!
 *  @brief  Disables color interrupt
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 */
void apds9960DisableColorInterrupt(apds9960_HandleTypedef* handle){
	handle->enable = (handle->enable & ~(1<<4)) | (0<<4);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_ENABLE, handle->enable );
}


/*!
 *  @brief  Clears interrupt
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 */
void apds9960ClearInterrupt(apds9960_HandleTypedef* handle){
	apds9960WriteRegister(handle->i2c, handle->addr, APDS9960_AICLEAR);
}

/*!
 *  @brief  Sets interrupt limits
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  low
 *          Low limit
 *  @param  high
 *          High limit
 */
void apds9960SetIntLimits(apds9960_HandleTypedef* handle, uint16_t low, uint16_t high){
	apds9960Write8(handle->i2c, handle->addr, APDS9960_AILTIL, low & 0xFF);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_AILTH, low >> 8);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_AIHTL, high & 0xFF);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_AIHTH, high >> 8);
}

/*!
 *  @brief  Enables the device
 *          Disables the device (putting it in lower power sleep mode)
 *  @param 	handle Pointer to a apds9960_HandleTypedef structure that contains
 *    the configuration information for the APDS9960.
 *  @param  en
 *          Enable (True(1)/False(0))
 */
void apds9960Enable(apds9960_HandleTypedef* handle, bool en){
	handle->enable = (handle->enable & ~(1)) | (en);
	apds9960Write8(handle->i2c, handle->addr, APDS9960_ENABLE, handle->enable);
}
