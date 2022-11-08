/*
 * apds9960.h
 *
 *  Created on: 18 juin 2019
 *      Author: clement
 *  Port of Adafruit_APDS9960 library
 */

#ifndef APDS9960_APDS9960_H_
#define APDS9960_APDS9960_H_

#include "stm32wbxx_hal.h"

#include <stdbool.h>

#define APDS9960_ADDRESS (0x39) /**< I2C Address */



/** I2C Registers */
enum {
  APDS9960_RAM = 0x00,
  APDS9960_ENABLE = 0x80,
  APDS9960_ATIME = 0x81,
  APDS9960_WTIME = 0x83,
  APDS9960_AILTIL = 0x84,
  APDS9960_AILTH = 0x85,
  APDS9960_AIHTL = 0x86,
  APDS9960_AIHTH = 0x87,
  APDS9960_PILT = 0x89,
  APDS9960_PIHT = 0x8B,
  APDS9960_PERS = 0x8C,
  APDS9960_CONFIG1 = 0x8D,
  APDS9960_PPULSE = 0x8E,
  APDS9960_CONTROL = 0x8F,
  APDS9960_CONFIG2 = 0x90,
  APDS9960_ID = 0x92,
  APDS9960_STATUS = 0x93,
  APDS9960_CDATAL = 0x94,
  APDS9960_CDATAH = 0x95,
  APDS9960_RDATAL = 0x96,
  APDS9960_RDATAH = 0x97,
  APDS9960_GDATAL = 0x98,
  APDS9960_GDATAH = 0x99,
  APDS9960_BDATAL = 0x9A,
  APDS9960_BDATAH = 0x9B,
  APDS9960_PDATA = 0x9C,
  APDS9960_POFFSET_UR = 0x9D,
  APDS9960_POFFSET_DL = 0x9E,
  APDS9960_CONFIG3 = 0x9F,
  APDS9960_GPENTH = 0xA0,
  APDS9960_GEXTH = 0xA1,
  APDS9960_GCONF1 = 0xA2,
  APDS9960_GCONF2 = 0xA3,
  APDS9960_GOFFSET_U = 0xA4,
  APDS9960_GOFFSET_D = 0xA5,
  APDS9960_GOFFSET_L = 0xA7,
  APDS9960_GOFFSET_R = 0xA9,
  APDS9960_GPULSE = 0xA6,
  APDS9960_GCONF3 = 0xAA,
  APDS9960_GCONF4 = 0xAB,
  APDS9960_GFLVL = 0xAE,
  APDS9960_GSTATUS = 0xAF,
  APDS9960_IFORCE = 0xE4,
  APDS9960_PICLEAR = 0xE5,
  APDS9960_CICLEAR = 0xE6,
  APDS9960_AICLEAR = 0xE7,
  APDS9960_GFIFO_U = 0xFC,
  APDS9960_GFIFO_D = 0xFD,
  APDS9960_GFIFO_L = 0xFE,
  APDS9960_GFIFO_R = 0xFF,
};

enum {
	/* Bit fields */
	APDS9960_ENABLE_PON            = 0b00000001,
	APDS9960_ENABLE_AEN            = 0b00000010,
	APDS9960_ENABLE_PEN            = 0b00000100,
	APDS9960_ENABLE_WEN            = 0b00001000,
	APDS9960_ENABLE_AIEN           = 0b00010000,
	APDS9960_ENABLE_PIEN           = 0b00100000,
	APDS9960_ENABLE_GEN            = 0b01000000,
	APDS9960_ENABLE_GVALID 		   = 0b00000001,
};

/** ADC gain settings */
typedef enum {
  APDS9960_AGAIN_1X = 0x00,  /**< No gain */
  APDS9960_AGAIN_4X = 0x01,  /**< 2x gain */
  APDS9960_AGAIN_16X = 0x02, /**< 16x gain */
  APDS9960_AGAIN_64X = 0x03  /**< 64x gain */
} apds9960AGain_t;

/** Proxmity gain settings */
typedef enum {
  APDS9960_PGAIN_1X = 0x00, /**< 1x gain */
  APDS9960_PGAIN_2X = 0x04, /**< 2x gain */
  APDS9960_PGAIN_4X = 0x08, /**< 4x gain */
  APDS9960_PGAIN_8X = 0x0C  /**< 8x gain */
} apds9960PGain_t;

/** Pulse length settings */
typedef enum {
  APDS9960_PPULSELEN_4US = 0x00,  /**< 4uS */
  APDS9960_PPULSELEN_8US = 0x40,  /**< 8uS */
  APDS9960_PPULSELEN_16US = 0x80, /**< 16uS */
  APDS9960_PPULSELEN_32US = 0xC0  /**< 32uS */
} apds9960PPulseLen_t;

/** LED drive settings */
typedef enum {
  APDS9960_LEDDRIVE_100MA = 0x00, /**< 100mA */
  APDS9960_LEDDRIVE_50MA = 0x40,  /**< 50mA */
  APDS9960_LEDDRIVE_25MA = 0x80,  /**< 25mA */
  APDS9960_LEDDRIVE_12MA = 0xC0   /**< 12.5mA */
} apds9960LedDrive_t;

/** LED boost settings */
typedef enum {
  APDS9960_LEDBOOST_100PCNT = 0x00, /**< 100% */
  APDS9960_LEDBOOST_150PCNT = 0x10, /**< 150% */
  APDS9960_LEDBOOST_200PCNT = 0x20, /**< 200% */
  APDS9960_LEDBOOST_300PCNT = 0x30  /**< 300% */
} apds9960LedBoost_t;

/** Dimensions */
enum {
  APDS9960_DIMENSIONS_ALL = 0x00,        // All dimensions
  APDS9960_DIMENSIONS_UP_DOWN = 0x01,    // Up/Down dimensions
  APGS9960_DIMENSIONS_LEFT_RIGHT = 0x02, // Left/Right dimensions
};

/** FIFO Interrupts */
enum {
  APDS9960_GFIFO_1 = 0x00,  // Generate interrupt after 1 dataset in FIFO
  APDS9960_GFIFO_4 = 0x01,  // Generate interrupt after 2 datasets in FIFO
  APDS9960_GFIFO_8 = 0x02,  // Generate interrupt after 3 datasets in FIFO
  APDS9960_GFIFO_16 = 0x03, // Generate interrupt after 4 datasets in FIFO
};

/** Gesture Gain */
enum {
  APDS9960_GGAIN_1 = 0x00, // Gain 1x
  APDS9960_GGAIN_2 = 0x01, // Gain 2x
  APDS9960_GGAIN_4 = 0x02, // Gain 4x
  APDS9960_GGAIN_8 = 0x03, // Gain 8x
};

/** Pulse Lenghts */
enum {
  APDS9960_GPULSE_4US = 0x00,  // Pulse 4us
  APDS9960_GPULSE_8US = 0x01,  // Pulse 8us
  APDS9960_GPULSE_16US = 0x02, // Pulse 16us
  APDS9960_GPULSE_32US = 0x03, // Pulse 32us
};


/* Gesture wait time values */
enum {
	APDS9960_GWTIME_0MS            =  0,
	APDS9960_GWTIME_2_8MS           = 1,
	APDS9960_GWTIME_5_6MS           = 2,
	APDS9960_GWTIME_8_4MS           = 3,
	APDS9960_GWTIME_14_0MS          = 4,
	APDS9960_GWTIME_22_4MS          = 5,
	APDS9960_GWTIME_30_8MS          = 6,
	APDS9960_GWTIME_39_2MS           =7,
};


/* Geasture results */
enum {
	APDS9960_GESTURE_FAR            = 0x00,
	APDS9960_GESTURE_RIGHT          = 0x01,
	APDS9960_GESTURE_LEFT           = 0x02,
	APDS9960_GESTURE_UP             = 0x03,
	APDS9960_GESTURE_DOWN           = 0x04,
	APDS9960_GESTURE_NEAR           = 0x05,
};

/* Default values */
#define DEFAULT_ATIME           103     // 103ms
#define DEFAULT_WTIME           246     // 27ms
#define DEFAULT_PROX_PPULSE     0x87    // 16us, 8 pulses
#define DEFAULT_GESTURE_PPULSE  0x89    // 16us, 10 pulses
#define DEFAULT_POFFSET_UR      0       // 0 offset
#define DEFAULT_POFFSET_DL      0       // 0 offset
#define DEFAULT_CONFIG1         0x60    // No 12x wait (WTIME) factor
#define DEFAULT_LDRIVE          APDS9960_LEDDRIVE_100MA
#define DEFAULT_PGAIN           APDS9960_PGAIN_4X
#define DEFAULT_AGAIN           APDS9960_AGAIN_4X
#define DEFAULT_PILT            0       // Low proximity threshold
#define DEFAULT_PIHT            50      // High proximity threshold
#define DEFAULT_AILT            0xFFFF  // Force interrupt for calibration
#define DEFAULT_AIHT            0
#define DEFAULT_PERS            0x11    // 2 consecutive prox or ALS for int.
#define DEFAULT_CONFIG2         0x01    // No saturation interrupts or LED boost
#define DEFAULT_CONFIG3         0       // Enable all photodiodes, no SAI
#define DEFAULT_GPENTH          40      // Threshold for entering gesture mode
#define DEFAULT_GEXTH           30      // Threshold for exiting gesture mode
#define DEFAULT_GCONF1          APDS9960_GFIFO_4    // 4 gesture events for int., 1 for exit
#define DEFAULT_GGAIN           APDS9960_GGAIN_4
#define DEFAULT_GLDRIVE         APDS9960_LEDDRIVE_100MA
#define DEFAULT_GWTIME          APDS9960_GWTIME_2_8MS
#define DEFAULT_GOFFSET         0       // No offset scaling for gesture mode
#define DEFAULT_GPULSE          0xC9    // 32us, 10 pulses
#define DEFAULT_GCONF3          0       // All photodiodes active during gesture
#define DEFAULT_GIEN            0       // Disable gesture interrupts

typedef struct {
	uint16_t addr;

	I2C_HandleTypeDef* i2c;

	uint8_t enable; 	// (GEN << 6) | (PIEN << 5) | (AIEN << 4) | (WEN << 3) | (PEN << 2) | (AEN << 1) | PON
	uint8_t pers; 		//(PPERS << 4) | APERS
	uint8_t control; 	//(LDRIVE << 6) | (PGAIN << 2) | AGAIN
	uint8_t config1; 	// WLONG << 1
	uint8_t config2; 	// (PSIEN << 7) | (CPSIEN << 6) | (LED_BOOST << 4) | 1
	uint8_t config3;	// GDIMS
	uint8_t config4;
	uint8_t ppulse; 	//(PPLEN << 6) | PPULSE
	uint8_t status;     // CPSAT | PGSAT | PINT | AINT | reserver | GINT | PVALID | AVALID
	uint8_t gconf1;
	uint8_t gconf2;		// (GGAIN << 5) | (GLDRIVE << 3) | GWTIME
	uint8_t gconf3;		// GDIMS
	uint8_t gconf4;		//(GIEN << 1) | GMODE
	uint8_t gpulse;		//(GPLEN << 6) | GPULSE
	uint8_t gstatus;    // GFOV << 1 | GVALID

	uint8_t gestCnt;
	uint8_t UCount;
	uint8_t DCount;
	uint8_t LCount;
	uint8_t RCount;

	uint8_t  id;        /* Chip ID */
} apds9960_HandleTypedef;

HAL_StatusTypeDef apds9960Init(apds9960_HandleTypedef* handle);

void apds9960SetADCIntegrationTime(apds9960_HandleTypedef* handle, uint16_t iTimeMS);
float apds9960GetADCIntegrationTime(apds9960_HandleTypedef* handle);
void apds9960SetADCGain(apds9960_HandleTypedef* handle, apds9960AGain_t gain);
apds9960AGain_t apds9960GetADCGain(apds9960_HandleTypedef* handle);
void apds9960SetLED(apds9960_HandleTypedef* handle, apds9960LedDrive_t drive, apds9960LedBoost_t boost);

// proximity
void apds9960EnableProximity(apds9960_HandleTypedef* handle, bool en);
void apds9960SetProxGain(apds9960_HandleTypedef* handle, apds9960PGain_t gain);
apds9960PGain_t apds9960GetProxGain(apds9960_HandleTypedef* handle);
void apds9960SetProxPulse(apds9960_HandleTypedef* handle, apds9960PPulseLen_t pLen, uint8_t pulses);
void apds9960EnableProximityInterrupt(apds9960_HandleTypedef* handle);
void apds9960DisableProximityInterrupt(apds9960_HandleTypedef* handle);
uint8_t apds9960ReadProximity(apds9960_HandleTypedef* handle);
void apds9960SetProximityInterruptThreshold(apds9960_HandleTypedef* handle, uint8_t low, uint8_t high, uint8_t persistance);
bool apds9960GetProximityInterrupt(apds9960_HandleTypedef* handle);

// gesture
void apds9960EnableGesture(apds9960_HandleTypedef* handle, bool en);
bool apds9960GestureValid(apds9960_HandleTypedef* handle);
void apds9960SetGestureDimensions(apds9960_HandleTypedef* handle, uint8_t dims);
void apds9960SetGestureFIFOThreshold(apds9960_HandleTypedef* handle, uint8_t thresh);
void apds9960SetGestureGain(apds9960_HandleTypedef* handle, uint8_t gain);
void apds9960SetGestureLEDDrive(apds9960_HandleTypedef* handle, uint8_t drive);
void apds9960SetGestureWaitTime(apds9960_HandleTypedef* handle, uint8_t time);
void apds9960SetGestureProximityThreshold(apds9960_HandleTypedef* handle, uint8_t thresh);
void apds9960SetGestureExitThreshold(apds9960_HandleTypedef* handle, uint8_t thresh);
void apds9960SetGestureOffset(apds9960_HandleTypedef* handle, uint8_t offset_up, uint8_t offset_down,
                      uint8_t offset_left, uint8_t offset_right);
uint8_t apds9960ReadGesture(apds9960_HandleTypedef* handle);
void apds9960ResetCounts(apds9960_HandleTypedef* handle);

// light & color
void apds9960EnableColor(apds9960_HandleTypedef* handle, bool en);
bool apds9960ColorDataReady(apds9960_HandleTypedef* handle);
void apds9960GetColorData(apds9960_HandleTypedef* handle, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
uint16_t apds9960CalculateColorTemperature(uint16_t r, uint16_t g, uint16_t b);
uint16_t apds9960CalculateLux(uint16_t r, uint16_t g, uint16_t b);
void apds9960EnableColorInterrupt(apds9960_HandleTypedef* handle);
void apds9960DisableColorInterrupt(apds9960_HandleTypedef* handle);
void apds9960ClearInterrupt(apds9960_HandleTypedef* handle);
void apds9960SetIntLimits(apds9960_HandleTypedef* handle, uint16_t l, uint16_t h);

// turn on/off elements
void apds9960Enable(apds9960_HandleTypedef* handle, bool en);

#endif /* APDS9960_APDS9960_H_ */
