/*
 * tof.c
 *
 *  Created on: 6 sept. 2019
 *      Author: clement
 */


// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include "tof.h"
#include "vl53l0x_api.h"
// --------------------------------------------------------------------------------------------------------------------
// ----- local constant macros
// --------------------------------------------------------------------------------------------------------------------
#if TRACE_UART
#define debug_printf    trace_printf
#else
#define debug_printf(x, ...)    do {} while(0)
#endif

int LeakyFactorFix8 = (int)( 0.6 *256);
VL53L0X_Dev_t VL53L0XDev = {.Id=1, .DevLetter='t', .I2cHandle=&hi2c1, .I2cDevAddr=0x52};
/**
 * Global ranging struct
 */
VL53L0X_RangingMeasurementData_t RangingMeasurementData;
// --------------------------------------------------------------------------------------------------------------------
// ----- local function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// ----- local function prototypes
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local functions
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public functions
// --------------------------------------------------------------------------------------------------------------------

int DetectSensor() {
    uint16_t Id;
    int status;

    VL53L0X_Dev_t *pDev;
    pDev = &VL53L0XDev;
    pDev->I2cDevAddr = 0x52;
    pDev->Present = 0;
    HAL_Delay(2);

//    /* Set I2C standard mode (400 KHz) before doing the first register access */
//    if (status == VL53L0X_ERROR_NONE)
//    	status = VL53L0X_WrByte(pDev, 0x88, 0x00);

    /* Try to read one register using default 0x52 address */
    status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
    if (status) {
    	debug_printf("# Read id fail\n");
    	return 1;
    }
    if (Id == 0xEEAA) {

    	status = VL53L0X_DataInit(pDev);
    	if( status == 0 ){
    		pDev->Present = 1;
    	}
    	else{
    		debug_printf("VL53L0X_DataInit fail\n");
    		return 1;
    	}
    	pDev->Present = 1;
    }
    else {
    	debug_printf("unknown ID %x\n", Id);
    	status = 1;
    }
    /* if fail r can't use for any reason then put the  device back to reset */


    return status;
}

/**
 *  Setup all detected sensors for single shot mode and setup ranging configuration
 */
void SetupSingleShot(RangingConfig_e rangingConfig){
    int status;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
	uint8_t isApertureSpads;
	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;
	if( VL53L0XDev.Present){
		status=VL53L0X_StaticInit(&VL53L0XDev);
		if( status ){
			debug_printf("VL53L0X_StaticInit %d failed\n",i);
		}

		status = VL53L0X_PerformRefCalibration(&VL53L0XDev, &VhvSettings, &PhaseCal);
		if( status ){
			debug_printf("VL53L0X_PerformRefCalibration failed\n");
		}

		status = VL53L0X_PerformRefSpadManagement(&VL53L0XDev, &refSpadCount, &isApertureSpads);
		if( status ){
			debug_printf("VL53L0X_PerformRefSpadManagement failed\n");
		}

		status = VL53L0X_SetDeviceMode(&VL53L0XDev, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
		if( status ){
			debug_printf("VL53L0X_SetDeviceMode failed\n");
		}

		status = VL53L0X_SetLimitCheckEnable(&VL53L0XDev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
		if( status ){
			debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
		}

		status = VL53L0X_SetLimitCheckEnable(&VL53L0XDev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
		if( status ){
			debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
		}
		/* Ranging configuration */
		switch(rangingConfig) {
		case LONG_RANGE:
			signalLimit = (FixPoint1616_t)(0.1*65536);
			sigmaLimit = (FixPoint1616_t)(60*65536);
			timingBudget = 33000;
			preRangeVcselPeriod = 18;
			finalRangeVcselPeriod = 14;
			break;
		case HIGH_ACCURACY:
			signalLimit = (FixPoint1616_t)(0.25*65536);
			sigmaLimit = (FixPoint1616_t)(18*65536);
			timingBudget = 200000;
			preRangeVcselPeriod = 14;
			finalRangeVcselPeriod = 10;
			break;
		case HIGH_SPEED:
			signalLimit = (FixPoint1616_t)(0.25*65536);
			sigmaLimit = (FixPoint1616_t)(32*65536);
			timingBudget = 20000;
			preRangeVcselPeriod = 14;
			finalRangeVcselPeriod = 10;
			break;
		default:
			debug_printf("Not Supported");
		}

		status = VL53L0X_SetLimitCheckValue(&VL53L0XDev,  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
		if( status ){
			debug_printf("VL53L0X_SetLimitCheckValue failed\n");
		}

		status = VL53L0X_SetLimitCheckValue(&VL53L0XDev,  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
		if( status ){
			debug_printf("VL53L0X_SetLimitCheckValue failed\n");
		}

		status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDev,  timingBudget);
		if( status ){
			debug_printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed\n");
		}

		status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDev,  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
		if( status ){
			debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
		}

		status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDev,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
		if( status ){
			debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
		}

		status = VL53L0X_PerformRefCalibration(&VL53L0XDev, &VhvSettings, &PhaseCal);
		if( status ){
			debug_printf("VL53L0X_PerformRefCalibration failed\n");
		}

		VL53L0XDev.LeakyFirst=1;
	}
}
