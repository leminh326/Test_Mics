/*
 * iqs333.c
 *
 *  Created on: 23 oct. 2019
 *      Author: clement
 */


// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include "IQS333.h"
#include "IQS333_init.h"
#include "IQSxxx_I2C.h"
#include "peripheral_config.h"

#include "stm32wbxx_ll_i2c.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"
// --------------------------------------------------------------------------------------------------------------------
// ----- local constant macros
// --------------------------------------------------------------------------------------------------------------------
// #define DEBUG

#define WHEEL1_TOP_OFFSET				950				// UCL
#define WHEEL1_BOTTOM_OFFSET			200				// LCL
#define WHEEL1_CHANNELS					0x0E
#define WHEEL1_FLIPPED					false
#define WHEEL2_TOP_OFFSET				950				// UCL
#define WHEEL2_BOTTOM_OFFSET			200				// LCL
#define WHEEL2_CHANNELS					0x70
#define WHEEL2_FLIPPED					false
#define WHEEL_RESOLUTION				800
#define TOUCH_BUTTON					0x80			// CH7

// --------------------------------------------------------------------------------------------------------------------
// ----- local function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------
// Enum to show the loop state
typedef enum Loop_state {
	Init,
	Run,
	Read_Version
} Loop_state_e;

// Enum to init state
typedef enum Init_state {
	Channels,
	Setup,
	ReAti
} Init_state_e;

struct events {
	bool touch;		// A touch event occurred
	bool w1_event;  // A wheel1 event occurred
	bool w2_event;  // A wheel2 event occurred
};

// Which state of the loop are we in?
Loop_state_e Loop = Read_Version;
Init_state_e InitState = Channels;

IQS333_t iqs333;				// Create variable for iqs333

// Indicate chip is ready for polling
bool chipReady = false;

// Buffer to read data into
uint8_t buffer[20];

// Touch button
bool touch = false;
int8_t retries = 20;
// --------------------------------------------------------------------------------------------------------------------
// ----- local function prototypes
// --------------------------------------------------------------------------------------------------------------------
static bool IQS333_IsI2CAckFailed(uint32_t Timeout, uint32_t Tickstart);
IQSxxx_I2C_Status_e IQS333_Write(uint16_t regAddress,uint16_t size,const uint8_t *data,IQSxxx_I2C_Start_e start, IQSxxx_I2C_Stop_e stop);
IQSxxx_I2C_Status_e IQS333_Read(uint16_t regAddress,uint16_t size, uint8_t *data, IQSxxx_I2C_Start_e start,IQSxxx_I2C_Stop_e stop);
bool IQS333_WaitForDeviceReady(uint32_t timeout);
void process_IQS333_events(void);
void wheel1_processing(void);
void wheel2_processing(void);
void redo_ati(void);
uint8_t setup_iqs333(void);
// --------------------------------------------------------------------------------------------------------------------
// ----- local functions
// --------------------------------------------------------------------------------------------------------------------
static IQSxxx_I2C_Status_e IQS333_WaitOnI2cFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart)
{

	//printf("Waiton\r\n");
	//printf("Wait on Flag: %d I2C3->ISR: %lu Status: %d\r\n", Flag, I2Cx->ISR, Status);
	while ((((I2Cx->ISR & (Flag)) == (Flag)) ? SET : RESET)  == Status)
	{
		/* Check if a NACK is detected */
		//printf("1\r\n");
		if (IQS333_IsI2CAckFailed(Timeout, Tickstart) == true)
		{
			//printf("nack\r\n");
			return I2C_ERROR;
		}
		//printf("2\r\n");
		uint32_t now = HAL_GetTick();
		//printf("3\r\n");
		//printf("now: %d Tickstart %d diff %d\r\n", now,Tickstart, now-Tickstart);
		if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
		{
			//printf("timeout\r\n");
			return I2C_ERROR;
		}
	}
	//printf("Flag changed: %d I2Cx->ISR: %lu Status: %d\r\n", Flag, I2Cx->ISR, Status);

	return I2C_OK;
}

static bool IQS333_IsI2CAckFailed(uint32_t Timeout, uint32_t Tickstart)
{
	if (LL_I2C_IsActiveFlag_NACK(I2C3) == SET)
	{
		LL_I2C_GenerateStopCondition(I2C3);
		/* Wait until STOP Flag is reset */
		while (LL_I2C_IsActiveFlag_STOP(I2C3) == RESET)
		{
			if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
			{
				printf("timeout stop\r\n");
				return true;
			}
		}

		/* Clear NACKF Flag */
		LL_I2C_ClearFlag_NACK(I2C3);
//		/* Clear STOP Flag */
		LL_I2C_ClearFlag_STOP(I2C3);

//		/* Flush TX register */
//		/* If a pending TXIS flag is set */
//		/* Write a dummy data in TXDR to clear it */
//		if (LL_I2C_IsActiveFlag_TXIS(I2C3) != RESET)
//		{
//			printf("clear txis\r\n");
//			I2C3->TXDR = 0x00U;
//		}
//
//		/* Flush TX register if not empty */
//		if (LL_I2C_IsActiveFlag_TXE(I2C3) == RESET)
//		{
//			printf("clear txe\r\n");
//			LL_I2C_ClearFlag_TXE(I2C3);
//		}

		/* Clear Configuration Register 2 */
//		I2C3->CR2 |= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));
		//printf("ack failed\r\n");

		return true;
	}
	return false;
}

IQSxxx_I2C_Status_e IQS333_Write(uint16_t regAddress,uint16_t size,const uint8_t *data, IQSxxx_I2C_Start_e start, IQSxxx_I2C_Stop_e stop)
{
	return IQSxxx_I2C_Write(IQS333_I2C.Instance,IQS333_I2C_ADDRESS,regAddress, REGISTER_ADDRESS_8, size, data, start, stop);
}

IQSxxx_I2C_Status_e IQS333_Read(uint16_t regAddress,uint16_t size, uint8_t *data, IQSxxx_I2C_Start_e start,IQSxxx_I2C_Stop_e stop)
{
	return IQSxxx_I2C_Read(IQS333_I2C.Instance,IQS333_I2C_ADDRESS,regAddress, REGISTER_ADDRESS_8, size, data, start, stop);
}

bool IQS333_WaitForDeviceReady(uint32_t timeout)
{
	uint32_t Waitstart = HAL_GetTick();
	bool isDeviceReady = false;
	do{
		uint32_t Tickstart = HAL_GetTick();
		if ((Tickstart - Waitstart) > timeout)
		{
			break;
		}
		// send write start
		LL_I2C_ClearFlag_TXE(I2C3);
		LL_I2C_HandleTransfer(I2C3, IQS333_I2C_ADDRESS<<1, LL_I2C_ADDRESSING_MODE_7BIT, 0, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

		if (IQS333_WaitOnI2cFlagUntilTimeout(I2C3,I2C_ISR_TC,RESET,1,Tickstart) != I2C_OK)
		{
			//printf("read tC timeout\r\n");
			osDelay(1);
			continue;
		}
		isDeviceReady = true;
	}while (isDeviceReady != true);

	if ( isDeviceReady != true)
	{
		return false;
	}
	printf("IQS33_DeviceReady\r\n");

	return true;
//	printf("IQS333_WaitForDeviceReady start\r\n");
//	uint32_t tickstart,nackwait;
//	nackwait = 0;
//    tickstart = HAL_GetTick();
//	while (1) {
//		printf("start\r\n");
//
//		LL_I2C_SetSlaveAddr(I2C3,IQS333_I2C_ADDRESS<<1);
//		LL_I2C_DisableAutoEndMode(I2C3);
//		LL_I2C_SetTransferRequest(I2C3,LL_I2C_REQUEST_WRITE);
//		LL_I2C_SetTransferSize(I2C3,1);
//		LL_I2C_GenerateStartCondition(I2C3);
//	    tickstart = HAL_GetTick();
//		while (1) {
//			if ((LL_I2C_IsActiveFlag_BUSY(I2C3) == SET)){
//				//printf("IQS333_WaitForDeviceReady LL_I2C_IsActiveFlag_BUSY\r\n");
//				nackwait++;
//			}
//			if(IQS333_IsI2CAckFailed(20,tickstart))
//			{
//				printf("IQS333_WaitForDeviceReady LL_I2C_IsActiveFlag_NACK\r\n");
//				LL_I2C_ClearFlag_NACK(I2C3);
//				nackwait = 0;
//				break;
//			}
////			if (LL_I2C_IsActiveFlag_NACK(I2C3)) {
////				printf("IQS333_WaitForDeviceReady LL_I2C_IsActiveFlag_NACK\r\n");
////				LL_I2C_ClearFlag_NACK(I2C3);
////				nackwait = 0;
////				//Stop condition ?!
////				break;
////			}
//	        if (nackwait > 300)
//	        {
//				printf("IQS333_WaitForDeviceReady no NAck received\r\n");
//	        	return true;
//	        }
//		}
//        if (((HAL_GetTick() - tickstart) > timeout) || (timeout == 0U))
//        {
//			printf("IQS333_WaitForDeviceReady timeout\r\n");
//			LL_I2C_GenerateStopCondition(I2C3);
//			/* Wait until STOP Flag is reset */
//			while (LL_I2C_IsActiveFlag_STOP(I2C3) == RESET)
//			{
//				if (((HAL_GetTick() - tickstart) > 20))
//				{
//					printf("timeout stop\r\n");
//					return true;
//				}
//			}
//
//			/* Clear NACKF Flag */
//			LL_I2C_ClearFlag_NACK(I2C3);
//	//		/* Clear STOP Flag */
//			LL_I2C_ClearFlag_STOP(I2C3);
//
//        	return false;
//        }
//        osDelay(1);
//	}
//	return false;
}

/**
 * @brief	Process the Events that the IQS333 reported to us. Here we will run a state machine
 * 			to handle the different modes
 * @param	None
 * @retval	None
 */
void process_IQS333_events(void)
{
	struct events event;

	event.touch = false;
	event.w1_event = false;
	event.w2_event = false;

	// Check that no reset has occurred
	if(iqs333.systemFlags.Show_Reset)
	{
		// Something went wrong so we have to restart
		Loop = Read_Version;
		return;
	}

	// Check touch button
	if((iqs333.touch_byte.Touch_Byte_0 & TOUCH_BUTTON) == TOUCH_BUTTON)
	{
		touch = true;
		event.touch = true;
	}
	else if((iqs333.touch_byte.Touch_Byte_0 & TOUCH_BUTTON) != TOUCH_BUTTON
			&& touch == true)
	{
		touch = false;
		event.touch = true;
	}

	// Handle wheel 1 Coordinates
	if(iqs333.touch_byte.Touch_Byte_0 & WHEEL1_CHANNELS) {
		wheel1_processing();
		event.w1_event = true;
	}

	// Handle wheel 2 Coordinates
	if(iqs333.touch_byte.Touch_Byte_0 & WHEEL2_CHANNELS) {
		wheel2_processing();
		event.w2_event = true;
	}

	if(event.touch || event.w1_event || event.w2_event) {
		//formatJson(&event);
		//printf(jsonString);
		printf("EVENNNNT EVENTTTTTTTTTTTT\r\n");
		printf("====================================\r\n");
	}
}

/**
 * @brief	Process wheel 1's data to be from 0 to Max
 * @param	None
 * @retval	None
 */
void wheel1_processing(void)
{
	if(iqs333.wheel1.Wheel < WHEEL1_BOTTOM_OFFSET) {
		iqs333.wheel1.Wheel = WHEEL1_BOTTOM_OFFSET;
	}
	else if (iqs333.wheel1.Wheel > WHEEL1_TOP_OFFSET) {
		iqs333.wheel1.Wheel = WHEEL1_TOP_OFFSET;
	}
	// Now calculate the new wheel Coordinates
	iqs333.wheel1.Wheel = (uint16_t)((float)(iqs333.wheel1.Wheel - WHEEL1_BOTTOM_OFFSET)/(WHEEL1_TOP_OFFSET-WHEEL1_BOTTOM_OFFSET)*WHEEL_RESOLUTION);

	if(WHEEL1_FLIPPED) {
		iqs333.wheel1.Wheel = WHEEL_RESOLUTION - iqs333.wheel1.Wheel;
	}
}

/**
 * @brief	Process wheel 2's data to be from 0 to Max
 * @param	None
 * @retval	None
 */
void wheel2_processing(void)
{
	if(iqs333.wheel2.Wheel < WHEEL2_BOTTOM_OFFSET) {
		iqs333.wheel2.Wheel = WHEEL2_BOTTOM_OFFSET;
	}
	else if (iqs333.wheel2.Wheel > WHEEL2_TOP_OFFSET) {
		iqs333.wheel2.Wheel = WHEEL2_TOP_OFFSET;
	}
	// Now calculate the new wheel Coordinates
	iqs333.wheel2.Wheel = (uint16_t)((float)(iqs333.wheel2.Wheel - WHEEL2_BOTTOM_OFFSET)/(WHEEL2_TOP_OFFSET-WHEEL2_BOTTOM_OFFSET)*WHEEL_RESOLUTION);

	if(WHEEL2_FLIPPED) {
		iqs333.wheel2.Wheel = WHEEL_RESOLUTION - iqs333.wheel2.Wheel;
	}
}

/**
 * @brief	Check which setup we need to run for the IQS333
 * @param
 * @retval	None
 */
uint8_t setup_iqs333(void)
{
	uint8_t res = 1;

	switch(InitState)
	{
		case Channels:
			printf("Channels\r\n");
			// Switch to Prox mode if we have to
			res |= IQS333_Write(FLAGS_ADDR, 1, &System_flags[0], I2C_Repeat_Start, I2C_No_Stop);
			// Switch on all necessary Channels
			res |= IQS333_Write(ACTIVE_CHANNELS_ADDR, 2, &Active_Channels[0], I2C_Repeat_Start, I2C_Stop);
			if (res != I2C_OK) {
				printf("Error setting Active Channels\r\n");
				break;
			}
			printf("Channels1\r\n");
			// Run redo_ati
			redo_ati();
			printf("Channels2\r\n");
			// Switch to the actual setup
			InitState = Setup;

			break;

		case Setup:
			printf("Setup\r\n");
			// Write Multiplier settings
			res |= IQS333_Write(MULTIPLIERS_ADDR, sizeof(Multipliers), &Multipliers[0], I2C_Repeat_Start, I2C_No_Stop);
			// Write Proxsettings
			res |= IQS333_Write(PROXSETTINGS_ADDR, sizeof(Proxsettings), &Proxsettings[0], I2C_Repeat_Start, I2C_No_Stop);
			// Write Thresholds
			res |= IQS333_Write(THRESHOLDS_ADDR, sizeof(Thresholds), &Thresholds[0], I2C_Repeat_Start, I2C_No_Stop);
			// Write Timings
			res |= IQS333_Write(TIMINGS_ADDR, sizeof(Timings), &Timings[0], I2C_Repeat_Start, I2C_No_Stop);
			// Write ATI Targets
			res |= IQS333_Write(ATI_TARGETS_ADDR, sizeof(Ati_Targets), &Ati_Targets[0], I2C_Repeat_Start, I2C_No_Stop);
			// Write PWM Settings
			res |= IQS333_Write(PWM_ADDR, sizeof(Pwm_Settings), &Pwm_Settings[0], I2C_Repeat_Start, I2C_No_Stop);
			// Write PWM Limits
			res |= IQS333_Write(PWM_LIMIT_ADDR, sizeof(Pwm_Limits), &Pwm_Limits[0], I2C_Repeat_Start, I2C_No_Stop);
			// Write Buzzer Settings
			res |= IQS333_Write(BUZZER_ADDR, sizeof(Buzzer), &Buzzer[0], I2C_Repeat_Start, I2C_Stop);
			if ( res != I2C_OK) {
				printf("Error setting Active Channels\r\n");
				InitState = Channels;
				break;
			}
			// Go to re-ATI loop
			InitState = ReAti;
			break;

		case ReAti:
			printf("ReAti\r\n");
			// Redo ATI and switch to run mode
			redo_ati();

			// Switch on Event Mode
			//event_mode(false);

			InitState = Channels;
			Loop = Run;
			break;
		default:
			InitState = Channels;
	}

	return res;
}

/**
 * @brief	ATI Helper function for iqs333 - this is a blocking function
 */
void redo_ati(void)
{
	printf("redo_ati redo_ati\r\n");

	uint8_t res;

	// Fill buffer with first settings
	buffer[0] = REDO_ATI;
	// Wait for Redo Ati to complete
	while (1)
	{
		printf("REDO ATI xrite\r\n");
		if (!IQS333_WaitForDeviceReady(20))
		{

			continue;
		}
		printf("redo_ati IQS333_IsDeviceReady\r\n");

		res = IQS333_Write(PROXSETTINGS_ADDR, 1, buffer, LL_I2C_GENERATE_NOSTARTSTOP,I2C_Stop);
		printf("redo_ati PROXSETTINGS_ADDR res:%d\r\n", res);
		if (res)
		{
			break;
		}
	}

	// Wait for Redo Ati to complete
	while (1)
	{
		printf("REDO ATI loop\r\n");
		if (!IQS333_WaitForDeviceReady(20))
		{
			continue;
		}
		printf("loop IQS333_IsDeviceReady\r\n");
		//HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c3,IQS333_I2C_ADDRESS<<1,FLAGS_ADDR,I2C_MEMADD_SIZE_8BIT,&iqs333.systemFlags.SystemFlags,1,200);
		res = IQS333_Read(FLAGS_ADDR, 1, &iqs333.systemFlags.SystemFlags, LL_I2C_GENERATE_NOSTARTSTOP,I2C_Stop);
		printf("Redo_ATI res: %d %d\r\n",res,iqs333.systemFlags.SystemFlags);
		if ( res && (iqs333.systemFlags.ATI_Busy == 0))
		{
			break;
		}
	}
	printf("REDO ATI ENNND\r\n");
}


// --------------------------------------------------------------------------------------------------------------------
// ----- public functions
// --------------------------------------------------------------------------------------------------------------------

void IQS333_Process(void)
{
	uint8_t res = 0;
	int16_t timeout = 10000;

	// Read IQS device for information
	// Acquire all the necessary data
	if (IQS333_WaitForDeviceReady(20)){
		chipReady = true;
	}

	// handle the loop state
	switch(Loop)
	{
	case Init:
		if(chipReady){

			if(setup_iqs333() != I2C_OK)
			{
				printf("Error occurred in setup\r\n");
			}
			// We are done processing this comms window
			chipReady = false;
		}
		break;
	case Run:

		// Refresh IQS333 Data
		if(chipReady){
			// System flags - 1 Byte
			res = IQS333_Read(FLAGS_ADDR, 1, &iqs333.systemFlags.SystemFlags, I2C_Repeat_Start, I2C_No_Stop);
			// Read Wheel Coordinates - 4 bytes (2 bytes for each wheel)
			res |= IQS333_Read(WHEEL_COORDS_ADDR, 4, &iqs333.wheel1.Wheel_Low, I2C_Repeat_Start, I2C_No_Stop);
			// Read Touch Bytes - 2 bytes
			res |= IQS333_Read(TOUCH_BYTES_ADDR, 2, &iqs333.touch_byte.Touch_Byte_0, I2C_Repeat_Start, I2C_Stop);

			if (res != I2C_OK) {
				printf("Error reading from IQS333\r\n");
			}

			process_IQS333_events();

			// We are done processing this comms window
			chipReady = false;
		}

		break;

	case Read_Version:
		if(chipReady){
			printf("Readversion\r\n");
			// Check which IC we are using

			buffer[0] = Proxsettings[0];
			buffer[1] = Proxsettings[1];
			buffer[2] = Proxsettings[2] & (~EVENT_MODE);
			IQS333_Write(PROXSETTINGS_ADDR, 3, buffer, LL_I2C_GENERATE_NOSTARTSTOP, I2C_Stop);


			timeout = 10000;
			osDelay(10);
			IQS333_WaitForDeviceReady(20);
			printf("IQS333_IsDeviceReady\r\n");

			IQS333_Read(VERSION_INFO_ADDR, 2, buffer, LL_I2C_GENERATE_NOSTARTSTOP, I2C_Stop);
			printf("VERSION_INFO_ADDR\r\n");

			// Set the appropriate IC
			if(buffer[0] == PRODUCT_NUMBER && buffer[1] == VERSION_NUMBER){
				printf("IQS333 Detected...\r\n");
				Loop = Init;
				osDelay(10);
			}
			// No valid IC type found
			else {

				if (timeout <= 0) break;
				printf("Err invalid IC...\r\n");
				osDelay(500);
			}
		}

	break;

	default:
		// Wrong state, start from scratch
		Loop = Read_Version;
	}
}
