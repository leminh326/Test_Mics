/*
 * IQSxxx_I2C.c
 *
 *  Created on: 31 oct. 2019
 *      Author: clement
 */


// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include "IQSxxx_I2C.h"
#include "IQS5xx.h"

#include <inttypes.h>
#include <stdint.h>
// --------------------------------------------------------------------------------------------------------------------
// ----- local constant macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local function prototypes
// --------------------------------------------------------------------------------------------------------------------
static IQSxxx_I2C_Status_e IQSxxx_WaitOnI2cFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart);
static bool IQSxxx_IsI2CAckFailed(I2C_TypeDef *I2Cx, uint32_t Timeout, uint32_t Tickstart);
// --------------------------------------------------------------------------------------------------------------------
// ----- local functions
// --------------------------------------------------------------------------------------------------------------------
static IQSxxx_I2C_Status_e IQSxxx_WaitOnI2cFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart)
{

	//printf("Waiton\r\n");
	//printf("Wait on Flag: %d I2C3->ISR: %lu Status: %d\r\n", Flag, I2Cx->ISR, Status);
	while ((((I2Cx->ISR & (Flag)) == (Flag)) ? SET : RESET)  == Status)
	{
		/* Check if a NACK is detected */
		//printf("1\r\n");
		if (IQSxxx_IsI2CAckFailed( I2Cx, Timeout, Tickstart) == true)
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

static bool IQSxxx_IsI2CAckFailed(I2C_TypeDef *I2Cx, uint32_t Timeout, uint32_t Tickstart)
{
	if (LL_I2C_IsActiveFlag_NACK(I2Cx) == SET)
	{
		LL_I2C_GenerateStopCondition(I2Cx);
		/* Wait until STOP Flag is reset */
		while (LL_I2C_IsActiveFlag_STOP(I2Cx) == RESET)
		{
			if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
			{
				//printf("timeout stop\r\n");
				return true;
			}
		}

		/* Clear NACKF Flag */
		LL_I2C_ClearFlag_NACK(I2Cx);
//		/* Clear STOP Flag */
		LL_I2C_ClearFlag_STOP(I2Cx);

//		/* Flush TX register */
//		/* If a pending TXIS flag is set */
//		/* Write a dummy data in TXDR to clear it */
//		if (LL_I2C_IsActiveFlag_TXIS(I2C3) != RESET)
//		{
//			//printf("clear txis\r\n");
//			I2C3->TXDR = 0x00U;
//		}
//
//		/* Flush TX register if not empty */
//		if (LL_I2C_IsActiveFlag_TXE(I2C3) == RESET)
//		{
//			//printf("clear txe\r\n");
//			LL_I2C_ClearFlag_TXE(I2C3);
//		}

		/* Clear Configuration Register 2 */
//		I2C3->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));
		//printf("ack failed\r\n");

		return true;
	}
	return false;
}
// --------------------------------------------------------------------------------------------------------------------
// ----- public functions
// --------------------------------------------------------------------------------------------------------------------
IQSxxx_I2C_Status_e IQSxxx_I2C_Write(I2C_TypeDef *I2Cx,uint32_t slaveAddress, uint16_t regAddress, uint8_t regSize ,uint16_t size,const uint8_t *data, IQSxxx_I2C_Start_e start,IQSxxx_I2C_Stop_e stop)
{
	uint32_t request, endMode;
	switch (start)
	{
	case I2C_Repeat_Start:
		request = LL_I2C_GENERATE_START_WRITE;
		break;
	case I2C_Start:
		request = LL_I2C_GENERATE_START_WRITE;
		break;
	case I2C_No_Start:
		request = LL_I2C_GENERATE_NOSTARTSTOP;
		break;
	}

	switch (stop)
	{
	case I2C_No_Stop:
		endMode = LL_I2C_MODE_SOFTEND;
		break;
	case I2C_Stop:
		endMode = LL_I2C_MODE_AUTOEND;
		break;
	}
	//printf("IQS333_Write %02X : %d\r\n",regAddress, size);
	uint32_t Tickstart = HAL_GetTick();
    // send write start
	LL_I2C_ClearFlag_TXE(I2Cx);

    LL_I2C_HandleTransfer(I2Cx, slaveAddress<<1, LL_I2C_ADDRESSING_MODE_7BIT, regSize, LL_I2C_MODE_SOFTEND, request);
    if (IQSxxx_WaitOnI2cFlagUntilTimeout(I2Cx,I2C_ISR_TXE,RESET,5,Tickstart) != I2C_OK)
    {
    	return I2C_ERROR;
    }
	LL_I2C_ClearFlag_TXE(I2Cx);
	if (regSize == REGISTER_ADDRESS_16)
	{
		LL_I2C_TransmitData8(I2Cx, regAddress>>8);
		//printf("LL_I2C_TransmitData8\r\n");

		if (IQSxxx_WaitOnI2cFlagUntilTimeout(I2Cx,I2C_ISR_TXE,RESET,5,Tickstart) != I2C_OK)
		{
			return I2C_ERROR;
		}
		LL_I2C_ClearFlag_TXE(I2Cx);
	}
    LL_I2C_TransmitData8(I2Cx, regAddress);
    //printf("LL_I2C_TransmitData8\r\n");

    if (IQSxxx_WaitOnI2cFlagUntilTimeout(I2Cx,I2C_ISR_TC,RESET,5,Tickstart) != I2C_OK)
    {
    	return I2C_ERROR;
    }
    //printf("LL_I2C_IsActiveFlag_TC\r\n");

    LL_I2C_HandleTransfer(I2Cx, slaveAddress<<1, LL_I2C_ADDRESSING_MODE_7BIT, size , endMode, LL_I2C_GENERATE_START_WRITE);

    //printf("LL_I2C_HandleTransfer\r\n");

    int32_t i;
    for (i=0; i<size; i++){
        if (IQSxxx_WaitOnI2cFlagUntilTimeout(I2Cx,I2C_ISR_TXE,RESET,5,Tickstart) != I2C_OK)
        {
        	return I2C_ERROR;
        }
    	LL_I2C_ClearFlag_TXE(I2Cx);

    	LL_I2C_TransmitData8(I2Cx, data[i]);
        //printf("LL_I2C_TransmitData8\r\n");
    }

    if (stop == I2C_Stop){
        if (IQSxxx_WaitOnI2cFlagUntilTimeout(I2Cx,I2C_ISR_STOPF,RESET,5,Tickstart) != I2C_OK)
        {
        	return I2C_ERROR;
        }
		LL_I2C_ClearFlag_STOP(I2Cx);
		//printf("LL_I2C_ClearFlag_STOP\r\n");
    }
    else
    {
    	if (IQSxxx_WaitOnI2cFlagUntilTimeout(I2Cx,I2C_ISR_TC,RESET,5,Tickstart) != I2C_OK)
    	{
    		return I2C_ERROR;
    	}
    	//printf("LL_I2C_IsActiveFlag_TC\r\n");
    }

	//printf("Write Done\r\n");

	return I2C_OK;
}


IQSxxx_I2C_Status_e IQSxxx_I2C_Read(I2C_TypeDef *I2Cx,uint32_t slaveAddress, uint16_t regAddress, uint8_t regSize, uint16_t size,uint8_t *data, IQSxxx_I2C_Start_e start,IQSxxx_I2C_Stop_e stop)
{
	//printf("IQS333_Read %02X : %d\r\n",regAddress, size);

	uint32_t request, endMode;
	switch (start)
	{
	case I2C_Repeat_Start:
		request = LL_I2C_GENERATE_START_WRITE;
		break;
	case I2C_Start:
		request = LL_I2C_GENERATE_START_WRITE;
		break;
	case I2C_No_Start:
		request = LL_I2C_GENERATE_NOSTARTSTOP;
		break;
	}

	switch (stop)
	{
	case I2C_No_Stop:
		endMode = LL_I2C_MODE_SOFTEND;
		break;
	case I2C_Stop:
		endMode = LL_I2C_MODE_AUTOEND;
		break;
	}

//	uint32_t 	check = LL_I2C_IsEnabledIT_TC(I2Cx);
//	//printf("TC Interrupt enabled %" PRIu32 "\r\n",check);
//	check = LL_I2C_IsEnabledIT_STOP(I2Cx);
//	//printf("STOP Interrupt enabled %" PRIu32 "\r\n",check);
//	check = LL_I2C_IsEnabledIT_TX(I2Cx);
//	//printf("TXIS Interrupt enabled %" PRIu32 "\r\n",check);
//	check = LL_I2C_IsEnabledIT_RX(I2Cx);
//	//printf("RXIE Interrupt enabled %" PRIu32 "\r\n",check);
//	LL_I2C_EnableIT_TX(I2Cx);
//	LL_I2C_EnableIT_RX(I2Cx);
//	LL_I2C_EnableIT_TC(I2Cx);
//	LL_I2C_EnableIT_NACK(I2Cx);
//	LL_I2C_EnableIT_STOP(I2Cx);
//	check = LL_I2C_IsEnabledIT_TC(I2Cx);
//	//printf("TC Interrupt enabled %" PRIu32 "\r\n",check);
//	check = LL_I2C_IsEnabledIT_STOP(I2Cx);
//	//printf("STOP Interrupt enabled %" PRIu32 "\r\n",check);
//	check = LL_I2C_IsEnabledIT_TX(I2Cx);
//	//printf("TXIS Interrupt enabled %" PRIu32 "\r\n",check);
//	check = LL_I2C_IsEnabledIT_RX(I2Cx);
//	//printf("RXIE Interrupt enabled %" PRIu32 "\r\n",check);

	uint32_t Tickstart = HAL_GetTick();
	// send write start
	LL_I2C_ClearFlag_TXE(I2Cx);
	LL_I2C_HandleTransfer(I2Cx, slaveAddress<<1, LL_I2C_ADDRESSING_MODE_7BIT, regSize, LL_I2C_MODE_SOFTEND, request);
	if (IQSxxx_WaitOnI2cFlagUntilTimeout(I2Cx,I2C_ISR_TXE,RESET,5,Tickstart) != I2C_OK)
	{
		//printf("read txe timeout\r\n");

		return I2C_ERROR;
	}
	//printf("TXE SET\r\n");
	LL_I2C_ClearFlag_TXE(I2Cx);
	//printf("TXE Clear\r\n");
	if (regSize == REGISTER_ADDRESS_16)
	{
		LL_I2C_TransmitData8(I2Cx, regAddress>>8);
		//printf("LL_I2C_TransmitData8\r\n");

		if (IQSxxx_WaitOnI2cFlagUntilTimeout(I2Cx,I2C_ISR_TXE,RESET,5,Tickstart) != I2C_OK)
		{
			return I2C_ERROR;
		}
		LL_I2C_ClearFlag_TXE(I2Cx);
	}
    LL_I2C_TransmitData8(I2Cx, regAddress);
    //printf("LL_I2C_TransmitData8\r\n");

	if (IQSxxx_WaitOnI2cFlagUntilTimeout(I2Cx,I2C_ISR_TC,RESET,5,Tickstart) != I2C_OK)
	{
		//printf("read tC timeout\r\n");

		return I2C_ERROR;
	}
	//printf("LL_I2C_IsActiveFlag_TC\r\n");
	//
	//	//printf("LL_I2C_IsActiveFlag_TXE\r\n");

	// Send read start condition
	LL_I2C_HandleTransfer(I2Cx, slaveAddress<<1, LL_I2C_ADDRESSING_MODE_7BIT, size, endMode, LL_I2C_GENERATE_START_READ);
	//printf("LL_I2C_HandleTransfer\r\n");

	int32_t i;
	for (i=0; i<size; i++){
		////printf("for loop\r\n");
		if (IQSxxx_WaitOnI2cFlagUntilTimeout(I2Cx,I2C_ISR_RXNE,RESET,5,Tickstart) != I2C_OK )
		{
			return I2C_ERROR;
		}
		//printf("LL_I2C_IsActiveFlag_RXNE\r\n");
		data[i] = LL_I2C_ReceiveData8(I2Cx);
		////printf("LL_I2C_ReceiveData8\r\n");
	}

	if (stop == I2C_Stop){
		if (IQSxxx_WaitOnI2cFlagUntilTimeout(I2Cx,I2C_ISR_STOPF,RESET,5,Tickstart) != I2C_OK)
		{
			return I2C_ERROR;
		}
		LL_I2C_ClearFlag_STOP(I2Cx);
		//printf("LL_I2C_ClearFlag_STOP\r\n");
//		if (!IQSxxx_WaitOnI2cFlagUntilTimeout(I2Cx,I2C_ISR_BUSY,SET,5,Tickstart))
//		{
//			return false;
//		}
		////printf("LL_I2C_IsActiveFlag_BUSY\r\n");
	}
	else
	{
		if (IQSxxx_WaitOnI2cFlagUntilTimeout(I2Cx,I2C_ISR_TC,RESET,5,Tickstart) != I2C_OK)
		{
			return I2C_ERROR;
		}
		//printf("LL_I2C_IsActiveFlag_TC\r\n");
	}
	//printf("Read Done\r\n");

	return I2C_OK;
}

//void CommsIQS5xx_Write(unsigned char write_addr, unsigned char *data, unsigned char NoOfBytes)
//{
//	unsigned char i;
//	CommsIQS5xx_send((IQS5xx_ADDR << 1) + 0x00);// device address + write
//	CommsIQS5xx_send(write_addr);// IQS5xx address-command
//	for (i = 0 ; i < NoOfBytes ; i++)
//		CommsIQS5xx_send(data[i]);
//}
//
//void CommsIQS5xx_Read(unsigned char read_addr, unsigned char *data, unsigned char NoOfBytes)
//{
//	unsigned char i;
//	CommsIQS5xx_send((IQS5xx_ADDR << 1) + 0x00);// device address + write
//	CommsIQS5xx_send(read_addr);// IQS5xx address-command
//	CommsIQS5xx_repeat_start();
//	CommsIQS5xx_send((IQS5xx_ADDR << 1) + 0x01);// device address + read
//
//	if (NoOfBytes > 1)
//	{
//		for (i = 0; i < NoOfBytes -1; i++)
//			data[i] = CommsIQS5xx_read_ack(); // all bytes except last must be followed by an ACK
//	}
//	data[NoOfBytes-1] = CommsIQS5xx_read_nack();// last byte read must be followed by a NACK
//}
//
//unsigned char CommsIQS5xx_Read_First_Byte(unsigned char start_addr)
//{
//	CommsIQS5xx_send((IQS5xx_ADDR << 1) + 0x00);
//	CommsIQS5xx_send(start_addr);
//	CommsIQS5xx_repeat_start();
//	CommsIQS5xx_send((IQS5xx_ADDR << 1) + 0x01);
//	return CommsIQS5xx_read_ack();
//}
//
//unsigned char CommsIQS5xx_Read_Next_Cont(void)
//{
//	return CommsIQS5xx_read_ack();
//}
//
//unsigned char CommsIQS5xx_Read_Next_Done(void)
//{
//	return CommsIQS5xx_read_nack();
//}
//
//void CommsIQS5xx_Initiate_Conversion(void)
//{
//	CommsIQS5xx_start();
//	CommsIQS5xx_send((IQS5xx_ADDR << 1) + 0x00);
//	CommsIQS5xx_stop();
//}
