/*
 * IQS5xx.c
 *
 *  Created on: 31 oct. 2019
 *      Author: clement
 */


// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include "IQS5xx.h"

#include "IQSxxx_I2C.h"
#include "IQS5xx_Init.h"
#include "stdio.h"
#include <string.h>
// --------------------------------------------------------------------------------------------------------------------
// ----- local constant macros
// --------------------------------------------------------------------------------------------------------------------
#define	END_WINDOW				(uint16_t)0xEEEE

// --------------------------------------------------------------------------------------------------------------------
// ----- local function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------

typedef struct {
  uint8_t ID;
  uint16_t Xpos;
  uint16_t Ypos;
  uint16_t TouchStrength;
  uint8_t Area;
}IQS5xx_TypeDef;


uint8_t NoOfFingers;
uint8_t XYInfoByte;
uint8_t prevXYInfoByte;
IQS5xx_TypeDef IQS5xx[5];
uint8_t nbButton = 0;
IQS5xx_ButtonTypeDef *IQS5xxButtons[5];
uint8_t nbTrackpad = 0;
IQS5xx_TrackpadTypeDef *IQS5xxTrackpads[5];
//uint16_t SnapStatus[TOTALTXS_VAL << 1];
// --------------------------------------------------------------------------------------------------------------------
// ----- local function prototypes
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local functions
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public functions
// --------------------------------------------------------------------------------------------------------------------

void IQS5xx_Settings(void)
{

}

void IQS5xx_EndCommWindows(void)
{
	  uint8_t ui8DataBuffer[1];
	  IQSxxx_I2C_Write(IQS5xx_I2C.Instance,IQS5xx_I2C_ADDR,END_WINDOW, REGISTER_ADDRESS_16, 1, &ui8DataBuffer[0], I2C_Start, I2C_Stop);
}

void IQS5xx_CheckVersion(void)
{
	uint8_t data_buffer[6];
	IQSxxx_I2C_Status_e status;
	status = IQSxxx_I2C_Read(IQS5xx_I2C.Instance,IQS5xx_I2C_ADDR,ProductNumber_adr, REGISTER_ADDRESS_16, 6,data_buffer,I2C_Start,I2C_Stop);
	if (status != I2C_OK)
	{
		printf("Failed to checked version. Status: %d\r\n", status);
	}
	else
	{
		printf("Product : %d Project : %d Version : %d.%d\r\n",(uint16_t) (data_buffer[0]<<8) + data_buffer[1],(uint16_t)(data_buffer[2]<<8) + data_buffer[3],data_buffer[4], data_buffer[5]);
	}
}

void IQS5xx_Refresh_Data(void)
{
	memset(IQS5xx, 0x0,sizeof(IQS5xx));

	uint8_t data_buffer[37], i;
	// Read the XY data,but only read as many XY co-ordinates as shown in the XYInfoByte:
	i = 0;
	IQSxxx_I2C_Read(IQS5xx_I2C.Instance,IQS5xx_I2C_ADDR,SystemInfo0_adr,REGISTER_ADDRESS_16, 2,data_buffer,I2C_Start,I2C_No_Stop);
	XYInfoByte = data_buffer[0];
	//printf("XYInfoByte : %u\r\n", XYInfoByte);
	//process XYInfoBYte data:
	if ((XYInfoByte & IQS5xx_SHOW_RESET) != 0)// check for an unexpected reset.
	{
		// IQS5xx must be configured with application specific settings, since after reset all will return to default
		//Acknowledge RESET
	    static  uint8_t System_ctrl_0 = IQS5xx_ACK_RESET;

	    IQSxxx_I2C_Write(IQS5xx_I2C.Instance,IQS5xx_I2C_ADDR,SystemControl0_adr, REGISTER_ADDRESS_16, 1, &System_ctrl_0,I2C_Start,I2C_Stop);
		//printf("RESEEEEETTTTT\r\n");
	    //return;
	}
	IQSxxx_I2C_Read(IQS5xx_I2C.Instance,IQS5xx_I2C_ADDR,NoOfFingers_adr,REGISTER_ADDRESS_16, 1,data_buffer,I2C_Start,I2C_No_Stop);
	NoOfFingers = data_buffer[0];
	//printf("NoOfFingers : %u\r\n", NoOfFingers);

	if ((NoOfFingers > 0) && (NoOfFingers <= 5) )
	{
		IQSxxx_I2C_Read(IQS5xx_I2C.Instance,IQS5xx_I2C_ADDR,AbsoluteX_adr,REGISTER_ADDRESS_16, 7*NoOfFingers,data_buffer,I2C_Start,I2C_No_Stop);
		// sort the received XY data in structure
		for (i = 0; i < 5; i++)
		{
			IQS5xx[i].ID = i;
			IQS5xx[i].Xpos = (int)(data_buffer[(i*7)])<<8;
			IQS5xx[i].Xpos |= data_buffer[(i*7)+1];
			IQS5xx[i].Ypos = (int)(data_buffer[(i*7)+2])<<8;
			IQS5xx[i].Ypos |= data_buffer[(i*7)+3];
			IQS5xx[i].TouchStrength = (int)(data_buffer[(i*7)+4])<<8;
			IQS5xx[i].TouchStrength |= data_buffer[(i*7)+5];
			IQS5xx[i].Area = data_buffer[(i*7)+6];
			//printf("Finger: %u X:%d Y:%d TS: %d A:%u\r\n",i,IQS5xx[i].Xpos, IQS5xx[i].Ypos,IQS5xx[i].TouchStrength, IQS5xx[i].Area);
		}
	}
	else
	{
		//Last read to close the
	}



	// Read Touch status

	//IQSxxx_I2C_Read(IQS5xx_I2C.Instance,IQS5xx_I2C_ADDR,AbsoluteX_adr,7*NoOfFingers,data_buffer,I2C_Start,I2C_No_Stop);

//	if ((XYInfoByte & SNAP_OUTPUT) != 0)// if there are active snap outputs
//		{
//		CommsIQS5xx_repeat_start();
//		CommsIQS5xx_Read(SNAP_STATUS, &data_buffer[0], (TOTALTXS_VAL<<1)); // 2 bytes per Tx for snap status
//		// sort the received SNAP data
//		for (i = 0; i < TOTALTXS_VAL; i++)
//			SnapStatus[i] = (((unsigned int)data_buffer[i<<1])<<8) + ((unsigned int)data_buffer[(i<<1) + 1]); // add the upper and lower bytes to get the full word
//		}
//	else
//	{
//		for (i = 0; i < TOTALTXS_VAL; i++)
//			SnapStatus[i] = 0x0000; // no snaps, so set registers all to zero
//	}
//	CommsIQS5xx_stop();
}

void IQS5xx_Process_New_Data (void)
{
	// Let's verify Button
	for (uint8_t j=0; j<nbButton; j++)
	{
		bool isPressed = false;
		for (uint8_t i=0; i<5; i++)
		{
			if ((IQS5xx[i].Xpos == IQS5xxButtons[j]->Xpos) && (IQS5xx[i].Ypos == IQS5xxButtons[j]->Ypos))
			{
				isPressed = true;
				IQS5xxButtons[j]->buttonState = buttonPressed;
				IQS5xxButtons[j]->callback(IQS5xxButtons[j]->buttonState);
				break;
			}
		}
		if (IQS5xxButtons[j]->buttonState == buttonPressed && isPressed == false)
		{
			IQS5xxButtons[j]->buttonState = buttonReleased;
			IQS5xxButtons[j]->callback(IQS5xxButtons[j]->buttonState);
		}
	}

	//TODO do trackpad processing with multi finger

	for (uint8_t j=0; j<nbTrackpad; j++)
	{
		bool isPressed = false;
		for (uint8_t i=0; i<5; i++)
		{
			if ((IQS5xx[i].Xpos >= IQS5xxTrackpads[j]->Xmin)
					&& (IQS5xx[i].Xpos <= IQS5xxTrackpads[j]->Xmax)
					&& (IQS5xx[i].Ypos >= IQS5xxTrackpads[j]->Ymin)
					&& (IQS5xx[i].Ypos <= IQS5xxTrackpads[j]->Ymax))
			{
				//isTouched = true;
				uint8_t x,y;
				x = IQS5xx[i].Xpos - IQS5xxTrackpads[j]->Xmin;
				y = IQS5xx[i].Ypos - IQS5xxTrackpads[j]->Ymin;
				IQS5xxTrackpads[j]->callback(x,y);
				break;
			}
		}
	}
}

void IQS5xx_EnableALP(void)
{
	uint8_t data_buffer[1];
	IQSxxx_I2C_Read(IQS5xx_I2C.Instance,IQS5xx_I2C_ADDR,ALPChannelSetup0_adr,REGISTER_ADDRESS_16,1,data_buffer,I2C_Start,I2C_No_Stop);
	data_buffer[0] |= IQS5xx_ALP_ENABLE;
    IQSxxx_I2C_Write(IQS5xx_I2C.Instance,IQS5xx_I2C_ADDR,ALPChannelSetup0_adr,REGISTER_ADDRESS_16, 1,data_buffer,I2C_Start,I2C_Stop);
}

void IQS5xx_DisableALP(void)
{
	uint8_t data_buffer[1];
	IQSxxx_I2C_Read(IQS5xx_I2C.Instance,IQS5xx_I2C_ADDR,ALPChannelSetup0_adr,REGISTER_ADDRESS_16,1,data_buffer,I2C_Start,I2C_No_Stop);
	data_buffer[0] &= !IQS5xx_ALP_ENABLE;
    IQSxxx_I2C_Write(IQS5xx_I2C.Instance,IQS5xx_I2C_ADDR,ALPChannelSetup0_adr,REGISTER_ADDRESS_16, 1,data_buffer,I2C_Start,I2C_Stop);
}

void IQS5xx_RegisterButton(IQS5xx_ButtonTypeDef* button)
{
	IQS5xxButtons[nbButton] = button;
	IQS5xxButtons[nbButton]->buttonState = buttonReleased;
	nbButton = nbButton + 1;
}

void IQS5xx_RegisterTrackpad(IQS5xx_TrackpadTypeDef* trackpad)
{
	IQS5xxTrackpads[nbTrackpad] = trackpad;
	nbTrackpad = nbTrackpad + 1;
}

void IQS5xx_UnregisterAll()
{
	memset(IQS5xxButtons, 0x0,sizeof(IQS5xxButtons));
	memset(IQS5xxTrackpads, 0x0,sizeof(IQS5xxTrackpads));
	nbButton = 0;
	nbTrackpad = 0;
}
