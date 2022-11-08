/**
******************************************************************************
* @file    gatt_service.c
* @author  MCD Application Team
* @brief   My Very Own Service (Custom STM)
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
*                             www.st.com/SLA0044
*
******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "common_blesvc.h"
#include "ble_common.h"
#include "ble.h"
#include "gatt_service.h"
#include "main.h"
#include "app_common.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t WinkySensorServiceHandle;              /**< Service handle */
  uint16_t WinkySensorWriteCharacteristicHandle;  /**< Write Characteristic handle */
  uint16_t WinkySensorNotifyCharacteristicHandle; /**< Notify Characteristic handle */
  uint16_t WinkyOtaRebootReqCharHdle;                /**< Characteristic handle */
} WinkySensorServiceContext_t;

/* Private defines -----------------------------------------------------------*/
/* My Very Own Service and Characteristics UUIDs */
/*
The following 128bits UUIDs have been generated from the random UUID
generator:
D973F2E0-B19E-11E2-9E96-0800200C9A66: Service 128bits UUID
D973F2E1-B19E-11E2-9E96-0800200C9A66: Characteristic_1 128bits UUID
D973F2E2-B19E-11E2-9E96-0800200C9A66: Characteristic_2 128bits UUID
*/
#define COPY_WINKY_SERVICE_UUID(uuid_struct)                 COPY_UUID_128(uuid_struct,0x00,0x00,0xfe,0x40,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_WINKY_WRITE_CHARACTERISTIC_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0xfe,0x41,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_WINKY_NOTIFY_CHARACTERISTIC_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0xfe,0x42,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)

/** Max_Attribute_Records = 2*no_of_char + 1
  * service_max_attribute_record = 1 for My Very Own service +
  *                                2 for My Very Own Write characteristic +
  *                                2 for My Very Own Notify characteristic +
  *                                1 for client char configuration descriptor +
  *                                
  */
#define WINKY_SERVICE_MAX_ATT_RECORDS                8

#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              1
#define CHARACTERISTIC_NOTIFY_ATTRIBUTE_OFFSET             2

/* Private macros ------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
  uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
      uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
        uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Private variables ---------------------------------------------------------*/

static const uint8_t BM_REQ_CHAR_UUID[16] = {0x19, 0xed, 0x82, 0xae,
                                       0xed, 0x21, 0x4c, 0x9d,
                                       0x41, 0x45, 0x22, 0x8e,
                                       0x11, 0xFE, 0x00, 0x00};

/**
* START of Section BLE_DRIVER_CONTEXT
*/
PLACE_IN_SECTION("BLE_DRIVER_CONTEXT") static WinkySensorServiceContext_t WinkySensorServiceContext;

/**
* END of Section BLE_DRIVER_CONTEXT
*/
/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t WinkySensorService_EventHandler(void *pckt);

/* Functions Definition ------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
* @brief  Event handler
* @param  Event: Address of the buffer holding the Event
* @retval Ack: Return whether the Event has been managed or not
*/
static SVCCTL_EvtAckStatus_t WinkySensorService_EventHandler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blue_aci *blue_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);
  
  switch(event_pckt->evt)
  {
  case EVT_VENDOR:
    {
      blue_evt = (evt_blue_aci*)event_pckt->data;
      switch(blue_evt->ecode)
      {
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blue_evt->data;
  	  APP_DBG_MSG("attribute_modified\r\n");

        if(attribute_modified->Attr_Handle == (WinkySensorServiceContext.WinkySensorWriteCharacteristicHandle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
        {
        	WINKYAPP_Context.validationValue = attribute_modified->Attr_Data[0];
        }
        else if(attribute_modified->Attr_Handle == (WinkySensorServiceContext.WinkySensorNotifyCharacteristicHandle + CHARACTERISTIC_NOTIFY_ATTRIBUTE_OFFSET))
        {
        	/**
        	 * Descriptor handle
        	 */
        	return_value = SVCCTL_EvtAckFlowEnable;
        	/**
        	 * Notify to application
        	 */
        	if(attribute_modified->Attr_Data[0] & COMSVC_Notification)
        	{
        		APP_DBG_MSG("IMU NOTIFICATION ENABLE\r\n");
				#if (IMU_EXAMPLE != 0)
        		WINKYAPP_Context.IMUNotification_Status = 1;
				#endif
        	}
        	else
        	{
        		APP_DBG_MSG("IMU NOTIFICATION DISABLE\r\n");
				#if (IMU_EXAMPLE != 0)
        		WINKYAPP_Context.IMUNotification_Status = 0;
				#endif

        	}

        }
        else if(attribute_modified->Attr_Handle == (WinkySensorServiceContext.WinkyOtaRebootReqCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
        {
            *(uint32_t*)SRAM1_BASE = *(uint32_t*)attribute_modified->Attr_Data;
            NVIC_SystemReset();
        }
        break;
        
      default:
        break;
      }
    }
    break; /* HCI_EVT_VENDOR_SPECIFIC */
    
  default:
    break;
  }
  
  return(return_value);
}/* end SVCCTL_EvtAckStatus_t */

/* Public functions ----------------------------------------------------------*/

/**
* @brief  Service initialization
* @param  None
* @retval None
*/
void WinkySensorService_Init(void)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;
  Char_UUID_t  uuid16;
  
  /**
  *	Register the event handler to the BLE controller
  */
  SVCCTL_RegisterSvcHandler(WinkySensorService_EventHandler);
  
  /**
  *  Add My Very Own Service
  */
  COPY_WINKY_SERVICE_UUID(uuid16.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                       (Service_UUID_t *) &uuid16,
                       PRIMARY_SERVICE,
                       WINKY_SERVICE_MAX_ATT_RECORDS,
                       &(WinkySensorServiceContext.WinkySensorServiceHandle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    ERROR_HANDLER(); /* UNEXPECTED */
  }
  
  /**
  *  Add My Very Own Write Characteristic
  */
  COPY_WINKY_WRITE_CHARACTERISTIC_UUID(uuid16.Char_UUID_128);
  ret = aci_gatt_add_char(WinkySensorServiceContext.WinkySensorServiceHandle,
                    UUID_TYPE_128, &uuid16,
                    WINKY_WRITE_CHARACTERISTIC_VALUE_LENGTH,                                   
                    CHAR_PROP_WRITE_WITHOUT_RESP|CHAR_PROP_READ,
                    ATTR_PERMISSION_NONE,
                    GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
                    10, /* encryKeySize */
                    1, /* isVariable */
                    &(WinkySensorServiceContext.WinkySensorWriteCharacteristicHandle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    ERROR_HANDLER(); /* UNEXPECTED */
  }
  
  /**
  *   Add My Very Own Notify Characteristic
  */
  COPY_WINKY_NOTIFY_CHARACTERISTIC_UUID(uuid16.Char_UUID_128);
  ret = aci_gatt_add_char(WinkySensorServiceContext.WinkySensorServiceHandle,
                    UUID_TYPE_128, &uuid16,
                    WINKY_NOTIFY_CHARACTERISTIC_VALUE_LENGTH,
                    CHAR_PROP_NOTIFY,
                    ATTR_PERMISSION_NONE,
                    GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
                    10, /* encryKeySize */
                    1, /* isVariable: 1 */
                    &(WinkySensorServiceContext.WinkySensorNotifyCharacteristicHandle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    ERROR_HANDLER(); /* UNEXPECTED */
  }
  
  /**
   *  Add Boot Request Characteristic
   */
  ret = aci_gatt_add_char(WinkySensorServiceContext.WinkySensorServiceHandle,
		  UUID_TYPE_128,
		  (Char_UUID_t *)BM_REQ_CHAR_UUID,
		  WINKY_OTAREBOOT_CHARACTERISTIC_VALUE_LENGTH,
		  CHAR_PROP_WRITE_WITHOUT_RESP,
		  ATTR_PERMISSION_NONE,
		  GATT_NOTIFY_ATTRIBUTE_WRITE,
		  10,
		  0,
		  &(WinkySensorServiceContext.WinkyOtaRebootReqCharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
	  ERROR_HANDLER(); /* UNEXPECTED */
  }

  return;
} /* WinkySensorService_Init() */

/**
* @brief Characteristic update
* @param UUID: UUID of the characteristic
* @param newValueLength: Length of the new value data to be written
* @param pNewValue: Pointer to the new value data 
*/
tBleStatus WinkySensorWriteCharacteristic_Update(uint16_t UUID, uint16_t newValueLength, uint8_t *pNewValue)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  
  switch(UUID)
  {
  case WINKY_NOTIFY_CHARACTERISTIC_UUID:
    if (newValueLength <= WINKY_NOTIFY_CHARACTERISTIC_VALUE_LENGTH)
    {
      ret = aci_gatt_update_char_value(WinkySensorServiceContext.WinkySensorServiceHandle,
                                          WinkySensorServiceContext.WinkySensorNotifyCharacteristicHandle,
                                          0, /* charValOffset */
                                          newValueLength, /* charValueLen */
                                          (uint8_t *) pNewValue);
    }
    break;
  default:
    break;
  }
  
  return ret;
}/* end WinkySensorWriteCharacteristic_Update() */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
