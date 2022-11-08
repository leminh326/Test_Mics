/**
  ******************************************************************************
  * @file    eeprom_common.h
  * @author  MMY Application Team
  * @version V1.0.0
  * @date    17-August-2018
  * @brief   This file provides set of driver functions to manage communication 
  *          between MCU and M95xx M24xx memory chips 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_COMMON_H
#define __EEPROM_COMMON_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#if (defined USE_STM32L0XX_NUCLEO)
#include "stm32l0xx_hal.h"
#include "stm32l0xx_nucleo.h"
#include "stm32l0xx_hal_conf.h"
#include "stm32l0xx_hal_def.h"	 
#elif (defined USE_STM32F4XX_NUCLEO)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal_def.h"  
#endif
#include <stdint.h>

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
/** @addtogroup M24
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup M24_Exported_Types
  * @{
  */
/**
 * @brief  EEPROM status enumerator definition
 */
typedef enum
{
  EEPROMEX_OK      = 0,
  EEPROMEX_ERROR   = 1,
  EEPROMEX_BUSY    = 2,
  EEPROMEX_TIMEOUT = 3,
  EEPROMEX_ADDR_OVERFLOW = 4
} EEPROMEX_StatusTypeDef;

/**
 * @brief  M24xx Memory information structure definition
 */
typedef enum
{
  M24M01 = 0,
  M24256 = 1,
  M24C02 = 2,
  M24xx  = 4,
  
}M24xx_Member;

/**
 * @brief  M95xx Memory information structure definition
 */
typedef enum
{
  M95M01 = 5,
  M95256 = 6,
  M95040 = 7,
  M95xx  = 8
}M95xx_Member;

/**
 * @brief  EEPROMEX driver structure definition
 */
typedef struct
{
  EEPROMEX_StatusTypeDef       (*Init)( void );
  EEPROMEX_StatusTypeDef       (*DeInit)( void );
  EEPROMEX_StatusTypeDef       (*IsReady)( const uint32_t );
  EEPROMEX_StatusTypeDef       (*ReadRegister)( uint8_t * const, const uint8_t );
  EEPROMEX_StatusTypeDef       (*WriteRegister)( const uint8_t, const uint8_t );  
  EEPROMEX_StatusTypeDef       (*ReadByte)( uint8_t * const , const uint32_t , const uint16_t);
  EEPROMEX_StatusTypeDef       (*WriteByte)(const uint8_t * const , const uint32_t ,const uint16_t );
  EEPROMEX_StatusTypeDef       (*ReadPage)(uint8_t * const , const uint32_t , const uint16_t );
  EEPROMEX_StatusTypeDef       (*WritePage)( const uint8_t * const , const uint32_t ,const uint16_t ,const uint16_t ,const uint16_t );  
  EEPROMEX_StatusTypeDef       (*ReadData)( uint8_t * const, const uint32_t, const uint16_t, const uint16_t);
  EEPROMEX_StatusTypeDef       (*WriteData)( const uint8_t * const, const uint32_t, const uint16_t, const uint16_t,const uint16_t );
  EEPROMEX_StatusTypeDef       (*WriteID)( const uint8_t * const pData, const uint32_t TarAddr,const uint16_t PageSize,const uint16_t DeviceAddr, const uint16_t NbByte );
  EEPROMEX_StatusTypeDef       (*ReadID)( uint8_t * const pData, const uint32_t TarAddr,const uint16_t PageSize,const uint16_t DeviceAddr, const uint16_t NbByte );
  EEPROMEX_StatusTypeDef       (*LockID)( const uint16_t ); 
  EEPROMEX_StatusTypeDef       (*LockStatus)( uint8_t * const, const uint16_t );
  void                          *pData;
} EEPROMEX_DrvTypeDef;

/* Exported constants --------------------------------------------------------*/
/** @defgroup M24_Exported_Constants
  * @{
  */
#ifndef NULL
#define NULL      (void *) 0
#endif
#define MIN_TRIALS              1
#define LOCKID                  0x02   /* Byte to be send to lock ID page*/    
#define ADDRLID_16              0x0400
  
#define ADDRLID_SPI             0x0480    
#define LOCKDATA_SPI            0x02
    
#define BITCOUNT                0x01  /* Initial count value */
#define IDMASK                  0x04  /* Mask fith bit of I2C mem device address */
    
#define EEPROMEX_WREN           0x06  /*!< Write enable instruction */
#define EEPROMEX_WRDI           0x04  /*!< Write disable instruction */
#define EEPROMEX_RDSR           0x05  /*!< Read Status Register instruction  */
#define EEPROMEX_WRSR           0x01  /*!< Write Status Register instruction */
#define EEPROMEX_WRITE          0x02  /*!< Lower Half Write to Memory instruction */
#define EEPROMEX_UPWRITE        0x0A  /*!< Upper Half Write to Memory instruction */
#define EEPROMEX_READ           0x03  /*!< Lower Half Read from Memory instruction */
#define EEPROMEX_UPREAD         0x0B  /*!< Upper Half Read from Memory instruction */
#define EEPROMEX_RDID           0x83  /*!< Read identifiction instruction */
#define EEPROMEX_WRID           0x82  /*!< Write identifiction instruction */
#define EEPROMEX_RDLS           0x83  /*!< Read ID page lock status instruction */
#define EEPROMEX_LID            0x82  /*!< Lock ID page in read only mode instruction */
#define EEPROMEX_FullProtect    0x0C  /*!< Whole memory protect from write - Write BP1 BP2 bits as 11 in status register */
#define EEPROMEX_UHalfProtect   0x08  /*!< Upper Half Protect from write - Write BP1 BP2 bits as 10 in status register */
#define EEPROMEX_UQuarterProtect 0x04  /*!< Upper Quarter Protect from write - Write BP1 BP2 bits as 01 in status register */
#define EEPROMEX_UnProtect      0x00  /*!< Un Protect from read only mode - Write BP1 BP2 bits as 10 in status register */

/**
  * @}
  */
  
/* External variables --------------------------------------------------------*/

  extern uint16_t DeviceAddr;
  extern uint16_t PageSize;

/** @addtogroup M24_Imported_Variables
 * @{
 */
/* EEPROMEX driver structure */
extern EEPROMEX_DrvTypeDef M24_i2c_Drv;
extern EEPROMEX_DrvTypeDef M95_spi_Drv;
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Imported functions ------------------------------------------------------- */
/** @addtogroup M24xx_and_M95xx__Imported_Functions
 * @{
 */
extern EEPROMEX_StatusTypeDef M24_IO_Init( void );
extern EEPROMEX_StatusTypeDef M24_IO_MemWrite( const uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size );
extern EEPROMEX_StatusTypeDef M24_IO_MemRead( uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size );
extern EEPROMEX_StatusTypeDef M24_IO_IsDeviceReady( const uint8_t DevAddr, const uint32_t Trials );

extern EEPROMEX_StatusTypeDef M95_IO_Init( void );
extern EEPROMEX_StatusTypeDef M95_IO_DeInit( void );
extern EEPROMEX_StatusTypeDef M95_IO_MemWrite( const uint8_t * const pData,const uint8_t DevAddr, const uint32_t TarAddr,  const uint16_t Size, const uint8_t Inst );
extern EEPROMEX_StatusTypeDef M95_IO_MemRead(uint8_t * const pData, const uint32_t TarAddr, const uint8_t TargetName, const uint16_t Size, const uint8_t Inst  );
extern EEPROMEX_StatusTypeDef M95_IO_WriteCmd( const uint8_t TxData, const uint8_t DevAddr);
extern EEPROMEX_StatusTypeDef M95_IO_Write( const uint8_t TxData);
extern EEPROMEX_StatusTypeDef M95_IO_IsDeviceReady( const uint8_t DevAddr, const uint32_t Trials );
extern EEPROMEX_StatusTypeDef M95_IO_ReadReg( const uint8_t * const pData, const uint8_t DevAddr );
extern EEPROMEX_StatusTypeDef M95_IO_WriteReg( const uint8_t Data, const uint8_t DevAddr );
extern EEPROMEX_StatusTypeDef M95_IO_WriteID( const uint8_t * const pData, const uint8_t DevAddr, const uint32_t TarAddr,const uint16_t Size );
extern EEPROMEX_StatusTypeDef M95_IO_ReadID( uint8_t * const pData, const uint32_t TarAddr, const uint8_t DevAddr, const uint16_t Size );
extern EEPROMEX_StatusTypeDef M24_IO_Write( const uint8_t * const pData, const uint8_t DevAddr, const uint16_t Size );
extern EEPROMEX_StatusTypeDef M24_IO_Read( uint8_t * const pData, const uint8_t DevAddr, const uint16_t Size );

/**
 * @}
 */

/* Exported functions ------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */ 
  
/**
  * @}
  */

#ifdef __cplusplus
  }
#endif
#endif /* __EEPROM_COMMON_H */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
