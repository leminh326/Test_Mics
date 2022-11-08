/*
 * wnk_eeprom.h
 *
 *  Created on: 14 oct. 2019
 *      Author: clement
 */

#ifndef WINKY_EEPROM_IMPL_H_
#define WINKY_EEPROM_IMPL_H_

/* Includes ------------------------------------------------------------------*/
#include "eeprom.h"

#define SPI_EEPROM


#define SPI_MEM_ADDR 0xC0U


EEPROMEX_StatusTypeDef WINKY_EEPROMEX_Init(void);
uint8_t WINKY_EEPROMEX_isInitialized(void);
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_IsDeviceReady(const uint32_t Trials);
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_ReadRegister(uint8_t * const pData);
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_WriteRegister(const uint8_t Cmd);
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_ReadByte(uint8_t * const pData, const uint32_t TarAddr);
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_WriteByte(const uint8_t * const pData, const uint32_t TarAddr);
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_ReadPage(uint8_t * const pData, const uint32_t TarAddr,const uint16_t Size);
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_WritePage(const uint8_t * const pData, const uint32_t TarAddr, const uint16_t Size);
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_ReadData(uint8_t * const pData, const uint32_t TarAdrr, const uint16_t Size);
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_WriteData(const uint8_t * const pData, const uint32_t TarAddr, const uint16_t Size);
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_ReadID(uint8_t * const pData, const uint32_t TarAddr, const uint16_t Size);
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_WriteID(const uint8_t * const pData, const uint32_t TarAddr, const uint16_t Size);
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_LockID(const uint8_t TargetName);
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_LockStatus(uint8_t * const pData);
EEPROMEX_StatusTypeDef WINKY_SPI_DeInit( void );

#endif /* WINKY_EEPROM_IMPL_H_ */
