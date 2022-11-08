/*
 * eeprom.h
 *
 *  Created on: 14 oct. 2019
 *      Author: clement
 */

#ifndef EEPROM_H_
#define EEPROM_H_

// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include "eeprom_common.h"
#include "peripheral_config.h"

#define EEPROMEX_SPI_TIMEOUT              200U

#define EEPROMEX_SPI_SLAVESEL             0x03U
#define EEPROMEX_SPI_ADDRSEL              0x0CU
#define EEPROMEX_SPI_ADDRSHIFT            0x02U
#define EEPROMEX_SPI_WIPFLAG              0x01U
#define EEPROMEX_SPI_WELFLAG              0x02U

#define hEEPROMEX_spi hspi1


void EEPROMEX_CTRL_HIGH(void);
void EEPROMEX_CTRL_LOW(void);
#endif /* EEPROM_H_ */
