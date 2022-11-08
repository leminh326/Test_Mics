/*
 * eeprom.c
 *
 *  Created on: 14 oct. 2019
 *      Author: clement
 */

/* Includes ------------------------------------------------------------------*/
#include "eeprom.h"

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef STM32_SPI1_MemWrite( const uint8_t *const, uint16_t Size );
static HAL_StatusTypeDef STM32_SPI1_MemRead( uint8_t *pData, uint16_t Size );
static HAL_StatusTypeDef STM32_SPI1_Write( const uint8_t *const pTxData );
static void STM32_SPI1_MspInit( void );

EEPROMEX_StatusTypeDef EEPROMEX_SPI_MemWrite(const uint8_t * const pData, const uint16_t Size);
EEPROMEX_StatusTypeDef EEPROMEX_SPI_MemRead( uint8_t * const pData, const uint16_t Size);
EEPROMEX_StatusTypeDef EEPROMEX_SPI_Write(const uint8_t *const pTxData );
EEPROMEX_StatusTypeDef EEPROMEX_SPI_IsDeviceReady( const uint8_t DevAddr, const uint32_t Trials );


EEPROMEX_StatusTypeDef M95_IO_Init( void );
EEPROMEX_StatusTypeDef M95_IO_MemWrite(const uint8_t * const pData, const uint8_t DevAddr, const uint32_t TarAddr, const uint16_t Size, const uint8_t Inst );
EEPROMEX_StatusTypeDef M95_IO_MemRead(uint8_t * const pData, const uint32_t TarAddr, const uint8_t TargetName, const uint16_t Size, const uint8_t Inst );
EEPROMEX_StatusTypeDef M95_IO_WriteCmd( const uint8_t TxData, const uint8_t DevAddr);
EEPROMEX_StatusTypeDef M95_IO_Write( const uint8_t TxData);
EEPROMEX_StatusTypeDef M95_IO_WriteAddr( const uint32_t TarAddr, const uint8_t DevAddr);
EEPROMEX_StatusTypeDef M95_IO_IsDeviceReady( const uint8_t DevAddr, const uint32_t Trials );
EEPROMEX_StatusTypeDef M95_IO_ReadReg( const uint8_t * const pData, const uint8_t DevAddr );
EEPROMEX_StatusTypeDef M95_IO_WriteReg( const uint8_t Data, const uint8_t DevAddr );
EEPROMEX_StatusTypeDef M95_IO_WriteID( const uint8_t * const pData, const uint8_t DevAddr, const uint32_t TarAddr,const uint16_t Size );
EEPROMEX_StatusTypeDef M95_IO_ReadID( uint8_t * const pData, const uint32_t TarAddr, const uint8_t DevAddr, const uint16_t Size );

EEPROMEX_StatusTypeDef EEPROMEX_ConvertStatus( const HAL_StatusTypeDef status );

/* Functions Definition ------------------------------------------------------*/
/**
  * @brief  This function give high on selected control pin
  * @param  pin : selected control pin
  * @retval None
  */
void EEPROMEX_CTRL_HIGH( void ){
  HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port,EEPROM_CS_Pin,GPIO_PIN_SET);
}

/**
  * @brief  This function give low on selected control pin
  * @param  pin : selected control pin
  * @retval None
  */
void EEPROMEX_CTRL_LOW( void ){

  HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port,EEPROM_CS_Pin,GPIO_PIN_RESET);
}
/******************************** LINK EEPROM COMPONENT *****************************/


/****************************************** EEPROM SPI Driver *******************************************/

/**
  * @brief  Initializes peripherals used by the SPI EEPROMEX driver
  * @param  None
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_IO_Init( void )
{
	//Spi Configuration already one in peripheral config. nothing to do

	EEPROMEX_CTRL_HIGH();
	return EEPROMEX_OK;
}

EEPROMEX_StatusTypeDef M95_IO_DeInit( void )
{
  return EEPROMEX_OK;
}

/**
* @brief  Checks if target device is ready for communication
* @note   This function is used with Memory devices
* @param  DevAddr : Target device address
* @param  Trials : Number of trials
* @retval EEPROMEX enum status
*/
EEPROMEX_StatusTypeDef M95_IO_IsDeviceReady( const uint8_t DevAddr, const uint32_t Trials )
{
  uint8_t status = 0;
  EEPROMEX_CTRL_LOW();
  if ( M95_IO_Write( EEPROMEX_RDSR ) != EEPROMEX_OK )
    return EEPROMEX_ERROR;
  do {
    EEPROMEX_SPI_MemRead( &status, 1 );
  } while ( (status & EEPROMEX_SPI_WIPFLAG) == 1 );
  EEPROMEX_CTRL_HIGH();
  return EEPROMEX_OK;
}
/**
  * @brief  Reads register of target memory.
  * @param  pData: pointer to store register content.
  * @param  DevAddr : Target device address.
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_IO_ReadReg( const uint8_t * const pData, const uint8_t DevAddr ){
  EEPROMEX_CTRL_LOW();
  if ( M95_IO_Write( EEPROMEX_RDSR ) != EEPROMEX_OK )
    return EEPROMEX_ERROR;
  if ( EEPROMEX_SPI_MemRead( ( uint8_t * )pData, 1 ) != EEPROMEX_OK )
    return EEPROMEX_ERROR;
  EEPROMEX_CTRL_HIGH();
  return EEPROMEX_OK;
}

/**
  * @brief  Write register of target memory
  * @param  DevAddr : Target device address
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_IO_WriteReg( const uint8_t Data, const uint8_t DevAddr ){
  if ( M95_IO_WriteCmd( EEPROMEX_WREN, DevAddr  ) != EEPROMEX_OK )
    return EEPROMEX_ERROR;
  EEPROMEX_CTRL_LOW();
  if (M95_IO_Write( EEPROMEX_WRSR ) != EEPROMEX_OK )
    return EEPROMEX_ERROR;
  if (M95_IO_Write( Data ) != EEPROMEX_OK )
    return EEPROMEX_ERROR;
  EEPROMEX_CTRL_HIGH();
  if ( M95_IO_WriteCmd( EEPROMEX_WRDI, DevAddr ) != EEPROMEX_OK )
    return EEPROMEX_ERROR;
  M95_IO_IsDeviceReady( DevAddr, 1 );
  return EEPROMEX_OK;
}

/**
  * @brief  Reads data from the memory.
  * @param  pData: pointer to store read data
  * @param  DevAddr : Target device address
  * @param  TarAddr : SPI data memory address to read
  * @param  Size : Size in bytes of the value to be read
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_IO_MemRead( uint8_t * const pData, const uint32_t TarAddr,
                                       const uint8_t DevAddr, const uint16_t Size, const uint8_t Inst )
{
  EEPROMEX_CTRL_LOW();
  if (M95_IO_Write(Inst) != EEPROMEX_OK)
    return EEPROMEX_ERROR;
  if (M95_IO_WriteAddr(TarAddr, DevAddr) != EEPROMEX_OK )
    return EEPROMEX_ERROR;
  if (EEPROMEX_SPI_MemRead( pData, Size ) != EEPROMEX_OK )
    return EEPROMEX_ERROR;
  EEPROMEX_CTRL_HIGH();
  return EEPROMEX_OK;
}

/**
  * @brief  Write data, at specific address, through spi to the M95xx
  * @param  pData: pointer to the data to write
  * @param  DevAddr : Target device address
  * @param  TarAddr : I2C data memory address to write
  * @param  Size : Size in bytes of the value to be written
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_IO_MemWrite( const uint8_t * const pData, const uint8_t DevAddr,
                                        const uint32_t TarAddr,const uint16_t Size, const uint8_t Inst)
{
  if ( M95_IO_WriteCmd( EEPROMEX_WREN, DevAddr  ) != EEPROMEX_OK )
    return EEPROMEX_ERROR;
  EEPROMEX_CTRL_LOW();
  if ( M95_IO_Write(Inst) != EEPROMEX_OK )
    return EEPROMEX_ERROR;
  if ( M95_IO_WriteAddr(TarAddr, DevAddr) != EEPROMEX_OK )
    return EEPROMEX_ERROR;
  if ( EEPROMEX_SPI_MemWrite( pData, Size )!= EEPROMEX_OK )
     return EEPROMEX_ERROR;
  EEPROMEX_CTRL_HIGH();
  if ( M95_IO_WriteCmd( EEPROMEX_WRDI, DevAddr ) != EEPROMEX_OK )
    return EEPROMEX_ERROR;
  return EEPROMEX_OK;
}

/**
  * @brief  Write commands to target memory.
  * @param  Cmd : Command to be send
  * @param  DevAddr : Target device address
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_IO_WriteCmd( const uint8_t Cmd, const uint8_t DevAddr)
{
  EEPROMEX_StatusTypeDef status;
  EEPROMEX_CTRL_LOW();
  status = EEPROMEX_SPI_Write( &Cmd );
  EEPROMEX_CTRL_HIGH();
  return status;
}

/**
  * @brief  Write one byte to the memory .
  * @param  TxData: Byte to be send
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_IO_Write( const uint8_t TxData )
{
  EEPROMEX_StatusTypeDef status;
  status = EEPROMEX_SPI_Write( &TxData );
  return status;
}

/**
  * @brief  Send memory address to target memory
  * @param  TarAddr : SPI data memory address to read
  * @param  DevAddr : Target device address
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_IO_WriteAddr( const uint32_t TarAddr, const uint8_t DevAddr)
{
  uint8_t AddrSize;
  static uint8_t Addr[3];
  uint8_t count, temp;
  uint32_t addrpacket = TarAddr;
  AddrSize = ( DevAddr & EEPROMEX_SPI_ADDRSEL ) >> EEPROMEX_SPI_ADDRSHIFT;
  for (count = AddrSize; count > 0; count--){
    temp = (uint8_t)( addrpacket & 0xFF );
    Addr[count - 1] = temp;
    addrpacket = addrpacket >> 8;
  }
  return  EEPROMEX_SPI_MemWrite( Addr, AddrSize );
}


/******************************** LINK EEPROMEX *****************************/
/**
  * @brief  This functions converts HAL status to EEPROMEX status
  * @param  status : HAL status to convert
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef EEPROMEX_ConvertStatus( const HAL_StatusTypeDef status )
{
  switch( status )
  {
    case HAL_OK:
      return EEPROMEX_OK;
    case HAL_ERROR:
      return EEPROMEX_ERROR;
    case HAL_BUSY:
      return EEPROMEX_BUSY;
    case HAL_TIMEOUT:
      return EEPROMEX_TIMEOUT;

    default:
      return EEPROMEX_TIMEOUT;
  }
}

/*********************************** SPI LOW Level drivers *****************************************/


EEPROMEX_StatusTypeDef EEPROMEX_SPI_MemRead( uint8_t * const pData, const uint16_t Size )
{
  return EEPROMEX_ConvertStatus( STM32_SPI1_MemRead( pData, Size ) );
}

EEPROMEX_StatusTypeDef EEPROMEX_SPI_MemWrite( const uint8_t * const pData, const uint16_t Size )
{
  return EEPROMEX_ConvertStatus( STM32_SPI1_MemWrite( pData, Size ) );
}

EEPROMEX_StatusTypeDef EEPROMEX_SPI_Write(const uint8_t *const pTxData )
{
  return EEPROMEX_ConvertStatus( STM32_SPI1_Write(pTxData) );
}


/******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/


/*******************************SPI section start****************************/

static HAL_StatusTypeDef STM32_SPI1_MemRead(uint8_t *pData, uint16_t Size){
  return HAL_SPI_Receive(&hEEPROMEX_spi,pData,Size, EEPROMEX_SPI_TIMEOUT);
}

static HAL_StatusTypeDef STM32_SPI1_MemWrite( const uint8_t *const pData, uint16_t Size){
  uint8_t *pbuffer = (uint8_t *)pData;
  return HAL_SPI_Transmit(&hEEPROMEX_spi, pbuffer, Size, EEPROMEX_SPI_TIMEOUT);
}

static HAL_StatusTypeDef STM32_SPI1_Write( const uint8_t *const pTxData){
  return HAL_SPI_Transmit(&hEEPROMEX_spi,(uint8_t *)pTxData, 1, EEPROMEX_SPI_TIMEOUT);
}

/* SPI section end*/
