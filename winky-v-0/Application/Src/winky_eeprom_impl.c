/*
 * eeprom_impl.c
 *
 *  Created on: 14 oct. 2019
 *      Author: clement
 */


/* Includes ------------------------------------------------------------------*/
#include "winky_eeprom_impl.h"


  uint16_t DeviceAddr = 0xC7;
  uint16_t PageSize   = 16;
  uint64_t MemorySize = 256; /*in bytes*/
  int Full_Write = 0;

static EEPROMEX_DrvTypeDef *EEPROMEX_SPI_Drv = NULL;
static uint8_t EEPromEXInitialized_SPI = 0;

/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/
/**
  * @brief  Initializes peripherals used by the eeprom expansion driver
  * @param  None
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_Init( void )
{
	if (!EEPromEXInitialized_SPI)
	{
		if (M95_spi_Drv.Init == NULL)
		{
			return EEPROMEX_ERROR;
		}
		if (M95_spi_Drv.Init() == EEPROMEX_OK)
		{
			EEPROMEX_SPI_Drv = &M95_spi_Drv;
		}
		else
			return EEPROMEX_ERROR;
	}
	return EEPROMEX_OK;
}

/**
  * @brief  Checks if the memory is available
  * @param  Trials : Number of trials
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_IsDeviceReady( const uint32_t Trials )
{
   if (EEPROMEX_SPI_Drv->IsReady == NULL)
  {
    return EEPROMEX_ERROR;
  }
 return EEPROMEX_SPI_Drv->IsReady( Trials );
}

/**
  * @brief  Reads register content of spi memory
  * @param  pData   : pointer to the data to read
  * @param  TargetName : memory name to read
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_ReadRegister( uint8_t * const pData )
{

  if (((uint8_t)DeviceAddr & 0xF0) == SPI_MEM_ADDR){
   if ( EEPROMEX_SPI_Drv->ReadRegister == NULL )
   {
     return EEPROMEX_ERROR;
   }
   return EEPROMEX_SPI_Drv->ReadRegister( pData, DeviceAddr );
  }
  else
    return EEPROMEX_ERROR;
}

/**
  * @brief  Write register of spi memory
  * @param  Cmd     : instruction to write on memory
  * @param  TargetName : memory name to write
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_WriteRegister( const uint8_t Cmd )
{

  if (((uint8_t)DeviceAddr & 0xF0) == SPI_MEM_ADDR)
  {
   if (EEPROMEX_SPI_Drv->WriteRegister == NULL)
   {
     return EEPROMEX_ERROR;
   }
   return EEPROMEX_SPI_Drv->WriteRegister(Cmd, DeviceAddr);
  }
  else
    return EEPROMEX_ERROR;
}

/**
  * @brief  Reads data in identification page of the memory at specific address
  * @param  pData   : pointer to the data to read
  * @param  TarAddr : memory address to read
  * @param  TargetName : memory name to read
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_ReadByte(uint8_t * const pData, const uint32_t TarAddr){

	if ((TarAddr + 1)>MemorySize)
		return EEPROMEX_ADDR_OVERFLOW;

	if (((uint8_t)DeviceAddr & 0xF0) == SPI_MEM_ADDR )
	{
		if (EEPROMEX_SPI_Drv->ReadByte == NULL)
		{
			return EEPROMEX_ERROR;
		}
		return EEPROMEX_SPI_Drv->ReadByte( pData, TarAddr, DeviceAddr);
	}
	else
		return EEPROMEX_ERROR;
}

/**
  * @brief  Reads complete page from the memory at page start address
  * @param  pData   : pointer to the data to read
  * @param  TarAddr : starting page address to read
  * @param  TargetName : memory name to read
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_ReadPage(uint8_t * const pData, const uint32_t TarAddr,const uint16_t Size )
{

	if ((TarAddr + Size)>MemorySize)
		return EEPROMEX_ADDR_OVERFLOW;


	EEPROMEX_StatusTypeDef returnedStatus;

	if (((uint8_t)DeviceAddr & 0xF0) == SPI_MEM_ADDR)
	{
		if (EEPROMEX_SPI_Drv->ReadPage == NULL)
		{
			returnedStatus = EEPROMEX_ERROR;
		}

		uint32_t iNumberOfPage = (TarAddr + Size) / PageSize;
		uint32_t iRemainder = (TarAddr + Size) % PageSize;
		uint32_t PageAddress = TarAddr * PageSize;
		uint32_t iPageNumber = TarAddr;

		if (iRemainder!=0)
		{
			iNumberOfPage+=1;
		}
		if (iNumberOfPage<=1)
		{
			returnedStatus = EEPROMEX_SPI_Drv->ReadPage( pData, PageAddress,PageSize);
		}
		else
		{
			for (uint32_t iCounter=0; iCounter<iNumberOfPage; iCounter++)
			{
				uint32_t iPageAddress = iPageNumber * PageSize;
				returnedStatus = EEPROMEX_SPI_Drv->ReadPage( &pData[0+iCounter*PageSize], iPageAddress,PageSize);
				iPageNumber++;
				osDelay(5);
			}
		}
	}
  else
    returnedStatus = EEPROMEX_ERROR;

  return returnedStatus;
}

/**
  * @brief  Writes data in the memory at specific address
  * @param  pData : pointer to the data to write
  * @param  TarAddr : I2C data memory address to write
  * @param  TargetName : memory name to write
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_WriteByte(const uint8_t * const pData, const uint32_t TarAddr)
{
  if ((TarAddr + 1)>MemorySize)
      return EEPROMEX_ADDR_OVERFLOW;

if (((uint8_t)DeviceAddr & 0xF0) == SPI_MEM_ADDR){
   EEPROMEX_StatusTypeDef ret_value;
   if ( EEPROMEX_SPI_Drv->WriteByte == NULL )
   {
     return EEPROMEX_ERROR;
   }
   ret_value = EEPROMEX_SPI_Drv->WriteByte( pData, TarAddr,DeviceAddr);
    if ( ret_value == EEPROMEX_OK )
    {
       while ( WINKY_EEPROMEX_IsDeviceReady(MIN_TRIALS) != EEPROMEX_OK ) {};
       return EEPROMEX_OK;
    }
   return ret_value;
  }
  else
    return EEPROMEX_ERROR;

}
/**
  * @brief  Writes complete page in the memory at starting page address
  * @param  pData : pointer to the data to write
  * @param  TarAddr : I2C/SPI data memory starting page address(page no.) to write
  * @param  TargetName : memory name to write
  * @param  Size : Size in bytes of the value to be written
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_WritePage(const uint8_t * const pData, const uint32_t TarAddr, const uint16_t Size )
{
  if ((TarAddr + Size)>MemorySize)
      return EEPROMEX_ADDR_OVERFLOW;

  EEPROMEX_StatusTypeDef returnedStatus;
  if (((uint8_t)DeviceAddr & 0xF0) == SPI_MEM_ADDR)
  {
   if (EEPROMEX_SPI_Drv->WritePage == NULL)
   {
     return EEPROMEX_ERROR;
   }

   uint32_t iNumberOfPage = (Size) / PageSize;
   uint32_t iRemainder = (Size) % PageSize;
   uint32_t iPageNumber = TarAddr;
   if(iRemainder!=0)
   {
       iNumberOfPage +=1 ;
   }
   if(iNumberOfPage<=1)
   {
      /* iPageAddress is the absolute Address to write, depending on the page number*/
      uint32_t iPageAddress = TarAddr * PageSize;
      return EEPROMEX_SPI_Drv->WritePage(pData, iPageAddress, DeviceAddr, PageSize, Size );
   }
   else
   {
     for (uint32_t iCounter=0U; iCounter<iNumberOfPage; iCounter++)
     {
       uint32_t iPageAddress = iPageNumber*PageSize;
       returnedStatus = EEPROMEX_SPI_Drv->WritePage(&pData[0+ iCounter*PageSize], iPageAddress, DeviceAddr, PageSize, Size );
       iPageNumber++;
       osDelay(6);
     }
   }
   return returnedStatus;
  }
  else
   return EEPROMEX_ERROR;
}

/**
  * @brief  Reads data in the memory at specific address
  * @param  pData   : pointer to the data to write
  * @param  TarAddr : memory address to write
  * @param  TargetName : memory name to write
  * @param  Size    : Size in bytes of the value to be written
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_ReadData( uint8_t * const pData, const uint32_t TarAddr, const uint16_t Size )
{
  EEPROMEX_StatusTypeDef ret_value;

  if (TarAddr+Size>MemorySize)
	  return EEPROMEX_ADDR_OVERFLOW;

  if (((uint8_t)DeviceAddr & 0xF0) == SPI_MEM_ADDR)
  {
	  if ( EEPROMEX_SPI_Drv->ReadData == NULL )
	  {
		  return EEPROMEX_ERROR;
	  }
	  ret_value = EEPROMEX_SPI_Drv->ReadData( pData, TarAddr, DeviceAddr, Size );
	  if (ret_value == EEPROMEX_OK)
	  {
		  while (WINKY_EEPROMEX_IsDeviceReady( 1 ) != EEPROMEX_OK)
		  {
		  };
	  }
	  return ret_value;
  }
  else
	  return EEPROMEX_ERROR;
}

/**
  * @brief  Writes data in the memory at specific address
  * @param  pData : pointer to the data to write
  * @param  TarAddr : I2C data memory address to write
  * @param  TargetName : memory name to write
  * @param  Size : Size in bytes of the value to be written
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_WriteData( const uint8_t * const pData, const uint32_t TarAddr, const uint16_t Size )
{
  EEPROMEX_StatusTypeDef ret_value;

  if ((TarAddr + Size)>= MemorySize)
    return EEPROMEX_ADDR_OVERFLOW;

if (((uint8_t)DeviceAddr & 0xF0) == SPI_MEM_ADDR)
  {
   if ( EEPROMEX_SPI_Drv->WriteData == NULL )
    {
     return EEPROMEX_ERROR;
    }

   ret_value = EEPROMEX_SPI_Drv->WriteData( pData, TarAddr, DeviceAddr,PageSize, Size );
    if (ret_value == EEPROMEX_OK)
      {
       while (WINKY_EEPROMEX_IsDeviceReady( 1 ) != EEPROMEX_OK)
       {
       };
      }
   return ret_value;
  }
  else
    return EEPROMEX_ERROR;
}

/**
  * @brief  Writes data in identification page of the memory at specific address
  * @param  pData : pointer to the data to write
  * @param  TarAddr : I2C data memory address to write
  * @param  TargetName : memory name to write
  * @param  Size : Size in bytes of the value to be written
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_WriteID( const uint8_t * const pData, const uint32_t TarAddr, const uint16_t Size )
{

	if ((TarAddr + Size)>MemorySize)
		return EEPROMEX_ADDR_OVERFLOW;

	if (((uint8_t)DeviceAddr & 0xF0) == SPI_MEM_ADDR){
		if (EEPROMEX_SPI_Drv->WriteID == NULL)
		{
			return EEPROMEX_ERROR;
		}
		EEPROMEX_StatusTypeDef ret_value;
		ret_value = EEPROMEX_SPI_Drv->WriteID( pData, TarAddr, PageSize, DeviceAddr, Size );
		if (ret_value == EEPROMEX_OK)
		{
			while (WINKY_EEPROMEX_IsDeviceReady( 1 ) != EEPROMEX_OK)
			{
			};
			return EEPROMEX_OK;
		}
		return ret_value;
	}
  else
    return EEPROMEX_ERROR;
}

/**
  * @brief  Reads data in identification page of the memory at specific address
  * @param  pData   : pointer to the data to write
  * @param  TarAddr : memory address to write
  * @param  TargetName : memory name to write
  * @param  Size    : Size in bytes of the value to be written
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_ReadID( uint8_t * const pData, const uint32_t TarAddr, const uint16_t Size )
{

if (((uint8_t)DeviceAddr & 0xF0) == SPI_MEM_ADDR){
    if (EEPROMEX_SPI_Drv->ReadID == NULL)
    {
      return EEPROMEX_ERROR;
    }
    return EEPROMEX_SPI_Drv->ReadID( pData, TarAddr, PageSize, DeviceAddr ,Size );
  }
  else
    return EEPROMEX_ERROR;
}

/**
  * @brief  Reads data in identification page of the memory at specific address
  * @param  pData   : pointer to the data to write
  * @param  TargetName : memory name to write
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_LockStatus( uint8_t * const pData)
{
if (((uint8_t)DeviceAddr & 0xF0) == SPI_MEM_ADDR){
    if (EEPROMEX_SPI_Drv->LockStatus == NULL)
    {
      return EEPROMEX_ERROR;
    }
    return EEPROMEX_SPI_Drv->LockStatus( pData, DeviceAddr );
  }
  else
    return EEPROMEX_ERROR;
}

/**
  * @brief
  * @param  TargetName : memory name to write
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef WINKY_EEPROMEX_LockID( const uint8_t TargetName )
{
if (((uint8_t)DeviceAddr & 0xF0) == SPI_MEM_ADDR)
  {
    if (EEPROMEX_SPI_Drv->LockID == NULL)
    {
      return EEPROMEX_ERROR;
    }
    return EEPROMEX_SPI_Drv->LockID( DeviceAddr );
  }
  else
    return EEPROMEX_ERROR;
}

EEPROMEX_StatusTypeDef WINKY_SPI_DeInit( void )
{
  if (M95_spi_Drv.DeInit() == EEPROMEX_OK)
    {
      EEPROMEX_SPI_Drv = &M95_spi_Drv;
    }
  return EEPROMEX_OK;
}
