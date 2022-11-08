/**
  ******************************************************************************
  * @file    m95xx.c 
  * @author  MMY Application Team
  * @version V1.0.0
  * @date    17-August-2018
  * @brief   This file provides set of driver functions to manage communication 
  *          between BSP and M95xx chip.
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

/* Includes ------------------------------------------------------------------*/
#include "eeprom_common.h"


/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */ 

/** @defgroup M95
  * @brief This file provides set of driver functions to manage communication
  *        between BSP and M95xx chip.
  * @{
  */

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup M95_Private_Defines
  * @{
  */ 
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
EEPROMEX_StatusTypeDef M95_spi_Init( void );
EEPROMEX_StatusTypeDef M95_spi_DeInit( void );
EEPROMEX_StatusTypeDef M95_spi_IsDeviceReady( const uint32_t );
EEPROMEX_StatusTypeDef M95_spi_ReadData( uint8_t * const pData, const uint32_t TarAddr, const uint16_t DeviceAddr, const uint16_t NbByte );
EEPROMEX_StatusTypeDef M95_spi_WriteByte( const uint8_t * const pData, const uint32_t TarAddr,const uint16_t DeviceAddr);
EEPROMEX_StatusTypeDef M95_spi_WritePage( const uint8_t * const pData, const uint32_t TarAddr,const uint16_t DeviceAddr,const uint16_t PageSize,const uint16_t NByte);
EEPROMEX_StatusTypeDef M95_spi_WriteData( const uint8_t * const pData, const uint32_t TarAddr, const uint16_t DeviceAddr, const uint16_t PageSize, const uint16_t NByte );
EEPROMEX_StatusTypeDef M95_spi_WriteReg( const uint8_t Data, const uint8_t DevAddr );
EEPROMEX_StatusTypeDef M95_spi_ReadReg( uint8_t * const pData, const uint8_t DevAddr );
EEPROMEX_StatusTypeDef M95_spi_WriteID( const uint8_t * const pData, const uint32_t TarAddr,const uint16_t PageSize,const uint16_t DeviceAddr, const uint16_t NbByte );
EEPROMEX_StatusTypeDef M95_spi_ReadID( uint8_t * const pData, const uint32_t TarAddr,const uint16_t PageSize,const uint16_t DeviceAddr, const uint16_t NbByte );
EEPROMEX_StatusTypeDef M95_spi_ReadByte( uint8_t * const pData, const uint32_t TarAddr, const uint16_t DeviceAddr);
EEPROMEX_StatusTypeDef M95_spi_ReadPage( uint8_t * const pData, const uint32_t TarAddr, const uint16_t PageSize );
EEPROMEX_StatusTypeDef M95_spi_LockID( const uint16_t DeviceAddr );
EEPROMEX_StatusTypeDef M95_spi_LockStatus( uint8_t * const pData, const uint16_t DeviceAddr );
  
EEPROMEX_DrvTypeDef M95_spi_Drv =
{
  M95_spi_Init,
  M95_spi_DeInit,
  M95_spi_IsDeviceReady,
  M95_spi_ReadReg,
  M95_spi_WriteReg,
  M95_spi_ReadByte,
  M95_spi_WriteByte,
  M95_spi_ReadPage,
  M95_spi_WritePage,
  M95_spi_ReadData,
  M95_spi_WriteData,
  M95_spi_WriteID,
  M95_spi_ReadID,
  M95_spi_LockID, 
  M95_spi_LockStatus,
  NULL
};
/**
  * @}
  */ 

/* Public functions ---------------------------------------------------------*/
/** @defgroup M95_Public_Functions
  * @{
  */

/**
  * @brief  Set M95 eeprom Initialization
  * @param  None
  * @retval EEPROM enum status
  */
EEPROMEX_StatusTypeDef M95_spi_Init( void ) 
{
  /* Configure the low level interface */
  return M95_IO_Init( ); 
}
EEPROMEX_StatusTypeDef M95_spi_DeInit( void )
{
  return M95_IO_DeInit( ); 
}
/**
  * @brief  Check M95 availability
  * @param  Trials : number of max tentative tried
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_spi_IsDeviceReady( const uint32_t Trials ) 
{   
  return M95_IO_IsDeviceReady( DeviceAddr, Trials );          
}

/**
  * @brief  Read status registor of selected SPI memory
  * @param  pData : pointer of the data to store content of status registor 
  * @param  DeviceAddr : Device address of selected SPI memory
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_spi_ReadReg( uint8_t * const pData, const uint8_t DevAddr ) 
{
  M95_spi_IsDeviceReady(MIN_TRIALS);
  return M95_IO_ReadReg( pData, DevAddr );
}
/**
  * @brief  Write status registor of selected SPI memory
  * @param  Data : Content to write  to status registor of SPI memory    
  * @param  DeviceAddr : Device address of selected SPI memory
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_spi_WriteReg( const uint8_t Data, const uint8_t DevAddr )
{
  M95_spi_IsDeviceReady(MIN_TRIALS);
  return M95_IO_WriteReg( Data, DevAddr );
}

/**
  * @brief  Read single byte from specified SPI address
  * @param  pData : pointer of the data to store
  * @param  TarAddr : SPI data memory address to read
  * @param  DeviceAddr : Device Address of selected memory
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_spi_ReadByte( uint8_t * const pData, const uint32_t TarAddr, const uint16_t DeviceAddr )
{ 
  EEPROMEX_StatusTypeDef status;
  M95_spi_IsDeviceReady(MIN_TRIALS);
  /* Condition Matters only for 4Kb SPI ie M95040, for others EEPROMEX_WRITE & EEPROMEX_UPWRITE are same */
  if (DeviceAddr == 0xC6)
  {
    if (TarAddr < 256) /* Lower Half for 4Kbit  */
      status = M95_IO_MemRead( pData, TarAddr, DeviceAddr, 1 ,EEPROMEX_READ);
    else
      status = M95_IO_MemRead( pData, TarAddr, DeviceAddr, 1 ,EEPROMEX_UPREAD);
  }
  else
    status = M95_IO_MemRead( pData, TarAddr, DeviceAddr, 1 ,EEPROMEX_READ);
  return status;
}

/**
  * @brief  Read full page of the memory
  * @param  pData : pointer of the data to store
  * @param  TarAddr : SPI data memory address to read
  * @param  PageSize : Size of the page of selected memory
  * @param  NbByte : number of bytes to read
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_spi_ReadPage( uint8_t * const pData, const uint32_t TarAddr, const uint16_t PageSize )
{    		
  if ( M95_spi_IsDeviceReady(MIN_TRIALS) != EEPROMEX_OK ) 
  {
    return EEPROMEX_TIMEOUT;
  }
  return M95_IO_MemRead( pData, TarAddr, DeviceAddr, PageSize, EEPROMEX_READ );
}
/**
  * @brief  Read N bytes starting from specified SPI address
  * @param  pData : pointer of the data to store
  * @param  TarAddr : SPI data memory address to read
  * @param  NbByte : number of bytes to read
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_spi_ReadData( uint8_t * const pData, const uint32_t TarAddr, 
                                         const uint16_t DeviceAddr, const uint16_t Size )
{ 
  EEPROMEX_StatusTypeDef status;
  uint32_t targetAddress = TarAddr;
  M95_spi_IsDeviceReady(MIN_TRIALS);
  
  if (DeviceAddr == 0xC6)     /* Required for 4Kb SPI EEPROM only*/
  {
   if ((TarAddr + Size) <= 256) /* Lower Half for 4Kbit */
    status = M95_IO_MemRead( pData, TarAddr, DeviceAddr, Size, EEPROMEX_READ );
   else if (TarAddr > 256)
     status =  M95_IO_MemRead( pData, TarAddr, DeviceAddr, Size, EEPROMEX_UPREAD );
   else if ((TarAddr + Size > 256)&&(TarAddr <= 256))
   { 
     uint32_t temp1,temp2;
     temp1=(256 - targetAddress);        /* no. of bytes in lower half */ 
     M95_IO_MemRead( pData, targetAddress, DeviceAddr, temp1, EEPROMEX_READ );
     targetAddress += temp1;
     temp2=(TarAddr + Size)-256; /* no. of bytes in upper half */
     status =  M95_IO_MemRead( &pData[0+temp1], targetAddress, DeviceAddr, temp2, EEPROMEX_UPREAD );
   }
  }
  
  else
    status =  M95_IO_MemRead( pData, TarAddr, DeviceAddr, Size, EEPROMEX_READ);
     
  return status;
}

/**
  * @brief  Write a single byte to a specified address of SPI memory
  * @param  pData : pointer of the data to write
  * @param  TarAddr : SPI data memory address to write
  * @param  DeviceAddr : Device Address of selected memory
  * @retval EEPROMEX enum status
*/
EEPROMEX_StatusTypeDef M95_spi_WriteByte( const uint8_t * const pData, const uint32_t TarAddr,const uint16_t DeviceAddr)
{
  EEPROMEX_StatusTypeDef status;
  M95_spi_IsDeviceReady(MIN_TRIALS);
  /* Condition Matters only for 4Kb SPI ie M95040, for others EEPROMEX_WRITE & EEPROMEX_UPWRITE are same */
  if (DeviceAddr == 0xC6)
  {
    if (TarAddr < 256)
      status = M95_IO_MemWrite( pData, DeviceAddr, TarAddr, 1 ,EEPROMEX_WRITE);
    else
      status = M95_IO_MemWrite( pData, DeviceAddr, TarAddr, 1 ,EEPROMEX_UPWRITE);
  }
  else
    status = M95_IO_MemWrite( pData, DeviceAddr, TarAddr, 1 ,EEPROMEX_WRITE);
  
  M95_spi_IsDeviceReady(MIN_TRIALS);
  return status;
}

/*
  * @brief  Write maximum of pagesize bytes starting from specified SPI Address
  * @param  pData : pointer of the data to write
  * @param  TarAddr : SPI data memory address to write
  * @param  DeviceAddr : Device Address of selected memory
  * @param  PageSize : Size of the page of selected memory
  * @param  NbByte : number of bytes to write
  * @retval EEPROMEX enum status
*/

EEPROMEX_StatusTypeDef M95_spi_WritePage( const uint8_t * const pData, const uint32_t TarAddr, 
                                          const uint16_t DeviceAddr,const uint16_t PageSize,const uint16_t NByte)
{
  EEPROMEX_StatusTypeDef status;
  M95_spi_IsDeviceReady(MIN_TRIALS);  
  status = M95_IO_MemWrite( pData, DeviceAddr, TarAddr, PageSize, EEPROMEX_WRITE);
  M95_spi_IsDeviceReady(MIN_TRIALS);
  return status;
}

/**
  * @brief  Write N data bytes starting from specified SPI Address
  * @param  pData : pointer of the data to write
  * @param  TarAddr : SPI data memory address to write
  * @param  DeviceAddr : Device Address of the selected memory
  * @param  PageSize : Size of the page of selected memory
  * @param  NbByte : number of bytes to write
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_spi_WriteData( const uint8_t * const pData, const uint32_t TarAddr ,
                                          const uint16_t DeviceAddr ,const uint16_t PageSize, const uint16_t Size )
{
  uint32_t iNumberOfPage;
  int Page = 0;
  EEPROMEX_StatusTypeDef status = EEPROMEX_OK;
  uint32_t targetAddress = TarAddr;
  /*to handle dynamically start writing address*/
  if (targetAddress >= PageSize)
  {
     iNumberOfPage =  (Size / PageSize);
     Page = (targetAddress / PageSize);

    if (Size < PageSize)
    {
     if(((targetAddress + Size) / PageSize) > Page)
     {
        iNumberOfPage += 1;
     }
    }
    else 
    {
       if ((targetAddress % PageSize) > 0)
      {
        iNumberOfPage += 1;
      }
    } 
  }
  else  
  {
    iNumberOfPage = ( targetAddress + Size ) / PageSize;
  }
  
  uint32_t iRemainder = ( targetAddress + Size ) % PageSize;
  const uint8_t * pageIndex = pData;
  if (iRemainder>0)
  {
    iNumberOfPage += 1;
  }
  
  if ( M95_spi_IsDeviceReady(MIN_TRIALS) != EEPROMEX_OK )
  {
    return EEPROMEX_TIMEOUT;
  }  
  
  if (targetAddress == 0)       /*If target address from which read/write will be done starts from 0*/
  {
    for (int index = 0;index < iNumberOfPage;index++)
    { 
      uint32_t iSize = PageSize;
       if (index+1 == iNumberOfPage) /*For last page alignments*/
        {
          if (iRemainder == 0)
          {
            iSize = PageSize;
          }
          else 
          {
            iSize = iRemainder;
          }  
        }
        
       if (DeviceAddr == 0xC6) {          
         if (index < 16)
           status = M95_IO_MemWrite( pageIndex, DeviceAddr, targetAddress, iSize ,EEPROMEX_WRITE);
         else
           status = M95_IO_MemWrite( pageIndex, DeviceAddr, targetAddress, iSize ,EEPROMEX_UPWRITE);
       }
       else
          status = M95_IO_MemWrite( pageIndex, DeviceAddr, targetAddress, iSize ,EEPROMEX_WRITE);
         
        targetAddress += iSize;
        pageIndex += iSize;
        while( M95_spi_IsDeviceReady(MIN_TRIALS) != EEPROMEX_OK ) {}; 
        HAL_Delay(6);
     }
     return status;    
  }
  else
  {
    for (int index = 0;index < iNumberOfPage;index++)
    {
        uint32_t iSize = PageSize;
        if (index == 0) /*To align initial writing address*/
        { 
          if (targetAddress <= PageSize)
            iSize = (PageSize - targetAddress)>0? (PageSize - targetAddress) : PageSize;
          else
            iSize = PageSize - (targetAddress % PageSize); 
        }
          
        if (index+1 == iNumberOfPage) /*For last page alignments*/
        {
          if (iRemainder == 0)
          {
            iSize = PageSize;
          }
          else 
          {
            iSize = iRemainder;
          }  
        }
        
        
         /* Condition Matters only for 4Kb SPI ie M95040, for others EEPROMEX_WRITE & EEPROMEX_UPWRITE are same */
        if (DeviceAddr == 0xC6) {          
         if (targetAddress < 256)
           status = M95_IO_MemWrite( pageIndex, DeviceAddr, targetAddress, iSize ,EEPROMEX_WRITE);
         else
           status = M95_IO_MemWrite( pageIndex, DeviceAddr, targetAddress, iSize ,EEPROMEX_UPWRITE);
        }
        else
          status = M95_IO_MemWrite( pageIndex, DeviceAddr, targetAddress, iSize ,EEPROMEX_WRITE);
         
        targetAddress += iSize;
        pageIndex += iSize;
        HAL_Delay(6);             
        while( M95_spi_IsDeviceReady(MIN_TRIALS) != EEPROMEX_OK ) {};      
     }
     return status;     
  }
}

/**
  * @brief  Write Identification Page
  * @param  pData : pointer of the data to write
  * @param  TarAddr : SPI data memory address to write
  * @param  PageSize : Size of the page of selected memory
  * @param  DeviceAddr : Device Address of the selected memory
  * @param  NbByte : number of bytes to write
  * @retval EEPROMEX enum status
  */

EEPROMEX_StatusTypeDef M95_spi_WriteID( const uint8_t * const pData, const uint32_t TarAddr,
                                        const uint16_t PageSize,const uint16_t DeviceAddr, const uint16_t NbByte )
{
  EEPROMEX_StatusTypeDef status;
  uint32_t mem_addr = TarAddr;
  const uint8_t *pdata_index = (const uint8_t *)pData;
  uint16_t  count;
  uint16_t temp = PageSize;
  uint16_t  bitcount = BITCOUNT;
  uint32_t mask = 0 ;         
  if ( M95_spi_IsDeviceReady(MIN_TRIALS) != EEPROMEX_OK )
  {
    return EEPROMEX_TIMEOUT;
  }  
  while( temp / ( 1 << bitcount ) != 0 )
  {  /* Generate mask for address*/
   mask |= ( 1 << (bitcount - 1) );
   bitcount++;
  }
  mem_addr &= mask;                        /* Mask address address according to pagesize*/
  count = PageSize - TarAddr % PageSize;  /* Calculate available space in the ID page */
  if ( TarAddr % PageSize == 0 && NbByte <= PageSize )
  {  /*If adress is aligned to page */
    status = M95_IO_MemWrite( pdata_index, DeviceAddr, mem_addr, NbByte, EEPROMEX_WRID );
  }
  else if ( NbByte <= count )
  {  /* Address byte is not aligned with page and no byte must be less than free bytes in ID page*/
    status = M95_IO_MemWrite( pdata_index, DeviceAddr, mem_addr, NbByte, EEPROMEX_WRID );
  }
  else 
    return EEPROMEX_ERROR;   /* Return error if above two condtions does'nt met */
  M95_spi_IsDeviceReady(MIN_TRIALS);
  return status; 
}

/**
  * @brief  Read Identification Page
  * @param  pData : pointer of the data to read
  * @param  DeviceAddr : Device Address of the selected memory
  * @param  PageSize : Size of the page of selected memory
  * @param  NbByte : number of bytes to write
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_spi_ReadID( uint8_t * const pData, const uint32_t TarAddr,
                                       const uint16_t PageSize,const uint16_t DeviceAddr, const uint16_t NbByte )
{
  EEPROMEX_StatusTypeDef status1;
  uint32_t mem_addr;
  uint16_t count;
  uint16_t temp;  
  uint8_t bitcount;
  uint32_t mask;
  M95_spi_IsDeviceReady(MIN_TRIALS);
  mask = 0;
  bitcount = BITCOUNT;
  temp = PageSize; 
  mem_addr = TarAddr;
  while( temp / ( 1 << bitcount ) != 0 )
  {
   mask |= ( 1 << (bitcount - 1) );
   bitcount++;
  }
  mem_addr &= mask;
  count = PageSize - mem_addr % PageSize;
  if ( count > NbByte )
    status1 =  M95_IO_MemRead( pData, TarAddr, DeviceAddr, NbByte, EEPROMEX_RDID );
  else
    status1 =  M95_IO_MemRead( pData, TarAddr, DeviceAddr, count, EEPROMEX_RDID );
  return status1;
}

/**
  * @brief  Reads the Identification Page lock status
  * @param  pData : pointer of the data to read
  * @param  DeviceAddr : Size of the selected memory
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_spi_LockStatus( uint8_t * const pData, const uint16_t DeviceAddr )
{
    return M95_IO_MemRead( pData, ADDRLID_SPI, DeviceAddr, 1, EEPROMEX_RDLS );
}

/**
  * @brief  Locks the Identification page in read-only mode
  * @param  DeviceAddr : Device Address of the selected memory
  * @retval EEPROMEX enum status
  */
EEPROMEX_StatusTypeDef M95_spi_LockID( const uint16_t DeviceAddr )
{
  EEPROMEX_StatusTypeDef status = EEPROMEX_OK;
  uint8_t lock_data = LOCKDATA_SPI;
  status = M95_IO_MemWrite( &lock_data, DeviceAddr, ADDRLID_SPI, 1, EEPROMEX_LID );
  while( M95_spi_IsDeviceReady(MIN_TRIALS) != EEPROMEX_OK ) {}; 
  return status;
}  

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

