/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../inc/MarlinConfig.h"

#if HAS_SPI_FLASH

#include "W25Qxx.h"

W25QXXFlash W25QXX;

#ifndef NC
  #define NC -1
#endif

MarlinSPI W25QXXFlash::mySPI(2);
#define SPI_FLASH_CS_H() OUT_WRITE(SPI_FLASH_CS_PIN, HIGH)
#define SPI_FLASH_CS_L() OUT_WRITE(SPI_FLASH_CS_PIN, LOW)

bool flash_dma_mode = true;

void W25QXXFlash::init(uint8_t spiRate) {
  class SPISettings spiConfig;
  OUT_WRITE(SPI_FLASH_CS_PIN, HIGH);

  spiConfig = SPISettings(20000000,kSPI_MsbFirst,SPI_MODE0,kSPI_Data8Bits,false);
  mySPI.beginTransaction(spiConfig);
}

/**
 * @brief  Receive a single byte from the SPI port.
 *
 * @return Byte received
 */
uint8_t W25QXXFlash::spi_flash_Rec() {
  const uint8_t returnByte = mySPI.transfer8Reflash(0xFF);
  return returnByte;
}

uint8_t W25QXXFlash::spi_flash_read_write_byte(uint8_t data) {
  const uint8_t returnByte = mySPI.transfer8Reflash(data);
  return returnByte;
}

/**
 * @brief  Receive a number of bytes from the SPI port to a buffer
 *
 * @param  buf   Pointer to starting address of buffer to write to.
 * @param  nbyte Number of bytes to receive.
 * @return Nothing
 *
 * @details Uses DMA
 */

void W25QXXFlash::spi_flash_Read(uint8_t *buf, uint16_t nbyte) {
  // mySPI.TransferNotdma(0, const_cast<uint8_t*>(buf), nbyte);//caden
  
    #define MAX_NotDMA_Flash (0x400 - 1)
    uint8_t test_tx_buff[1024] = {0xFF}; // add by dylan
    while (nbyte > 0) {
      mySPI.TransferMulflash(&test_tx_buff[0], 
                              buf,(nbyte > MAX_NotDMA_Flash ? MAX_NotDMA_Flash : nbyte)); // add by dylan
      buf+=MAX_NotDMA_Flash;
      nbyte = nbyte > MAX_NotDMA_Flash ? nbyte - MAX_NotDMA_Flash : 0;
    }
    #undef MAX_NotDMA_Flash

    // #define MAX_NotDMA_Flash (0x400 - 1)
    // while (nbyte > 0) {
    //   mySPI.TransferMulflash(NULL,buf,(nbyte > MAX_NotDMA_Flash ? MAX_NotDMA_Flash : nbyte));
    //   buf+=MAX_NotDMA_Flash;
    //   nbyte = nbyte > MAX_NotDMA_Flash ? nbyte - MAX_NotDMA_Flash : 0;
    // }
    // #undef MAX_NotDMA_Flash
    // uint16_t i;
    // uint8_t rx[2];
    // if(nbyte % 2 != 0)
    // {
    //     for(i=0;i<nbyte;i++)
    //     {
    //       *buf = mySPI.transfer(NULL);
    //       buf++;
    //     }     
    // }
    // else
    // {
    //     for(i=0;i<nbyte;i+=2)
    //     {
    //       mySPI.TransferNotdma_rec(rx,2);
    //       buf[i] = rx[0];
    //       buf[i+1] = rx[1];
    //     }       
    // }

  // uint16_t i;
  // for(i=0;i<nbyte;i++)
  // {
  //   *buf = mySPI.transfer(NULL);
  //    buf++;
  // }
}

/**
 * @brief  Send a single byte on SPI port
 *
 * @param  b Byte to send
 *
 * @details
 */
void W25QXXFlash::spi_flash_Send(uint8_t b) { mySPI.transfer8Reflash(b); }

/**
 * @brief  Write token and then write from 512 byte buffer to SPI (for SD card)
 *
 * @param  buf   Pointer with buffer start address
 * @return Nothing
 *
 * @details Use DMA
 */
void W25QXXFlash::spi_flash_SendBlock(uint8_t token, const uint8_t *buf) {
  int16_t i;
  mySPI.transfer8Reflash(token);
  mySPI.TransferMulflash(const_cast<uint8_t*>(buf),NULL, 512);//caden
}

uint16_t W25QXXFlash::W25QXX_ReadID(void) {
  uint16_t Temp = 0;
  SPI_FLASH_CS_L();
  spi_flash_Send(0x90);
  spi_flash_Send(0x00);
  spi_flash_Send(0x00);
  spi_flash_Send(0x00);
  Temp |= spi_flash_Rec() << 8;
  Temp |= spi_flash_Rec();
  SPI_FLASH_CS_H();
  return Temp;
}

void W25QXXFlash::SPI_FLASH_WriteEnable(void) {
  // Select the FLASH: Chip Select low
  SPI_FLASH_CS_L();
  // Send "Write Enable" instruction
  spi_flash_Send(W25X_WriteEnable);
  // Deselect the FLASH: Chip Select high
  SPI_FLASH_CS_H();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_WaitForWriteEnd
* Description    : Polls the status of the Write In Progress (WIP) flag in the
*                  FLASH's status register and loop until write operation has
*                  completed.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void W25QXXFlash::SPI_FLASH_WaitForWriteEnd(void) {
  uint8_t FLASH_Status = 0;

  // Select the FLASH: Chip Select low
  SPI_FLASH_CS_L();
  // Send "Read Status Register" instruction
  spi_flash_Send(W25X_ReadStatusReg);
//  SERIAL_ECHO_MSG("4_8");
  // Loop as long as the memory is busy with a write cycle
  do{
    /* Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    // SERIAL_IMPL.printf("FLASH_Status:%x\r\n",FLASH_Status);
    FLASH_Status = spi_flash_Rec();
    // SERIAL_IMPL.printf("FLASH_Status:%x\r\n",FLASH_Status);
    }while ((FLASH_Status & WIP_Flag) == 0x01); // Write in progress
  // SERIAL_ECHO_MSG("4_9");
  // Deselect the FLASH: Chip Select high
  SPI_FLASH_CS_H();
}

void W25QXXFlash::SPI_FLASH_SectorErase(uint32_t SectorAddr) {
  // Send write enable instruction
  SPI_FLASH_WriteEnable();
  // SERIAL_ECHO_MSG("4_1");
  // Sector Erase
  // Select the FLASH: Chip Select low
  SPI_FLASH_CS_L();
  // SERIAL_ECHO_MSG("4_2");
  // Send Sector Erase instruction
  spi_flash_Send(W25X_SectorErase);
  // SERIAL_ECHO_MSG("4_3");
  // Send SectorAddr high nybble address byte
  spi_flash_Send((SectorAddr & 0xFF0000) >> 16);
  // SERIAL_ECHO_MSG("4_4");
  // Send SectorAddr medium nybble address byte
  spi_flash_Send((SectorAddr & 0xFF00) >> 8);
  // SERIAL_ECHO_MSG("4_5");
  // Send SectorAddr low nybble address byte
  spi_flash_Send(SectorAddr & 0xFF);
  // SERIAL_ECHO_MSG("4_6");
  // Deselect the FLASH: Chip Select high

  SPI_FLASH_CS_H();
  //  SERIAL_ECHO_MSG("4_7");
  // Wait the end of Flash writing
  SPI_FLASH_WaitForWriteEnd();
   
}

void W25QXXFlash::SPI_FLASH_BlockErase(uint32_t BlockAddr) {
  SPI_FLASH_WriteEnable();
  SPI_FLASH_CS_L();
  // Send Sector Erase instruction
  spi_flash_Send(W25X_BlockErase);
  // Send SectorAddr high nybble address byte
  spi_flash_Send((BlockAddr & 0xFF0000) >> 16);
  // Send SectorAddr medium nybble address byte
  spi_flash_Send((BlockAddr & 0xFF00) >> 8);
  // Send SectorAddr low nybble address byte
  spi_flash_Send(BlockAddr & 0xFF);

  SPI_FLASH_CS_H();

  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BulkErase
* Description    : Erases the entire FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void W25QXXFlash::SPI_FLASH_BulkErase(void) {
  // Send write enable instruction
  SPI_FLASH_WriteEnable();

  // Bulk Erase
  // Select the FLASH: Chip Select low
  SPI_FLASH_CS_L();

  // Send Bulk Erase instruction
  spi_flash_Send(W25X_ChipErase);
  // Deselect the FLASH: Chip Select high
  SPI_FLASH_CS_H();
  // Wait the end of Flash writing
  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_PageWrite
* Description    : Writes more than one byte to the FLASH with a single WRITE
*                  cycle(Page WRITE sequence). The number of byte can't exceed
*                  the FLASH page size.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH,
*                    must be equal or less than "SPI_FLASH_PageSize" value.
* Output         : None
* Return         : None
*******************************************************************************/
void W25QXXFlash::SPI_FLASH_PageWrite(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
  // Enable the write access to the FLASH
  SPI_FLASH_WriteEnable();

  // Select the FLASH: Chip Select low
  SPI_FLASH_CS_L();
  // Send "Write to Memory " instruction
  spi_flash_Send(W25X_PageProgram);
  // Send WriteAddr high nybble address byte to write to
  spi_flash_Send((WriteAddr & 0xFF0000) >> 16);
  // Send WriteAddr medium nybble address byte to write to
  spi_flash_Send((WriteAddr & 0xFF00) >> 8);
  // Send WriteAddr low nybble address byte to write to
  spi_flash_Send(WriteAddr & 0xFF);

  NOMORE(NumByteToWrite, SPI_FLASH_PerWritePageSize);

  // While there is data to be written on the FLASH
  while (NumByteToWrite--) {
    // Send the current byte
    spi_flash_Send(*pBuffer);
    // Point on the next byte to be written
    pBuffer++;
  }

  // Deselect the FLASH: Chip Select high
  SPI_FLASH_CS_H();

  // Wait the end of Flash writing
  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferWrite
* Description    : Writes block of data to the FLASH. In this function, the
*                  number of WRITE cycles are reduced, using Page WRITE sequence.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void W25QXXFlash::SPI_FLASH_BufferWrite(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
  uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % SPI_FLASH_PageSize;
  count = SPI_FLASH_PageSize - Addr;
  NumOfPage = NumByteToWrite / SPI_FLASH_PageSize;
  NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

  if (Addr == 0) { // WriteAddr is SPI_FLASH_PageSize aligned
    if (NumOfPage == 0) { // NumByteToWrite < SPI_FLASH_PageSize
      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
    }
    else { // NumByteToWrite > SPI_FLASH_PageSize
      while (NumOfPage--) {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr += SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }
      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else { // WriteAddr is not SPI_FLASH_PageSize aligned
    if (NumOfPage == 0) { // NumByteToWrite < SPI_FLASH_PageSize
      if (NumOfSingle > count) { // (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize
        temp = NumOfSingle - count;
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
        WriteAddr += count;
        pBuffer += count;
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, temp);
      }
      else
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
    }
    else { // NumByteToWrite > SPI_FLASH_PageSize
      NumByteToWrite -= count;
      NumOfPage = NumByteToWrite / SPI_FLASH_PageSize;
      NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
      WriteAddr += count;
      pBuffer += count;

      while (NumOfPage--) {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr += SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      if (NumOfSingle != 0)
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
    }
  }
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferRead
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void W25QXXFlash::SPI_FLASH_BufferRead(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead) {
  // Select the FLASH: Chip Select low
  SPI_FLASH_CS_L();

  // Send "Read from Memory " instruction
  spi_flash_Send(W25X_ReadData);

  // Send ReadAddr high nybble address byte to read from
  spi_flash_Send((ReadAddr & 0xFF0000) >> 16);
  // Send ReadAddr medium nybble address byte to read from
  spi_flash_Send((ReadAddr & 0xFF00) >> 8);
  // Send ReadAddr low nybble address byte to read from
  spi_flash_Send(ReadAddr & 0xFF);

  if (NumByteToRead <= 32 || !flash_dma_mode) {

    while (NumByteToRead--) { // While there is data to be read
      // Read a byte from the FLASH
      *pBuffer = spi_flash_Rec();
      // Point to the next location where the byte read will be saved
      pBuffer++;
    }
  }
  else
  {

    spi_flash_Read(pBuffer, NumByteToRead);
  }

  SPI_FLASH_CS_H();
}

#endif // HAS_SPI_FLASH
