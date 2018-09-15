/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
 
/* Includes ------------------------------------------------------------------*/

#include "w25q16.h"
 
 
#define SPI_FLASH_PageSize              256
 
 
/* USER CODE BEGIN 0 */
 
/* USER CODE END 0 */
 

 
uint8_t d_read,d_send;
 
/* SPI1 init function */

 
/********************************************FLASH API********************************************************/
 
//函数功能: 从串行Flash读取一个字节数据
uint8_t SPI_FLASH_ReadByte(void)
{

	  if(HAL_SPI_TransmitReceive(&FLASH_SPI,&d_send,&d_read,1,0xffff)!=HAL_OK)
    d_read=0xff;
//  if(HAL_SPI_TransmitReceive_DMA(&FLASH_SPI,&d_send,&d_read,1)!=HAL_OK)
//    d_read=0xff;
//	while (HAL_SPI_GetState(&FLASH_SPI) != HAL_SPI_STATE_READY)
//	{
//	  
//	} 
  return d_read;    
}
 
 
// 函数功能: 往串行Flash读取写入一个字节数据并接收一个字节数据
uint8_t SPI_FLASH_SendByte(uint8_t byte)
{
	
	if (HAL_SPI_TransmitReceive(&FLASH_SPI, &byte, &d_read, 1, 0xffff) != HAL_OK)
   return 1;
//  if(HAL_SPI_TransmitReceive_DMA(&FLASH_SPI,&byte,&d_read,1)!=HAL_OK)
//   return 1;
//	while (HAL_SPI_GetState(&FLASH_SPI) != HAL_SPI_STATE_READY)
//	{
//	} 
   return 0; 
}
 
 
 
 
 
 
 
 
 
//读取Flash ID
uint32_t SPI_FLASH_ReadID(void)
{
  uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
 
  /* 选择串行FLASH: CS低电平 */
  FLASH_L
 
  /* 发送命令：读取芯片型号ID */
  SPI_FLASH_SendByte(0x9F);
 
  Temp0 = SPI_FLASH_ReadByte();
  Temp1 = SPI_FLASH_ReadByte();
  Temp2 = SPI_FLASH_ReadByte();
 
  /* 禁用串行Flash：CS高电平 */
  FLASH_H
  
  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
  return Temp;
}
 
 
//从Flash读取数据
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
  FLASH_L
 
  SPI_FLASH_SendByte(0x03);
 
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  SPI_FLASH_SendByte((ReadAddr & 0xFF00) >> 8);
  SPI_FLASH_SendByte(ReadAddr  & 0xFF);
 
  while (NumByteToRead--) /* 读取数据 */
  {
    *pBuffer = SPI_FLASH_ReadByte();
    pBuffer++;
  }
 
  FLASH_H
}
 
 
 
void SPI_FLASH_PageWrite(uint8_t* pBuffer,uint32_t addr,uint16_t NumByteToRead)
{
	  uint16_t i;
	
    SPI_FLASH_WriteEnable();            //SET WEL
	
	  FLASH_L
	
	  SPI_FLASH_SendByte(0x02);
    SPI_FLASH_SendByte((uint8_t)((addr)>>16)); 	//发送24bit地址    
    SPI_FLASH_SendByte((uint8_t)((addr)>>8));   
    SPI_FLASH_SendByte((uint8_t)addr); 
	
	  for(i=0;i<NumByteToRead;i++)
	  {
			SPI_FLASH_SendByte(pBuffer[i]);//循环写数  
		}
	
	  FLASH_H                             //取消片选 
	  SPI_FLASH_WaitForWriteEnd();				//等待写入结束
}
 
 
//FLASH写函数
void SPI_FLASH_BufferWrite(uint8_t* pBuffer,uint32_t WriteAddr,uint32_t NumByteToWrite)   
{ 			 		 
	uint32_t pageremain;	   
	pageremain=256-WriteAddr%256; //单页剩余的字节数		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
	while(1)
	{	   
		SPI_FLASH_PageWrite(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	
 
			NumByteToWrite-=pageremain;			  //减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
			else pageremain=NumByteToWrite; 	  //不够256个字节了
		}
	}	    
} 
 
 
/* 函数功能: 擦除扇区 */
void SPI_FLASH_SectorErase(uint32_t SectorAddr)
{
  SPI_FLASH_WriteEnable();
  SPI_FLASH_WaitForWriteEnd();
 
  FLASH_L
  SPI_FLASH_SendByte(0x20);
  SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
  SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
  SPI_FLASH_SendByte(SectorAddr & 0xFF);
 
  FLASH_H
 
  SPI_FLASH_WaitForWriteEnd();
}
 
 
/* 函数功能: 擦除整个芯片 */
void SPI_FLASH_BulkErase(void)
{
  SPI_FLASH_WriteEnable();
 
  FLASH_L
  SPI_FLASH_SendByte(0xC7);
  FLASH_H
 
  SPI_FLASH_WaitForWriteEnd();
}
 
 
//写使能
void SPI_FLASH_WriteEnable(void)
{
  FLASH_L
  SPI_FLASH_SendByte(0x06);
  FLASH_H
}
 
 
 
//等待写入完毕
void SPI_FLASH_WaitForWriteEnd(void)
{
  uint8_t FLASH_Status = 0;
 
  FLASH_L
 
  SPI_FLASH_SendByte(0x05);
 
  do
  {
    FLASH_Status = SPI_FLASH_ReadByte();	 
  }
  while ((FLASH_Status & 0x01) == SET); 
 
  FLASH_H
}
 
 
 
/**
  * 函数功能: 进入掉电模式
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void SPI_Flash_PowerDown(void)   
{ 
  FLASH_L
 
  SPI_FLASH_SendByte(0xB9);
 
  FLASH_H
}   
 
/**
  * 函数功能: 唤醒串行Flash
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void SPI_Flash_WAKEUP(void)   
{
  FLASH_L
 
  SPI_FLASH_SendByte(0xAB);
 
  FLASH_H 
}   
 
 
 
 
 
/**
  * @}
  */
 
/**
  * @}
  */
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
