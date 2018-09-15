/**
  ******************************************************************************
  * File Name          : SPI.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __w25q16_H
#define __w25q16_H
#ifdef __cplusplus
 extern "C" {
#endif
 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "spi.h"
	 
#define	 FLASH_SPI       hspi3  
	 
#define  FLASH_ID    0xEF4015	 
	 
#define  FLASH_L	   HAL_GPIO_WritePin(FLASH_CS_GPIO_Port,FLASH_CS_Pin,GPIO_PIN_RESET) ; 		//W25QXX的片选信�?
#define  FLASH_H	 	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET); //W25QXX的片选信�? 
	 

/* USER CODE BEGIN Includes */
 
/* USER CODE END Includes */
 

 
/* USER CODE BEGIN Private defines */
 
/* USER CODE END Private defines */
 
extern void _Error_Handler(char *, int);
 

 
/* USER CODE BEGIN Prototypes */
	 static const char * FR_Table[] = 
	 {
		 "FR_OK���ɹ�", /* (0) Succeeded */
		 "FR_DISK_ERR���ײ�Ӳ������", /* (1) A hard error occurred in the low level disk I/O layer */
		 "FR_INT_ERR������ʧ��", /* (2) Assertion failed */
		 "FR_NOT_READY����������û�й���",
		 /* (3) The physical drive cannot work */
		 "FR_NO_FILE���ļ�������", /* (4) Could not find the file */
		 "FR_NO_PATH��·��������", /* (5) Could not find the path */
		 "FR_INVALID_NAME����Ч�ļ���", /* (6) The path name format is invalid */
		 "FR_DENIED�����ڽ�ֹ���ʻ���Ŀ¼�������ʱ��ܾ�", /* (7) Access denied due to prohibited access or directory full */
		 "FR_EXIST���ļ��Ѿ�����",
		 /* (8) Access denied due to prohibited access */
		 "FR_INVALID_OBJECT���ļ�����Ŀ¼������Ч", /* (9) The file/directory object is invalid */
		 "FR_WRITE_PROTECTED������������д����", /* (10) The physical drive is write protected */
		 "FR_INVALID_DRIVE���߼���������Ч", /* (11) The logical drive number is invalid */
		 "FR_NOT_ENABLED�������޹�����",
		 /* (12) The volume has no work area */
		 "FR_NO_FILESYSTEM��û����Ч��FAT��",
		 /* (13) There is no valid FAT volume */
		 "FR_MKFS_ABORTED�����ڲ�������f_mkfs()����ֹ", /* (14) The f_mkfs() aborted due to any parameter error */
		 "FR_TIMEOUT���ڹ涨��ʱ�����޷���÷��ʾ�����", /* (15) Could not get a grant to access the volume within defined period */
		 "FR_LOCKED�������ļ�������Բ������ܾ�", /* (16) The operation is rejected according to the file sharing policy */
		 "FR_NOT_ENOUGH_CORE���޷����䳤�ļ���������",
		 /* (17) LFN working buffer could not be allocated */
		 "FR_TOO_MANY_OPEN_FILES����ǰ�򿪵��ļ�������_FS_SHARE",
		 /* (18) Number of open files > _FS_SHARE */
		 "FR_INVALID_PARAMETER��������Ч"	                     /* (19) Given parameter is invalid */
	 };
/* USER CODE END Prototypes */
 
#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */
 
 
 
 
/***************FLASH API******************/
 
uint32_t SPI_FLASH_ReadID(void);
 
void SPI_FLASH_BufferWrite(uint8_t* pBuffer,uint32_t WriteAddr,uint32_t NumByteToWrite);
 
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
 
void SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
 
void SPI_FLASH_SectorErase(uint32_t SectorAddr);
 
void SPI_FLASH_BulkErase(void);
 
void SPI_FLASH_WriteEnable(void);
 
void SPI_FLASH_WaitForWriteEnd(void);
 
void SPI_Flash_PowerDown(void);
 
void SPI_Flash_WAKEUP(void);
 
 
 
 
/**
  * @}
  */
 
/**
  * @}
  */
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
