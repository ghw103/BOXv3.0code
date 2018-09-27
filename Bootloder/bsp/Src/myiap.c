/**
  ******************************************************************************
  * File Name          : 
  * Description        : This file provides code for the configuration
  *                     
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
#include "myiap.h"
#include "w25q16.h"
#include "fatfs.h"
#include "flash_if.h"
#include "command.h"
#include "string.h"
#include "usart.h"
#include "cat1023.h"
#include "i2c.h"
pFunction   JumpToApplication;
uint32_t    JumpAddress;

/*****************************************************************************************/
uint8_t  volatile  MenuIndex = 0;
uint32_t  volatile  FlashProtection = 0;
/*****************************************************************************************/
uint8_t  contactPack[] = { 0xAA, 0X55, 0xA5, 0x5A, 0x00, 0xFF };
uint8_t  contackAck[] = { 0x5A, 0XA5, 0x55, 0xAA, 0x00, 0xFF };
uint8_t sendDatPackReq[] = { 0xAA, 0X55, 0xA5, 0x5A, 0x01, 0xFE };
uint8_t sendDatPackAck[] = { 0xAA, 0X55, 0xA5, 0x5A, 0x02, 0xFD };
uint8_t revDataOk[] = { 0xAA, 0X55, 0xA5, 0x5A, 0xA0, 0x5F };
#define binFileLenaddr 496
#define recv_data_size   1048
uint8_t     recv_data[recv_data_size];
int    recv_datalen = 0;
uint32_t  revPackNum = 0, revPacklenth;
uint32_t  binDataLen = 0, binFileLen= 0;
uint8_t stm32_version ;

/*****************************************************************************************/
const char DownloadFile[] = "box.bin";
const char UploadFile[] = "F0upload.bin";
/*****************************************************************************************/
UINT fnum; /* 文件成功读写数量 */
BYTE ReadBuffer[1024] = { 0 }; /* 读缓冲区 */
BYTE WriteBuffer[] = "14424242424今天是个好日子，新建文件系统测试文件\n"; /* 写缓冲区*/  
FILINFO finfno = { 0 };
uint8_t workBuffer[_MAX_SS];
uint8_t eeRead[16];
/*****************************************************************************************/
void fatfstest(void)
{
	

//	printf("》串行FLASH还没有文件系统，即将进行格式化...\n");
//	//		/* 格式化 */
//	retUSER = f_mkfs((TCHAR const*)USERPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));
//			printf("retmkfs %s\r\n", FR_Table[retUSER]);
//		retUSER = f_mount(NULL, (TCHAR const*)USERPath,0);	
//		printf("retUSER %s\r\n", FR_Table[retUSER]);
//		retUSER = f_mount(&USERFatFS, (TCHAR const*)USERPath, 0);
//		printf("retUSER %s\r\n", FR_Table[retUSER]);

		printf("****** 即将进行文件写入测试... ******\n");	
		retUSER = f_open(&USERFile, "1.txt", FA_CREATE_ALWAYS | FA_WRITE);
		printf("retUSER %s\r\n", FR_Table[retUSER]);
		retUSER = f_write(&USERFile, WriteBuffer, sizeof(WriteBuffer), &fnum);
		printf("retUSER %s\r\n", FR_Table[retUSER]);
		f_close(&USERFile);
		printf("****** 即将进行文件读取测试... ******\n");
	//taskENTER_CRITICAL(); 
	retUSER = f_open(&USERFile, "1.txt", FA_OPEN_EXISTING | FA_READ); 	
	//taskEXIT_CRITICAL(); 
	printf("retUSER %s\r\n", FR_Table[retUSER]);
	
		retUSER = f_read(&USERFile, ReadBuffer, sizeof(ReadBuffer), &fnum); 
	printf("retUSER %s\r\n", FR_Table[retUSER]);
	printf("》文件读取成功,读到字节数据：%d\n", fnum);
	printf("》读取得的文件数据为：\n%s \n", ReadBuffer);	
	f_close(&USERFile);
}

void main_iap(void)
{

	/* Test if any sector of Flash memory where user application will be loaded is write protected */
	FlashProtection = FLASH_If_GetWriteProtectionStatus();
	if ((FlashProtection & FLASHIF_PROTECTION_WRPENABLED) != 0)
	{
//		MenuIndex = 4;
		printf("  Disable write protect \n");
		//MenuIndex = 0;
	}
	else
	{
//		MenuIndex = 0;
		printf("  Enable write protection \n");
	}
	
	switch (MenuIndex)
	{
		/* From card to FLASH */
	case 0:
		if (f_open(&USERFile, DownloadFile, FA_READ) != FR_OK)
		{
			printf(" Open file error           \n");
			MenuIndex = 3;
			break;
		}
		else
		{
			MX_I2C3_Init();
			HAL_Delay(100);
			I2C_EEPROM_ReadBuffer((512+ binFileLenaddr),(uint8_t*)eeRead,4);
			binFileLen = eeRead[0] << 24 | eeRead[1] << 16 | eeRead[2] << 8 | eeRead[3];
			if (f_stat(DownloadFile, &finfno) != FR_OK)
			{
				break;
			}
			if ((finfno.fsize) != binFileLen)
			{
				f_close(&USERFile);
				f_unlink(DownloadFile);
				__set_FAULTMASK(1);
				HAL_NVIC_SystemReset();
				break;
			}
			printf(" Download ongoing         \n");
			if (COMMAND_DOWNLOAD() != DOWNLOAD_OK)
			{
				printf(" Flash download error       \n");
			}
			else
			{
				
				printf(" Download done            \n");
				__set_FAULTMASK(1);
				HAL_NVIC_SystemReset();
				HAL_Delay(100);
				//RunApplication();
			}
		}
		break;
		/* From FLASH to card */
	case 1:
		if ((FlashProtection & FLASHIF_PROTECTION_RDPENABLED) == 0)
		{  
			/* Remove UPLOAD file if exists on flash disk */
			f_unlink(UploadFile);
			/* Open new file to be wrriten with flash data */
			if (f_open(&USERFile, UploadFile, FA_WRITE | FA_OPEN_ALWAYS) != FR_OK)
			{
				printf(" Open file error       \n");
			}
			else
			{
				/* Display LCD message */
				printf(" Upload ongoing         \n");
				COMMAND_UPLOAD();
				printf(" Upload done           \n ");
			}
		}
		else
		{
			printf(" Read protection active   \n");
		}
		break;
		/* Erase FLASH */
	case 2:
		printf(" Erase ongoing        \n");
		if (FLASH_If_Erase(APPLICATION_ADDRESS) != 0)
		{
			/* Clear the LCD */
			printf(" Flash erase error          \n");
		}
		else
		{
			/* Clear the LCD */
			printf(" User Flash erased        \n");
		}
		break;
		/* Run user application */
	case 3:
		RunApplication();
		break;
		/* Disable/Enable FLASH protection */
	case 4:
		if (FlashProtection != FLASHIF_PROTECTION_NONE)
		{
			/* Disable the write protection */
			if (FLASH_If_WriteProtectionConfig(OB_WRPSTATE_DISABLE) == FLASHIF_OK)
			{
				printf(" Write Protection disabled  \n ");
				printf(" Press tamper button  \n ");
				printf(" System will now restart   \n ");             
				/* Launch the option byte loading */
				HAL_FLASH_OB_Launch();
			}
			else
			{
				printf(" Error: Flash write    \n ");
				printf("  un-protection failed      \n ");

			}
		}
		else
		{
			if (FLASH_If_WriteProtectionConfig(OB_WRPSTATE_ENABLE) == FLASHIF_OK)
			{
				printf(" Write Protection enable  \n ");
				printf(" Press tamper button  \n ");
				printf(" System will now restart   \n ");

				/* Launch the option byte loading */
				HAL_FLASH_OB_Launch();
			}
			else
			{
				printf("Error: Flash write   \n ");
				printf("  protection failed   \n ");
			}
		}
		break;

	default:
		/* Clear the LCD */
		printf(" Selection Error      \n ");
		break;
	}
}
/**
  * @brief  This function runs a previously loaded application
  * @param  None
  * @retval None
  */
//uint32_t xx = 0;
void RunApplication(void)
{
	HAL_DeInit();
	HAL_SPI_MspDeInit(&hspi3);
	HAL_UART_MspDeInit(&huart1);
	__set_PRIMASK(1);
//	xx = ((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000);
	//printf("%d\n", ((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000));
	/* Test if user code is programmed starting from address "APPLICATION_ADDRESS" */
	if(((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000) == 0x20000000)
	{
//		SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;
//		__ASM("CPSID  I");

		/* Jump to user application */
		JumpAddress = *(__IO uint32_t*)(APPLICATION_ADDRESS + 4);
		JumpToApplication = (pFunction) JumpAddress;
		/* Initialize user application's Stack Pointer */
#if   (defined ( __GNUC__ ))
		    /* Compensation as the Stack Pointer is placed at the very end of RAM */
		__set_MSP((*(__IO uint32_t*) APPLICATION_ADDRESS) - 64);
#else  /* (defined  (__GNUC__ )) */
		__set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
#endif /* (defined  (__GNUC__ )) */

		JumpToApplication();
	}
	else
	{

//		/*  Error message */
	printf(" The user application\n");
	printf(	" isn't loaded properly \n");
	}
}
int socketConnect(int* n, char* addr, int port)
{
	//	int sockfd, error;
	//	struct sockaddr_in servaddr;
	//	socklen_t len;
	//	struct hostent *host;
	//    
	//	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	//	if (sockfd < 0) return sockfd;
	//
	//	host = gethostbyname(addr);
	//	if (NULL == host || host->h_addrtype != AF_INET) 
	//	{
	//		close(sockfd);
	//		return -2;
	//	}
	//
	//	memset(&servaddr, 0, sizeof(servaddr));
	//	servaddr.sin_family = AF_INET;
	//	servaddr.sin_port = htons(port);
	//	memcpy(&servaddr.sin_addr, host->h_addr, sizeof(struct in_addr));
	//	//inet_aton(srv, &(servaddr.sin_addr));
	////inet_pton(AF_INET, srv, &servaddr.sin_addr);
	//
	//	// TODO: Use SetSockOpt to
	//	error = connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));
	//
	//	if (error == 0)
	//	{
	//		error = getsockname(sockfd, (struct sockaddr *)&servaddr, &len);
	//		//if (error  >= 0) printf("Server %s connected, local port %d\n", srv, ntohs(servaddr.sin_port));
	//		*n = sockfd;
	//		return sockfd;
	//	}
	//	else
	//	{
	//		//printf("Error connecting %d\n", error);
	//		close(sockfd);
			return 0;// error;
//	}
}
//int8_t IAP_upgread(int socket)
//{
//	dog();
//	enum  File_Sta IAP_sta  = ServerIdle;
//	FILINFO finfno = { 0 };
//	while (1)
//	{
//		switch (IAP_sta)
//		{
//		case ServerIdle: //发送握手包
//			send(socket, contactPack, 6, 0);
//			recv_datalen = recv(socket, recv_data, recv_data_size, 0);
//			if (memcmp(recv_data, contackAck, recv_datalen) == 0)
//			{
//				IAP_sta =	clientCnn;
//			}
//			break;
//		case clientCnn: ////连接上服务器，请求发送bin文件信息
//			recv_datalen = 0;
//			send(socket, contactPack, 6,0);
//			binDataLen = 0;
//			binFileLen = 0;
//			IAP_sta = clientReq; 
//			break;
//		case clientReq: //获取bin文件信息
//			recv_datalen = recv(socket, recv_data, recv_data_size, 0);
//				stm32_version = *(uint32_t*)(recv_data);
//				printf("stm32_version:%d\n", stm32_version);
//				binFileLen = *(uint32_t*)(recv_data + 10);
//				printf("binlen:%d\n", binFileLen);
//				recv_datalen = 0;
//			//		retUSER = f_open(&USERFile, "1.txt", FA_CREATE_ALWAYS | FA_WRITE);
////				retUSER = f_mkfs((TCHAR const*)USERPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));
////			printf("retUSER %s\r\n", FR_Table[retUSER]);
//			dog();
//			taskENTER_CRITICAL();
//			
//			retUSER = f_open(&USERFile, DownloadFile, FA_CREATE_ALWAYS | FA_WRITE);
//				taskEXIT_CRITICAL();
//			printf("retopen %s\r\n", FR_Table[retUSER]);
//				if (retUSER!= FR_OK)
//				{
//					//printf("retopen %s\r\n", FR_Table[retUSER]);
//					/* file Open for write Error */
//					printf("openerro\n");
//					return -1;
//					//while (1) ;
//				}
//		
//				IAP_sta = senddatPak; 
//				revPackNum = 0;
//			break;
//		case senddatPak:
//			send(socket, sendDatPackAck, 6, 0);
////			recv_datalen = 0;
//			dog();
//			recv_datalen = read(socket, recv_data, recv_data_size);
//			//校验数据包头
//			printf("recv_len:%d\n", recv_datalen);
//			printf("recvpack:%d\n", *(uint32_t*)(recv_data));
//			if(recv_datalen)//接收到数据
//			{
//				if (*(uint32_t*)(recv_data) == revPackNum)
//				{
//					revPacklenth = *(uint32_t*)(recv_data + 4);
//					if (recv_datalen == (revPacklenth+10))
//					{
//						binDataLen += *(uint32_t*)(recv_data + 4); 
////						retUSER = FR_OK;
////						fnum = 1034;
////			
//					//	 taskENTER_CRITICAL();  //or portENTER_CRITICAL();
//						dog();
//						retUSER = f_write(&USERFile, recv_data + 10, recv_datalen - 10, &fnum);
//						f_sync(&USERFile);
//						f_lseek(&USERFile, binDataLen);
//						//taskEXIT_CRITICAL();   //or portEXIT_CRITICAL();
//					//	printf("retwrite %s\r\n", FR_Table[retUSER]);
//					}
//					if ((fnum == 0) || (retUSER != FR_OK))
//					{
//						/* 'STM32.TXT' file Write or EOF Error */
//						printf("writeerro\n");
//						return -1;
//					} 
//					//osDelay(100);
//					recv_datalen = 0;
//					revPackNum++;
//					if (binDataLen == binFileLen)
//					{
//						
//						IAP_sta = sendAllOk; 
//						f_close(&USERFile);
//						printf("-->Receive  Bin Finish\n");
//					//	HAL_Delay(2000);
//						printf("-->Reboot  Device  for Updata Bin \n");
//					}
//					else
//					{
//						printf("Receive: %d%% Bin File\n", (binDataLen * 100 / binFileLen));
//						retUSER = -1;
//					}
//				}
//			}
//			
//			break;
//		case sendAllOk:
//			send(socket, revDataOk, 6, 0);
//			dog();
//			if (f_stat(DownloadFile, &finfno) != FR_OK) {
//				/* 'STM32.TXT' file Open for write Error */
//				//return DOWNLOAD_FILE_FAIL;
//			}
//			printf("feilsize:%d\n", finfno.fsize);
//			
//			IAP_sta = 6;
//			printf("数据接收完毕可以升级\n");
//		//	f_unlink(DownloadFile);
//			break;
//			//		case:
//			//			break;
//			//			
//			//		case:
//			//			break;
//			//		case:
//			//			break;
//			//		case:
//			//			break;
//			//		case:
//			//			break;
//			//		case:
//			//			break;
//			//		case:
//			//			break;
//			
//					default :
//						HAL_Delay(100);
//			break;
//		}
//	}
////	rc = recv(socket, &buffer[bytes], (size_t)(len - bytes), 0);
////	rc = send(socket, buffer, len, 0);	
//	
//	
//}

void dog(void)
{
	HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
}