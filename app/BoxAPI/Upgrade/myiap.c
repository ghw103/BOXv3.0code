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
#include <string.h>
#include "BoxAPI.h"
#include "w25qxx.h"
#include "cat1023.h"

pFunction JumpToApplication;
uint32_t JumpAddress;

/*****************************************************************************************/
uint32_t volatile MenuIndex = 0;
uint32_t volatile FlashProtection = 0;
/*****************************************************************************************/
//uint8_t  contactPack[] = { 0xAA, 0X55, 0xA5, 0x5A, 0x00, 0xFF };
//uint8_t  contackAck[] = { 0x5A, 0XA5, 0x55, 0xAA, 0x00, 0xFF };
//uint8_t sendDatPackReq[] = { 0xAA, 0X55, 0xA5, 0x5A, 0x01, 0xFE };
//uint8_t sendDatPackAck[] = { 0xAA, 0X55, 0xA5, 0x5A, 0x02, 0xFD };
//uint8_t revDataOk[] = { 0xAA, 0X55, 0xA5, 0x5A, 0xA0, 0x5F };

#define recv_data_size 1128
#define binFileLenaddr 496
/*****************************************************************************************/
const char DownloadFile[] = "box.bin";
const char UploadFile[] = "F0upload.bin";
/*****************************************************************************************/

/*****************************************************************************************/
void fatfstest(void)
{
	UINT fnum;													   /* 文件成功读写数量 */
	BYTE ReadBuffer[1024] = {0};								   /* 读缓冲区 */
	BYTE WriteBuffer[] = "今天是个好日子，新建文件系统测试文件\n"; /* 写缓冲区*/

	uint8_t workBuffer[_MAX_SS];
	printf("》串行FLASH还没有文件系统，即将进行格式化...\n");
	//		/* 格式化 */
	retUSER = f_mkfs((TCHAR const *)USERPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));
	printf("retmkfs %s\r\n", FR_Table[retUSER]);
	retUSER = f_mount(NULL, (TCHAR const *)USERPath, 0);
	printf("retUSER %s\r\n", FR_Table[retUSER]);
	retUSER = f_mount(&USERFatFS, (TCHAR const *)USERPath, 0);
	printf("retUSER %s\r\n", FR_Table[retUSER]);

	printf("****** 即将进行文件写入测试... ******\n");
	retUSER = f_open(&USERFile, "1.txt", FA_CREATE_ALWAYS | FA_WRITE);
	printf("retUSER %s\r\n", FR_Table[retUSER]);
	retUSER = f_write(&USERFile, WriteBuffer, sizeof(WriteBuffer), &fnum);
	printf("retUSER %s\r\n", FR_Table[retUSER]);
	f_close(&USERFile);
	printf("****** 即将进行文件读取测试... ******\n");
	taskENTER_CRITICAL();
	retUSER = f_open(&USERFile, "1.txt", FA_OPEN_EXISTING | FA_READ);
	taskEXIT_CRITICAL();
	printf("retUSER %s\r\n", FR_Table[retUSER]);

	retUSER = f_read(&USERFile, ReadBuffer, sizeof(ReadBuffer), &fnum);
	printf("retUSER %s\r\n", FR_Table[retUSER]);
	printf("》文件读取成功,读到字节数据：%d\n", fnum);
	printf("》读取得的文件数据为：\n%s \n", ReadBuffer);
	f_close(&USERFile);
}

void main_iap(void)
{

	switch (MenuIndex)
	{
		/* From card to FLASH */
	case 0:
		if (f_open(&USERFile, DownloadFile, FA_READ) != FR_OK)
		{
			printf(" Open file error           \n");
		}
		else
		{
			printf(" Download ongoing         \n");
			if (COMMAND_DOWNLOAD() != DOWNLOAD_OK)
			{
				printf(" Flash download error       \n");
			}
			else
			{
				printf(" Download done            \n");
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
void RunApplication(void)
{
	/* Test if user code is programmed starting from address "APPLICATION_ADDRESS" */
	if (((*(__IO uint32_t *)APPLICATION_ADDRESS) & 0x2FFE0000) == 0x20000000)
	{
		/* Jump to user application */
		JumpAddress = *(__IO uint32_t *)(APPLICATION_ADDRESS + 4);
		JumpToApplication = (pFunction)JumpAddress;
		/* Initialize user application's Stack Pointer */
#if (defined(__GNUC__))
		/* Compensation as the Stack Pointer is placed at the very end of RAM */
		__set_MSP((*(__IO uint32_t *)APPLICATION_ADDRESS) - 64);
#else  /* (defined  (__GNUC__ )) */
		__set_MSP(*(__IO uint32_t *)APPLICATION_ADDRESS);
#endif /* (defined  (__GNUC__ )) */
		JumpToApplication();
	}
	else
	{
		/* Clear the LCD */
		//		BSP_LCD_Clear(LCD_COLOR_WHITE);
		//		/*  Error message */
		//		BSP_LCD_DisplayStringAtLine(1, (uint8_t*)" The user application");
		//		BSP_LCD_DisplayStringAtLine(2, (uint8_t*)" isn't loaded properly ");
	}
}
int socketConnect(int *n, char *addr, int port)
{
	int socket, error = -1;
	struct sockaddr_in servaddr;
	socklen_t len;
	struct hostent *host;

	char str[INET_ADDRSTRLEN];

	socket = socket(AF_INET, SOCK_STREAM, 0);
	if (socket < 0)
		return socket;

	host = gethostbyname(addr);
	if (NULL == host || host->h_addrtype != AF_INET)
	{
		close(socket);
		return -2;
	}
	printf("\address: %s\n", inet_ntop(host->h_addrtype, host->h_addr, str, sizeof(str)));
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(port);
	memcpy(&servaddr.sin_addr, host->h_addr, sizeof(struct in_addr));
	//inet_aton(srv, &(servaddr.sin_addr));
	//inet_pton(AF_INET, srv, &servaddr.sin_addr);
	// TODO: Use SetSockOpt to
	error = connect(socket, (struct sockaddr *)&servaddr, sizeof(servaddr));
	if (error == 0)
	{
		error = getsockname(socket, (struct sockaddr *)&servaddr, &len);
		//if (error  >= 0) printf("Server %s connected, local port %d\n", srv, ntohs(servaddr.sin_port));
		*n = socket;
		return socket;
	}
	else
	{
		//printf("Error connecting %d\n", error);
		close(socket);
		return error;
	}
}
int8_t IAP_upgread(int socket)
{
	uint8_t contactPackReq[] = {0x74, 0x68, 0x69, 0x73, 0x62, 0x33};
	uint8_t contactPackAck[] = {0x74, 0x68, 0x69, 0x73, 0x73, 0x65, 0x76};
	uint8_t sendDatPackReq[] = {0x72, 0x65, 0x71, 0x6d, 0x73, 0x67};
	uint8_t sendDatPackAck[] = {0x72, 0x63, 0x76, 0x6d, 0x73, 0x67};
	uint8_t sendfilenum[4] = {0};
	uint8_t revallfile[] = {0x72, 0x63, 0x76, 0x61, 0x6c, 0x6c};

	uint8_t eeRead[16] = {0};

	uint8_t recv_data[recv_data_size];
	int recv_datalen = 0;
	uint32_t revPackNum = 0, revPacklenth;
	uint32_t binDataLen = 0, binFileLen = 0;
	uint8_t stm32_version;
	UINT fnum; /* 文件成功读写数量 */
	dog();
	enum File_Sta IAP_sta = ServerIdle;
	FILINFO finfno = {0};
	while (1)
	{
		switch (IAP_sta)
		{
		case ServerIdle: //发送握手包
			send(socket, contactPackReq, 6, 0);
			//				send(socket, BUFF, 1, 0);
			recv_datalen = UP_read(socket, recv_data, 7, 2000);
			if (recv_datalen <= 0)
			{
				return recv_datalen;
			}
			if (memcmp(recv_data, contactPackAck, recv_datalen) == 0)
			//			if(memcmp(recv_data, RBUFF, recv_datalen) == 0)
			{
				IAP_sta = clientCnn;
			}
			break;
		case clientCnn: ////连接上服务器，请求发送bin文件信息
			recv_datalen = 0;
			send(socket, sendDatPackReq, 6, 0);
			binDataLen = 0;
			binFileLen = 0;
			IAP_sta = clientReq;
			break;
		case clientReq: //获取bin文件信息
			recv_datalen = UP_read(socket, recv_data, 14, 2000);
			if (recv_datalen <= 0)
			{
				return recv_datalen;
			}
			///*/*/*/*/*	stm32_version = *(uint32_t *)(recv_data);*/*/*/*/*/
			stm32_version = recv_data[0] << 24 | recv_data[1] << 16 | recv_data[2] << 8 | recv_data[3];
			printf("stm32_version:%d\n", stm32_version);
			B3_eewrite((uint8_t *)(recv_data+10), binFileLenaddr,4);
			binFileLen = recv_data[10] << 24 | recv_data[11] << 16 | recv_data[12] << 8 | recv_data[13];
			printf("binlen:%d\n", binFileLen);
			recv_datalen = 0;
			dog();
			taskENTER_CRITICAL();
			retUSER = f_open(&USERFile, DownloadFile, FA_CREATE_ALWAYS | FA_WRITE);
			taskEXIT_CRITICAL();
			printf("retopen %s\r\n", FR_Table[retUSER]);
			if (retUSER != FR_OK)
			{
				//printf("retopen %s\r\n", FR_Table[retUSER]);
				/* file Open for write Error */
				printf("openerro\n");
				return -1;
				//while (1) ;
			}
			IAP_sta = senddatPak;
			revPackNum = 0;
			break;
		case senddatPak: //发送文件
						 //		printf("revPackNum:%d \n", revPackNum);

			sendfilenum[0] = (revPackNum >> 24) & 0x000000ff;
			sendfilenum[1] = (revPackNum >> 16) & 0x000000ff;
			sendfilenum[2] = (revPackNum >> 8) & 0x000000ff;
			sendfilenum[3] = (revPackNum)&0x000000ff;
			send(socket, sendfilenum, 4, 0);
			dog();
			revPacklenth = binFileLen - binDataLen;
			//		printf("recvlenth:%d\r\n", revPacklenth);
			if (revPacklenth > 1024)
			{
				revPacklenth = 1034;
			}
			else
			{
				revPacklenth += 10;
			}
			memset(recv_data, 0, recv_data_size);

			recv_datalen = UP_read(socket, recv_data, revPacklenth, 2000);
			//		printf("recv_len:%d\n", recv_datalen);
			if (recv_datalen <= 0)
			{
				f_close(&USERFile);
				//f_unlink(DownloadFile);
				return recv_datalen;
			}
			//校验数据包头
			//		printf("recvpack:%d\n", *(uint32_t *)(recv_data));
			if (recv_datalen) //接收到数据
			{
				if ((recv_data[0] << 24 | recv_data[1] << 16 | recv_data[2] << 8 | recv_data[3]) == revPackNum)
				{
					revPacklenth = recv_data[4] << 24 | recv_data[5] << 16 | recv_data[6] << 8 | recv_data[7];
					if (recv_datalen == (revPacklenth + 10))
					{
						binDataLen += recv_data[4] << 24 | recv_data[5] << 16 | recv_data[6] << 8 | recv_data[7];
						dog();
						taskENTER_CRITICAL(); //or portENTER_CRITICAL();
						retUSER = f_write(&USERFile, recv_data + 10, recv_datalen - 10, &fnum);
						taskEXIT_CRITICAL(); //or portEXIT_CRITICAL();
						f_sync(&USERFile);
						f_lseek(&USERFile, binDataLen);
					}
					if ((fnum == 0) || (retUSER != FR_OK))
					{
						//binDataLen -= *(uint32_t*)(recv_data + 4);
						/* 'STM32.TXT' file Write or EOF Error */
						f_close(&USERFile);
						printf("writeerro\n");
						return -1;
					}
					else
					{
						recv_datalen = 0;
						revPackNum++;
					}
					if (binDataLen == binFileLen)
					{
						IAP_sta = sendAllOk;
						f_close(&USERFile);
						printf("-->Receive  Bin Finish\n");
						//	HAL_Delay(2000);
						printf("-->Reboot  Device  for Updata Bin \n");
					}
					else
					{
						printf("binDataLen:%d \n", binDataLen);
						printf("Receive: %d%% Bin File\n", (binDataLen * 100 / binFileLen));
						retUSER = -1;
					}
				}
				//				}
			}
			break;
		case sendAllOk:

			dog();
			if (f_stat(DownloadFile, &finfno) != FR_OK)
			{
				
			}
			if ((finfno.fsize) != binFileLen)
			{
				//f_close(&USERFile);
				f_unlink(DownloadFile);
				return -1;
			}
			printf("feilsize:%d\n", finfno.fsize);
			IAP_sta = 6;
			printf("接收完毕\n");
			send(socket, revallfile, 6, 0);
			__set_FAULTMASK(1);
			HAL_NVIC_SystemReset();
			//	f_unlink(DownloadFile);
			break;
			//		case:
			//			break;
		default:
			osDelay(100);
			break;
		}
	}
}

void dog(void)
{
	HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
}
int UP_read(int my_socket, unsigned char *buffer, int len, int timeout_ms)
{
	TickType_t xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
	TimeOut_t xTimeOut;
	struct timeval interval = {timeout_ms / 1000, (timeout_ms % 1000) * 1000};
	if (interval.tv_sec < 0 || (interval.tv_sec == 0 && interval.tv_usec <= 0))
	{
		interval.tv_sec = 0;
		interval.tv_usec = 1000;
	}
	int recvLen = 0;
	vTaskSetTimeOutState(&xTimeOut); /* Record the time at which this function was entered. */
	do
	{
		int rc = 0;
		setsockopt(my_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&interval, sizeof(struct timeval));
		rc = recv(my_socket, buffer + recvLen, len - recvLen, 0);
		if (rc > 0)
			recvLen += rc;
		else if (rc < 0)
		{
			recvLen = rc;
			break;
		}
	} while (recvLen < len && xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE);
	return recvLen;
}
