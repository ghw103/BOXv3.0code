/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
/* ------------------------ LWIP includes --------------------------------- */
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/memp.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip.h"
#include "netif/ethernet.h"
#include "app_ethernet.h"
/* ------------------------ FreeModbus includes --------------------------- */
#include "mb.h"
#include "mbcrc.h"
/* ------------------------ Project includes ------------------------------ */
#include "stm32f4xx_nucleo_144.h"
#include "usart.h"
#include "gpio.h"
#include "iwdg.h"
#include "crc.h"
#include "iwdg.h"

#include <string.h>
/* ------------------------ upgrade includes --------------------------- */
#include "myiap.h"
#include "fatfs.h"
/* ------------------------ MQTT includes --------------------------- */
#include "MQTTClient.h"
#include "mqttjson.h"
/* ------------------------ user includes --------------------------- */
#include "DAC.h"
#include "R8025t.h"
#include "Relay.h"
#include "ADS1230.h"
#include "w25q16.h"
#include "w25qxx.h"
#include "cat1023.h"
#include "BoxAPI.h"
#include "log.h"
#include "common.h"
/* ------------------------ set includes --------------------------- */
#include "recod.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId monitorTaskHandle;
osThreadId periodTaskHandle;
osThreadId MQTTTaskHandle;
osThreadId MQTTCallTaskHandle;
osThreadId upgreadTaskHandle;
osThreadId FindMeTaskHandle;
osThreadId serverTaskHandle;
osThreadId sockconnTaskHandle;
osThreadId sock485TaskHandle;
osMessageQId recvQueueHandle;
osMessageQId MBQueueHandle;
osMessageQId SockQueueHandle;
osMessageQId Rs485QueueHandle;
osMutexId MBholdingMutexHandle;
osMutexId sockMutexHandle;
osSemaphoreId TIMEBinarySemHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const *argument);
void Monitor(void const *argument);
void Period(void const *argument);
void MQTT_clien(void const *argument);
void MQTTCall(void const *argument);
void Upgread(void const *argument);
void FindMe(void const *argument);
void Sockserver(void const *argument);
void sockconnection(void const *argument);
void sock485(void const *argument);

extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
#define cleinlent 3
#define RS485buflen  128
struct netconn *newconnbuff[cleinlent];
uint8_t mqttstatus = 0;
int B3socket_read(int sock, unsigned char *buffer, int len, int timeout_ms);
/* USER CODE END FunctionPrototypes */
/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void)
{
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Create the mutex(es) */
	/* definition and creation of MBholdingMutex */
	osMutexDef(MBholdingMutex);
	MBholdingMutexHandle = osMutexCreate(osMutex(MBholdingMutex));

	/* definition and creation of sockMutex */
	osMutexDef(sockMutex);
	sockMutexHandle = osMutexCreate(osMutex(sockMutex));

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* definition and creation of TIMEBinarySem */
	osSemaphoreDef(TIMEBinarySem);
	TIMEBinarySemHandle = osSemaphoreCreate(osSemaphore(TIMEBinarySem), 1);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh, 0, 256);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of monitorTask */
	osThreadDef(monitorTask, Monitor, osPriorityBelowNormal, 0, 128);
	monitorTaskHandle = osThreadCreate(osThread(monitorTask), NULL);

	/* definition and creation of periodTask */
	osThreadDef(periodTask, Period, osPriorityNormal, 0, 512);
	periodTaskHandle = osThreadCreate(osThread(periodTask), NULL);

	/* definition and creation of MQTTTask */
	osThreadDef(MQTTTask, MQTT_clien, osPriorityAboveNormal, 0, 512);
	MQTTTaskHandle = osThreadCreate(osThread(MQTTTask), NULL);

	/* definition and creation of MQTTCallTask */
	osThreadDef(MQTTCallTask, MQTTCall, osPriorityNormal, 0, 256);
	MQTTCallTaskHandle = osThreadCreate(osThread(MQTTCallTask), NULL);

	/* definition and creation of upgreadTask */
	//  osThreadDef(upgreadTask, Upgread, osPriorityHigh, 0, 2048);
	//  upgreadTaskHandle = osThreadCreate(osThread(upgreadTask), NULL);

	/* definition and creation of FindMeTask */
	osThreadDef(FindMeTask, FindMe, osPriorityAboveNormal, 0, 512);
	FindMeTaskHandle = osThreadCreate(osThread(FindMeTask), NULL);

	/* definition and creation of serverTask */
	osThreadDef(serverTask, Sockserver, osPriorityAboveNormal, 0, 512);
	serverTaskHandle = osThreadCreate(osThread(serverTask), NULL);

	/* definition and creation of sockconnTask */
	osThreadDef(sockconnTask, sockconnection, osPriorityNormal, 0, 384);
	sockconnTaskHandle = osThreadCreate(osThread(sockconnTask), NULL);

	/* definition and creation of sock485Task */
	osThreadDef(sock485Task, sock485, osPriorityAboveNormal, 0, 512);
	sock485TaskHandle = osThreadCreate(osThread(sock485Task), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Create the queue(s) */
	/* definition and creation of recvQueue */
	/* what about the sizeof here??? cd native code */
	osMessageQDef(recvQueue, 4, uint16_t);
	recvQueueHandle = osMessageCreate(osMessageQ(recvQueue), NULL);

	/* definition and creation of MBQueue */
	/* what about the sizeof here??? cd native code */
	osMessageQDef(MBQueue, 4, uint16_t);
	MBQueueHandle = osMessageCreate(osMessageQ(MBQueue), NULL);

	/* definition and creation of SockQueue */
	/* what about the sizeof here??? cd native code */
	//  osMessageQDef(SockQueue, 2, struct netconn);
	//  SockQueueHandle = osMessageCreate(osMessageQ(SockQueue), NULL);
	//
	//  /* definition and creation of Rs485Queue */
	///* what about the sizeof here??? cd native code */
	//  osMessageQDef(Rs485Queue, 4, uint16_t);
	//  Rs485QueueHandle = osMessageCreate(osMessageQ(Rs485Queue), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */

	/* definition and creation of Rs485Queue */
	/* what about the sizeof here??? cd native code */
	osMessageQDef(Rs485Queue, 2, sizeof(RS485_MSG_T));
	Rs485QueueHandle = osMessageCreate(osMessageQ(Rs485Queue), NULL);
	/* definition and creation of SockQueue */
	/* what about the sizeof here??? cd native code */
	osMessageQDef(SockQueue, 2, sizeof(struct netconn));
	SockQueueHandle = osMessageCreate(osMessageQ(SockQueue), NULL);

	/* USER CODE END RTOS_QUEUES */
}
//uint8_t Write_test[256] = "45678654423456;kp[i[piopioputorwutiwutiueoiutiom,l3443u590387095782095902750729435 ";
//uint8_t Read_test[256] = {0};
/* StartDefaultTask function */
void StartDefaultTask(void const *argument)
{
	/* init code for FATFS */
	MX_FATFS_Init();

	/* USER CODE BEGIN StartDefaultTask */
	//I2C_EEPROM_WriteBuffer((512 + 496), (uint8_t *)Write_test, 100);
	//I2C_EEPROM_ReadBuffer((512 + 496), (uint8_t *)Read_test, 100);
	//printf((char *)Read_test);
	//printf("\r\n");
	loadDHCPset(&dhcp_flage);
	loadipar(NULL, NULL, NULL);
	//dhcp_flage = 1;
	/* Create tcp_ip stack thread */
	tcpip_init(NULL, NULL);
	/* Initialize the LwIP stack */
	Netif_Config();
	//	/* Initialize webserver demo */
	////	http_server_netconn_init();

	/* Notify user about the network interface config */
	User_notification(&gnetif);
	Init8025();
	/* Start DHCPClient */
	if (dhcp_flage)
	{
		osThreadDef(DHCP, DHCP_thread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 2);
		osThreadCreate(osThread(DHCP), &gnetif);
	}
	/* Infinite loop */
	for (;;)
	{
		osThreadTerminate(NULL);
		osDelay(1);
	}
	/* USER CODE END StartDefaultTask */
}

/* Monitor function */
void Monitor(void const *argument)
{
	/* USER CODE BEGIN Monitor */
	static portTickType xLastWakeTime;
	const portTickType xFrequency = pdMS_TO_TICKS(4000);

	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	for (;;)
	{
		//			  HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
		//  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		//		  rede_adc();
		//

		HAL_IWDG_Refresh(&hiwdg);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	//  /* Infinite loop */
	//  for(;;)
	//  {
	//	  BSP_LED_Toggle(LED_GREEN);
	//    osDelay(1000);
	//  }
	/* USER CODE END Monitor */
}

/* Period function */
void Period(void const *argument)
{
	/* USER CODE BEGIN Period */
	STDATETIME time;
	uint8_t time_buf[15];
	uint8_t flage[26];
	uint16_t dac;
	uint8_t modeflage;
	BaseType_t xResult;
	uint8_t restoretime;
	char pcWriteBuffer[500];
#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
	//	I2C_EEPROM_ReadBuffer(EE_timeaddr, time_buf, 48);
	//	//
	//	I2C_EEPROM_ReadBuffer(EE_timeflageaddr, flage, 24);
	//	//	for (u_int8_t i = 0; i < 24; i++)
	//	//	{
	//	//		printf("dac%2d:%d  ", i, flage[i]);
	//	//	}
	//	I2C_EEPROM_ReadBuffer(EE_modeflageaddr, &modeflage, 1);
	//	printf("mode:%d\n", modeflage);
	restoretime = 0;

	/* Infinite loop */
	for (;;)
	{
		xResult = xSemaphoreTake(TIMEBinarySemHandle, (TickType_t)portMAX_DELAY);
		if (xResult == pdTRUE)
		{
			BSP_LED_On(LED_GREEN);
			//BSP_LED_Toggle(LED_GREEN);
			if (HAL_GPIO_ReadPin(restore_GPIO_Port, restore_Pin) == 0)
			{
				restoretime++;
				if (restoretime > 3)
				{
					restoretime = 0;
					HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
					//				  while (!HAL_GPIO_ReadPin(restore_GPIO_Port, restore_Pin))
					//				  {
					//					  osDelay(10);
					//				  }
					restory();
				}
			}
			else
			{
				restoretime = 0;
			}
			HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
			UpdateDateTime(&time);
			if (modeflage == 0)
			{
				if (time.minute == 0)
				{
					if (flage[time.hour] == 1)
					{
						dac = (uint16_t)((time_buf[time.hour * 2] << 8) | time_buf[(time.hour * 2) + 1]);

						printf("time:%d dac:%d  \n", time.hour, dac);
						if (dac > 1000)
						{
							dac = 1000;
						}
						dac = dac * 3.055f;
						if (dac > 3055)
							dac = 3055;
						spi1_dac_write_chb(dac);
						spi1_dac_write_cha(dac);
					}
				}
			}
		}
		else
		{
			/* 失败 */
		}
		//		vTaskList((char *)&pcWriteBuffer);
					  //  printf("任务�?     任务状�??	优先�?	 剩余�? 	任务序号\r\n");
		//			  printf("%s\r\n", pcWriteBuffer);
		//	  	  sprintf((char *)time_buf,
		//	  		  "%04d-%02d-%02d %02d:%02d:%02d",
		//	  		  time.year + 2000,
		//	  		  time.month,
		//	  		  time.day,
		//	  		  time.hour,
		//	  		  time.minute,
		//	  		  time.second);
		//
		//	  	  printf("%s\n", time_buf);
		osDelay(50);
		BSP_LED_Off(LED_GREEN);
	}
	/* USER CODE END Period */
}

/* MQTT_clien function */
void MQTT_clien(void const *argument)
{
	/* USER CODE BEGIN MQTT_clien */

	MQTTClient client;
	Network network;
	unsigned char sendbuf[80], readbuf[80];
	int rc = 0;
	MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;
	argument = 0;
	char address[30] = {0};
	uint16_t port = 0;
	uint8_t mqttlock = 0;
	loadMQTTpar(&mqttlock, address, &port);
	if (mqttlock == 0)
	{
		for (;;)
		{
			vTaskDelete(NULL);
			osDelay(10);
		}
	}
	NetworkInit(&network);
	MQTTClientInit(&client, &network, 30000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));
	/* Infinite loop */
	for (;;)
	{
		//	char* address = "45.40.243.84";
		if ((rc = NetworkConnect(&network, address, port)) != 0)
			printf("Return code from network connect is %d\n", rc);
#if defined(MQTT_TASK)
		if ((rc = MQTTStartTask(&client)) != pdPASS)
			printf("Return code from start tasks is %d\n", rc);
#endif
		connectData.MQTTVersion = 3;
		connectData.clientID.cstring = "mqtt1test";
		connectData.keepAliveInterval = 10; //seconds
											// connectData.cleansession = 1;
		connectData.username.cstring = "";
		connectData.password.cstring = "";

		if ((rc = MQTTConnect(&client, &connectData)) != 0)
			printf("Return code from MQTT connect is %d\n", rc);
		else
			printf("MQTT Connected\n");

		if ((rc = MQTTSubscribe(&client, "test", 2, NULL)) != 0)
			printf("Return code from MQTT subscribe is %d\n", rc);
		/* Infinite loop */
		for (;;)
		{

			MQTTMessage message;
			//			if (xQueueReceive(MBQueueHandle, &mylog, 10) == pdPASS)
			//			{
			//				printf("log:%s", mylog);
			////				char payload[128];
			//				message.qos = 1;
			//				message.retained = 0;
			//				message.payload = mylog;
			//				//  sprintf(payload, "message number %d", count);
			//				  message.payloadlen = strlen(mylog);
			//		  				if ((rc = MQTTPublish(&client, "log", &message)) != 0)
			//					printf("Return code from MQTT publish is %d\n", rc);
			//			}
			if ((rc = MQTTYield(&client, 100)) != 0)
			{
				printf("Return code from yield is %d\n", rc);
				mqttstatus = 0;
				close(network.my_socket);
				break;
			}
			mqttstatus = 1;
			osDelay(10);
		}
		osDelay(1000);
	}
	/* USER CODE END MQTT_clien */
}

/* MQTTCall function */
void MQTTCall(void const *argument)
{
	/* USER CODE BEGIN MQTTCall */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END MQTTCall */
}

/* Upgread function */
void Upgread(void const *argument)
{
	/* USER CODE BEGIN Upgread */
	int upsocket;
	char address[30] = {0};
	uint16_t port = 0;
	
	uint8_t workBuffer[_MAX_SS];
	uint8_t recoon = 10;
	retUSER = f_mkfs((TCHAR const *)USERPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));

	retUSER = f_mount(NULL, (TCHAR const *)USERPath, 0);
	retUSER = f_mount(&USERFatFS, (TCHAR const *)USERPath, 0);

	loaduppar(address, &port);
	/* Infinite loop */
	for (;;)
	{
		HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
		while (recoon > 0)
		{
			if ((socketConnect(&upsocket, address, port)) >= 0)
			{
				if (IAP_upgread(upsocket) <= 0)
				{
					close(upsocket);
					//	break ;
				}
				else
				{
					break;
				}
			}
			osDelay(900);
			HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
			recoon--;
		}
		f_unlink(DownloadFile);
		__set_FAULTMASK(1);
		HAL_NVIC_SystemReset();
		while (1)
		{
			osDelay(100);
		}
	}
	/* USER CODE END Upgread */
}

/* FindMe function */
void FindMe(void const *argument)
{
	/* USER CODE BEGIN FindMe */
	__I uint8_t nomcorrect[9] = {0x09, (returnid + 1), 0x63, 0x6f, 0x72, 0x72, 0x65, 0x63, 0x74};
	__I uint8_t mqttcorrect[9] = {0x09, (returnid + 3), 0x63, 0x6f, 0x72, 0x72, 0x65, 0x63, 0x74};
	__I uint8_t savecorrect[9] = {0x09, (returnid + 5), 0x63, 0x6f, 0x72, 0x72, 0x65, 0x63, 0x74};
	__I uint8_t upcorrect[9] = {0x09, (returnid + 6), 0x63, 0x6f, 0x72, 0x72, 0x65, 0x63, 0x74};
	__I uint8_t rebcorrect[9] = {0x09, (returnid + 7), 0x63, 0x6f, 0x72, 0x72, 0x65, 0x63, 0x74};
	__I uint8_t rescorrect[9] = {0x09, (returnid + 8), 0x63, 0x6f, 0x72, 0x72, 0x65, 0x63, 0x74};
	__I uint8_t findme[6] = {"FindB3"};
	__I uint8_t readmqtt[10] = {"ReadB3mqtt"};
	__I uint8_t readsta[12] = {"ReadB3status"};
	__I uint8_t save[13] = {"Saveparameter"};
	__I uint8_t upgrade[12] = {"UpgradeB3"};
	__I uint8_t rebootb3[13] = {"RebootB3admin"};
	__I uint8_t restoreb3[14] = {"RestoreB3admin"};
	__I uint8_t correct[7] = {"correct"};

	uint32_t HWCRCValue = 0;
	uint32_t RVCRCValue = 0;
	unsigned char rec_buffer[200] = {0};
	uint8_t parameter[200];
	uint8_t ipaddrbuff[62];
	uint8_t saveipflage = 0;
	uint8_t upgradeflage = 0;
	//unsigned char UDP_Server_SendBuf[RCV_BUFFER_LEN] = { "hello" };
	LWIP_UNUSED_ARG(argument);
	uint8_t optval = 1;
	struct sockaddr_in server, client;
	int socket, RecvLen;
	socklen_t length = sizeof(struct sockaddr);
	ip_addr_t LocalIP;

	memset(&server, 0, sizeof(struct sockaddr_in));
	server.sin_family = AF_INET;
	server.sin_port = htons(1992);
	server.sin_addr.s_addr = htonl(INADDR_ANY);
	socket = socket(AF_INET, SOCK_DGRAM, 0);
	setsockopt(socket, SOL_SOCKET, SO_BROADCAST, (void *)&optval, sizeof(uint8_t));
	bind(socket, (struct sockaddr *)&server, sizeof(server));
	/* Infinite loop */
	for (;;)
	{
		//mac
		memset(rec_buffer, 0, 200);
		memset(parameter, 0, 200);
		HWCRCValue = 0;
		RVCRCValue = 0;
		RecvLen = recvfrom(socket, rec_buffer, sizeof(rec_buffer), 0, (struct sockaddr *)&client, &length); //
		if (RecvLen > 0)
		{
			if (strncmp((char *)rec_buffer, (char *)findme, RecvLen) == 0) //findme ok
			{
				loadpar(parameter);
				sendto(socket, parameter, 72, 0, (struct sockaddr *)&client, length);
			}
			else if (strncmp((char *)rec_buffer, (char *)readmqtt, RecvLen) == 0) //mqtt ok
			{
				loadparmqttbuf(parameter);
				sendto(socket, parameter, 135, 0, (struct sockaddr *)&client, length);
			}
			else if (strncmp((char *)rec_buffer, (char *)readsta, RecvLen) == 0) //status ok
			{
				loadtime(parameter);
				sendto(socket, parameter, 34, 0, (struct sockaddr *)&client, length);
			}
			else if (strncmp((char *)rec_buffer, (char *)save, RecvLen) == 0) //save  ok
			{
				msg_info("save\r\n");
				if (saveipflage)
				{
					savepar(ipaddrbuff);
					memset(ipaddrbuff, 0, 62);
					saveipflage = 0;
					sendto(socket, (uint8_t *)savecorrect, 9, 0, (struct sockaddr *)&client, length);
				}
				else
					sendto(socket, (uint8_t *)"error", 5, 0, (struct sockaddr *)&client, length);
			}
			else if (strncmp((char *)rec_buffer, (char *)upgrade, RecvLen) == 0) //upgrade
			{
				msg_info("upgrade");
					if (upgradeflage==0)
					{
						upgradeflage = 1;
						sendto(socket, (uint8_t *)upcorrect, 9, 0, (struct sockaddr *)&client, length);
						osDelay(10);
						/* definition and creation of upgreadTask */
						osThreadDef(upgreadTask, Upgread, osPriorityHigh, 0, 2048);
						upgreadTaskHandle = osThreadCreate(osThread(upgreadTask), NULL);
					}
			}
			else if (strncmp((char *)rec_buffer, (char *)rebootb3, RecvLen) == 0) //reboot ok
			{
					msg_info("reboot\r\n");
					sendto(socket, (uint8_t *)rebcorrect, 9, 0, (struct sockaddr *)&client, length);
					__set_FAULTMASK(1);
					HAL_NVIC_SystemReset();
			}
			else if (strncmp((char *)rec_buffer, (char *)restoreb3, RecvLen) == 0) //restore
			{
				msg_info("restore\r\n");
				//	sendto(socket, (uint8_t *) rescorrect, 9, 0, (struct sockaddr *)&client, length);
				//W25QXX_Erase_Chip();
				sendto(socket, (uint8_t *)rescorrect, 9, 0, (struct sockaddr *)&client, length);
			}
			else if (RecvLen == setmqttbufflen) //setmqtt ok
			{
				if (rec_buffer[0] == 0x87 && rec_buffer[1] == 0x68)
				{
					HWCRCValue = user_CRC((uint8_t *)rec_buffer, 131);
					RVCRCValue = (rec_buffer[131] << 24 | rec_buffer[132] << 16 | rec_buffer[133] << 8 | rec_buffer[134]);
					if (HWCRCValue == RVCRCValue)
					{
						saveparmqttbuf(rec_buffer);
						sendto(socket, (uint8_t *)mqttcorrect, 9, 0, (struct sockaddr *)&client, length);
					}
				}
			}
			else if (RecvLen == ipsetstastuslen) //setipaddrs ok
			{
				if (rec_buffer[0] == 0x5c && rec_buffer[1] == 0x66)
				{
					msg_info("ipset\r\n");
					HWCRCValue = user_CRC((uint8_t *)rec_buffer, 88);
					RVCRCValue = (rec_buffer[88] << 24 | rec_buffer[89] << 16 | rec_buffer[90] << 8 | rec_buffer[91]);

					if (HWCRCValue == RVCRCValue)
					{
						memcpy(ipaddrbuff, rec_buffer, 62);
						saveipflage = 1;
						sendto(socket, (uint8_t *)nomcorrect, 9, 0, (struct sockaddr *)&client, length);
					}
				}

				//	savepar(rec_buffer);
				//	loadpar(parameter);
				//	sendto(socket, (uint8_t *) parameter, 66, 0, (struct sockaddr *)&client, length);
			}
			else if (RecvLen == setusername) //setusrname ok
			{
				if (strncmp((char *)rec_buffer + 2, (char *)"passwd", 6) == 0)
				{
					msg_info("setname\r\n");
					saveuser((rec_buffer + 8));
					loaduser(parameter);
					sendto(socket, (uint8_t *)parameter, 28, 0, (struct sockaddr *)&client, length);
					//	msg_info("name:%s\r\n", parameter);
				}
			}
			//sendto(socket, parameter, 5, 0, (struct sockaddr *)&client, length);
		}

		osDelay(1);
	}
	/* USER CODE END FindMe */
}

/* Sockserver function */
void Sockserver(void const *argument)
{
	/* USER CODE BEGIN Sockserver */
	struct netconn *conn, *newconn;
	uint8_t conncant = 0, oldconncant = 0;
	err_t err, accept_err;
	BaseType_t xResult;

	struct netbuf *buf;
	void *data;
	u16_t len;

	LWIP_UNUSED_ARG(argument);
	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_TCP);
	if (conn != NULL)
	{
		/* Bind connection to well known port number 7. */
		err = netconn_bind(conn, NULL, 8088);
		printf("bind\r\n");
		if (err == ERR_OK)
		{
			/* Tell connection to go into listening mode. */
			netconn_listen(conn);
			/* Infinite loop */
			for (;;)
			{
				/* Grab new connection. */
				printf("new\r\n");
				accept_err = netconn_accept(conn, &newconn);
				printf("accept_err:%d\r\n", accept_err);
				//		printf("newconn:%d\r\n",(int)&newconn->write_offset);
				/* Process the new connection. */
				if (accept_err == ERR_OK)
				{
					if (xSemaphoreTake(sockMutexHandle, (TickType_t)200) == pdTRUE)
					{

						for (;;)
						{
							if (newconnbuff[conncant] == 0)
							{
								newconnbuff[conncant] = newconn;
								break;
							}
							conncant++;
							if (conncant >= cleinlent)
							{
								conncant = 0;
								netconn_close(newconnbuff[oldconncant]);
								netconn_delete(newconnbuff[oldconncant]);
								newconnbuff[oldconncant] = 0;
								oldconncant++;
								if (oldconncant >= cleinlent)
								{
									oldconncant = 0;
								}
							}
						}
						xSemaphoreGive(sockMutexHandle);
					}
					else
					{
						osDelay(1);
					}
					osDelay(1);
				}

				//					          while (netconn_recv(newconn, &buf) == ERR_OK)
				//					          {
				//					            do
				//					            {
				//					              netbuf_data(buf, &data, &len);
				//					              netconn_write(newconn, data, len, NETCONN_COPY);
				//
				//					            }
				//					            while (netbuf_next(buf) >= 0);
				//
				//					            netbuf_delete(buf);
				//					          }
				//					//
				//					//          /* Close connection and discard connection identifier. */
				//					          netconn_close(newconn);
				//					          netconn_delete(newconn);
			}
			osDelay(1);
		}
		osDelay(1);
	}
	else
	{
		netconn_delete(newconn);
	}

	/* USER CODE END Sockserver */
}

/* sockconnection function */
void sockconnection(void const *argument)
{
	/* USER CODE BEGIN sockconnection */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END sockconnection */
}

/* sock485 function */
void sock485(void const *argument)
{
	/* USER CODE BEGIN sock485 */
	int rs485socket;
	int ret;
	BaseType_t xResult;

	unsigned char recv_buffer[RS485buflen];

	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(50); /*  wait_time */
	char *address = "192.168.1.113";
	RS485_MSG_T *tcp_c_msg;
	//memset(tcp_c_msg, 0, sizeof(*tcp_c_msg));
	/* Infinite loop */
	for (;;)
	{

		if ((socketConnect(&rs485socket, address, 1993)) >= 0)
		{
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
			while (1)
			{
				xResult = xQueueReceive(Rs485QueueHandle,
										(void *)&tcp_c_msg,
										(TickType_t)xMaxBlockTime); /* time */
				if (xResult == pdPASS)
				{
					//  FreeRTOS_write(&network, tcp_c_msg->Data, tcp_c_msg->lengh, 100);
					write(rs485socket, tcp_c_msg->Data, tcp_c_msg->lengh);
					memset(&rs485_MSG, 0, sizeof(rs485_MSG));
				}
				ret = B3socket_read(rs485socket, recv_buffer, RS485buflen, 50);
				if (ret > 0)
				{
					HAL_GPIO_WritePin(RS485DIR_GPIO_Port, RS485DIR_Pin, GPIO_PIN_SET);
					taskENTER_CRITICAL();
					HAL_UART_Transmit(&huart1, recv_buffer, ret, 100); // (strlen((char*)recv_buffer)
					taskEXIT_CRITICAL();
					HAL_GPIO_WritePin(RS485DIR_GPIO_Port, RS485DIR_Pin, GPIO_PIN_RESET);
					memset(recv_buffer, 0, ret);
				}

				else if (ret != -2)
				{
					//				  connectflage = 1;
					__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
					__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
					close(rs485socket);
					break;
				}
				osDelay(1);
			}
		}
		osDelay(1000);
	}
	/* USER CODE END sock485 */
}

/* USER CODE BEGIN Application */
int B3socket_read(int sock, unsigned char *buffer, int len, int timeout_ms)
{
	uint8_t myerrno;
	struct timeval interval = {timeout_ms / portTICK_PERIOD_MS / 1000, (timeout_ms % (portTICK_PERIOD_MS * 1000)) * 1000};
	//	if (interval.tv_sec < 0 || (interval.tv_sec == 0 && interval.tv_usec <= 0))
	//	{
	//		interval.tv_sec = 0;
	//		interval.tv_usec = 1000;
	//	}
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&interval, sizeof(struct timeval));
	//	setsockopt(n->my_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&xTicksToWait, sizeof( xTicksToWait));
	//	printf("err:%d", eer);
	//	int bytes = 0;
	//	while (bytes < len)
	//	{
	int rc = recv(sock, buffer, len, 0);
	myerrno = errno;
	if (rc == -1)
	{
		//printf("errno:%d\n", myerrno);
		if (myerrno == EAGAIN || myerrno == EINTR)
			return -2;
		//	else if (errno == EINTR)return -3 ;
		else if (myerrno > 0)
			return 0; //0閺夆晝鍋炵敮鎾棘椤撶偟�??
		else
			return -1;

		//if (errno != ENOTCONN && errno != ECONNRESET)
		//{
		//}
	}
	//		else if (rc == 0) return -1;
	//else if (rc==0) break;
	//		else bytes += rc;
	//	}
	return rc;
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
