#ifndef __COMMMOCN_H
#define __COMMMOCN_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C"
{
#endif

	//typedef struct Network Network;

	extern osMessageQId Rs485QueueHandle;
	extern osSemaphoreId TIMEBinarySemHandle;
	//extern osMessageQId MBQueueHandle;
	extern __IO uint32_t usTick;

	//struct Network
	//{
	//	int my_socket;
	//	int (*mqttread) (Network*, unsigned char*, int, int);
	//	int (*mqttwrite) (Network*, unsigned char*, int, int);
	//	void (*disconnect) (Network*);
	//};

#define UAST_BUFFER_SIZE 128
	typedef struct Msg
	{
		uint8_t lengh;
		uint8_t Data[UAST_BUFFER_SIZE];
	} RS485_MSG_T; /* 定义一个结构体用于消息队列 */

extern RS485_MSG_T rs485_MSG;

#define PORT 8088

//#define EE_ipaddr 0

#define EE_timeaddr 0

#define EE_timeflageaddr 80

#define EE_modeflageaddr 110
//#define EE_setipflageaddr 120
	typedef enum
	{
		DO1 = 1,

		DO2,

		DO3,

		DO4,

		DO5,

		DO6,

		DO7,

		DO8

	} DO;

	//extern uint8_t IP_ADDRESS[4];
	//extern uint8_t NETMASK_ADDRESS[4];
	//extern uint8_t GATEWAY_ADDRESS[4];

	/* Semaphore to signal Ethernet Link state update */
	//extern osSemaphoreId Netif_LinkSemaphore;
	///* Ethernet link thread Argument */
	//extern struct link_str link_arg;

	//extern	 uint8_t aRxBuffer[32];
	//extern uint8_t setip_flage;

	//extern	 struct netif gnetif;

	void bsp_init(void);
	void EEinit(void);

	void User_UART_IRQHandler(UART_HandleTypeDef *huart);
	uint32_t user_GetTick(void);
	void user_Tick(void);

#endif
