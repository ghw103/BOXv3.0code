
#include "common.h"
//#include "cmsis_os.h"
#include "usart.h"
//#include "gpio.h"
//#include "iwdg.h"
#include "DAC.h"
#include "R8025t.h"
//#include "Relay.h"
#include "ADS1230.h"
//#include "recod.h"
#include <string.h>
//#include "eeprom.h"
#include "cat1023.h"
//#include "usart.h"
//#include "fatfs.h"
#include "app_ethernet.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
__IO uint32_t user_sTick;
RS485_MSG_T rs485_MSG;
//const char ConfigFile[] = "Config.json";

////////void EEinit(void)
////////{
////////	uint8_t setipflage = 0;
////////	//				/* 关中断*/
////////	//			taskDISABLE_INTERRUPTS();
////////	I2C_EEPROM_ReadBuffer(EE_setipflageaddr, &setipflage, 1);
////////	/* 开中断 */
////////	//		taskENABLE_INTERRUPTS();
////////	printf("flage:%d  ", setipflage);
////////	if (setipflage == 1)
////////	{
////////		I2C_EEPROM_ReadBuffer(EE_ipaddr, IP_ADDRESS, 4);
////////		for (uint8_t i = 0; i < 4; i++)
////////		{
////////			printf("ip%2d:%d  ", i, IP_ADDRESS[i]);
////////		}
////////		I2C_EEPROM_ReadBuffer(EE_ipaddr + 4, GATEWAY_ADDRESS, 4);
////////		for (uint8_t i = 0; i < 4; i++)
////////		{
////////			printf("way%2d:%d  ", i, GATEWAY_ADDRESS[i]);
////////		}
////////		I2C_EEPROM_ReadBuffer(EE_ipaddr + 8, NETMASK_ADDRESS, 4);
////////		for (uint8_t i = 0; i < 4; i++)
////////		{
////////			printf("way%2d:%d  ", i, NETMASK_ADDRESS[i]);
////////		}
////////	}
//////	/*else
//////	{
//////		IP_ADDRESS[0] = 192;
//////		IP_ADDRESS[1] = 168;
//////		IP_ADDRESS[2] = 1;
//////		IP_ADDRESS[3] = 90;
//////		NETMASK_ADDRESS[0] = 255;
//////		NETMASK_ADDRESS[1] = 255;
//////		NETMASK_ADDRESS[2] = 255;
//////		NETMASK_ADDRESS[3] = 0;
//////		GATEWAY_ADDRESS[0] = 192;
//////		GATEWAY_ADDRESS[1] = 168;
//////		GATEWAY_ADDRESS[2] = 1;
//////		GATEWAY_ADDRESS[3] = 254;
//////	}*/
//////}
void bsp_init(void)
{

	HAL_GPIO_WritePin(RS485DIR_GPIO_Port, RS485DIR_Pin, GPIO_PIN_SET);

	Init8025();

	spi1_dac_write_chb(0);
	spi1_dac_write_cha(0);
	InitADline();
	printf("systemok\n");
}

//void Loadparameter()
//{
//	if (f_open(&USERFile, ConfigFile, FA_READ) != FR_OK)
//	{
//		printf(" Open file error           \n");
//	}
//}

void User_UART_IRQHandler(UART_HandleTypeDef *huart)
{

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	RS485_MSG_T *rx_msg;
	//	char * buff = "$rs485$";
	rx_msg = &rs485_MSG;
	//	memset(rx_msg, 0, sizeof(*rx_msg));
	uint8_t clean;
	if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET))
	{
//		if (rx_msg->lengh == 0)
//		{
//			rx_msg->lengh += 7;
//			//	rx_msg->Data += *buff;
//		}
		/* 关中断*/
		//	taskDISABLE_INTERRUPTS();
		//  __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);   //??????
		rx_msg->Data[rx_msg->lengh++] = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);

		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
		/* 开中断 */
		//	taskENABLE_INTERRUPTS();
	}
	if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)) //空闲中断
	{
		/* 关中断*/
		//	taskDISABLE_INTERRUPTS();
		clean = huart->Instance->DR;

		clean = huart->Instance->SR;

		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		//			HAL_GPIO_WritePin(_485DIR_GPIO_Port, _485DIR_Pin, GPIO_PIN_SET);
		//			HAL_UART_Transmit(&huart1, rx_msg->Data, rx_msg->lengh, 0xFFFF);
		//			HAL_GPIO_WritePin(_485DIR_GPIO_Port, _485DIR_Pin, GPIO_PIN_RESET);

		/* 向消息队列发数据 */
		xQueueSendFromISR(Rs485QueueHandle, (void *)&rx_msg, &xHigherPriorityTaskWoken);
		/* 如果 xHigherPriorityTaskWoken = pdTRUE ，那么退出中断后切到当前最高优先级任务执行 */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

		__HAL_UART_CLEAR_IDLEFLAG(huart);
		//			memset(rx_msg, 0, sizeof(*rx_msg));
		/* 开中断 */
		//	taskENABLE_INTERRUPTS();
	}
}

//int sockConnect(int * sock, char *addr, char *port)
//{
//	struct sockaddr_in address;
//	socklen_t len;
//	int rc = -1;
//	int socket;
//	sa_family_t family = AF_INET;
//	struct addrinfo *result = NULL;
//	struct addrinfo hints = { 0, AF_UNSPEC, SOCK_STREAM, IPPROTO_TCP, 0, NULL, NULL, NULL };
//	//	static struct timeval tv;
//
//	if((rc = getaddrinfo(addr, port, &hints, &result)) == 0)
//	{
//		struct addrinfo *res = result;
//		while (res)
//		{
//			if (res->ai_family == AF_INET)
//			{
//				result = res;
//				break;
//			}
//			res = res->ai_next;
//		}
//		if (result->ai_family == AF_INET)
//		{
//			address.sin_port = ((struct sockaddr_in *)(result->ai_addr))->sin_port;  // htons(port);
//			address.sin_family = AF_INET;
//			address.sin_addr = ((struct sockaddr_in *)(result->ai_addr))->sin_addr;
//		}
//		else
//			rc = -1;
//		freeaddrinfo(result);
//	}
//	if (rc == 0)
//	{
//		socket = socket(AF_INET, SOCK_STREAM, 0);
//		if (socket != -1)
//		{
//			//			eer = setsockopt(n->my_socket闂傚倷鐒︾?笛呯矙閹达富鏁嗛柣婊?绔瀇S0CKET, SO_RCVTIMEO闂??????(char *)&nNetTimeout, sizeof(nNetTimeout));
//			//			printf("ferr:%d", eer);
//			if(family == AF_INET)
//				rc = connect(socket, (struct sockaddr *)&address, sizeof(address));
//		}
//	}
//		if (rc == 0)
//		{
//			rc = getsockname(socket, (struct sockaddr *)&address, &len);
//			*sock = socket;
//		//	if (error  >= 0) printf("Server %s connected, local port %d\n", srv, ntohs(servaddr.sin_port));
//			//return n->my_socket;
//		}
//
//	return rc;
//}

void user_Tick(void)
{
	user_sTick++;
}
uint32_t user_GetTick(void)
{

	return user_sTick;
}
