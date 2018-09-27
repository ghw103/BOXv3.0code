#include "BoxAPI.h"
//#include "common.h"
#include "crc.h"
#include "mbcrc.h"
#include "cJSON.h"
#include "version.h"
#include "w25qxx.h"
#include "log.h"
#include "recod.h"
//#include "DAC.h"
#include "R8025t.h"
#include "cat1023.h"
//#include "lwip.h"
//#include "lwip/init.h"
//#include "lwip/netif.h"
//#if defined ( __CC_ARM )  /* MDK ARM Compiler */
//#include "lwip/sio.h"
//#endif /* MDK ARM Compiler */
//#include "ethernetif.h"
#include "app_ethernet.h"
#include "fatfs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

///* Semaphore to signal Ethernet Link state update */
//osSemaphoreId Netif_LinkSemaphore = NULL;
///* Ethernet link thread Argument */
//struct link_str link_arg;

uint8_t B3Macaddr[6] = {0};
#define parparameteraddr 0;

uint8_t B3_eeread(uint8_t *pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead)
{
	return I2C_EEPROM_ReadBuffer( (ReadAddr+512), (uint8_t *)pBuffer,  NumByteToRead);//前面的512字节被使用
}
uint8_t B3_eewrite(uint8_t *pBuffer, uint16_t ReadAddr, uint16_t NumByteToWrite)
{
	return I2C_EEPROM_WriteBuffer( (ReadAddr+512), (uint8_t *)pBuffer,  NumByteToWrite);
}

uint32_t user_CRC(uint8_t *pBuff, uint32_t len)
{
	uint32_t i;
	uint8_t buff[4];
	uint32_t *p32 = (uint32_t *)buff;
	CRC->CR = 1; //复位CRC寄存器
	buff[0] = buff[1] = buff[2] = buff[3] = 0XFF;
	for (i = 0; i < len; i++)
	{
		buff[3] = pBuff[i];
		CRC->DR = *p32;
	}
	return CRC->DR;
}

void restory()
{
	uint8_t clean[512] = {0};
	//  taskENTER_CRITICAL();    //or portENTER_CRITICAL();
	W25QXX_Write((uint8_t *)clean, 0, 512);
	//taskEXIT_CRITICAL();      //or portEXIT_CRITICAL();
	__set_FAULTMASK(1);
	HAL_NVIC_SystemReset();
}

void SetMac(void)
{
	__IO uint32_t uwCRCValue = 0;
	static uint32_t CpuID[3];
	static uint32_t Lock_Code;

	CpuID[0] = *(__I uint32_t *)(0x1FFF7A10 + 0x00);
	CpuID[1] = *(__I uint32_t *)(0x1FFF7A10 + 0x04);
	CpuID[2] = *(__I uint32_t *)(0x1FFF7A10 + 0x08);

	//	printf("id:%x-%x-%x\n", CpuID[0], CpuID[1], CpuID[2]);

	uwCRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)CpuID, 3);
	//	printf("crc1: %2X\n", uwCRCValue>>24&0x000000ff);
	//	printf("crc2: %2X\n", uwCRCValue >> 16 & 0x000000ff);
	//	printf("crc3: %2X\n",((uwCRCValue >> 8 & 0x000000ff )| (uwCRCValue & 0x000000ff)));
	////	printf("crc4: %2X\n", uwCRCValue & 0x000000ff);
	////	printf("crc4: %2X\n", uwCRCValue & 0x000000ff);
	//	printf("crc: %X\n", uwCRCValue);
	B3Macaddr[0] = 0x94;
	B3Macaddr[1] = 0xDF;
	B3Macaddr[2] = 0x4E;
	B3Macaddr[3] = (uint8_t)(uwCRCValue >> 24 & 0x000000ff);
	B3Macaddr[4] = (uint8_t)(uwCRCValue >> 16 & 0x000000ff);
	B3Macaddr[5] = (uint8_t)((uwCRCValue >> 8 & 0x000000ff) | (uwCRCValue & 0x000000ff));
}

void loaduppar(char *ipaddr, uint16_t *port)
{
	uint8_t up[32] = {0};
	W25QXX_Read((uint8_t *)up, 30, 32);
	memcpy(ipaddr, up, 30);
	*port = (up[30] << 8 | up[31]);
	//memcpy(port, up + 30, 2);
	//	msg_debug("ip:%x\r\n",*port);
}
void loadipar(uint8_t *ipaddr, uint8_t *NETMASK, uint8_t *gatway)
{
	uint8_t ip[18] = {0};
	W25QXX_Read((uint8_t *)ip, 11, 18);
	if ((ip[0]) == 0)
	{
		/* IP addresses initialization */
		IP_ADDRESS[0] = 192;
		IP_ADDRESS[1] = 168;
		IP_ADDRESS[2] = 1;
		IP_ADDRESS[3] = 91;
		NETMASK_ADDRESS[0] = 255;
		NETMASK_ADDRESS[1] = 255;
		NETMASK_ADDRESS[2] = 255;
		NETMASK_ADDRESS[3] = 0;
		GATEWAY_ADDRESS[0] = 192;
		GATEWAY_ADDRESS[1] = 168;
		GATEWAY_ADDRESS[2] = 1;
		GATEWAY_ADDRESS[3] = 1;
	}
	else
	{
		memcpy(IP_ADDRESS, ip, 4);
		memcpy(NETMASK_ADDRESS, ip + 4, 4);
		memcpy(GATEWAY_ADDRESS, ip + 8, 4);
	}
	//	msg_debug("ip:%x\r\n",ip);

	msg_debug("ip:%d %d %d %d\r\n", IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
	//	msg_debug("mac:%s\r\n", ip);
	if ((ip[12]) == 0)
	{
		SetMac();
	}
	else
	{
		memcpy(B3Macaddr, ip + 12, 6);
	}
}
void loadMQTTpar(uint8_t *mqttlock, char *ipaddr, uint16_t *port)
{
	uint8_t mqtt[32] = {0};
	W25QXX_Read((uint8_t *)mqttlock, 102, 1);

	W25QXX_Read((uint8_t *)mqtt, 103, 32);
	memcpy(ipaddr, mqtt, 30);
	*port = (mqtt[30] << 8 | mqtt[31]);
	//	memcpy(port, mqtt + 30, 2);
}

void loadDHCPset(uint8_t *DHCP)
{
	uint8_t setB3dhcp[2] = {0};

	W25QXX_Read((uint8_t *)setB3dhcp, 10, 1);
	*DHCP = setB3dhcp[0];

	//		printf("spidata：\n%s \n", datatemp);
	//	cJSON * para;
	////	cJSON *arry;
	//
	//	para = cJSON_CreateObject();
	//	cJSON_AddNumberToObject(para, "SN", "fengxin");
	//	cJSON_AddNumberToObject(para, "B3id", "fengxin");
	//	cJSON_AddNumberToObject(para, "HWversion", HW_VERSION);
	//	cJSON_AddNumberToObject(para, "FWversion", FW_VERSION);
	//	cJSON_AddStringToObject(para, "username", "fengxin");
	//	cJSON_AddStringToObject(para, "passwd", "fengxin");
	//	cJSON_AddNumberToObject(para, "IPaddr", fengxin);
	//	cJSON_AddNumberToObject(para, "NETMASK", fengxin);
	//	cJSON_AddNumberToObject(para, "GATEWAY", fengxin);
	//	cJSON_AddNumberToObject(para, "MACaddr", fengxin);
	//	cJSON_AddNumberToObject(para, "dhcp", fengxin);
	//	cJSON_AddStringToObject(para, "MQTTIP", "fengxin");
	//	cJSON_AddNumberToObject(para, "MQTTpor", "fengxin");
	//	cJSON_AddStringToObject(para, "UPIP", "fengxin");
	//	cJSON_AddNumberToObject(para, "UPpor", "fengxin");
	//
	//
	//	char *out = cJSON_Print(para);     //将json形式打印成正常字符串形式
	//	printf("%s\n", out);
	//
	//	// 释放内存
	//	cJSON_Delete(para);
	//	free(out);
}
void loadparmqttbuf(uint8_t *parameter)
{
	__IO uint32_t uwCRCValue = 0;
	W25QXX_Read((uint8_t *)parameter, 100, 131);
	parameter[0] = 0x87;
	parameter[1] = (returnid + 2);
	uwCRCValue = user_CRC((uint8_t *)parameter, 131);
	//uwCRCValue = ~uwCRCValue;
	parameter[131] = (uint8_t)(uwCRCValue >> 24 & 0x000000ff);
	parameter[132] = (uint8_t)(uwCRCValue >> 16 & 0x000000ff);
	parameter[133] = (uint8_t)(uwCRCValue >> 8 & 0x000000ff);
	parameter[134] = (uint8_t)(uwCRCValue & 0x000000ff);
}
void saveparmqttbuf(uint8_t *parameter)
{
	//	__IO uint32_t uwCRCValue = 0;
	uint8_t zero[30] = {0};
	W25QXX_Write((uint8_t *)zero, 103, 30);
	W25QXX_Write((uint8_t *)parameter, 100, 131);
	//	uwCRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)parameter, 129);
	//uwCRCValue = ~uwCRCValue;
	//	parameter[129] = (uint8_t)(uwCRCValue >> 24 & 0x000000ff);
	//	parameter[130] = (uint8_t)(uwCRCValue >> 16 & 0x000000ff);
	//	parameter[131] = (uint8_t)(uwCRCValue >> 8 & 0x000000ff);
	//	parameter[132] = (uint8_t)(uwCRCValue & 0x000000ff);
}
void loadsta(uint8_t *parameter)
{
	//W25QXX_Read((uint8_t*)parameter, 300, 28);
}
void loadpar(uint8_t *parameter)
{
	__IO uint32_t uwCRCValue = 0;
	uint8_t macadddr[6];
	uint8_t ipadddr[12];

	W25QXX_Read((uint8_t *)parameter, 0, 68);
	parameter[0] = 0x48;
	parameter[1] = returnid;
	parameter[4] = (HW_VERSION >> 8) & 0x00ff;
	parameter[5] = HW_VERSION & 0x00ff;
	parameter[6] = (FW_VERSION >> 8) & 0x00ff;
	parameter[7] = FW_VERSION & 0x00ff;
	parameter[8] = (BOOT_VERSION >> 8) & 0x00ff;
	parameter[9] = BOOT_VERSION & 0x00ff;

	net_get_ipaddress(&gnetif, ipadddr);
	memcpy((parameter + 11), ipadddr, 12);
	//	memcpy((parameter + 15), NETMASK_ADDRESS, 4);
	//	memcpy((parameter + 19), GATEWAY_ADDRESS, 4);

	net_macaddr_t mac = {0};
	net_get_mac_address(&gnetif, macadddr);
	memcpy((parameter + 23), macadddr, 6);
	//	parameter[23] = mac.mac[0];
	//	parameter[24] = mac.mac[1];
	//	parameter[25] = mac.mac[2];
	//	parameter[26] = mac.mac[3];
	//	parameter[27] = mac.mac[4];
	//	parameter[28] = mac.mac[5];

	//	printf("mac:%02X%02X%02X%02X%02X%02X\r\n",
	//		macadddr[0],
	//		macadddr[1],
	//		macadddr[2],
	//		macadddr[3],
	//		macadddr[4],
	//		macadddr[5]);
	parameter[29] = mqttstatus;
	uwCRCValue = user_CRC((uint8_t *)parameter, 68);
	//uwCRCValue = ~uwCRCValue;
	parameter[68] = (uint8_t)(uwCRCValue >> 24 & 0x000000ff);
	parameter[69] = (uint8_t)(uwCRCValue >> 16 & 0x000000ff);
	parameter[70] = (uint8_t)(uwCRCValue >> 8 & 0x000000ff);
	parameter[71] = (uint8_t)(uwCRCValue & 0x000000ff);
}
void savepar(uint8_t *parameter)
{
	uint8_t zero[30] = {0};
	W25QXX_Write((uint8_t *)zero, 30, 30);
	W25QXX_Write((uint8_t *)parameter, 0, 62);
}

void saveuser(uint8_t *parameter)
{

	W25QXX_Write((uint8_t *)parameter, 62, 26);
}

void loaduser(uint8_t *parameter)
{

	W25QXX_Read((uint8_t *)parameter, 62, 26);
}

void loadtime(uint8_t *parameter)
{
	STDATETIME time;
	uint8_t time_buf[15];
	__IO uint32_t uwCRCValue = 0;
	memcpy(parameter + 2, B3Status, reB3Statuslen);
	parameter[14] = 0x0; //dianliu
	parameter[15] = 0x0;
	UpdateDateTime(&time);
	sprintf((char *)time_buf,
			"%04d%02d%02d%02d%02d%02d",
			time.year + 2000,
			time.month,
			time.day,
			time.hour,
			time.minute,
			time.second);
	printf("%s\n", time_buf);
	memcpy(parameter + 16, time_buf, 14);
	parameter[0] = 0x22;
	parameter[1] = (returnid + 4);
	uwCRCValue = user_CRC(parameter, 30);
	//uwCRCValue = ~uwCRCValue;
	parameter[30] = (uint8_t)(uwCRCValue >> 24 & 0x000000ff);
	parameter[31] = (uint8_t)(uwCRCValue >> 16 & 0x000000ff);
	parameter[32] = (uint8_t)(uwCRCValue >> 8 & 0x000000ff);
	parameter[33] = (uint8_t)(uwCRCValue & 0x000000ff);
	//W25QXX_Read((uint8_t*)parameter, 60, 26);
}
