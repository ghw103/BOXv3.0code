#include "BoxAPI.h"
//#include "common.h"
#include "crc.h"
#include "mbcrc.h"
#include "cJSON.h"
#include "version.h"
#include "w25qxx.h"
#include  "log.h"
#include "recod.h"
//#include "DAC.h"
#include "R8025t.h"
//#include "lwip.h"
//#include "lwip/init.h"
//#include "lwip/netif.h"
//#if defined ( __CC_ARM )  /* MDK ARM Compiler */
//#include "lwip/sio.h"
//#endif /* MDK ARM Compiler */
//#include "ethernetif.h"
#include "app_ethernet.h"
#include<stdio.h>
#include<stdlib.h>
#include<string.h>

///* Semaphore to signal Ethernet Link state update */
//osSemaphoreId Netif_LinkSemaphore = NULL;
///* Ethernet link thread Argument */
//struct link_str link_arg;

uint8_t B3Macaddr[6] = { 0 };
#define  parparameteraddr 0;



void SetMac(void)
{
	__IO uint32_t uwCRCValue = 0;
	static uint32_t CpuID[3];
	static uint32_t Lock_Code;

	CpuID[0] = *(__I uint32_t *)(0x1FFF7A10 + 0x00);
	CpuID[1] = *(__I uint32_t *)(0x1FFF7A10 + 0x04);
	CpuID[2] = *(__I uint32_t *)(0x1FFF7A10 + 0x08);
  
//	printf("id:%x-%x-%x\n", CpuID[0], CpuID[1], CpuID[2]);  
	
	
	uwCRCValue= HAL_CRC_Accumulate(&hcrc, (uint32_t *)CpuID, 3);
//	printf("crc1: %2X\n", uwCRCValue>>24&0x000000ff);
//	printf("crc2: %2X\n", uwCRCValue >> 16 & 0x000000ff);
//	printf("crc3: %2X\n",((uwCRCValue >> 8 & 0x000000ff )| (uwCRCValue & 0x000000ff)));
////	printf("crc4: %2X\n", uwCRCValue & 0x000000ff);
////	printf("crc4: %2X\n", uwCRCValue & 0x000000ff);
//	printf("crc: %X\n", uwCRCValue);
	B3Macaddr[0] =0x94;
	B3Macaddr[1] =0xDF;
	B3Macaddr[2] =0x4E;
	B3Macaddr[3] = (uint8_t)(uwCRCValue >> 24 & 0x000000ff);
	B3Macaddr[4] = (uint8_t)(uwCRCValue >> 16 & 0x000000ff);
	B3Macaddr[5] = (uint8_t)((uwCRCValue >> 8 & 0x000000ff)| (uwCRCValue & 0x000000ff));
}

void loaduppar(char * ipaddr,uint16_t *port)
{
	uint8_t up[32] = { 0 };
	W25QXX_Read((uint8_t*)up, 28, 32);
	memcpy(ipaddr, up, 30);
	*port = (up[30] << 8 | up[31]);
	//memcpy(port, up + 30, 2);
//	msg_debug("ip:%x\r\n",*port);
}
void loadipar(uint8_t * ipaddr, uint8_t * NETMASK, uint8_t * gatway)
{
	uint8_t ip[18] = { 0 };
	W25QXX_Read((uint8_t*)ip, 9, 18);
	if ((ip[0]) == 0xff)
	{
		/* IP addresses initialization */
		  IP_ADDRESS[0] = 192;
		  IP_ADDRESS[1] = 168;
		  IP_ADDRESS[2] = 1;
		  IP_ADDRESS[3] = 90;
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

	
//	msg_debug("ip:%d %d %d %d\r\n", IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
//	msg_debug("mac:%s\r\n", ip);
	if ((ip[12]) == 0xff)
	{
		SetMac();
	}
	else
	{
		memcpy(B3Macaddr, ip + 12, 6);
	}
	
}
void loadMQTTpar(char * ipaddr, uint16_t *port)
{
	uint8_t mqtt[32] = { 0 };
	W25QXX_Read((uint8_t*)mqtt, 101, 32);
	memcpy(ipaddr, mqtt, 30);
	*port = (mqtt[30] << 8 | mqtt[31]);
//	memcpy(port, mqtt + 30, 2);
}
void loadDHCPset(uint8_t * DHCP)
{
	uint8_t setB3dhcp[2] = { 0 };

	
	W25QXX_Read((uint8_t*)setB3dhcp, 8, 1);
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
void loadparmqttbuf(uint8_t * parameter)
{
	__IO uint32_t uwCRCValue = 0;
	W25QXX_Read((uint8_t*)parameter, 100, 129);
	uwCRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)parameter, 129);
	//uwCRCValue = ~uwCRCValue;
	parameter[129] = (uint8_t)(uwCRCValue >> 24 & 0x000000ff);
	parameter[130] = (uint8_t)(uwCRCValue >> 16 & 0x000000ff);
	parameter[131] = (uint8_t)(uwCRCValue >> 8 & 0x000000ff);
	parameter[132] = (uint8_t)(uwCRCValue & 0x000000ff);
}
void saveparmqttbuf(uint8_t * parameter)
{
//	__IO uint32_t uwCRCValue = 0;
	W25QXX_Write((uint8_t*)parameter, 100, 129);
//	uwCRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)parameter, 129);
	//uwCRCValue = ~uwCRCValue;
//	parameter[129] = (uint8_t)(uwCRCValue >> 24 & 0x000000ff);
//	parameter[130] = (uint8_t)(uwCRCValue >> 16 & 0x000000ff);
//	parameter[131] = (uint8_t)(uwCRCValue >> 8 & 0x000000ff);
//	parameter[132] = (uint8_t)(uwCRCValue & 0x000000ff);
}
void loadsta(uint8_t * parameter)
{
	//W25QXX_Read((uint8_t*)parameter, 300, 28);
	
}
void loadpar(uint8_t * parameter)
{
	__IO uint32_t uwCRCValue = 0;
	W25QXX_Read((uint8_t*)parameter, 0, 66);
	parameter[2] = (HW_VERSION>>8)&0x00ff;
	parameter[3] = HW_VERSION  & 0x00ff;
	parameter[4] = (FW_VERSION >> 8) & 0x00ff;
	parameter[5] = FW_VERSION  & 0x00ff;
	parameter[6] = (BOOT_VERSION >> 8) & 0x00ff;
	parameter[7] = BOOT_VERSION  & 0x00ff;
	uwCRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)parameter, 66);
	//uwCRCValue = ~uwCRCValue;
	parameter[66]=(uint8_t)(uwCRCValue >> 24 & 0x000000ff);
	parameter[67] = (uint8_t)(uwCRCValue >> 16 & 0x000000ff);
	parameter[68] = (uint8_t)(uwCRCValue >> 8 & 0x000000ff);
	parameter[69] = (uint8_t)(uwCRCValue & 0x000000ff);

}
void savepar(uint8_t * parameter)
{
	
	W25QXX_Write((uint8_t*)parameter, 0, 60);
}

void saveuser(uint8_t * parameter)
{
	
	W25QXX_Write((uint8_t*)parameter, 60, 26);
}

void loaduser(uint8_t * parameter)
{
	
	W25QXX_Read((uint8_t*)parameter, 60, 26);
}

void loadtime(uint8_t * parameter)
{
	STDATETIME time;
	uint8_t time_buf[15];
	__IO uint32_t uwCRCValue = 0;
	memcpy(parameter, B3Status, reB3Statuslen);
				
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
	memcpy(parameter + 14, time_buf, 14);
	uwCRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)parameter, 28);
	//uwCRCValue = ~uwCRCValue;
	parameter[28] = (uint8_t)(uwCRCValue >> 24 & 0x000000ff);
	parameter[29] = (uint8_t)(uwCRCValue >> 16 & 0x000000ff);
	parameter[30] = (uint8_t)(uwCRCValue >> 8 & 0x000000ff);
	parameter[31] = (uint8_t)(uwCRCValue & 0x000000ff);
	//W25QXX_Read((uint8_t*)parameter, 60, 26);
}
