/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Socket_RTOS/Src/app_ethernet.c 
  * @author  MCD Application Team
  * @brief   Ethernet specefic module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
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
#include "main.h"
#include "lwip/dhcp.h"
#include "app_ethernet.h"
#include "stm32f4xx_nucleo_144.h"
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */
#include "ethernetif.h"
#include "app_ethernet.h"

#include "string.h"
#include <stdlib.h>   /* atoi() */
#include <stdbool.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];
uint8_t dhcp_flage= 0;
extern	 osSemaphoreId Netif_LinkSemaphore ;

/* Private variables ---------------------------------------------------------*/
#ifdef USE_DHCP
#define MAX_DHCP_TRIES  4
__IO uint8_t DHCP_state = DHCP_OFF;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  */
void Netif_Config(void)
{
	ip_addr_t ipaddr;
	ip_addr_t netmask;
	ip_addr_t gw;
	if (dhcp_flage)
	{
		ip_addr_set_zero_ip4(&ipaddr);
		ip_addr_set_zero_ip4(&netmask);
		ip_addr_set_zero_ip4(&gw);
	}
	else
	{
		/* IP addresses initialization without DHCP (IPv4) */
		IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
		IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1], NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
		IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);
		/* USE_DHCP */
	}

  
	/* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
	struct ip_addr *netmask, struct ip_addr *gw,
	void *state, err_t (* init)(struct netif *netif),
	err_t (* input)(struct pbuf *p, struct netif *netif))
  
	Adds your network interface to the netif_list. Allocate a struct
	netif and pass a pointer to this structure as the first argument.
	Give pointers to cleared ip_addr structures when using DHCP,
	or fill them with sane numbers otherwise. The state pointer may be NULL.
  
	The init function pointer must point to a initialization function for
	your ethernet netif interface. The following code illustrates it's use.*/
  
	netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
  
	/*  Registers the default network interface. */
	netif_set_default(&gnetif);
  
	if (netif_is_link_up(&gnetif))
	{
		/* When the netif is fully configured this function must be called.*/
		netif_set_up(&gnetif);
	}
	else
	{
		/* When the netif link is down this function must be called */
		netif_set_down(&gnetif);
	}

	/* Set the link callback function, this function is called on change of link status*/
	netif_set_link_callback(&gnetif, ethernetif_update_config);
  
	/* create a binary semaphore used for informing ethernetif of frame reception */
	osSemaphoreDef(Netif_SEM);
	Netif_LinkSemaphore = osSemaphoreCreate(osSemaphore(Netif_SEM), 1);
  
	link_arg.netif = &gnetif;
	link_arg.semaphore = Netif_LinkSemaphore;
	/* Create the Ethernet link handler thread */
#if defined(__GNUC__)
	osThreadDef(LinkThr, ethernetif_set_link, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 5);
#else
	osThreadDef(LinkThr, ethernetif_set_link, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);
#endif
	osThreadCreate(osThread(LinkThr), &link_arg);
}
/**
  * @brief  Notify the User about the nework interface config status 
  * @param  netif: the network interface
  * @retval None
  */
void User_notification(struct netif *netif) 
{
  if (netif_is_up(netif))
 {
	 if (dhcp_flage)
	 {
		 DHCP_state = DHCP_START; 
	 }
    /* Update DHCP state machine */
  
	 else
	 {
		#ifdef USE_LCD
			uint8_t iptxt[20];
			sprintf((char *)iptxt, "%s", ip4addr_ntoa((const ip4_addr_t *)&netif->ip_addr));
			printf ("Static IP address: %s\n", iptxt);
		#else    
			/* Turn On LED 1 to indicate ETH and LwIP init success*/
			BSP_LED_On(LED1);
		#endif /* USE_LCD */
	}
  }
  else
  {  
	  if (dhcp_flage)
	  {
		  /* Update DHCP state machine */
		  DHCP_state = DHCP_LINK_DOWN;
		  /* USE_DHCP */ 
	  }

#ifdef USE_LCD
	  printf("The network cable is not connected \n");
#else    
    /* Turn On LED 2 to indicate ETH and LwIP init error */
    BSP_LED_On(LED2);
#endif /* USE_LCD */
  } 
}

/**
  * @brief  This function notify user about link status changement.
  * @param  netif: the network interface
  * @retval None
  */
void ethernetif_notify_conn_changed(struct netif *netif)
{
//#ifndef USE_DHCP

		ip_addr_t ipaddr;
		ip_addr_t netmask;
		ip_addr_t gw; 

 
//#endif
  
  if(netif_is_link_up(netif))
  {
#ifdef USE_LCD        
	  printf("The network cable is now connected \n");
#else
    BSP_LED_Off(LED2);
    BSP_LED_On(LED1);
#endif /* USE_LCD */
    
//#ifdef USE_DHCP
	  if(dhcp_flage)
	  {
		  /* Update DHCP state machine */
		  DHCP_state = DHCP_START;
	  }
	  else
	  {
	
//#else

	  /* IP addresses initialization without DHCP (IPv4) */
	  IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
	  IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1], NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
	  IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]); 
 
    netif_set_addr(netif, &ipaddr , &netmask, &gw);
    
#ifdef USE_LCD        
    uint8_t iptxt[20];
    sprintf((char *)iptxt, "%s", ip4addr_ntoa((const ip4_addr_t *)&netif->ip_addr));
		  printf("Static IP address: %s\n", iptxt);
#endif /* USE_LCD */
		}
//#endif /* USE_DHCP */   
    
    /* When the netif is fully configured this function must be called.*/
    netif_set_up(netif);     
  }
  else
  {
//#ifdef USE_DHCP
	  if(dhcp_flage)
	  {
		  /* Update DHCP state machine */
		  DHCP_state = DHCP_LINK_DOWN;
	  }
//#endif /* USE_DHCP */
    
    /*  When the netif link is down this function must be called.*/
    netif_set_down(netif);
    
#ifdef USE_LCD
	  printf("The network cable is not connected \n");
#else
    BSP_LED_Off(LED1);
    BSP_LED_On(LED2);
#endif /* USE_LCD */    
  }
}
int net_get_mac_address(struct netif *netif,net_macaddr_t* macAddress)
{
	
	memcpy(macAddress->mac, netif->hwaddr, netif->hwaddr_len);
	return 0;
}
#if defined(USE_DHCP)
/**
  * @brief  DHCP Process
  * @param  argument: network interface
  * @retval None
  */
void DHCP_thread(void const * argument)
{
  struct netif *netif = (struct netif *) argument;
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
  struct dhcp *dhcp;
#ifdef USE_LCD 
  uint8_t iptxt[20];
#endif
  
  for (;;)
  {
    switch (DHCP_state)
    {
    case DHCP_START:
      {
        ip_addr_set_zero_ip4(&netif->ip_addr);
        ip_addr_set_zero_ip4(&netif->netmask);
        ip_addr_set_zero_ip4(&netif->gw);       
        dhcp_start(netif);
        DHCP_state = DHCP_WAIT_ADDRESS;
#ifdef USE_LCD
	      printf("  State: Looking for DHCP server ...\n");
#endif
      }
      break;
      
    case DHCP_WAIT_ADDRESS:
      {                
        if (dhcp_supplied_address(netif)) 
        {
          DHCP_state = DHCP_ADDRESS_ASSIGNED;	
         
#ifdef USE_LCD 
          sprintf((char *)iptxt, "%s", ip4addr_ntoa((const ip4_addr_t *)&netif->ip_addr));   
	        printf("IP address assigned by a DHCP server: %s\n", iptxt);
#else
          BSP_LED_On(LED1);   
#endif 
        }
        else
        {
          dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);
    
          /* DHCP timeout */
          if (dhcp->tries > MAX_DHCP_TRIES)
          {
            DHCP_state = DHCP_TIMEOUT;
            
            /* Stop DHCP */
            dhcp_stop(netif);
            
            /* Static address used */
//            IP_ADDR4(&ipaddr, IP_ADDR0 ,IP_ADDR1 , IP_ADDR2 , IP_ADDR3 );
//            IP_ADDR4(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
//            IP_ADDR4(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
	            IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
	          IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1], NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
	          IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]); 
            netif_set_addr(netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
            
#ifdef USE_LCD  
            sprintf((char *)iptxt, "%s", ip4addr_ntoa((const ip4_addr_t *)&netif->ip_addr));
	          printf("DHCP Timeout !! \n");
	          printf("Static IP address: %s\n", iptxt);  
#else
            BSP_LED_On(LED1);  
#endif
          }
        }
      }
      break;
  case DHCP_LINK_DOWN:
    {
      /* Stop DHCP */
      dhcp_stop(netif);
      DHCP_state = DHCP_OFF; 
    }
    break;
    default: break;
    }
    
    /* wait 250 ms */
    osDelay(250);
  }
}
#endif  /* USE_DHCP */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
