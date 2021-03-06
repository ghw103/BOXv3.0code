/**
  ******************************************************************************
  * @file    
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOXAPI_H
#define __BOXAPI_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ipstastuslen 68
#define ipsetstastuslen 92
#define setmqttbufflen 135
#define setusername 34

#define ipaddrflash 32
#define ipaddrflash 32
#define returnid 201
  extern uint8_t mqttstatus;
  uint32_t user_CRC(uint8_t *pBuff, uint32_t len);

  void restory();
  void SetMac(void);
  void loaduppar(char *ipaddr, uint16_t *port);
  void loadipar(uint8_t *ipaddr, uint8_t *NETMASK, uint8_t *gatway);
  void loadMQTTpar(uint8_t *mqttlock, char *ipaddr, uint16_t *port);
  void loadDHCPset(uint8_t *DHCP);
  void loadpar(uint8_t *parameter);
  void savepar(uint8_t *parameter);
  void loadparmqttbuf(uint8_t *parameter);
  void saveuser(uint8_t *parameter);
  void loaduser(uint8_t *parameter);
  void loadtime(uint8_t *parameter);
  void saveparmqttbuf(uint8_t *parameter);
  uint8_t B3_eeread(uint8_t *pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead);
  uint8_t B3_eewrite(uint8_t *pBuffer, uint16_t ReadAddr, uint16_t NumByteToWrite);
#endif /* __BOXAPI_H */