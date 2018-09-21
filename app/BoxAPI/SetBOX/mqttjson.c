/**
 ******************************************************************************
 * @file    Src/.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    4-April-2016
 * @brief   This file provides all the IAP command functions.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

#include "mqttjson.h"
#include "cjson.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void login(STDATETIME time)
{
	//	STDATETIME time;
	uint8_t time_buf[20];
	cJSON *usr;
	cJSON *arry;
	cJSON *root, *js_body;
	//

	sprintf((char *)time_buf,
			"%04d%02d%02d%02d%02d%02d",
			time.year + 2000,
			time.month,
			time.day,
			time.hour,
			time.minute,
			time.second);
	//
	//	  printf("%s\n", time_buf);

	root = cJSON_CreateObject();				  //���������ݶ���
	cJSON_AddStringToObject(root, "type", "BOX"); //�����ֵ�����ַ���
	cJSON_AddStringToObject(root, "time", (char *)time_buf);
	cJSON_AddItemToObject(root, "machineInfo", usr = cJSON_CreateObject());
	cJSON_AddStringToObject(usr, "machineId", "0001"); //�����ֵ�����ַ���
	cJSON_AddStringToObject(usr, "topic", "test");
	// cJSON_AddItemToObject(root,"body", js_body = cJSON_CreateArray());
	//   cJSON_AddNumberToObject(usr,"num",1);  //������

	char *out = cJSON_Print(root); //��json��ʽ��ӡ�������ַ�����ʽ
	printf("%s\n", out);

	// �ͷ��ڴ�
	cJSON_Delete(root);

	free(out);
}
