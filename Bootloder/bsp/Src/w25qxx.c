#include "w25qxx.h" 


//////////////////////////////////////////////////////////////////////////////////	 
								  
////////////////////////////////////////////////////////////////////////////////// 	
 
uint16_t W25QXX_TYPE=W25Q128;	//Ĭ����W25Q128

//4KbytesΪһ��Sector
//16������Ϊ1��Block
//W25Q128
//����Ϊ16M�ֽ�,����128��Block,4096��Sector 
													 
//��ʼ��SPI FLASH��IO��
void W25QXX_Init(void)
{ 
	const uint8_t TEXT_Buffer[] = { "xieruceshi nihao  nenggouxieru " };
	#define SIZE sizeof(TEXT_Buffer)
	uint8_t datatemp[SIZE];
	uint8_t retry=100;
	//  GPIO_InitTypeDef  GPIO_InitStructure;
	// 
	//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��
	//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//ʹ��GPIOGʱ��

	//	  //GPIOB14
	//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;//PB14
	//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
	//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	//  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��

	//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//PG7
	//  GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��
	// 
	//	GPIO_SetBits(GPIOG,GPIO_Pin_7);//PG7���1,��ֹNRF����SPI FLASH��ͨ�� 
	//	W25QXX_CS_H;			//SPI FLASH��ѡ��
	//	SPI1_Init();		   			//��ʼ��SPI
	//	SPI1_SetSpeed(SPI_BaudRatePrescaler_2);		//����Ϊ42Mʱ��,����ģʽ 
		while(retry--)
	{
		W25QXX_TYPE = W25QXX_ReadID(); 	//��ȡFLASH ID.
		printf("ID:%x\n", W25QXX_TYPE);
		if (W25QXX_TYPE == W25Q128)
		{
			break;
		}
		HAL_Delay(100);
		
	}
//		W25QXX_Write((uint8_t*)TEXT_Buffer, 500, SIZE);
//		W25QXX_Read((uint8_t*)datatemp, 500, SIZE);
//		printf("spidata��\n%s \n", datatemp);	
//	W25QXX_Erase_Chip();
	
}  

//��ȡW25QXX��״̬�Ĵ���
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
uint8_t W25QXX_ReadSR(void)   
{  
	uint8_t data[2]={W25X_ReadStatusReg,0Xff};   
	W25QXX_CS_L;                             //ʹ������   
//	SPI1_ReadWriteByte(W25X_ReadStatusReg);   
if(HAL_SPI_TransmitReceive(&FLASH_SPI, (uint8_t*)data, (uint8_t *)data, 2,100) != HAL_OK)
  {
    /* Transfer error in transmission process */
			printf("spi_dma_Error!");
//    Error_Handler();
  }	

	//���Ͷ�ȡ״̬�Ĵ�������    
//	byte=SPI1_ReadWriteByte(0Xff);             //��ȡһ���ֽ�  
	W25QXX_CS_H;                            //ȡ��Ƭѡ     
	return data[1];   
} 
//дW25QXX״̬�Ĵ���
//ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д!!!
void W25QXX_Write_SR(uint8_t sr)   
{   
	uint8_t data[2]={W25X_WriteStatusReg,sr};   
	W25QXX_CS_L;  //ʹ������  
if(HAL_SPI_Transmit(&FLASH_SPI, (uint8_t*)data, 2,100) != HAL_OK)
  {
    /* Transfer error in transmission process */
		
//    Error_Handler();
  }	 


//	SPI1_ReadWriteByte(W25X_WriteStatusReg);   //����дȡ״̬�Ĵ�������    
//	SPI1_ReadWriteByte(sr);               //д��һ���ֽ�  
	W25QXX_CS_H;                            //ȡ��Ƭѡ     	      
}   
//W25QXXдʹ��	
//��WEL��λ   
void W25QXX_Write_Enable(void)   
{
	uint8_t data[1]={W25X_WriteEnable};   
	W25QXX_CS_L; //ʹ������ 
if(HAL_SPI_Transmit(&FLASH_SPI, (uint8_t*)data, 1,100) != HAL_OK)
  {
    /* Transfer error in transmission process */
			//LCD_ShowString(200,700,210,24,24,"spi_dma_Error!");
//    Error_Handler();
  }	 
	while (HAL_SPI_GetState(&FLASH_SPI) != HAL_SPI_STATE_READY)
  {
  } 	
//    SPI1_ReadWriteByte(W25X_WriteEnable);      //����дʹ��  
	W25QXX_CS_H;                            //ȡ��Ƭѡ     	      
} 
//W25QXXд��ֹ	
//��WEL����  
void W25QXX_Write_Disable(void)   
{  
	uint8_t data[1]={W25X_WriteDisable}; 
	W25QXX_CS_L;                            //ʹ������  
if(HAL_SPI_Transmit(&FLASH_SPI, (uint8_t*)data,1,100) != HAL_OK)
  {
    /* Transfer error in transmission process */
			//LCD_ShowString(200,700,210,24,24,"spi_dma_Error!");
//    Error_Handler();
  }	
	while (HAL_SPI_GetState(&FLASH_SPI) != HAL_SPI_STATE_READY)
  {
  } 
//    SPI1_ReadWriteByte(W25X_WriteDisable);     //����д��ָֹ��    
	W25QXX_CS_H;                            //ȡ��Ƭѡ     	      
} 		
//��ȡоƬID
//����ֵ����:				   
//0XEF13,��ʾоƬ�ͺ�ΪW25Q80  
//0XEF14,��ʾоƬ�ͺ�ΪW25Q16    
//0XEF15,��ʾоƬ�ͺ�ΪW25Q32  
//0XEF16,��ʾоƬ�ͺ�ΪW25Q64 
//0XEF17,��ʾоƬ�ͺ�ΪW25Q128 	  
uint16_t W25QXX_ReadID(void)
{
	uint8_t data[6]={0x90,0x00,0x00,0x00,0x00,0x00}; 
	uint16_t Temp = 0;	  
	W25QXX_CS_L;	
	if (HAL_SPI_TransmitReceive(&FLASH_SPI, (uint8_t*)data, (uint8_t *)data, 6,100) != HAL_OK)
  {
    /* Transfer error in transmission process */
	  printf("spi_dma_Error!");
  }
//	while (HAL_SPI_GetState(&FLASH_SPI) != HAL_SPI_STATE_READY)
//  {
//  } 	
//	SPI1_ReadWriteByte(0x90);//���Ͷ�ȡID����	    
//	SPI1_ReadWriteByte(0x00); 	    
//	SPI1_ReadWriteByte(0x00); 	    
//	SPI1_ReadWriteByte(0x00); 	 			   
//	Temp|=SPI1_ReadWriteByte(0xFF)<<8;  
//	Temp|=SPI1_ReadWriteByte(0xFF);	 
	Temp|=data[4]<<8;  
	Temp|=data[5];	 
	W25QXX_CS_H;				    
	return Temp;
}   		    
//��ȡSPI FLASH  
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)   
{ 
	uint8_t data[6]={W25X_ReadData, (ReadAddr & 0xFF0000) >> 16, (ReadAddr & 0xFF00) >> 8, ReadAddr & 0xFF}; 										    
	W25QXX_CS_L;                            //ʹ������  
if(HAL_SPI_Transmit(&FLASH_SPI, (uint8_t*)data, 4,100) != HAL_OK)
  {
    /* Transfer error in transmission process */
 //  LCD_ShowString(200,700,210,24,24,"spi_dma_Error!");
  }	
 	
//    SPI1_ReadWriteByte(W25X_ReadData);         //���Ͷ�ȡ����   
//    SPI1_ReadWriteByte((uint8_t)((ReadAddr)>>16));  //����24bit��ַ    
//    SPI1_ReadWriteByte((uint8_t)((ReadAddr)>>8));   
//    SPI1_ReadWriteByte((uint8_t)ReadAddr);   
	if(HAL_SPI_TransmitReceive(&FLASH_SPI, (uint8_t*)0xff, (uint8_t *)pBuffer, NumByteToRead,100) != HAL_OK)
  {
    /* Transfer error in transmission process */
  // LCD_ShowString(200,700,210,24,24,"spi_dma_Error!");
  }

//    for(i=0;i<NumByteToRead;i++)
//	{ 
//        pBuffer[i]=SPI1_ReadWriteByte(0XFF);   //ѭ������  
//    }
	W25QXX_CS_H;  				    	      
}  
//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!	 
void W25QXX_Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
  W25QXX_Write_Enable();                  //SET WEL 
	W25QXX_CS_L;                            //ʹ������   
	uint8_t data[4]={W25X_PageProgram,(WriteAddr)>>16,(WriteAddr)>>8,WriteAddr}; 
//    SPI1_ReadWriteByte(W25X_PageProgram);      //����дҳ����   
//    SPI1_ReadWriteByte((uint8_t)((WriteAddr)>>16)); //����24bit��ַ    
//    SPI1_ReadWriteByte((uint8_t)((WriteAddr)>>8));   
//    SPI1_ReadWriteByte((uint8_t)WriteAddr); 
  		if(HAL_SPI_Transmit(&FLASH_SPI, (uint8_t *)data,4,100) != HAL_OK)
  {
    /* Transfer error in transmission process */
  // LCD_ShowString(200,700,210,24,24,"spi_dma_Error!");
  }

//    for(i=0;i<NumByteToWrite;i++)SPI1_ReadWriteByte(pBuffer[i]);//ѭ��д��  
		if(HAL_SPI_Transmit(&FLASH_SPI, (uint8_t *)pBuffer, NumByteToWrite,100) != HAL_OK)
  {
    /* Transfer error in transmission process */
  // LCD_ShowString(200,700,210,24,24,"spi_dma_Error!");
  }

	W25QXX_CS_H;                            //ȡ��Ƭѡ 
	W25QXX_Wait_Busy();					   //�ȴ�д�����
} 
//�޼���дSPI FLASH 
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ���� 
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 			 		 
	uint16_t pageremain;	   
	pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//д�������
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=NumByteToWrite; 	  //����256���ֽ���
		}
	};	    
} 
//дSPI FLASH  
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)						
//NumByteToWrite:Ҫд����ֽ���(���65535)   
uint8_t W25QXX_BUFFER[4096];		 
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;	   
 	uint16_t i;    
	uint8_t * W25QXX_BUF;	  
   	W25QXX_BUF=W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;//������ַ  
	secoff=WriteAddr%4096;//�������ڵ�ƫ��
	secremain=4096-secoff;//����ʣ��ռ��С   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//������
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//������4096���ֽ�
	while(1) 
	{	
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//��Ҫ����
		{
//			LCD_ShowString(200,500,200,16,16,"xuyaovacgu!");
			W25QXX_Erase_Sector(secpos);//�����������
			for(i=0;i<secremain;i++)	   //����
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//д����������  
		}
		
	else
//		LCD_ShowString(200,500,200,16,16,"no!");
		W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(NumByteToWrite==secremain)break;//д�������
		else//д��δ����
		{
			secpos++;//������ַ��1
			secoff=0;//ƫ��λ��Ϊ0 	 

		   	pBuffer+=secremain;  //ָ��ƫ��
			WriteAddr+=secremain;//д��ַƫ��	   
		   	NumByteToWrite-=secremain;				//�ֽ����ݼ�
			if(NumByteToWrite>4096)secremain=4096;	//��һ����������д����
			else secremain=NumByteToWrite;			//��һ����������д����
		}	 
	};	 
}
//��������оƬ		  
//�ȴ�ʱ�䳬��...
void W25QXX_Erase_Chip(void)   
{    
		uint8_t data[1]={W25X_ChipErase}; 	
    W25QXX_Write_Enable();                  //SET WEL 
    W25QXX_Wait_Busy();   
  	W25QXX_CS_L;                            //ʹ������   
	if(HAL_SPI_Transmit_DMA(&FLASH_SPI, (uint8_t *)data, 1) != HAL_OK)
  {
    /* Transfer error in transmission process */
  // LCD_ShowString(200,700,210,24,24,"spi_dma_Error!");
  }
	while (HAL_SPI_GetState(&FLASH_SPI) != HAL_SPI_STATE_READY)
  {
  } 
//    SPI1_ReadWriteByte(W25X_ChipErase);        //����Ƭ��������  
	W25QXX_CS_H;                            //ȡ��Ƭѡ   
printf("flash cachu...\n");	
	W25QXX_Wait_Busy();   				   //�ȴ�оƬ��������
printf("flash cachuok!");
}   
//����һ������
//Dst_Addr:������ַ ����ʵ����������
//����һ��ɽ��������ʱ��:150ms
void W25QXX_Erase_Sector(uint32_t Dst_Addr)   
{  
	uint8_t data[4]={W25X_SectorErase,(Dst_Addr)>>16,(Dst_Addr)>>8,Dst_Addr}; 
	//����falsh�������,������   
// 	printf("fe:%x\r\n",Dst_Addr);	  
 	Dst_Addr*=4096;
    W25QXX_Write_Enable();                  //SET WEL 	 
    W25QXX_Wait_Busy();   
  	W25QXX_CS_L;                            //ʹ������   
		

		if(HAL_SPI_Transmit(&FLASH_SPI, (uint8_t *)data, 4,100) != HAL_OK)
  {
    /* Transfer error in transmission process */
//   LCD_ShowString(200,700,210,24,24,"spi_dma_Error!");
  }

//    SPI1_ReadWriteByte(W25X_SectorErase);      //������������ָ�� 
//    SPI1_ReadWriteByte((uint8_t)((Dst_Addr)>>16));  //����24bit��ַ    
//    SPI1_ReadWriteByte((uint8_t)((Dst_Addr)>>8));   
//    SPI1_ReadWriteByte((uint8_t)Dst_Addr);  
	W25QXX_CS_H;                            //ȡ��Ƭѡ     	      
    W25QXX_Wait_Busy();   				   //�ȴ��������
}  
//�ȴ�����
void W25QXX_Wait_Busy(void)   
{   
	while((W25QXX_ReadSR()&0x01)==0x01);   // �ȴ�BUSYλ���
}  
//�������ģʽ
void W25QXX_PowerDown(void)   
{ 
		uint8_t data[1]={W25X_PowerDown}; 	
  	W25QXX_CS_L; //ʹ������   
	if(HAL_SPI_Transmit_DMA(&FLASH_SPI, (uint8_t *)data, 1) != HAL_OK)
  {
    /* Transfer error in transmission process */
//   LCD_ShowString(200,700,210,24,24,"spi_dma_Error!");
  }	
	while (HAL_SPI_GetState(&FLASH_SPI) != HAL_SPI_STATE_READY)
  {
  } 
//    SPI1_ReadWriteByte(W25X_PowerDown);        //���͵�������  
	W25QXX_CS_H;                            //ȡ��Ƭѡ     	      
    HAL_Delay(1);                               //�ȴ�TPD  
}   
//����
void W25QXX_WAKEUP(void)   
{  
  		uint8_t data[1]={W25X_ReleasePowerDown}; 	
  	W25QXX_CS_L; //ʹ������   
	if(HAL_SPI_Transmit_DMA(&FLASH_SPI, (uint8_t *)data, 1) != HAL_OK)
  {
    /* Transfer error in transmission process */
//   LCD_ShowString(200,700,210,24,24,"spi_dma_Error!");
  }	 
	while (HAL_SPI_GetState(&FLASH_SPI) != HAL_SPI_STATE_READY)
  {
  } 	 
//    SPI1_ReadWriteByte(W25X_ReleasePowerDown);   //  send W25X_PowerDown command 0xAB    
	W25QXX_CS_H;                            //ȡ��Ƭѡ     	      
    HAL_Delay(1);                               //�ȴ�TRES1
}   


























