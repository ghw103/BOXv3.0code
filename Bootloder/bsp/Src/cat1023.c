
#include "cat1023.h"
#include "i2c.h"
//#include "cmsis_os.h"
#include "stm32f4xx_nucleo_144.h"


void I2C_EE_PageWrite(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite);

uint8_t I2C_EEPROM_WriteBuffer(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t Length)
{
	uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0;

	Addr = WriteAddr % EEPROM_PAGESIZE;
	count = EEPROM_PAGESIZE - Addr;
	NumOfPage =  Length / EEPROM_PAGESIZE;
	NumOfSingle = Length % EEPROM_PAGESIZE;
 
	/* If WriteAddr is I2C_PageSize aligned  */
	if (Addr == 0) 
	{
		/* If NumByteToWrite < I2C_PageSize */
		if (NumOfPage == 0) 
		{
			I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
		}
		/* If NumByteToWrite > I2C_PageSize */
		else  
		{
			while (NumOfPage--)
			{
				I2C_EE_PageWrite(pBuffer, WriteAddr, EEPROM_PAGESIZE); 
				WriteAddr +=  EEPROM_PAGESIZE;
				pBuffer += EEPROM_PAGESIZE;
			}

			if (NumOfSingle != 0)
			{
				I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
			}
		}
	}
	/* If WriteAddr is not I2C_PageSize aligned  */
	else 
	{
		/* If NumByteToWrite < I2C_PageSize */
		if (NumOfPage == 0) 
		{
			I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
		}
		/* If NumByteToWrite > I2C_PageSize */
		else
		{
			Length -= count;
			NumOfPage =  Length / EEPROM_PAGESIZE;
			NumOfSingle = Length % EEPROM_PAGESIZE;	
      
			if (count != 0)
			{  
				I2C_EE_PageWrite(pBuffer, WriteAddr, count);

				WriteAddr += count;
				pBuffer += count;
			} 
      
			while (NumOfPage--)
			{
				I2C_EE_PageWrite(pBuffer, WriteAddr, EEPROM_PAGESIZE);

				WriteAddr +=  EEPROM_PAGESIZE;
				pBuffer += EEPROM_PAGESIZE;  
			}
			if (NumOfSingle != 0)
			{
				I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle); 
			}
		}
	}  
	return 1;
}


 uint8_t I2C_EEPROM_ReadBuffer(uint16_t Memory_Address, uint8_t *pBuffer, uint16_t Length)
{
	if (HAL_I2C_Mem_Read_DMA(&hi2c3, (uint16_t)CAT1023_ADDR_READ, Memory_Address, I2C_MEMADD_SIZE_8BIT, (uint8_t *)pBuffer, Length) != HAL_OK)
	{
		/* Reading process Error */
		//Error_Handler();
	}

	/* Wait for the end of the transfer */
		while (HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY)
		{
		}
	return 0;
}
/**
  * @brief  Writes more than one byte to the EEPROM with a single WRITE
  *   cycle. The number of byte can't exceed the EEPROM page size.
  * @param pBuffer : pointer to the buffer containing the data to be 
  *   written to the EEPROM.
  * @param WriteAddr : EEPROM's internal address to write to.
  * @param NumByteToWrite : number of bytes to write to the EEPROM.
  * @retval : None
  */
void I2C_EE_PageWrite(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
{
	if (HAL_I2C_Mem_Write_DMA(&hi2c3, (uint16_t)CAT1023_ADDR_WRITE, WriteAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)pBuffer, NumByteToWrite) != HAL_OK)
	{
		/* Writing process Error */
		//	Error_Handler();
	}

	/* Wait for the end of the transfer */
	/*  Before starting a new communication transfer, you need to check the current   
	  state of the peripheral; if it�s busy you need to wait for the end of current
	  transfer before starting a new one.
	  For simplicity reasons, this example is just waiting till the end of the 
	  transfer, but application may perform other tasks while transfer operation
	  is ongoing. */
	while (HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY)
	{
	}

	/* Check if the EEPROM is ready for a new operation */
	while (HAL_I2C_IsDeviceReady(&hi2c3, CAT1023_ADDR_WRITE, EEPROM_MAX_TRIALS, I2Cx_TIMEOUT_MAX) == HAL_TIMEOUT) ;

	/* Wait for the end of the transfer */
	while (HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY)
	{
	}
	
	
}
/**
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	/* Turn LED1 on: Transfer in transmission process is correct */
	BSP_LED_On(LED_GREEN);
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	/* Turn LED2 on: Transfer in reception process is correct */
	BSP_LED_On(LED_GREEN);
}

/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
	/* Turn LED3 on: Transfer error in reception/transmission process */
	BSP_LED_On(LED_RED);
}