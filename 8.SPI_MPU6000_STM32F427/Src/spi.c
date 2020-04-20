/*
 * spi.c
 *
 *  Created on: Feb 15, 2020
 *      Author: kaidi wang
 */

#include "spi.h"

SPI_HandleTypeDef hspi1;
/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    //Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
/**********************************************************************************************************
*函 数 名: Spi_GyroEnable
*功能说明: 陀螺仪CS脚使能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi_GyroEnable(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
}

/**********************************************************************************************************
*函 数 名: Spi_GyroDisable
*功能说明: 陀螺仪CS脚失能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi_GyroDisable(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
}

//单字节的通过SPI的读写函数
uint8_t SPI_SingleReadandWrite(uint8_t TxData)
{
	uint8_t Rxdata;
	HAL_SPI_TransmitReceive(&hspi1,&TxData,&Rxdata,1, 1000);
	return Rxdata;          		    //返回收到的数据
}

//多个字节通过SPI的读写函数
void SPI_MultiWriteAndRead( uint8_t Txdata, uint8_t *Rxdata, int len)
{

	uint8_t b;
	while(len--)
	{
		HAL_SPI_TransmitReceive(&hspi1,&Txdata,&b,1, 50);
	}
	if(Rxdata)
	{
		*(Rxdata++) = b;
	}
}

/**********************************************************************************************************
*函 数 名: Spi_GyroSingleWrite
*功能说明: 陀螺仪单个寄存器写入
*形    参: 寄存器地址 写入值
*返 回 值: 无
**********************************************************************************************************/
void Spi_GyroSingleWrite(uint8_t reg, uint8_t value)
{
	Spi_GyroEnable();
	//Spi_SingleWirteAndRead(GYRO_SPI, reg);
	SPI_SingleReadandWrite(reg);
	SPI_SingleReadandWrite(value);
	//Spi_SingleWirteAndRead(GYRO_SPI, value);
	Spi_GyroDisable();
}

/**********************************************************************************************************
*函 数 名: Spi_GyroMultiRead
*功能说明: 陀螺仪多个寄存器读出
*形    参: 寄存器地址 读出缓冲区 读出长度
*返 回 值: 无
**********************************************************************************************************/
void Spi_GyroMultiRead(uint8_t reg,uint8_t *data, uint8_t length)
{
	Spi_GyroEnable();
	//Spi_SingleWirteAndRead(GYRO_SPI, reg | 0x80);
	SPI_SingleReadandWrite(reg|0x80);
	SPI_MultiWriteAndRead(reg,data,length);
	//SPI_MultiWriteAndRead(GYRO_SPI, data, NULL, length);
	Spi_GyroDisable();
}
