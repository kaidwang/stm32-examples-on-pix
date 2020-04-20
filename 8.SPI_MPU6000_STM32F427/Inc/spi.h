#include "stm32f4xx_hal.h"
extern SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void);//初始化函数
void Spi_GyroEnable(void);//使能MPU6000
void Spi_GyroDisable(void);//关闭MPU6000

uint8_t SPI_SingleReadandWrite(uint8_t TxData);
void SPI_MultiWriteAndRead( uint8_t Txdata, uint8_t *Rxdata, int len);

void Spi_GyroSingleWrite(uint8_t reg, uint8_t value);
void Spi_GyroMultiRead(uint8_t reg,uint8_t *data, uint8_t length);

void Error_Handler(void);//声明错误处理函数
