#include "stm32f4xx_hal.h"
#include "mathTool.h"
#include "delay.h"
void MPU6000_Init(void);

void MPU6000_ReadAcc(Vector3f_t* acc);
void MPU6000_ReadGyro(Vector3f_t* gyro);
void MPU6000_ReadTemp(float* temp);
