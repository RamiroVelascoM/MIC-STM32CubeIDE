/*
 * MPU6050.c
 *
 *  Created on: May 21, 2024
 *      Author: Gerónimo Spiazzi
 */

#include "mpu6050.h"

uint8_t bufData[14];

void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data = 0;

    // Verifies connection with MPU6050
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I, 1, &data, 1, MPU_TIMEOUT);

    if (data == WHO_AM_I_DEFAULT_VALUE){
        // Wake up MPU6050
        data = 0;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, POWER_MANAGEMENT_REG, 1, &data, 1, MPU_TIMEOUT);

        // Set accelerometer range (+/- 2g, 4g, 8g, 16g)
        data = 0; // 2g (default)
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, MPU_TIMEOUT);

        // Set gyroscope range (+/- 250, 500, 1000, 2000 degree/s)
        data = 0; // 250 degree/s (default)
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, MPU_TIMEOUT);
    }
}


void MPU6050_Read_Data_DMA(I2C_HandleTypeDef *hi2c){
	HAL_I2C_Mem_Read_DMA(hi2c, MPU6050_ADDR, ACCEL_XOUT_REG, 1, bufData, 14);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	extern _sMPUData mpuValues;
	/*
	mpuValues.accelX[0] = bufData[1];
	mpuValues.accelX[1] = bufData[0];

	mpuValues.accelY[0] = bufData[3];
	mpuValues.accelY[1] = bufData[2];

	mpuValues.accelZ[0] = bufData[5];
	mpuValues.accelZ[1] = bufData[4];

	mpuValues.gyroX[0] = bufData[9];
	mpuValues.gyroX[1] = bufData[8];

	mpuValues.gyroY[0] = bufData[11];
	mpuValues.gyroY[1] = bufData[10];

	mpuValues.gyroZ[0] = bufData[13];
	mpuValues.gyroZ[1] = bufData[12];
	*/
	mpuValues.buffer[0] = bufData[1];
	mpuValues.buffer[1] = bufData[0];
	mpuValues.buffer[2] = bufData[3];
	mpuValues.buffer[3] = bufData[2];
	mpuValues.buffer[4] = bufData[5];
	mpuValues.buffer[5] = bufData[4];
	mpuValues.buffer[6] = bufData[9];
	mpuValues.buffer[7] = bufData[8];
	mpuValues.buffer[8] = bufData[11];
	mpuValues.buffer[9] = bufData[10];
	mpuValues.buffer[10] = bufData[13];
	mpuValues.buffer[11] = bufData[12];
}
