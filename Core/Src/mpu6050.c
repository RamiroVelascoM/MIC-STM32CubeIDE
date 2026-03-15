/*
 * MPU6050.c
 *
 * Author: Ramiro Velasco
 *
 */

#include "mpu6050.h"

#include "math.h"
#include <stdbool.h>

#define NUM_SAMPLES			1000
#define SCALE_FACTOR		16384

uint8_t bufData[14];

extern _sMPUData myMPU;

void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data = 0;

    // Verifies connection with MPU6050
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I, 1, &data, 1, MPU_TIMEOUT);

    if (data == WHO_AM_I_DEFAULT_VALUE){
        // Wake up MPU6050
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, POWER_MANAGEMENT_REG, 1, &data, 1, MPU_TIMEOUT);

        // Set accelerometer range of +/- 2g (default)
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, MPU_TIMEOUT);

        // Set gyroscope range of +/- 500 degree/s
        data = 0x08;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, MPU_TIMEOUT);

		// Set Digital Low Pass Filter (44 Hz)
        data = 0x03;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, CONFIG, 1, &data, 1, MPU_TIMEOUT);

		myMPU.AccX = 0, myMPU.AccY = 0, myMPU.AccZ = 0;
		myMPU.GyrX = 0, myMPU.GyrY = 0, myMPU.GyrZ = 0;
		myMPU.OffsetAccX = 0, myMPU.OffsetAccY = 0, myMPU.OffsetAccZ = 0;
		myMPU.OffsetGyrX = 0, myMPU.OffsetGyrY = 0, myMPU.OffsetGyrZ = 0;
		myMPU.RawAccX = 0, myMPU.RawAccY = 0, myMPU.RawAccZ = 0;
		myMPU.RawGyrX = 0, myMPU.RawGyrY = 0, myMPU.RawGyrZ = 0;
		myMPU.Yaw = 0, myMPU.Yaw_x10 = 0, myMPU.rawYaw = 0;

		for (uint8_t i=0; i<14; i++){
			bufData[i] = 0;
			myMPU.buffer[i] = 0;
		}
    }
}

void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c){
	for (uint16_t i=0; i<NUM_SAMPLES; i++){
		HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_REG, 1, bufData, 14, MPU_TIMEOUT);
		myMPU.RawAccX += (int16_t)((bufData[0] << 8) | bufData[1]);
		myMPU.RawAccY += (int16_t)((bufData[2] << 8) | bufData[3]);
		myMPU.RawAccZ += (int16_t)((bufData[4] << 8) | bufData[5]);

		myMPU.RawGyrX += (int16_t)((bufData[8] << 8) | bufData[9]);
		myMPU.RawGyrY += (int16_t)((bufData[10] << 8) | bufData[11]);
		myMPU.RawGyrZ += (int16_t)((bufData[12] << 8) | bufData[13]);
	}
    myMPU.OffsetAccX = (int16_t)(myMPU.RawAccX / NUM_SAMPLES);
    myMPU.OffsetAccY = (int16_t)(myMPU.RawAccY / NUM_SAMPLES);
    myMPU.OffsetAccZ = (int16_t)(myMPU.RawAccZ / NUM_SAMPLES) - SCALE_FACTOR;

    myMPU.OffsetGyrX = (int16_t)(myMPU.RawGyrX / NUM_SAMPLES);
	myMPU.OffsetGyrY = (int16_t)(myMPU.RawGyrY / NUM_SAMPLES);
	myMPU.OffsetGyrZ = (int16_t)(myMPU.RawGyrZ / NUM_SAMPLES);
}

void MPU6050_ReadAll(I2C_HandleTypeDef *hi2c){
	// Read data from MPU
	HAL_I2C_Mem_Read_DMA(hi2c, MPU6050_ADDR, ACCEL_XOUT_REG, 1, bufData, 14);

}

void MPU6050_GetYaw(I2C_HandleTypeDef *hi2c){
	if (myMPU.GyrZ > GYRO_THRESHOLD || myMPU.GyrZ < -GYRO_THRESHOLD){
		myMPU.rawYaw += (int64_t)(myMPU.GyrZ * 10); // 10 DELTA T
	}
	myMPU.Yaw = myMPU.rawYaw/GYRO_SENSITIVITY;
	myMPU.Yaw_x10 = myMPU.Yaw*MPU_PID_SCALE;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	// ACC: GET RAW INFORMATION
	myMPU.AccX = (int16_t)(((bufData[0] << 8) | bufData[1]));
	myMPU.AccY = (int16_t)(((bufData[2] << 8) | bufData[3]));
	myMPU.AccZ = (int16_t)(((bufData[4] << 8) | bufData[5]));
	// ACC: CALCULATE TRUE ACCELERATION
	myMPU.AccX -= myMPU.OffsetAccX;
	myMPU.AccY -= myMPU.OffsetAccY;
	myMPU.AccZ -= myMPU.OffsetAccZ;
	// ACC: LOAD DATA INTO THE BUFFER
	myMPU.buffer[0] = (uint8_t)(myMPU.AccX & 0xFF);
	myMPU.buffer[1] = (uint8_t)((myMPU.AccX >> 8) & 0xFF);
	myMPU.buffer[2] = (uint8_t)(myMPU.AccY & 0xFF);
	myMPU.buffer[3] = (uint8_t)((myMPU.AccY >> 8) & 0xFF);
	myMPU.buffer[4] = (uint8_t)(myMPU.AccZ & 0xFF);
	myMPU.buffer[5] = (uint8_t)((myMPU.AccZ >> 8) & 0xFF);

	// GYR: GET RAW INFORMATION
	myMPU.GyrX = (int16_t)(((bufData[8] << 8) | bufData[9]));
	myMPU.GyrY = (int16_t)(((bufData[10] << 8) | bufData[11]));
	myMPU.GyrZ = (int16_t)(((bufData[12] << 8) | bufData[13]));
	// GYR: CALCULATE TRUE ACCELERATION
	myMPU.GyrX -= myMPU.OffsetGyrX;
	myMPU.GyrY -= myMPU.OffsetGyrY;
	myMPU.GyrZ -= myMPU.OffsetGyrZ;
	// GYR: LOAD DATA INTO THE BUFFER
	myMPU.buffer[6] = (uint8_t)(myMPU.GyrX & 0xFF);
	myMPU.buffer[7] = (uint8_t)((myMPU.GyrX >> 8) & 0xFF);
	myMPU.buffer[8] = (uint8_t)(myMPU.GyrY & 0xFF);
	myMPU.buffer[9] = (uint8_t)((myMPU.GyrY >> 8) & 0xFF);
	myMPU.buffer[10] = (uint8_t)(myMPU.GyrZ & 0xFF);
	myMPU.buffer[11] = (uint8_t)((myMPU.GyrZ >> 8) & 0xFF);

	myMPU.buffer[12] = (uint8_t)(myMPU.Yaw & 0xFF);
	myMPU.buffer[13] = (uint8_t)((myMPU.Yaw >> 8) & 0xFF);
}

void MPU6050_ResetYaw(I2C_HandleTypeDef *hi2c){
	myMPU.GyrZ = 0;
	myMPU.rawYaw = 0;
	myMPU.Yaw = 0;
	myMPU.Yaw_x10 = 0;
}


