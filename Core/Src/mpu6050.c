/*
 * MPU6050.c
 *
 * Author: Ramiro Velasco
 *
 */

#include "mpu6050.h"

#include "math.h"
#include <stdbool.h>

#define READING_ALL			0
#define READING_ACC 		1
#define READING_GYR 		2
#define NUM_SAMPLES			1000
#define SCALE_FACTOR		16384
//#define TIME_FACTOR			10/1000

uint8_t bufData[14];
//uint8_t buf_FIFO[42];
//uint8_t countFIFO = 42;

extern _sMPUData myMPU;

void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data = 0;

    // Verifies connection with MPU6050
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I, 1, &data, 1, MPU_TIMEOUT);

    if (data == WHO_AM_I_DEFAULT_VALUE){
        // Wake up MPU6050
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, POWER_MANAGEMENT_REG, 1, &data, 1, MPU_TIMEOUT);

        // Set data rate of 1 KHz (default)
        data = 0x07;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, MPU_TIMEOUT);

        // Set accelerometer range of +/- 2g (default)
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, MPU_TIMEOUT);

        // Set gyroscope range of +/- 250 degree/s (default)
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, MPU_TIMEOUT);

		// Set Digital Low Pass Filter
        data = 0x04;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, CONFIG, 1, &data, 1, MPU_TIMEOUT);

		/* DMP */
		/*
		// Disable DMP
		data = 0x00;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, USER_CTRL, 1, &data, 1, MPU_TIMEOUT);

		// Reset DMP
		data = 0x80;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, USER_CTRL, 1, &data, 1, MPU_TIMEOUT);

		// Enable DMP
		data = 0x08;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, USER_CTRL, 1, &data, 1, MPU_TIMEOUT);

		// Enable FIFO for gyroscope and accelerometer data
		data = 0x78;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, FIFO_EN, 1, &data, 1, MPU_TIMEOUT);

		// Reset FIFO
		data = 0x04;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, USER_CTRL, 1, &data, 1, MPU_TIMEOUT);

		// Enable FIFO
		data = 0x40;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, USER_CTRL, 1, &data, 1, MPU_TIMEOUT);
		*/
		/* DMP */

		myMPU.AccX = 0, myMPU.AccY = 0, myMPU.AccZ = 0;
		myMPU.GyrX = 0, myMPU.GyrY = 0, myMPU.GyrZ = 0;
		myMPU.OffsetAccX = 0, myMPU.OffsetAccY = 0, myMPU.OffsetAccZ = 0;
		myMPU.OffsetGyrX = 0, myMPU.OffsetGyrY = 0, myMPU.OffsetGyrZ = 0;
		myMPU.RawAccX = 0, myMPU.RawAccY = 0, myMPU.RawAccZ = 0;
		myMPU.RawGyrX = 0, myMPU.RawGyrY = 0, myMPU.RawGyrZ = 0;

		for (uint8_t i=0; i<14; i++){
			bufData[i] = 0;
			if (i<12)
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
	// Read data from FIFO
	// if (FIFO_OFLOW_INT == 1) -> DATA OVERFLOW -> NEW DATA!
	//HAL_I2C_Mem_Read_DMA(hi2c, MPU6050_ADDR, FIFO_R_W, 1, buf_FIFO, countFIFO);

}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	/*
	myMPU.Qw = (((uint32_t)buf_FIFO[0] << 24) | ((uint32_t)buf_FIFO[1] << 16) | ((uint32_t)buf_FIFO[2] << 8) | buf_FIFO[3]);
	myMPU.Qx = (((uint32_t)buf_FIFO[4] << 24) | ((uint32_t)buf_FIFO[5] << 16) | ((uint32_t)buf_FIFO[6] << 8) | buf_FIFO[7]);
	myMPU.Qy = (((uint32_t)buf_FIFO[8] << 24) | ((uint32_t)buf_FIFO[9] << 16) | ((uint32_t)buf_FIFO[10] << 8) | buf_FIFO[11]);
	myMPU.Qz = (((uint32_t)buf_FIFO[12] << 24) | ((uint32_t)buf_FIFO[13] << 16) | ((uint32_t)buf_FIFO[14] << 8) | buf_FIFO[15]);

	myMPU.GyrX = (((uint32_t)buf_FIFO[16] << 24) | ((uint32_t)buf_FIFO[17] << 16) | ((uint32_t)buf_FIFO[18] << 8) | buf_FIFO[19]);
	myMPU.GyrY = (((uint32_t)buf_FIFO[20] << 24) | ((uint32_t)buf_FIFO[21] << 16) | ((uint32_t)buf_FIFO[22] << 8) | buf_FIFO[23]);
	myMPU.GyrZ = (((uint32_t)buf_FIFO[24] << 24) | ((uint32_t)buf_FIFO[25] << 16) | ((uint32_t)buf_FIFO[26] << 8) | buf_FIFO[27]);

	myMPU.AccX = (((uint32_t)buf_FIFO[28] << 24) | ((uint32_t)buf_FIFO[29] << 16) | ((uint32_t)buf_FIFO[30] << 8) | buf_FIFO[31]);
	myMPU.AccY = (((uint32_t)buf_FIFO[32] << 24) | ((uint32_t)buf_FIFO[33] << 16) | ((uint32_t)buf_FIFO[34] << 8) | buf_FIFO[35]);
	myMPU.AccZ = (((uint32_t)buf_FIFO[36] << 24) | ((uint32_t)buf_FIFO[37] << 16) | ((uint32_t)buf_FIFO[38] << 8) | buf_FIFO[39]);
	*/

	// ACC: GET RAW INFORMATION
	myMPU.AccX = (((bufData[0] << 8) | bufData[1]));
	myMPU.AccY = (((bufData[2] << 8) | bufData[3]));
	myMPU.AccZ = (((bufData[4] << 8) | bufData[5]));
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
	myMPU.GyrX = (((bufData[8] << 8) | bufData[9]));
	myMPU.GyrY = (((bufData[10] << 8) | bufData[11]));
	myMPU.GyrZ = (((bufData[12] << 8) | bufData[13]));
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
}


