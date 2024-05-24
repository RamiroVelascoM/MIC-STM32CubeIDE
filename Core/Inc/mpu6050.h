/*
 * MPU6050.h
 *
 *  Created on: May 21, 2024
 *      Author: Gerónimo Spiazzi
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f1xx_hal.h"

// MPU6050 address
#define MPU6050_ADDR 0xD0 // 0x68 << 1

// MPU6050 registers
#define WHO_AM_I 0x75
#define POWER_MANAGEMENT_REG 0x6B
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_REG 0x43
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_REG 0x3B
#define MPU_TIMEOUT	1000//HAL_MAX_DELAY

// Default Values
#define WHO_AM_I_DEFAULT_VALUE 0x68

typedef struct {
	uint8_t accelX[2];
	uint8_t accelY[2];
	uint8_t accelZ[2];
	uint8_t gyroX[2];
	uint8_t gyroY[2];
	uint8_t gyroZ[2];
	uint8_t buffer[12];
}_sMPUData;

// Prototypes
void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_Data_DMA(I2C_HandleTypeDef *hi2c);

#endif /* INC_MPU6050_H_ */
