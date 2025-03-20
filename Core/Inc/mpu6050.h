/*
 * MPU6050.h
 *
 * Author: Ramiro Velasco
 *
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f1xx_hal.h"

// MPU6050 address
#define MPU6050_ADDR 			0xD0 // 0x68 << 1

// MPU6050 registers
#define WHO_AM_I_DEFAULT_VALUE 	0x68
#define WHO_AM_I 				0x75
#define POWER_MANAGEMENT_REG 	0x6B
#define CONFIG					0x1A
#define GYRO_CONFIG_REG 		0x1B
#define SMPLRT_DIV_REG 			0x19
#define GYRO_XOUT_REG 			0x43
#define ACCEL_CONFIG_REG	 	0x1C
#define ACCEL_XOUT_REG 			0x3B

#define FIFO_EN					0x23
#define USER_CTRL				0x6A
#define FIFO_COUNTH 			0x72
#define FIFO_R_W				0x74

#define MPU_TIMEOUT				1000//HAL_MAX_DELAY

typedef struct {
	// STORING DATA BUFFER
	uint8_t buffer[12];

	// POSITION, VELOCITY AND ACCELERATION DATA
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
	int16_t OffsetAccX;
	int16_t OffsetAccY;
	int16_t OffsetAccZ;
	int32_t RawAccX;
	int32_t RawAccY;
	int32_t RawAccZ;

	// GYROSCOPE DATA
	int16_t GyrX;
	int16_t GyrY;
	int16_t GyrZ;
	int16_t OffsetGyrX;
	int16_t OffsetGyrY;
	int16_t OffsetGyrZ;
	int32_t RawGyrX;
	int32_t RawGyrY;
	int32_t RawGyrZ;

	int16_t Roll;
	int16_t Pitch;
	int16_t Yaw;

	int32_t Qw;
	int32_t Qx;
	int32_t Qy;
	int32_t Qz;
}_sMPUData;


/**
 * @brief MPU6050_Init
 *
 * Inicializacion del modulo MPU6050
 *
 * @param hi2c: Handler del protocolo de comunicacion I2C
 * @return void
 */
void MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief MPU6050_Calibrate
 *
 * Calibracion inicial del modulo MPU6050
 *
 * @param hi2c: Handler del protocolo de comunicacion I2C
 * @return void
 */
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c);

/**
 * @brief MPU6050_ReadAll
 *
 * Lectura del acelerometro y giroscopio del MPU6050
 *
 * @param hi2c: Handler del protocolo de comunicacion I2C
 * @return void
 */
void MPU6050_ReadAll(I2C_HandleTypeDef *hi2c);


#endif /* INC_MPU6050_H_ */
