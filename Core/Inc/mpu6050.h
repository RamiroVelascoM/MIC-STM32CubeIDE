/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
#include "stm32f1xx_hal.h"

// MPU6050 structure
typedef struct
{
    int16_t Ax;
    int16_t Ay;
    int16_t Az;

    int16_t Gx;
    int16_t Gy;
    int16_t Gz;
}_sMPUData;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, _sMPUData *DataStruct);
