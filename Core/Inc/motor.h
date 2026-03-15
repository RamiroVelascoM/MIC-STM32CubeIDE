/**
 * Motor library, created by Ramiro Velasco
 *
 *
 */

#ifndef _motor_H
#define _motor_H

#include "stm32f1xx_hal.h"

#define LEFT			0
#define RIGHT			1
#define DIR_FORWARD		0
#define DIR_BACKWARDS	1
#define MAX_PULSES		9999

typedef struct{
	int8_t pow;
	uint8_t dir;
	int16_t pulses;
	uint16_t base;
}_sMOTOR;

void Motors_Init(_sMOTOR *motorL, _sMOTOR *motorR);

void Set_Power_Motor(TIM_HandleTypeDef *htim, _sMOTOR *motorL, _sMOTOR *motorR, int8_t powLEFT, int8_t powRIGHT);

#endif  // _motor_H
