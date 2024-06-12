/*
 * motor.c
 *
 *  Created on: Jun 08, 2024
 *      Author: Ramiro Velasco
 */

#include "motor.h"

void Motors_Init(_sMOTOR *motorL, _sMOTOR *motorR)
{
	motorL->dir = DIR_FORWARD;
	motorL->pow = 0;
	motorL->pulses = 0;
	motorR->dir = DIR_FORWARD;
	motorR->pow = 0;
	motorR->pulses = 0;
}

void SetPowerMotor(TIM_HandleTypeDef *htim, _sMOTOR *motorL, _sMOTOR *motorR, int8_t powLEFT, int8_t powRIGHT)
{
	if (powLEFT < 0){
		motorL->dir = DIR_BACKWARDS;
		motorL->pulses = -powLEFT*100;
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, motorL->pulses);	// TIM4->CCR1
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);				// TIM4->CCR2
	}
	else{
		motorL->dir = DIR_FORWARD;
		motorL->pulses = powLEFT*100;
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);				// TIM4->CCR1
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, motorL->pulses);	// TIM4->CCR2
	}

	if (powRIGHT < 0){
		motorR->dir = DIR_BACKWARDS;
		motorR->pulses = -powRIGHT*100;
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, motorR->pulses);	// TIM4->CCR3
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 0);				// TIM4->CCR4
	}
	else{
		motorR->dir = DIR_FORWARD;
		motorR->pulses = powRIGHT*100;
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0);				// TIM4->CCR3
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, motorR->pulses);	// TIM4->CCR4
	}
}
