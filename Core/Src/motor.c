/*
 * motor.c
 *
 *  Created on: Jun 08, 2024
 *      Author: Ramiro Velasco
 */

#include "motor.h"
#include <stdlib.h>

void Motors_Init(_sMOTOR *motorL, _sMOTOR *motorR)
{
	motorL->dir = DIR_FORWARD;
	motorL->pow = 0;
	motorL->pulses = 0;
	motorL->base = 3500;
	motorR->dir = DIR_FORWARD;
	motorR->pow = 0;
	motorR->pulses = 0;
	motorR->base = 3500;
}

void Set_Power_Motor(TIM_HandleTypeDef *htim, _sMOTOR *motorL, _sMOTOR *motorR, int8_t powLEFT, int8_t powRIGHT)
{
	if (powLEFT < 0){
		motorL->dir = DIR_BACKWARDS;
		motorL->pulses = -powLEFT*100;
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, motorL->pulses);	// TIM4->CCR4
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0);				// TIM4->CCR3
	}
	else{
		motorL->dir = DIR_FORWARD;
		motorL->pulses = powLEFT*100;
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 0);				// TIM4->CCR4
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, motorL->pulses);	// TIM4->CCR3
	}

	if (powRIGHT < 0){
		motorR->dir = DIR_BACKWARDS;
		motorR->pulses = -powRIGHT*100;
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, motorR->pulses);	// TIM4->CCR2
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);				// TIM4->CCR1
	}
	else{
		motorR->dir = DIR_FORWARD;
		motorR->pulses = powRIGHT*100;
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);				// TIM4->CCR2
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, motorR->pulses);	// TIM4->CCR1
	}
}

void Set_RampPower_Motor(TIM_HandleTypeDef *htim, _sMOTOR *motorL, _sMOTOR *motorR, int8_t powLEFT, int8_t powRIGHT)
{
    // Variables estáticas: mantienen su valor entre llamadas a la función
    static int8_t rampL = 0;
    static int8_t rampR = 0;

    // --- RAMPA MOTOR IZQUIERDO ---
    if (rampL < powLEFT) {
        rampL += RAMP_STEP;
        if (rampL > powLEFT) rampL = powLEFT;
    }
    else if (rampL > powLEFT) {
        rampL -= RAMP_STEP;
        if (rampL < powLEFT) rampL = powLEFT;
    }

    // --- RAMPA MOTOR DERECHO ---
    if (rampR < powRIGHT) {
        rampR += RAMP_STEP;
        if (rampR > powRIGHT) rampR = powRIGHT;
    }
    else if (rampR > powRIGHT) {
        rampR -= RAMP_STEP;
        if (rampR < powRIGHT) rampR = powRIGHT;
    }

    // --- ACTUALIZACIÓN DE LA ESTRUCTURA (Para que el main sepa la potencia real) ---
    motorL->pow = rampL;
    motorR->pow = rampR;

    // --- APLICACIÓN DE PWM (Usando las variables rampL y rampR) ---

    // Motor Izquierdo
    if (rampL < 0){
        motorL->dir = DIR_BACKWARDS;
        motorL->pulses = (-rampL) * 100;
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, motorL->pulses);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0);
    }
    else {
        motorL->dir = DIR_FORWARD;
        motorL->pulses = rampL * 100;
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, motorL->pulses);
    }

    // Motor Derecho
    if (rampR < 0){
        motorR->dir = DIR_BACKWARDS;
        motorR->pulses = (-rampR) * 100;
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, motorR->pulses);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
    }
    else {
        motorR->dir = DIR_FORWARD;
        motorR->pulses = rampR * 100;
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, motorR->pulses);
    }
}
