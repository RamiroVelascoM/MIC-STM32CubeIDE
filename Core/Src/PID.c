/*
 * PID.c
 *
 *  Created on: Sep 02, 2025
 *      Author: Ramiro Velasco
 */

#include "PID.h"

void PID_Init(_sPID *pid, int32_t Kp, int32_t Ki, int32_t Kd, int32_t min_max, int8_t base) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prevError = 0;
    pid->integral = 0;
    pid->output = 0;
    pid->outputMin = -min_max;
    pid->outputMax = min_max;
    pid->base = base;
}

int8_t PID_Compute(_sPID *pid, int32_t error){
    int32_t p_term = pid->Kp * error;
    int32_t i_term = pid->Ki * pid->integral;
    int32_t d_term = pid->Kd * ((error - pid->prevError) / 10); // Actualizacion del PID cada 10 ms

    // Suma PID con escala para no usar flotantes con escala base de 100
    pid->output = (p_term + i_term + d_term) / 100;

    if ((pid->output >= pid->outputMax) || (pid->output <= pid->outputMin)) {
        // No hacer nada con el integral.
    } else {
        pid->integral += error;
    }

    const int32_t INTEGRAL_LIMIT = 32000;
    if (pid->integral > INTEGRAL_LIMIT) {
        pid->integral = INTEGRAL_LIMIT;
    } else if (pid->integral < -INTEGRAL_LIMIT) {
        pid->integral = -INTEGRAL_LIMIT;
    }

    // Ajuste de salida / overflow
    if (pid->output > pid->outputMax)
        pid->output = pid->outputMax;
    else if (pid->output < pid->outputMin)
        pid->output = pid->outputMin;

    // Guardar error
    pid->prevError = error;

    return (int8_t)pid->output;
}

void PID_Reset(_sPID *pid) {
    pid->integral = 0;
    pid->prevError = 0;
}
