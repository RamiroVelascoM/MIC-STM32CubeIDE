/*
 * PID.h
 *
 *  Created on: Sep 02, 2025
 *      Author: Ramiro Velasco
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

typedef struct {
    uint16_t Kp;
    uint16_t Ki;
    uint16_t Kd;

    int32_t prevError;
    int32_t integral;
    int32_t output;

    int32_t outputMin;
    int32_t outputMax;
    uint8_t base;
} _sPID;

void PID_Init(_sPID *pid, uint16_t Kp, uint16_t Ki, uint16_t Kd, int32_t min_max, uint8_t base);
int8_t PID_Compute(_sPID *pid, int32_t error);
void PID_Reset(_sPID *pid);

#endif /* INC_PID_H_ */
