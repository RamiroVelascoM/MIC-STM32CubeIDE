/**
 * TCRT5000 library, created by Ramiro Velasco
 *
 *
 */

#ifndef _TCRT5000_H
#define _TCRT5000_H

#include "stm32f1xx_hal.h"

#define NUMCHANNELSADC			8
#define SIZEBUFADC				64

typedef struct{
	uint8_t valueToMm[8];
	uint8_t index[8];
	uint16_t data[8];
	uint16_t lookUpTable[8][20];
	uint16_t value[8];
	uint16_t buf[8][64];
	uint32_t sum[8];
}_sADC;

void TCRT5000_Init(_sADC *dataADC);

void TCRT5000_DynamicFilter(_sADC *dataADC);

void TCRT5000_ADCtoMm(_sADC *dataADC);

#endif  // _TCRT5000_H
