/**
 * TCRT5000 library, created by Ramiro Velasco
 *
 *
 */

#ifndef _TCRT5000_H
#define _TCRT5000_H

#include "stm32f1xx_hal.h"

#define NUMCHANNELSADC			8
#define LOOK_UP_TABLE_VALUES	20
#define SIZEBUFADC				24

typedef struct{
	uint8_t valueToMm[NUMCHANNELSADC];
	uint8_t index[NUMCHANNELSADC];
	uint16_t data[NUMCHANNELSADC];
	uint16_t lookUpTable[NUMCHANNELSADC][LOOK_UP_TABLE_VALUES];
	uint16_t value[NUMCHANNELSADC];
	uint16_t buf[NUMCHANNELSADC][SIZEBUFADC];
	uint32_t sum[NUMCHANNELSADC];
	uint8_t	threshold[NUMCHANNELSADC];
}_sADC;

void TCRT5000_Init(_sADC *dataADC);

void TCRT5000_DynamicFilter(_sADC *dataADC);

void TCRT5000_ADCtoMm(_sADC *dataADC);

#endif  // _TCRT5000_H
