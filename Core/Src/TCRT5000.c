/*
 * TCRT5000.c
 *
 *  Created on: Jun 15, 2024
 *      Author: Ramiro Velasco
 */

#include "TCRT5000.h"

//const uint16_t refValuesADC[20] = {280, 305, 330, 380, 415, 450, 510, 565, 640, 750, 900, 1080, 1250, 1600, 2040, 2400, 3100, 3720, 3800, 3835};
const uint16_t refValuesADC[20] = {250, 275, 325, 350, 400, 450, 520, 600, 750, 870, 1000, 1200, 1530, 1960, 2560, 3300, 3650, 3825, 3835, 3840};
//const uint16_t refValuesADC[20] = {3875, 3870, 3860, 3840, 3400, 2560, 1960, 1530, 1200, 1000, 870, 750, 600, 520, 450, 400, 350, 325, 275, 250};


void TCRT5000_DynamicFilter(_sADC *dataADC){
	for (uint8_t c=0; c<NUMCHANNELSADC; c++)
	{
		dataADC->sum[c] -= dataADC->buf[c][dataADC->index[c]];
		dataADC->sum[c] += dataADC->data[c];
		dataADC->buf[c][dataADC->index[c]] = dataADC->data[c];
		dataADC->value[c] = dataADC->sum[c]/SIZEBUFADC;
		dataADC->index[c]++;
		dataADC->index[c] &= (SIZEBUFADC-1);
	}
}

void TCRT5000_Init(_sADC *dataADC){
	for (uint8_t c=0; c<NUMCHANNELSADC; c++){
		for (uint8_t i=0; i<20; i++){
			dataADC->lookUpTable[c][i] = refValuesADC[i];
		}
		dataADC->valueToMm[c] = 0;
		dataADC->value[c] = 0;
	}
}

void TCRT5000_ADCtoMm(_sADC *dataADC){
	for (uint8_t c=0; c<NUMCHANNELSADC; c++){
		for (uint8_t i=0; i<20; i++){
			if (dataADC->value[c] >= dataADC->lookUpTable[c][i]){
				dataADC->valueToMm[c] = (i*5);
			}
		}
	}
}
