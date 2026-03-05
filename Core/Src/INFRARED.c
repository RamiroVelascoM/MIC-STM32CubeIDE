/*
 * INFRARED.c
 *
 *  Created on: May 10, 2025
 *      Author: Gerónimo Spiazzi
 */

#include "INFRARED.h"

const uint8_t mm_reference[] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150};

void Infrared_Init(InfraredHandle_s *infraredData){
	//uint16_t aux[LOOKUP_SIZE] = {3943, 3929, 3839, 2820, 2047, 1547, 1136, 1026, 758, 683, 554, 542, 476, 403, 363, 325, 260, 230, 205};
	uint16_t aux[LOOKUP_SIZE] = {3840, 3835, 3825, 3650, 3300, 2560, 1960, 1530, 1200, 1000, 870, 750, 600, 520, 450, 400, 350, 325, 275, 250, 208, 169, 145, 135, 120, 106, 80, 65, 42, 30};

	for(uint8_t i=0; i<LOOKUP_SIZE; i++)
		infraredData->lookUp[i] = aux[i];

	infraredData->threshold[RIGHT_SENSOR] = 100;
	infraredData->threshold[RIGHT_DIAGONAL_SENSOR] = 110;
	infraredData->threshold[FRONT_RIGHT_SENSOR] = 55;
	infraredData->threshold[FRONT_UNDER_SENSOR] = 1;
	infraredData->threshold[FRONT_LEFT_SENSOR] = 55;
	infraredData->threshold[LEFT_DIAGONAL_SENSOR] = 110;
	infraredData->threshold[LEFT_SENSOR] = 100;
	infraredData->threshold[REAR_UNDER_SENSOR] = 1;
}

void Infrared_Filter(InfraredHandle_s *infraredData){
	for(uint8_t channel = 0; channel < ADC_CHANNELS; channel++){
		// Delete oldest value
		infraredData->acumulated[channel] -= infraredData->bufADC[channel][infraredData->index[channel]];

		// Load newest value
		infraredData->acumulated[channel] += infraredData->rawSamples[channel];

		// Update buffer with the newest sample
		infraredData->bufADC[channel][infraredData->index[channel]] = infraredData->rawSamples[channel];

		// Calculate average
		infraredData->filteredSamples[channel] = infraredData->acumulated[channel] / SIZE_BUF_ADC;

		// Increase index value
		infraredData->index[channel]++;

		// Overflow control
		infraredData->index[channel] &= (SIZE_BUF_ADC - 1);
	}
}

void Infrared_To8Bits(InfraredHandle_s *infraredData){
	uint8_t index = 0;
	for(uint8_t channel = 0; channel < ADC_CHANNELS; channel++){
		infraredData->auxSamples[index++] = (uint8_t)(infraredData->filteredSamples[channel] & 0xFF);
		infraredData->auxSamples[index++] = (uint8_t)((infraredData->filteredSamples[channel] >> 8) & 0xFF);
	}
}

void Infrared_Convert(InfraredHandle_s *infraredData){
    for (uint8_t channel = 0; channel < ADC_CHANNELS; channel++) {
    	if (channel != FRONT_UNDER_SENSOR && channel != REAR_UNDER_SENSOR){
			for(uint8_t j = 0; j < LOOKUP_SIZE; j++){
				if(infraredData->filteredSamples[channel] >= infraredData->lookUp[j]){
					infraredData->millimeterSamples[channel] = (j + 1) * 5;
					break;
				}
			}
    	} else {
    		if (infraredData->filteredSamples[channel] <= 3800)
				infraredData->millimeterSamples[channel] = 1;
			else
				infraredData->millimeterSamples[channel] = 0;
    	}
    }
}

