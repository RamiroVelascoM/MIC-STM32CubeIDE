/*
 * INFRARED.c
 *
 *  Created on: May 10, 2025
 *      Author: Gerónimo Spiazzi
 */

#include "INFRARED.h"

const uint8_t mm_reference[] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150};

void Infrared_Init(InfraredHandle_s *infraredData){
	uint16_t table[LOOKUP_SIZE] = {3840, 3835, 3825, 3650, 3300, 2560, 1960, 1530, 1200, 1000, 870, 750, 600, 520, 450, 400, 350, 325, 275, 250, 208, 169, 145, 135, 120, 106, 80, 65, 42, 30};
	//uint16_t table[LOOKUP_SIZE] = {4023, 4003, 3938, 3485, 2600, 2115, 1770, 1520, 1350, 1200, 1110, 995, 920, 860, 800, 740, 695, 650, 610, 575, 545, 515, 495, 470, 445, 425, 400, 385, 370, 353};

	for(uint8_t i=0; i<LOOKUP_SIZE; i++)
		infraredData->lookUp[i] = table[i];

	infraredData->threshold[RIGHT_SIDE] = 100;
	infraredData->threshold[FRONT_RIGHT] = 125;
	infraredData->threshold[FRONT_1] = 100;
	infraredData->threshold[GROUND_FRONT] = 1;
	infraredData->threshold[FRONT_2] = 100;
	infraredData->threshold[FRONT_LEFT] = 125;
	infraredData->threshold[LEFT_SIDE] = 100;
	infraredData->threshold[GROUND_BACK] = 1;
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
    	if (channel != GROUND_FRONT && channel != GROUND_BACK){
    		/*
			for(uint8_t j = 0; j < LOOKUP_SIZE; j++){
				if(infraredData->filteredSamples[channel] >= infraredData->lookUp[j]){
					infraredData->millimeterSamples[channel] = (j + 1) * 5;
					break;
				}
			}
			*/
    		for(uint8_t j = 1; j < LOOKUP_SIZE; j++){
				if(infraredData->filteredSamples[channel] >= infraredData->lookUp[j]){
					uint16_t adc0 = infraredData->lookUp[j-1];
					uint16_t adc1 = infraredData->lookUp[j];

					uint8_t dist0 = (j * 5);
					uint8_t dist1 = (j + 1) * 5;

					uint32_t deltaDist = dist1 - dist0;
					uint32_t deltaAdcTotal = adc0 - adc1;
					uint32_t deltaAdcParcial = adc0 - infraredData->filteredSamples[channel];

					infraredData->millimeterSamples[channel] = dist0 + ((deltaDist * deltaAdcParcial) / deltaAdcTotal);
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

