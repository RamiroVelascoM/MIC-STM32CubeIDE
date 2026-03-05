/*
 * INFRARED.h
 *
 *  Created on: May 10, 2025
 *      Author: Gustavo
 */

#ifndef INC_INFRARED_H_
#define INC_INFRARED_H_

#include <stdint.h>

#define ADC_CHANNELS			8
#define SIZE_BUF_ADC			64
#define FILTER_SIZE				16
#define LOOKUP_SIZE				30//20

#define RIGHT_SENSOR			0
#define RIGHT_DIAGONAL_SENSOR	1
#define FRONT_RIGHT_SENSOR		2
#define FRONT_UNDER_SENSOR		3
#define FRONT_LEFT_SENSOR		4
#define LEFT_DIAGONAL_SENSOR	5
#define LEFT_SENSOR				6
#define REAR_UNDER_SENSOR		7

typedef struct{
	//uint8_t iwBufADC, irBufADC;
	uint8_t index[SIZE_BUF_ADC];
	uint16_t bufADC[ADC_CHANNELS][SIZE_BUF_ADC];
	uint16_t rawSamples[ADC_CHANNELS];
	uint16_t filteredSamples[ADC_CHANNELS];
	uint8_t auxSamples[ADC_CHANNELS*2];
	uint32_t acumulated[ADC_CHANNELS];
	uint8_t millimeterSamples[ADC_CHANNELS];
	uint16_t lookUp[LOOKUP_SIZE];
	uint8_t	threshold[ADC_CHANNELS];
} InfraredHandle_s;


//const uint16_t adcReferences[REFERENCE_SAMPLES] = {3943, 3929, 3839, 2820, 2047, 1547, 1136, 1026, 758, 683, 554, 542, 476, 403, 363, 325, 260, 230, 205};

void Infrared_Init(InfraredHandle_s *infraredData);

void Infrared_Filter(InfraredHandle_s *infraredData);

void Infrared_Convert(InfraredHandle_s *infraredData);

void Infrared_To8Bits(InfraredHandle_s *infraredData);

#endif /* INC_INFRARED_H_ */
