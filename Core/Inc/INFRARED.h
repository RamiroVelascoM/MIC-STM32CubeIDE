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
#define SIZE_BUF_ADC			32
#define LOOKUP_SIZE				30//20

#define LEFT_SIDE				6
#define RIGHT_SIDE				0
#define FRONT_LEFT				5
#define FRONT_RIGHT				1
#define FRONT_1					2
#define FRONT_2					4
#define GROUND_BACK				7
#define GROUND_FRONT			3

typedef struct{
	uint8_t index[SIZE_BUF_ADC];
	uint16_t bufADC[ADC_CHANNELS][SIZE_BUF_ADC];
	uint16_t rawSamples[ADC_CHANNELS];
	uint16_t filteredSamples[ADC_CHANNELS];
	uint32_t acumulated[ADC_CHANNELS];
	uint8_t millimeterSamples[ADC_CHANNELS];
	uint16_t lookUp[LOOKUP_SIZE];
	uint8_t	threshold[ADC_CHANNELS];
} InfraredHandle_s;

void Infrared_Init(InfraredHandle_s *infraredData);

void Infrared_Filter(InfraredHandle_s *infraredData);

void Infrared_Convert(InfraredHandle_s *infraredData);

#endif /* INC_INFRARED_H_ */
