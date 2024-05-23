/*
 * UNERBUS.h
 *
 * Created: 04/01/24 08:48:38
 *  Author: German
 */ 


#ifndef UNERBUS_H_
#define UNERBUS_H_

#include <stdint.h>

//#define AVRGCC_ATMEGA

/*!< UNERBUS */
typedef struct UNERBUSHandle
{
	struct{
		uint8_t		*buf;
		uint8_t		maxIndexRingBuf;
		uint8_t		iRead;
		uint8_t		iWrite;
		uint8_t		iData;
		uint8_t		newData;
		uint8_t		nBytes;
		uint8_t		header;
		uint8_t		timeout;
		uint8_t		cks;
	} rx;
	struct{
		uint8_t		*buf;
		uint8_t		maxIndexRingBuf;
		uint8_t		iRead;
		uint8_t		iWrite;
		uint8_t		cks;
	} tx;
	
	void (*MyDataReady)(struct UNERBUSHandle *aBus, uint8_t iStartData);
	uint8_t (*WriteUSARTByte)(uint8_t value);
	
	uint8_t			iiTXw;
} _sUNERBUSHandle;


void UNERBUS_Init(_sUNERBUSHandle *aBus);

void UNERBUS_Write(_sUNERBUSHandle *aBus, uint8_t *buf, uint8_t lenBuf);

void UNERBUS_WriteByte(_sUNERBUSHandle *aBus, uint8_t value);

void UNERBUS_Send(_sUNERBUSHandle *aBus, uint8_t cmdID, uint8_t lenCMD);

void UNERBUS_SendToBuf(_sUNERBUSHandle *aBus, uint8_t cmdID, uint8_t lenCMD, uint8_t *bufForSend);

void UNERBUS_ReceiveByte(_sUNERBUSHandle *aBus, uint8_t value);

void UNERBUS_ReceiveBuf(_sUNERBUSHandle *aBus, uint8_t *buf, uint8_t lenBuf);

void UNERBUS_WriteConstString(_sUNERBUSHandle *aBus, const char *buf, uint8_t lastString);

void UNERBUS_GetBuf(_sUNERBUSHandle *aBus, uint8_t *buf, uint8_t lenBuf);

uint8_t UNERBUS_GetUInt8(_sUNERBUSHandle *aBus);

int8_t UNERBUS_GetInt8(_sUNERBUSHandle *aBus);

uint32_t UNERBUS_GetUInt32(_sUNERBUSHandle *aBus);

int32_t UNERBUS_GetInt32(_sUNERBUSHandle *aBus);

uint16_t UNERBUS_GetUInt16(_sUNERBUSHandle *aBus);

int16_t UNERBUS_GetInt16(_sUNERBUSHandle *aBus);

float UNERBUS_GetFloat(_sUNERBUSHandle *aBus);

uint8_t UNERBUS_GetIndexRead(_sUNERBUSHandle *aBus);

void UNERBUS_MoveIndexRead(_sUNERBUSHandle *aBus, int8_t newIndexRead);

void UNERBUS_ResetNewData(_sUNERBUSHandle *aBus);

void UNERBUS_Task(_sUNERBUSHandle *aBus);

void UNERBUS_Timeout(_sUNERBUSHandle *aBus);

void UNERBUS_AttachOnDataReady(void (*aOnDataReady)(_sUNERBUSHandle *aBus, uint8_t iStartData));



#endif /* UNERBUS_H_ */
