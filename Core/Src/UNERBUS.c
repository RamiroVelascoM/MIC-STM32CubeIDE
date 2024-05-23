/*
 * UNERBUS.c
 *
 * Created: 04/01/24 08:49:00
 *  Author: German
 */ 

#include "UNERBUS.h"
#include <stdlib.h>

#ifdef AVRGCC_ATMEGA
#include <avr/pgmspace.h>
#endif

typedef union{
	uint8_t		u8[4];
	uint8_t		i8[4];
	uint16_t	u16[2];
	int16_t		i16[2];
	uint32_t	u32;
	int32_t		i32;
	float		f;	
}_uUNERBUSWork;

static uint8_t HEADER[7] = {'U', 'N', 'E', 'R', 0x00, ':', 0x00};

static _uUNERBUSWork w;

static void UNERBUS_DecodeHeader(_sUNERBUSHandle *aBus){
	uint8_t value;
	uint8_t index = aBus->rx.iWrite;

	while (aBus->rx.iRead != index)
	{
		value = aBus->rx.buf[aBus->rx.iRead];
		switch(aBus->rx.header){
		case 0:
			if(value == HEADER[aBus->rx.header]){
				aBus->rx.header = 1;
				aBus->rx.timeout = 5;
				aBus->rx.cks = value;
			}
			break;
		case 1:
		case 2:
		case 3:
		case 5:
			if(value == HEADER[aBus->rx.header]){
				aBus->rx.cks ^= value;
				aBus->rx.header++;
			}
			else{
				aBus->rx.header = 0;
				aBus->rx.iRead--;
			}
			break;
		case 4:
			aBus->rx.cks ^= value;
			aBus->rx.nBytes = value;
			aBus->rx.iData = aBus->rx.iRead+2;
			aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
			aBus->rx.newData = 0;
			aBus->rx.header = 5;
			break;
		case 6:
			aBus->rx.nBytes--;
			if(aBus->rx.nBytes)
				aBus->rx.cks ^= value;
			else{
				aBus->rx.header = 0;
				if(value == aBus->rx.cks){
					if(aBus->MyDataReady != NULL)
						aBus->MyDataReady(aBus, aBus->rx.iData);
					else
						aBus->rx.newData = 1;
				}
			}
			break;
		default:
			aBus->rx.header = 0;
		}

		aBus->rx.iRead &= aBus->rx.maxIndexRingBuf;
		aBus->rx.iRead++;
		aBus->rx.iRead &= aBus->rx.maxIndexRingBuf;
	}
}

void UNERBUS_Init(_sUNERBUSHandle *aBus){
	aBus->rx.header = 0;
	aBus->rx.iRead = 0;
	aBus->rx.iWrite = 0;
	aBus->rx.newData = 0;
	aBus->tx.iRead = 0;
	aBus->tx.iWrite = 0;
	aBus->iiTXw = 6;
}


void UNERBUS_Write(_sUNERBUSHandle *aBus, uint8_t *buf, uint8_t lenBuf){
	for (uint8_t i=0; i<lenBuf; i++)
	{
		aBus->tx.buf[aBus->iiTXw++] = buf[i];
		aBus->iiTXw &= aBus->tx.maxIndexRingBuf;
	}
}

void UNERBUS_WriteByte(_sUNERBUSHandle *aBus, uint8_t value){
	aBus->tx.buf[aBus->iiTXw++] = value;
	aBus->iiTXw &= aBus->tx.maxIndexRingBuf;
}

void UNERBUS_Send(_sUNERBUSHandle *aBus, uint8_t cmdID, uint8_t lenCMD){
	uint8_t i;

	i = aBus->tx.iWrite + 7;
	i &= aBus->tx.maxIndexRingBuf;
	
	if(aBus->iiTXw == i)
		return;
	
	HEADER[4] = lenCMD + 1;
	HEADER[6] = cmdID;


	aBus->tx.cks = 0;

	lenCMD += 6;
	
	for (i=0; i<lenCMD; i++)
	{
		if(i < 7)
			aBus->tx.buf[aBus->tx.iWrite] = HEADER[i];
		aBus->tx.cks ^= aBus->tx.buf[aBus->tx.iWrite];
		aBus->tx.iWrite++;
		aBus->tx.iWrite &= aBus->tx.maxIndexRingBuf;

	}

	aBus->tx.buf[aBus->tx.iWrite++] = aBus->tx.cks;	
	aBus->tx.iWrite &= aBus->tx.maxIndexRingBuf;
	aBus->iiTXw = (aBus->tx.iWrite + 7);
	aBus->iiTXw &= aBus->tx.maxIndexRingBuf;
}

void UNERBUS_SendToBuf(_sUNERBUSHandle *aBus, uint8_t cmdID, uint8_t lenCMD, uint8_t *bufForSend){
	uint8_t i;

	i = aBus->tx.iWrite + 7;
	i &= aBus->tx.maxIndexRingBuf;
	
	if(aBus->iiTXw == i)
		return;
	
	HEADER[4] = lenCMD + 1;
	HEADER[6] = cmdID;

	aBus->tx.cks = 0;

	lenCMD += 6;
	
	for (i=0; i<lenCMD; i++)
	{
		if(i < 7)
			bufForSend[i] = HEADER[i];
		else{
			bufForSend[i] = aBus->tx.buf[aBus->tx.iWrite++];
			aBus->tx.iWrite &= aBus->tx.maxIndexRingBuf;
		}
		aBus->tx.cks ^= bufForSend[i];
	}

	bufForSend[i] = aBus->tx.cks;
	aBus->tx.iRead = aBus->tx.iWrite;
	aBus->iiTXw = (aBus->tx.iWrite + 7);
	aBus->iiTXw &= aBus->tx.maxIndexRingBuf;
	
}

void UNERBUS_ReceiveByte(_sUNERBUSHandle *aBus, uint8_t value){
	aBus->rx.buf[aBus->rx.iWrite++] = value;
	aBus->rx.iWrite &= aBus->rx.maxIndexRingBuf;
}

void UNERBUS_ReceiveBuf(_sUNERBUSHandle *aBus, uint8_t *buf, uint8_t lenBuf){
	for (uint8_t i=0; i<lenBuf; i++)
	{
		aBus->rx.buf[aBus->rx.iWrite++] = buf[i];
		aBus->rx.iWrite &= aBus->rx.maxIndexRingBuf;
	}
}

void UNERBUS_WriteConstString(_sUNERBUSHandle *aBus, const char *buf, uint8_t lastString){
	uint8_t i=0, value;
	
	#ifdef AVRGCC_ATMEGA
	value = pgm_read_byte(buf);
	#else
	value = buf[i];
	#endif 
	while(value){
		aBus->tx.buf[aBus->tx.iWrite++] = value;
		aBus->tx.iWrite &= aBus->tx.maxIndexRingBuf;
		i++;
		#ifdef AVRGCC_ATMEGA
		value = pgm_read_byte(buf+i);
		#else
		value = buf[i];
		#endif
	}
	if(lastString){
		aBus->iiTXw = (aBus->tx.iWrite + 7);
		aBus->iiTXw &= aBus->tx.maxIndexRingBuf;
	}
}

void UNERBUS_GetBuf(_sUNERBUSHandle *aBus, uint8_t *buf, uint8_t lenBuf){
	for (uint8_t i=0; i<lenBuf; i++)
	{
		buf[i] =  aBus->rx.buf[aBus->rx.iData++];
		aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	}
}


uint8_t UNERBUS_GetUInt8(_sUNERBUSHandle *aBus){
	w.u8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	
	return w.u8[0];
}

int8_t UNERBUS_GetInt8(_sUNERBUSHandle *aBus){
	w.i8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;

	return w.i8[0];
}


uint32_t UNERBUS_GetUInt32(_sUNERBUSHandle *aBus){
	w.u8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[1] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[2] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[3] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;

	return w.u32;
}

int32_t UNERBUS_GetInt32(_sUNERBUSHandle *aBus){
	w.u8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[1] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[2] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[3] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;

	return w.i32;	
}

uint16_t UNERBUS_GetUInt16(_sUNERBUSHandle *aBus){
	w.u8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[1] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;

	return w.u16[0];	
}

int16_t UNERBUS_GetInt16(_sUNERBUSHandle *aBus){
	w.u8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[1] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;

	return w.i16[0];
}

float UNERBUS_GetFloat(_sUNERBUSHandle *aBus){
	w.u8[0] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[1] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[2] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
	w.u8[3] = aBus->rx.buf[aBus->rx.iData++];
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;

	return w.f;	
}

uint8_t UNERBUS_GetIndexRead(_sUNERBUSHandle *aBus){
	return aBus->rx.iData;
}

void UNERBUS_MoveIndexRead(_sUNERBUSHandle *aBus, int8_t newIndexRead){
	aBus->rx.iData += newIndexRead;
	aBus->rx.iData &= aBus->rx.maxIndexRingBuf;
}

void UNERBUS_ResetNewData(_sUNERBUSHandle *aBus){
	aBus->rx.newData = 0;
}

void UNERBUS_Task(_sUNERBUSHandle *aBus){
	if(aBus->rx.iRead != aBus->rx.iWrite)
		UNERBUS_DecodeHeader(aBus);
		
	if(aBus->WriteUSARTByte != NULL){
		if(aBus->tx.iRead != aBus->tx.iWrite){
			if(aBus->WriteUSARTByte(aBus->tx.buf[aBus->tx.iRead])){
				aBus->tx.iRead++;
				aBus->tx.iRead &= aBus->tx.maxIndexRingBuf;
			}
		}
	}	
}


void UNERBUS_Timeout(_sUNERBUSHandle *aBus){
	if(aBus->rx.timeout){
		aBus->rx.timeout--;
		if(!aBus->rx.timeout)
			aBus->rx.header = 0;
	}
}



