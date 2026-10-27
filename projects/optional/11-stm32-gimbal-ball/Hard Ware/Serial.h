#ifndef __SERIAL_H
#define __SERIAL_H
#include <stdio.h>
void Serial_Init(void);
	
void Serial_SendByte(uint8_t Byte);

void Serial_SendArray(uint8_t *Array,uint16_t Length);

void Serial_SendString (char* String);

void Serial_SendNumber(unsigned int Number,unsigned char Length);

void Serial_Printf(char*format,...);

unsigned char Serial_GetRxFlag(void);

unsigned char Serial_GetRxData(void);
	
#endif
