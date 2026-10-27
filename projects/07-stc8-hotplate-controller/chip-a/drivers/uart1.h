#ifndef UART1_H
#define UART1_H

void UART1_Init(unsigned long baud);
void UART1_SendByte(unsigned char dat);
bit UART1_ByteReady(void);
unsigned char UART1_ReadByte(void);

#endif /* UART1_H */
