#ifndef __MYIIC_H
#define __MYIIC_H

void MyIIC_Init(void);

void MyIIC_Start(void);

void MyIIC_Stop (void);

void MyIIC_SendByte(unsigned char Byte );

unsigned char MyIIC_ReceiveByte(void );

void MyIIC_SendAck(unsigned char Ack );

unsigned char MyIIC_ReceiveAck(void );

#endif
