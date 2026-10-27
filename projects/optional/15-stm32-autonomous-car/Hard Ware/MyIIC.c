#include "stm32f10x.h"                  // Device header

void MyIIC_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_10,(BitAction)BitValue);
}
void MyIIC_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_11,(BitAction)BitValue);
}
unsigned char MyIIC_R_SDA(void)
{
	unsigned char BitValue;
	BitValue=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
	return BitValue ;
}
void MyIIC_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef LED_Init;
	LED_Init.GPIO_Mode =GPIO_Mode_Out_OD;
	LED_Init.GPIO_Pin =GPIO_Pin_10|GPIO_Pin_11;
	LED_Init.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&LED_Init);
	GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);
}

void MyIIC_Start(void)
{
	MyIIC_W_SDA(1);
	MyIIC_W_SCL(1);
	MyIIC_W_SDA(0);
	MyIIC_W_SCL(0);
}
void MyIIC_Stop (void)
{
	MyIIC_W_SDA(0);
	MyIIC_W_SCL(1);
	MyIIC_W_SDA(1);
}
void MyIIC_SendByte(unsigned char Byte )
{
	unsigned char i = 0;
	for( i = 0; i < 8 ; i ++)
	{
		MyIIC_W_SDA(Byte & (0x80 >> i));
		MyIIC_W_SCL(1);
		MyIIC_W_SCL(0);
	}
}
unsigned char MyIIC_ReceiveByte(void )
{
	unsigned char i = 0;
	unsigned char Byte = 0;
	for( i = 0; i < 8 ; i ++)
	{
		MyIIC_W_SDA(1);
		MyIIC_W_SCL(1);
		if( MyIIC_R_SDA() == 1 )
		{
			Byte |= (0x80 >> i);
		}
		MyIIC_W_SCL(0);
	}	
	return Byte ;
}
void MyIIC_SendAck(unsigned char Ack )
{
	MyIIC_W_SDA(Ack);
	MyIIC_W_SCL(1);
	MyIIC_W_SCL(0);
}
unsigned char MyIIC_ReceiveAck(void )
{
	unsigned char Ack = 0;
	MyIIC_W_SDA(1);
	MyIIC_W_SCL(1);
	Ack = MyIIC_R_SDA();
	MyIIC_W_SCL(0);
	return Ack ;
}
