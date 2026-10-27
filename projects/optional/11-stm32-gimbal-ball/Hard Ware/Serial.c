#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

unsigned char Serial_RxData;
unsigned char Serial_RxFlag;
void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1 ,ENABLE);
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA ,ENABLE);
	GPIO_InitTypeDef LED_Init;
	LED_Init.GPIO_Mode =GPIO_Mode_AF_PP;
	LED_Init.GPIO_Pin =GPIO_Pin_9;
	LED_Init.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&LED_Init);
	LED_Init.GPIO_Mode =GPIO_Mode_IPU;
	LED_Init.GPIO_Pin =GPIO_Pin_10;
	LED_Init.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&LED_Init);
	//配置USART
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits =USART_StopBits_1;
	USART_InitStructure.USART_WordLength =USART_WordLength_8b;
	USART_Init(USART1 ,&USART_InitStructure);
	//开启中断(可在软件中读取接收信号此时可不使用中断)
	USART_ITConfig (USART1,USART_IT_RXNE,ENABLE);
	
	NVIC_PriorityGroupConfig (NVIC_PriorityGroup_2 );
	NVIC_InitTypeDef NVIC_InicStructure;
	NVIC_InicStructure.NVIC_IRQChannel =USART1_IRQn;
	NVIC_InicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InicStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InicStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InicStructure);
	
	//开启USART
	USART_Cmd(USART1 ,ENABLE );
}
void Serial_SendByte(uint8_t Byte)
{
	//发送数据函数
	USART_SendData(USART1,Byte);
	//获取标志位判断发送是否结束
	while(USART_GetFlagStatus(USART1 ,USART_FLAG_TXE) == RESET);
}
void Serial_SendArray(uint8_t *Array,uint16_t Length)
{
	uint16_t i;
	for(i = 0;i < Length ;i++)
	{
		Serial_SendByte(Array[i]);
	}
}
void Serial_SendString (char* String)
{
	unsigned short i = 0;
	for(i = 0 ; String[i] != '\0'; i++ )
	{
		Serial_SendByte(String[i]);
	}
}

unsigned int Serial_Pow(unsigned int x ,unsigned int y )
{
	unsigned int i = 1;
	while(y--)
	{
	i*=x;
	}
	return i;
}
void Serial_SendNumber(unsigned int Number,unsigned char Length)
{
	unsigned char i = 0;
	for( i = 0; i < Length ; i++)
	{
		Serial_SendByte(Number/Serial_Pow(10,Length - i - 1)%10 +'0');
	}
}

int fputc(int ch ,FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}
//封装sprintf函数封装
void Serial_Printf(char*format,...)
{
	char String[100];
	va_list arg;
	va_start(arg,format);
	vsprintf(String, format , arg);
	va_end (arg);
	Serial_SendString (String);
}

unsigned char Serial_GetRxFlag(void)
{
	if(Serial_RxFlag == 1)
	{
		Serial_RxFlag=0;
		return 1;
	}
	return 0;
}

unsigned char Serial_GetRxData(void)
{
	return Serial_RxData;
}

void USART1_IRQHandler(void)
{
	//判断是否接收到数据
	if(USART_GetFlagStatus(USART1 , USART_FLAG_RXNE)==SET)
		{
			//读取数据函数
			Serial_RxData = USART_ReceiveData(USART1);
			Serial_RxFlag = 1;
			//清除标志位
			USART_ClearITPendingBit (USART1 , USART_FLAG_RXNE);
		}
}


