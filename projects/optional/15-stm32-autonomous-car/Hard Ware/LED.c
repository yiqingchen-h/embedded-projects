#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
void Hui_Du_Init(void)
{
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);//开启AFIO时钟解除复用
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	
	GPIO_InitTypeDef LED_Init;
	
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//解除默认复用功能
	
//	LED_Init.GPIO_Mode =GPIO_Mode_Out_PP;
//	LED_Init.GPIO_Pin =GPIO_Pin_4;
//	LED_Init.GPIO_Speed =GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB,&LED_Init);
//	
//	LED_Init.GPIO_Mode = GPIO_Mode_IPU;
//	LED_Init.GPIO_Pin =GPIO_Pin_3;
//	GPIO_Init(GPIOB,&LED_Init);
	
	LED_Init.GPIO_Mode =GPIO_Mode_Out_PP;
	LED_Init.GPIO_Pin =GPIO_Pin_4;
	GPIO_Init(GPIOA,&LED_Init);
	
	LED_Init.GPIO_Mode = GPIO_Mode_IPU;
	LED_Init.GPIO_Pin =GPIO_Pin_5;
	GPIO_Init(GPIOA,&LED_Init);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}
//int Du_Qu(void)
//{
//	unsigned char i = 0;
//	int Shui_Ju = 0;
//	for(i = 0; i < 8 ; i++)
//	{
//		GPIO_ResetBits(GPIOB,GPIO_Pin_4);
//		Delay_us(5);
//		Shui_Ju |= (GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_3) << i);
//		GPIO_SetBits(GPIOB,GPIO_Pin_4);
//		Delay_us(5);
//		
//	}
//	return Shui_Ju;
//}

int Du_Qu_H(void)
{
	unsigned char i_H = 0;
	int Shui_Ju_H = 0;
	for(i_H = 0; i_H < 8 ; i_H++)
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		delay_us(5);
		Shui_Ju_H |= (GPIO_ReadInputDataBit(GPIOA , GPIO_Pin_5) << i_H);
		GPIO_SetBits(GPIOA,GPIO_Pin_4);
		delay_us(5);
	}
	return Shui_Ju_H;
}

