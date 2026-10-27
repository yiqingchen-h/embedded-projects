#include "stm32f10x.h"                  // Device header
#include "Delay.h"
void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_Key;
	//设置上拉输入
	GPIO_Key.GPIO_Mode =GPIO_Mode_IPU;
	GPIO_Key.GPIO_Pin =GPIO_Pin_1|GPIO_Pin_7;
	GPIO_Key.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Key);
}
unsigned char Key_GetNun(void)
{
	uint8_t KeyNum = 0;
	if(GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_1)==0)
	{
		Delay_ms (20);
		while(GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_1)==0);
		Delay_ms (20);
		KeyNum = 1;
	}
	if(GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_7)==0)
	{
		Delay_ms (20);
		while(GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_7)==0);
		Delay_ms (20);
		KeyNum = 2;
	}
	return KeyNum;
}

