#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void Speed_Init(void)
{
	PWM_Init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_Speed;
	//设置上拉输入
	GPIO_Speed.GPIO_Mode =GPIO_Mode_Out_PP;
	GPIO_Speed.GPIO_Pin =GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Speed.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Speed);
	
}

void PWM_XianFu (int *PWM1,int *PWM2)
{
	if(*PWM1 > 6000)
	{
		*PWM1 = 6000;
	}
	else if(*PWM1 < -6000)
	{
		*PWM1 = -6000;
	}
	if(*PWM2 > 6000)
	{
		*PWM2 = 6000;
	}
	else if(*PWM2 < -6000)
	{
		*PWM2 = -6000;
	}
	
}




void Speed_FangXiang(int PWM1,int PWM2)
{
	
	if(PWM1 > 0)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
		GPIO_ResetBits(GPIOB,GPIO_Pin_13);
	}
	else
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_13);
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	}
	if(PWM2 < 0)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
	}
	else
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
	}
	if(PWM1 < 0)
	{
		PWM1 = -PWM1;
	}
	if(PWM2 < 0)
	{
		PWM2 = -PWM2;
	}
	PWM_OC2_CCR(PWM1);
	if(PWM2 != 0)
	PWM3_OC2_CCR(PWM2);
	else
	PWM3_OC2_CCR(0);
	
}


