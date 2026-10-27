#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "Speed.h"
#define IN1_Zuo_Q GPIO_Pin_4
#define IN2_Zuo_Q GPIO_Pin_5
#define IN1_You_Q GPIO_Pin_6
#define IN2_You_Q GPIO_Pin_7
#define IN1_Zuo_H GPIO_Pin_0
#define IN2_Zuo_H GPIO_Pin_1
#define IN1_You_H GPIO_Pin_10
#define IN2_You_H GPIO_Pin_11

void Speed_Init(void)
{
	PWM_Init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef LED_Init;
	LED_Init.GPIO_Mode =GPIO_Mode_Out_PP;
	LED_Init.GPIO_Pin =IN1_Zuo_Q|IN2_Zuo_Q|IN1_You_Q|IN2_You_Q;//4,6,0,10,L298N,IN1¿Ú
	LED_Init.GPIO_Speed =GPIO_Speed_50MHz; 						//5£¬7£¬1£¬11£¬L298N£¬In2¿Ú
	GPIO_Init(GPIOA,&LED_Init);
	LED_Init.GPIO_Mode =GPIO_Mode_Out_PP;
	LED_Init.GPIO_Pin =IN1_Zuo_H|IN2_Zuo_H|IN1_You_H|IN2_You_H;
	LED_Init.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&LED_Init);
}

void Zuo_Qian(unsigned short CCR)
{
	TIM_SetCompare1(TIM2,CCR);
}
void You_Qian(unsigned short CCR)
{
	TIM_SetCompare2(TIM2,CCR);
}	

void Zuo_Hou(unsigned short CCR)
{
	TIM_SetCompare4(TIM2,CCR);
}	

void You_Hou(unsigned short CCR)
{
	TIM_SetCompare3(TIM2,CCR);
}	

void Qian_Jin(unsigned char Speed_X)
{
	GPIO_ResetBits (GPIOA,IN1_Zuo_Q |IN1_You_Q);
	GPIO_ResetBits (GPIOB,IN1_Zuo_H|IN1_You_H);
	GPIO_SetBits(GPIOA,IN2_Zuo_Q|IN2_You_Q);
	GPIO_SetBits(GPIOB,IN2_You_H |IN2_Zuo_H);
	{ 
		switch(Speed_X)
		{
			case 1:
			{
				Zuo_Qian(200);
				Zuo_Hou(200);
				You_Hou(200);
				You_Qian(200);
				break;
			}
			case 2:
			{
				Zuo_Qian(300);
				Zuo_Hou(300);
				You_Hou(300);
				You_Qian(300);
				break;
			}
			case 3:
			{
				Zuo_Qian(600);
				Zuo_Hou(600);
				You_Hou(600);
				You_Qian(600);
				break;
			}
			case 4:
			{
				Zuo_Qian(800);
				Zuo_Hou(800);
				You_Hou(800);
				You_Qian(800);
				break;
			}
			case 5:
			{
				Zuo_Qian(1000);
				Zuo_Hou(1000);
				You_Hou(1000);
				You_Qian(1000);
				break;
			}
		}
	}
}
void Hou_Tui(unsigned char Speed_X)
{
	GPIO_SetBits (GPIOA,IN1_Zuo_Q|IN1_You_Q);
	GPIO_SetBits (GPIOB,IN1_Zuo_H|IN1_You_H);
	GPIO_ResetBits(GPIOA,IN2_Zuo_Q|IN2_You_Q);
	GPIO_ResetBits(GPIOB,IN2_You_H |IN2_Zuo_H);
	{
		switch(Speed_X)
		{
			case 1:
			{
				Zuo_Qian(200);
				Zuo_Hou(200);
				You_Hou(200);
				You_Qian(200);
				break;
			}
			case 2:
			{
				Zuo_Qian(300);
				Zuo_Hou(300);
				You_Hou(300);
				You_Qian(300);
				break;
			}
			case 3:
			{
				Zuo_Qian(600);
				Zuo_Hou(600);
				You_Hou(600);
				You_Qian(600);
				break;
			}
			case 4:
			{
				Zuo_Qian(800);
				Zuo_Hou(800);
				You_Hou(800);
				You_Qian(800);
				break;
			}
			case 5:
			{
				Zuo_Qian(1000);
				Zuo_Hou(1000);
				You_Hou(1000);
				You_Qian(1000);
				break;
			}
		}
	}
}
void Zuo_Zhuan(unsigned char Speed_X)
{
	unsigned short Speed_Zuo = 0;
	switch(Speed_X)
	{
		case 1:
				Speed_Zuo = 100;
		break;
		case 2:
				Speed_Zuo = 150;
		break;        
		case 3:       
				Speed_Zuo = 200;
		break;        
		case 4:       
				Speed_Zuo = 250;
		break;        
		case 5:      
				Speed_Zuo= 350;
		break;
	}
	GPIO_WriteBit (GPIOA,IN1_Zuo_Q,Bit_RESET);
	GPIO_WriteBit (GPIOB,IN1_Zuo_H,Bit_RESET);
	GPIO_WriteBit (GPIOA,IN2_Zuo_Q,Bit_SET);
	GPIO_WriteBit (GPIOB,IN2_Zuo_H,Bit_SET);
	Zuo_Qian(Speed_Zuo);
	Zuo_Hou(Speed_Zuo - 100);
	You_Qian(0);
	You_Hou (0);
}

void You_Zhuan(unsigned char Speed_X)
{
	unsigned short Speed_You = 0;
	switch(Speed_X)
	{
		case 1:
				Speed_You = 100;
		break;
		case 2:
				Speed_You = 150;
		break;        
		case 3:       
				Speed_You = 200;
		break;        
		case 4:       
				Speed_You = 250;
		break;        
		case 5:      
				Speed_You= 350;
		break;
	}
	GPIO_WriteBit (GPIOA,IN1_You_Q,Bit_RESET);
	GPIO_WriteBit (GPIOB,IN1_You_H,Bit_RESET);
	GPIO_WriteBit (GPIOA,IN2_You_Q,Bit_SET);
	GPIO_WriteBit (GPIOB,IN2_You_H,Bit_SET);
	You_Qian(Speed_You);
	You_Hou(Speed_You - 100);
	Zuo_Qian(0);
	Zuo_Hou (0);
}
void Zhi_Dong(void)
{
	GPIO_SetBits(GPIOA,IN2_Zuo_Q|IN2_You_Q|IN1_You_Q|IN1_Zuo_Q);
	GPIO_SetBits(GPIOB,IN1_You_H|IN1_Zuo_H|IN2_You_H|IN2_Zuo_H);
	Zuo_Qian(1000);
	Zuo_Hou(1000);
	You_Qian(1000);
	You_Hou (1000);
}

