#include "stm32f10x.h"                  // Device header
#include "Delay.h"  
#include "OLED.h"
#include "Speed.h"
#include "XunJi.h"
int main(void)
{
	
	OLED_Init();
	Speed_Init();
	Xun_Ji_Init();
	OLED_ShowString(1,1,"Zhong:");
	OLED_ShowString(2,1,"Zuo:");
	OLED_ShowString(3,1,"You:");
	while(1)
	{
		unsigned char Zhuang = Fang_Xiang();
		EXTI_GenerateSWInterrupt(EXTI_Line12);
//		EXTI_GenerateSWInterrupt(EXTI_Line14);
//		EXTI_GenerateSWInterrupt(EXTI_Line13);
		if(Zhuang == Zheng_Chang)
		{
			Qian_Jin(Speed_Two);
		}
		 if(Zhuang == Pian_Zuo)
		{
			Zuo_Zhuan(Speed_Five);
		}
		else if(Zhuang == Pian_You)
		{
			You_Zhuan(Speed_Five);
		}
		else if(Zhuang == OFF)
		{
			Delay_ms(10);
			if(Fang_Xiang() == OFF)
			{
				Zhi_Dong();
			}
		}
		OLED_ShowNum(1,8,GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_12),1);
		OLED_ShowNum(2,6,GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_13),1);
		OLED_ShowNum(3,6,GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_14),1);
		OLED_ShowNum(4,2,Fang_Xiang(),1);
	}
}
