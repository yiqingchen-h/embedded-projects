#include "stm32f10x.h"                  // Device header
#include "MPU6050.h"
#include "inv_mpu.h"
#include "Delay.h"
#include "OLED.h"
#include "Pid.h"
#include "LED.h"
#include "PWM.h"
void Time_Init(void)
{
	//开启APB1上TIM2{（2-4）普通计时器（1）高级计时器}的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	//选择时基单元时钟(使用内部时钟控制TIM2)-默认使用内部时钟可不写
	TIM_InternalClockConfig(TIM3);
	//配置时钟单元
	//使用结构体配置
	TIM_TimeBaseInitTypeDef Timer_TIM_Init;
	//指定时钟的分频
	Timer_TIM_Init.TIM_ClockDivision = TIM_CKD_DIV1 ;
	//配置计数器模式
	Timer_TIM_Init.TIM_CounterMode = TIM_CounterMode_Up;
	//ARR自动重装器值(计数个数)（取值在0-65525）定时时间计算：定时频率=72MHz/（PSC+1）/（ARR+1），定时频率1Hz时定时时间为1s
	Timer_TIM_Init.TIM_Period = 10000-1;
	//PSC预分频器值（计数频率快慢）越大越慢
	Timer_TIM_Init.TIM_Prescaler = 72-1;
	//重复计时器-高级计时器才有（分频作用）普通计时器没有所以直接给0
	Timer_TIM_Init.TIM_RepetitionCounter = 0;
	//初始化函数（TIM_TimeBaseInit会生成一个更新事件来重新装载预分频器的值和重复计数器的值（立即发生））
	//所以在配置时会立刻进入中断一次
	TIM_TimeBaseInit(TIM3,&Timer_TIM_Init);
	//清除更新中断位以避免初始化完成进入中断
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	//使能中断-开启更新中断到NVIC通路
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	//配置NVIC(选择分组)
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//结构体配置
	NVIC_InitTypeDef Timer_NVIC_Init;
	//开启定时器2在NVIC中的通道
	Timer_NVIC_Init.NVIC_IRQChannel = TIM3_IRQn;
	//使能NVIC的IRQ通道
	Timer_NVIC_Init.NVIC_IRQChannelCmd = ENABLE;
	//配置优先级
	Timer_NVIC_Init.NVIC_IRQChannelPreemptionPriority = 0;
	Timer_NVIC_Init.NVIC_IRQChannelSubPriority = 0;
	//NCIC初始化
	NVIC_Init(&Timer_NVIC_Init);
	//启动定时器
	TIM_Cmd(TIM3,ENABLE);
}

int PWM_You = 2000;
int PWM_Zuo = 2000;


unsigned char Zero_GeShu(int Xun)
{
	int a = 0;
	unsigned char Ge_Shu = 0;
	for(a = 0 ;a < 8 ; a ++)
	{
		if((Xun & (1 << a)) != 1<<a)
		{
			Ge_Shu++;
		}
	}
	return Ge_Shu;
}


int Xun_Ji_GeShu = 0;
void Xunji(int *PWM_You ,int *PWM_Zuo)
{
	
	Xun_Ji_GeShu = Du_Qu_H();

	if((Xun_Ji_GeShu | 0x7F) == 0x7F )
	{
		*PWM_You -= 600;
		*PWM_Zuo += 600;
	}
	else if((Xun_Ji_GeShu | 0xBF) == 0xBF)
	{
		*PWM_You -= 500;
		*PWM_Zuo += 500;
	}
	else if((Xun_Ji_GeShu | 0xDF) == 0xDF )
	{
		*PWM_You -= 400;
		*PWM_Zuo += 400;
	}
	
	//左转
	else if((Xun_Ji_GeShu | 0xFE) == 0xFE )
	{
		*PWM_Zuo -= 600;
		*PWM_You += 600;
	}
	else if((Xun_Ji_GeShu | 0xFD) == 0xFD)
	{
		*PWM_Zuo -= 500;
		*PWM_You += 500;
	}
	else if((Xun_Ji_GeShu | 0xFB) == 0xFB )
	{
		*PWM_Zuo -= 400;
		*PWM_You += 400;
	}
}



	//TIM3定时器中断函数
void TIM3_IRQHandler(void)
{
	Speed(PWM_You,PWM_Zuo);
	Xunji(&PWM_You , &PWM_Zuo);
	PWM_OC2_CCR(PWM_You);
	PWM3_OC2_CCR(PWM_Zuo);
	//读取姿态信息
	if(MPU6050_DMP_Get_Data(&pitch,&roll,&yaw) == 0 )
	{
		Xunji(&PWM_You , &PWM_Zuo);
//		MPU_Get_Gyroscope(&gx,&gy,&gz);
//		MPU_Get_Accelerometer(&ax,&ay,&az);
//		PWM = vertical_PID_value(roll,0)+velocity_PID_value(10);
//		OLED_ShowSignedNum(2,1,pitch,3);
//		OLED_ShowSignedNum(3,1,roll,3);
//		OLED_ShowSignedNum(4,1,yaw,3);
		
		
		
	}
	//清除标志位
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}	

