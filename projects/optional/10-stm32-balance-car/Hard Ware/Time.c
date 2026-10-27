#include "stm32f10x.h"                  // Device header
#include "MPU6050.h"
#include "inv_mpu.h"
#include "Delay.h"
#include "OLED.h"
#include "Pid.h"
#include "Speed.h"
#include "Time.h"

float CeLiang;
short CeLiang_Gyrz;
int PWM1,PWM2;
int Zuo_SuDu ,Yuo_SuDu;//extern float CeLiang;
//extern int PWM1,PWM2;
//extern int Zuo_SuDu ,Yuo_SuDu;
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

void Tim_BIanMaQi_Init(void)
{
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_Speed;
	//设置上拉输入
	GPIO_Speed.GPIO_Mode =GPIO_Mode_IN_FLOATING;
	GPIO_Speed.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Speed.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Speed);
	
	GPIO_Speed.GPIO_Mode =GPIO_Mode_IN_FLOATING;
	GPIO_Speed.GPIO_Pin =GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Speed.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Speed);
	
	
	
	//开启APB1上TIM2{（2-4）普通计时器（1）高级计时器}的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	//选择时基单元时钟(使用内部时钟控制TIM2)-默认使用内部时钟可不写
	TIM_InternalClockConfig(TIM2);
	//配置时钟单元
	//使用结构体配置
	TIM_TimeBaseInitTypeDef Timer_TIM_Init;
	//指定时钟的分频
	Timer_TIM_Init.TIM_ClockDivision = TIM_CKD_DIV1 ;
	//配置计数器模式
	Timer_TIM_Init.TIM_CounterMode = TIM_CounterMode_Up;
	//ARR自动重装器值(计数个数)（取值在0-65525）定时时间计算：定时频率=72MHz/（PSC+1）/（ARR+1），定时频率1Hz时定时时间为1s
	Timer_TIM_Init.TIM_Period = 65536-1;
	//PSC预分频器值（计数频率快慢）越大越慢
	Timer_TIM_Init.TIM_Prescaler = 1-1;
	//重复计时器-高级计时器才有（分频作用）普通计时器没有所以直接给0
	Timer_TIM_Init.TIM_RepetitionCounter = 0;
	//初始化函数（TIM_TimeBaseInit会生成一个更新事件来重新装载预分频器的值和重复计数器的值（立即发生））
	//所以在配置时会立刻进入中断一次	
	TIM_TimeBaseInit(TIM2,&Timer_TIM_Init);
	TIM_TimeBaseInit(TIM4,&Timer_TIM_Init);
	
	
	//选择编码器模式
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	//编码器模式初始化结构体
	TIM_ICInitTypeDef TIM_ICInitStructure;
	//结构体所有参数默认填入
	TIM_ICStructInit(&TIM_ICInitStructure);
	//设置滤波长度
	TIM_ICInitStructure.TIM_ICFilter = 10;
	//初始化定时器 编码器模式
	TIM_ICInit(TIM2,&TIM_ICInitStructure);
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	//启动定时器
	TIM_Cmd(TIM2,ENABLE);
	TIM_Cmd(TIM4,ENABLE);
	


	//MPU6050中断
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_MPU6050;
	//设置上拉输入
	GPIO_MPU6050.GPIO_Mode =GPIO_Mode_IPU;
	GPIO_MPU6050.GPIO_Pin =GPIO_Pin_5;
	GPIO_MPU6050.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_MPU6050);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级2， 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	
	
}
/*
获取定时器当前编码器值
参数为TIMx，x为1-4
TIM_GetCounter();
将编码器计数器清0
参数1：TIMx，x为1—4，参数2给 0
TIM_SetCounter()
*/

int Jue_DuiZhi1(int x)
{
	return x < 1000 ? x : -(65535 - (int)x);
}
int Jue_DuiZhi2(int x)
{
	return x < 1000 ? -x : (65535 - (int)x);
}


	//TIM3定时器中断函数
//void TIM3_IRQHandler(void)
//{
////	OLED_ShowSignedNum(4,1,1,1);
//	//读取姿态信息
//	if(MPU6050_DMP_Get_Data(&pitch,&roll,&yaw) == 0)
//	{
//		if( roll < 30 && roll > -30)
//		{
//			CeLiang = roll;
//			Zuo_SuDu = Jue_DuiZhi(TIM_GetCounter(TIM4));
//			Yuo_SuDu = Jue_DuiZhi(TIM_GetCounter(TIM2));
//			TIM_SetCounter(TIM4,0);
//			TIM_SetCounter(TIM2,0);
//	//		MPU_Get_Gyroscope(&gx,&gy,&gz);
//	//		MPU_Get_Accelerometer(&ax,&ay,&az);
//			PWM1 = vertical_PID_value(CeLiang , -1) + velocity_PID_value((Zuo_SuDu+Yuo_SuDu)/2);	
//			PWM_XianFu(&PWM1 , &PWM1);
//			Speed_FangXiang(PWM1,PWM1);
//			
//			

//		}
//		else 
//		{
//			Speed_FangXiang(0,0);
//		}
//		
//		
//	}
//	//清除标志位
//	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
//}	



void EXTI9_5_IRQHandler(void)
{
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 0) //低电平触发
	{
		EXTI->PR=1<<5; //清楚中断标志位
//		MPU_Get_Gyroscope(&gx,&gy,&gz);
		MPU6050_DMP_Get_Data(&pitch, &roll, &yaw); //获取欧拉角
				OLED_ShowSignedNum(1,1,pitch,3);
		OLED_ShowSignedNum(2,1,roll,3);
		OLED_ShowSignedNum(3,10,yaw,3);
		if( roll < 30 && roll > -30)
		{
			CeLiang = roll;
			CeLiang_Gyrz = gz;
			Zuo_SuDu = Jue_DuiZhi1(TIM_GetCounter(TIM4));
			Yuo_SuDu = Jue_DuiZhi2(TIM_GetCounter(TIM2));
			TIM_SetCounter(TIM4,0);
			TIM_SetCounter(TIM2,0);
	//		MPU_Get_Gyroscope(&gx,&gy,&gz);
	//		MPU_Get_Accelerometer(&ax,&ay,&az);
			int Zhuan = ZhuanXiang_Pid_Value(CeLiang_Gyrz,0);
			PWM1 = vertical_PID_value(CeLiang , 0) + velocity_PID_value((Zuo_SuDu+Yuo_SuDu)/2);	
			PWM_XianFu(&PWM1 , &PWM1);
			Speed_FangXiang(PWM1 - Zhuan,PWM1 + Zhuan);
			
			

		}
		else 
		{
			Speed_FangXiang(0,0);
			Zuo_SuDu = Jue_DuiZhi1(TIM_GetCounter(TIM4));
			Yuo_SuDu = Jue_DuiZhi2(TIM_GetCounter(TIM2));
			TIM_SetCounter(TIM4,0);
			TIM_SetCounter(TIM2,0);
		}
		
		
		/*这里写相关应用*/
	}
}

