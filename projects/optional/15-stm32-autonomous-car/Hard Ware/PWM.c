#include "stm32f10x.h"                  // Device header

void PWM_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_Speed;
	//设置上拉输入
	GPIO_Speed.GPIO_Mode =GPIO_Mode_Out_PP;
	GPIO_Speed.GPIO_Pin =GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Speed.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Speed);
	
	
	//配置GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef PWM_GPIO_Init;
	//配置复用推挽输出——使用定时器控制GPIO口时需要复用推挽或开漏将输出控制转移到片上外设
	//GPIO口配置
	PWM_GPIO_Init.GPIO_Mode =GPIO_Mode_AF_PP;
	PWM_GPIO_Init.GPIO_Pin =GPIO_Pin_11|GPIO_Pin_8;
	PWM_GPIO_Init.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&PWM_GPIO_Init);
	//GPIO_SetBits(GPIOA,GPIO_Pin_1);
	//开启APB1上TIM2{（2-4）普通计时器（1）高级计时器}的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	//选择时基单元时钟(使用内部时钟控制TIM2)-默认使用内部时钟可不写
	TIM_InternalClockConfig(TIM1);
	//配置时钟单元
	//使用结构体配置
	TIM_TimeBaseInitTypeDef Timer_TIM_Init;	
	//指定时钟的分频
	Timer_TIM_Init.TIM_ClockDivision = TIM_CKD_DIV1 ;
	//配置计数器模式
	Timer_TIM_Init.TIM_CounterMode = TIM_CounterMode_Up;
	//ARR自动重装器值(计数个数)（取值在0-65525）定时时间计算：定时频率=72MHz/（PSC+1）/（ARR+1），定时频率1Hz时定时时间为1s
	Timer_TIM_Init.TIM_Period = 7200-1;
	//PSC预分频器值（计数频率快慢）越大越慢
	Timer_TIM_Init.TIM_Prescaler = 1-1;
	//重复计时器-高级计时器才有（分频作用）普通计时器没有所以直接给0
	Timer_TIM_Init.TIM_RepetitionCounter = 0;
	//初始化函数（TIM_TimeBaseInit会生成一个更新事件来重新装载预分频器的值和重复计数器的值（立即发生））
	//所以在配置时会立刻进入中断一次
	TIM_TimeBaseInit(TIM1,&Timer_TIM_Init);
	//清除更新中断位以避免初始化完成进入中断
	//结构体
	TIM_OCInitTypeDef PWM_OC_Init;
	//快速给结构体赋值（防止不使用的结构体变量导致程序出错）
	TIM_OCStructInit(&PWM_OC_Init);
	//PWM_OC_Init.TIM_OCIdleState = 0;
	//配置输出比较模式
	PWM_OC_Init.TIM_OCMode = TIM_OCMode_PWM1 ;
	//PWM_OC_Init.TIM_OCNIdleState = 0;
	//PWM_OC_Init.TIM_OCNPolarity = 0;
	//配置输出比较极性
	PWM_OC_Init.TIM_OCPolarity = TIM_OCPolarity_High;
	//PWM_OC_Init.TIM_OutputNState = 0;
	//配置输出使能
	PWM_OC_Init.TIM_OutputState = TIM_OutputState_Enable;
	//配置CCR寄存器值
	//TIM_SetCompare1（1—4通道1到通道4）可单独修改通道CCR值
	PWM_OC_Init.TIM_Pulse =0;
	//初始化输出比较通道
	//PWM分辨率：Reso=1/（ARR+1）
	//PWM占空比：Duty=CRR/（ARR+1）
	//PWM频率：Freq=CK_PSC/（PSC+1）/（ARR+1）
	TIM_OC2Init(TIM1,&PWM_OC_Init);
	TIM_OC1Init(TIM1,&PWM_OC_Init);
	TIM_OC3Init(TIM1,&PWM_OC_Init);
	TIM_OC4Init(TIM1,&PWM_OC_Init);
	//启动定时器
	TIM_Cmd(TIM1,ENABLE);
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
}


int PWM_XianFu (int PWM1)
{
	if(PWM1 > 2300)
	{
		return 2300;
	}
	return PWM1;
}

void Speed(int PWM1,int PWM2)
{
	if(PWM1 < 0)
		{
			GPIO_SetBits(GPIOB,GPIO_Pin_13);
			GPIO_ResetBits(GPIOB,GPIO_Pin_12);
		}
		else
		{
			GPIO_SetBits(GPIOB,GPIO_Pin_12);
			GPIO_ResetBits(GPIOB,GPIO_Pin_13);
		}
		if(PWM2 > 0)
		{
			GPIO_SetBits(GPIOB,GPIO_Pin_15);
			GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		}
		else
		{
			GPIO_SetBits(GPIOB,GPIO_Pin_14);
			GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		}
}

//右轮
void PWM_OC2_CCR(int CCR)
{
//	int ZuoSuDu = PWM_XianFu(CCR);
	TIM_SetCompare1(TIM1,CCR);
}
void PWM1_OC2_CCR(int CCR)
{
	TIM_SetCompare2(TIM1,CCR);
}	

void PWM2_OC2_CCR(int CCR)
{
	TIM_SetCompare3(TIM1,CCR);
}	
//左轮
void PWM3_OC2_CCR(int CCR)
{
//	int YouSuDu = PWM_XianFu(CCR);
	TIM_SetCompare4(TIM1,CCR);
}	
