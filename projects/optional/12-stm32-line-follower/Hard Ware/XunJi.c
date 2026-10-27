#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "XunJi.h"

void Xun_Ji_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef LED_Init;
	LED_Init.GPIO_Mode =GPIO_Mode_IN_FLOATING;
	LED_Init.GPIO_Pin =GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	LED_Init.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&LED_Init);
	/*
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_AFIO ,ENABLE );
	//配置AFIO的数据选择器，选择EXTI的GPIO引脚
	GPIO_EXTILineConfig (GPIO_PortSourceGPIOB ,GPIO_PinSource12 );
	GPIO_EXTILineConfig (GPIO_PortSourceGPIOB ,GPIO_PinSource13 );
	GPIO_EXTILineConfig (GPIO_PortSourceGPIOB ,GPIO_PinSource14 );
	
	//配置EXTI
    EXTI_InitTypeDef EXTI_Initstructure;
    EXTI_Initstructure.EXTI_Line    = EXTI_Line12;          //PB12   GPIO_Pin_12
    EXTI_Initstructure.EXTI_LineCmd = ENABLE ;              //开启中断
    EXTI_Initstructure.EXTI_Mode    = EXTI_Mode_Interrupt ; //选择中断模式或事件模式
    EXTI_Initstructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling ;//选择中断触发方式
    EXTI_Init(&EXTI_Initstructure);    //初始化EXTI
	
	EXTI_Initstructure.EXTI_Line    = EXTI_Line13;          //PB13   GPIO_Pin_13
    EXTI_Initstructure.EXTI_LineCmd = ENABLE ;              //开启中断
    EXTI_Initstructure.EXTI_Mode    = EXTI_Mode_Interrupt ; //选择中断模式或事件模式
    EXTI_Initstructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling ;//选择中断触发方式
    EXTI_Init(&EXTI_Initstructure);    //初始化EXTI
	
	EXTI_Initstructure.EXTI_Line    = EXTI_Line14;          //PB14   GPIO_Pin_14
    EXTI_Initstructure.EXTI_LineCmd = ENABLE ;              //开启中断
    EXTI_Initstructure.EXTI_Mode    = EXTI_Mode_Interrupt ; //选择中断模式或事件模式
    EXTI_Initstructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling ;//选择中断触发方式
    EXTI_Init(&EXTI_Initstructure);    //初始化EXTI
	
	//配置NVIC
    NVIC_PriorityGroupConfig (NVIC_PriorityGroup_2 );  //设置中断分组方式为2个抢占2个响应
    NVIC_InitTypeDef NVIC_Initstructure;
    
    //通过参数跳转值定义，在stm32f10x.h 根据芯片容量类型选择所需要的中断通道（中断有20跟通道）
    NVIC_Initstructure.NVIC_IRQChannel = EXTI15_10_IRQn ;  //指定中断通道PB12
    NVIC_Initstructure.NVIC_IRQChannelCmd =  ENABLE ;      //指定中断通道使能
    NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 1; //指定抢占优先级
    NVIC_Initstructure.NVIC_IRQChannelSubPriority = 1;        //指定响应优先级
    NVIC_Init (&NVIC_Initstructure);   //初始化NVIC
	*/
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	GPIO_InitTypeDef LED_Init;
//	LED_Init.GPIO_Mode =GPIO_Mode_IN_FLOATING;
//	LED_Init.GPIO_Pin =GPIO_Pin_3|GPIO_Pin_3|GPIO_Pin_5;
//	LED_Init.GPIO_Speed =GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB,&LED_Init);
//	
//	RCC_APB2PeriphClockCmd (RCC_APB2Periph_AFIO ,ENABLE );
//	//配置AFIO的数据选择器，选择EXTI的GPIO引脚
//	GPIO_EXTILineConfig (GPIO_PortSourceGPIOB ,GPIO_PinSource3 );
//	GPIO_EXTILineConfig (GPIO_PortSourceGPIOB ,GPIO_PinSource4 );
//	GPIO_EXTILineConfig (GPIO_PortSourceGPIOB ,GPIO_PinSource5 );
//	
//	//配置EXTI
//    EXTI_InitTypeDef EXTI_Initstructure;
//    EXTI_Initstructure.EXTI_Line    = EXTI_Line3;          //PB12   GPIO_Pin_12
//    EXTI_Initstructure.EXTI_LineCmd = ENABLE ;              //开启中断
//    EXTI_Initstructure.EXTI_Mode    = EXTI_Mode_Interrupt ; //选择中断模式或事件模式
//    EXTI_Initstructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling ;//选择中断触发方式
//    EXTI_Init(&EXTI_Initstructure);    //初始化EXTI
//	
//	EXTI_Initstructure.EXTI_Line    = EXTI_Line4;          //PB13   GPIO_Pin_13
//    EXTI_Initstructure.EXTI_LineCmd = ENABLE ;              //开启中断
//    EXTI_Initstructure.EXTI_Mode    = EXTI_Mode_Interrupt ; //选择中断模式或事件模式
//    EXTI_Initstructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling ;//选择中断触发方式
//    EXTI_Init(&EXTI_Initstructure);    //初始化EXTI
//	
//	EXTI_Initstructure.EXTI_Line    = EXTI_Line5;          //PB14   GPIO_Pin_14
//    EXTI_Initstructure.EXTI_LineCmd = ENABLE ;              //开启中断
//    EXTI_Initstructure.EXTI_Mode    = EXTI_Mode_Interrupt ; //选择中断模式或事件模式
//    EXTI_Initstructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling ;//选择中断触发方式
//    EXTI_Init(&EXTI_Initstructure);    //初始化EXTI
//	
//	//配置NVIC
//    NVIC_PriorityGroupConfig (NVIC_PriorityGroup_2 );  //设置中断分组方式为2个抢占2个响应
//    NVIC_InitTypeDef NVIC_Initstructure;
//    
//    //通过参数跳转值定义，在stm32f10x.h 根据芯片容量类型选择所需要的中断通道（中断有20跟通道）
//    NVIC_Initstructure.NVIC_IRQChannel = EXTI3_IRQn ;  //指定中断通道PB12
//    NVIC_Initstructure.NVIC_IRQChannelCmd =  ENABLE ;      //指定中断通道使能
//    NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 0; //指定抢占优先级
//    NVIC_Initstructure.NVIC_IRQChannelSubPriority = 1;        //指定响应优先级
//    NVIC_Init (&NVIC_Initstructure);   //初始化NVIC
//	
//	NVIC_Initstructure.NVIC_IRQChannel = EXTI4_IRQn ;  //指定中断通道PB12
//    NVIC_Initstructure.NVIC_IRQChannelCmd =  ENABLE ;      //指定中断通道使能
//    NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 0; //指定抢占优先级
//    NVIC_Initstructure.NVIC_IRQChannelSubPriority =0;        //指定响应优先级
//    NVIC_Init (&NVIC_Initstructure);   //初始化NVIC
//	
//	NVIC_Initstructure.NVIC_IRQChannel = EXTI9_5_IRQn ;  //指定中断通道PB12
//    NVIC_Initstructure.NVIC_IRQChannelCmd =  ENABLE ;      //指定中断通道使能
//    NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 1; //指定抢占优先级
//    NVIC_Initstructure.NVIC_IRQChannelSubPriority = 1;        //指定响应优先级
//    NVIC_Init (&NVIC_Initstructure);   //初始化NVIC
}

unsigned char ZhuangTai_Zhong(void)
{
	return Zhong;
}

unsigned char ZhuangTai_Zuo(void)
{
	return Zuo;
}

unsigned char ZhuangTai_You(void)
{
	return You;
}

unsigned char Fang_Xiang(void)
{
	if(GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_12) == 1 &&GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_13) == 0 &&GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_14)== 0)
	{
//		Qian_Jin();
		return Zheng_Chang;
	}
	else if(GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_12) == 1 &&GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_13)==1 &&GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_14)==0)
	{
		return Pian_Zuo;
	}
	else if(GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_12)== 1&&GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_13) ==0 &&GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_14)==1)
	{
		return Pian_You;
	}
	else if(ZhuangTai_Zhong() == 0&&ZhuangTai_Zuo() ==1 &&ZhuangTai_You()==0)
	{
		return Pian_Zuo;
	}
	else if(ZhuangTai_Zhong() == 0&&ZhuangTai_Zuo() == 0 &&ZhuangTai_You()==1 )
	{
		return Pian_You;
	}
	return OFF;
}



//void EXTI3_IRQHandler (void)
//{
//	if(EXTI_GetITStatus (EXTI_Line3 ) == SET)
//	{
//		Delay_ms(5);
//		Zhong = GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_3);
//		//清除中断挂起标志位
//        EXTI_ClearITPendingBit(EXTI_Line3 );
//	}
//}

//void EXTI4_IRQHandler (void)
//{
//	if(EXTI_GetITStatus (EXTI_Line4 ) == SET)
//	{
//		Delay_ms(5);
//		Zhong = GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_4);
//		//清除中断挂起标志位
//        EXTI_ClearITPendingBit(EXTI_Line4 );
//	}
//}

//void EXTI9_5_IRQHandler (void)
//{
//	if(EXTI_GetITStatus (EXTI_Line5 ) == SET)
//	{
//		Delay_ms(5);
//		Zhong = GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_5);
//		//清除中断挂起标志位
//        EXTI_ClearITPendingBit(EXTI_Line5 );
//	}
//}

void EXTI15_10_IRQHandler(void)
{
	Zhong = GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_12);
	//判断EXTI14中断标志位是否被置 1
	if(EXTI_GetITStatus (EXTI_Line12 ) == SET)
	{
		Delay_ms(5);
		Zhong = GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_12);
		//清除中断挂起标志位
        EXTI_ClearITPendingBit(EXTI_Line12 );
	}
	 
	
	if(EXTI_GetITStatus (EXTI_Line13 ) == SET)
	{
		Delay_ms(5);
		Zuo = GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_13);
		EXTI_ClearITPendingBit(EXTI_Line13 );
	}
	
	if(EXTI_GetITStatus (EXTI_Line14) == SET)
	{
		Delay_ms(5);
		You = GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_14);
		EXTI_ClearITPendingBit(EXTI_Line15 );
	}
}
