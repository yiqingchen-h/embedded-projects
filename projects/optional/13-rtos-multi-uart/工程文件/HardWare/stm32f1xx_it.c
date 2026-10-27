/*-------------------------------------------------*/
/*            超子说物联网STM32系列开发板          */
/*-------------------------------------------------*/
/*                                                 */
/*          实现各种中断服务函数的源文件           */
/*                                                 */
/*-------------------------------------------------*/

#include "stm32f1xx_hal.h"   
#include "stm32f1xx_it.h"  
#include "UART.h"
#include "FreeRTOS.h"
#include "task.h"
extern void xPortSysTickHandler(void);  // 原函数只在.c文件中进行了声明外部使用要 extern声明
/*串口1中断函数*/
void USART1_IRQHandler(void)
{
    //串口中断服务器函数
    HAL_UART_IRQHandler(&UART_HandleType);
    //判断中断标志位-判断是否为总线空闲标志位
    if (__HAL_UART_GET_FLAG(&UART_HandleType, UART_FLAG_IDLE))
    {
        //清除空闲中断标志位
        __HAL_UART_CLEAR_IDLEFLAG(&UART_HandleType);
        // 记录一共接收了多少数据 - 在终止中断接收时RxXferCount会被清空
        Uart1_Prt.RxCounter += (U1_Rx_MAX - UART_HandleType.RxXferCount);
        // 终止中断串口的接收
        HAL_UART_AbortReceive_IT(&UART_HandleType);
    }
}
void USART2_IRQHandler(void)
{
    // 串口中断服务器函数
    HAL_UART_IRQHandler(&UART_HandleType2);
    // 判断中断标志位-判断是否为总线空闲标志位
    if (__HAL_UART_GET_FLAG(&UART_HandleType2, UART_FLAG_IDLE)) {
        // 清除空闲中断标志位
        __HAL_UART_CLEAR_IDLEFLAG(&UART_HandleType2);
        // 记录一共接收了多少数据 - 在终止中断接收时RxXferCount会被清空
        Uart2_Prt.RxCounter += (U2_Rx_MAX - UART_HandleType2.RxXferCount);
        // 终止中断串口的接收
        HAL_UART_AbortReceive_IT(&UART_HandleType2);
    }
}
void USART3_IRQHandler(void)
{
    // 串口中断服务器函数
    HAL_UART_IRQHandler(&UART_HandleType3);
    // 判断中断标志位-判断是否为总线空闲标志位
    if (__HAL_UART_GET_FLAG(&UART_HandleType3, UART_FLAG_IDLE)) {
        // 清除空闲中断标志位
        __HAL_UART_CLEAR_IDLEFLAG(&UART_HandleType3);
        // 记录一共接收了多少数据 - 在终止中断接收时RxXferCount会被清空
        Uart3_Prt.RxCounter += (U3_Rx_MAX - UART_HandleType3.RxXferCount);
        // 终止中断串口的接收
        HAL_UART_AbortReceive_IT(&UART_HandleType3);
    }
}

    /*-------------------------------------------------*/
    /*函数名：不可屏蔽中断处理函数                     */
    /*参  数：无                                       */
    /*返回值：无                                       */
    /*-------------------------------------------------*/
void NMI_Handler(void)
{

}

/*-------------------------------------------------*/
/*函数名：硬件出错后进入的中断处理函数             */
/*参  数：无                                       */
/*返回值：无                                       */
/*-------------------------------------------------*/
void HardFault_Handler(void)
{

}
/*-------------------------------------------------*/
/*函数名：软中断，SWI 指令调用的处理函数           */
/*参  数：无                                       */
/*返回值：无                                       */
/*-------------------------------------------------*/
// void SVC_Handler(void)
// {
	
// }
/*-------------------------------------------------*/
/*函数名：可挂起的系统服务处理函数                 */
/*参  数：无                                       */
/*返回值：无                                       */
/*-------------------------------------------------*/
// void PendSV_Handler(void)
// {
	
// }
/*-------------------------------------------------*/
/*函数名：SysTic系统嘀嗒定时器处理函数             */
/*参  数：无                                       */
/*返回值：无                                       */
/*-------------------------------------------------*/

void SysTick_Handler(void)
{  
	//HAL_IncTick();	
    if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED){  // 判断调度器是否启用
        xPortSysTickHandler();                                  // FreeRTOS滴答时钟
    }
}
