/*-------------------------------------------------*/
/*            超子说物联网STM32系列开发板          */
/*-------------------------------------------------*/
/*                                                 */
/*          实现各种中断服务函数的源文件           */
/*                                                 */
/*-------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
extern void xPortSysTickHandler(void);  // 原函数只在.c文件中进行了声明外部使用要 extern声明
/*串口1中断函数*/
void USART1_IRQHandler(void)
{
    // 串口中断服务器函数
    HAL_UART_IRQHandler(&UART_HandleType);
    // 判断中断标志位-判断是否为总线空闲标志位
    if (__HAL_UART_GET_FLAG(&UART_HandleType, UART_FLAG_IDLE)) {
        // 清除空闲中断标志位
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
extern UART_HandleTypeDef UART_HandleType3;
void USART3_IRQHandler(void)
{
  uint32_t tmp_flag = 0;

  // 检查是否是 IDLE 中断（接收是否结束）
  tmp_flag = __HAL_UART_GET_FLAG(&UART_HandleType3, UART_FLAG_IDLE);

  if ((tmp_flag != RESET))
  {
    // 清除空闲中断标志位
    __HAL_UART_CLEAR_IDLEFLAG(&UART_HandleType3);

    // 2. 停止 DMA 传输 (防止处理数据时又有新数据进来导致错乱，视情况而定)
    HAL_UART_DMAStop(&UART_HandleType3);

    // 3. 计算接收到的数据长度
    // 缓冲区总大小 - DMA剩余待传输数据量
    EspRx.RxLen = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);

    // 4. 置位接收完成标志
    EspRx.RxFlag = 1;
    if (EspRx.RxLen < RX_BUFFER_SIZE)
    {
      EspRx.RxBuffer[EspRx.RxLen] = '\0';
    }else if(EspRx.RxLen >= RX_BUFFER_SIZE){
			EspRx.RxBuffer[(EspRx.RxLen-1)] = '\0';
		}

    // 5. 如果使用的是 DMA_NORMAL 模式，下次接收需要重新开启 DMA
    // 如果数据处理很快，可以在主循环处理完后再开启，防止覆盖
    // 这里为了简单，我们假设处理很快，暂不关闭 DMA，而是稍后在主循环重置
  }

  // 调用 HAL 库默认处理 (处理其他可能的 UART 中断)
  HAL_UART_IRQHandler(&UART_HandleType3);
}

// DMA1 Stream0 中断入口 //WS2812B使用
void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tim4_ch1);
}

/**
 * @brief DMA 中断服务函数
 */
void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UART_HandleType3.hdmarx);
}

// IIC1_事件中断
void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&I2C_HandleType_IIC1); // 中断处理函数
}

/*-------------------------------------------------*/
/*函数名：不可屏蔽中断处理函数                     */
/*参  数：无                                       */
/*返回值：无                                       */
/*-------------------------------------------------*/
void  NMI_Handler(void)
{

}

/*-------------------------------------------------*/
/*函数名：硬件出错后进入的中断处理函数             */
/*参  数：无                                       */
/*返回值：无                                       */
/*-------------------------------------------------*/
void HardFault_Handler(void)
{
	while(1){
	}
}
/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}
/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
/*-------------------------------------------------*/
/*函数名：软中断，SWI 指令调用的处理函数           */
/*参  数：无                                       */
/*返回值：无                                       */
/*                由FreeRTOS代替                   */
/*-------------------------------------------------*/
//void SVC_Handler(void)
//{
//	
//}
/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}
/*-------------------------------------------------*/
/*函数名：可挂起的系统服务处理函数                 */
/*参  数：无                                       */
/*返回值：无                                       */
/*                由FreeRTOS代替                   */
/*-------------------------------------------------*/
//void PendSV_Handler(void)
//{
//	
//}
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
#include "RCC.h"
void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim7);
}
