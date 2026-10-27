#include "main.h"
#include "UART.h"
#include "string.h"
//printf重定向使用头文件
#include "stdarg.h"
#include "stdio.h"
// unsigned char Tx_Buff[64];
// unsigned char Rx_Buff[64];
// unsigned char Rx_State = 0;

unsigned char U1_Rx_Buff[U1_Rx_Size];
unsigned char U1_Tx_Buff[U1_Tx_Size];

unsigned char U2_Rx_Buff[U2_Rx_Size];
unsigned char U2_Tx_Buff[U2_Tx_Size];

//初始化指针
UCB Uart1_Prt;
UCB Uart2_Prt;
void UART1_Prt_Init(void){
    Uart1_Prt.RxInptr = &Uart1_Prt.RxLocation[0];
    Uart1_Prt.RxOutptr = &Uart1_Prt.RxLocation[0];
    Uart1_Prt.RxEndptr = &Uart1_Prt.RxLocation[9];
    Uart1_Prt.RxCounter = 0;
    //储存缓冲区位置
    Uart1_Prt.RxInptr->start = U1_Rx_Buff;

    Uart1_Prt.TxInptr        = &Uart1_Prt.TxLocation[0];
    Uart1_Prt.TxOutptr       = &Uart1_Prt.TxLocation[0];
    Uart1_Prt.TxEndptr       = &Uart1_Prt.TxLocation[9];
    Uart1_Prt.TxCounter      = 0;
    Uart1_Prt.TxInptr->start = U1_Tx_Buff;
}
void UART2_Prt_Init(void)
{
    Uart2_Prt.RxInptr   = &Uart2_Prt.RxLocation[0];
    Uart2_Prt.RxOutptr  = &Uart2_Prt.RxLocation[0];
    Uart2_Prt.RxEndptr  = &Uart2_Prt.RxLocation[9];
    Uart2_Prt.RxCounter = 0;
    // 储存缓冲区位置
    Uart2_Prt.RxInptr->start = U2_Rx_Buff;

    Uart2_Prt.TxInptr        = &Uart2_Prt.TxLocation[0];
    Uart2_Prt.TxOutptr       = &Uart2_Prt.TxLocation[0];
    Uart2_Prt.TxEndptr       = &Uart2_Prt.TxLocation[9];
    Uart2_Prt.TxCounter      = 0;
    Uart2_Prt.TxInptr->start = U2_Tx_Buff;
}

    // 发送数据 -U1
void UART1_Tx_Data(unsigned char *data, unsigned int len)
{
    if ((U1_Tx_Size - Uart1_Prt.TxCounter) >= len)
    {
        Uart1_Prt.TxInptr->start = &U1_Tx_Buff[Uart1_Prt.TxCounter];
    }
    else
    {
        Uart1_Prt.TxCounter      = 0;
        Uart1_Prt.TxInptr->start = U1_Tx_Buff;
    }
    memcpy(Uart1_Prt.TxInptr->start, data, len);
    Uart1_Prt.TxCounter += len;
    Uart1_Prt.TxInptr->end = &U1_Tx_Buff[Uart1_Prt.TxCounter - 1];
    Uart1_Prt.TxInptr++;
    if (Uart1_Prt.TxInptr == Uart1_Prt.TxEndptr)
    {
        Uart1_Prt.TxInptr = &Uart1_Prt.TxLocation[0];
    }
}
void UART2_Tx_Data(unsigned char *data, unsigned int len)
{
    if ((U2_Tx_Size - Uart2_Prt.TxCounter) >= len) {
        Uart2_Prt.TxInptr->start = &U2_Tx_Buff[Uart2_Prt.TxCounter];
    } else {
        Uart2_Prt.TxCounter      = 0;
        Uart2_Prt.TxInptr->start = U2_Tx_Buff;
    }
    memcpy(Uart2_Prt.TxInptr->start, data, len);
    Uart2_Prt.TxCounter += len;
    Uart2_Prt.TxInptr->end = &U2_Tx_Buff[Uart2_Prt.TxCounter - 1];
    Uart2_Prt.TxInptr++;
    if (Uart2_Prt.TxInptr == Uart2_Prt.TxEndptr) {
        Uart2_Prt.TxInptr = &Uart2_Prt.TxLocation[0];
    }
}

UART_HandleTypeDef UART_HandleType;
void UART1_Init(unsigned int bandrate)
{
    //配置串口1
    UART_HandleType.Instance = USART1;
    //设置波特率
    UART_HandleType.Init.BaudRate = bandrate;
    //关闭硬件流控
    UART_HandleType.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    //设置传输模式
    UART_HandleType.Init.Mode = UART_MODE_TX_RX;
    //设置校验模式-目前为无校验
    UART_HandleType.Init.Parity = UART_PARITY_NONE;
    //设置停止位长度
    UART_HandleType.Init.StopBits = UART_STOPBITS_1;
    //设置数据位长度
    UART_HandleType.Init.WordLength = UART_WORDLENGTH_8B;
    //初始化串口函数
    HAL_UART_Init(&UART_HandleType);
    // //开启串口中断接收函数
    // HAL_UART_Receive_IT(&UART_HandleType, Rx_Buff,20);
    //初始化指针
    UART1_Prt_Init();
    __HAL_UART_ENABLE_IT(&UART_HandleType, UART_IT_IDLE);   //开启串口空闲中断
    // 开启串口中断接收函数
    HAL_UART_Receive_IT(&UART_HandleType, Uart1_Prt.RxInptr->start = U1_Rx_Buff, U1_Rx_MAX);
}

UART_HandleTypeDef UART_HandleType2;
void UART2_Init(unsigned int bandrate)
{
    // 配置串口2
    UART_HandleType2.Instance = USART2;
    // 设置波特率
    UART_HandleType2.Init.BaudRate = bandrate;
    // 关闭硬件流控
    UART_HandleType2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    // 设置传输模式
    UART_HandleType2.Init.Mode = UART_MODE_TX_RX;
    // 设置校验模式-目前为无校验
    UART_HandleType2.Init.Parity = UART_PARITY_NONE;
    // 设置停止位长度
    UART_HandleType2.Init.StopBits = UART_STOPBITS_1;
    // 设置数据位长度
    UART_HandleType2.Init.WordLength = UART_WORDLENGTH_8B;
    // 初始化串口函数
    HAL_UART_Init(&UART_HandleType2);
    UART2_Prt_Init();
    __HAL_UART_ENABLE_IT(&UART_HandleType2, UART_IT_IDLE); // 开启串口空闲中断
    HAL_UART_Receive_IT(&UART_HandleType2, Uart2_Prt.RxInptr->start = U2_Rx_Buff, U2_Rx_MAX);
}
UART_HandleTypeDef UART_HandleType3;
DMA_HandleTypeDef hdma_usart3_rx;
UART_Rx_Struct EspRx = {0};
void UART3_Init(unsigned int bandrate)
{
    // 配置串口3
    UART_HandleType3.Instance = USART3;
    // 设置波特率
    UART_HandleType3.Init.BaudRate = bandrate;
    // 关闭硬件流控
    UART_HandleType3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    // 设置传输模式
    UART_HandleType3.Init.Mode = UART_MODE_TX_RX;
    // 设置校验模式-目前为无校验
    UART_HandleType3.Init.Parity = UART_PARITY_NONE;
    // 设置停止位长度
    UART_HandleType3.Init.StopBits = UART_STOPBITS_1;
    // 设置数据位长度
    UART_HandleType3.Init.WordLength = UART_WORDLENGTH_8B;
    // 初始化串口函数
    HAL_UART_Init(&UART_HandleType3);
    __HAL_UART_ENABLE_IT(&UART_HandleType3, UART_IT_IDLE); // 开启串口空闲中断
    HAL_UART_Receive_DMA(&UART_HandleType3, EspRx.RxBuffer, RX_BUFFER_SIZE);
}

//初始化串口函数的回调函数——用以初始化GPIO
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
       // 开启GPIOA时钟和UART1时钟
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();
			  __USART1_CLK_ENABLE();
			  __GPIOA_CLK_ENABLE();
        /*
        //串口1重映射使用
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_AFIO_CLK_ENABLE();
        __HAL_AFIO_REMAP_USART1_ENABLE();       //开启串口1的重映射
        */
			 // TX 引脚 (PA9)
        GPIO_InitTypeDef GPIO_InitType;
        GPIO_InitType.Pin = GPIO_PIN_9;
        GPIO_InitType.Mode = GPIO_MODE_AF_PP;
			  GPIO_InitType.Pull = GPIO_PULLUP;                 //上拉
        GPIO_InitType.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			  GPIO_InitType.Alternate =  GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);
        GPIO_InitType.Pin   = GPIO_PIN_10;
        GPIO_InitType.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitType.Pull  = GPIO_PULLUP;
			  GPIO_InitType.Alternate =  GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);

       /*中断控制*/
        // 设置中断源和优先级-中断源在stm32f103xb.h文件69行开始
        HAL_NVIC_SetPriority(USART1_IRQn,2,0);
        // 使能中断
        HAL_NVIC_EnableIRQ(USART1_IRQn);

    }
    else if (huart->Instance == USART2) {
        // 开启GPIOA时钟和UART2 时钟
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_InitType;
        GPIO_InitType.Pin   = GPIO_PIN_2;
        GPIO_InitType.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitType.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);
        GPIO_InitType.Pin  = GPIO_PIN_3;
        GPIO_InitType.Mode = GPIO_MODE_AF_PP;
        GPIO_InitType.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);
        // 设置中断源和优先级-中断源在stm32f103xb.h文件69行开始
        HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
        // 使能中断
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
    else if (huart->Instance == USART3) {
        // 开启GPIOA时钟和UART1时钟
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE(); // 开启 DMA1 时钟
        GPIO_InitTypeDef GPIO_InitType;
        GPIO_InitType.Pin = GPIO_PIN_8  | GPIO_PIN_9;
        GPIO_InitType.Mode = GPIO_MODE_AF_PP;
        GPIO_InitType.Pull = GPIO_PULLUP;
        GPIO_InitType.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitType.Alternate = GPIO_AF7_USART3; // 关键：复用映射到 AF7
        HAL_GPIO_Init(GPIOD, &GPIO_InitType);

        // 3. DMA 配置 (USART3_RX 对应 DMA1 Stream1 Channel4)
        hdma_usart3_rx.Instance = DMA1_Stream1;
        hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4; // 通道 4
        hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE; // 外设地址不增
        hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;     // 内存地址自增
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode = DMA_NORMAL; // 普通模式 (接收完一包需重置，或处理完重置)
                                               // 也可以用 DMA_CIRCULAR，但处理定长命令 NORMAL 逻辑更清晰
        hdma_usart3_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_usart3_rx);      //dma初始化
        // 关联 DMA 句柄到 UART 句柄
        __HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);
        // 4. 中断配置
        // DMA 中断
        HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
        // 设置中断源和优先级-中断源在stm32f103xb.h文件69行开始
        HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
        // 使能中断
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}
/**
 * @brief 发送字符串到 ESP8266
 */
void ESP_SendString(char *str)
{
    HAL_UART_Transmit(&UART_HandleType3, (uint8_t *)str, strlen(str), 100);
}
//串口接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // //判断谁触发的中断
    // if (huart->Instance == USART1)
    // {
    //     //C语言再带库string的拷贝函数
    //     memcpy(Tx_Buff, Rx_Buff, 20);
    //     //接收标志位置1
    //     //表示接收完成
    //     Rx_State = 1;
    //     //重新开启中断接收函数-接收完成后会直接关闭中断接收因此需要重新开启
    //     HAL_UART_Receive_IT(&UART_HandleType, Rx_Buff, 20);
    // }
}

//中断接收错误回调函数
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 判断谁触发的中断
    if (huart->Instance == USART1) {

    }
    else if (huart->Instance == USART2) {

    }
    else if (huart->Instance == USART3) {

    }
}
// 中断发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 判断谁触发的中断
    if (huart->Instance == USART1) {
        //发送完成清空标志位
        Uart1_Prt.TxStater = 0;
    }
    else if (huart->Instance == USART2) {
        // 发送完成清空标志位
        Uart2_Prt.TxStater = 0;
    }
    else if (huart->Instance == USART3) {

    }
}
//中断串口终止接收回调函数
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
    // 判断谁触发的中断
    if (huart->Instance == USART1) {
        Uart1_Prt.RxInptr->end = &U1_Rx_Buff[Uart1_Prt.RxCounter - 1];
        Uart1_Prt.RxInptr++;
        if (Uart1_Prt.RxInptr == Uart1_Prt.RxEndptr)
        {
            Uart1_Prt.RxInptr = &Uart1_Prt.RxLocation[0];
        }
        if ((U1_Rx_Size - Uart1_Prt.RxCounter) < U1_Rx_MAX)
        {
            Uart1_Prt.RxCounter = 0;
            // 储存缓冲区位置
            Uart1_Prt.RxInptr->start = U1_Rx_Buff;
        }
        else
        {
            Uart1_Prt.RxInptr->start = &U1_Rx_Buff[Uart1_Prt.RxCounter];
        }
        // 开启串口中断接收函数
        HAL_UART_Receive_IT(&UART_HandleType, Uart1_Prt.RxInptr->start, U1_Rx_MAX);
    }
}


void U1_print(char * fmt , ...){
    unsigned char TempBuff[256];
    unsigned int i = 0;
    va_list ap;
    va_start(ap,fmt);
    vsprintf((char *)TempBuff, fmt, ap);
    va_end(ap);
    for (i = 0; i < strlen((char *)TempBuff); i++)
    {
        //等待发送缓冲区空
        while (!__HAL_UART_GET_FLAG(&UART_HandleType, UART_FLAG_TXE));
        UART_HandleType.Instance->DR = TempBuff[i];
    }
    while (!__HAL_UART_GET_FLAG(&UART_HandleType, UART_FLAG_TC));
}
void U3_print(char *fmt, ...)
{
    unsigned char TempBuff[256];
    unsigned int i = 0;
    va_list ap;
    va_start(ap, fmt);
    vsprintf((char *)TempBuff, fmt, ap);
    va_end(ap);
    for (i = 0; i < strlen((char *)TempBuff); i++)
    {
        // 等待发送缓冲区空
        while (!__HAL_UART_GET_FLAG(&UART_HandleType3, UART_FLAG_TXE))
            ;
        UART_HandleType3.Instance->DR = TempBuff[i];
    }
    while (!__HAL_UART_GET_FLAG(&UART_HandleType3, UART_FLAG_TC))
        ;
}
