#include "stm32f1xx_hal.h"
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

unsigned char U3_Rx_Buff[U3_Rx_Size];
unsigned char U3_Tx_Buff[U3_Tx_Size];
//初始化指针
UCB Uart1_Prt;
UCB Uart2_Prt;
UCB Uart3_Prt;
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
void UART3_Prt_Init(void)
{
    Uart3_Prt.RxInptr   = &Uart3_Prt.RxLocation[0];
    Uart3_Prt.RxOutptr  = &Uart3_Prt.RxLocation[0];
    Uart3_Prt.RxEndptr  = &Uart3_Prt.RxLocation[9];
    Uart3_Prt.RxCounter = 0;
    // 储存缓冲区位置
    Uart3_Prt.RxInptr->start = U3_Rx_Buff;

    Uart3_Prt.TxInptr        = &Uart3_Prt.TxLocation[0];
    Uart3_Prt.TxOutptr       = &Uart3_Prt.TxLocation[0];
    Uart3_Prt.TxEndptr       = &Uart3_Prt.TxLocation[9];
    Uart3_Prt.TxCounter      = 0;
    Uart3_Prt.TxInptr->start = U3_Tx_Buff;
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
void UART3_Tx_Data(unsigned char *data, unsigned int len)
{
    if ((U3_Tx_Size - Uart3_Prt.TxCounter) >= len) {
        Uart3_Prt.TxInptr->start = &U3_Tx_Buff[Uart3_Prt.TxCounter];
    } else {
        Uart3_Prt.TxCounter      = 0;
        Uart3_Prt.TxInptr->start = U3_Tx_Buff;
    }
    memcpy(Uart3_Prt.TxInptr->start, data, len);
    Uart3_Prt.TxCounter += len;
    Uart3_Prt.TxInptr->end = &U3_Tx_Buff[Uart3_Prt.TxCounter - 1];
    Uart3_Prt.TxInptr++;
    if (Uart3_Prt.TxInptr == Uart3_Prt.TxEndptr) {
        Uart3_Prt.TxInptr = &Uart3_Prt.TxLocation[0];
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
    UART3_Prt_Init();
    __HAL_UART_ENABLE_IT(&UART_HandleType3, UART_IT_IDLE); // 开启串口空闲中断
    HAL_UART_Receive_IT(&UART_HandleType3, Uart3_Prt.RxInptr->start = U3_Rx_Buff, U3_Rx_MAX);
}

//初始化串口函数的回调函数——用以初始化GPIO
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
       // 开启GPIOA时钟和UART1时钟
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();
        /*
        //串口1重映射使用
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_AFIO_CLK_ENABLE();
        __HAL_AFIO_REMAP_USART1_ENABLE();       //开启串口1的重映射
        */
        GPIO_InitTypeDef GPIO_InitType;
        GPIO_InitType.Pin = GPIO_PIN_9;
        GPIO_InitType.Mode = GPIO_MODE_AF_PP;
        GPIO_InitType.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);
        GPIO_InitType.Pin   = GPIO_PIN_10;
        GPIO_InitType.Mode  = GPIO_MODE_AF_INPUT;
        GPIO_InitType.Pull  = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);
        /*
        //串口1重映射使用
                GPIO_InitType.Pin   = GPIO_PIN_6;
                GPIO_InitType.Mode  = GPIO_MODE_AF_PP;
                GPIO_InitType.Speed = GPIO_SPEED_FREQ_MEDIUM;
                HAL_GPIO_Init(GPIOB, &GPIO_InitType);
                GPIO_InitType.Pin  = GPIO_PIN_7;
                GPIO_InitType.Mode = GPIO_MODE_AF_INPUT;
                GPIO_InitType.Pull = GPIO_PULLUP;
                HAL_GPIO_Init(GPIOB, &GPIO_InitType);
        */

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
        GPIO_InitType.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);
        GPIO_InitType.Pin  = GPIO_PIN_3;
        GPIO_InitType.Mode = GPIO_MODE_AF_INPUT;
        GPIO_InitType.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);
        // 设置中断源和优先级-中断源在stm32f103xb.h文件69行开始
        HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
        // 使能中断
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
    else if (huart->Instance == USART3) {
        // 开启GPIOA时钟和UART1时钟
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_USART3_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_InitType;
        GPIO_InitType.Pin   = GPIO_PIN_10;
        GPIO_InitType.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitType.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOB, &GPIO_InitType);
        GPIO_InitType.Pin  = GPIO_PIN_11;
        GPIO_InitType.Mode = GPIO_MODE_AF_INPUT;
        GPIO_InitType.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitType);
        // 设置中断源和优先级-中断源在stm32f103xb.h文件69行开始
        HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
        // 使能中断
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
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
        // 发送完成清空标志位
        Uart3_Prt.TxStater = 0;
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
    else if (huart->Instance == USART2) {
        Uart2_Prt.RxInptr->end = &U2_Rx_Buff[Uart2_Prt.RxCounter - 1];
        Uart2_Prt.RxInptr++;
        if (Uart2_Prt.RxInptr == Uart2_Prt.RxEndptr) {
            Uart2_Prt.RxInptr = &Uart2_Prt.RxLocation[0];
        }
        if ((U2_Rx_Size - Uart2_Prt.RxCounter) < U2_Rx_MAX) {
            Uart2_Prt.RxCounter = 0;
            // 储存缓冲区位置
            Uart2_Prt.RxInptr->start = U2_Rx_Buff;
        } else {
            Uart2_Prt.RxInptr->start = &U2_Rx_Buff[Uart2_Prt.RxCounter];
        }
        // 开启串口中断接收函数
        HAL_UART_Receive_IT(&UART_HandleType2, Uart2_Prt.RxInptr->start, U2_Rx_MAX);
    }
    if (huart->Instance == USART3) {
        Uart3_Prt.RxInptr->end = &U3_Rx_Buff[Uart3_Prt.RxCounter - 1];
        Uart3_Prt.RxInptr++;
        if (Uart3_Prt.RxInptr == Uart3_Prt.RxEndptr) {
            Uart3_Prt.RxInptr = &Uart3_Prt.RxLocation[0];
        }
        if ((U3_Rx_Size - Uart3_Prt.RxCounter) < U3_Rx_MAX) {
            Uart3_Prt.RxCounter = 0;
            // 储存缓冲区位置
            Uart3_Prt.RxInptr->start = U3_Rx_Buff;
        } else {
            Uart3_Prt.RxInptr->start = &U3_Rx_Buff[Uart3_Prt.RxCounter];
        }
        // 开启串口中断接收函数
        HAL_UART_Receive_IT(&UART_HandleType3, Uart3_Prt.RxInptr->start, U3_Rx_MAX);
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

void U2_print(char *fmt, ...)
{
    unsigned char TempBuff[256];
    unsigned int i = 0;
    va_list ap;
    va_start(ap, fmt);
    vsprintf((char *)TempBuff, fmt, ap);
    va_end(ap);
    for (i = 0; i < strlen((char *)TempBuff); i++) {
        // 等待发送缓冲区空
        while (!__HAL_UART_GET_FLAG(&UART_HandleType2, UART_FLAG_TXE));
        UART_HandleType2.Instance->DR = TempBuff[i];
    }
    while (!__HAL_UART_GET_FLAG(&UART_HandleType2, UART_FLAG_TC));
}

void U3_print(char *fmt, ...)
{
    unsigned char TempBuff[256];
    unsigned int i = 0;
    va_list ap;
    va_start(ap, fmt);
    vsprintf((char *)TempBuff, fmt, ap);
    va_end(ap);
    for (i = 0; i < strlen((char *)TempBuff); i++) {
        // 等待发送缓冲区空
        while (!__HAL_UART_GET_FLAG(&UART_HandleType3, UART_FLAG_TXE));
        UART_HandleType3.Instance->DR = TempBuff[i];
    }
    while (!__HAL_UART_GET_FLAG(&UART_HandleType3, UART_FLAG_TC));
}
