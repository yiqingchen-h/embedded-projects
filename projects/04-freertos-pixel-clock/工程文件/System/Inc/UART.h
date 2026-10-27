#ifndef __UART_H
#define __UART_H
//接收缓冲区数组
#define U1_Rx_Size 2048
//一次性最大接收值
#define U1_Rx_MAX 256
//发送缓冲区数组
#define U1_Tx_Size 2048

// 接收缓冲区数组
#define U2_Rx_Size 2048
// 一次性最大接收值
#define U2_Rx_MAX 256
// 发送缓冲区数组
#define U2_Tx_Size 2048

#include "main.h"
typedef struct{
    unsigned char *start;
    unsigned char *end;
}LCB;//管理指针位置
typedef struct {
    unsigned int RxCounter;
    unsigned int TxCounter;
    unsigned char TxStater;
    LCB RxLocation[10];
    LCB TxLocation[10];
    LCB *RxInptr;
    LCB *RxOutptr;
    LCB *RxEndptr;
    LCB *TxInptr;
    LCB *TxOutptr;
    LCB *TxEndptr;
} UCB; // 管理串口指针位置
extern UCB Uart1_Prt;
extern UCB Uart2_Prt;
void UART1_Init(unsigned int bandrate);
void UART2_Init(unsigned int bandrate);


//发送数据
void UART1_Tx_Data(unsigned char *data, unsigned int len);
void UART2_Tx_Data(unsigned char *data, unsigned int len);
// 结构体外部声明方便轮询阻塞方式时调用
extern UART_HandleTypeDef UART_HandleType;
extern UART_HandleTypeDef UART_HandleType2;

// extern unsigned char Rx_State;
// extern unsigned char Tx_Buff[64];
// extern unsigned char Rx_Buff[64];
//轮询阻塞接收函数
//HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
//轮询阻塞发送函数
//HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)

void U1_print(char *fmt, ...);
void U2_print(char *fmt, ...);

// esp12f串口
#define RX_BUFFER_SIZE 3000 // 缓冲区大小
typedef struct
{
    uint8_t RxBuffer[RX_BUFFER_SIZE]; // 原始 DMA 接收缓冲
    uint16_t RxLen;                   // 本次接收到的数据长度
    uint8_t RxFlag;                   // 接收完成标志 (1:完成, 0:未完成)
} UART_Rx_Struct;                     // 接收管理结构体
extern UART_HandleTypeDef UART_HandleType3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_Rx_Struct EspRx;


void UART3_Init(unsigned int bandrate);
void U3_print(char *fmt, ...);

void ESP_SendString(char *str);
void ESP_Process_Data(void); // 主循环调用的处理函数
#endif
