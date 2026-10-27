#ifndef __UART_H
#define __UART_H
#include "stm32f1xx_hal.h"
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

// 接收缓冲区数组
#define U3_Rx_Size 2048
// 一次性最大接收值
#define U3_Rx_MAX 256
// 发送缓冲区数组
#define U3_Tx_Size 2048

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
extern UCB Uart3_Prt;

extern unsigned char U2_Rx_Buff[U2_Rx_Size];

void UART1_Init(unsigned int bandrate);
void UART2_Init(unsigned int bandrate);
void UART3_Init(unsigned int bandrate);

//发送数据
void UART1_Tx_Data(unsigned char *data, unsigned int len);
void UART2_Tx_Data(unsigned char *data, unsigned int len);
void UART3_Tx_Data(unsigned char *data, unsigned int len);
// 结构体外部声明方便轮询阻塞方式时调用
extern UART_HandleTypeDef UART_HandleType;
extern UART_HandleTypeDef UART_HandleType2;
extern UART_HandleTypeDef UART_HandleType3;
// extern unsigned char Rx_State;
// extern unsigned char Tx_Buff[64];
// extern unsigned char Rx_Buff[64];
//轮询阻塞接收函数
//HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
//轮询阻塞发送函数
//HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)

void U1_print(char *fmt, ...);
void U2_print(char *fmt, ...);
void U3_print(char *fmt, ...);

#endif
