#include "stm32f1xx_hal.h"
#include "RCC.h"
#include "UART.h"
int main()
{
    HAL_Init();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    RccClock_HSE_Init();
    UART1_Init(921600);
    UART2_Init(921600);
    UART3_Init(921600);
    U1_print("%d %c %x %s \r\n", 0x30, 0x30, 0x30,"U1");
    U2_print("%d %c %x %s \r\n", 0x30, 0x30, 0x30,"U2");
    U3_print("%d %c %x %s \r\n", 0x30, 0x30, 0x30,"U3");
    // 配置中断分组
    while (1) {
        // if (Rx_State == 1)
        // {
        //     Rx_State = 0;
        //     HAL_UART_Transmit_IT(&UART_HandleType, Tx_Buff,20);
        // }
        //串口1收发
        if(Uart1_Prt.RxOutptr != Uart1_Prt.RxInptr){
            UART1_Tx_Data(Uart1_Prt.RxOutptr->start, (Uart1_Prt.RxOutptr->end - Uart1_Prt.RxOutptr->start +1));
            Uart1_Prt.RxOutptr++;
            if (Uart1_Prt.RxOutptr == Uart1_Prt.RxEndptr)
            {
                Uart1_Prt.RxOutptr = &Uart1_Prt.RxLocation[0];
            }
        }
        if ((Uart1_Prt.TxOutptr != Uart1_Prt.TxInptr)&&(Uart1_Prt.TxStater != 1)) {
            Uart1_Prt.TxStater = 1;
            HAL_UART_Transmit_IT(&UART_HandleType, Uart1_Prt.TxOutptr->start, (Uart1_Prt.TxOutptr->end
            -Uart1_Prt.TxOutptr->start + 1));
            Uart1_Prt.TxOutptr++;
            if (Uart1_Prt.TxOutptr == Uart1_Prt.TxEndptr) {
                Uart1_Prt.TxOutptr = &Uart1_Prt.TxLocation[0];
            }
        }
        //串口2收发
        if (Uart2_Prt.RxOutptr != Uart2_Prt.RxInptr) {
            UART2_Tx_Data(Uart2_Prt.RxOutptr->start, (Uart2_Prt.RxOutptr->end - Uart2_Prt.RxOutptr->start + 1));
            Uart2_Prt.RxOutptr++;
            if (Uart2_Prt.RxOutptr == Uart2_Prt.RxEndptr) {
                Uart2_Prt.RxOutptr = &Uart2_Prt.RxLocation[0];
            }
        }
        if ((Uart2_Prt.TxOutptr != Uart2_Prt.TxInptr) && (Uart2_Prt.TxStater != 1)) {
            Uart2_Prt.TxStater = 1;
            HAL_UART_Transmit_IT(&UART_HandleType2, Uart2_Prt.TxOutptr->start, (Uart2_Prt.TxOutptr->end - Uart2_Prt.TxOutptr->start + 1));
            Uart2_Prt.TxOutptr++;
            if (Uart2_Prt.TxOutptr == Uart2_Prt.TxEndptr) {
                Uart2_Prt.TxOutptr = &Uart2_Prt.TxLocation[0];
            }
        }
        //串口3收发
        if (Uart3_Prt.RxOutptr != Uart3_Prt.RxInptr) {
            UART3_Tx_Data(Uart3_Prt.RxOutptr->start, (Uart3_Prt.RxOutptr->end - Uart3_Prt.RxOutptr->start + 1));
            Uart3_Prt.RxOutptr++;
            if (Uart3_Prt.RxOutptr == Uart3_Prt.RxEndptr) {
                Uart3_Prt.RxOutptr = &Uart3_Prt.RxLocation[0];
            }
        }
        if ((Uart3_Prt.TxOutptr != Uart3_Prt.TxInptr) && (Uart3_Prt.TxStater != 1)) {
            Uart3_Prt.TxStater = 1;
            HAL_UART_Transmit_IT(&UART_HandleType3, Uart3_Prt.TxOutptr->start, (Uart3_Prt.TxOutptr->end - Uart3_Prt.TxOutptr->start + 1));
            Uart3_Prt.TxOutptr++;
            if (Uart3_Prt.TxOutptr == Uart3_Prt.TxEndptr) {
                Uart3_Prt.TxOutptr = &Uart3_Prt.TxLocation[0];
            }
        }
    }
}
/*轮询接收方式*/
// // 将接收数据返回
// switch (HAL_UART_Receive(&UART_HandleType, buff, 200, 200)) {
//     // 判断接收成功
//     case HAL_OK:
//         HAL_UART_Transmit(&UART_HandleType, buff, 200, 200);
//         break;
//     // 接收超时
//     case HAL_TIMEOUT:
//         // 判断是否有接收到数据
//         // 因为RxXferCount的值在进入函数无论是否接收到数据就默认减一
//         // 因此判断是否接收值为要接收数据长度减一
//         if (UART_HandleType.RxXferCount != (200 - 1)) {
//             HAL_UART_Transmit(&UART_HandleType, buff, (200 - UART_HandleType.RxXferCount - 1), 200);
//             break;
//         }
//         break;
//      }
