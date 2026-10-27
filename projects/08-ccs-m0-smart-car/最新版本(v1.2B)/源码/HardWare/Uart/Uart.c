#include "Uart.h"

unsigned char Rx_Data[20];  //外部读取数据使用
struct Uart_0_Buff {
    unsigned char Rx_Data[120];
    unsigned char Tx_Data[100];
    unsigned char Rx_position;
    unsigned char Rx_New_Position;
    unsigned char Rx_Old_Position;
};
struct Uart_0_Buff U0;
void Uart_0_Init(void)
{
    U0.Rx_New_Position = 0;
    U0.Rx_Old_Position = 0;
    U0.Rx_position = 0;
    /*
    波特率设置函数
    uart：目标串口
    clockFreq：串口时钟频率
    baudRate“目标波特率
    */
    //DL_UART_configBaudRate(UART_Regs * uart,uint32_t clockFreq,uint32_t baudRate )
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);     // 清除中断标志位
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);           // 开启串口接收中断标志位
}
void U1_Rx(void)
{
    unsigned char i = 0;
    if(U0.Rx_New_Position != U0.Rx_Old_Position && U0.Rx_New_Position!=0)
    {
        for(i = 0; i < U0.Rx_New_Position - U0.Rx_Old_Position;i++)
        {
            Rx_Data[i] = U0.Rx_Data[i+U0.Rx_Old_Position];
        }
        U0.Rx_Old_Position = U0.Rx_New_Position;
    }
}
void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {    // 获取当前优先级最高的中断标志位
        case DL_UART_MAIN_IIDX_RX:                              // 接收数据标志位
            U0.Rx_Data[U0.Rx_position] = DL_UART_Main_receiveData(UART_0_INST);  // 接收数据
            U0.Rx_New_Position++;
            if(U0.Rx_New_Position < 100)
            {
                U0.Rx_position = U0.Rx_New_Position;
            }
            else {
                unsigned char size = U0.Rx_New_Position - U0.Rx_Old_Position;
                for (unsigned char i = 0; i < size ; i++)
                {
                    U0.Rx_Data[i] = U0.Rx_Data[U0.Rx_Old_Position + i];
                }
                U0.Rx_New_Position = size;
                U0.Rx_position = size;
                U0.Rx_Old_Position = 0; 
            }
            DL_UART_Main_transmitData(UART_0_INST, U0.Rx_Data[U0.Rx_position-1]);  // 发送数据请求--不能连续调用如连续调用两次则第一次可能会被第二次覆盖
            //DL_UART_Main_transmitDataBlocking(UART_0_INST, U0.Rx_Data[U0.Rx_position-1]);         //阻塞发送数据 可连续调用
            break;
        default:
            break;
    }
}
//重定向printf

int fputc(int c ,FILE* stream)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, c);
    return c;
}

int fputs(const char* restrict s , FILE* restrict stream)
{
    uint16_t i,len;
    len = strlen(s);
    for(i = 0; i < len;i++)
    {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, s[i]);
    }
    return 0;
}
int puts(const char* _prt)
{
    int count = fputs(_prt,stdout);
    count += fputs("\n",stdout);
    return count;
}

void Uart_1_Init(void)
{
    /*
    波特率设置函数
    uart：目标串口
    clockFreq：串口时钟频率
    baudRate“目标波特率
    */
    //DL_UART_configBaudRate(UART_Regs * uart,uint32_t clockFreq,uint32_t baudRate )
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);     // 清除中断标志位
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);           // 开启串口接收中断标志位
}
unsigned char OpenCv_Rx[100];
unsigned short restore_falg = 0;                        //等待磁吸完成标志位
unsigned i = 0;
unsigned char Paper_Size = 6;                           // 便于测试实际运行需要修改为0
unsigned char servos_location[6] = {1,1,1,1,1,1};                       // 便于测试实际运行需要修改为0
void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {    // 获取当前优先级最高的中断标志位
        case DL_UART_MAIN_IIDX_RX:                              // 接收数据标志位
             OpenCv_Rx[i] = DL_UART_Main_receiveData(UART_1_INST);  // 接收数据
             servos_location[i] = 1;
             Paper_Size++;
             i++;
             //------------------------------------------------ //磁吸使用
            // if(OpenCv_Rx[i] == 0x0A)
            // {
            //     Electromagnets_ON();
            //     if(Ok_falg == 0 && restore_falg == 0){
            //         restore_falg = 150;
            //         Stepper_Move_Up(120,subdivision_64,0);
            //     }
            //     else if(Ok_falg == 1 && restore_falg == 0){
            //         restore_falg = 150;
            //         Stepper_Move_Up(-120,subdivision_64,0);
            //     }
            // }
            //---------------------------------------------
            // i++;
            //DL_UART_Main_transmitData(UART_0_INST, U0.Rx_Data[U0.Rx_position-1]);  // 发送数据请求--不能连续调用如连续调用两次则第一次可能会被第二次覆盖
            //DL_UART_Main_transmitDataBlocking(UART_1_INST, OpenCv_Rx[i - 1]);         //阻塞发送数据 可连续调用
            break;
        default:
            break;
    }
}
unsigned char grabORput_size = 1;               //抓取或放弃标志
void UART_2_L_INST_IRQHandler(void)
{
    unsigned char Data = DL_UART_Main_receiveData(UART_2_L_INST);  // 接收数据
    if(Data == 0x0A)                                    // 看到图案 
        {
            Send_SingleServo(01,2400,100);
            Paper_Size--;
            restore_falg = 150;
        }
        else if(Data == 0x0B)
        {
            // grabORput_size++;
            // DL_TimerG_stopCounter(TIMER_0_INST);   // 关闭定时器
            // DL_TimerG_startCounter(Trace_Timer_INST);   // 启动定时器
            // if(Ok_falg == 0 && restore_falg == 0){
            //     restore_falg = 300;
            //     Stepper_Move_Up(-120,subdivision_16,1);
            // }
            // else if(Ok_falg == 1 && restore_falg == 0){
            //     restore_falg = 300;
            //     Stepper_Move_Up(120,subdivision_16,1);
            // }
        }
    DL_UART_Main_transmitDataBlocking(UART_2_L_INST, Data);         //阻塞发送数据 可连续调用
}