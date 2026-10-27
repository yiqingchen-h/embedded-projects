 #include "IMU.h"

volatile unsigned char imu_data = 0;

void Imu_init(void)
{

    //NVIC_ClearPendingIRQ(UART_2_L_INST_INT_IRQN);//清除串口中断标志  不要在串口3使用！！！！！！！！！！！
    NVIC_EnableIRQ(UART_3_R_INST_INT_IRQN);//使能串口中断

}
unsigned char start = 0;
unsigned char data_length = 0;
unsigned char RxBuff[9];
unsigned char RxBuffnew[9];
int angle[3];
void GetDataDeal(void)
{
    if(RxBuffnew[0] == 0x53) //姿态角度输出
    {
        angle[0] = (RxBuffnew[2] << 8 | RxBuffnew[1]) / 32768.0 * 180.0;
        angle[1] = (RxBuffnew[4] << 8 | RxBuffnew[3]) / 32768.0 * 180.0;
        angle[2] = (int)(RxBuffnew[6] << 8 | RxBuffnew[5]) / 32768.0 * 180.0;
        printf("z:%d \r\n" ,angle[2]);                
    }
    memset(RxBuffnew,0,9);
}
Pid_Data revolve_Pid;
float Revolve_Pid(float target_value , float Measurements) // 目标值  测量值
{
    float out = 0;
    revolve_Pid.steering_Error = Measurements - target_value;      // 本次误差
    revolve_Pid.steering_Error_sum +=revolve_Pid.steering_Error;  // 误差累加
    revolve_Pid.steering_Error_difference = revolve_Pid.steering_Error - revolve_Pid.steering_last_Error;    //本次误差 和 上次误差 的差
    revolve_Pid.steering_last_Error = revolve_Pid.steering_Error; //记录上次误差

    revolve_Pid.steering_Error_sum = revolve_Pid.steering_Error_sum > 500 ?0:revolve_Pid.steering_Error_sum;  //处理累加值防止一直累加
    revolve_Pid.steering_Error_sum = revolve_Pid.steering_Error_sum < -500 ?0:revolve_Pid.steering_Error_sum;
    out = revolve_Pid.steering_Kp*revolve_Pid.steering_Error + revolve_Pid.steering_Kd * revolve_Pid.steering_Error_difference + revolve_Pid.steering_Error_sum * steering_Pid.steering_Ki;
    out = out > 90 ? 90:out;
    out = out < -90 ? -90:out;
    return out;
}
void DueData(uint8_t inputdata)  
{
    if (inputdata == 0x55 && start == 0)
    {
        start = 1;  
        data_length = 0;
        return;
    }
    if(start == 1)
    {
        if(inputdata == 0x53) //因为速率太高，不能获取全部信息，该历程只获取姿态角度
        {
            RxBuff[0] = 0x53;
            start = 2 ;
        }
        else
        {
            start = 0;
        }
    }     
    if (start == 2)
    {
        RxBuff[data_length] = inputdata; //保存数据
        data_length ++;
        if (data_length == 8) //接收到完整的数据
        {
            start = 0; //清0
            memcpy(RxBuffnew,RxBuff,9);
            memset(RxBuff,0,9);    
            GetDataDeal();              //处理数据获得对应角度
            int speed = Revolve_Pid(307,angle[2]);
            printf("speed:%d",speed);
            if(speed < 307){
                stepping_R(1,subdivision_16,speed);
                stepping_L(1,subdivision_16,speed);
            }else{
                    stepping_R(0,subdivision_16,speed);
                    stepping_L(0,subdivision_16,speed);
            }
            if(angle[2]<310 && angle[2] > 305){
                NVIC_DisableIRQ(UART_3_R_INST_INT_IRQN);    //关闭串口中断
                Stepper_Stop();
                DL_TimerG_startCounter(Trace_Timer_INST);   // 启动定时器
            }
        }   
    }
}

//串口的中断服务函数
void UART_3_R_INST_IRQHandler(void)
{
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_3_R_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
            //接收发送过来的数据保存在变量中
            imu_data = DL_UART_Main_receiveData(UART_3_R_INST);
            DueData(imu_data);//解析imu数据
            break;

        default://其他的串口中断
            break;
    }

}

