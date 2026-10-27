//1.2版本 添加步进云台驱动+PWM舵机 -- 基本完成等待树莓派信息
#include "SYS.h"
#include "lcd.h"
#include "lcd_init.h"
float Measurements = 0;
float speed = 0;
unsigned char test_Speed_R = 0;
unsigned char test_Speed_L = 0;
int main(void){
    SYSCFG_DL_init();
    Uart_0_Init();
    Uart_1_Init();
    LCD_Init();//屏幕初始化
    //LCD_Fill(0,0,LCD_W,LCD_H,BLACK);//清全屏
    Bus_Servos_Gimbal_Init();                   //总线云台初始化 -- 回正
    NVIC_EnableIRQ(GPIO_Re_INT_IRQN);         // Re中断
    NVIC_EnableIRQ(Switch_INT_IRQN);
    NVIC_EnableIRQ(Trace_Timer_INST_INT_IRQN);      // 开启定时器中断
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);      // 开启定时器中断
    NVIC_EnableIRQ(UART_2_L_INST_INT_IRQN);           // 开启串口接收中断标志位 -- 使用蓝牙模块接收信号测试使用
    Servos_Init();//                             // 舵机初始化回正90°
    DL_UART_Main_transmitDataBlocking(UART_1_INST, 0x00);
    while(1){
        if(Ping() == 0){
                LCD_ShowString(0,16*3,"ping_OK",BLACK,WHITE,16,0);
                break;
            }
       }
    LCD_ShowString(0,16*4,"Kp:",BLACK,WHITE,16,0);
    LCD_ShowString(0,16*5,"Kd:",BLACK,WHITE,16,0);
    LCD_ShowString(0,16*6,"Measurements:",BLACK,WHITE,16,0);
    LCD_ShowString(0,16*7,"Pid_speed:",BLACK,WHITE,16,0);
    LCD_ShowString(0,16*8,"Speed_L:",BLACK,WHITE,16,0);
    LCD_ShowString(8*15,16*8,"Speed_R:",BLACK,WHITE,16,0);
    LCD_ShowString(0,16*9,"Black_fast_flag:",BLACK,WHITE,16,0);
    //原地掉头PID参数
    revolve_Pid.steering_Kp = 10;
    revolve_Pid.steering_Ki = 0;   
    revolve_Pid.steering_Kd = 0;   
    //循迹PID参数
    steering_Pid.steering_Kp = 4.7; // 8 细分 Kp = 11.5    16细分 kp = 4.7  （应加装总线云台小车重量大大增加 现增大Kp至 8.7）
    steering_Pid.steering_Ki = 0;   //      16细分 ki = 0.08
    steering_Pid.steering_Kd = -0.8;   // 8 细分 Kd = -2.7      16细分 kd = -0.8
    Stepper_Subdivision_Init(subdivision_16);  // 在串口模式下用以配置地址
    // Stepper_Uart_Init_R(0);
    // Stepper_Uart_Subdivision_R(8);            // 8细分时最大值3000 最小值   
    float Re = 0;
    unsigned char Key = 0;
    unsigned char Pid_mode = 0;
    Servos_Move(0);                                         //舵机右转
    Send_SingleServo_double(01,2000,500,00,2500,100); //云台右转 准备磁吸
    while(1)
    {
        GetDataDeal();
        if(Ok_falg == 2 && OpenCv_Rx[0] == 0)
        {
            DL_UART_Main_transmitDataBlocking(UART_1_INST, 0xFF);         //阻塞发送数据 可连续调用
            LCD_Fill(0,0,LCD_W,LCD_H,BLACK);//清全屏
        }
        if (Ok_falg == 2 && OpenCv_Rx[0] != 0)
        {
            while(1)
            {
                LCD_ShowString(0,16*4,"shape:",BLACK,WHITE,16,0);
                LCD_ShowString(0,16*5,"shape:",BLACK,WHITE,16,0);
                LCD_ShowString(0,16*6,"shape:",BLACK,WHITE,16,0);
                LCD_ShowString(0,16*7,"shape:",BLACK,WHITE,16,0);
                LCD_ShowString(0,16*8,"shape:",BLACK,WHITE,16,0);
                LCD_ShowString(0,16*9,"shape:",BLACK,WHITE,16,0);
                LCD_ShowString(0,16*10,"shape:",BLACK,WHITE,16,0);
                //LCD_ShowString(0,16*11,"shape:",BLACK,WHITE,16,0);
                LCD_ShowIntNum(8*7,16*4,OpenCv_Rx[0],2,BLACK,WHITE,16);
                LCD_ShowIntNum(8*7,16*5,OpenCv_Rx[2],2,BLACK,WHITE,16);
                LCD_ShowIntNum(8*7,16*6,OpenCv_Rx[4],2,BLACK,WHITE,16);
                LCD_ShowIntNum(8*7,16*7,OpenCv_Rx[6],2,BLACK,WHITE,16);
                LCD_ShowIntNum(8*7,16*8,OpenCv_Rx[8],2,BLACK,WHITE,16);
                LCD_ShowIntNum(8*7,16*9,OpenCv_Rx[10],2,BLACK,WHITE,16);
                LCD_ShowIntNum(8*7,16*10,OpenCv_Rx[12],2,BLACK,WHITE,16);
                //LCD_ShowIntNum(8*7,16*11,OpenCv_Rx[14],2,BLACK,WHITE,16);
                
                LCD_ShowString(8*10,16*4,"color:",BLACK,WHITE,16,0);
                LCD_ShowString(8*10,16*5,"color:",BLACK,WHITE,16,0);
                LCD_ShowString(8*10,16*6,"color:",BLACK,WHITE,16,0);
                LCD_ShowString(8*10,16*7,"color:",BLACK,WHITE,16,0);
                LCD_ShowString(8*10,16*8,"color:",BLACK,WHITE,16,0);
                LCD_ShowString(8*10,16*9,"color:",BLACK,WHITE,16,0);
                LCD_ShowString(8*10,16*10,"color:",BLACK,WHITE,16,0);
                //LCD_ShowString(8*10,16*11,"color:",BLACK,WHITE,16,0);
                LCD_ShowIntNum(8*17,16*4,OpenCv_Rx[1],2,BLACK,WHITE,16);
                LCD_ShowIntNum(8*17,16*5,OpenCv_Rx[3],2,BLACK,WHITE,16);
                LCD_ShowIntNum(8*17,16*6,OpenCv_Rx[5],2,BLACK,WHITE,16);
                LCD_ShowIntNum(8*17,16*7,OpenCv_Rx[7],2,BLACK,WHITE,16);
                LCD_ShowIntNum(8*17,16*8,OpenCv_Rx[9],2,BLACK,WHITE,16);
                LCD_ShowIntNum(8*17,16*9,OpenCv_Rx[11],2,BLACK,WHITE,16);
                LCD_ShowIntNum(8*17,16*10,OpenCv_Rx[13],2,BLACK,WHITE,16);
                //LCD_ShowIntNum(8*17,16*11,OpenCv_Rx[15],2,BLACK,WHITE,16);
            }
            
        }
        Re = Get_Re();
        Key = Get_switch();
        switch(Key)
        {
            case 1:
                Pid_mode ++;
                Pid_mode = Pid_mode > 2 ? 1 :Pid_mode;
                break;
            case 2:
                //Imu_init();                                 // 开启脱落仪接收  -- 原地转向
                 DL_TimerG_startCounter(Trace_Timer_INST);   // 启动定时器
                break;
            case 3:
                //Stepper_Uart_control_R(test_speed);
                DL_TimerG_stopCounter(Trace_Timer_INST);   // 关闭定时器
                Stepper_Stop();
                break;
            default:
                break;
        }
        switch (Pid_mode)
        {
            case 1:
                revolve_Pid.steering_Kp += Re;
                LCD_ShowString(0,16*3,"current _Kp",BLACK,WHITE,16,0);
                break;
            case 2:
                revolve_Pid.steering_Kd += Re;
                LCD_ShowString(0,16*3,"current _Kd",BLACK,WHITE,16,0);
                break;
            default:
                LCD_ShowString(0,16*3,"current _NO",BLACK,WHITE,16,0);
                break;
        }
        LCD_ShowFloatNum1(8*3,16*4,revolve_Pid.steering_Kp,4,BLACK,WHITE,16);
        LCD_ShowFloatNum1(8*3,16*5,revolve_Pid.steering_Kd,4,BLACK,WHITE,16);
        LCD_ShowFloatNum1(8*13,16*6,Measurements,4,BLACK,WHITE,16);
        LCD_ShowFloatNum1(8*10,16*7,speed,4,BLACK,WHITE,16);
        LCD_ShowIntNum(8*9,16*8,test_Speed_R,3,BLACK,WHITE,16);
        LCD_ShowIntNum(8*24,16*8,test_Speed_L,3,BLACK,WHITE,16);
        LCD_ShowIntNum(8*17,16*9,Black_fast_flag,3,BLACK,WHITE,16);

    }
}
// 因为云台的使用导致小车单侧重量大大增加因此再次降低小车速度
void Trace_Timer_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(Trace_Timer_INST)) {  // 获取触发中断原因和判断优先级
        case DL_TIMER_IIDX_ZERO:   // 0 触发中断  -- 计数器值为0时触发
            if(Black_fast_flag == 1){
                time_size++;
                if(time_size > 500 && time_size < 2500 && Ok_falg >= 2){
                    Left_size_flag = 1;
                }else if(time_size >= 2500 && Ok_falg >= 2){
                    Left_size_flag = 2;
                }
            }
            Measurements = Read_Grayscale();
            //printf("Measurements:%f\t\n",Measurements);
            if(Measurements == 0xE){                        // 结束停止
                Stepper_Stop();
                DL_TimerG_stopCounter(Trace_Timer_INST);   // 关闭定时器
                break;
            }
            //printf("Measurements:%f\t\n",Measurements);
            if (Measurements == 0xFF || Measurements == 0xF)break;
            //---------------------------------------           //左转
            if(Measurements == 0x0A){       //左转标志位
            stepping_R(0,subdivision_16,5);        // 0 70
            stepping_L(0,subdivision_16,1);        // 0 20
            test_Speed_R = 70;
            test_Speed_L = 0;
            //stepping_L(1,subdivision_8,0);
            break;
            }
            if(Measurements == 0x0B){       //右转标志位
            stepping_R(1,subdivision_16,5);        // 0 20
            stepping_L(1,subdivision_16,5);        // 0 60
            test_Speed_R = -20;
            test_Speed_L = 70;
            //stepping_L(1,subdivision_8,0);
            break;
            }
            if(turnings_Ok_decelerate_flag == 0){
               steering_Pid.steering_Kp = 4.7; 
            }
            //-----------------------------------------         //正常循迹
            speed = Grayscale_Pid(0,Measurements);  // 右偏返回正数
            unsigned char speed_R = 120 + speed;                    //120 +
            unsigned char speed_L = 120 - speed;                    //120 -
            if(turnings_Ok_decelerate_flag > 1){
                steering_Pid.steering_Kp = 9.5;
                turnings_Ok_decelerate_flag --;
                if(turnings_Ok_decelerate_flag < 3){
                steering_Pid.steering_Kp = 4.7;
                }
                speed_R -= 70;
                speed_L -= 70;
            }
            stepping_R(0,subdivision_16,speed_R);
            stepping_L(1,subdivision_16,speed_L);
            test_Speed_R = 120 + speed;
            test_Speed_L = 120 - speed;
            break;
            //-----------------------------------------
        default:
            break;
    }
    if(L_Flag > 1){
        L_Flag --;
        if(L_Flag == 1){
            Send_SingleServo_double(01,2000,500,00,1700,100); //云台左转 准备磁吸  //延后左转 转动过快时会撞击
        }
    }
    if(R_Flag > 1){
        R_Flag --;
        if(R_Flag == 1){
            Send_SingleServo_double(01,2000,500,00,2500,100); //云台右转 准备磁吸  //延后左转 转动过快时会撞击
        }
    }
    if (restore_falg > 1){          //等待磁吸完成
            restore_falg --;
            if(restore_falg == 1){
                    //printf("restore:---------------\t\n");
                    Send_SingleServo(01,2000,100);
                    restore_falg = 0;
            }
        }
}

void TIMER_0_INST_IRQHandler(void)
{
 switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {  // 获取触发中断原因和判断优先级
        case DL_TIMER_IIDX_ZERO:   // 0 触发中断  -- 计数器值为0时触发
            break;
        default:
            break;
    }
}
