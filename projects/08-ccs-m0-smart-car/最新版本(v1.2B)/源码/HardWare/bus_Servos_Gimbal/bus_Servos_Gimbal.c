#include "bus_Servos_Gimbal.h"

//总线单舵机角度发送
/*
uint8_t index:要控制舵机的id -- 目前上舵机id：00 下舵机id：01
uint16_t pwm：控制舵机要转动的角度 -- 输入范围 500 -2500		//上舵机应机械结构原因最小角度为800
uint16_t time_ms：舵机完成指定角度需要的时间 --输入范围 0 - 9999
*/
void Send_SingleServo(uint8_t index, uint16_t pwm, uint16_t time_ms) {
        char cmd[16];
        sprintf(cmd, "#%03dP%04dT%04d!", index, pwm, time_ms);
        for(int i = 0 ; i < strlen(cmd) ;i++)
        {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)cmd[i]);
        }
}

//总线双舵机角度发送
/*
uint8_t index:要控制舵机的id -- 目前上舵机id：01 下舵机id：00
uint16_t pwm：控制舵机要转动的角度 -- 输入范围 500 -2500
uint16_t time_ms：舵机完成指定角度需要的时间 --输入范围 0 - 9999
*/
void Send_SingleServo_double(uint8_t index_1, uint16_t pwm_1, uint16_t time_ms_1 , uint8_t index_2, uint16_t pwm_2, uint16_t time_ms_2) {
        char cmd[33];
        sprintf(cmd, "{#%03dP%04dT%04d!#%03dP%04dT%04d!}", index_1, pwm_1, time_ms_1 , index_2, pwm_2 ,time_ms_2);
        for(int i = 0 ; i < strlen(cmd) ;i++)
        {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)cmd[i]);
        }
}
//除了第一行都为针对当前程序做的调整
unsigned char L_Flag = 0;
unsigned char R_Flag = 0;
void Bus_Servos_Gimbal_Init(void)
{
    Send_SingleServo_double(01,1500,100,00,2100,500); // 1500 2100 回正  //2000 2500 右侧准备抓取 // 2500 2500右侧抓取  // 2000 1700 左侧准备抓取
    if(Ok_falg == 2 && Left_size_flag == 0)
    {
        L_Flag = 150;
    }
    if(Left_size_flag == 1){
        Servos_Move(180);
        L_Flag = 150;
    }else if(Left_size_flag == 2){
        Servos_Move(0);
        R_Flag = 150;
    }
}

