#include "Trace.h"
//获取8位数据中0的个数
unsigned char get_count_zeros(unsigned char Grayscale_Data) {
    unsigned char ones = 0;
    unsigned char n = Grayscale_Data;
    while (n) {
        ones++;
        n &= (n - 1); // 消除最低位的1
    }
    return 8 - ones; // 总位数减1的个数
}

// 只处理正常循迹 -- 只有两位灰度值
unsigned char Last_bit;
unsigned char get_Effective_bit(unsigned char Grayscale_Data)
{
    unsigned char i = 0;
    unsigned char Bit_R = 10;
    unsigned char Bit_L = -10;
    unsigned char return_Bit = 0;
    for (i = 0; i < 8 ;i++)
    {
        if(((Grayscale_Data >> i) & 0x01) == 0)
        {
            Bit_R = i + 1;
        }
    }
    for (i = 0; i < 8 ;i++)
    {
        if(((Grayscale_Data << i) & 0x80) == 0)
        {
            Bit_L = 8 - i;
        }
    }
    if(Bit_R - Bit_L <= 2 && Bit_R - Bit_L >= -2)
    {
        if(Bit_R > 4 || Bit_L > 4)
        {
           return_Bit =  (Bit_R > Bit_L) ?Bit_R : Bit_L;
        }
        else if(Bit_R <= 4 || Bit_L <= 4)
        {
           return_Bit =  (Bit_R < Bit_L) ?Bit_R : Bit_L;
        }
    }
    else {
        return Last_bit;
    }
    Last_bit = return_Bit;
    return return_Bit;
}
unsigned char count_zeros_abnormal_Count_Flag = 0;  // 异常0信号标志位
unsigned char Black_fast_flag = 0;                  // 黑色快标志位
unsigned char last_Count_zeros = 0;                 // 上一次记的0个数
unsigned char Left_flag = 0;                        // 左转标志位
unsigned char Right_flag = 0;                       // 右转标志位
unsigned char wait_stop = 0;                        // 转弯完成等待小车静止防止直接反向启动电机小车翘头
unsigned int prevent_coiled_flag = 0;              // 防止连续检测黑色块 -- 双重保护标志位
unsigned char Ok_falg = 0;                          // 完成圈数
unsigned char move = 0;                             // 检测空白后继续前进数标志
unsigned char white_flag = 0;                       // 检测纯白次数标志
unsigned char Left_size_flag = 0;                   // 左转时空白色标志位次数
unsigned short turnings_Ok_decelerate_flag = 0;     // 转弯完成后一段时间内减速标志位
unsigned int time_size = 0;                         // 记录大致时间
unsigned char last_Left_size_flag = 0;
float Read_Grayscale(void)
{
    // printf("Left_size_flag:%d\t\n",Left_size_flag);
    // printf("Black_fast_flag:%d\t\n",Black_fast_flag);
    // printf("prevent_coiled_flag:%d\t\n",prevent_coiled_flag);
    // printf("last_Count_zeros:%d\t\n",last_Count_zeros);
    // printf("Ok_falg:%d\t\n",Ok_falg);
    if((Left_size_flag == 1 || Left_size_flag == 2) && (Left_size_flag != last_Left_size_flag)){
            last_Left_size_flag = Left_size_flag;
            Bus_Servos_Gimbal_Init();
        }
    if((Ok_falg >= 4 && Paper_Size <= 0) || (Ok_falg >= 4 && Black_fast_flag == 1 && Left_flag == 1)){
        return 0x0E;}                                   //返回完成结束标志位
    if(wait_stop > 0){
        Stepper_Stop();
        wait_stop --;
        return 0xF;
    }
    float Measurements = 0xFF;     //pid传入的测量值
    unsigned char Grayscale_Data = get_Digital_Output_Grayscale();  //获取灰度值                                    
    //printf("Grayscale_Data:%d\t\n",Grayscale_Data);
    unsigned count_zeros = get_count_zeros(Grayscale_Data);
    if (count_zeros > 0 && count_zeros < 4)
    {
        unsigned char Effective_bit = get_Effective_bit(Grayscale_Data);  //正常循迹处理 -- 3个灰度及以下  （特殊场景需要单独处理）
        count_zeros_abnormal_Count_Flag = 0;                       //检测到非长白线将标志位置0
        if(Left_flag == 1){
            turnings_Ok_decelerate_flag = 400;                  
            if(Effective_bit != 4 && Effective_bit != 5 ){          // 没有回正仍然需要转弯
                return 0xA;
            }
            else {
                white_flag ++;
                Left_flag = 0;                                      // 成功回正左转标志位重置
                Stepper_Stop();                                     // 停止转动防止左转完成轮子处于停止状态直接启动
                wait_stop = 25;
                move = 50;
                return 0xF;
            }
        }
        if(Right_flag == 1){
            turnings_Ok_decelerate_flag = 100;
            if(move > 0){
                move --;
                return 0x0;
            }                                        
            if(Effective_bit != 4 && Effective_bit != 5 ){          // 没有回正仍然需要转弯
                return 0xB;
            }
            else {
                Right_flag = 0;                                      // 成功回正右转标志位重置
                Stepper_Stop();                                     // 停止转动防止右转完成轮子处于停止状态直接启动
                wait_stop = 25;
                move = 50;
                return 0xF;
            }
        }
        //printf("Effective_bit:%d\t\n",Effective_bit);
        switch (Effective_bit)
            {
                case 4:       // 正常行驶
                case 5:
                    Measurements = 0;
                    break;
                case 6:       // 轻微左偏
                    Measurements = -2.5;
                    break;
                case 7:     // 较大左偏
                    Measurements = -3.5;
                    break;
                case 8:     // 严重左偏
                    Measurements = -5;
                    break;
                case 3:      // 轻微右偏
                    Measurements = 2.5;
                    break;
                case 2:     // 较大右偏
                    Measurements = 3.5;
                    break;
                case 1:      // 严重右偏
                    Measurements = 5;
                    break;
                default:
                    Measurements = 0xFF;                // 未知位置继续上次运行速度尝试循迹Measurements
                    break;
            }
    }
    else if(count_zeros == 0){                          // 纯白
        if(move > 1){
                //printf("move:%d",move); 
                move --;
                return 0x0;
            }              
        count_zeros_abnormal_Count_Flag ++;
        if(count_zeros_abnormal_Count_Flag > 100) // 长时间白线
        {
            Stepper_Stop();
            DL_TimerG_stopCounter(Trace_Timer_INST);   // 关闭定时器
            count_zeros_abnormal_Count_Flag = 0;
            return 0xF;                                // 跳过后续Pid计算直接终止
        }
        if(Black_fast_flag == 1)            // 第一次黑色快后出线需要左转
        {
            count_zeros_abnormal_Count_Flag = 0;
            Left_flag = 1;
            return 0xA;  // 返回停止持续运转 -- 开启左转固定角度;
        }
        if(Black_fast_flag == 3)            // 第三次黑色快后出线需要右转
        {
            count_zeros_abnormal_Count_Flag = 0;
            Right_flag = 1;
            return 0xB;  // 返回停止持续运转 -- 开启右转固定角度;
        }
    return 0;                                          // 未知位置直行尝试循迹
    }
    //&& Grayscale_Data != 0x80 && Grayscale_Data != 0xE0 && Grayscale_Data != 0xC0
    else if((count_zeros >= 4 && last_Count_zeros == 0xB  && Black_fast_flag == 1 && white_flag >= 2) //第二次检测黑色块
    || (count_zeros >= 4 && Black_fast_flag != 1 && last_Count_zeros == 0xB)           // 第1次和第三次
    ){     // 检测到黑色块
        move = 50;                                                 // 由于车身增长添加移动标志位 在检测到需要转弯后在前进一段时间帮助转弯
        Black_fast_flag ++;
        Black_fast_flag = Black_fast_flag > 3 ? 1 : Black_fast_flag;
        time_size = 0;
        last_Count_zeros = 0;
        count_zeros_abnormal_Count_Flag = 0;                       // 检测到非长白线将标志位置0
        prevent_coiled_flag = 0;                                   // 防止连续检测黑色块 -- 双重保护
        if(Black_fast_flag == 2){
            turnings_Ok_decelerate_flag = 50;
        }
        if(Black_fast_flag == 3){
            move = 15;
            Ok_falg++; //圈数增加
            // if(Ok_falg == 2 && Left_size_flag == 0){
            //     // Servos_Move(180);                                           // 舵机左转
            //     // Bus_Servos_Gimbal_Init();
            //     //printf("************************************************************************");
            // }
            //Bus_Servos_Gimbal_Init();                                                                       // 回正
            white_flag = 0;
            }                        // 一次Black_fast_flag == 3 及运行一圈整                                   
    return 0;
    }
    count_zeros_abnormal_Count_Flag = 0;
    if(count_zeros <= 3 && last_Count_zeros != 0xB){                                          // 防止连续检测黑色快
        prevent_coiled_flag ++;
        if(((Left_size_flag == 1 && Ok_falg >= 1) || Left_size_flag == 0) && Black_fast_flag == 1){
            if(prevent_coiled_flag > 1000){                   // 防止连续检测黑色块 -- 双重保护
                last_Count_zeros = 0xB;
             }
        }else if(Black_fast_flag != 1){
            if(prevent_coiled_flag > 20){                   // 防止连续检测黑色块 -- 双重保护
                last_Count_zeros = 0xB;
             }
        } 
    }
    return Measurements; 
}

Pid_Data steering_Pid;
float Grayscale_Pid(float target_value , float Measurements) // 目标值  测量值
{
    float out = 0;
    steering_Pid.steering_Error = Measurements - target_value;      // 本次误差
    steering_Pid.steering_Error_sum +=steering_Pid.steering_Error;  // 误差累加
    steering_Pid.steering_Error_difference = steering_Pid.steering_Error - steering_Pid.steering_last_Error;    //本次误差 和 上次误差 的差
    steering_Pid.steering_last_Error = steering_Pid.steering_Error; //记录上次误差

    steering_Pid.steering_Error_sum = steering_Pid.steering_Error_sum > 500 ?0:steering_Pid.steering_Error_sum;  //处理累加值防止一直累加
    steering_Pid.steering_Error_sum = steering_Pid.steering_Error_sum < -500 ?0:steering_Pid.steering_Error_sum;
    out = steering_Pid.steering_Kp*steering_Pid.steering_Error + steering_Pid.steering_Kd * steering_Pid.steering_Error_difference + steering_Pid.steering_Error_sum * steering_Pid.steering_Ki;
    out = out > 90 ? 90:out;
    out = out < -90 ? -90:out;
    return out;
}


