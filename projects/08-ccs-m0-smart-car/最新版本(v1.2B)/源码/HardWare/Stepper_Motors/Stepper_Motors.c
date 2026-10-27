#include "Stepper_Motors.h"

//-----------
//驱动42*38步进电机时需要交换蓝绿两线的线序 否则电机会抖动
//-----------
unsigned short steps_Count_R = 0;
unsigned short stepping_Flag_R = 0;
unsigned short steps_Count_L = 0;
unsigned short stepping_Flag_L = 0;
void Stepper_Init_R(unsigned char level)
{
    uint32_t period = 0;
    
    switch(level) {
        case 0:  // 最快速度
            period = 100 - 1;
            break;
        case 1:
            period = 200 - 1;
            break;
        case 2:
            period = 300 - 1;
            break;
        // ... 其他速度等级
    }   
    
    // 配置MSPM0定时器
    DL_TimerG_setLoadValue(Stepping_STEP_R_INST, period); //设置Arr
    DL_TimerG_setCaptureCompareValue(Stepping_STEP_R_INST,period / 2, GPIO_Stepping_STEP_R_C0_IDX); // 50%占空比
}

void Stepper_Init_L(unsigned char level)
{
    uint32_t period = 0;
    
    switch(level) {
        case 0:  // 最快速度
            period = 100 - 1;
            break;
        case 1:
            period = 200 - 1;
            break;
        case 2:
            period = 300 - 1;
            break;
        // ... 其他速度等级
    }   
    
    // 配置MSPM0定时器
    DL_TimerA_setLoadValue(Stepping_STEP_L_INST, period); //设置Arr
    DL_TimerA_setCaptureCompareValue(Stepping_STEP_L_INST,period / 2, GPIO_Stepping_STEP_L_C1_IDX); // 50%占空比
}

//PWM 右电机溢出中断函数
void Stepping_STEP_L_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(Stepping_STEP_L_INST)) {  // 获取触发中断原因和判断优先级
        case DL_TIMER_IIDX_ZERO :   //  溢出触发中断
            if(steps_Count_L > 0) {
                steps_Count_L--;
            } 
            else if(steps_Count_L <= 0 && stepping_Flag_L == 0) {
                // 停止PWM输出
                DL_TimerA_stopCounter(Stepping_STEP_L_INST);
            }
            break;
        default:
            break;
    }    
}

//PWM 左边电机溢出中断函数
void Stepping_STEP_R_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(Stepping_STEP_R_INST)) {  // 获取触发中断原因和判断优先级
        case DL_TIMER_IIDX_ZERO :   //  溢出触发中断
            if(steps_Count_R > 0) {
                steps_Count_R--;
            } 
            else if(steps_Count_R <= 0 && stepping_Flag_R == 0) {
                // 停止PWM输出
                DL_TimerG_stopCounter(Stepping_STEP_R_INST);
            }
            break;
        default:
            break;
    }    
}

//细分设置
void Stepper_Subdivision_Init(unsigned char subdivision)
{
    switch (subdivision) {
    case subdivision_8:
        //电机1
        DL_GPIO_clearPins(Stepping_GPIO_MS1_1_PORT,Stepping_GPIO_MS1_1_PIN);
        DL_GPIO_clearPins(Stepping_GPIO_MS2_1_PORT,Stepping_GPIO_MS2_1_PIN);
        //电机2
        DL_GPIO_clearPins(Stepping_GPIO_MS1_2_PORT,Stepping_GPIO_MS1_2_PIN);
        DL_GPIO_clearPins(Stepping_GPIO_MS2_2_PORT,Stepping_GPIO_MS2_2_PIN);
    break;
    case subdivision_16:
    //电机1
        DL_GPIO_setPins(Stepping_GPIO_MS1_1_PORT,Stepping_GPIO_MS1_1_PIN);
        DL_GPIO_setPins(Stepping_GPIO_MS2_1_PORT,Stepping_GPIO_MS2_1_PIN);
        //电机2
        DL_GPIO_setPins(Stepping_GPIO_MS1_2_PORT,Stepping_GPIO_MS1_2_PIN);
        DL_GPIO_setPins(Stepping_GPIO_MS2_2_PORT,Stepping_GPIO_MS2_2_PIN);
        break;
    case subdivision_32:
        //电机1
        DL_GPIO_setPins(Stepping_GPIO_MS1_1_PORT,Stepping_GPIO_MS1_1_PIN);
        DL_GPIO_clearPins(Stepping_GPIO_MS2_1_PORT,Stepping_GPIO_MS2_1_PIN);
        //电机2
        DL_GPIO_setPins(Stepping_GPIO_MS1_2_PORT,Stepping_GPIO_MS1_2_PIN);
        DL_GPIO_clearPins(Stepping_GPIO_MS2_2_PORT,Stepping_GPIO_MS2_2_PIN);
        break;
    case subdivision_64:
          //电机1
        DL_GPIO_clearPins(Stepping_GPIO_MS1_1_PORT,Stepping_GPIO_MS1_1_PIN);
        DL_GPIO_setPins(Stepping_GPIO_MS2_1_PORT,Stepping_GPIO_MS2_1_PIN);
        //电机2
        DL_GPIO_clearPins(Stepping_GPIO_MS1_2_PORT,Stepping_GPIO_MS1_2_PIN);
        DL_GPIO_setPins(Stepping_GPIO_MS2_2_PORT,Stepping_GPIO_MS2_2_PIN);
        break;
    default:
        break;
}
}
//设置转动方向
void Stepper_direction_R(unsigned char direction)
{
    if (direction) {
        DL_GPIO_setPins(Stepping_GPIO_DIR_1_PORT,Stepping_GPIO_DIR_1_PIN);
    }
    else {
        DL_GPIO_clearPins(Stepping_GPIO_DIR_1_PORT,Stepping_GPIO_DIR_1_PIN);
    }
}

void Stepper_direction_L(unsigned char direction)
{
    if (direction) {
        DL_GPIO_setPins(Stepping_GPIO_DIR_2_PORT,Stepping_GPIO_DIR_2_PIN);
    }
    else {
        DL_GPIO_clearPins(Stepping_GPIO_DIR_2_PORT,Stepping_GPIO_DIR_2_PIN);
    }
}

// 控制电机旋转固定角度
/*
控制电机转动步数
step：角度 -- 可传正负值
Flog：当前细分
level: 速度
*/
void Stepper_Move_R(int step, unsigned char Flog,unsigned char level)
{
    stepping_Flag_R = 0;
    if(step > 0)
    {
        Stepper_direction_R(1);      // 设置方向
        DL_GPIO_clearPins(Stepping_GPIO_EN_1_PORT, Stepping_GPIO_EN_1_PIN); // 使能电机1
    }
    else if(step < 0){
        step = -step;
        Stepper_direction_R(0);      // 设置方向
        DL_GPIO_clearPins(Stepping_GPIO_EN_1_PORT, Stepping_GPIO_EN_1_PIN); // 使能电机1
    }
    else  {
        DL_GPIO_setPins(Stepping_GPIO_EN_1_PORT, Stepping_GPIO_EN_1_PIN); // 失能电机1
    }
    Stepper_Init_R(level);                    // 速度设置
    switch (Flog) {
        case subdivision_8:
            step *= 4.4444445;
            break;
        case subdivision_16:
            step *= 8.8888889;
            break;
        case subdivision_32:
            step *= 17.777778;
            break;
        case subdivision_64:
            step *= 35.555556;
            break;
        default:
            break;
    }
    steps_Count_R = step;

    DL_TimerG_startCounter(Stepping_STEP_R_INST); // 开启PWM输出
    //DL_Timer_clearInterruptFlag(TIMER0_INST, DL_TIMER_IIDX_ZERO); //防止开启中断后直接进入中断
    NVIC_EnableIRQ(Stepping_STEP_R_INST_INT_IRQN);   // 开启PWM中断
    
}

// 控制电机旋转固定角度
/*
控制电机转动步数
step：角度 -- 可传正负值
Flog：当前细分
level: 速度
*/
void Stepper_Move_L(int step, unsigned char Flog,unsigned char level)
{
    stepping_Flag_L = 0;
    if(step > 0)
    {
        Stepper_direction_L(1);      // 设置方向
        DL_GPIO_clearPins(Stepping_GPIO_EN_2_PORT, Stepping_GPIO_EN_2_PIN); // 使能电机2
    }
    else if(step < 0){
        step = -step;
        Stepper_direction_L(0);      // 设置方向
        DL_GPIO_clearPins(Stepping_GPIO_EN_2_PORT, Stepping_GPIO_EN_2_PIN); // 使能电机2 
    }
    else  {
        DL_GPIO_setPins(Stepping_GPIO_EN_2_PORT, Stepping_GPIO_EN_2_PIN); // 失能电机2 
    }
    Stepper_Init_L(level);                    // 速度设置
    switch (Flog) {
        case subdivision_8:
            step *= 4.4444445;
            break;
        case subdivision_16:
            step *= 8.8888889;
            break;
        case subdivision_32:
            step *= 17.777778;
            break;
        case subdivision_64:
            step *= 35.555556;
            break;
        default:
            break;
    }
    steps_Count_L = step;
    DL_TimerA_startCounter(Stepping_STEP_L_INST); // 开启PWM输出
    //DL_Timer_clearInterruptFlag(TIMER0_INST, DL_TIMER_IIDX_ZERO); //防止开启中断后直接进入中断
    NVIC_EnableIRQ(Stepping_STEP_L_INST_INT_IRQN);   // 开启PWM中断
    
}


//无极调节速度设置 -- 双电机控制
void Stepper_stepping_Init(unsigned char level , unsigned char Flog)
{
    uint32_t period = 0;
     switch (Flog) {
        case subdivision_8:
            period = 1015 - (level * 25 / 10);
            break;
        case subdivision_16:
            period = 415 - (level*3);
            break;
        case subdivision_32:
            period = 1015 - (level / 35 * 10);
            break;
        case subdivision_64:
            period = 415 - (level*3.5);
            break;
        default:
            break;
    }
    // 配置MSPM0定时器
    DL_Timer_setLoadValue(Stepping_STEP_L_INST, period); //设置Arr
    DL_Timer_setLoadValue(Stepping_STEP_R_INST, period); //设置Arr
    DL_TimerA_setCaptureCompareValue(Stepping_STEP_L_INST,period / 2, GPIO_Stepping_STEP_L_C1_IDX); // 50%占空比
    DL_TimerA_setCaptureCompareValue(Stepping_STEP_R_INST,period / 2, GPIO_Stepping_STEP_R_C0_IDX); // 50%占空比
}
//右电机控制
void Stepper_stepping_Init_R(unsigned char level , unsigned char Flog)
{
    uint32_t period = 0;
     switch (Flog) {
        case subdivision_8:
            period = 1015 - (level * 25 / 10);
            break;
        case subdivision_16:
            period = 615 - (level*3);
            break;
        case subdivision_32:
            period = 415 - (level / 35 * 10);
            break;
        case subdivision_64:
            period = 415 - (level*3.5);
            break;
        default:
            break;
    }
    // 配置MSPM0定时器
    DL_Timer_setLoadValue(Stepping_STEP_R_INST, period); //设置Arr
    DL_TimerA_setCaptureCompareValue(Stepping_STEP_R_INST,period / 2, GPIO_Stepping_STEP_R_C0_IDX); // 50%占空比
    //DL_TimerA_setCaptureCompareValue(Stepping_STEP_INST,period / 2, GPIO_Stepping_STEP_C3_IDX); // 50%占空比
}

//左电机控制
void Stepper_stepping_Init_L(unsigned char level , unsigned char Flog)
{
    uint32_t period = 0;
     switch (Flog) {
        case subdivision_8:
            // period = 415 - (level * 25 / 10);
             period = 1015 - (level * 25 / 10);
            break;
        case subdivision_16:
            period = 615 - (level * 3);
            break;
        case subdivision_32:
            period = 1015 - (level / 35 * 10); //level 值范围：0 - 290
            break;
        case subdivision_64:
            period = 415 - (level * 35 / 10);
            break;
        default:
            break;
    }
    // 配置MSPM0定时器
    DL_Timer_setLoadValue(Stepping_STEP_L_INST, period); //设置Arr
    //DL_TimerA_setCaptureCompareValue(Stepping_STEP_INST,period / 2, GPIO_Stepping_STEP_C1_IDX); // 50%占空比
    DL_TimerA_setCaptureCompareValue(Stepping_STEP_L_INST,period / 2, GPIO_Stepping_STEP_L_C1_IDX); // 50%占空比
}

// 步进运动
void stepping( unsigned char direction ,unsigned char Flog,unsigned char level)
{
    stepping_Flag_R = 1;
    stepping_Flag_L = 1;  
    if(level)
    {
        Stepper_direction_R(direction);      // 设置方向
        Stepper_stepping_Init(level,Flog);                    // 无极调节速度设置
        DL_GPIO_clearPins(Stepping_GPIO_EN_1_PORT, Stepping_GPIO_EN_1_PIN); // 使能电机1
        DL_GPIO_clearPins(Stepping_GPIO_EN_2_PORT, Stepping_GPIO_EN_2_PIN); // 使能电机2
        DL_TimerG_startCounter(Stepping_STEP_L_INST); // 开启PWM输出
        DL_TimerG_startCounter(Stepping_STEP_R_INST); // 开启PWM输出
        //DL_Timer_clearInterruptFlag(TIMER0_INST, DL_TIMER_IIDX_ZERO); //防止开启中断后直接进入中断
        //NVIC_EnableIRQ(Stepping_STEP_INST_INT_IRQN);   // 开启PWM中断
    }
    else {
        DL_GPIO_setPins(Stepping_GPIO_EN_1_PORT, Stepping_GPIO_EN_1_PIN); // 使能电机1
        DL_GPIO_setPins(Stepping_GPIO_EN_2_PORT, Stepping_GPIO_EN_2_PIN); // 使能电机2
    }
    
}
unsigned char Smooth_start_Flag_R = 30;
unsigned char Smooth_start_Flag_L = 30;

// 左电机步进运动
void stepping_R( unsigned char direction ,unsigned char Flog,unsigned char level)
{
    stepping_Flag_R = 1;
    if(level)
    {
        if (Smooth_start_Flag_R > 1){
            level /= (Smooth_start_Flag_R / 2);
            Smooth_start_Flag_R--;
        }
        Stepper_direction_R(direction);      // 设置方向
        Stepper_stepping_Init_R(level,Flog);                    // 无极调节速度设置
        DL_GPIO_clearPins(Stepping_GPIO_EN_1_PORT, Stepping_GPIO_EN_1_PIN); // 使能电机1
        DL_TimerG_startCounter(Stepping_STEP_R_INST); // 开启PWM输出
        //DL_Timer_clearInterruptFlag(TIMER0_INST, DL_TIMER_IIDX_ZERO); //防止开启中断后直接进入中断
        //NVIC_EnableIRQ(Stepping_STEP_INST_INT_IRQN);   // 开启PWM中断
    }
    else {
        DL_GPIO_setPins(Stepping_GPIO_EN_1_PORT, Stepping_GPIO_EN_1_PIN); // 使能电机1
    }
}

void stepping_L( unsigned char direction ,unsigned char Flog,unsigned char level)
{
    stepping_Flag_L = 1;
    if(level)
    {
        if (Smooth_start_Flag_L > 1) {
            level /= (Smooth_start_Flag_L / 2);
            Smooth_start_Flag_L--;
        }
        Stepper_direction_L(direction);      // 设置方向
        Stepper_stepping_Init_L(level,Flog);                    // 无极调节速度设置
        DL_GPIO_clearPins(Stepping_GPIO_EN_2_PORT, Stepping_GPIO_EN_2_PIN); // 使能电机1
        DL_TimerG_startCounter(Stepping_STEP_L_INST); // 开启PWM输出
        //DL_Timer_clearInterruptFlag(TIMER0_INST, DL_TIMER_IIDX_ZERO); //防止开启中断后直接进入中断
        //NVIC_EnableIRQ(Stepping_STEP_INST_INT_IRQN);   // 开启PWM中断
    }
    else {
        DL_GPIO_setPins(Stepping_GPIO_EN_2_PORT, Stepping_GPIO_EN_2_PIN); // 使能电机1
    }
}

void Stepper_Stop(void)
{
    Smooth_start_Flag_R = 30;
    Smooth_start_Flag_L = 30;
    DL_GPIO_setPins(Stepping_GPIO_EN_2_PORT, Stepping_GPIO_EN_2_PIN); // 失能电机2
    DL_GPIO_setPins(Stepping_GPIO_EN_1_PORT, Stepping_GPIO_EN_1_PIN); // 失能电机1
    DL_TimerG_stopCounter(Stepping_STEP_R_INST);                    // 关闭定时器
    DL_TimerA_stopCounter(Stepping_STEP_L_INST);
}


/* 函数：软件UART的CRC-8校验计算
 * 参数：datagram - 数据报数组指针（包含待校验数据和CRC存储位）
 *       datagramLength - 数据报总长度（含CRC字节）
 * 多项式：0x07（x⁸ + x² + x + 1）
 * 初始值：0x00
 * 计算规则：LSB优先（从字节最低位开始处理）
 */
void tmc2209_crc(unsigned char* datagram, unsigned char datagramLength)
{
    int i, j;
    // 定位CRC存储位置（数据报最后一个字节）
    unsigned char* crc = datagram + (datagramLength - 1);
    unsigned char currentByte;  // 当前处理的字节
    *crc = 0;           // 初始化CRC寄存器为0x00

    // 遍历数据报中除CRC外的所有字节
    for (i = 0; i < (datagramLength - 1); i++) {
        currentByte = datagram[i];  // 获取当前字节
        
        // 处理当前字节的8个位（从LSB到MSB）
        for (j = 0; j < 8; j++) {
            /* 核心计算逻辑：
             * 1. 比较CRC最高位与当前数据位（LSB）
             * 2. 若异或结果为1：CRC左移后异或多项式0x07
             * 3. 若异或结果为0：仅CRC左移
             */
            if ((*crc >> 7) ^ (currentByte & 0x01)) { 
                *crc = (*crc << 1) ^ 0x07;  // 多项式异或
            } else {
                *crc = (*crc << 1);         // 普通左移
            }
            currentByte >>= 1;  // 移出已处理的LSB，准备处理下一位
        } // 结束位处理
    } // 结束字节处理
}
/*开启内部细分并设置旋转方向
direction 为 假时 逆时针旋转 真时 顺时针旋转
*/
void Stepper_Uart_Init_R(unsigned char direction)
{
    unsigned char frame[8]  = { //设置顺时针旋转并开启内部细分设置
            0x05,
            0x00,
            (0x00|0x80),
            0x00,
            0x00,
            0x00,
            0x89,
            0xB7
    };
    if(direction == 0)
    {
        frame[6] = 0x81;
        frame[7] = 0xC7;
    }
    for (unsigned char i = 0; i < 8 ; i++)
    {
        DL_UART_Main_transmitDataBlocking(UART_3_R_INST, frame[i]);         //阻塞发送数据 可连续调用
    }
}

// 设置速度
void uart_set_velocity(unsigned char frame[], int velocity) { // fd: 串口文件描述符
    frame[0] = 0x05;                    // Sync byte
    frame[1] = 0x00;                    // 地址为 0 - 3  在串口模式下由普通模式细分配置引脚实现
    frame[2] = 0xA2;                    // 置速度寄存器地址 - 0x22 在写入时需要将寄存及| 0x80 得出0x22|0x80 = 0xA2
    frame[3] = (velocity >> 24) & 0xFF, // 数据高位在前
    frame[4] = (velocity >> 16) & 0xFF,
    frame[5] = (velocity >> 8)  & 0xFF,
    frame[6] = velocity & 0xFF,         // 数据低字节
    frame[7] = 0x00,                    // CRC占位
    tmc2209_crc(frame, 8);  // 计算CRC
}
// Uart——发送速度配置
void Stepper_Uart_control_R(long long speed)
{
    unsigned char frame[8];
    uart_set_velocity(frame,speed);
    for (unsigned char i = 0; i < 8 ; i++)
    {
        DL_UART_Main_transmitDataBlocking(UART_3_R_INST, frame[i]);         //阻塞发送数据 可连续调用
    }
}

/*
 * 生成TMC2209细分配置数组（仅修改MRES位）
 * microstep 细分值（支持256/128/64/32/16/8/4/2/0）
 * 返回值 5位细分配置数组
 */
uint8_t* tmc2209_set_microstep(uint8_t microstep) {
    //static uint8_t chopconf[4] = {0x53, 0x00, 0x01, 0x10}; // 默认256细分
    static uint8_t chopconf[4] = {0x00, 0x01, 0x00, 0x53}; // 默认256细分
    // 细分值到MRES的映射表（位24-27）[1,7]
    switch(microstep) {
        case 128: chopconf[2] = 0x11; chopconf[2] |= 0x10;break; // MRES=0001   //chopconf[3] |= 0x10 高细分开启插值
        case 64:  chopconf[2] = 0x12; chopconf[2] |= 0x10; break; // MRES=0010
        case 32:  chopconf[2] = 0x13; break; // MRES=0011
        case 16:  chopconf[2] = 0x14; break; // MRES=0100
        case 8:   chopconf[2] = 0x15; break; // MRES=0101
        case 4:   chopconf[2] = 0x16; break; // MRES=0110
        case 2:   chopconf[2] = 0x17; break; // MRES=0111
        case 0:   chopconf[2] = 0x18; break; // MRES=1000（全步）
        // 默认保持256细分（MRES=0000）
    }
    return chopconf;
}
/*
    细分设置
    无返回值
    输出需要的细分：256/128/64/32/16/8/4/2/0
*/
void Stepper_Uart_Subdivision_R(unsigned char Subdivision)
{
    unsigned char* Subdivision_data;
    Subdivision_data = tmc2209_set_microstep(Subdivision);
    unsigned char frame[8];
    frame[0] = 0x05;                               // Sync byte
    frame[1] = 0x00;                               // 地址为 0 - 3  在串口模式下由普通模式细分配置引脚实现
    frame[2] = 0x6C | 0x80;                        // 寄存器地址 - 0x22 在写入时需要将寄存及| 0x80 得出0x22|0x80 = 0xA2
    frame[3] = *Subdivision_data ++;
    frame[4] = *Subdivision_data ++;
    frame[5] = *Subdivision_data ++;
    frame[6] = *Subdivision_data;
    tmc2209_crc(frame,8); //获取校验位
    for (unsigned char i = 0; i < 8 ; i++)
    {
        DL_UART_Main_transmitDataBlocking(UART_3_R_INST, frame[i]);         //阻塞发送数据 可连续调用
    }
}




//以下为步进云台控制
//----------------------------------------------------------------------------------
//设置转动方向
void Stepper_direction_Up(unsigned char direction)
{
    if (direction) {
        DL_GPIO_setPins(Stepping_Gimbal_GPIO_Gimbal_DIR_Up_PORT,Stepping_Gimbal_GPIO_Gimbal_DIR_Up_PIN);
    }
    else {
        DL_GPIO_clearPins(Stepping_Gimbal_GPIO_Gimbal_DIR_Up_PORT,Stepping_Gimbal_GPIO_Gimbal_DIR_Up_PIN);
    }
}

void Stepper_direction_Down(unsigned char direction)
{
    if (direction) {
        DL_GPIO_setPins(Stepping_Gimbal_GPIO_Gimbal_DIR_Down_PORT,Stepping_Gimbal_GPIO_Gimbal_DIR_Down_PIN);
    }
    else {
        DL_GPIO_clearPins(Stepping_Gimbal_GPIO_Gimbal_DIR_Down_PORT,Stepping_Gimbal_GPIO_Gimbal_DIR_Down_PIN);
    }
}
unsigned short steps_Count_Up = 0;
unsigned short stepping_Flag_Up = 0;
unsigned short steps_Count_Down = 0;
unsigned short stepping_Flag_Down = 0;
void Stepper_Init_Up(unsigned char level)
{
    uint32_t period = 0;
    
    switch(level) {
        case 0:  // 最快速度
            period = 100 - 1;
            break;
        case 1:
            period = 200 - 1;
            break;
        case 2:
            period = 300 - 1;
            break;
        case 3:
            period = 400 - 1;
            break;
        case 4:
            period = 500 - 1;
            break;
        case 5:
            period = 600 - 1;
            break;
        case 6:
            period = 1000 - 1;
            break;
        case 7:
            period = 6000 - 1;
            break;
        // ... 其他速度等级
    }   
    // 配置MSPM0定时器
    DL_TimerG_setLoadValue(Stepping_Gimbal_STEP_Up_INST, period); //设置Arr
    DL_TimerG_setCaptureCompareValue(Stepping_Gimbal_STEP_Up_INST,period / 2, GPIO_Stepping_Gimbal_STEP_Up_C1_IDX); // 50%占空比
}

void Stepper_Init_Down(unsigned char level)
{
    uint32_t period = 0;
    
    switch(level) {
        case 0:  // 最快速度
            period = 100 - 1;
            break;
        case 1:
            period = 200 - 1;
            break;
        case 2:
            period = 300 - 1;
            break;
        case 6:
            period = 700 - 1;
            break;
        // ... 其他速度等级
    }   
    
    // 配置MSPM0定时器
    DL_TimerA_setLoadValue(Stepping_Gimbal_STEP_Down_INST, period); //设置Arr
    DL_TimerA_setCaptureCompareValue(Stepping_Gimbal_STEP_Down_INST,period / 2, GPIO_Stepping_Gimbal_STEP_Down_C0_IDX); // 50%占空比
}

// 控制电机旋转固定角度
/*
控制电机转动步数
step：角度 -- 可传正负值
Flog：当前细分
level: 速度
*/
void Stepper_Move_Up(int step, unsigned char Flog,unsigned char level)
{
    stepping_Flag_Up = 0;
    if(step > 0)
    {
        Stepper_direction_Up(1);      // 设置方向
        DL_GPIO_clearPins(Stepping_Gimbal_GPIO_Gimbal_EN_Up_PORT, Stepping_Gimbal_GPIO_Gimbal_EN_Up_PIN); // 使能电机1
    }
    else if(step < 0){
        step = -step;
        Stepper_direction_Up(0);      // 设置方向
        DL_GPIO_clearPins(Stepping_Gimbal_GPIO_Gimbal_EN_Up_PORT, Stepping_Gimbal_GPIO_Gimbal_EN_Up_PIN); // 使能电机1
    }
    else  {
        DL_GPIO_setPins(Stepping_Gimbal_GPIO_Gimbal_EN_Up_PORT, Stepping_Gimbal_GPIO_Gimbal_EN_Up_PIN); // 失能电机1
    }
    Stepper_Init_Up(level);                    // 速度设置
    switch (Flog) {
        case subdivision_8:
            step *= 4.4444445;
            break;
        case subdivision_16:
            step *= 8.8888889;
            break;
        case subdivision_32:
            step *= 17.777778;
            break;
        case subdivision_64:
            step *= 35.555556;
            break;
        default:
            break;
    }
    steps_Count_Up = step;

    DL_TimerG_startCounter(Stepping_Gimbal_STEP_Up_INST); // 开启PWM输出
    //DL_Timer_clearInterruptFlag(Stepping_Gimbal_STEP_Up_INST, DL_TIMER_IIDX_ZERO); //防止开启中断后直接进入中断
    NVIC_EnableIRQ(Stepping_Gimbal_STEP_Up_INST_INT_IRQN);   // 开启PWM中断
    
}

// 控制电机旋转固定角度
/*
控制电机转动步数
step：角度 -- 可传正负值
Flog：当前细分
level: 速度
*/
void Stepper_Move_Down(int step, unsigned char Flog,unsigned char level)
{
    stepping_Flag_Down = 0;
    if(step > 0)
    {
        Stepper_direction_Down(1);      // 设置方向
        DL_GPIO_clearPins(Stepping_Gimbal_GPIO_Gimbal_EN_Down_PORT, Stepping_Gimbal_GPIO_Gimbal_EN_Down_PIN); // 使能电机2
    }
    else if(step < 0){
        step = -step;
        Stepper_direction_Down(0);      // 设置方向
        DL_GPIO_clearPins(Stepping_Gimbal_GPIO_Gimbal_EN_Down_PORT, Stepping_Gimbal_GPIO_Gimbal_EN_Down_PIN); // 使能电机2 
    }
    else  {
        DL_GPIO_setPins(Stepping_Gimbal_GPIO_Gimbal_EN_Down_PORT, Stepping_Gimbal_GPIO_Gimbal_EN_Down_PIN); // 失能电机2 
    }
    Stepper_Init_Down(level);                    // 速度设置
    switch (Flog) {
        case subdivision_8:
            step *= 4.4444445;
            break;
        case subdivision_16:
            step *= 8.8888889;
            break;
        case subdivision_32:
            step *= 17.777778;
            break;
        case subdivision_64:
            step *= 35.555556;
            break;
        default:
            break;
    }
    steps_Count_Down = step;
    DL_TimerA_startCounter(Stepping_Gimbal_STEP_Down_INST); // 开启PWM输出
    //DL_Timer_clearInterruptFlag(Stepping_Gimbal_STEP_Down_INST, DL_TIMER_IIDX_ZERO); //防止开启中断后直接进入中断
    NVIC_EnableIRQ(Stepping_Gimbal_STEP_Down_INST_INT_IRQN);   // 开启PWM中断
    
}

//PWM 上电机溢出中断函数
void Stepping_Gimbal_STEP_Up_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(Stepping_Gimbal_STEP_Up_INST)) {  // 获取触发中断原因和判断优先级
        case DL_TIMER_IIDX_ZERO :   //  溢出触发中断
            if(steps_Count_Up > 0) {
                steps_Count_Up--;
            } 
            else if(steps_Count_Up <= 0 && stepping_Flag_Up == 0) {
                // 停止PWM输出
                DL_TimerA_stopCounter(Stepping_Gimbal_STEP_Up_INST);
            }
            break;
        default:
            break;
    }    
}

//PWM 左边电机溢出中断函数
void Stepping_Gimbal_STEP_Down_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(Stepping_Gimbal_STEP_Down_INST)) {  // 获取触发中断原因和判断优先级
        case DL_TIMER_IIDX_ZERO :   //  溢出触发中断
            if(steps_Count_Down > 0) {
                steps_Count_Down--;
            } 
            else if(steps_Count_Down <= 0 && stepping_Flag_Down == 0) {
                // 停止PWM输出
                DL_TimerG_stopCounter(Stepping_Gimbal_STEP_Down_INST);
            }
            break;
        default:
            break;
    }    
}



