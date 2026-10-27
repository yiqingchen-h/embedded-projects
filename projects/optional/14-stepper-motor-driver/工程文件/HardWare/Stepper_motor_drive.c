#include "stm32f1xx_hal.h"
#include "Stepper_motor_drive.h"
TIM_HandleTypeDef TIM_HandleType2;
void TIM2_PWM_Init(unsigned int arr, unsigned int psc)
{
    TIM_HandleType2.Instance         = TIM2; // 需要配置的定时器选择
    TIM_HandleType2.Init.Prescaler   = psc;
    TIM_HandleType2.Init.CounterMode = TIM_COUNTERMODE_UP; // 选择计数模式
    TIM_HandleType2.Init.Period      = arr;
    // TIM_HandleType2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; // 时钟分频因子 - 对基础定时无用
    TIM_HandleType2.Init.RepetitionCounter = 0;
    TIM_HandleType2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_PWM_Init(&TIM_HandleType2); // 初始化PWM基础参
    // 在调用初始化函数HAL_TIM_****_Init时会产生更新事件位以用来更新psc和rep的值因此在初始化后需立即清除事件标识位
    // 防止初始化完成后不清除标志位 在开启定时器时直接进入定时器事件
    __HAL_TIM_CLEAR_FLAG(&TIM_HandleType2, TIM_FLAG_UPDATE); // 清除标志位

    TIM_OC_InitTypeDef TIM2_OC_InitType_PWM = {0};                 // PWM没有专用结构体
    TIM2_OC_InitType_PWM.OCMode             = TIM_OCMODE_PWM1;     // 选择PWM1模式
    TIM2_OC_InitType_PWM.Pulse              = arr / 2;             // 设置比较值ccr
    TIM2_OC_InitType_PWM.OCPolarity         = TIM_OCPOLARITY_HIGH; // 设置有效电平
    TIM2_OC_InitType_PWM.OCFastMode         = TIM_OCFAST_DISABLE;  // 关闭快速模式

    HAL_TIM_PWM_ConfigChannel(&TIM_HandleType2, &TIM2_OC_InitType_PWM, TIM_CHANNEL_1); // 设置PWM通道
    __HAL_TIM_MOE_ENABLE(&TIM_HandleType2);                                            // 使能主输出
                                                                                       //   HAL_TIM_PWM_Start(&TIM_HandleType2, TIM_CHANNEL_1);     //轮询打开通道1
    //HAL_TIM_PWM_Start_IT(&TIM_HandleType2, TIM_CHANNEL_1);                             // 中断打开通道1
}
// PWM初始化硬件回调函数
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitType;
    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        // PA8:PWM输出脚  PA1：DIR（旋转方向控制脚高电平正转） PA2：EN使能脚 低电平运转
        GPIO_InitType.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
        GPIO_InitType.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitType.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);
        //-----------------------------------------------------------------------------------------------------中断模式使用
        HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}
// PWM中断回调函数
unsigned short steps_Count = 0;
unsigned char stepping_Flag = 0;
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // 判断产生中断的通道
        {
            if (steps_Count > 0) {
                steps_Count--;
            } else if (steps_Count <= 0 && stepping_Flag == 0) {
                HAL_TIM_PWM_Stop_IT(&TIM_HandleType2, TIM_CHANNEL_1);
            }
        }
    }
}
/*步进电机速度控制*/
//level : 0 - 4
//0 运行速度最快
void Stepper_Init(unsigned char level)
{
    // arr = 200 - 1 psc = 72 - 1 此时频率1Mhz 输出脉冲1us 为步进电机的最快速度
    // 因为当前为64细分受step口速率影响只有level:0的速度较正常其他速度都过慢
    switch (level) {
        case 0:
            TIM2_PWM_Init(200 - 1, 72 - 1);
            break;
        case 1:
            TIM2_PWM_Init(200 - 1, 720 - 1);
            break;
        case 2:
            TIM2_PWM_Init(2000 - 1, 720 - 1);
            break;
        case 3:
            TIM2_PWM_Init(2000 - 1, 7200 - 1);
            break;
        case 4:
            TIM2_PWM_Init(20000 - 1, 7200 - 1);
            break;
        default:
            break;
    }
    
}

// 控制电机旋转固定角度
/*
控制电机转动步数
step：角度
direction：方向
Flog：当前细分
*/
void Stepper_Move(uint32_t step, GPIO_PinState direction, unsigned char Flog)
{
    stepping_Flag = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, direction);      // 设置方向
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // 使能电机
    HAL_TIM_PWM_Start_IT(&TIM_HandleType2, TIM_CHANNEL_1);
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
    steps_Count = step;
}
// 步进运动
//--不完善
void stepping(uint32_t step, GPIO_PinState direction, unsigned char Flog)
{
    stepping_Flag = 1;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, direction);      // 设置方向
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // 使能电机
    HAL_TIM_PWM_Start_IT(&TIM_HandleType2, TIM_CHANNEL_1);
    switch (Flog) {
        case subdivision_8:
        //通过更改arr的值来控制脉冲信号快慢来调整速度
        __HAL_TIM_SET_AUTORELOAD(&TIM_HandleType2, step);
        break;
        case subdivision_16:

            break;
        case subdivision_32:

            break;
        case subdivision_64:

            break;
        default:
            break;
    }
}
