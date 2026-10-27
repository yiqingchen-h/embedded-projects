#include "stm32f1xx_hal.h"
#include "Time.h"
void Speed_Init(void)
{
    GPIO_InitTypeDef GPIO_InitType;
    // 电机运行方向切换
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitType.Pin   = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitType.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitType.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitType);
}

void Set_Speend(int PWM_CCR)
{
    if(PWM_CCR > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    else if(PWM_CCR < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    PWM_CCR =(PWM_CCR > 0 ?PWM_CCR :-PWM_CCR);       //保证RCC为正数
    PWM_CCR = (PWM_CCR < 700 ? PWM_CCR : 700);      //限幅
    PWM_CCR = (PWM_CCR > 50 ? PWM_CCR : 0);         //滤除不会使电机转动PWM波
    __HAL_TIM_SET_COMPARE(&TIM_HandleType1, TIM_CHANNEL_4, PWM_CCR);
}



