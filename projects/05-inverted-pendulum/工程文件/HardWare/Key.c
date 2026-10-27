#include "stm32f1xx_hal.h"
 
void Key_Init(){
    // 开启APB2_GPIOB时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // 恢复GPIO默认状态
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_9 | GPIO_PIN_10);
    // 配置GPIO模式
    GPIO_InitTypeDef GPIO_InitType;
    GPIO_InitType.Mode = GPIO_MODE_INPUT;
    GPIO_InitType.Pin  = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_9 | GPIO_PIN_10;
    //输入模式有效
    GPIO_InitType.Pull = GPIO_PULLUP;
    //输出模式有效
    //GPIO_InitType.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitType);
}

unsigned char set = 0;
unsigned char set1 = 0;
/*返回值：按下的按键
抬起按键返回 --目前只有抬起返回
 */
unsigned char Key_Scan(void)
{
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0 && set == 0) { //s9
        unsigned int i = 0x02FF0;
        set = 1;
        while (i--) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 1) {
                set = 0;
                return 0;
            }
        }

    } else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 1 && set == 1) {
        unsigned int i = 0x02FF0;
        while (i--) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0) {
                set = 0;
                return 0;
            }
        }
        set = 0;
        return 1;
    }
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0 && set == 0) { // s9
        unsigned int i = 0x02FF0;
        set            = 1;
        while (i--) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 1) {
                set = 0;
                return 0;
            }
        }

    } else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 1 && set == 1) {
        unsigned int i = 0x02FF0;
        while (i--) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0) {
                set = 0;
                return 0;
            }
        }
        set = 0;
        return 9;
    }
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0 && set == 0) { // s9
        unsigned int i = 0x02FF0;
        set            = 1;
        while (i--) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1) {
                set = 0;
                return 0;
            }
        }

    } else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1 && set == 1) {
        unsigned int i = 0x02FF0;
        while (i--) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0) {
                set = 0;
                return 0;
            }
        }
        set = 0;
        return 13;
    }
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0 && set == 0) { // s9
        unsigned int i = 0x02FF0;
        set            = 1;
        while (i--) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 1) {
                set = 0;
                return 0;
            }
        }

    } else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 1 && set == 1) {
        unsigned int i = 0x02FF0;
        while (i--) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0) {
                set = 0;
                return 0;
            }
        }
        set = 0;
        return 5;
    }
    return 0;
}
//外部中断按键初始化
void Key_Init_It(void)
{
    // 开启APB2_GPIOB时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // 恢复GPIO默认状态
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_9 | GPIO_PIN_10);
    // 配置GPIO模式
    GPIO_InitTypeDef GPIO_InitType;
    GPIO_InitType.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitType.Pin  = GPIO_PIN_6;
    // 输入模式有效
    GPIO_InitType.Pull = GPIO_PULLUP;
    // 输出模式有效
    // GPIO_InitType.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitType);

    GPIO_InitType.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitType.Pin  = GPIO_PIN_7;
    // 输入模式有效
    GPIO_InitType.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitType);

    //配置中断分组
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    // 设置中断源和优先级-中断源EXTI9_5_IRQn在stm32f103xb.h文件69行开始
    HAL_NVIC_SetPriority(EXTI9_5_IRQn,4,0);
    //使能中断
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

unsigned char set_It = 0;
unsigned char key    = 0;
void Key_Get_It(unsigned x)
{
    set_It = 1;
    key    = x;
}
unsigned char Kry_Scan_It(void)
{
    if (set_It == 1)
    {
        set_It = 0;
        return key;
    }
    else
        return 0;
}
//中断处理回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) 
    {
        case GPIO_PIN_6:
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0 && set_It == 0) {
                unsigned int i = 0x22FF0;
                while (i--) {
                    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 1) {
                        return ;
                    }
                }
                Key_Get_It(6);
            }
            break;
        case GPIO_PIN_7:
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 1 && set_It == 0) {
                unsigned int i = 0x22FF0;
                while (i--) {
                    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0) {
                        return;
                    }
                }
                Key_Get_It(7);
            }
            break;
        default:
            break;
    }
}


//按键事件模式
void Key_Init_Evevt()
{
    // 开启APB2_GPIOB时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // 恢复GPIO默认状态
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    // 配置GPIO模式
    GPIO_InitTypeDef GPIO_InitType;
    GPIO_InitType.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitType.Pin  = GPIO_PIN_6;
    // 输入模式有效
    GPIO_InitType.Pull = GPIO_PULLUP;
    // 输出模式有效
    // GPIO_InitType.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitType);
}
