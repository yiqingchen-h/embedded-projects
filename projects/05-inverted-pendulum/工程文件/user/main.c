#include "stm32f1xx_hal.h"
#include "Rcc.h"
#include "Time.h"
#include "OLED.h"
#include "UART.h"
#include "Speed.h"
#include "ADC.h"
#include "Key.h"

// kp取值数量级大致为 输出范围/输入范围
/*单环控制使用*/
/*float Speed_Ki = 0.6, Speed_Kp = 15, Speed_Kd = -0.05;
float Places_Ki = 0.03, Places_Kp = 1, Places_Kd = -0.01;*/
/*双环控制使用*/
// float Speed_Ki = 1, Speed_Kp = 20, Speed_Kd = 0;
// float Places_Ki = 0, Places_Kp = 0.007, Places_Kd = -0.06;
/*倒立摆调试使用*/
float Speed_Ki = 0.15, Speed_Kp = 10, Speed_Kd = 0.1;
float Places_Ki = 0, Places_Kp = -30, Places_Kd = -12;

short speed = 0, pwm = 0, my_speed = 0, speed_Places = 0, my_Places = 0;
short angle_now = 0, angle_zero = 2050;
unsigned char balance_start_flag = 0, balance_error_flag = 0;

unsigned char Key = 0;

int main(void)
{
    RccClock_HSE_Init();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    OLED_Init();
    Tim_Pwm_Init(71, 999);
    Speed_Init();
    HAL_TIM_encoder_Init(65535 - 1);
    UART2_Init(9600);
    ADC7_Init();
    Time3_Init(5000 - 1, 72 - 1);
    HAL_TIM_Base_Stop_IT(&TIM_HandleType3);
    Key_Init();

    U2_print("balance test\r\n");

    while (1) {
        Key = Key_Scan();
        switch (Key) {
            case 1:
                angle_zero = ADC_B;
                angle_now = ADC_B;
                speed = 0;
                pwm = 0;
                my_speed = 0;
                speed_Places = 0;
                my_Places = 0;
                balance_error_flag = 0;
                balance_start_flag = 1;
                HAL_TIM_Base_Start_IT(&TIM_HandleType3);
                break;

            case 5:
                balance_start_flag = 0;
                balance_error_flag = 0;
                my_speed = 0;
                pwm = 0;
                Set_Speend(0);
                HAL_TIM_Base_Stop_IT(&TIM_HandleType3);
                break;

            case 9:
                my_Places += 100;
                my_Places = my_Places > 1500 ? 1500 : my_Places;
                break;

            case 13:
                my_Places -= 100;
                my_Places = my_Places < -1500 ? -1500 : my_Places;
                break;

            default:
                break;
        }

        OLED_ShowString(0, 0, "Speed:", OLED_6X8);
        OLED_ShouMetrication(37, 0, speed, 3, OLED_6X8);

        OLED_ShowString(0, 1, "Place:", OLED_6X8);
        OLED_ShouMetrication(37, 1, speed_Places, 5, OLED_6X8);

        OLED_ShowString(0, 2, "Target:", OLED_6X8);
        OLED_ShouMetrication(49, 2, my_Places, 5, OLED_6X8);

        OLED_ShowString(0, 3, "ADC:", OLED_6X8);
        OLED_ShowNum(25, 3, ADC_B, 4, OLED_6X8);
        OLED_ShowString(61, 3, "Zero:", OLED_6X8);
        OLED_ShowNum(91, 3, angle_zero, 4, OLED_6X8);

        OLED_ShowString(0, 4, "TSpeed:", OLED_6X8);
        OLED_ShouMetrication(49, 4, my_speed, 4, OLED_6X8);
        OLED_ShowString(80, 4, "PWM:", OLED_6X8);
        OLED_ShouMetrication(104, 4, pwm, 3, OLED_6X8);

        OLED_ShowString(0, 5, "Run:", OLED_6X8);
        OLED_ShowNum(25, 5, balance_start_flag, 1, OLED_6X8);
        OLED_ShowString(49, 5, "Err:", OLED_6X8);
        OLED_ShowNum(73, 5, balance_error_flag, 1, OLED_6X8);
    }
}
