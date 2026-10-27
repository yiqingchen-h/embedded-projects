#include "stm32f1xx_hal.h"
#include "UART.h"

TIM_HandleTypeDef TIM_HandleType1;
TIM_HandleTypeDef TIM_HandleType2;
TIM_HandleTypeDef TIM_HandleType3;

void Tim_Pwm_Init(unsigned short psc, unsigned short arr)
{
    TIM_HandleType1.Instance               = TIM1;
    TIM_HandleType1.Init.Prescaler         = psc;
    TIM_HandleType1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    TIM_HandleType1.Init.Period            = arr;
    TIM_HandleType1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    TIM_HandleType1.Init.RepetitionCounter = 0;
    TIM_HandleType1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_PWM_Init(&TIM_HandleType1);
    __HAL_TIM_CLEAR_FLAG(&TIM_HandleType1, TIM_FLAG_UPDATE);

    TIM_OC_InitTypeDef TIM1_OC_InitType_PWM = {0};
    TIM1_OC_InitType_PWM.OCMode             = TIM_OCMODE_PWM1;
    TIM1_OC_InitType_PWM.Pulse              = 100;
    TIM1_OC_InitType_PWM.OCPolarity         = TIM_OCPOLARITY_HIGH;
    TIM1_OC_InitType_PWM.OCFastMode         = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&TIM_HandleType1, &TIM1_OC_InitType_PWM, TIM_CHANNEL_4);
    __HAL_TIM_MOE_ENABLE(&TIM_HandleType1);
    HAL_TIM_PWM_Start(&TIM_HandleType1, TIM_CHANNEL_4);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitType;
    if (htim->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitType.Pin   = GPIO_PIN_11;
        GPIO_InitType.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitType.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);
    }
}

TIM_Encoder_InitTypeDef TIM_Encoder_InitType_TIM2;
void HAL_TIM_encoder_Init(unsigned int arr)
{
    TIM_HandleType2.Instance               = TIM2;
    TIM_HandleType2.Init.Prescaler         = 0;
    TIM_HandleType2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    TIM_HandleType2.Init.Period            = arr;
    TIM_HandleType2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    TIM_HandleType2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitType_TIM2.EncoderMode  = TIM_ENCODERMODE_TI12;
    TIM_Encoder_InitType_TIM2.IC1Polarity  = TIM_ICPOLARITY_RISING;
    TIM_Encoder_InitType_TIM2.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    TIM_Encoder_InitType_TIM2.IC2Polarity  = TIM_ICPOLARITY_RISING;
    TIM_Encoder_InitType_TIM2.IC1Prescaler = TIM_ICPSC_DIV1;
    TIM_Encoder_InitType_TIM2.IC2Prescaler = TIM_ICPSC_DIV1;
    TIM_Encoder_InitType_TIM2.IC1Filter    = 0x0;
    TIM_Encoder_InitType_TIM2.IC2Filter    = 0x0;

    HAL_TIM_Encoder_Init(&TIM_HandleType2, &TIM_Encoder_InitType_TIM2);
    __HAL_TIM_CLEAR_FLAG(&TIM_HandleType2, TIM_FLAG_UPDATE);
    HAL_TIM_Encoder_Start(&TIM_HandleType2, TIM_CHANNEL_ALL);
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_InitType = {0};
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitType.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitType.Mode  = GPIO_MODE_AF_INPUT;
        GPIO_InitType.Pull  = GPIO_PULLUP;
        GPIO_InitType.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);
    }
}

unsigned short encoder_count = 0;

short Read_Encoder_Value(void)
{
    short count   = 0;
    encoder_count = __HAL_TIM_GET_COUNTER(&TIM_HandleType2);
    __HAL_TIM_SET_COUNTER(&TIM_HandleType2, 0);

    count = encoder_count;
    if (count > 30000) {
        count -= 65535;
    }
    return count;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }
}

void Time3_Init(unsigned int arr, unsigned int psc)
{
    TIM_HandleType3.Instance           = TIM3;
    TIM_HandleType3.Init.Prescaler     = psc;
    TIM_HandleType3.Init.CounterMode   = TIM_COUNTERMODE_UP;
    TIM_HandleType3.Init.Period        = arr;
    TIM_HandleType3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_HandleType3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(&TIM_HandleType3);
    __HAL_TIM_CLEAR_FLAG(&TIM_HandleType3, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&TIM_HandleType3, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&TIM_HandleType3);
}

#include "PID.h"
#include "speed.h"
#include "Time.h"
#include "UART.h"
#include "ADC.h"

short Speed_Count = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        short place_speed = 0;
        short angle_pwm   = 0;
        short speed_pwm   = 0;

        angle_now = ADC_B;
        speed     = Read_Encoder_Value();
        speed /= 2;
        speed_Places += speed;

        speed_Places = speed_Places > 4000 ? 4000 : speed_Places;
        speed_Places = speed_Places < -4000 ? -4000 : speed_Places;

        if (balance_start_flag == 0) {
            pwm = 0;
            Set_Speend(0);
            return;
        }

        if (angle_now > (angle_zero + 600) || angle_now < (angle_zero - 600)) {
            balance_start_flag = 0;
            balance_error_flag = 1;
            pwm                = 0;
            my_speed           = 0;
            Set_Speend(0);
            return;
        }

        place_speed = (my_Places - speed_Places) / 20;
        place_speed = place_speed > 120 ? 120 : place_speed;
        place_speed = place_speed < -120 ? -120 : place_speed;
        my_speed    = place_speed;

        angle_pwm = PID_Place(angle_now, angle_zero);
        speed_pwm = Pid_Speed(speed, my_speed);
        pwm       = angle_pwm + speed_pwm;

        pwm = pwm > 700 ? 700 : pwm;
        pwm = pwm < -700 ? -700 : pwm;

        Set_Speend(pwm);

        Speed_Count++;
        if (Speed_Count >= 10) {
            U2_print("%d,%d,%d,%d,%d\r\n", angle_now, angle_zero, speed_Places, my_speed, pwm);
            Speed_Count = 0;
        }
    }
}
