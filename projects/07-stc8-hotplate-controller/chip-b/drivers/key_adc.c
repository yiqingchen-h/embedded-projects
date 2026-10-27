#include "bsp_config.h"
#include "delay.h"
#include "key_adc.h"
#include "stc8g1k08a.h"

#define ADC_POWER_BIT               0x80
#define ADC_START_BIT               0x40
#define ADC_FLAG_BIT                0x20

void KEY_ADC_Init(void)
{
    P5M1 |= 0x20U;
    P5M0 &= (unsigned char)(~0x20U);
    ADC_CONTR = ADC_POWER_BIT;
    DelayMs(2U);
}

unsigned int KEY_ADC_ReadRaw(void)
{
    unsigned char i;
    unsigned long sum;
    unsigned int value;

    sum = 0UL;
    for (i = 0U; i < 4U; i++)
    {
        ADC_CONTR = ADC_POWER_BIT | BOARD_KEY_ADC_CHANNEL;
        ADC_CONTR |= ADC_START_BIT;

        while ((ADC_CONTR & ADC_FLAG_BIT) == 0U)
        {
        }

        ADC_CONTR &= (unsigned char)(~ADC_FLAG_BIT);
        value = ((unsigned int)ADC_RES << 2) | (ADC_RESL & 0x03U);
        sum += value;
    }

    return (unsigned int)(sum >> 2);
}

key_id_t KEY_ADC_Decode(unsigned int adc_value)
{
    if (adc_value <= BOARD_KEY3_MAX)
    {
        return KEY_ID_3;
    }

    if (adc_value <= BOARD_KEY1_MAX)
    {
        return KEY_ID_1;
    }

    if (adc_value <= BOARD_KEY2_MAX)
    {
        return KEY_ID_2;
    }

    return KEY_ID_NONE;
}

const char *KEY_ADC_Name(key_id_t key)
{
    if (key == KEY_ID_1)
    {
        return "K1";
    }

    if (key == KEY_ID_2)
    {
        return "K2";
    }

    if (key == KEY_ID_3)
    {
        return "K3";
    }

    return "NONE";
}
