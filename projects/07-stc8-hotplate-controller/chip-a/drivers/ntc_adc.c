#include "bsp_config.h"
#include "delay.h"
#include "ntc_adc.h"
#include "stc8g1k08a.h"

#define ADC_POWER_BIT               0x80
#define ADC_START_BIT               0x40
#define ADC_FLAG_BIT                0x20

static ntc_range_t g_range = NTC_RANGE_DIAG;
static unsigned int g_raw = 0U;
static signed int g_temp_c10 = 0;
static unsigned char g_fault = 0U;
static unsigned char g_startup_diag = 1U;
static unsigned char g_mid_to_high_hits = 0U;
static unsigned char g_high_to_mid_hits = 0U;

/* 量程表按 20°C 一个点展开，查表后再做线性插值 */
static const unsigned int code g_mid_raw_table[] = {967U, 885U, 748U, 573U, 404U, 270U, 177U, 116U, 78U, 54U};
static const unsigned int code g_high_raw_table[] = {915U, 842U, 748U, 640U, 530U, 427U, 339U, 267U, 210U, 166U, 132U, 106U};

static void NTCADC_ApplyRangePins(ntc_range_t range)
{
    if (range == NTC_RANGE_MID)
    {
        P32 = 0;
        P33 = 1;
    }
    else if (range == NTC_RANGE_HIGH)
    {
        P32 = 1;
        P33 = 0;
    }
    else
    {
        P32 = 1;
        P33 = 1;
    }
}

static unsigned int NTCADC_ReadAverageRaw(void)
{
    unsigned char i;
    unsigned long sum;
    unsigned int value;

    sum = 0UL;
    /* 连续取 8 次均值 */
    for (i = 0U; i < 8U; i++)
    {
        ADC_CONTR = ADC_POWER_BIT | BOARD_ADC_CHANNEL;
        ADC_CONTR |= ADC_START_BIT;
        while ((ADC_CONTR & ADC_FLAG_BIT) == 0U)
        {
        }

        ADC_CONTR &= (unsigned char)(~ADC_FLAG_BIT);
        value = ((unsigned int)ADC_RES << 2) | (ADC_RESL & 0x03U);
        sum += value;
    }

    return (unsigned int)(sum >> 3);
}

static signed int NTCADC_InterpolateTemp(unsigned int raw,
                                         unsigned int raw0,
                                         signed int temp0_c10,
                                         unsigned int raw1,
                                         signed int temp1_c10)
{
    long numerator;
    long denominator;
    long delta;

    if (raw0 == raw1)
    {
        return temp0_c10;
    }

    numerator = (long)((long)raw - (long)raw0) * (long)(temp1_c10 - temp0_c10);
    denominator = (long)raw1 - (long)raw0;
    if (denominator == 0L)
    {
        return temp0_c10;
    }

    if (numerator >= 0L)
    {
        delta = (numerator + (denominator / 2L)) / denominator;
    }
    else
    {
        delta = (numerator - (denominator / 2L)) / denominator;
    }

    return (signed int)((long)temp0_c10 + delta);
}

static signed int NTCADC_TableLookup(unsigned int raw,
                                     const unsigned int code *table,
                                     unsigned char count,
                                     signed int start_temp_c10,
                                     signed int step_temp_c10)
{
    unsigned char i;

    /* raw 越大温度越低，所以这里按从大到小的表查。 */
    if (raw >= table[0])
    {
        return NTCADC_InterpolateTemp(raw,
                                      table[0],
                                      start_temp_c10,
                                      table[1],
                                      (signed int)(start_temp_c10 + step_temp_c10));
    }

    for (i = 0U; i < (unsigned char)(count - 1U); i++)
    {
        if (raw >= table[i + 1U])
        {
            return NTCADC_InterpolateTemp(raw,
                                          table[i],
                                          (signed int)(start_temp_c10 + (signed int)i * step_temp_c10),
                                          table[i + 1U],
                                          (signed int)(start_temp_c10 + (signed int)(i + 1U) * step_temp_c10));
        }
    }

    return NTCADC_InterpolateTemp(raw,
                                  table[count - 2U],
                                  (signed int)(start_temp_c10 + (signed int)(count - 2U) * step_temp_c10),
                                  table[count - 1U],
                                  (signed int)(start_temp_c10 + (signed int)(count - 1U) * step_temp_c10));
}

static unsigned char NTCADC_RawToTemp(unsigned int raw, ntc_range_t range, signed int *temp_c10)
{
    if ((raw <= 0U) || (raw >= 1023U))
    {
        return 0U;
    }

    if (range == NTC_RANGE_HIGH)
    {
        *temp_c10 = NTCADC_TableLookup(raw,
                                       g_high_raw_table,
                                       (unsigned char)(sizeof(g_high_raw_table) / sizeof(g_high_raw_table[0])),
                                       800,
                                       200);
    }
    else
    {
        *temp_c10 = NTCADC_TableLookup(raw,
                                       g_mid_raw_table,
                                       (unsigned char)(sizeof(g_mid_raw_table) / sizeof(g_mid_raw_table[0])),
                                       0,
                                       200);
    }

    return 1U;
}

void NTCADC_Init(void)
{
    P3M1 &= (unsigned char)(~0x0CU);
    P3M0 |= 0x0CU;

    P5M1 |= 0x20U;
    P5M0 &= (unsigned char)(~0x20U);

    ADC_CONTR = ADC_POWER_BIT;
    DelayMs(2U);

    g_range = NTC_RANGE_DIAG;
    g_raw = 0U;
    g_temp_c10 = 0;
    g_fault = 0U;
    g_startup_diag = 1U;
    g_mid_to_high_hits = 0U;
    g_high_to_mid_hits = 0U;
    NTCADC_ApplyRangePins(g_range);
}

void NTCADC_SetRange(ntc_range_t range)
{
    g_range = range;
    g_mid_to_high_hits = 0U;
    g_high_to_mid_hits = 0U;
    NTCADC_ApplyRangePins(range);
}

unsigned char NTCADC_Task50ms(void)
{
    signed int temp_c10;

    g_raw = NTCADC_ReadAverageRaw();
    if (g_startup_diag != 0U)
    {
        /* 上电先用诊断档判断探头是不是明显开路/短路。 */
        if ((g_raw <= BSP_ADC_FAULT_LOW) || (g_raw >= BSP_ADC_FAULT_HIGH))
        {
            g_fault = 1U;
        }
        else
        {
            g_fault = 0U;
            g_startup_diag = 0U;
            NTCADC_SetRange(NTC_RANGE_MID);
        }
        return 1U;
    }

    if ((g_raw <= BSP_ADC_FAULT_LOW) || (g_raw >= BSP_ADC_FAULT_HIGH))
    {
        g_fault = 1U;
        return 1U;
    }

    if (NTCADC_RawToTemp(g_raw, g_range, &temp_c10) == 0U)
    {
        g_fault = 1U;
        return 1U;
    }

    if ((temp_c10 < BSP_TEMP_VALID_MIN_C10) || (temp_c10 > BSP_TEMP_VALID_MAX_C10))
    {
        g_fault = 1U;
        return 1U;
    }

    g_fault = 0U;
    g_temp_c10 = temp_c10;

    if (g_range == NTC_RANGE_MID)
    {
        /* 中温档 ADC 掉得太低，说明阻值已经进入高温区。 */
        if (g_raw <= BSP_RANGE_MID_TO_HIGH_ADC)
        {
            if (g_mid_to_high_hits < BSP_RANGE_SWITCH_COUNT)
            {
                g_mid_to_high_hits++;
            }
            if (g_mid_to_high_hits >= BSP_RANGE_SWITCH_COUNT)
            {
                NTCADC_SetRange(NTC_RANGE_HIGH);
            }
        }
        else
        {
            g_mid_to_high_hits = 0U;
        }
    }
    else if (g_range == NTC_RANGE_HIGH)
    {
        /* 高温档 ADC 回升到阈值以上，再切回中温档。 */
        if (g_raw >= BSP_RANGE_HIGH_TO_MID_ADC)
        {
            if (g_high_to_mid_hits < BSP_RANGE_SWITCH_COUNT)
            {
                g_high_to_mid_hits++;
            }
            if (g_high_to_mid_hits >= BSP_RANGE_SWITCH_COUNT)
            {
                NTCADC_SetRange(NTC_RANGE_MID);
            }
        }
        else
        {
            g_high_to_mid_hits = 0U;
        }
    }

    return 1U;
}

unsigned int NTCADC_GetRaw(void)
{
    return g_raw;
}

signed int NTCADC_GetTempC10(void)
{
    return g_temp_c10;
}

unsigned char NTCADC_IsFault(void)
{
    return g_fault;
}

ntc_range_t NTCADC_GetRange(void)
{
    return g_range;
}
