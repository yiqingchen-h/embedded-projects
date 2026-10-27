#ifndef NTC_ADC_H
#define NTC_ADC_H

typedef enum
{
    NTC_RANGE_DIAG = 0,
    NTC_RANGE_MID = 1,
    NTC_RANGE_HIGH = 2
} ntc_range_t;

void NTCADC_Init(void);
void NTCADC_SetRange(ntc_range_t range);
unsigned char NTCADC_Task50ms(void);
unsigned int NTCADC_GetRaw(void);
signed int NTCADC_GetTempC10(void);
unsigned char NTCADC_IsFault(void);
ntc_range_t NTCADC_GetRange(void);

#endif /* NTC_ADC_H */
