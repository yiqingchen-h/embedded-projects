#ifndef KEY_ADC_H
#define KEY_ADC_H

typedef enum
{
    KEY_ID_NONE = 0,
    KEY_ID_1,
    KEY_ID_2,
    KEY_ID_3
} key_id_t;

void KEY_ADC_Init(void);
unsigned int KEY_ADC_ReadRaw(void);
key_id_t KEY_ADC_Decode(unsigned int adc_value);
const char *KEY_ADC_Name(key_id_t key);

#endif /* KEY_ADC_H */
