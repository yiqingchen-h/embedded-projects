#ifndef ADC_H_
#define ADC_H_
#include "hal_data.h"
void ADC_init(void);
int ADC_Read(unsigned short *value, unsigned short num);
#endif
