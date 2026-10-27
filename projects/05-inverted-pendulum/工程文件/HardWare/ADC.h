#ifndef __ADC_H
#define __ADC_H
extern ADC_HandleTypeDef ADC_HandleType_ADC7;
extern ADC_ChannelConfTypeDef ADC_ChannelConfType_ADC7;
void ADC7_Init(void);
extern unsigned short ADC7_DmaBuff[10];
extern short ADC_B;
#endif