#ifndef HEATER_CTRL_H
#define HEATER_CTRL_H

void HEATERCTRL_Init(void);
void HEATERCTRL_SetRunEnabled(unsigned char enabled);
void HEATERCTRL_Update100ms(unsigned char power_percent);
unsigned char HEATERCTRL_IsHeating(void);

#endif /* HEATER_CTRL_H */
