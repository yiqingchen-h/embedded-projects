#ifndef SYSTICK_H_
#define SYSTICK_H_
#include "hal_data.h"
#include <stdint.h>
#define HAL_MAX_DELAY 0xFFFFFFU
extern volatile uint32_t dwTick;

fsp_err_t SystickInit(void);
void HAL_Delay(uint32_t dwTime);
uint32_t HAL_GetTick(void);
#endif
