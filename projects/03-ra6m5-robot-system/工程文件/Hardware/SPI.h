#ifndef SPI_H_
#define SPI_H_
#include "hal_data.h"
#include <stdio.h>
#include "SysTick.h"
extern volatile bool g_lcd_spi_tx_complete;
void SPI_LCD_Init(void);
#endif
