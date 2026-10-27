#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

#include "board_u23.h"

#define BSP_TARGET_BOARD_NAME       "U23"
#define BSP_FOSC_HZ                 24000000UL
#define BSP_UART1_BAUD              115200UL
#define BSP_UART_DEBUG_ENABLE       0
#define BSP_UART1_RAW_ECHO_ENABLE   0

/*
 * The template assumes STC-ISP programs the internal IRC to 24MHz.
 * Firmware only clears the clock divider and keeps UART1 on default pins.
 */
#define BSP_CLOCK_SOURCE_INTERNAL_IRC  1

#endif /* BSP_CONFIG_H */
