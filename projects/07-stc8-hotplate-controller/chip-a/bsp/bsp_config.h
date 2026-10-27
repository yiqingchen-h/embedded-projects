#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

#include "board_u22.h"

#define BSP_TARGET_BOARD_NAME       "U22"
#define BSP_FOSC_HZ                 24000000UL
#define BSP_UART1_BAUD              115200UL

#define BSP_SET_TEMP_DEFAULT_C      230U
#define BSP_SET_TEMP_MIN_C          30U
#define BSP_SET_TEMP_MAX_C          250U
#define BSP_OVERTEMP_C              270U
#define BSP_FAN_ON_TEMP_C           80U
#define BSP_FAN_OFF_TEMP_C          60U

#define BSP_ADC_SAMPLE_PERIOD_MS    50U
#define BSP_PID_PERIOD_MS           100U
#define BSP_STATUS_PERIOD_MS        200U
#define BSP_COMM_TIMEOUT_MS         2000U

#define BSP_CONTROL_WINDOW_SLOTS    10U
#define BSP_CONTROL_SLOT_MS         100U

#define BSP_RANGE_SWITCH_COUNT      3U
#define BSP_RANGE_MID_TO_HIGH_ADC   160U
#define BSP_RANGE_HIGH_TO_MID_ADC   780U
#define BSP_ADC_FAULT_LOW           2U
#define BSP_ADC_FAULT_HIGH          1020U

#define BSP_TEMP_VALID_MIN_C10      (-200)
#define BSP_TEMP_VALID_MAX_C10      3000

#define BSP_PID_KP_X100             200
#define BSP_PID_KI_X100             4
#define BSP_PID_KD_X100             0

/*
 * The template assumes STC-ISP programs the internal IRC to 24MHz.
 * Firmware only clears the clock divider and keeps UART1 on default pins.
 */
#define BSP_CLOCK_SOURCE_INTERNAL_IRC  1

#endif /* BSP_CONFIG_H */
