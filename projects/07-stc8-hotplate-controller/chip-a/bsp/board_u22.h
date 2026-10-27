#ifndef BOARD_U22_H
#define BOARD_U22_H

/*
 * Confirmed U22 resources from Netlist_Schematic1_2026-04-16.asc:
 * U22.1 -> PWM  (P5.4, heater drive request)
 * U22.3 -> ADC  (P5.5 / ADC5, NTC sample)
 * U22.5 -> RX1  (P3.0)
 * U22.6 -> TX1  (P3.1)
 * U22.7 -> P3.2 (mid-range switch, active low)
 * U22.8 -> P3.3 (high-range switch, active low)
 */

#define BOARD_UART1_RX_NET          "RX1"
#define BOARD_UART1_TX_NET          "TX1"
#define BOARD_UART1_USE_P30_P31     1

#define BOARD_HEATER_NET            "PWM"
#define BOARD_ADC_NET               "ADC"
#define BOARD_ADC_CHANNEL           5U

#define BOARD_NTC_RANGE_DIAG_PULLUP_OHMS  639500UL
#define BOARD_NTC_RANGE_MID_PULLUP_OHMS   19500UL
#define BOARD_NTC_RANGE_HIGH_PULLUP_OHMS  1500UL

#endif /* BOARD_U22_H */
