#ifndef BOARD_U23_H
#define BOARD_U23_H

/*
 * Confirmed U23 resources from Netlist_Schematic1_2026-04-16.asc:
 * U23.1 -> FAN
 * U23.3 -> KEY
 * U23.5 -> RX2 (board net, MCU UART1 RXD on P3.0)
 * U23.6 -> TX2 (board net, MCU UART1 TXD on P3.1)
 * U23.7 -> SCL (P3.2)
 * U23.8 -> SDA (P3.3)
 *
 * KEY ADC channel is inferred from the SOP8 pin mapping:
 * U23.3 is treated as P5.5 / ADC5.
 */

#define BOARD_UART1_RX_NET          "RX2"
#define BOARD_UART1_TX_NET          "TX2"
#define BOARD_UART1_USE_P30_P31     1

#define BOARD_FAN_NET               "FAN"
#define BOARD_KEY_NET               "KEY"
#define BOARD_OLED_SCL_NET          "SCL"
#define BOARD_OLED_SDA_NET          "SDA"

#define BOARD_KEY_ADC_CHANNEL       5U

#define BOARD_KEY3_MAX              150U
#define BOARD_KEY1_MAX              620U
#define BOARD_KEY2_MAX              850U

#endif /* BOARD_U23_H */
