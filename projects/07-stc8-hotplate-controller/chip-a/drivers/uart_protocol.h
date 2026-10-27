#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

typedef struct
{
    unsigned int set_temp;
    unsigned char run_on;
} uart_command_t;

void UART_PROTOCOL_Init(void);
void UART_PROTOCOL_ProcessByte(unsigned char rx);
bit UART_PROTOCOL_CommandReady(void);
void UART_PROTOCOL_GetCommand(uart_command_t *cmd);
unsigned char UART_PROTOCOL_TakeAck(void);
void UART_PROTOCOL_SendOk(void);
void UART_PROTOCOL_SendStatus(unsigned int set_temp,
                              unsigned int cur_temp,
                              unsigned char fan_on,
                              unsigned char heat_on,
                              unsigned char run_on);

#endif /* UART_PROTOCOL_H */
