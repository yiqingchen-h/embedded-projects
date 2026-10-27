#ifndef FOC_INTERACTION_H
#define FOC_INTERACTION_H
#include <stdbool.h>
#include <stdint.h>
#include "motor_config.h"
/* 固件与文本协议版本独立演进。 */
#if MOTOR_PROFILE_G2208
#define INTERACTION_VERSION "1.10.2-g2208-smooth"
#else
#define INTERACTION_VERSION "1.4.6-serial"
#endif
/** @brief 初始化输入诊断、参数和串口协议状态。 @param uart_online 串口初始化结果。 */
void interaction_init(bool uart_online);
/** @brief 主循环执行一次有界输入和串口服务，不等待完整响应。 */
void interaction_service(void);
/** @brief 查询可立即执行的串口工作。 @return false时可等下一次SysTick/UART中断。 */
bool interaction_pending(void);
#endif
