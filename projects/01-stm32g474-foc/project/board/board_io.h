#ifndef FOC_BOARD_IO_H
#define FOC_BOARD_IO_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* 正式固件串口速率；与上位机一致，历史115200镜像由工具显式选择。 */
#define BOARD_UART_BAUD_RATE 921600U
typedef struct {
    uint32_t rx; /* 已接收字节，含坏字节。 */
    uint32_t tx; /* 已交给硬件的字节。 */
    uint32_t errors; /* UART硬件错误次数。 */
    uint32_t irq_max_cycles; /* 最长一次UART服务周期数，不含异常入退栈。 */
    uint32_t dma_errors; /* DMA传输错误次数。 */
    uint32_t rx_overruns; /* DMA接收环被覆盖次数。 */
    /* UART硬件PE/FE/NE/ORE累计位图，区分链路位错误和接收服务超时。 */
    uint32_t error_flags;
} board_uart_stats_t;
/** @brief 初始化开关GPIO和USART3硬件FIFO，优先级14，RXFNE按字节服务。
 * @return 初始化成功时true；UART或FIFO配置失败不开放中断。 */
bool board_io_init(void);
/** @brief 主循环有界搬运RX DMA、推进切速并发送优先命令。 */
void board_uart_service(void);
/** @brief 查询完整波形块的发送空位。 @return TX空闲且无命令或切速。 */
bool board_uart_wave_ready(void);
/** @brief 取得下一块512字节DMA缓冲，仅主循环且ready时调用。 @return 固定缓冲。 */
uint8_t *board_uart_wave_buffer(void);
/** @brief 发布完整数据块。 @param size 长度1..512。 @return 已提交。 */
bool board_uart_wave_submit(size_t size);
/** @brief 获取当前UART波特率。 @return 请求值。 */
uint32_t board_uart_baud(void);
/** @brief 读取170MHz DWT供有界通信测试定时。 @return 自然回绕的tick。 */
uint32_t board_uart_ticks(void);
/** @brief 排队切速，旧响应物理发完100ms后执行；调用者确认停机和OFF。
 * @param baud 四个允许档之一。 @return 已排队。 */
bool board_uart_request_baud(uint32_t baud);
/** @brief 成功解析PING后确认新速率，避免两秒回退。 */
void board_uart_confirm_baud(void);
/** @brief 查询切速是否等待旧TX或新PING。 @return 握手中。 */
bool board_uart_baud_pending(void);
/** @brief 读取PC7/8/9高电平位。 @return bit0/1/2对应三路物理电平。 */
uint8_t board_switch_raw(void);
/** @brief 有完整响应入队后开启TXE服务。 */
void board_uart_kick(void);
/** @brief 获取近似一致的单项通信计数，无全局锁。 @return 统计快照。 */
board_uart_stats_t board_uart_stats(void);
/** @brief 毫秒时基。 @return HAL毫秒计数。 */
uint32_t board_millis(void);
#endif
