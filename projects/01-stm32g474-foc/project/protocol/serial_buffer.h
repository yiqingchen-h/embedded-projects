#ifndef FOC_SERIAL_BUFFER_H
#define FOC_SERIAL_BUFFER_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* 固定容量，单生产者/单消费者；保留一个空槽区分满与空。 */
#define SERIAL_RX_CAPACITY 512U
#define SERIAL_TX_CAPACITY 1024U
/** @brief 初始化收发缓冲，启用中断前调用。 */
void serial_buffer_init(void);
/** @brief 中断输入字节；错误后直到换行才输出损坏行标记。
 * @param byte 接收字节。 @param error 本字节或此前发生硬件错误。 */
void serial_buffer_receive(uint8_t byte, bool error);
/** @brief 标记没有可用字节的硬件接收错误。 */
void serial_buffer_error(void);
/** @brief 主循环读取一个接收项。 @param token 接收项输出。 @return 有数据时true。 */
bool serial_buffer_read(uint16_t *token);
/** @brief 查询接收缓冲。 @return 存在未消费项时true。 */
bool serial_buffer_rx_pending(void);
/** @brief 查询发送空位。 @return 可排队字节数。 */
size_t serial_buffer_tx_free(void);
/** @brief 主循环原子发布完整响应，不等待。 @param data 内容。 @param size 字节数。
 * @return 容量足够并入队时true，否则不写入。 */
bool serial_buffer_write(const char *data, size_t size);
/** @brief 中断取下一个发送字节。 @param byte 输出字节。 @return 有数据时true。 */
bool serial_buffer_transmit(uint8_t *byte);
/** @brief 读取接收溢出统计。 @return 丢弃次数。 */
uint32_t serial_buffer_overflows(void);
#endif
