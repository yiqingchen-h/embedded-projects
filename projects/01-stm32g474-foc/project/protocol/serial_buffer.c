#include "serial_buffer.h"
/* Cortex-M 无数据缓存；volatile 存储和索引保证同核中断发布顺序。
 * RX 仅中断写head、主循环写tail；TX 反向，未使用全局关中断。 */
static volatile uint16_t g_serial_rx[SERIAL_RX_CAPACITY];
static volatile uint8_t g_serial_tx[SERIAL_TX_CAPACITY];
/* 各索引均为自然对齐16位，读写在目标MCU上不可分割。 */
static volatile uint16_t g_serial_rx_head, g_serial_rx_tail, g_serial_tx_head, g_serial_tx_tail;
/* 丢弃状态仅由接收中断维护。 */
static bool g_serial_dropping;
/* 统计仅由接收中断递增。 */
static volatile uint32_t g_serial_overflows;
/** @brief 见 serial_buffer.h。 */
void
serial_buffer_init(void)
{
    g_serial_rx_head = g_serial_rx_tail = g_serial_tx_head = g_serial_tx_tail = 0U;
    g_serial_overflows = 0U;
    g_serial_dropping = false;
}
/** @brief 见 serial_buffer.h。 */
void
serial_buffer_error(void) { g_serial_dropping = true; }
/** @brief 见 serial_buffer.h。 */
void
serial_buffer_receive(uint8_t byte, bool error)
{
    uint16_t next = (uint16_t)((g_serial_rx_head + 1U) % SERIAL_RX_CAPACITY);
    if (error) { g_serial_dropping = true; }
    if (next == g_serial_rx_tail)
    {
        ++g_serial_overflows;
        g_serial_dropping = true;
        return;
    }
    if (g_serial_dropping)
    {
        /* 坏字节即使等于LF也不能作为可信同步边界。 */
        if (byte != '\n' || error) { return; }
        g_serial_rx[g_serial_rx_head] = 0x100U | '\n';
        g_serial_dropping = false;
    }
    else { g_serial_rx[g_serial_rx_head] = byte; }
    g_serial_rx_head = next;
}
/** @brief 见 serial_buffer.h。 */
bool
serial_buffer_read(uint16_t *token)
{
    if (g_serial_rx_head == g_serial_rx_tail) { return false; }
    *token = g_serial_rx[g_serial_rx_tail];
    g_serial_rx_tail = (uint16_t)((g_serial_rx_tail + 1U) % SERIAL_RX_CAPACITY);
    return true;
}
/** @brief 见 serial_buffer.h。 */
size_t
serial_buffer_tx_free(void)
{
    return (g_serial_tx_tail + SERIAL_TX_CAPACITY - g_serial_tx_head - 1U) % SERIAL_TX_CAPACITY;
}
/** @brief 见 serial_buffer.h。 */
bool
serial_buffer_write(const char *data, size_t size)
{
    if (size > serial_buffer_tx_free()) { return false; }
    uint16_t head = g_serial_tx_head;
    for (size_t i = 0U; i < size; ++i)
    {
        g_serial_tx[head] = (uint8_t)data[i];
        head = (uint16_t)((head + 1U) % SERIAL_TX_CAPACITY);
    }
    g_serial_tx_head = head;
    return true;
}
/** @brief 见 serial_buffer.h。 */
bool
serial_buffer_transmit(uint8_t *byte)
{
    if (g_serial_tx_head == g_serial_tx_tail) { return false; }
    *byte = g_serial_tx[g_serial_tx_tail];
    g_serial_tx_tail = (uint16_t)((g_serial_tx_tail + 1U) % SERIAL_TX_CAPACITY);
    return true;
}
/** @brief 见 serial_buffer.h。 */
uint32_t
serial_buffer_overflows(void) { return g_serial_overflows; }

/** @brief 见 serial_buffer.h。 */
bool
serial_buffer_rx_pending(void) { return g_serial_rx_head != g_serial_rx_tail; }
