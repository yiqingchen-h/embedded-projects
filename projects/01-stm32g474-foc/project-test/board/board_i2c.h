#ifndef FOC_BOARD_I2C_H
#define FOC_BOARD_I2C_H
#include <stdbool.h>
#include <stdint.h>

/** @brief 初始化 H1 的 PB6/PB7 开漏软件 I2C，保持外部上拉。 */
void board_i2c_init(void);
/**
 * @brief 发送一次 I2C 写事务，仅供裸机主循环调用。
 * @param address 7 位地址。
 * @param control OLED 控制字节。
 * @param data 有效数据指针，长度为零时可为空。
 * @param size 数据长度。
 * @return 全部字节 ACK 且 STOP 后总线空闲返回 true；无应答、延时失效或总线异常返回 false。
 */
bool board_i2c_write(uint8_t address, uint8_t control, const uint8_t *data, uint16_t size);
#endif
