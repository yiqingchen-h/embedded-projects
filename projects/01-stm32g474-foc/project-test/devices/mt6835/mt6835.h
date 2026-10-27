#ifndef FOC_MT6835_H
#define FOC_MT6835_H
#include <stdbool.h>
#include <stdint.h>
/* 同一片选期间发送并接收全部字节，模式 3，MSB 先发。 */
typedef bool (*mt6835_transfer_t)(const uint8_t *tx, uint8_t *rx, uint16_t count);
typedef struct
{
    /* 21 位机械角，0～2097151。 */
    uint32_t angle;
    /* bit0 超速、bit1 弱磁场、bit2 欠压。 */
    uint8_t status;
} mt6835_sample_t;
/** @brief USER_ID 易失寄存器双图样回读并恢复，确认双向通信；不写 EEPROM、不标定。 @param transfer SPI 接口。 @return 所有步骤包括恢复成功为 true。 */
bool mt6835_probe(mt6835_transfer_t transfer);
/** @brief 突发读取角度、状态及 CRC。 @param sample 成功时填充的结果。 @return 通信及 CRC 正确为 true；磁场状态另行判断。 */
bool mt6835_read(mt6835_sample_t *sample);
/** @brief 计算厂商角度帧的 CRC-8，初值 0、多项式 0x07、不反射。 @param data 三个角度/状态字节。 @return CRC 值。 */
uint8_t mt6835_crc(const uint8_t data[3]);
#endif
