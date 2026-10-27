#ifndef SOFTWARE_IIC_H_
#define SOFTWARE_IIC_H_

#include "hal_data.h"
#include <stdbool.h>
#include <stdint.h>

/* 软件 IIC 固定使用 RASC 中已配置好的 P206/P207。 */
#define SOFTWARE_IIC_SCL_PIN BSP_IO_PORT_02_PIN_06
#define SOFTWARE_IIC_SDA_PIN BSP_IO_PORT_02_PIN_07

/* 初始化总线并尝试释放可能被从机占用的 SDA。 */
fsp_err_t SoftwareIIC_Init(void);
/* 产生 IIC 起始信号。 */
void SoftwareIIC_Start(void);
/* 产生 IIC 停止信号。 */
void SoftwareIIC_Stop(void);
/* 发送 1 字节并读取从机 ACK，返回 true 表示收到 ACK。 */
bool SoftwareIIC_WriteByte(uint8_t data);
/* 读取 1 字节，ack 为 true 时发送 ACK，否则发送 NACK。 */
uint8_t SoftwareIIC_ReadByte(bool ack);
/* 主机发送 ACK。 */
void SoftwareIIC_SendAck(void);
/* 主机发送 NACK。 */
void SoftwareIIC_SendNack(void);
/* EEPROM 顺序写事务：设备地址 + 存储地址 + 数据区。 */
bool SoftwareIIC_Write(uint8_t dev_addr, uint8_t mem_addr, uint8_t const *data, uint32_t size);
/* EEPROM 随机读事务：先写存储地址，再重复起始读取数据。 */
bool SoftwareIIC_Read(uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint32_t size);

#endif
