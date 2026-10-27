#ifndef __IIC_H
#define __IIC_H

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h" // 引入信号量/互斥锁头文件
/** 
 * SCL SDA 引脚号
*/
#define GPIOx_I2C GPIOB
#define PIN_SCL GPIO_PIN_6
#define PIN_SDA GPIO_PIN_7
extern I2C_HandleTypeDef I2C_HandleType_IIC1;

void IIC1_Init(void);
/**
 * @brief  IIC 写数据函数 (封装)
 * @param  DevAddress: 设备地址 (7位地址需左移1位，例如 0xA0)
 * @param  MemAddress: 寄存器地址/内存地址
 * @param  MemAddSize: 寄存器地址长度 (I2C_MEMADD_SIZE_8BIT 或 I2C_MEMADD_SIZE_16BIT)
 * @param  pData:      要写入的数据缓冲区指针
 * @param  Size:       要写入的数据长度
 * @retval 0:成功, 其他:失败
 */
uint8_t IIC_Write(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

/**
 * @brief  IIC 读数据函数 (封装)
 * @param  DevAddress: 设备地址 (7位地址需左移1位，例如 0xA0)
 * @param  MemAddress: 寄存器地址/内存地址
 * @param  MemAddSize: 寄存器地址长度 (I2C_MEMADD_SIZE_8BIT 或 I2C_MEMADD_SIZE_16BIT)
 * @param  pData:      接收数据的缓冲区指针
 * @param  Size:       要读取的数据长度
 * @retval 0:成功, 其他:失败
 */
uint8_t IIC_Read(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
/* 直接接收函数 (用于 AHT20 这种不需要寄存器地址的读取) */
uint8_t IIC_Receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size);


#endif
