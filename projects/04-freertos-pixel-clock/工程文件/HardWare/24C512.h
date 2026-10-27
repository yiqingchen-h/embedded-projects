#ifndef __24C512_H
#define __24C512_H
#include "main.h"
/*
 * AT24C512 设备地址计算:
 * Binary: 1 0 1 0 A2 A1 A0 R/W
 * Config: A2=0, A1=0, A0=1
 * Result: 1 0 1 0 0  0  1  0  = 0xA2
 */
#define AT24C512_ADDR 0xA2

/*
 * AT24C512 参数定义
 * 容量: 512Kbit = 65536 Byte
 * 页大小: 128 Byte (写操作不能跨页)
 */
#define AT24C512_PAGE_SIZE 128
#define AT24C512_TOTAL_SIZE 65536

/* 函数声明 */
uint8_t AT24C512_Check(void);
uint8_t AT24C512_ReadBuffer(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead);
uint8_t AT24C512_WriteBuffer(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite);
#endif
