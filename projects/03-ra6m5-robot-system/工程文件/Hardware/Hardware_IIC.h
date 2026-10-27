#ifndef HARDWARE_IIC_H_
#define HARDWARE_IIC_H_
#include "hal_data.h"
#include <stdio.h>
#include "SysTick.h"

typedef struct I2CDev {
    char *name;
    int (*Init)(struct I2CDev *ptDev);
    int (*Write)(struct I2CDev *ptDev, unsigned char ucAddr, unsigned char const *wBuf, unsigned int dwSize);
    int (*Read)(struct I2CDev *ptDev, unsigned char ucAddr, unsigned char *rBuf, unsigned int dwSize);
} I2CDev, *PI2CDev;

struct I2CDev *EEPROMGetDevice(void); // 获取控制结构体

#endif
