#include "Hardware_IIC.h"
#include "software_iic.h"

/* 当前板上 EEPROM 仍使用 7 位从机地址 0x50。 */
#define EEPROM_DEV_ADDR 0x50U
/* 当前存储芯片页大小保持 8 字节，与原 SCI-I2C 实现一致。 */
#define EE_PAGE_SIZE 8U

unsigned char Hardware_IIC4_init(void)
{
    fsp_err_t err = SoftwareIIC_Init();
    if (FSP_SUCCESS == err)
    {
        printf("Success to open device: software iic!\r\n");
        return 1;
    }

    printf("Failed to open device: software iic!\r\n");
    return 0;
}

static int EEPROMDrvInit(struct I2CDev *ptDev)
{
    /* 保持原有设备抽象不变，只把底层初始化切到软件 IIC。 */
    if (NULL == ptDev->name)
    {
        return -1;
    }

    if (FSP_SUCCESS != SoftwareIIC_Init())
    {
        printf("Failed to open device: software iic!\r\n");
        return -1;
    }

    printf("Success to open device: software iic!\r\n");
    return 0;
}

static int EEPROMDrvWriteByte(struct I2CDev *ptDev, unsigned char ucAddr, unsigned char const ucData)
{
    /* 单字节写直接复用软件 IIC 的 EEPROM 写事务。 */
    if (NULL == ptDev->name)
    {
        return -1;
    }

    return SoftwareIIC_Write(EEPROM_DEV_ADDR, ucAddr, &ucData, 1U) ? 0 : -1;
}

static int EEPROMDrvWritePage(struct I2CDev *ptDev, unsigned char ucAddr, unsigned char const *wbuf, unsigned int dwSize)
{
    /* 页写保持原有约束，避免一次写超过页边界。 */
    if ((NULL == ptDev->name) || (NULL == wbuf) || (0U == dwSize) || (dwSize > EE_PAGE_SIZE))
    {
        return -1;
    }

    if (!SoftwareIIC_Write(EEPROM_DEV_ADDR, ucAddr, wbuf, dwSize))
    {
        return -1;
    }

    /* EEPROM 页写后需要等待内部擦写完成。 */
    R_BSP_SoftwareDelay((dwSize + 1U) * 5U, BSP_DELAY_UNITS_MILLISECONDS);
    return 0;
}

static int EEPROMDrvWriteBuff(struct I2CDev *ptDev, unsigned char ucAddr, unsigned char const *wbuf, unsigned int dwSize)
{
    /* 多字节写保留原先的分页拆分逻辑，保证行为与旧版本一致。 */
    if ((NULL == ptDev->name) || (NULL == wbuf) || (0U == dwSize))
    {
        return -1;
    }

    if (1U == dwSize)
    {
        return EEPROMDrvWriteByte(ptDev, ucAddr, *wbuf);
    }

    if ((ucAddr + dwSize) >= 256U)
    {
        return -1;
    }

    unsigned char nAddr = ucAddr;
    unsigned char ucSize;

    if ((ucAddr == 0U || (ucAddr / EE_PAGE_SIZE) != 0U) && (dwSize <= EE_PAGE_SIZE))
    {
        ucSize = (unsigned char) dwSize;
    }
    else
    {
        ucSize = EE_PAGE_SIZE - (ucAddr % EE_PAGE_SIZE);
    }

    if (0 != EEPROMDrvWritePage(ptDev, nAddr, wbuf, ucSize))
    {
        return -1;
    }

    nAddr += ucSize;
    wbuf += ucSize;
    dwSize -= ucSize;

    while (0U != dwSize)
    {
        if (dwSize <= EE_PAGE_SIZE)
        {
            ucSize = (unsigned char) dwSize;
        }
        else
        {
            ucSize = EE_PAGE_SIZE;
        }

        if (0 != EEPROMDrvWritePage(ptDev, nAddr, wbuf, ucSize))
        {
            return -1;
        }

        nAddr += ucSize;
        wbuf += ucSize;
        dwSize -= ucSize;
    }

    return 0;
}

static int EEPROMDrvRead(struct I2CDev *ptDev, unsigned char ucAddr, unsigned char *rbuf, unsigned int dwSize)
{
    /* 随机读通过“写地址 + 重启 + 读数据”完成。 */
    if ((NULL == ptDev->name) || (NULL == rbuf) || (0U == dwSize))
    {
        return -1;
    }

    return SoftwareIIC_Read(EEPROM_DEV_ADDR, ucAddr, rbuf, dwSize) ? 0 : -1;
}

static I2CDev gEepromDev = {
    .name = "EEPROM",
    .Init = EEPROMDrvInit,
    .Write = EEPROMDrvWriteBuff,
    .Read = EEPROMDrvRead
};

struct I2CDev *EEPROMGetDevice(void)
{
    return &gEepromDev;
}

void sci_i2c4_master_callback(i2c_master_callback_args_t *p_args)
{
    /* 保留旧回调符号，避免 RASC 仍生成该入口时出现链接问题。 */
    FSP_PARAMETER_NOT_USED(p_args);
}


