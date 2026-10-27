#include "24C512.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief  检查 AT24C512 是否在线
 * @retval 0:在线, 1:离线
 */
uint8_t AT24C512_Check(void)
{
    // 发送空包检测 ACK，尝试 2 次，超时 10ms
    if (HAL_I2C_IsDeviceReady(&I2C_HandleType_IIC1, AT24C512_ADDR, 2, 10) == HAL_OK)
    {
        return 0; // 设备在线
    }
    return 1; // 设备离线或地址错误
}

/**
 * @brief  从指定地址读取数据 (支持跨页连续读)
 * @param  ReadAddr:  起始地址 (0 ~ 65535)
 * @param  pBuffer:   数据接收缓冲区
 * @param  NumToRead: 读取长度
 * @retval 0:成功, 1:失败
 */
uint8_t AT24C512_ReadBuffer(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead)
{
    // 调用底层 IIC_Read
    // 注意：AT24C512 必须使用 16位地址模式 (I2C_MEMADD_SIZE_16BIT)
    if (IIC_Read(AT24C512_ADDR, ReadAddr, I2C_MEMADD_SIZE_16BIT, pBuffer, NumToRead) != 0)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief  页写入底层函数 (静态内部调用)
 * @note   单次写入不可超过 128 字节，且不可跨越页边界
 */
static uint8_t AT24C512_WritePage(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite)
{
    // 调用底层 IIC_Write
    if (IIC_Write(AT24C512_ADDR, WriteAddr, I2C_MEMADD_SIZE_16BIT, pBuffer, NumToWrite) != 0)
    {
        return 1;
    }

    /*
     * 关键延时：EEPROM 内部擦写周期 (tWR)
     * AT24C512 手册规定最大 5ms。
     * 使用 vTaskDelay 挂起当前任务，避免死等浪费 CPU。
     */
    vTaskDelay(5);

    return 0;
}

/**
 * @brief  向指定地址写入数据 (智能处理跨页)
 * @param  WriteAddr:  起始地址 (0 ~ 65535)
 * @param  pBuffer:    数据源缓冲区
 * @param  NumToWrite: 写入长度
 * @retval 0:成功, 1:失败
 */
uint8_t AT24C512_WriteBuffer(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite)
{
    uint8_t status = 0;
    uint16_t pageremain; // 当前页剩余空间

    /* 1. 计算当前页还能写多少字节 */
    // 例如：地址 120，页大小 128。剩余 8 字节。
    pageremain = AT24C512_PAGE_SIZE - (WriteAddr % AT24C512_PAGE_SIZE);

    /* 2. 如果要写的数据比剩余空间还小，那就直接写 */
    if (NumToWrite <= pageremain)
    {
        pageremain = NumToWrite;
    }

    while (1)
    {
        /* 3. 执行一次页写入 */
        status = AT24C512_WritePage(WriteAddr, pBuffer, pageremain);
        if (status != 0)
            return 1; // 写入失败

        /* 4. 判断是否写完 */
        if (NumToWrite == pageremain)
        {
            break; // 全部写完，退出
        }
        else
        {
            /* 5. 调整指针和计数器，准备写下一页 */
            pBuffer += pageremain;    // 数据指针后移
            WriteAddr += pageremain;  // 地址后移
            NumToWrite -= pageremain; // 剩余长度减小

            /* 6. 计算下一次写入长度 */
            if (NumToWrite > AT24C512_PAGE_SIZE)
            {
                pageremain = AT24C512_PAGE_SIZE; // 下一次写满一整页
            }
            else
            {
                pageremain = NumToWrite; // 下一次写剩下的所有
            }
        }
    }

    return 0;
}
