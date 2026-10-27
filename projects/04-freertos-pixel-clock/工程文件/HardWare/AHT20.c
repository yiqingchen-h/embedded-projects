#include "AHT20.h"
#include "FreeRTOS.h"
#include "task.h"
uint8_t AHT20_Init(void)
{
    uint8_t read_buf = 0;
    HAL_Delay(80);
    // 2. 读取状态字节
    if (IIC_Receive(AHT20_ADDRESS, &read_buf, 1) != 0)
    {
        return 1;
    }
    // 3. 检查状态字 Bit[3] (NOR 模式校准标志位)
    // 如果 Bit[3] != 1，需要发送初始化命令
    if ((read_buf & 0x08) == 0)
    {
        uint8_t init_cmd[] = {0x08, 0x00};
        // 发送命令 0xBE (初始化)，参数 0x08, 0x00
        // 这里可以使用我们之前封装的 IIC_Write (Mem_Write模式，0xBE视作寄存器地址)
        if (IIC_Write(AHT20_ADDRESS, 0xBE, I2C_MEMADD_SIZE_8BIT, init_cmd, 2) != 0)
        {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief  触发测量并读取温湿度数据
 * @param  pResult: 存储结果的结构体指针
 * @retval 0:读取成功, 1:读取失败
 */
uint8_t AHT20_Read_Measure(AHT20_Data_t *pResult)
{
    uint8_t cmd_params[] = {0x33, 0x00};
    uint8_t data_buf[6];
    uint32_t raw_humid = 0;
    uint32_t raw_temp = 0;

    /* 1. 发送触发测量命令: 0xAC, 参数: 0x33, 0x00 */
    // 使用 IIC_Write 封装 (将 0xAC 当作寄存器地址发送)
    if (IIC_Write(AHT20_ADDRESS, 0xAC, I2C_MEMADD_SIZE_8BIT, cmd_params, 2) != 0)
    {
        pResult->Alive = 0;
        return 1;
    }

    /* 2. 等待测量完成 (手册要求 > 75ms) */
    vTaskDelay(100);

    /* 3. 读取 6 字节数据 (状态 + 湿度2.5B + 温度2.5B) */
    // 注意：AHT20 读取数据是直接读，不需要先写寄存器地址，所以不能用 IIC_Read (Mem_Read)
    // 必须直接使用 HAL_I2C_Master_Receive
    if (IIC_Receive(AHT20_ADDRESS, data_buf, 6) != 0)
    {
        pResult->Alive = 0;
        return 1;
    }

    /* 4. 检查状态字节 (Byte 0) 的 Bit[7] (Busy 指示) */
    if ((data_buf[0] & 0x80) != 0)
    {
        // 设备忙，可能数据未准备好
        pResult->Alive = 1;
        return 1;
    }

    /* 5. 数据解析 (根据手册公式) */
    // 湿度: Byte 1, Byte 2, Byte 3的前4位
    raw_humid = ((uint32_t)data_buf[1] << 12) |
                ((uint32_t)data_buf[2] << 4) |
                ((uint32_t)data_buf[3] >> 4);
    // 温度: Byte 3的后4位, Byte 4, Byte 5
    raw_temp = ((uint32_t)(data_buf[3] & 0x0F) << 16) |
               ((uint32_t)data_buf[4] << 8) |
               ((uint32_t)data_buf[5]);
    /* 6. 转换为物理量 */
    // 湿度公式: (RAW / 2^20) * 100%
    pResult->Humidity = (float)raw_humid / 1048576.0f * 100.0f;

    // 温度公式: ((RAW / 2^20) * 200) - 50
    pResult->Temperature = ((float)raw_temp / 1048576.0f * 200.0f) - 50.0f;

    pResult->Alive = 1;

    return 0;
}
