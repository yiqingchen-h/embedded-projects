#include "software_iic.h"

#define SOFTWARE_IIC_DELAY_US (5U)

static bool g_software_iic_ready = false;

/* 软件 IIC 的节拍延时，当前约对应 100 kHz 附近的时序。 */
static void SoftwareIIC_Delay(void)
{
    R_BSP_SoftwareDelay(SOFTWARE_IIC_DELAY_US, BSP_DELAY_UNITS_MICROSECONDS);
}

/* 开漏输出下，写高等价于释放总线，写低等价于主动拉低。 */
static void SoftwareIIC_SCLWrite(bsp_io_level_t level)
{
    g_ioport.p_api->pinWrite(g_ioport.p_ctrl, SOFTWARE_IIC_SCL_PIN, level);
}

/* 开漏输出下，写高等价于释放总线，写低等价于主动拉低。 */
static void SoftwareIIC_SDAWrite(bsp_io_level_t level)
{
    g_ioport.p_api->pinWrite(g_ioport.p_ctrl, SOFTWARE_IIC_SDA_PIN, level);
}

/* 读取 SDA 实际电平，用于判断 ACK 或读取数据位。 */
static bsp_io_level_t SoftwareIIC_SDARead(void)
{
    bsp_io_level_t level = BSP_IO_LEVEL_LOW;
    g_ioport.p_api->pinRead(g_ioport.p_ctrl, SOFTWARE_IIC_SDA_PIN, &level);
    return level;
}

/* 将总线释放为空闲态：SCL/SDA 都为高。 */
static void SoftwareIIC_BusIdle(void)
{
    SoftwareIIC_SDAWrite(BSP_IO_LEVEL_HIGH);
    SoftwareIIC_SCLWrite(BSP_IO_LEVEL_HIGH);
    SoftwareIIC_Delay();
}

/* 产生一个标准时钟脉冲。 */
static void SoftwareIIC_ClockPulse(void)
{
    SoftwareIIC_SCLWrite(BSP_IO_LEVEL_HIGH);
    SoftwareIIC_Delay();
    SoftwareIIC_SCLWrite(BSP_IO_LEVEL_LOW);
    SoftwareIIC_Delay();
}

fsp_err_t SoftwareIIC_Init(void)
{
    /* 本工程里 g_ioport 已经在更早的硬件初始化里打开，这里不再重复 open，
       避免重新套用整板 pin 配置引入额外副作用。 */
    SoftwareIIC_BusIdle();
    g_software_iic_ready = true;
    return FSP_SUCCESS;
}

void SoftwareIIC_Start(void)
{
    /* 起始条件：SCL 为高时，SDA 从高跳变到低。 */
    SoftwareIIC_BusIdle();
    SoftwareIIC_SDAWrite(BSP_IO_LEVEL_LOW);
    SoftwareIIC_Delay();
    SoftwareIIC_SCLWrite(BSP_IO_LEVEL_LOW);
    SoftwareIIC_Delay();
}

void SoftwareIIC_Stop(void)
{
    /* 停止条件：SCL 为高时，SDA 从低跳变到高。 */
    SoftwareIIC_SDAWrite(BSP_IO_LEVEL_LOW);
    SoftwareIIC_Delay();
    SoftwareIIC_SCLWrite(BSP_IO_LEVEL_HIGH);
    SoftwareIIC_Delay();
    SoftwareIIC_SDAWrite(BSP_IO_LEVEL_HIGH);
    SoftwareIIC_Delay();
}

bool SoftwareIIC_WriteByte(uint8_t data)
{
    /* 按 MSB first 发送 8 位数据。 */
    for (uint8_t i = 0; i < 8U; i++)
    {
        SoftwareIIC_SDAWrite((data & 0x80U) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
        SoftwareIIC_Delay();
        SoftwareIIC_SCLWrite(BSP_IO_LEVEL_HIGH);
        SoftwareIIC_Delay();
        SoftwareIIC_SCLWrite(BSP_IO_LEVEL_LOW);
        SoftwareIIC_Delay();
        data <<= 1;
    }

    /* 第 9 个时钟释放 SDA，由从机驱动 ACK。 */
    SoftwareIIC_SDAWrite(BSP_IO_LEVEL_HIGH);
    SoftwareIIC_Delay();
    SoftwareIIC_SCLWrite(BSP_IO_LEVEL_HIGH);
    SoftwareIIC_Delay();

    bool ack = (BSP_IO_LEVEL_LOW == SoftwareIIC_SDARead());

    SoftwareIIC_SCLWrite(BSP_IO_LEVEL_LOW);
    SoftwareIIC_Delay();
    return ack;
}

uint8_t SoftwareIIC_ReadByte(bool ack)
{
    uint8_t data = 0;

    /* 读取阶段必须先释放 SDA，由从机输出数据位。 */
    SoftwareIIC_SDAWrite(BSP_IO_LEVEL_HIGH);
    for (uint8_t i = 0; i < 8U; i++)
    {
        data <<= 1;
        SoftwareIIC_SCLWrite(BSP_IO_LEVEL_HIGH);
        SoftwareIIC_Delay();
        if (BSP_IO_LEVEL_HIGH == SoftwareIIC_SDARead())
        {
            data |= 0x01U;
        }
        SoftwareIIC_SCLWrite(BSP_IO_LEVEL_LOW);
        SoftwareIIC_Delay();
    }

    /* 第 9 个时钟由主机回 ACK/NACK。 */
    if (ack)
    {
        SoftwareIIC_SendAck();
    }
    else
    {
        SoftwareIIC_SendNack();
    }

    return data;
}

void SoftwareIIC_SendAck(void)
{
    /* ACK 为主机在第 9 个时钟把 SDA 拉低。 */
    SoftwareIIC_SDAWrite(BSP_IO_LEVEL_LOW);
    SoftwareIIC_Delay();
    SoftwareIIC_ClockPulse();
    SoftwareIIC_SDAWrite(BSP_IO_LEVEL_HIGH);
}

void SoftwareIIC_SendNack(void)
{
    /* NACK 为主机在第 9 个时钟保持 SDA 为高。 */
    SoftwareIIC_SDAWrite(BSP_IO_LEVEL_HIGH);
    SoftwareIIC_Delay();
    SoftwareIIC_ClockPulse();
}

bool SoftwareIIC_Write(uint8_t dev_addr, uint8_t mem_addr, uint8_t const *data, uint32_t size)
{
    uint8_t write_addr = (uint8_t) (dev_addr << 1U);

    if ((!g_software_iic_ready) || ((size > 0U) && (NULL == data)))
    {
        return false;
    }

    /* EEPROM 写时序：Start + 设备写地址 + 存储地址 + N 字节数据 + Stop。 */
    SoftwareIIC_Start();
    if (!SoftwareIIC_WriteByte(write_addr))
    {
        SoftwareIIC_Stop();
        return false;
    }

    if (!SoftwareIIC_WriteByte(mem_addr))
    {
        SoftwareIIC_Stop();
        return false;
    }

    for (uint32_t i = 0; i < size; i++)
    {
        if (!SoftwareIIC_WriteByte(data[i]))
        {
            SoftwareIIC_Stop();
            return false;
        }
    }

    SoftwareIIC_Stop();
    return true;
}

bool SoftwareIIC_Read(uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint32_t size)
{
    uint8_t write_addr = (uint8_t) (dev_addr << 1U);
    uint8_t read_addr = (uint8_t) (write_addr | 0x01U);

    if ((!g_software_iic_ready) || (NULL == data) || (0U == size))
    {
        return false;
    }

    /* EEPROM 随机读时序：Start + 写器件地址 + 写存储地址 + 重启 + 读器件地址 + 连续读数据。 */
    SoftwareIIC_Start();
    if (!SoftwareIIC_WriteByte(write_addr))
    {
        SoftwareIIC_Stop();
        return false;
    }

    if (!SoftwareIIC_WriteByte(mem_addr))
    {
        SoftwareIIC_Stop();
        return false;
    }

    SoftwareIIC_Start();
    if (!SoftwareIIC_WriteByte(read_addr))
    {
        SoftwareIIC_Stop();
        return false;
    }

    for (uint32_t i = 0; i < size; i++)
    {
        data[i] = SoftwareIIC_ReadByte(i + 1U < size);
    }

    SoftwareIIC_Stop();
    return true;
}
