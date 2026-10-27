#include "mt6835.h"
#include <stddef.h>

/* 总线仅由主循环使用，初始化和读取均不进入控制中断。 */
static mt6835_transfer_t g_mt_transfer;

/** @brief 访问USER_ID易失影子寄存器。
 * @param write true写入、false读取。 @param value 输入或输出字节。 @return 事务成功为true。 */
static bool
user_id(bool write, uint8_t *value)
{
    uint8_t tx[3] = {write ? 0x60U : 0x30U, 0x01U, write ? *value : 0U};
    uint8_t rx[3];
    if (!g_mt_transfer(tx, rx, 3U)) { return false; }
    if (!write) { *value = rx[2]; }
    return true;
}

/** @brief 见mt6835.h。 */
bool
mt6835_probe(mt6835_transfer_t transfer)
{
    uint8_t saved = 0U;
    uint8_t value;
    bool ok = true;
    g_mt_transfer = transfer;
    if (NULL == transfer || !user_id(false, &saved)) { return false; }
    for (uint8_t i = 0U; i < 2U && ok; ++i)
    {
        uint8_t pattern = i == 0U ? 0x5AU : 0xA5U;
        value = pattern;
        ok = user_id(true, &value) && user_id(false, &value) && value == pattern;
    }
    /* 图样失败也尝试恢复，不发送EEPROM编程、置零或自动标定命令。 */
    value = saved;
    bool restored = user_id(true, &value) && user_id(false, &value) && value == saved;
    return ok && restored;
}

/** @brief 见mt6835.h；只允许0x3读命令，两组原文一致后发布。 */
bool
mt6835_read_config(mt6835_config_t *config)
{
    if(NULL==config || NULL==g_mt_transfer) { return false; }
    const uint8_t addresses[2]={0x0DU,0x11U};
    uint8_t values[2]={0U,0U};
    for(unsigned repeat=0U;repeat<2U;++repeat)
    {
        for(unsigned i=0U;i<2U;++i)
        {
            uint8_t tx[3]={0x30U,addresses[i],0U},rx[3];
            if(!g_mt_transfer(tx,rx,3U)) { return false; }
            if(repeat==0U) { values[i]=rx[2]; }
            else if(values[i]!=rx[2]) { return false; }
        }
    }
    *config=(mt6835_config_t){values[0],values[1]};
    return true;
}

/** @brief 见mt6835.h；只允许更小或相同迟滞的四种模式，保留所有厂商位。 */
bool
mt6835_set_hysteresis(uint8_t code)
{
    mt6835_config_t before,after;
    if(code<4U || code>7U || !mt6835_read_config(&before)) { return false; }
    uint8_t value=(before.hysteresis&0xF8U)|code;
    if(value==before.hysteresis) { return true; }
    uint8_t tx[3]={0x60U,0x0DU,value},rx[3];
    if(g_mt_transfer(tx,rx,3U) && mt6835_read_config(&after) &&
        after.hysteresis==value && after.bandwidth==before.bandwidth)
    { return true; }
    /* 与EEPROM编程0xC完全分离；回读失败也只恢复已保存的同一个影子字节。 */
    tx[2]=before.hysteresis;
    (void)g_mt_transfer(tx,rx,3U);
    return false;
}

/** @brief 见mt6835.h。 */
uint8_t
mt6835_crc(const uint8_t data[3])
{
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            crc = (uint8_t)((crc & 0x80U) ? ((uint32_t)crc << 1) ^ 0x07U : (uint32_t)crc << 1);
        }
    }
    return crc;
}

/** @brief 见mt6835.h。 */
mt6835_result_t
mt6835_read(mt6835_sample_t *sample)
{
    const uint8_t tx[6] = {0xA0U, 0x03U, 0U, 0U, 0U, 0U};
    uint8_t rx[6];
    if (NULL == sample || NULL == g_mt_transfer || !g_mt_transfer(tx, rx, 6U))
    {
        return MT6835_IO_ERROR;
    }
    if (mt6835_crc(&rx[2]) != rx[5]) { return MT6835_CRC_ERROR; }
    sample->angle = ((uint32_t)rx[2] << 13) | ((uint32_t)rx[3] << 5) | (rx[4] >> 3);
    sample->status = rx[4] & 7U;
    return MT6835_OK;
}
