#include "mt6835.h"
#include <stddef.h>

/* 当前板卡提供的 SPI 接口。 */
static mt6835_transfer_t g_mt_transfer;

/** @brief 访问 USER_ID 的易失影子寄存器。 @param write 是否写。 @param value 写值或返回读值。 @return 总线事务成功为 true。 */
static bool
user_id(bool write, uint8_t *value)
{
    uint8_t tx[3] = {write ? 0x60U : 0x30U, 0x01U, write ? *value : 0U};
    uint8_t rx[3];
    if (!g_mt_transfer(tx, rx, 3U)) { return false; }
    if (!write) { *value = rx[2]; }
    return true;
}

/** @brief 见 mt6835.h。 */
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
    /* 即使图样失败也尝试恢复；没有 EEPROM 编程命令。 */
    value = saved;
    bool restored = user_id(true, &value) && user_id(false, &value) && value == saved;
    return ok && restored;
}

/** @brief 见 mt6835.h。 */
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

/** @brief 见 mt6835.h。 */
bool
mt6835_read(mt6835_sample_t *sample)
{
    const uint8_t tx[6] = {0xA0U, 0x03U, 0U, 0U, 0U, 0U};
    uint8_t rx[6];
    if (NULL == sample || NULL == g_mt_transfer || !g_mt_transfer(tx, rx, 6U) ||
        mt6835_crc(&rx[2]) != rx[5]) { return false; }
    sample->angle = ((uint32_t)rx[2] << 13) | ((uint32_t)rx[3] << 5) | (rx[4] >> 3);
    sample->status = rx[4] & 7U;
    return true;
}
