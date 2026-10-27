#include "drv8353f.h"
#include <stddef.h>

/* 板卡 SPI 适配接口；初始化之前不允许访问。 */
static drv8353f_transfer_t g_drv_transfer;
/* 地址 2～6：6PWM+COAST；50/100mA 栅极电流；4us 驱动；400ns 死区、锁存 OCP、2us 消隐、0.06V VDS；CSA 20倍、双向、0.25V 保护。 */
static const uint16_t g_drv_config[5] = {DRV8353F_CONTROL_SAFE, 0x0300U, 0x0300U, 0x0310U, 0x0280U};
/* 当前确认的CSA配置；校准位和增益参与后续完整回读。 */
static uint16_t g_drv_csa = 0x0280U;

/** @brief 写配置并回读，拒绝假应答。 @param address 地址。 @param value 11 位配置。 @return 完全相等为 true。 */
static bool
write_verify(uint8_t address, uint16_t value)
{
    uint16_t readback;
    return NULL != g_drv_transfer &&
           g_drv_transfer((uint16_t)(((uint32_t)address << 11) | value), &readback) &&
           drv8353f_read(address, &readback) && readback == value;
}

/** @brief 见 drv8353f.h。 */
bool
drv8353f_read(uint8_t address, uint16_t *value)
{
    uint16_t received;
    if (NULL == g_drv_transfer || NULL == value || address > 6U ||
        !g_drv_transfer((uint16_t)(0x8000U | ((uint32_t)address << 11)), &received))
    {
        return false;
    }
    *value = received & 0x07FFU;
    return true;
}

/** @brief 见 drv8353f.h。 */
bool
drv8353f_init(drv8353f_transfer_t transfer)
{
    return drv8353f_init_gain(transfer, 20U);
}

/** @brief 见 drv8353f.h。 */
bool
drv8353f_init_gain(drv8353f_transfer_t transfer, uint8_t gain)
{
    if (gain != 20U && gain != 40U) { return false; }
    g_drv_csa = gain == 40U ? 0x02C0U : 0x0280U;
    g_drv_transfer = transfer;
    /* 首先强制高阻，再解锁和配置；六个物理输入必须事先保持低。 */
    if (!write_verify(2U, DRV8353F_CONTROL_SAFE)) { return false; }
    for (uint8_t i = 1U; i < 5U; ++i)
    {
        if (!write_verify((uint8_t)(i + 2U), i == 4U ? g_drv_csa : g_drv_config[i])) { return false; }
    }
    /* 锁定后地址 2 的 COAST 位仍可改，其余配置不得意外改变。 */
    return write_verify(3U, 0x0600U) && drv8353f_verify(true);
}

/** @brief 见 drv8353f.h。 */
bool
drv8353f_verify(bool coast)
{
    for (uint8_t i = 0U; i < 5U; ++i)
    {
        uint16_t value;
        uint16_t expected = g_drv_config[i];
        if (i == 0U && !coast) { expected &= (uint16_t)~4U; }
        if (i == 1U) { expected = 0x0600U; }
        if (i == 4U) { expected = g_drv_csa; }
        if (!drv8353f_read((uint8_t)(i + 2U), &value) || value != expected) { return false; }
    }
    return true;
}

/** @brief 见 drv8353f.h。 */
bool
drv8353f_coast(bool coast)
{
    return write_verify(2U, (uint16_t)(coast ? DRV8353F_CONTROL_SAFE : (DRV8353F_CONTROL_SAFE & ~4U)));
}

/** @brief 见 drv8353f.h。 */
bool
drv8353f_faults(uint16_t *fault1, uint16_t *fault2)
{
    return drv8353f_read(0U, fault1) && drv8353f_read(1U, fault2);
}

/** @brief 见 drv8353f.h。 */
bool
drv8353f_calibrate(bool enabled)
{
    if (!drv8353f_verify(true)) { return false; }
    uint16_t value = (uint16_t)((g_drv_csa & ~0x001CU) | (enabled ? 0x001CU : 0U));
    if (!write_verify(3U, 0x0300U) || !write_verify(6U, value) || !write_verify(3U, 0x0600U))
    {
        return false;
    }
    g_drv_csa = value;
    return drv8353f_verify(true);
}
