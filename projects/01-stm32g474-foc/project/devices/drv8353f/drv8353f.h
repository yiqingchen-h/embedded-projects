#ifndef FOC_DRV8353F_H
#define FOC_DRV8353F_H
#include <stdbool.h>
#include <stdint.h>

/* 一次完整的 16 位 SPI 事务，由板卡层管理片选及超时。 */
typedef bool (*drv8353f_transfer_t)(uint16_t tx, uint16_t *rx);
/* 六路 PWM、全桥故障关断、报告温度预警；常态保持 COAST。 */
#define DRV8353F_CONTROL_SAFE 0x0484U

/** @brief 配置并逐项回读保护与采样参数，保持高阻。 @param transfer SPI 事务接口。 @return 成功为 true；失败后调用者必须拉低 ENABLE。 */
bool drv8353f_init(drv8353f_transfer_t transfer);
/** @brief 在六输入低且功率禁用时初始化指定采样增益，保持COAST。 @param transfer 总线接口。 @param gain 仅10、20或40。 @return 配置及回读成功。 */
bool drv8353f_init_gain(drv8353f_transfer_t transfer, uint8_t gain);
/** @brief 在COAST下切换三路CSA输入短接校准，调用者负责等待稳定。 @param enabled 是否短接输入。 @return 写入、重新锁定及回读成功。 */
bool drv8353f_calibrate(bool enabled);
/** @brief 读取状态或配置寄存器。 @param address 地址 0～6。 @param value 返回低 11 位。 @return 事务成功且参数有效为 true。 */
bool drv8353f_read(uint8_t address, uint16_t *value);
/** @brief 核对所有受保护配置未变化。 @param coast 当前是否应为高阻。 @return 全部一致为 true。 */
bool drv8353f_verify(bool coast);
/** @brief 仅切换高阻位，不改变保护参数。 @param coast true 强制高阻。 @return 写入并回读一致为 true。 */
bool drv8353f_coast(bool coast);
/** @brief 读取两组故障。 @param fault1 状态寄存器 0。 @param fault2 状态寄存器 1。 @return 通信成功为 true，调用者另判断故障值。 */
bool drv8353f_faults(uint16_t *fault1, uint16_t *fault2);
#endif
