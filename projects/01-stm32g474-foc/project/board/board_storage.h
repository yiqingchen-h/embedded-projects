#ifndef FOC_BOARD_STORAGE_H
#define FOC_BOARD_STORAGE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* 两份独立 4 KiB 槽，单/双 Bank 都落在第一 Bank 的相同地址。 */
#define BOARD_STORAGE_SLOT_SIZE 4096U
#define BOARD_STORAGE_BASE 0x0800E000UL
/* 仅主循环查询；记录最近一次保存结果，不读参数区或改变 Option Bytes。 */
typedef struct {
    /* 芯片当前的原始 Option Bytes，可从 DBANK 位核对实际布局。 */
    uint32_t optr;
    /* 厂商容量寄存器，单位 KiB。 */
    uint16_t flash_kb;
    /* 当前构建目标要求的布局，false 为单 Bank。 */
    bool build_dual_bank;
    /* 由 OPTR 的 DBANK 位读取的芯片当前布局。 */
    bool actual_dual_bank;
    /* NONE / OK / LAYOUT / ARGUMENT / UNLOCK / ERASE / PROGRAM / VERIFY / LOCK。 */
    const char *result;
    /* HAL Flash 错误码；预检查失败时为零。 */
    uint32_t hal_error;
    /* 最近保存槽地址，编程失败时为对应双字地址；未保存为零。 */
    uint32_t address;
    /* 擦除失败的页号；未发生擦除页错误为 UINT32_MAX。 */
    uint32_t page_error;
} board_storage_status_t;
/** @brief 查询布局及最近保存结果，布局不匹配时报告 LAYOUT。 @return 主循环状态副本。 */
board_storage_status_t board_storage_status(void);
/** @brief 检查链接布局与实际 DBANK 一致。 @return 一致为 true。 */
bool board_storage_layout_valid(void);
/** @brief 读取一份 Flash 记录。 @param slot 0 或 1。 @param data 输出。 @param size 字节数。 @return 已读取。 */
bool board_storage_read(unsigned slot, void *data, size_t size);
/** @brief 擦除旧槽并双字编程，调用者保证停机及 ADC 停止。 @param slot 0 或 1。 @param data 记录。 @param size 8 字节对齐长度。 @return 写入并逐字节回读一致。 */
bool board_storage_write(unsigned slot, const void *data, size_t size);
#endif
