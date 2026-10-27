#include "board_storage.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* 构建目标必须显式选择芯片当前的 Option Bytes 布局，不自动更改 DBANK。 */
#ifndef BOARD_FLASH_DUAL_BANK
#error Select BOARD_FLASH_DUAL_BANK in the build target
#endif

/* 保存失败信息保留到下一次写入；读参数和周期查询不会覆盖它。 */
static board_storage_status_t g_storage_status = {.result = "NONE", .page_error = UINT32_MAX};

/** @brief STM32G474RB 两种 Flash 布局均保留 Bank1 尾部 8 KiB。 */
bool
board_storage_layout_valid(void)
{
    return ((FLASH->OPTR & FLASH_OPTR_DBANK) != 0U) == (BOARD_FLASH_DUAL_BANK != 0);
}

/** @brief 直接读取容量和布局寄存器，不触发擦写或故障恢复。 @return 当前状态副本。 */
board_storage_status_t
board_storage_status(void)
{
    board_storage_status_t status = g_storage_status;
    status.optr = FLASH->OPTR;
    status.flash_kb = *(const uint16_t *)FLASHSIZE_BASE;
    status.build_dual_bank = BOARD_FLASH_DUAL_BANK != 0;
    status.actual_dual_bank = (status.optr & FLASH_OPTR_DBANK) != 0U;
    if (status.actual_dual_bank != status.build_dual_bank) { status.result = "LAYOUT"; }
    return status;
}

/** @brief 只在布局匹配时读取，防止将程序区解释为配置。 */
bool
board_storage_read(unsigned slot, void *data, size_t size)
{
    if (slot > 1 || size > BOARD_STORAGE_SLOT_SIZE || !board_storage_layout_valid()) { return false; }
    memcpy(data, (const void *)(BOARD_STORAGE_BASE + slot * BOARD_STORAGE_SLOT_SIZE), size);
    return true;
}

/** @brief 擦除独立槽、顺序编程且回读；旧槽保持有效直到新记录提交。 */
bool
board_storage_write(unsigned slot, const void *data, size_t size)
{
    g_storage_status = (board_storage_status_t){.result = "ARGUMENT", .page_error = UINT32_MAX};
    if (slot > 1 || NULL == data || size == 0 || size > BOARD_STORAGE_SLOT_SIZE || (size % 8U) != 0U) { return false; }
    g_storage_status.result = "LAYOUT";
    if (!board_storage_layout_valid()) { return false; }
    uint32_t address = BOARD_STORAGE_BASE + slot * BOARD_STORAGE_SLOT_SIZE;
    g_storage_status.address = address;
    uint32_t page_size = BOARD_FLASH_DUAL_BANK ? 2048U : 4096U;
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (address - FLASH_BASE) / page_size;
    erase.NbPages = BOARD_STORAGE_SLOT_SIZE / page_size;
    g_storage_status.result = "UNLOCK";
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        g_storage_status.hal_error = HAL_FLASH_GetError();
        return false;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    g_storage_status.result = "ERASE";
    bool ok = HAL_FLASHEx_Erase(&erase, &g_storage_status.page_error) == HAL_OK;
    for (size_t offset = 0; ok && offset < size; offset += 8U)
    {
        uint64_t word;
        memcpy(&word, (const uint8_t *)data + offset, sizeof(word));
        g_storage_status.result = "PROGRAM";
        g_storage_status.address = address + (uint32_t)offset;
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + (uint32_t)offset, word) == HAL_OK;
    }
    /* 在重新锁定之前保存错误码，避免后续 HAL 调用掩盖失败原因。 */
    if (!ok) { g_storage_status.hal_error = HAL_FLASH_GetError(); }
    HAL_StatusTypeDef locked = HAL_FLASH_Lock();
    if (!ok) { return false; }
    if (locked != HAL_OK)
    {
        g_storage_status.result = "LOCK";
        g_storage_status.hal_error = HAL_FLASH_GetError();
        return false;
    }
    /* HAL 擦写流程负责 Flash 数据缓存失效；回读比较包括 CRC 和最后提交字。 */
    g_storage_status.address = address;
    g_storage_status.result = "VERIFY";
    if (memcmp((const void *)address, data, size) != 0) { return false; }
    g_storage_status.result = "OK";
    return true;
}
