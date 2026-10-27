#include "hardware/Flash/flash.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <stdbool.h>
#include <stddef.h>

/* 直接命令阶段长度，当前 JEDEC ID 使用 1 字节命令 0x9F。 */
#define FLASH_DIRECT_COMMAND_LENGTH     (1U)

/* JEDEC ID 固定读取 3 字节：厂商、类型、容量。 */
#define FLASH_JEDEC_ID_LENGTH           (3U)

/* Flash 忙轮询间隔，单位 ms。 */
#define FLASH_READY_POLL_DELAY_MS       (1U)

/* 页写入等待超时时间，单位 ms。 */
#define FLASH_WRITE_TIMEOUT_MS          (500U)

/* 4KB 扇区擦除等待超时时间，单位 ms。 */
#define FLASH_SECTOR_ERASE_TIMEOUT_MS   (1000U)

/* 64KB 块擦除等待超时时间，单位 ms。 */
#define FLASH_BLOCK_ERASE_TIMEOUT_MS    (5000U)

/* 自检写入和读回整个 4KB 诊断扇区。 */
#define FLASH_TEST_PATTERN_BYTES        (FLASH_SECTOR_SIZE_BYTES)

/* 8 字节写入对齐检查掩码，匹配 OSPI_B 非 DMA 写入要求。 */
#define FLASH_ALIGN_MASK_8              (FLASH_WRITE_ALIGN_BYTES - 1UL)

/* 4KB 扇区地址对齐检查掩码。 */
#define FLASH_SECTOR_ALIGN_MASK         (FLASH_SECTOR_SIZE_BYTES - 1UL)

/* 64KB 块地址对齐检查掩码。 */
#define FLASH_BLOCK_ALIGN_MASK          (FLASH_BLOCK_SIZE_BYTES - 1UL)

/* Flash API 互斥锁静态内存，避免运行时动态分配。 */
static StaticSemaphore_t g_flash_mutex_storage;

/* Flash API 互斥锁句柄，保护 OSPI 直接命令、擦除和写入流程。 */
static SemaphoreHandle_t g_flash_mutex = NULL;

/* OSPI 驱动打开标记，允许 flash.init() 和其他 API 重复调用。 */
static bool g_flash_opened = false;

/* 最近一次读取到的 JEDEC ID，初始化时会更新。 */
static flash_id_t g_flash_last_id = {0};

/* Flash 自检写入缓冲区，按 OSPI_B CPU 写入要求做 8 字节对齐。 */
static uint8_t g_flash_test_write_buffer[FLASH_TEST_PATTERN_BYTES] BSP_ALIGN_VARIABLE(8);

/* Flash 自检读回缓冲区，按 8 字节对齐便于复用写入校验数据。 */
static uint8_t g_flash_test_read_buffer[FLASH_TEST_PATTERN_BYTES] BSP_ALIGN_VARIABLE(8);

/**
 * @brief  判断调度器是否已经运行
 *
 * @return bool  true 表示可使用 FreeRTOS 阻塞等待
 */
static bool flash_is_scheduler_running(void)
{
    return (taskSCHEDULER_RUNNING == xTaskGetSchedulerState());
}

/**
 * @brief  获取 Flash 互斥锁
 *
 * @return fsp_err_t  FSP_SUCCESS 表示已获得互斥锁
 */
static fsp_err_t flash_lock(void)
{
    if (NULL == g_flash_mutex)
    {
        g_flash_mutex = xSemaphoreCreateMutexStatic(&g_flash_mutex_storage);
        if (NULL == g_flash_mutex)
        {
            return FSP_ERR_INTERNAL;
        }
    }

    if (flash_is_scheduler_running())
    {
        if (pdTRUE != xSemaphoreTake(g_flash_mutex, portMAX_DELAY))
        {
            return FSP_ERR_IN_USE;
        }
    }

    return FSP_SUCCESS;
}

/**
 * @brief  释放 Flash 互斥锁
 *
 * @return void
 */
static void flash_unlock(void)
{
    if ((NULL != g_flash_mutex) && flash_is_scheduler_running())
    {
        (void)xSemaphoreGive(g_flash_mutex);
    }
}

/**
 * @brief  毫秒延时，兼容调度器启动前后
 *
 * @param ms  延时时间，单位 ms
 * @return void
 */
static void flash_delay_ms(uint32_t ms)
{
    TickType_t ticks;

    if (0U == ms)
    {
        return;
    }

    ticks = pdMS_TO_TICKS(ms);
    if (flash_is_scheduler_running() && (ticks > 0U))
    {
        vTaskDelay(ticks);
    }
    else
    {
        R_BSP_SoftwareDelay(ms, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

/**
 * @brief  检查 Flash 偏移范围是否合法
 *
 * @param address  Flash 内偏移地址
 * @param length   访问长度，单位 byte
 * @return bool  true 表示范围合法
 */
static bool flash_is_range_valid(uint32_t address, uint32_t length)
{
    uint32_t end_address = address + length;

    return ((length > 0U) &&
            (end_address >= address) &&
            (end_address <= FLASH_TOTAL_SIZE_BYTES));
}

/**
 * @brief  检查写入参数是否满足 OSPI_B CPU 写入约束
 *
 * @param address  Flash 内偏移地址
 * @param p_src    写入数据指针
 * @param length   写入长度
 * @return bool  true 表示参数满足 8 字节对齐
 */
static bool flash_is_write_aligned(uint32_t address, const uint8_t *p_src, uint32_t length)
{
    return ((0U == (address & FLASH_ALIGN_MASK_8)) &&
            (0U == (((uintptr_t)p_src) & FLASH_ALIGN_MASK_8)) &&
            (0U == (length & FLASH_ALIGN_MASK_8)));
}

/**
 * @brief  将 Flash 偏移地址转成 OSPI 内存映射地址
 *
 * @param address  Flash 内偏移地址
 * @return uint8_t*  内存映射地址
 */
static uint8_t *flash_address_to_ptr(uint32_t address)
{
    return (uint8_t *)(FLASH_BASE_ADDRESS + address);
}

/**
 * @brief  清空 Flash 测试结果
 *
 * @param p_result  测试结果指针，可传入 NULL
 * @return void
 */
static void flash_clear_test_result(flash_test_result_t *p_result)
{
    if (NULL != p_result)
    {
        p_result->name = NULL;
        p_result->address = 0U;
        p_result->offset = 0U;
        p_result->expected = 0U;
        p_result->actual = 0U;
        p_result->err = FSP_SUCCESS;
    }
}

/**
 * @brief  写入 Flash 测试失败信息
 *
 * @param p_result  测试结果指针，可传入 NULL
 * @param name      失败阶段名称
 * @param address   Flash 内偏移地址
 * @param offset    失败偏移
 * @param expected  期望值
 * @param actual    实际值
 * @param err       错误码
 * @return void
 */
static void flash_set_test_result(flash_test_result_t *p_result,
                                  const char *name,
                                  uint32_t address,
                                  uint32_t offset,
                                  uint8_t expected,
                                  uint8_t actual,
                                  fsp_err_t err)
{
    if (NULL != p_result)
    {
        p_result->name = name;
        p_result->address = address;
        p_result->offset = offset;
        p_result->expected = expected;
        p_result->actual = actual;
        p_result->err = err;
    }
}

/**
 * @brief  等待 Flash 空闲
 *
 * @param timeout_ms  超时时间，单位 ms
 * @return fsp_err_t  FSP_SUCCESS 表示 Flash 已空闲
 */
static fsp_err_t flash_wait_ready_unlocked(uint32_t timeout_ms)
{
    uint32_t elapsed_ms = 0U;
    spi_flash_status_t status;
    fsp_err_t err;

    while (1)
    {
        err = g_ospi0.p_api->statusGet(g_ospi0.p_ctrl, &status);
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        if (!status.write_in_progress)
        {
            return FSP_SUCCESS;
        }

        if (elapsed_ms >= timeout_ms)
        {
            return FSP_ERR_TIMEOUT;
        }

        flash_delay_ms(FLASH_READY_POLL_DELAY_MS);
        elapsed_ms += FLASH_READY_POLL_DELAY_MS;
    }
}

/**
 * @brief  打开 OSPI Flash，允许重复调用
 *
 * @return fsp_err_t  FSP_SUCCESS 表示 OSPI Flash 可用
 */
static fsp_err_t flash_open_unlocked(void)
{
    fsp_err_t err;

    if (g_flash_opened)
    {
        return FSP_SUCCESS;
    }

    err = g_ospi0.p_api->open(g_ospi0.p_ctrl, g_ospi0.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        return err;
    }

    g_flash_opened = true;
    err = flash_wait_ready_unlocked(FLASH_WRITE_TIMEOUT_MS);
    if (FSP_SUCCESS != err)
    {
        g_flash_opened = false;
    }

    return err;
}

/**
 * @brief  不加锁读取 JEDEC ID
 *
 * @param p_id  JEDEC ID 输出结构体
 * @return fsp_err_t  FSP_SUCCESS 表示读取成功
 */
static fsp_err_t flash_read_jedec_id_unlocked(flash_id_t *p_id)
{
    spi_flash_direct_transfer_t transfer =
    {
        .command = 0x9F,
        .command_length = FLASH_DIRECT_COMMAND_LENGTH,
        .address = 0U,
        .address_length = 0U,
        .data_length = FLASH_JEDEC_ID_LENGTH,
        .dummy_cycles = 0U,
    };
    fsp_err_t err;

    if (NULL == p_id)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    err = flash_open_unlocked();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = g_ospi0.p_api->directTransfer(g_ospi0.p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    p_id->manufacturer_id = (uint8_t)(transfer.data_u64 & 0xFFU);
    p_id->memory_type = (uint8_t)((transfer.data_u64 >> 8U) & 0xFFU);
    p_id->capacity = (uint8_t)((transfer.data_u64 >> 16U) & 0xFFU);
    g_flash_last_id = *p_id;

    return FSP_SUCCESS;
}

/**
 * @brief  不加锁读取 Flash 内存映射区
 *
 * @param address  Flash 内偏移地址
 * @param p_dest   读取缓冲区
 * @param length   读取长度
 * @return fsp_err_t  FSP_SUCCESS 表示读取成功
 */
static fsp_err_t flash_read_unlocked(uint32_t address, uint8_t *p_dest, uint32_t length)
{
    volatile uint8_t const *p_src;
    fsp_err_t err;

    if (NULL == p_dest)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    if (!flash_is_range_valid(address, length))
    {
        return FSP_ERR_INVALID_ADDRESS;
    }

    err = flash_open_unlocked();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    p_src = (volatile uint8_t const *)flash_address_to_ptr(address);
    for (uint32_t i = 0U; i < length; i++)
    {
        p_dest[i] = p_src[i];
    }

    return FSP_SUCCESS;
}

/**
 * @brief  不加锁擦除 Flash
 *
 * @param address     Flash 内偏移地址
 * @param size        擦除大小
 * @param timeout_ms  等待超时时间
 * @return fsp_err_t  FSP_SUCCESS 表示擦除完成
 */
static fsp_err_t flash_erase_unlocked(uint32_t address, uint32_t size, uint32_t timeout_ms)
{
    fsp_err_t err;

    if (!flash_is_range_valid(address, size))
    {
        return FSP_ERR_INVALID_ADDRESS;
    }

    err = flash_open_unlocked();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = g_ospi0.p_api->erase(g_ospi0.p_ctrl, flash_address_to_ptr(address), size);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return flash_wait_ready_unlocked(timeout_ms);
}

/**
 * @brief  不加锁写入 Flash
 *
 * @param address  Flash 内偏移地址
 * @param p_src    写入数据
 * @param length   写入长度
 * @return fsp_err_t  FSP_SUCCESS 表示写入完成
 */
static fsp_err_t flash_write_unlocked(uint32_t address, const uint8_t *p_src, uint32_t length)
{
    uint32_t offset = 0U;
    fsp_err_t err;

    if (NULL == p_src)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    if (!flash_is_range_valid(address, length))
    {
        return FSP_ERR_INVALID_ADDRESS;
    }

    if (!flash_is_write_aligned(address, p_src, length))
    {
        return FSP_ERR_INVALID_ALIGNMENT;
    }

    err = flash_open_unlocked();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    while (offset < length)
    {
        uint32_t current_address = address + offset;
        uint32_t page_remaining = FLASH_WRITE_CHUNK_BYTES - (current_address & (FLASH_WRITE_CHUNK_BYTES - 1UL));
        uint32_t chunk = length - offset;

        if (chunk > page_remaining)
        {
            chunk = page_remaining;
        }

        err = g_ospi0.p_api->write(g_ospi0.p_ctrl,
                                   &p_src[offset],
                                   flash_address_to_ptr(current_address),
                                   chunk);
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        err = flash_wait_ready_unlocked(FLASH_WRITE_TIMEOUT_MS);
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        offset += chunk;
    }

    return FSP_SUCCESS;
}

/**
 * @brief  初始化 OSPI Flash
 *
 * @return fsp_err_t  FSP_SUCCESS 表示 Flash 可用
 */
static fsp_err_t flash_init_impl(void)
{
    fsp_err_t err = flash_lock();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = flash_open_unlocked();
    if (FSP_SUCCESS == err)
    {
        err = flash_read_jedec_id_unlocked(&g_flash_last_id);
    }

    flash_unlock();
    return err;
}

/**
 * @brief  读取 JEDEC ID
 *
 * @param p_id  JEDEC ID 输出结构体
 * @return fsp_err_t  FSP_SUCCESS 表示读取成功
 */
static fsp_err_t flash_read_jedec_id_impl(flash_id_t *p_id)
{
    fsp_err_t err = flash_lock();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = flash_read_jedec_id_unlocked(p_id);

    flash_unlock();
    return err;
}

/**
 * @brief  读取 Flash 数据
 *
 * @param address  Flash 内偏移地址
 * @param p_dest   读取缓冲区
 * @param length   读取长度
 * @return fsp_err_t  FSP_SUCCESS 表示读取成功
 */
static fsp_err_t flash_read_impl(uint32_t address, uint8_t *p_dest, uint32_t length)
{
    fsp_err_t err = flash_lock();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = flash_read_unlocked(address, p_dest, length);

    flash_unlock();
    return err;
}

/**
 * @brief  擦除 4KB 扇区
 *
 * @param address  Flash 内偏移地址，必须 4KB 对齐
 * @return fsp_err_t  FSP_SUCCESS 表示擦除完成
 */
static fsp_err_t flash_erase_sector_impl(uint32_t address)
{
    fsp_err_t err;

    if (0U != (address & FLASH_SECTOR_ALIGN_MASK))
    {
        return FSP_ERR_INVALID_ALIGNMENT;
    }

    err = flash_lock();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = flash_erase_unlocked(address, FLASH_SECTOR_SIZE_BYTES, FLASH_SECTOR_ERASE_TIMEOUT_MS);

    flash_unlock();
    return err;
}

/**
 * @brief  擦除 64KB 块
 *
 * @param address  Flash 内偏移地址，必须 64KB 对齐
 * @return fsp_err_t  FSP_SUCCESS 表示擦除完成
 */
static fsp_err_t flash_erase_block_impl(uint32_t address)
{
    fsp_err_t err;

    if (0U != (address & FLASH_BLOCK_ALIGN_MASK))
    {
        return FSP_ERR_INVALID_ALIGNMENT;
    }

    err = flash_lock();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = flash_erase_unlocked(address, FLASH_BLOCK_SIZE_BYTES, FLASH_BLOCK_ERASE_TIMEOUT_MS);

    flash_unlock();
    return err;
}

/**
 * @brief  写入已擦除的 Flash 区域
 *
 * @param address  Flash 内偏移地址
 * @param p_src    写入数据
 * @param length   写入长度
 * @return fsp_err_t  FSP_SUCCESS 表示写入完成
 */
static fsp_err_t flash_write_impl(uint32_t address, const uint8_t *p_src, uint32_t length)
{
    fsp_err_t err = flash_lock();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = flash_write_unlocked(address, p_src, length);

    flash_unlock();
    return err;
}

/**
 * @brief  执行 Flash 擦写读自检
 *
 * @param address   Flash 内偏移地址
 * @param p_result  失败时保存失败信息，可传入 NULL
 * @return fsp_err_t  FSP_SUCCESS 表示自检通过
 */
static fsp_err_t flash_test_impl(uint32_t address, flash_test_result_t *p_result)
{
    fsp_err_t err;

    flash_clear_test_result(p_result);

    if (0U != (address & FLASH_SECTOR_ALIGN_MASK))
    {
        flash_set_test_result(p_result, "erase", address, 0U, 0U, 0U, FSP_ERR_INVALID_ALIGNMENT);
        return FSP_ERR_INVALID_ALIGNMENT;
    }

    err = flash_lock();
    if (FSP_SUCCESS != err)
    {
        flash_set_test_result(p_result, "lock", address, 0U, 0U, 0U, err);
        return err;
    }

    for (uint32_t i = 0U; i < FLASH_TEST_PATTERN_BYTES; i++)
    {
        g_flash_test_write_buffer[i] = (uint8_t)(0xA5U ^ i);
        g_flash_test_read_buffer[i] = 0U;
    }

    err = flash_erase_unlocked(address, FLASH_SECTOR_SIZE_BYTES, FLASH_SECTOR_ERASE_TIMEOUT_MS);
    if (FSP_SUCCESS != err)
    {
        flash_set_test_result(p_result, "erase", address, 0U, 0U, 0U, err);
        flash_unlock();
        return err;
    }

    err = flash_read_unlocked(address, g_flash_test_read_buffer, FLASH_TEST_PATTERN_BYTES);
    if (FSP_SUCCESS != err)
    {
        flash_set_test_result(p_result, "erase_read", address, 0U, 0U, 0U, err);
        flash_unlock();
        return err;
    }

    /* 擦除后先确认整个诊断扇区都读回 0xFF。 */
    for (uint32_t i = 0U; i < FLASH_TEST_PATTERN_BYTES; i++)
    {
        if (0xFFU != g_flash_test_read_buffer[i])
        {
            flash_set_test_result(p_result,
                                  "erase_verify",
                                  address,
                                  i,
                                  0xFFU,
                                  g_flash_test_read_buffer[i],
                                  FSP_ERR_INVALID_DATA);
            flash_unlock();
            return FSP_ERR_INVALID_DATA;
        }

        g_flash_test_read_buffer[i] = 0U;
    }

    err = flash_write_unlocked(address, g_flash_test_write_buffer, FLASH_TEST_PATTERN_BYTES);
    if (FSP_SUCCESS != err)
    {
        flash_set_test_result(p_result, "write", address, 0U, 0U, 0U, err);
        flash_unlock();
        return err;
    }

    err = flash_read_unlocked(address, g_flash_test_read_buffer, FLASH_TEST_PATTERN_BYTES);
    if (FSP_SUCCESS != err)
    {
        flash_set_test_result(p_result, "read", address, 0U, 0U, 0U, err);
        flash_unlock();
        return err;
    }

    for (uint32_t i = 0U; i < FLASH_TEST_PATTERN_BYTES; i++)
    {
        if (g_flash_test_read_buffer[i] != g_flash_test_write_buffer[i])
        {
            flash_set_test_result(p_result,
                                  "verify",
                                  address,
                                  i,
                                  g_flash_test_write_buffer[i],
                                  g_flash_test_read_buffer[i],
                                  FSP_ERR_INVALID_DATA);
            flash_unlock();
            return FSP_ERR_INVALID_DATA;
        }
    }

    flash_unlock();
    return FSP_SUCCESS;
}

const flash_api_t flash =
{
    .init = flash_init_impl,
    .read_jedec_id = flash_read_jedec_id_impl,
    .read = flash_read_impl,
    .erase_sector = flash_erase_sector_impl,
    .erase_block = flash_erase_block_impl,
    .write = flash_write_impl,
    .test = flash_test_impl,
};
