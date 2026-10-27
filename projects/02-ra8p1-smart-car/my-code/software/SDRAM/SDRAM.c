#include "software/SDRAM/SDRAM.h"

#include <stddef.h>

/* SDRAM 测试地址必须 4 字节对齐。 */
#define SDRAM_WORD_ALIGN_MASK           (0x03UL)

/**
 * @brief  清空 SDRAM 测试结果结构体
 *
 * @param p_result  测试结果结构体指针，可传入 NULL
 * @return void
 */
static void sdram_clear_result(sdram_test_result_t *p_result)
{
    if (NULL != p_result)
    {
        p_result->name = NULL;
        p_result->address = 0U;
        p_result->expected = 0U;
        p_result->actual = 0U;
    }
}

/**
 * @brief  写入 SDRAM 测试失败信息
 *
 * @param p_result  测试结果结构体指针，可传入 NULL
 * @param name      失败项目名称，可传入 NULL
 * @param address   失败地址
 * @param expected  期望值
 * @param actual    实际读回值
 * @return void
 */
static void sdram_set_result(sdram_test_result_t *p_result,
                             const char *name,
                             uint32_t address,
                             uint32_t expected,
                             uint32_t actual)
{
    if (NULL != p_result)
    {
        p_result->name = name;
        p_result->address = address;
        p_result->expected = expected;
        p_result->actual = actual;
    }
}

/**
 * @brief  检查 SDRAM 测试范围是否合法
 *
 * @param start_address  测试起始地址
 * @param words          测试字数，单位为 32bit word
 * @return fsp_err_t     FSP_SUCCESS 表示范围合法
 */
static fsp_err_t sdram_check_range(uint32_t start_address, uint32_t words)
{
    uint32_t byte_length = words * sizeof(uint32_t);
    uint32_t end_address = start_address + byte_length;
    uint32_t sdram_end_address = SDRAM_USER_START_ADDRESS + SDRAM_USER_SIZE_BYTES;

    if ((0U == words) || (0U != (start_address & SDRAM_WORD_ALIGN_MASK)))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    if ((byte_length / sizeof(uint32_t)) != words)
    {
        return FSP_ERR_INVALID_SIZE;
    }

    if ((end_address < start_address) ||
        (start_address < SDRAM_USER_START_ADDRESS) ||
        (end_address > sdram_end_address))
    {
        return FSP_ERR_INVALID_ADDRESS;
    }

    return FSP_SUCCESS;
}

/**
 * @brief  向 SDRAM 写入测试图案并读回校验
 *
 * @param start_address  测试起始地址
 * @param words          测试字数，单位为 32bit word
 * @param seed           测试图案种子
 * @param p_result       测试失败时保存失败信息，可传入 NULL
 * @return fsp_err_t     FSP_SUCCESS 表示本轮图案校验通过
 */
static fsp_err_t sdram_write_verify(uint32_t start_address,
                                    uint32_t words,
                                    uint32_t seed,
                                    sdram_test_result_t *p_result)
{
    volatile uint32_t *p_memory = (volatile uint32_t *) start_address;

    for (uint32_t i = 0U; i < words; i++)
    {
        p_memory[i] = seed ^ i;
    }

    for (uint32_t i = 0U; i < words; i++)
    {
        uint32_t expected = seed ^ i;
        uint32_t actual = p_memory[i];

        if (actual != expected)
        {
            sdram_set_result(p_result, "verify", start_address + (i * sizeof(uint32_t)), expected, actual);
            return FSP_ERR_INVALID_DATA;
        }
    }

    return FSP_SUCCESS;
}

/**
 * @brief  写入并读回一个 32bit SDRAM 单元
 *
 * @param address   测试地址
 * @param value     写入值
 * @param name      测试项目名称
 * @param p_result  测试失败时保存失败信息，可传入 NULL
 * @return fsp_err_t  FSP_SUCCESS 表示读回一致
 */
static fsp_err_t sdram_diag_write_read32(uint32_t address,
                                         uint32_t value,
                                         const char *name,
                                         sdram_test_result_t *p_result)
{
    volatile uint32_t *p_word = (volatile uint32_t *) address;

    *p_word = value;
    uint32_t actual = *p_word;
    if (actual != value)
    {
        sdram_set_result(p_result, name, address, value, actual);
        return FSP_ERR_INVALID_DATA;
    }

    return FSP_SUCCESS;
}

/**
 * @brief  测试单地址数据线
 *
 * @param start_address  诊断起始地址
 * @param p_result       诊断失败时保存失败信息，可传入 NULL
 * @return fsp_err_t     FSP_SUCCESS 表示数据线诊断通过
 */
static fsp_err_t sdram_diag_data_bus(uint32_t start_address, sdram_test_result_t *p_result)
{
    static const uint32_t patterns[] =
    {
        0x00000000UL,
        0xFFFFFFFFUL,
        0xAAAAAAAAUL,
        0x55555555UL,
        0x000000FFUL,
        0x0000FF00UL,
        0x00FF0000UL,
        0xFF000000UL,
    };

    for (uint32_t i = 0U; i < (sizeof(patterns) / sizeof(patterns[0])); i++)
    {
        fsp_err_t err = sdram_diag_write_read32(start_address, patterns[i], "pattern", p_result);
        if (FSP_SUCCESS != err)
        {
            return err;
        }
    }

    for (uint32_t bit = 0U; bit < 32U; bit++)
    {
        fsp_err_t err = sdram_diag_write_read32(start_address, (1UL << bit), "walking1", p_result);
        if (FSP_SUCCESS != err)
        {
            return err;
        }
    }

    for (uint32_t bit = 0U; bit < 32U; bit++)
    {
        fsp_err_t err = sdram_diag_write_read32(start_address, ~(1UL << bit), "walking0", p_result);
        if (FSP_SUCCESS != err)
        {
            return err;
        }
    }

    return FSP_SUCCESS;
}

/**
 * @brief  测试低地址范围是否存在地址线混叠
 *
 * @param start_address  诊断起始地址
 * @param words          诊断字数，单位为 32bit word
 * @param p_result       诊断失败时保存失败信息，可传入 NULL
 * @return fsp_err_t     FSP_SUCCESS 表示地址线诊断通过
 */
static fsp_err_t sdram_diag_address_bus(uint32_t start_address, uint32_t words, sdram_test_result_t *p_result)
{
    volatile uint32_t *p_base = (volatile uint32_t *) start_address;

    for (uint32_t i = 0U; i < words; i++)
    {
        p_base[i] = 0x00000000UL;
    }

    for (uint32_t offset = 1U; offset < words; offset <<= 1U)
    {
        p_base[offset] = 0xA5000000UL | offset;
    }

    if (0x00000000UL != p_base[0])
    {
        sdram_set_result(p_result, "addr base", start_address, 0x00000000UL, p_base[0]);
        return FSP_ERR_INVALID_DATA;
    }

    for (uint32_t offset = 1U; offset < words; offset <<= 1U)
    {
        uint32_t expected = 0xA5000000UL | offset;
        uint32_t actual = p_base[offset];
        if (actual != expected)
        {
            sdram_set_result(p_result, "addr line", start_address + (offset * sizeof(uint32_t)), expected, actual);
            return FSP_ERR_INVALID_DATA;
        }
    }

    return FSP_SUCCESS;
}

/**
 * @brief  测试指定 SDRAM 地址范围
 *
 * @param start_address  测试起始地址，必须 4 字节对齐
 * @param words          测试字数，单位为 32bit word
 * @param p_result       测试失败时保存失败地址和数据，可传入 NULL
 * @return fsp_err_t     FSP_SUCCESS 表示指定范围读写测试通过
 */
static fsp_err_t sdram_test_range_impl(uint32_t start_address, uint32_t words, sdram_test_result_t *p_result)
{
    static const uint32_t test_patterns[] =
    {
        0x00000000UL,
        0xFFFFFFFFUL,
        0xA5A5A5A5UL,
        0x5A5A5A5AUL,
    };

    fsp_err_t err = sdram_check_range(start_address, words);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    sdram_clear_result(p_result);

    for (uint32_t i = 0U; i < (sizeof(test_patterns) / sizeof(test_patterns[0])); i++)
    {
        err = sdram_write_verify(start_address, words, test_patterns[i], p_result);
        if (FSP_SUCCESS != err)
        {
            return err;
        }
    }

    return FSP_SUCCESS;
}

/**
 * @brief  测试默认 SDRAM 区域，默认避开 LCD framebuffer 起始区域
 *
 * @param p_result  测试失败时保存失败地址和数据，可传入 NULL
 * @return fsp_err_t  FSP_SUCCESS 表示 SDRAM 读写测试通过
 */
static fsp_err_t sdram_test_impl(sdram_test_result_t *p_result)
{
    return sdram_test_range_impl(SDRAM_TEST_START_ADDRESS, SDRAM_DEFAULT_TEST_WORDS, p_result);
}

/**
 * @brief  执行 SDRAM 数据线和地址线诊断
 *
 * @param p_result  诊断失败时保存失败项目和数据，可传入 NULL
 * @return fsp_err_t  FSP_SUCCESS 表示诊断通过
 */
static fsp_err_t sdram_diagnose_impl(sdram_test_result_t *p_result)
{
    fsp_err_t err = sdram_check_range(SDRAM_TEST_START_ADDRESS, SDRAM_DEFAULT_DIAG_WORDS);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    sdram_clear_result(p_result);

    err = sdram_diag_data_bus(SDRAM_TEST_START_ADDRESS, p_result);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = sdram_diag_address_bus(SDRAM_TEST_START_ADDRESS, SDRAM_DEFAULT_DIAG_WORDS, p_result);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return FSP_SUCCESS;
}

/**
 * @brief  获取 SDRAM 起始地址
 *
 * @return uint32_t  SDRAM 起始地址
 */
static uint32_t sdram_base_address_impl(void)
{
    return SDRAM_BASE_ADDRESS;
}

/**
 * @brief  获取 SDRAM 总容量
 *
 * @return uint32_t  SDRAM 总容量，单位 byte
 */
static uint32_t sdram_size_bytes_impl(void)
{
    return SDRAM_TOTAL_SIZE_BYTES;
}

/**
 * @brief  获取避开 LCD/ZI 后的用户区起始地址
 *
 * @return uint32_t  用户区起始地址
 */
static uint32_t sdram_user_start_address_impl(void)
{
    return SDRAM_USER_START_ADDRESS;
}

/**
 * @brief  获取避开 LCD/ZI 后的用户区大小
 *
 * @return uint32_t  用户区大小，单位 byte
 */
static uint32_t sdram_user_size_bytes_impl(void)
{
    return SDRAM_USER_SIZE_BYTES;
}

const sdram_api_t sdram =
{
    .test = sdram_test_impl,
    .test_range = sdram_test_range_impl,
    .diagnose = sdram_diagnose_impl,
    .base_address = sdram_base_address_impl,
    .size_bytes = sdram_size_bytes_impl,
    .user_start_address = sdram_user_start_address_impl,
    .user_size_bytes = sdram_user_size_bytes_impl,
};
