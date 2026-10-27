#ifndef SDRAM_H_
#define SDRAM_H_

#include "hal_data.h"

/* SDRAM 映射起始地址，RA8P1 外部 SDRAM 区域从 0x68000000 开始。 */
#define SDRAM_BASE_ADDRESS              (0x68000000UL)

/* W9812G2KB 容量为 128Mbit，即 16MB。 */
#define SDRAM_TOTAL_SIZE_BYTES          (16UL * 1024UL * 1024UL)

/*
 * 预留给 LCD framebuffer、.sdram_noinit、ZI-data 等启动时可能占用的 SDRAM 前段空间。
 * 当前包含 LCD/UI 缓冲和麦克风 SDRAM 采集缓冲，默认预留 6MB；
 * 如果后续增大 MICROPHONE_BUFFER_SECONDS 或显示缓冲，只改这里。
 */
#ifndef SDRAM_RESERVED_BYTES
#define SDRAM_RESERVED_BYTES            (0x001000000UL)
#endif

/* 用户业务可使用的 SDRAM 起始地址。 */
#define SDRAM_USER_START_ADDRESS        (SDRAM_BASE_ADDRESS + SDRAM_RESERVED_BYTES)

/* 用户业务可使用的 SDRAM 大小。 */
#define SDRAM_USER_SIZE_BYTES           (SDRAM_TOTAL_SIZE_BYTES - SDRAM_RESERVED_BYTES)

/* 默认测试地址，使用用户业务区起始地址，避免覆盖 LCD/ZI 区域。 */
#define SDRAM_TEST_START_ADDRESS        (SDRAM_USER_START_ADDRESS)

/* 默认测试 1024 个 32bit word，即 4KB。 */
#define SDRAM_DEFAULT_TEST_WORDS        (1024UL)

/* 默认诊断 1024 个 32bit word，用于检查低地址线混叠。 */
#define SDRAM_DEFAULT_DIAG_WORDS        (1024UL)

/* 放到 SDRAM 并在启动时清零，适合普通大缓存。 */
#define SDRAM_SECTION_ZERO              BSP_PLACE_IN_SECTION(".bss.sdram") BSP_ALIGN_VARIABLE(64)

/* 放到 SDRAM 但启动时不清零，适合 framebuffer 或后续自行初始化的大缓存。 */
#define SDRAM_SECTION_NOINIT            BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".sdram_noinit") BSP_ALIGN_VARIABLE(64)

/* 放到 SDRAM 非缓存区且启动时不清零，适合 DMA/外设直接访问缓存。 */
#define SDRAM_SECTION_NOINIT_NOCACHE    BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".sdram_noinit_nocache") BSP_ALIGN_VARIABLE(64)

typedef struct st_sdram_test_result
{
    const char *name;
    uint32_t address;
    uint32_t expected;
    uint32_t actual;
} sdram_test_result_t;

typedef struct st_sdram_api
{
    /**
     * @brief  测试默认 SDRAM 区域，默认避开 LCD framebuffer 起始区域
     *
     * @param p_result  测试失败时保存失败地址和数据，可传入 NULL
     * @return fsp_err_t  FSP_SUCCESS 表示 SDRAM 读写测试通过
     */
    fsp_err_t (*test)(sdram_test_result_t *p_result);

    /**
     * @brief  测试指定 SDRAM 地址范围
     *
     * @param start_address  测试起始地址，必须 4 字节对齐
     * @param words          测试字数，单位为 32bit word
     * @param p_result       测试失败时保存失败地址和数据，可传入 NULL
     * @return fsp_err_t     FSP_SUCCESS 表示指定范围读写测试通过
     */
    fsp_err_t (*test_range)(uint32_t start_address, uint32_t words, sdram_test_result_t *p_result);

    /**
     * @brief  执行 SDRAM 数据线和地址线诊断
     *
     * @param p_result  诊断失败时保存失败项目和数据，可传入 NULL
     * @return fsp_err_t  FSP_SUCCESS 表示诊断通过
     */
    fsp_err_t (*diagnose)(sdram_test_result_t *p_result);

    /**
     * @brief  获取 SDRAM 起始地址
     *
     * @return uint32_t  SDRAM 起始地址
     */
    uint32_t (*base_address)(void);

    /**
     * @brief  获取 SDRAM 总容量
     *
     * @return uint32_t  SDRAM 总容量，单位 byte
     */
    uint32_t (*size_bytes)(void);

    /**
     * @brief  获取避开 LCD/ZI 后的用户区起始地址
     *
     * @return uint32_t  用户区起始地址
     */
    uint32_t (*user_start_address)(void);

    /**
     * @brief  获取避开 LCD/ZI 后的用户区大小
     *
     * @return uint32_t  用户区大小，单位 byte
     */
    uint32_t (*user_size_bytes)(void);
} sdram_api_t;

extern const sdram_api_t sdram;

#endif /* SDRAM_H_ */
