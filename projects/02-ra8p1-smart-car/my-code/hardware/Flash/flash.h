#ifndef HARDWARE_FLASH_FLASH_H_
#define HARDWARE_FLASH_FLASH_H_

#include "hal_data.h"

#include <stdint.h>

#define FLASH_BASE_ADDRESS              (0x80000000UL)
#define FLASH_TOTAL_SIZE_BYTES          (0x02000000UL)
#define FLASH_SECTOR_SIZE_BYTES         (4096UL)
#define FLASH_BLOCK_SIZE_BYTES          (65536UL)
#define FLASH_TEST_ADDRESS              (0x01FF0000UL)
#define FLASH_TEST_RESERVED_SIZE_BYTES  (FLASH_BLOCK_SIZE_BYTES)
#define FLASH_WRITE_CHUNK_BYTES         (64UL)
#define FLASH_WRITE_ALIGN_BYTES         (8UL)

#define FLASH_JEDEC_MANUFACTURER_ID     (0xEFU)
#define FLASH_JEDEC_MEMORY_TYPE         (0x40U)
#define FLASH_JEDEC_CAPACITY            (0x19U)

typedef struct st_flash_id
{
    uint8_t manufacturer_id;
    uint8_t memory_type;
    uint8_t capacity;
} flash_id_t;

typedef struct st_flash_test_result
{
    const char *name;
    uint32_t address;
    uint32_t offset;
    uint8_t expected;
    uint8_t actual;
    fsp_err_t err;
} flash_test_result_t;

typedef struct st_flash_api
{
    /**
     * @brief  初始化 OSPI Flash 并读取 JEDEC ID
     *
     * @return fsp_err_t  FSP_SUCCESS 表示 Flash 驱动可用
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  读取 JEDEC ID
     *
     * @param p_id  JEDEC ID 输出结构体
     * @return fsp_err_t  FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*read_jedec_id)(flash_id_t *p_id);

    /**
     * @brief  从 Flash 内存映射区读取数据
     *
     * @param address  Flash 内偏移地址
     * @param p_dest   读取缓冲区
     * @param length   读取长度，单位 byte
     * @return fsp_err_t  FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*read)(uint32_t address, uint8_t *p_dest, uint32_t length);

    /**
     * @brief  擦除 4KB 扇区
     *
     * @param address  Flash 内偏移地址，必须 4KB 对齐
     * @return fsp_err_t  FSP_SUCCESS 表示擦除完成
     */
    fsp_err_t (*erase_sector)(uint32_t address);

    /**
     * @brief  擦除 64KB 块
     *
     * @param address  Flash 内偏移地址，必须 64KB 对齐
     * @return fsp_err_t  FSP_SUCCESS 表示擦除完成
     */
    fsp_err_t (*erase_block)(uint32_t address);

    /**
     * @brief  写入已擦除的 Flash 区域
     *
     * @param address  Flash 内偏移地址，必须 8 字节对齐
     * @param p_src    写入数据，必须 8 字节对齐
     * @param length   写入长度，必须 8 字节对齐
     * @return fsp_err_t  FSP_SUCCESS 表示写入完成
     */
    fsp_err_t (*write)(uint32_t address, const uint8_t *p_src, uint32_t length);

    /**
     * @brief  执行指定地址的擦写读自检
     *
     * @param address   Flash 内偏移地址，必须 4KB 对齐
     * @param p_result  失败时保存失败阶段和数据，可传入 NULL
     * @return fsp_err_t  FSP_SUCCESS 表示自检通过
     */
    fsp_err_t (*test)(uint32_t address, flash_test_result_t *p_result);
} flash_api_t;

extern const flash_api_t flash;

#endif /* HARDWARE_FLASH_FLASH_H_ */
