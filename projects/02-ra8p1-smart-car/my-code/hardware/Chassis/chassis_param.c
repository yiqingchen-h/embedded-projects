#include "hardware/Chassis/chassis_param.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* 底盘 PID 参数记录魔数，ASCII 为 CHPD。 */
#define CHASSIS_PARAM_MAGIC             (0x44504843UL)

/* 底盘 PID 参数记录版本，版本变化后旧参数会自动失效并回到默认值。 */
#define CHASSIS_PARAM_VERSION           (2UL)

/* 参数校验滚动种子。 */
#define CHASSIS_PARAM_CHECKSUM_SEED     (0xA5C35A5CUL)

/* 参数校验滚动增量。 */
#define CHASSIS_PARAM_CHECKSUM_STEP     (0x9E3779B9UL)

typedef struct st_chassis_param_record
{
    /* 记录魔数，用于判断扇区内是否存在有效底盘参数。 */
    uint32_t magic;

    /* 记录版本，用于后续格式升级。 */
    uint32_t version;

    /* 记录长度，单位 byte。 */
    uint32_t length;

    /* 记录校验值，计算时跳过本字段。 */
    uint32_t checksum;

    /* 四轮共用速度环比例系数。 */
    float kp;

    /* 四轮共用速度环积分系数。 */
    float ki;

    /* 保留字段，保证记录长度为 8 字节对齐。 */
    uint32_t reserved[2];
} chassis_param_record_t;

/* Flash 读回记录缓冲区，保持 8 字节对齐便于复用写入校验。 */
static chassis_param_record_t g_chassis_param_read_record BSP_ALIGN_VARIABLE(8);

/* Flash 写入记录缓冲区，满足 flash.write() 的源地址 8 字节对齐要求。 */
static chassis_param_record_t g_chassis_param_write_record BSP_ALIGN_VARIABLE(8);

/**
 * @brief  判断 PID 参数是否在 UI 调试允许范围内
 *
 * @param value  待检查参数
 * @return bool  true 表示参数有效
 */
static bool chassis_param_is_pid_value_valid(float value)
{
    return ((value == value) &&
            (value >= CHASSIS_PID_PARAM_MIN) &&
            (value <= CHASSIS_PID_PARAM_MAX));
}

/**
 * @brief  计算参数记录校验值
 *
 * @param p_record  参数记录指针
 * @return uint32_t  校验值
 */
static uint32_t chassis_param_checksum(const chassis_param_record_t *p_record)
{
    const uint8_t *p_data = (const uint8_t *)p_record;
    uint32_t checksum = CHASSIS_PARAM_CHECKSUM_SEED;
    uint32_t checksum_offset = (uint32_t)offsetof(chassis_param_record_t, checksum);
    uint32_t i = 0U;

    if (NULL == p_record)
    {
        return 0U;
    }

    for (i = 0U; i < (uint32_t)sizeof(chassis_param_record_t); i++)
    {
        if ((i >= checksum_offset) && (i < (checksum_offset + sizeof(uint32_t))))
        {
            continue;
        }

        checksum ^= (uint32_t)p_data[i];
        checksum = (checksum << 5U) | (checksum >> 27U);
        checksum += CHASSIS_PARAM_CHECKSUM_STEP;
    }

    return checksum;
}

/**
 * @brief  校验 Flash 中读出的参数记录
 *
 * @param p_record  参数记录指针
 * @return bool  true 表示记录有效
 */
static bool chassis_param_record_is_valid(const chassis_param_record_t *p_record)
{
    uint32_t checksum = 0U;

    if (NULL == p_record)
    {
        return false;
    }

    if (CHASSIS_PARAM_MAGIC != p_record->magic)
    {
        return false;
    }

    if (CHASSIS_PARAM_VERSION != p_record->version)
    {
        return false;
    }

    if ((uint32_t)sizeof(chassis_param_record_t) != p_record->length)
    {
        return false;
    }

    checksum = chassis_param_checksum(p_record);
    if (checksum != p_record->checksum)
    {
        return false;
    }

    if (!chassis_param_is_pid_value_valid(p_record->kp))
    {
        return false;
    }

    if (!chassis_param_is_pid_value_valid(p_record->ki))
    {
        return false;
    }

    return true;
}

/**
 * @brief  将 PID 参数打包为 Flash 记录
 *
 * @param p_params  PID 参数指针
 * @param p_record  参数记录输出指针
 * @return void
 */
static void chassis_param_pack_record(const chassis_pid_params_t *p_params, chassis_param_record_t *p_record)
{
    if ((NULL == p_params) || (NULL == p_record))
    {
        return;
    }

    (void)memset(p_record, 0, sizeof(chassis_param_record_t));
    p_record->magic = CHASSIS_PARAM_MAGIC;
    p_record->version = CHASSIS_PARAM_VERSION;
    p_record->length = (uint32_t)sizeof(chassis_param_record_t);
    p_record->kp = p_params->kp;
    p_record->ki = p_params->ki;
    p_record->checksum = chassis_param_checksum(p_record);
}

/**
 * @brief  从 Flash 加载底盘 PID 参数
 *
 * @param p_params  参数输出指针
 * @return fsp_err_t  FSP_SUCCESS 表示加载成功
 */
static fsp_err_t chassis_param_load_impl(chassis_pid_params_t *p_params)
{
    fsp_err_t err = FSP_SUCCESS;

    if (NULL == p_params)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    err = flash.init();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = flash.read(CHASSIS_PARAM_FLASH_ADDRESS,
                     (uint8_t *)&g_chassis_param_read_record,
                     (uint32_t)sizeof(g_chassis_param_read_record));
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    if (!chassis_param_record_is_valid(&g_chassis_param_read_record))
    {
        return FSP_ERR_INVALID_DATA;
    }

    p_params->kp = g_chassis_param_read_record.kp;
    p_params->ki = g_chassis_param_read_record.ki;

    return FSP_SUCCESS;
}

/**
 * @brief  保存底盘 PID 参数到 Flash
 *
 * @param p_params  待保存参数
 * @return fsp_err_t  FSP_SUCCESS 表示保存成功
 */
static fsp_err_t chassis_param_save_impl(const chassis_pid_params_t *p_params)
{
    fsp_err_t err = FSP_SUCCESS;

    if (NULL == p_params)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    if ((!chassis_param_is_pid_value_valid(p_params->kp)) ||
        (!chassis_param_is_pid_value_valid(p_params->ki)))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    chassis_param_pack_record(p_params, &g_chassis_param_write_record);

    err = flash.init();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = flash.erase_sector(CHASSIS_PARAM_FLASH_ADDRESS);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = flash.write(CHASSIS_PARAM_FLASH_ADDRESS,
                      (const uint8_t *)&g_chassis_param_write_record,
                      (uint32_t)sizeof(g_chassis_param_write_record));
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = flash.read(CHASSIS_PARAM_FLASH_ADDRESS,
                     (uint8_t *)&g_chassis_param_read_record,
                     (uint32_t)sizeof(g_chassis_param_read_record));
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    if (!chassis_param_record_is_valid(&g_chassis_param_read_record))
    {
        return FSP_ERR_INVALID_DATA;
    }

    return FSP_SUCCESS;
}

const chassis_param_api_t chassis_param =
{
    .load = chassis_param_load_impl,
    .save = chassis_param_save_impl,
};
