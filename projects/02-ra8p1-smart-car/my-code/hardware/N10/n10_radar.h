#ifndef HARDWARE_N10_N10_RADAR_H_
#define HARDWARE_N10_N10_RADAR_H_

#include "hal_data.h"

#include <stdbool.h>
#include <stdint.h>

/* N10 每帧点云点数量。 */
#define N10_RADAR_POINT_COUNT (16U)

/* N10 完整雷达容器扇区角度，单位度。 */
#define N10_RADAR_CLOUD_SECTOR_DEG (5U)

/* N10 完整雷达容器扇区数量。 */
#define N10_RADAR_CLOUD_SECTOR_COUNT (360U / N10_RADAR_CLOUD_SECTOR_DEG)

/* N10 完整雷达容器无效距离标记。 */
#define N10_RADAR_CLOUD_INVALID_DISTANCE_MM (0xFFFFU)

typedef struct st_n10_radar_point
{
    /**
     * @brief  点云角度，单位度
     */
    float angle_deg;

    /**
     * @brief  测距值，单位 mm
     */
    uint16_t distance_mm;

    /**
     * @brief  反射强度，范围 0~255
     */
    uint8_t peak;
} n10_radar_point_t;

typedef struct st_n10_radar_scan
{
    /**
     * @brief  成功解析帧序号，从 1 开始递增
     */
    uint32_t sequence;

    /**
     * @brief  码盘时间，单位 us
     */
    uint16_t speed_us;

    /**
     * @brief  雷达转速，单位 RPM
     */
    float rpm;

    /**
     * @brief  本帧起始角度，单位度
     */
    float start_angle_deg;

    /**
     * @brief  本帧结束角度，单位度
     */
    float end_angle_deg;

    /**
     * @brief  本帧 16 个点云数据
     */
    n10_radar_point_t points[N10_RADAR_POINT_COUNT];
} n10_radar_scan_t;

typedef struct st_n10_radar_cloud_sector
{
    /**
     * @brief  当前扇区采样点数量，包含距离为 0 的点
     */
    uint32_t sample_count;

    /**
     * @brief  当前扇区有效距离点数量
     */
    uint32_t valid_count;

    /**
     * @brief  当前扇区距离为 0 的点数量
     */
    uint32_t zero_count;

    /**
     * @brief  当前扇区最近有效距离，单位 mm
     */
    uint16_t min_distance_mm;

    /**
     * @brief  当前扇区最大反射强度
     */
    uint8_t peak_max;
} n10_radar_cloud_sector_t;

typedef struct st_n10_radar_cloud
{
    /**
     * @brief  完整 360° 雷达容器序号，从 1 开始递增
     */
    uint32_t sequence;

    /**
     * @brief  发布该容器时最近一次 N10 原始帧序号
     */
    uint32_t source_scan_sequence;

    /**
     * @brief  发布该容器时最近一次雷达转速，单位 RPM
     */
    float rpm;

    /**
     * @brief  360° 扇区数据，每个扇区覆盖 N10_RADAR_CLOUD_SECTOR_DEG 度
     */
    n10_radar_cloud_sector_t sectors[N10_RADAR_CLOUD_SECTOR_COUNT];
} n10_radar_cloud_t;

typedef struct st_n10_radar_api
{
    /**
     * @brief  初始化 N10 雷达 UART6 接收
     *
     * @return fsp_err_t FSP_SUCCESS 表示 UART6 已打开
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  处理 UART6 接收缓存中的 N10 连续字节流
     *
     * @return fsp_err_t FSP_SUCCESS 表示处理完成
     */
    fsp_err_t (*process)(void);

    /**
     * @brief  读取最近一次解析成功的雷达扫描数据
     *
     * @param p_scan  雷达扫描数据输出
     * @return fsp_err_t FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*latest_read)(n10_radar_scan_t *p_scan);

    /**
     * @brief  读取最近一次发布的完整 360° 雷达容器
     *
     * @param p_cloud  完整雷达容器输出
     * @return fsp_err_t FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*cloud_read)(n10_radar_cloud_t *p_cloud);
} n10_radar_api_t;

extern const n10_radar_api_t n10_radar;

#endif /* HARDWARE_N10_N10_RADAR_H_ */
