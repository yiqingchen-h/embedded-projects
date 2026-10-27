#ifndef HARDWARE_VISION_VISION_UART8_COMM_H_
#define HARDWARE_VISION_VISION_UART8_COMM_H_

#include "hal_data.h"

#include <stdbool.h>
#include <stdint.h>

/* 解析成功后是否按同协议回传 4 个视觉字段，1 开启，0 关闭。 */
#ifndef VISION_UART8_ECHO_BACK_ENABLE
#define VISION_UART8_ECHO_BACK_ENABLE (1U)
#endif

typedef struct st_vision_uart8_frame
{
    /**
     * @brief  视觉返回的第 1 个浮点字段
     */
    float data1;

    /**
     * @brief  视觉返回的第 2 个浮点字段
     */
    float data2;

    /**
     * @brief  视觉返回的第 3 个浮点字段
     */
    float data3;

    /**
     * @brief  视觉返回的第 4 个浮点字段
     */
    float data4;

    /**
     * @brief  是否至少成功解析过一帧有效数据
     */
    uint8_t frame_valid;

    /**
     * @brief  是否存在尚未被上层清除的新数据
     */
    uint8_t new_data_flag;

    /**
     * @brief  最近一帧成功解析时的系统时间，单位 ms
     */
    uint32_t tick_ms;

    /**
     * @brief  成功解析的帧数量
     */
    uint32_t rx_ok_count;

    /**
     * @brief  XOR 校验失败的帧数量
     */
    uint32_t rx_checksum_error_count;

    /**
     * @brief  格式错误或 UART 错误计数
     */
    uint32_t rx_format_error_count;

    /**
     * @brief  RX 环形缓冲溢出计数
     */
    uint32_t rx_overflow_count;

    /**
     * @brief  当前是否存在 UART8 发送中的数据
     */
    uint8_t tx_busy;

    /**
     * @brief  成功发送帧数量
     */
    uint32_t tx_ok_count;

    /**
     * @brief  发送失败或超时计数
     */
    uint32_t tx_error_count;
} vision_uart8_frame_t;

typedef struct st_vision_uart8_api
{
    /**
     * @brief  初始化 UART8 视觉通信模块
     *
     * @return fsp_err_t  FSP_SUCCESS 表示 UART8 已打开
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  处理 UART8 接收缓存中的视觉协议字节流
     *
     * 该接口只负责后台解析和可选回传，建议由 FreeRTOS 后台任务每 4ms 调用一次。
     * 其他任务通过 latest_read/has_new/clear_new 消费数据。
     *
     * @return fsp_err_t  FSP_SUCCESS 表示处理完成
     */
    fsp_err_t (*process)(void);

    /**
     * @brief  读取最近一次缓存的数据和统计信息
     *
     * 该接口只读取快照，不清除 new_data_flag，消费任务读完后按需调用 clear_new。
     *
     * @param p_frame  视觉数据输出指针
     * @return fsp_err_t  FSP_SUCCESS 表示读取成功
     */
    fsp_err_t (*latest_read)(vision_uart8_frame_t *p_frame);

    /**
     * @brief  查询是否存在尚未被上层清除的新数据
     *
     * @return bool  true 表示存在新数据
     */
    bool (*has_new)(void);

    /**
     * @brief  清除新数据标志
     *
     * @return void
     */
    void (*clear_new)(void);

    /**
     * @brief  按视觉协议发送最多 4 个浮点字段
     *
     * @param p_values  待发送浮点数组
     * @param length    数组中有效字段数量
     * @return fsp_err_t  FSP_SUCCESS 表示发送成功
     */
    fsp_err_t (*send4)(const float *p_values, uint8_t length);

    /**
     * @brief  注入一帧测试数据到 UART8 接收缓存
     *
     * @param data1  第 1 个测试字段
     * @param data2  第 2 个测试字段
     * @param data3  第 3 个测试字段
     * @param data4  第 4 个测试字段
     * @return fsp_err_t  FSP_SUCCESS 表示注入成功
     */
    fsp_err_t (*test_inject)(float data1, float data2, float data3, float data4);
} vision_uart8_api_t;

extern const vision_uart8_api_t vision_uart8;

#endif /* HARDWARE_VISION_VISION_UART8_COMM_H_ */
