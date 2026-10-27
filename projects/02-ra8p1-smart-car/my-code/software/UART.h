#ifndef UART_H_
#define UART_H_

#include "hal_data.h"

#include <stdint.h>

/* 默认调试串口波特率，J-Link OB 虚拟串口需要终端软件使用相同波特率。 */
#define UART_DEFAULT_BAUDRATE          (230400UL)

/* 发送等待超时时间，避免串口异常时永久卡死。 */
#define UART_TX_TIMEOUT_MS             (100U)

/* 默认分包发送大小，后续音频 PCM payload 默认按 512 字节拆分发送。 */
#define UART_WRITE_CHUNK_DEFAULT_BYTES (512UL)

typedef struct st_debug_uart_api
{
    /**
     * @brief  初始化 RASC 生成的 SCI9 UART 实例
     *
     * @param baudrate  串口波特率由 RASC 配置生成，本参数保留用于兼容旧调用
     * @return fsp_err_t  FSP_SUCCESS 表示初始化成功
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  发送 1 个字符
     *
     * @param ch  要发送的字符
     * @return fsp_err_t  FSP_SUCCESS 表示字符已写入发送寄存器
     */
    fsp_err_t (*put_char)(uint8_t ch);

    /**
     * @brief  发送一段数据
     *
     * @param p_data  待发送数据指针
     * @param length  待发送数据长度
     * @return fsp_err_t  FSP_SUCCESS 表示全部数据已写入发送寄存器
     */
    fsp_err_t (*write)(const uint8_t *p_data, uint32_t length);

    /**
     * @brief  分包发送一段数据
     *
     * @param p_data      待发送数据指针
     * @param length      待发送数据长度
     * @param chunk_size  单包长度，0 表示使用 UART_WRITE_CHUNK_DEFAULT_BYTES
     * @return fsp_err_t  FSP_SUCCESS 表示全部分包发送完成
     */
    fsp_err_t (*write_chunks)(const uint8_t *p_data, uint32_t length, uint32_t chunk_size);

    /**
     * @brief  按 MIC_DATA 协议发送一段 PCM16LE 单声道音频
     *
     * @param label           样本标签，只允许字母、数字、下划线，最长 31 字符
     * @param p_pcm           PCM16 采样数据指针
     * @param samples         采样点数量
     * @param sample_rate_hz  采样率，单位 Hz
     * @return fsp_err_t      FSP_SUCCESS 表示协议包发送完成
     */
    fsp_err_t (*write_pcm16_packet)(const char *label,
                                    const int16_t *p_pcm,
                                    uint32_t samples,
                                    uint32_t sample_rate_hz);

    /**
     * @brief  关闭 RASC 生成的 SCI9 UART 实例
     *
     * @return void
     */
    void (*deinit)(void);
} debug_uart_api_t;

extern const debug_uart_api_t uart;

#endif /* UART_H_ */
