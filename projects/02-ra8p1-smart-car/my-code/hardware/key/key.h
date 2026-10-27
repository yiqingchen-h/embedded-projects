#ifndef HARDWARE_KEY_KEY_H_
#define HARDWARE_KEY_KEY_H_

#include "hal_data.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  按键扫描参数，单位 ms
 */
#define KEY_DEBOUNCE_MS                 (10U)
#define KEY_LONG_PRESS_MS               (700U)
#define KEY_FIFO_SIZE                   (16U)
#define KEY_TIMER_PERIOD_MS             (1U)

/**
 * @brief  旋钮编码器参数，EC11 常见一格对应 4 个有效边沿
 */
#define KEY_ENCODER_PULSES_PER_STEP     (4)

/* 旋钮编码器极性反转开关。置 1 时交换 CW/CCW 方向。 */
#define KEY_ENCODER_POLARITY_REVERSED   (1)

#if KEY_ENCODER_POLARITY_REVERSED
#define KEY_ENCODER_POLARITY            (-1)
#else
#define KEY_ENCODER_POLARITY            (1)
#endif

/**
 * @brief  P409 当前作为旋转编码器按键启用
 */
#define KEY_ENABLE_ENCODER_BUTTON       (1U)

#define KEY1_PIN                        BSP_IO_PORT_01_PIN_06
#define KEY2_PIN                        BSP_IO_PORT_01_PIN_05
#define KEY3_PIN                        BSP_IO_PORT_08_PIN_04
#define KEY4_PIN                        BSP_IO_PORT_08_PIN_00
#define KEY_ENCODER_A_PIN               BSP_IO_PORT_00_PIN_14
#define KEY_ENCODER_B_PIN               BSP_IO_PORT_00_PIN_15
#define KEY_ENCODER_BUTTON_PIN          BSP_IO_PORT_04_PIN_09

typedef enum e_key_id
{
    KEY_ID_NONE = 0,
    KEY_ID_1,
    KEY_ID_2,
    KEY_ID_3,
    KEY_ID_4,
    KEY_ID_ENCODER,
    KEY_ID_ENCODER_BUTTON,
} key_id_t;

typedef enum e_key_event_type
{
    KEY_EVENT_NONE = 0,
    KEY_EVENT_CLICK,
    KEY_EVENT_LONG,
    KEY_EVENT_ROT_CW,
    KEY_EVENT_ROT_CCW,
} key_event_type_t;

/**
 * @brief  获取按键事件类型字符串
 *
 * @param type  按键事件类型
 * @return const char * 事件类型字符串
 */
static inline const char *key_event_type_string(key_event_type_t type)
{
    switch (type)
    {
        case KEY_EVENT_CLICK:
            return "CLICK";

        case KEY_EVENT_LONG:
            return "LONG";

        case KEY_EVENT_ROT_CW:
            return "ROT_CW";

        case KEY_EVENT_ROT_CCW:
            return "ROT_CCW";

        default:
            return "NONE";
    }
}

typedef struct st_key_event
{
    key_id_t id;
    key_event_type_t type;
    /**
     * @brief  CLICK/LONG 事件表示按下持续时间，单位 ms；旋钮事件表示当前编码器计数
     */
    int32_t count;
} key_event_t;

typedef struct st_key_api
{
    /**
     * @brief  初始化按键和旋钮编码器的软件状态
     *
     * @return fsp_err_t  FSP_SUCCESS 表示初始化成功
     */
    fsp_err_t (*init)(void);

    /**
     * @brief  执行一次按键和旋钮编码器扫描
     *
     * @param now_ms  当前系统时间，单位 ms
     * @return void
     */
    void (*process)(uint32_t now_ms);

    /**
     * @brief  从事件队列读取一个按键或编码器事件
     *
     * @param p_event  事件输出指针
     * @return bool    true 表示成功读取到事件
     */
    bool (*get_event)(key_event_t *p_event);

    /**
     * @brief  获取旋钮编码器当前逻辑计数
     *
     * @return int32_t  当前计数
     */
    int32_t (*get_encoder_count)(void);

    /**
     * @brief  设置旋钮编码器当前逻辑计数
     *
     * @param count  目标计数
     * @return void
     */
    void (*set_encoder_count)(int32_t count);
} key_api_t;

extern const key_api_t key;

#endif /* HARDWARE_KEY_KEY_H_ */
