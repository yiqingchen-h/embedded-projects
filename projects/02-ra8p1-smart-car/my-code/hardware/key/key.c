#include "hardware/key/key.h"

#include <stddef.h>

/**
 * @brief  普通按键的软件消抖状态
 */
typedef enum e_key_button_state
{
    KEY_BUTTON_IDLE = 0,
    KEY_BUTTON_PRESS_DEBOUNCE,
    KEY_BUTTON_PRESSED,
    KEY_BUTTON_RELEASE_DEBOUNCE,
    KEY_BUTTON_LONG_SENT,
    KEY_BUTTON_LONG_RELEASE_DEBOUNCE,
} key_button_state_t;

/**
 * @brief  单个按键的软件状态
 */
typedef struct st_key_button
{
    bsp_io_port_pin_t pin;
    key_id_t id;
    bool active_high;
    key_button_state_t state;
    uint32_t state_ms;
    uint32_t pressed_ms;
} key_button_t;

/**
 * @brief  按键和编码器事件 FIFO
 */
typedef struct st_key_fifo
{
    key_event_t buffer[KEY_FIFO_SIZE];
    uint8_t head;
    uint8_t tail;
} key_fifo_t;

static key_button_t g_buttons[] =
{
    {KEY1_PIN, KEY_ID_1, true, KEY_BUTTON_IDLE, 0U, 0U},
    {KEY2_PIN, KEY_ID_2, true, KEY_BUTTON_IDLE, 0U, 0U},
    {KEY3_PIN, KEY_ID_3, true, KEY_BUTTON_IDLE, 0U, 0U},
    {KEY4_PIN, KEY_ID_4, true, KEY_BUTTON_IDLE, 0U, 0U},
#if KEY_ENABLE_ENCODER_BUTTON
    {KEY_ENCODER_BUTTON_PIN, KEY_ID_ENCODER_BUTTON, true, KEY_BUTTON_IDLE, 0U, 0U},
#endif
};

static key_fifo_t g_fifo;
static uint8_t g_encoder_state;
static int32_t g_encoder_raw_pulses;
static int32_t g_encoder_report_count;

/**
 * @brief  旋钮编码器状态查表
 *
 * 索引为 (旧 AB 状态 << 2) | 新 AB 状态。
 * 返回 1/-1 表示方向，0 表示无效或无变化。
 */
static const int8_t g_encoder_table[16] =
{
    0,  1, -1,  0,
   -1,  0,  0,  1,
    1,  0,  0, -1,
    0, -1,  1,  0,
};

/**
 * @brief  计算两个毫秒时间点之间的差值
 *
 * @param now_ms    当前毫秒时间
 * @param start_ms  起始毫秒时间
 * @return uint32_t 时间差，允许无符号回绕
 */
static uint32_t key_elapsed_ms(uint32_t now_ms, uint32_t start_ms)
{
    return (uint32_t)(now_ms - start_ms);
}

/**
 * @brief  将一个事件写入 FIFO
 *
 * @param id     事件来源
 * @param type   事件类型
 * @param count  按键事件为持续时间，旋钮事件为编码器计数
 * @return void
 */
static void key_fifo_push(key_id_t id, key_event_type_t type, int32_t count)
{
    uint8_t next = (uint8_t)((g_fifo.head + 1U) % KEY_FIFO_SIZE);

    if (next != g_fifo.tail)
    {
        g_fifo.buffer[g_fifo.head].id = id;
        g_fifo.buffer[g_fifo.head].type = type;
        g_fifo.buffer[g_fifo.head].count = count;
        g_fifo.head = next;
    }
}

/**
 * @brief  读取 GPIO 是否为高电平
 *
 * @param pin  BSP 引脚编号
 * @return bool true 表示高电平
 */
static bool key_pin_is_high(bsp_io_port_pin_t pin)
{
    bsp_io_level_t level = BSP_IO_LEVEL_LOW;

    if (FSP_SUCCESS != R_IOPORT_PinRead(&g_ioport_ctrl, pin, &level))
    {
        return false;
    }

    return (BSP_IO_LEVEL_HIGH == level);
}

/**
 * @brief  判断一个普通按键当前是否处于按下状态
 *
 * @param p_button  按键状态指针
 * @return bool     true 表示按下
 */
static bool key_button_is_pressed(const key_button_t *p_button)
{
    bool is_high = key_pin_is_high(p_button->pin);

    return p_button->active_high ? is_high : !is_high;
}

/**
 * @brief  执行单个普通按键状态机
 *
 * @param p_button  按键状态指针
 * @param now_ms    当前毫秒时间
 * @return void
 */
static void key_button_process(key_button_t *p_button, uint32_t now_ms)
{
    bool pressed = key_button_is_pressed(p_button);

    switch (p_button->state)
    {
        case KEY_BUTTON_IDLE:
            if (pressed)
            {
                p_button->state = KEY_BUTTON_PRESS_DEBOUNCE;
                p_button->state_ms = now_ms;
            }
            break;

        case KEY_BUTTON_PRESS_DEBOUNCE:
            if (!pressed)
            {
                p_button->state = KEY_BUTTON_IDLE;
            }
            else if (key_elapsed_ms(now_ms, p_button->state_ms) >= KEY_DEBOUNCE_MS)
            {
                p_button->state = KEY_BUTTON_PRESSED;
                p_button->pressed_ms = p_button->state_ms;
            }
            break;

        case KEY_BUTTON_PRESSED:
            if (!pressed)
            {
                p_button->state = KEY_BUTTON_RELEASE_DEBOUNCE;
                p_button->state_ms = now_ms;
            }
            else if (key_elapsed_ms(now_ms, p_button->pressed_ms) >= KEY_LONG_PRESS_MS)
            {
                key_fifo_push(p_button->id, KEY_EVENT_LONG, (int32_t)key_elapsed_ms(now_ms, p_button->pressed_ms));
                p_button->state = KEY_BUTTON_LONG_SENT;
            }
            break;

        case KEY_BUTTON_RELEASE_DEBOUNCE:
            if (pressed)
            {
                p_button->state = KEY_BUTTON_PRESSED;
            }
            else if (key_elapsed_ms(now_ms, p_button->state_ms) >= KEY_DEBOUNCE_MS)
            {
                key_fifo_push(p_button->id, KEY_EVENT_CLICK, (int32_t)key_elapsed_ms(now_ms, p_button->pressed_ms));
                p_button->state = KEY_BUTTON_IDLE;
            }
            break;

        case KEY_BUTTON_LONG_SENT:
            if (!pressed)
            {
                p_button->state = KEY_BUTTON_LONG_RELEASE_DEBOUNCE;
                p_button->state_ms = now_ms;
            }
            break;

        case KEY_BUTTON_LONG_RELEASE_DEBOUNCE:
            if (pressed)
            {
                p_button->state = KEY_BUTTON_LONG_SENT;
            }
            else if (key_elapsed_ms(now_ms, p_button->state_ms) >= KEY_DEBOUNCE_MS)
            {
                p_button->state = KEY_BUTTON_IDLE;
            }
            break;

        default:
            p_button->state = KEY_BUTTON_IDLE;
            break;
    }
}

/**
 * @brief  读取旋钮编码器 A/B 两相组合状态
 *
 * @return uint8_t bit1 为 A 相，bit0 为 B 相
 */
static uint8_t key_encoder_read_state(void)
{
    uint8_t state = 0U;

    if (key_pin_is_high(KEY_ENCODER_A_PIN))
    {
        state |= 0x02U;
    }

    if (key_pin_is_high(KEY_ENCODER_B_PIN))
    {
        state |= 0x01U;
    }

    return state;
}

/**
 * @brief  执行一次旋钮编码器状态解码
 *
 * @return void
 */
static void key_encoder_process(void)
{
    uint8_t new_state = key_encoder_read_state();
    uint8_t table_index = (uint8_t)(((g_encoder_state & 0x03U) << 2) | (new_state & 0x03U));
    int8_t delta = g_encoder_table[table_index & 0x0FU];

    g_encoder_state = new_state;

    if (0 != delta)
    {
        g_encoder_raw_pulses += ((int32_t)delta * (int32_t)KEY_ENCODER_POLARITY);

        int32_t current_count = g_encoder_raw_pulses / KEY_ENCODER_PULSES_PER_STEP;
        if (current_count != g_encoder_report_count)
        {
            if (current_count > g_encoder_report_count)
            {
                key_fifo_push(KEY_ID_ENCODER, KEY_EVENT_ROT_CW, current_count);
            }
            else
            {
                key_fifo_push(KEY_ID_ENCODER, KEY_EVENT_ROT_CCW, current_count);
            }

            g_encoder_report_count = current_count;
        }
    }
}

/**
 * @brief  初始化按键模块
 *
 * @return fsp_err_t FSP_SUCCESS 表示初始化完成
 */
static fsp_err_t key_init_impl(void)
{
    g_fifo.head = 0U;
    g_fifo.tail = 0U;
    g_encoder_raw_pulses = 0;
    g_encoder_report_count = 0;

    for (uint32_t i = 0U; i < (sizeof(g_buttons) / sizeof(g_buttons[0])); i++)
    {
        /* 以启动时的稳定电平作为空闲态，适配默认下拉或默认上拉的实际硬件。 */
        g_buttons[i].active_high = !key_pin_is_high(g_buttons[i].pin);
        g_buttons[i].state = KEY_BUTTON_IDLE;
        g_buttons[i].state_ms = 0U;
        g_buttons[i].pressed_ms = 0U;
    }

    g_encoder_state = key_encoder_read_state();

    return FSP_SUCCESS;
}

/**
 * @brief  执行一次按键和编码器扫描
 *
 * @param now_ms  当前毫秒时间
 * @return void
 */
static void key_process_impl(uint32_t now_ms)
{
    key_encoder_process();

    for (uint32_t i = 0U; i < (sizeof(g_buttons) / sizeof(g_buttons[0])); i++)
    {
        key_button_process(&g_buttons[i], now_ms);
    }
}

/**
 * @brief  从 FIFO 读取一个事件
 *
 * @param p_event  事件输出指针
 * @return bool    true 表示读取成功
 */
static bool key_get_event_impl(key_event_t *p_event)
{
    if ((NULL == p_event) || (g_fifo.head == g_fifo.tail))
    {
        return false;
    }

    *p_event = g_fifo.buffer[g_fifo.tail];
    g_fifo.tail = (uint8_t)((g_fifo.tail + 1U) % KEY_FIFO_SIZE);

    return true;
}

/**
 * @brief  获取旋钮编码器当前逻辑计数
 *
 * @return int32_t 当前逻辑计数
 */
static int32_t key_get_encoder_count_impl(void)
{
    return g_encoder_raw_pulses / KEY_ENCODER_PULSES_PER_STEP;
}

/**
 * @brief  设置旋钮编码器当前逻辑计数
 *
 * @param count  目标逻辑计数
 * @return void
 */
static void key_set_encoder_count_impl(int32_t count)
{
    g_encoder_raw_pulses = count * KEY_ENCODER_PULSES_PER_STEP;
    g_encoder_report_count = count;
}

const key_api_t key =
{
    .init = key_init_impl,
    .process = key_process_impl,
    .get_event = key_get_event_impl,
    .get_encoder_count = key_get_encoder_count_impl,
    .set_encoder_count = key_set_encoder_count_impl,
};
