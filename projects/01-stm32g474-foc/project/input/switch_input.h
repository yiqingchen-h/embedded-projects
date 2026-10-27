#ifndef FOC_SWITCH_INPUT_H
#define FOC_SWITCH_INPUT_H
#include <stdbool.h>
#include <stdint.h>
/* TM-2024A-1：PC7按压、PC8默认下拨、PC9默认上拨，默认低有效。 */
#ifndef SWITCH_PRESS_ACTIVE_LEVEL
#define SWITCH_PRESS_ACTIVE_LEVEL 0U
#endif
#ifndef SWITCH_UP_ACTIVE_LEVEL
#define SWITCH_UP_ACTIVE_LEVEL 0U
#endif
#ifndef SWITCH_DOWN_ACTIVE_LEVEL
#define SWITCH_DOWN_ACTIVE_LEVEL 0U
#endif
/* 置1交换上下事件；原始电平与有效位仍保持物理引脚顺序。 */
#ifndef SWITCH_SWAP_UP_DOWN
#define SWITCH_SWAP_UP_DOWN 0U
#endif
/* 三路消抖和按压长按时间，单位ms。 */
#define SWITCH_DEBOUNCE_MS 20U
#define SWITCH_LONG_PRESS_MS 800U

typedef enum {
    SWITCH_NONE = 0, /* 无事件。 */
    SWITCH_CLICK, /* 单击释放。 */
    SWITCH_LONG_PRESS, /* 长按一次。 */
    SWITCH_DOWN, /* 下拨一次。 */
    SWITCH_UP /* 上拨一次。 */
} switch_event_t;
typedef struct {
    uint32_t changed_at[3]; /* 三路候选变化时刻，ms。 */
    uint32_t pressed_at; /* 消抖确认按下时刻。 */
    uint32_t invalid_transitions; /* 方向同时有效的冲突次数，协议保留ILLEGAL名称。 */
    uint8_t raw; /* bit0=S/PC7、bit1=D/PC8、bit2=U/PC9原始高电平。 */
    uint8_t stable; /* 同位序归一化有效状态，三路均消抖。 */
    uint8_t candidate; /* 三路候选状态。 */
    bool direction_armed; /* 稳定回中后允许一次方向事件。 */
    bool conflict; /* 防止持续冲突重复计数。 */
    bool press_armed; /* 上电释放后允许按压。 */
    bool pressing; /* 接受的按压正在持续。 */
    bool long_sent; /* 已产生长按。 */
} switch_input_t;
/** @brief 初始化并等待释放/回中，不产生事件。 @param state 状态。 @param raw 原始位。 @param now ms。 */
void switch_input_init(switch_input_t *state, uint8_t raw, uint32_t now);
/** @brief 主循环处理三路消抖、回中和按压，每次最多两个事件。
 * @param state 状态。 @param raw 原始位。 @param now ms。 @param events 至少两个元素。 @return 事件数。 */
uint8_t switch_input_update(switch_input_t *state, uint8_t raw, uint32_t now, switch_event_t events[2]);
#endif
