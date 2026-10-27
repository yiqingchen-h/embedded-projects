#include "Key.h"
#include <stdbool.h>
// void GPIO_Pin_ON(ioport_instance_t GPIO_instance,bsp_io_port_pin_t pin)
//{
//	GPIO_instance.p_api->pinWrite(GPIO_instance.p_ctrl,pin,1);
// }

// void GPIO_Pin_OFF(ioport_instance_t GPIO_instance,bsp_io_port_pin_t pin)
//{
//	GPIO_instance.p_api->pinWrite(GPIO_instance.p_ctrl,pin,0);
// }

// struct Key_flag{
//	bool key1_pressed;
// };

// struct Key_flag Read_Key_or_not = {false};  // 初始化 false

// unsigned char Key_State_Read(void){
//	bsp_io_level_t level = BSP_IO_LEVEL_LOW;  //接收按键状态变量
//	g_ioport.p_api->pinRead(g_ioport.p_ctrl,BSP_IO_PORT_00_PIN_00,&level);	//读取按键状态
//	if(level == BSP_IO_LEVEL_LOW ){		//判断是否按下
//		if(Read_Key_or_not.key1_pressed){		//判断按下后是否有松手的动作
//			return 0;
//		}
//		R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);	//消抖
//		g_ioport.p_api->pinRead(g_ioport.p_ctrl,BSP_IO_PORT_00_PIN_00,&level);	//再次读取按键状态确定按下
//		if(level == BSP_IO_LEVEL_LOW)
//		{
//			Read_Key_or_not.key1_pressed = true;
//			return 1;
//		}
//		else{
//			return 0;
//		}
//	}
//	Read_Key_or_not.key1_pressed = false;
//	return 0;
// }

// --- 编码器分辨率设置 ---
// 2: 半步模式 (推荐)。EC11通常转一格产生2个或4个脉冲。
//    设为2可以保证灵敏度，同时过滤掉极微小的抖动。
// #define ENC_DIVIDER  1

//// --- 状态机定义 ---
// typedef enum {
//     STATE_IDLE = 0,
//     STATE_DEBOUNCE,
//     STATE_PRESSED,
//     STATE_WAIT_DOUBLE,
//     STATE_WAIT_RELEASE
// } key_state_t;

//// --- 按键控制块 ---
// typedef struct {
//     bsp_io_port_pin_t pin;
//     key_id_t id;
//     key_state_t state;
//     uint32_t timer;
//     bool active_low;
// } key_dev_t;

//// --- 全局变量 ---
// static volatile uint32_t g_key_tick = 0;
// static volatile int32_t  g_encoder_raw_pulses = 0; // [修改] 存储原始脉冲数
// static int32_t           g_last_reported_count = 0; // 用于检测变化

//// 按键硬件定义
// static key_dev_t g_keys[KEY_COUNT] = {
//     { .pin = BSP_IO_PORT_00_PIN_01, .id = KEY_ID_1, .state = STATE_IDLE, .active_low = false },
//     { .pin = BSP_IO_PORT_00_PIN_04, .id = KEY_ID_2, .state = STATE_IDLE, .active_low = false },
//     { .pin = BSP_IO_PORT_04_PIN_13, .id = KEY_ID_3, .state = STATE_IDLE, .active_low = false },
//     { .pin = BSP_IO_PORT_04_PIN_12, .id = KEY_ID_4, .state = STATE_IDLE, .active_low = false },
//     { .pin = BSP_IO_PORT_06_PIN_15, .id = KEY_ID_5, .state = STATE_IDLE, .active_low = true  },
// };

//// 事件队列
// static struct {
//     key_event_t buf[KEY_FIFO_SIZE];
//     uint8_t head;
//     uint8_t tail;
// } g_fifo;

//// 编码器状态记录
// static uint8_t g_enc_old_state = 0;

//// --- 内部函数: 推送事件 ---
// static void key_fifo_push(key_id_t id, key_event_type_t type)
//{
//     uint8_t next = (g_fifo.head + 1) % KEY_FIFO_SIZE;
//     if (next != g_fifo.tail) {
//         g_fifo.buf[g_fifo.head].id = id;
//         g_fifo.buf[g_fifo.head].type = type;
//         g_fifo.head = next;
//     }
// }

//// --- 内部函数: 读取按键电平 ---
// static bool key_read_pin(uint8_t index)
//{
//     bsp_io_level_t level;
//     R_IOPORT_PinRead(&g_ioport_ctrl, g_keys[index].pin, &level);
//
//     if (g_keys[index].active_low) {
//         return (level == BSP_IO_LEVEL_LOW);
//     } else {
//         return (level == BSP_IO_LEVEL_HIGH);
//     }
// }

//// --- 内部函数: 编码器扫描 (高灵敏度版) ---
// static void encoder_scan_process(void)
//{
//     bsp_io_level_t level_a, level_b;
//
//     // 1. 读取 A, B 相电平
//     R_IOPORT_PinRead(&g_ioport_ctrl, ENC_PIN_A, &level_a);
//     R_IOPORT_PinRead(&g_ioport_ctrl, ENC_PIN_B, &level_b);
//
//     // 2. 组合成当前状态
//     uint8_t curr_state = (uint8_t)((level_a << 1) | level_b);
//
//     // 3. 状态索引表 (0=无变化, 1=CW, -1=CCW)
//     static const int8_t enc_table[16] = {
//         0, -1,  1,  0,
//         1,  0,  0, -1,
//        -1,  0,  0,  1,
//         0,  1, -1,  0
//     };
//
//     // 4. 查表
//     uint8_t index = (uint8_t)((g_enc_old_state << 2) | curr_state);
//     int8_t delta = enc_table[index & 0x0F];
//
//     // 5. 更新旧状态
//     g_enc_old_state = curr_state;
//
//     // 6. [核心修改] 直接累加原始脉冲，不进行阈值拦截
//     if (delta != 0) {
//         g_encoder_raw_pulses += delta;
//
//         // 计算当前的逻辑计数值 (除以分频系数)
//         int32_t current_logic_count = g_encoder_raw_pulses / ENC_DIVIDER;
//
//         // 只有当逻辑计数值发生变化时，才触发事件
//         if (current_logic_count != g_last_reported_count) {
//             if (current_logic_count > g_last_reported_count) {
//                 key_fifo_push(KEY_ID_5, KEY_EVENT_ROT_CW);
//             } else {
//                 key_fifo_push(KEY_ID_5, KEY_EVENT_ROT_CCW);
//             }
//             g_last_reported_count = current_logic_count;
//         }
//     }
// }

//// ================= 接口实现 =================

// void Key_Init(void)
//{
//     g_fifo.head = 0;
//     g_fifo.tail = 0;
//     g_key_tick = 0;
//     g_encoder_raw_pulses = 0;
//     g_last_reported_count = 0;
//
//     for(int i=0; i<KEY_COUNT; i++) {
//         g_keys[i].state = STATE_IDLE;
//         g_keys[i].timer = 0;
//     }
//
//     // 初始化编码器状态
//     bsp_io_level_t level_a, level_b;
//     R_IOPORT_PinRead(&g_ioport_ctrl, ENC_PIN_A, &level_a);
//     R_IOPORT_PinRead(&g_ioport_ctrl, ENC_PIN_B, &level_b);
//     g_enc_old_state = (uint8_t)((level_a << 1) | level_b);
// }

// void Key_Tick_Handler(void)
//{
//     g_key_tick++;

//}

// bool Key_Get_Event(key_event_t *p_event)
//{
//     if (g_fifo.head == g_fifo.tail) return false;
//     *p_event = g_fifo.buf[g_fifo.tail];
//     g_fifo.tail = (g_fifo.tail + 1) % KEY_FIFO_SIZE;
//     return true;
// }

//// 获取计数值 (返回处理后的逻辑值)
// int32_t Key_Get_Encoder_Count(void)
//{
//     return g_encoder_raw_pulses / ENC_DIVIDER;
// }

//// 设置计数值
// void Key_Set_Encoder_Count(int32_t count)
//{
//     g_encoder_raw_pulses = count * ENC_DIVIDER;
//     g_last_reported_count = count;
// }

// void Key_Scan(void)
//{
//     static uint32_t last_scan_tick = 0;
//
//     // 1. 编码器扫描 (全速运行)
//     encoder_scan_process();

//    // 2. 按键扫描 (5ms 节拍)
//    if ((uint32_t)(g_key_tick - last_scan_tick) < KEY_SCAN_INTERVAL) return;
//
//    uint32_t delta_ms = g_key_tick - last_scan_tick;
//    last_scan_tick = g_key_tick;

//    for (int i = 0; i < KEY_COUNT; i++)
//    {
//        key_dev_t *p_key = &g_keys[i];
//        bool pressed = key_read_pin((uint8_t)i);

//        switch (p_key->state)
//        {
//            case STATE_IDLE:
//                if (pressed) {
//                    p_key->state = STATE_DEBOUNCE;
//                    p_key->timer = 0;
//                }
//                break;

//            case STATE_DEBOUNCE:
//                p_key->timer += delta_ms;
//                if (p_key->timer >= TIME_DEBOUNCE) {
//                    if (pressed) {
//                        p_key->state = STATE_PRESSED;
//                        p_key->timer = 0;
//                    } else {
//                        p_key->state = STATE_IDLE;
//                    }
//                }
//                break;

//            case STATE_PRESSED:
//                if (pressed) {
//                    p_key->timer += delta_ms;
//                    if (p_key->timer >= TIME_LONG_PRESS) {
//                        key_fifo_push(p_key->id, KEY_EVENT_LONG);
//                        p_key->state = STATE_WAIT_RELEASE;
//                    }
//                } else {
//                    p_key->state = STATE_WAIT_DOUBLE;
//                    p_key->timer = 0;
//                }
//                break;

//            case STATE_WAIT_DOUBLE:
//                if (pressed) {
//                    key_fifo_push(p_key->id, KEY_EVENT_DOUBLE);
//                    p_key->state = STATE_WAIT_RELEASE;
//                } else {
//                    p_key->timer += delta_ms;
//                    if (p_key->timer >= TIME_DOUBLE_WAIT) {
//                        key_fifo_push(p_key->id, KEY_EVENT_SHORT);
//                        p_key->state = STATE_IDLE;
//                    }
//                }
//                break;

//            case STATE_WAIT_RELEASE:
//                if (!pressed) {
//                    p_key->state = STATE_IDLE;
//                }
//                break;
//        }
//    }
//}

// ================= 内部类型定义 =================

// 按键状态机状态
typedef enum
{
    STATE_IDLE = 0,    // 空闲
    STATE_DEBOUNCE,    // 消抖中
    STATE_PRESSED,     // 已按下
    STATE_WAIT_DOUBLE, // 等待双击
    STATE_WAIT_RELEASE // 等待释放
} key_state_t;

// 按键设备控制块
typedef struct
{
    bsp_io_port_pin_t pin; // 引脚号
    key_id_t id;           // 按键ID
    key_state_t state;     // 当前状态
    uint32_t timer;        // 计时器
    bool active_low;       // true:低电平有效, false:高电平有效
} key_dev_t;

// ================= 全局变量定义 =================

static volatile uint32_t g_key_tick = 0; // 系统滴答计数

// --- 编码器核心变量 ---
static volatile int32_t g_encoder_raw_pulses = 0; // 原始脉冲累计值
static volatile uint8_t g_enc_state = 0;          // 编码器相位历史状态 (AB)
static volatile int32_t g_last_stable_count = 0;  // 上一次汇报的逻辑计数值

// --- 普通按键硬件定义 ---
static key_dev_t g_keys[KEY_COUNT] = {
    {.pin = BSP_IO_PORT_00_PIN_01, .id = KEY_ID_1, .state = STATE_IDLE, .active_low = false},
    {.pin = BSP_IO_PORT_00_PIN_04, .id = KEY_ID_2, .state = STATE_IDLE, .active_low = false},
    {.pin = BSP_IO_PORT_04_PIN_13, .id = KEY_ID_3, .state = STATE_IDLE, .active_low = false},
    {.pin = BSP_IO_PORT_04_PIN_12, .id = KEY_ID_4, .state = STATE_IDLE, .active_low = false},
    {.pin = BSP_IO_PORT_06_PIN_15, .id = KEY_ID_5, .state = STATE_IDLE, .active_low = true},
};

// --- 事件 FIFO 队列 ---
static struct
{
    key_event_t buf[KEY_FIFO_SIZE];
    uint8_t head;
    uint8_t tail;
} g_fifo;

// --- 编码器状态机查找表 (格雷码解码) ---
// 索引 = (旧状态 << 2) | 新状态
// 输出: 0=无效/抖动, 1=顺时针, -1=逆时针
const int8_t g_enc_table[16] = {
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0};

// ================= 内部函数实现 =================

/**
 * @brief 推送事件到 FIFO 队列
 * @param id 按键ID
 * @param type 事件类型
 * @param value 此时的编码器数值 (快照)
 */
static void key_fifo_push(key_id_t id, key_event_type_t type, int32_t value)
{
    uint8_t next = (g_fifo.head + 1) % KEY_FIFO_SIZE;

    // 如果队列未满
    if (next != g_fifo.tail)
    {
        g_fifo.buf[g_fifo.head].id = id;
        g_fifo.buf[g_fifo.head].type = type;
        g_fifo.buf[g_fifo.head].count = value; // 保存这一刻的数值
        g_fifo.head = next;
    }
    // 如果满了，丢弃新事件(防止溢出覆盖)
}

/**
 * @brief 读取普通按键电平
 */
static bool key_read_pin(uint8_t index)
{
    bsp_io_level_t level;
    R_IOPORT_PinRead(&g_ioport_ctrl, g_keys[index].pin, &level);

    if (g_keys[index].active_low)
    {
        return (level == BSP_IO_LEVEL_LOW);
    }
    else
    {
        return (level == BSP_IO_LEVEL_HIGH);
    }
}

// ================= 中断回调函数 (核心) =================

/**
 * @brief 编码器外部中断回调 (A相和B相共用)
 *        在 FSP 中将两个 IRQ 的 Callback 都设置为此函数
 */
void encoder_irq_callback(external_irq_callback_args_t *p_args)
{
    (void)p_args; // 防止未使用参数警告

    bsp_io_level_t a_level, b_level;

    // 1. 读取当前 A, B 引脚电平
    R_IOPORT_PinRead(&g_ioport_ctrl, ENC_PIN_A, &a_level);
    R_IOPORT_PinRead(&g_ioport_ctrl, ENC_PIN_B, &b_level);

    // 2. 组合新状态 (Bit1: A, Bit0: B)
    uint8_t new_pin_state = (uint8_t)(((a_level == BSP_IO_LEVEL_HIGH) ? 0x02 : 0x00) |
                                      ((b_level == BSP_IO_LEVEL_HIGH) ? 0x01 : 0x00));

    // 3. 组合查表索引: (旧状态 << 2) | 新状态
    uint8_t table_index = (uint8_t)(((g_enc_state & 0x03) << 2) | new_pin_state);

    // 4. 查表获取方向变化
    int8_t direction = g_enc_table[table_index];

    // 5. 如果有有效移动
    if (direction != 0)
    {
        // 应用方向极性宏 (ENC_POLARITY)
        g_encoder_raw_pulses += (direction * ENC_POLARITY);

        // 计算当前的逻辑计数值 (应用分频宏 ENC_PULSES_PER_STEP)
        int32_t current_stable_count = g_encoder_raw_pulses / ENC_PULSES_PER_STEP;

        // 只有当“逻辑值”发生变化时，才推送事件
        // 这样可以过滤掉转动一格过程中的中间脉冲
        if (current_stable_count != g_last_stable_count)
        {
            if (current_stable_count > g_last_stable_count)
            {
                // 推送 CW 事件，并携带当前的数值
                key_fifo_push(KEY_ID_5, KEY_EVENT_ROT_CW, current_stable_count);
            }
            else
            {
                // 推送 CCW 事件，并携带当前的数值
                key_fifo_push(KEY_ID_5, KEY_EVENT_ROT_CCW, current_stable_count);
            }
            // 更新上一次汇报的值
            g_last_stable_count = current_stable_count;
        }
    }

    // 6. 保存当前状态作为下一次的旧状态
    g_enc_state = new_pin_state;
}

// ================= 外部接口实现 =================

void Key_Init(void)
{
    // 初始化变量
    g_fifo.head = 0;
    g_fifo.tail = 0;
    g_key_tick = 0;
    g_encoder_raw_pulses = 0;
    g_last_stable_count = 0;

    for (int i = 0; i < KEY_COUNT; i++)
    {
        g_keys[i].state = STATE_IDLE;
        g_keys[i].timer = 0;
    }

    // ---------------------------------------------------------
    // 【关键修复】手动配置编码器引脚
    // FSP 中如果选了 IRQ 模式，无法勾选 Pull-up。
    // 这里使用底层 API 强制配置为：输入 + 上拉 + IRQ使能。
    // ---------------------------------------------------------

    // 配置 P905 (ENC_PIN_A)
    R_IOPORT_PinCfg(&g_ioport_ctrl, ENC_PIN_A,
                    ((uint32_t)IOPORT_CFG_PORT_DIRECTION_INPUT |
                     (uint32_t)IOPORT_CFG_PULLUP_ENABLE |
                     (uint32_t)IOPORT_CFG_IRQ_ENABLE));

    // 配置 P906 (ENC_PIN_B)
    R_IOPORT_PinCfg(&g_ioport_ctrl, ENC_PIN_B,
                    ((uint32_t)IOPORT_CFG_PORT_DIRECTION_INPUT |
                     (uint32_t)IOPORT_CFG_PULLUP_ENABLE |
                     (uint32_t)IOPORT_CFG_IRQ_ENABLE));

    // 1. 读取编码器初始状态 (防止上电第一下误判)
    bsp_io_level_t a, b;
    R_IOPORT_PinRead(&g_ioport_ctrl, ENC_PIN_A, &a);
    R_IOPORT_PinRead(&g_ioport_ctrl, ENC_PIN_B, &b);
    g_enc_state = (uint8_t)(((a == BSP_IO_LEVEL_HIGH) ? 0x02 : 0x00) |
                            ((b == BSP_IO_LEVEL_HIGH) ? 0x01 : 0x00));

    // 2. 开启外部中断 (使用 FSP 生成的实例)
    g_external_irq_enc_a.p_api->open(g_external_irq_enc_a.p_ctrl, g_external_irq_enc_a.p_cfg);
    g_external_irq_enc_a.p_api->enable(g_external_irq_enc_a.p_ctrl);

    g_external_irq_enc_b.p_api->open(g_external_irq_enc_b.p_ctrl, g_external_irq_enc_b.p_cfg);
    g_external_irq_enc_b.p_api->enable(g_external_irq_enc_b.p_ctrl);
}

void Key_Tick_Handler(void)
{
    g_key_tick++;
}

bool Key_Get_Event(key_event_t *p_event)
{
    if (g_fifo.head == g_fifo.tail)
        return false; // 队列为空

    *p_event = g_fifo.buf[g_fifo.tail]; // 取出事件
    g_fifo.tail = (g_fifo.tail + 1) % KEY_FIFO_SIZE;
    return true;
}

int32_t Key_Get_Encoder_Count(void)
{
    // 返回处理后的逻辑值
    return g_encoder_raw_pulses / ENC_PULSES_PER_STEP;
}

void Key_Set_Encoder_Count(int32_t count)
{
    // 设置时需要乘回去，保持原始脉冲同步
    g_encoder_raw_pulses = count * ENC_PULSES_PER_STEP;
    g_last_stable_count = count;
}

void Key_Scan(void)
{
    static uint32_t last_scan_tick = 0;

    // 编码器由中断处理，这里只处理普通按键
    // 检查时间间隔
    if ((uint32_t)(g_key_tick - last_scan_tick) < KEY_SCAN_INTERVAL)
        return;

    uint32_t delta_ms = g_key_tick - last_scan_tick;
    last_scan_tick = g_key_tick;

    for (int i = 0; i < KEY_COUNT; i++)
    {
        key_dev_t *p_key = &g_keys[i];
        bool pressed = key_read_pin((uint8_t)i);

        switch (p_key->state)
        {
        case STATE_IDLE:
            if (pressed)
            {
                p_key->state = STATE_DEBOUNCE;
                p_key->timer = 0;
            }
            break;

        case STATE_DEBOUNCE:
            p_key->timer += delta_ms;
            if (p_key->timer >= TIME_DEBOUNCE)
            {
                if (pressed)
                {
                    p_key->state = STATE_PRESSED;
                    p_key->timer = 0;
                }
                else
                {
                    p_key->state = STATE_IDLE;
                }
            }
            break;

        case STATE_PRESSED:
            if (pressed)
            {
                p_key->timer += delta_ms;
                if (p_key->timer >= TIME_LONG_PRESS)
                {
                    // 长按事件，计数值传0
                    key_fifo_push(p_key->id, KEY_EVENT_LONG, 0);
                    p_key->state = STATE_WAIT_RELEASE;
                }
            }
            else
            {
                p_key->state = STATE_WAIT_DOUBLE;
                p_key->timer = 0;
            }
            break;

        case STATE_WAIT_DOUBLE:
            if (pressed)
            {
                // 双击事件，计数值传0
                key_fifo_push(p_key->id, KEY_EVENT_DOUBLE, 0);
                p_key->state = STATE_WAIT_RELEASE;
            }
            else
            {
                p_key->timer += delta_ms;
                if (p_key->timer >= TIME_DOUBLE_WAIT)
                {
                    // 单击事件，计数值传0
                    key_fifo_push(p_key->id, KEY_EVENT_SHORT, 0);
                    p_key->state = STATE_IDLE;
                }
            }
            break;

        case STATE_WAIT_RELEASE:
            if (!pressed)
            {
                p_key->state = STATE_IDLE;
            }
            break;
        }
    }
}
