#ifndef FOC_BOARD_TEST_H
#define FOC_BOARD_TEST_H
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /* ADC 外设初始化及校准成功。 */
    bool adc;
    /* SPI1 与 SPI3 主机初始化成功，不代表从机应答。 */
    bool spi;
    /* USART3 初始化成功。 */
    bool uart;
} board_test_init_t;
typedef struct
{
    /* 顺序为母线、相 A/B/C、电流 A/B/C、VREFINT。 */
    uint16_t raw[8];
    /* 母线通道首次转换值，仅用于核对长间隔转换勘误，不参与安全判定。 */
    uint16_t bus_first_raw;
    /* A/B/C 相通道首次转换值；raw[1..3] 保存紧接着的复测值。 */
    uint16_t phase_first_raw[3];
    /* A/B/C 相复测后，ADC1 正规序列首位实际选中的通道号。 */
    uint8_t phase_sq1[3];
    /* PA0～PA3 的 GPIO 模式与上下拉寄存器低八位。 */
    uint8_t pa_moder;
    uint8_t pa_pupdr;
    /* 根据出厂 VREFINT 校准值估算的 ADC 参考电压，mV。 */
    uint32_t vref_mv;
} board_adc_t;
typedef struct
{
    /* 拨动累计步数：PC9上拨加一，PC8下拨减一；方向待实物确认。 */
    int32_t nav_steps;
    /* 上拨、下拨的消抖动作次数；持续拨住不重复累计。 */
    uint32_t up_presses, down_presses;
    /* 原始与消抖后的触点状态：bit0上、bit1下、bit2按下，1表示接地有效。 */
    uint8_t switch_raw, switch_state;
    /* 已消抖按下事件次数。 */
    uint32_t presses;
    /* 本次持续按压时间，ms。 */
    uint32_t held_ms;
    /* 编码器 A、B、Z 的边沿次数。 */
    uint32_t encoder_edges[3];
    /* BOOT、nFAULT 事件次数。 */
    uint32_t boot_presses;
    uint32_t fault_edges;
    /* 串口接收字节、错误及缓冲溢出次数。 */
    uint32_t rx_bytes;
    uint32_t rx_errors;
} board_inputs_t;
typedef struct
{
    /* 六步脉冲中导通上管相的ADC原始值。 */
    uint16_t high_raw;
    /* 六步脉冲中导通下管相的ADC原始值。 */
    uint16_t low_raw;
    /* 六步脉冲中悬空相的ADC原始值，仅供观察。 */
    uint16_t floating_raw;
} board_six_step_sample_t;
/** @brief 初始化本板测试外设，不使能功率级。 @return 各外设初始化结果。 */
board_test_init_t board_test_init(void);
/** @brief 读取八通道，任何转换失败不输出新结果。 @param sample 返回数据。 @return 校准参考及转换有效为 true。 */
bool board_adc_read(board_adc_t *sample);
/** @brief DRV8353F 的 SPI3 模式1适配。 @param tx 16位发送字。 @param rx 返回字。 @return 总线事务成功为 true。 */
bool board_drv_transfer(uint16_t tx, uint16_t *rx);
/** @brief MT6835 的 SPI1 模式3适配。 @param tx 发送。 @param rx 接收。 @param count 字节数。 @return 总线事务成功为 true。 */
bool board_enc_transfer(const uint8_t *tx, uint8_t *rx, uint16_t count);
/** @brief 有界 UART 文本输出，不在中断内调用。 @param text 以零结尾的字符串。 */
void board_uart_write(const char *text);
/** @brief 主循环提交最多127字节的非阻塞短响应，由TXE中断发送；忙时不覆盖。 @param text 完整响应。 @return 已接受。 */
bool board_uart_try_write(const char *text);
/** @brief 从接收环形缓冲取一个字节。 @param value 接收存储。 @return 有数据为 true。 */
bool board_uart_get(char *value);
/** @brief 获取一致的输入快照。 @param inputs 输出。 */
void board_inputs_read(board_inputs_t *inputs);
/** @brief 唤醒驱动，保持六输入低；只在已检查电源时调用。 */
void board_driver_wake(void);
/** @brief 唤醒结束后使能故障监控。 @return 当前 nFAULT、PG 均为高且无停机锁存。 */
bool board_driver_arm(void);
/** @brief 设置空载脉冲的2A估算电流停止门限，依赖1mΩ/20倍增益和当前零点。 @param zero 三路零点。 @param vref_mv ADC参考mV。 @return 参数有效为true；该低速检查不替代硬件OCP。 */
bool board_current_guard(const uint16_t zero[3], uint32_t vref_mv);
/** @brief 关断 ENABLE 与定时器输出；中断可调用，停机锁存直到显式重新自检。 */
void board_trip(void);
/** @brief 是否被故障或 STOP 锁存。 @return 锁存为 true。 */
bool board_tripped(void);
/** @brief 只在初始化或显式重新自检时清除软件停机锁存。 */
void board_clear_trip(void);
/** @brief 单路TIM1单脉冲并采样相电压；ENABLE低的检查先做GPIO高低读回，完成后输入回低。 @param gate 0～5：AH/AL/BH/BL/CH/CL。 @param phase_raw 脉冲中相电压ADC值。 @param enabled true要求已唤醒并监控驱动；false要求ENABLE低，仅测试MCU波形。 @return 波形引脚及采样正常且无故障为 true。 */
bool board_gate_pulse(uint8_t gate, uint16_t *phase_raw, bool enabled);
/** @brief 无负载六步序列的一步：异相上、下桥各一个1ms硬件单脉冲，结束即全部关断。 @param step 0～5，对应 AH+BL、AH+CL、BH+CL、BH+AL、CH+AL、CH+BL。 @param sample 脉冲中高、低、悬空相的ADC原始值。 @return 输出读回、电流与定时器结束检查均通过为true；失败锁存关断。 */
bool board_six_step_pulse(uint8_t step, board_six_step_sample_t *sample);
/** @brief 禁用驱动、停止定时器并恢复六输入低，不清除故障锁存；主循环调用。 */
void board_driver_disable(void);
/** @brief 启动固定AH+BL的1秒硬件单脉冲，立即返回；要求已唤醒、武装及解除COAST。 @return 启动成功。 */
bool board_load_start(void);
/** @brief 查询单脉冲是否仍在计数；正常结束由毫秒中断同时拉低ENABLE。 @return 仍在运行。 */
bool board_load_running(void);
/** @brief 初始化独立看门狗，主循环正常完成一次迭代才喂狗。 @return 配置生效为true；超时撤销功率使能，调用者不得继续喂狗。 */
bool board_watchdog_init(void);
/** @brief 主循环喂独立看门狗。 */
void board_watchdog_feed(void);
/** @brief 每毫秒中断调用，采样上/下/按下开关并监测 PG 与 nFAULT。 */
void board_test_tick(void);
/** @brief 由 USART3 中断入口调用，接收及硬件错误处理。 */
void board_uart_irq(void);
/** @brief 由 EXTI 入口调用，记录编码器边沿及即时故障关断。 @param pins EXTI 挂起引脚位图。 */
void board_test_exti(uint32_t pins);
#endif
