#ifndef FOC_CONTROL_SIX_STEP_H
#define FOC_CONTROL_SIX_STEP_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    /* 定位前半段按时间渐增占空比，不推进换相。 */
    SIX_STEP_ALIGN_RAMP = 0,
    /* 定位后半段保持完整占空比，不推进换相。 */
    SIX_STEP_ALIGN_HOLD,
    /* 初始电频率牵引，完成指定次数的相邻扇区转移。 */
    SIX_STEP_PULLIN,
    /* 按既有斜率调整电频率；不重置扇区或相位余数。 */
    SIX_STEP_RAMP,
    /* 当前电频率已到目标，仍由定时积分换相。 */
    SIX_STEP_RUN
} six_step_stage_t;

typedef struct {
    /* 目标电频率、起始频率、斜率，单位分别为 mHz、mHz、mHz/s。 */
    volatile uint32_t target_mhz;
    uint32_t start_mhz, ramp_mhz_s;
    /* 定位与运行时间换算后的控制周期数。 */
    uint32_t align_ticks, run_ticks;
    /* 占空比，千分比；启动前已验证范围。 */
    uint16_t duty;
    /* 初始电频率牵引所需的扇区转移次数；0 绕过牵引。 */
    uint16_t pullin_steps;
} six_step_config_t;

typedef struct {
    /* 启动前验证好的参数副本，ISR 不重复验证。 */
    six_step_config_t config;
    /* 当前运行周期、电频率与斜率余数。 */
    volatile uint32_t ticks, frequency_mhz;
    uint32_t ramp_remainder;
    /* 扇区积分余数；1000 * 控制频率对应一次60°电角换向，整电周需六次。 */
    uint32_t phase;
    /* 当前扇区和有界运行状态。 */
    volatile uint8_t sector;
    bool running;
    /* 当前规划占空比，千分比；有效脉宽与保护由板卡后端处理。 */
    volatile uint16_t duty;
    /* 当前启动阶段；停机保留最后执行的阶段，RUN 不表示转子同步。 */
    volatile six_step_stage_t stage;
    /* 尚需完成的初始电频率扇区转移次数。 */
    uint16_t pullin_remaining;
    /* 定位前半段周期数；奇数总周期时后半段多保持一个周期。 */
    uint32_t align_ramp_ticks;
    /* 启动前算好的每周期整数增幅和余数，避免 ISR 乘除计算线性幅度。 */
    uint16_t duty_step;
    uint32_t duty_remainder, duty_accumulator;
} six_step_t;

/** @brief 从第一扇区启动定位渐增、初频牵引及加速；参数由启动流程验证。
 * @param state 状态，清除上次的扇区、相位及幅度积分。
 * @param config 已验证配置；定位0周期跳过定位，1周期直接满幅保持。 */
void six_step_start(six_step_t *state, const six_step_config_t *config);
/** @brief 推进一个 MOTOR_CONTROL_HZ 周期，达到总时限即停止并保留阶段与幅度。
 * @param state 状态；牵引最后一次转移仍保持初频，下一周期才加速。 */
void six_step_tick(six_step_t *state);
/** @brief 查询扇区中的高侧 PWM 相和低侧导通相。 @param sector 0..5。 @param high 高侧相 0..2。 @param low 低侧相 0..2。 */
void six_step_phases(uint8_t sector, uint8_t *high, uint8_t *low);
/** @brief 判断慢速扫描能否在下次快速触发前完整结束。 @param available 剩余定时器 tick。 @param scan 扫描最坏 tick。 @param launch 启动指令预算 tick。 @return 足够时为 true，边界相等也跳过。 */
bool six_step_regular_fits(uint32_t available, uint32_t scan, uint32_t launch);
/** @brief 用最短有效整脉冲实现低幅指令，可分散脉冲时刻以降低周期性音调。
 * @param budget ISR独占的脉宽余额，启动清零；单位为PWM比较tick。
 * @param noise ISR独占的伪随机状态，启动必须置为非零值。
 * @param requested 本周期所需比较tick。 @param minimum 已验证的最短有效比较tick，必须大于0。
 * @param limited 已触发软限流。 @param spread true分散脉冲，false保持原密度序列。
 * @return 本周期是否发出脉冲；输出宽度由调用者取max(requested,minimum)。
 * 分散模式累计脉宽不超过指令累计值，未用余额小于两倍minimum；限流或零指令立即清零。
 * 本函数不修改PWM频率、采样宽度或限流门限，不含除法、等待及硬件访问。 */
bool six_step_pulse_density(uint32_t *budget, uint32_t *noise, uint16_t requested,
    uint16_t minimum, bool limited, bool spread);
#endif
