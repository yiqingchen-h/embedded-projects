#ifndef FOC_BOARD_MOTOR_H
#define FOC_BOARD_MOTOR_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* 下周期高侧 PWM 相、常导通低侧相和占空比 tick。 */
    uint8_t high, low;
    uint16_t duty_ticks;
    /* false 表示本周期无有效导通窗口，例如限流跳脉冲。 */
    bool pulse;
} board_motor_output_t;
typedef struct {
    /* 当前浮空相 ADC 原始值。 */
    uint16_t raw;
    /* 本次采样的浮空相和实际扇区，均从 0 编号。 */
    uint8_t floating, sector;
    /* 导通窗口内已满足最短脉宽；换相后的退磁由ADC回调按真实COM消隐。 */
    bool valid;
    /* 每个 BEMF 转换递增，跳脉冲样本也递增。 */
    uint32_t sequence;
    /* 本次运行相对时间，单位 us；采样时刻按硬件触发与保持时间计算。 */
    uint32_t sample_us, committed_us, next_commit_us, now_us;
} board_motor_observation_t;
typedef struct {
    /* 有效零点、ADC 参考和慢速采样值。 */
    uint16_t zero[3], vdda_mv, bus_mv, phase_mv[3], c_raw;
    /* 无感模式三相为各自最近有效导通样本，不表示同时三相电压。 */
    bool phase_valid[3];
    /* 无感相电压各自的运行相对采样时刻，单位 us。 */
    uint32_t phase_sample_us[3];
    /* 内核修订版与诊断统计。 */
    uint16_t revision;
    /* 最近一次配置完成后的真实寄存器 2..6 回读值。 */
    uint16_t driver_registers[5];
    /* 回读通过仅表示配置一致，不代替功率波形验证。 */
    bool driver_verified;
    uint32_t fast_count, regular_count, regular_skipped, irq_max_cycles, slow_ms;
    /* 仅停机主循环更新的静止扫描结果；不增加ADC中断诊断记录。 */
    uint32_t rest_scans, rest_line_ptp_mv, rest_line_raw_ptp_mv;
    /* 本次RUN的TIM更新/有效峰顶次数；最近运行检查的数据年龄，停机后保留。 */
    uint32_t update_count, peak_count, bus_age_ms;
    /* 分路径最坏周期数及ADC服务延迟；启动清零，尚未实测不表示已满足预算。 */
    uint32_t current_max_cycles, bemf_max_cycles, peak_max_cycles, speed_max_cycles;
    /* 完整TIM执行段含低频PI及更高优先级抢占，便于验证完整周期负载。 */
    uint32_t tim_total_max_cycles;
    /* 真实速度回调次数，PWM整数分频，每20个周期一次。 */
    uint32_t speed_count;
    /* 有感高占空比换相主动全关闭一拍的次数，不表示外部栅极波形已验证。 */
    uint32_t encoder_blank_count;
    uint32_t sample_age_max_us;
    /* 仅有效导通窗口的低侧相电流可作为控制输入。 */
    bool calibrated, running;
    /* 当前选择的无感双 ADC 采样拓扑。 */
    bool bemf_enabled;
    /* 静态文本，ISR 只设置错误码，不格式化日志。 */
    const char *fault;
} board_motor_status_t;
/* 回调仅在 ADC JEOS 中断调用；不得等待、打印或访问 SPI。 */
typedef void (*board_motor_sample_fn)(uint16_t raw, uint8_t low, bool valid, board_motor_output_t *next);
/* 无感峰顶回调独占启动时基；controlled时采用板卡已校验的预约扇区和占空比。 */
typedef void (*board_motor_peak_fn)(bool controlled, uint8_t sector, uint16_t duty, board_motor_output_t *next);

/** @brief 停机设置无感或编码器峰顶回调；开环/STEP仍使用原电流回调。
 * @param callback 不等待总线的启动/输出回调；可推进非阻塞DMA。 */
void board_motor_set_peak_callback(board_motor_peak_fn callback);
/* ADC浮相回调只做轻量边沿检测及事件预约；不格式化、不访问总线。 */
typedef void (*board_motor_bemf_fn)(const board_motor_observation_t *frame);
/* TIM峰顶提交完成后按PWM整数分频调用速度PI，不影响本次COM时刻。 */
typedef void (*board_motor_speed_fn)(void);
/** @brief 停机装入ADC观测和低频速度回调。 @param bemf ADC回调。 @param speed 1kHz速度回调。 */
void board_motor_set_control_callbacks(board_motor_bemf_fn bemf, board_motor_speed_fn speed);
/** @brief 读取运行标志。 @return 正在运行。 */
bool board_motor_running(void);
/** @brief ADC过零事件直接发布下一换相预约；只在事件时检查实际时限及相序。
 * @param sector 下一扇区。 @param deadline_us 最早换相微秒。 @param duty 千分比幅度。
 * @param commit_us 首个满足预约的峰顶。 @return 成功发布。 */
bool board_motor_schedule(uint8_t sector, uint32_t deadline_us, uint16_t duty, uint32_t *commit_us);
/** @brief 速度中断发布PI幅度，下一峰顶采用。 @param duty 千分比。 */
void board_motor_set_control_duty(uint16_t duty);
/** @brief 读取DWT周期数。 @return 内核周期，自然回绕。 */
uint32_t board_motor_cycles(void);
/** @brief 保护速度快照/提交的数个对齐字段，禁止在期间计算PI或复制观测器。
 * @return 原PRIMASK，须交还unlock。 */
uint32_t board_motor_control_lock(void);
/** @brief 恢复短临界区前的中断状态。 @param mask lock返回值。 */
void board_motor_control_unlock(uint32_t mask);

/** @brief 初始化 SPI3/TIM1/ADC1/ADC2，默认原开环采样且功率禁用。 @return 硬件初始化完成。 */
bool board_motor_init(void);
/** @brief 停机且功率禁用时选择无感双 ADC 或原开环采样；切换后必须重新校准。
 * @param enabled true 为 ADC1 浮相电压、ADC2 电流；false 为原 ADC1 链路。
 * @return 配置完成；运行或功率未禁用时拒绝并保持原配置。 */
bool board_motor_set_bemf(bool enabled);
/** @brief FOC停机回收后恢复原六步寄存器配置，转换必须已停止。
 * @return ADC回收成功且功率级禁用。 */
bool board_motor_restore_profile(void);
/** @brief 获取当前无感采样和下一峰顶时间，可在快速采样回调使用。
 * @return 一致快照；停止或开环时 valid=false，时间以本次启动为基准。 */
board_motor_observation_t board_motor_observation(void);
/** @brief 停机配置增益、回读校准零点并实测最新母线/参考电压。 @param shunt_mohm 1 或 50。 @return 初始化、校准及慢速测量成功。 */
bool board_motor_calibrate(uint32_t shunt_mohm);
/** @brief 停机按指定CSA增益完整校准，供FOC低电流量程使用。
 * @param shunt_mohm 1或50mΩ。 @param gain 50mΩ允许10/20，1mΩ仅40。
 * @return 配置回读、零点和母线采集成功，否则功率禁用。
 * @note 原六步入口仍通过board_motor_calibrate采用原增益。 */
bool board_motor_calibrate_gain(uint32_t shunt_mohm,uint8_t gain);
/** @brief 功率禁用时检查ADC扫描时序、完成预热和首峰顶同步，再开启定时Injected与输出。
 * @param output 初始扇区。 @param sample 快速采样回调。
 * @return 已确认峰顶相位并启动；慢速扫描无法容纳时记录ADC TIMING并保持禁用。 */
bool board_motor_start(const board_motor_output_t *output, board_motor_sample_fn sample);
/** @brief 立即拉低 ENABLE、撤销 MOE，不依赖 SPI；允许中断调用。 @param reason NULL 为正常停机，否则记录故障。 */
void board_motor_stop(const char *reason);
/** @brief 停机回收ADC并测量电压；运行检查真实采样超时。 @param now 主循环调度参考，ms；运行超时使用当前HAL时钟。 */
void board_motor_service(uint32_t now);
/** @brief 获取一致快照。 @return 状态副本。 */
board_motor_status_t board_motor_status(void);
/** @brief 查询实际功率禁用且转换已停止，供 Flash 保存。 @return 可保存。 */
bool board_motor_storage_safe(void);
/** @brief 功率禁用时以约1ms、不固定10ms相位的电压扫描拒绝旋转中FOC重启。
 * @return 128个小组内各平均线电压峰峰变化不超过80mV且ADC有效；每组8次软件扫描。
 * 小组间0.9/1.1ms交替等待，完整窗口约160ms；只表示低反电动势。
 * 固定通道偏置不当作旋转；门限以下的极低速运动不能由本条件精确区分。 */
bool board_motor_standstill(void);
/** @brief 按绝对CSA稳定时间计算最短有效脉冲比较值，PWM频率提高时其占空比例随之提高。
 * @param shunt_mohm 实际电阻。 @return 定时器tick；50 mΩ为391、1 mΩ为646。 */
uint16_t board_motor_min_duty(uint32_t shunt_mohm);
/** @brief 千分比占空比转 TIM1 tick。 @param duty 千分比。 @return tick。 */
uint16_t board_motor_duty_ticks(uint32_t duty);
#endif
