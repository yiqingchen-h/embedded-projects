#ifndef FOC_BOARD_FOC_H
#define FOC_BOARD_FOC_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "foc_drive.h"
#include "encoder_foc.h"

/** @brief 查询FOC是否占用功率级及ADC；仅对齐标志读取。 @return 正在运行。 */
bool board_foc_active(void);
/** @brief 开始等占空比零电压矢量时序测试，不运行电流环或SMO。
 * @param duty 千分比50..950，三相相同。 @param duration_ms 20..2000ms。
 * @return 校准、配置和启动成功；仅可停机调用。 */
bool board_foc_bench_start(uint32_t duty, uint32_t duration_ms);
/** @brief 禁止MOS输出时连续同步采样SOA/SOB/SOC，结束自动回收。
 * @param duration_ms 20..500ms。@return 静止、驱动器COAST及ADC配置通过且已启动。 */
bool board_foc_zero_start(uint32_t duration_ms);
/** @brief 仅停机平移正式FOC采样触发，不改变PI/SMO及保存参数。
 * @param trigger 兼容10kHz坐标8300..8470，基准8400；正式20kHz实际CCR4为trigger-4250。
 * @return 已设置本轮位置。 */
bool board_foc_test_trigger(uint32_t trigger);
/** @brief 查询兼容10kHz坐标的会话采样位置；实际CCR4由MOTOR FOC报告。
 * @return 兼容坐标tick，复位默认8400。 */
uint32_t board_foc_test_trigger_ticks(void);
/** @brief 停机设置独立FOC启动接管速度，不写Flash。
 * @param rpm 正转机械速度1..1000RPM。 @return 功率关闭、参数合法且已应用。 */
bool board_foc_test_handoff(uint32_t rpm);
/** @brief 查询会话接管速度。 @return 机械RPM，上电按本电机档约50Hz计算，G2208为429。 */
uint32_t board_foc_test_handoff_rpm(void);
/** @brief 有界静态矢量相序/极性测试，独立于正常START；仅停机调用。
 * @param sector 固定矢量1..6。 @param spread 比较值偏移1..200 tick。
 * @return 20ms/0.5A保护测试已启动。 */
bool board_foc_vector_start(uint32_t sector, uint32_t spread);
/** @brief 500ms/0.5A高占空比位置扫描，仅停机调用。
 * @param trigger 上数侧CCR4，8200..8490。 @return 采样测试已启动。 */
bool board_foc_sample_start(uint32_t trigger);
/** @brief 在六种固定排序的8075上边界扫描采样位置。
 * @param trigger 上数侧CCR4。 @param sector 1..6。 @return 500ms有界扫描启动成功。 */
bool board_foc_sample_vector_start(uint32_t trigger,uint32_t sector);
/** @brief 每31周期切换正负小矢量，校验实际提交与采样区间电压；仅停机调用。
 * @param spread 比较值偏移1..200 tick。 @param duration_ms 20..500ms。
 * @return 已启动，沿用0.5A异常关断。 */
bool board_foc_switch_start(uint32_t spread, uint32_t duration_ms);
/** @brief 低电流dq闭环阶跃，参考电角固定30度；仅停机调用。
 * @param q_axis true为Iq阶跃，false为Id阶跃。 @param target_ma 50..300mA。
 * @param duration_ms 20..500ms。 @return 已启动，沿用0.5A异常关断。 */
bool board_foc_current_start(bool q_axis, uint32_t target_ma, uint32_t duration_ms);
/** @brief 0.2A定位200ms，再以0.3A/20Hz每秒强拖；固定时限且不接管。
 * @param frequency_millihz 1000..50000mHz，机械RPM为电频率Hz乘60再除以本档极对数。
 * @param duration_ms 500..5000ms。 @return 已启动，仍以0.5A异常关断。 */
bool board_foc_if_start(uint32_t frequency_millihz, uint32_t duration_ms);
/** @brief 有界I/f期间独立观察SMO；不切换角度或启用速度PI。
 * @param frequency_millihz 目标电频率mHz。 @param duration_ms 时限ms。 @return 启动成功。 */
bool board_foc_observe_start(uint32_t frequency_millihz,uint32_t duration_ms);
/** @brief 主循环报告独立估计及真实反电动势噪声。
 * @param out 响应缓冲。 @param size 容量。 */
void board_foc_observer_diagnostics(char *out,size_t size);
/** @brief 有界低电流完整接管测试；目标服从本档速度上限，时限500..32000ms。
 * @param target_rpm 机械目标。 @param duration_ms 时限。 @return 启动成功。 */
bool board_foc_closed_start(uint32_t target_rpm,uint32_t duration_ms);
/** @brief 独立FOC速度模式正式入口，使用本电机档模型和分级量程；新档模型待实测。
 * @param config 独立参数，噪声由本次PWM校准实测替换。 @param duration_ms 有界时限。
 * @return 静止、校准和初始化全部通过，复位不自动运行。 */
bool board_foc_speed_start(const foc_drive_config_t *config,uint32_t duration_ms);
/** @brief 以已标定绝对角直接启动FOC，不运行六步/强拖/SMO。
 * @param config 方向/零点及独立FOC参数，含可选的独立有符号Iq模式。 @param duration_ms 用户设置的到时停机时限，至少1000ms。
 * @return 静止、校准、编码器DMA及FOC配置全部通过。 */
bool board_foc_encoder_start(const encoder_foc_config_t *config,uint32_t duration_ms);
/** @brief 仅在正式速度闭环更新目标，不给SMO任何目标输入。
 * @param rpm 1..本档速度上限，G2208为1800。 @return 当前已接管并接受新目标。 */
bool board_foc_speed_target(uint32_t rpm);
/** @brief 仅正式ENCFOC/CURRENT实际提交后在线更新Iq，不改变限流和速度模式参数。
 * @param current_ma -500..500mA，0表示零电流闭环，保持功率启用；STOP仍立即撤销输出。
 * @return 电流模式正在运行且接受；切换模式必须先STOP并重新满足静止/标定资格。 */
bool board_foc_iq_target(int32_t current_ma);
/** @brief 在实际位置闭环中更新有符号毫度目标；不重置机械零点和轨迹速度。 */
bool board_foc_position_target(int32_t target_mdeg);
/** @brief 读取原始多圈坐标及位置控制快照，换算和格式化只在主循环执行。 */
void board_foc_position(char *out,size_t size,int32_t target_mdeg,int32_t tolerance_mdeg);
/** @brief 在主循环原子发布四项手感配置，不改动ISR动态状态。
 * @param config 完整有效配置。 @return 当前手感运行允许更新。 */
bool board_foc_haptic_configure(const encoder_haptic_config_t *config);
/** @brief 读取同一控制拍的真实机械角、计数、Iq和配置；不改变原BIN1。
 * @param out 响应。 @param size 容量。 @param config RAM中已确认配置。
 * @param current_limit_ma 独立限流mA。 @param run_ms 独立时限ms。 */
void board_foc_haptic(char *out,size_t size,const encoder_haptic_config_t *config,
    int32_t current_limit_ma,int32_t run_ms);
/** @brief 查询当前运行是否正式FOC入口，包含速度和独立Iq，供主循环发布快照。 @return 正式入口运行中。 */
bool board_foc_speed_active(void);
/** @brief 正式FOC接管并退去Id后设置本轮固定Iq，不写Flash。
 * @param current_ma 0恢复速度PI，1..500为固定Iq的mA值。
 * @return 当前运行和实际电流限幅允许该设置。 */
bool board_foc_hold_iq(uint32_t current_ma);
/** @brief 停机或正式CLOSED时设置谐波支路会话请求，不写Flash、不修改PI。
 * @param enabled 五次及随附七次启用请求；实际支路等待Id退出。
 * @return 当前状态允许切换；七次编译期开关只供独立对照。 */
bool board_foc_harmonic(bool enabled);
/** @brief 查询请求/实际启用状态及有界积分电压，格式化只在主循环。
 * @param out 输出缓冲。 @param size 缓冲容量。 */
void board_foc_harmonic_diagnostics(char *out,size_t size);

/** @brief 设置负序二次会话开关，停止或正式CLOSED时接受。
 * @param enabled 是否请求启用，默认true。 @return 状态允许且已更新。
 * @note 主循环调用，不写Flash，不修改五次请求或原PI。 */
bool board_foc_harmonic2(bool enabled);

/** @brief 查询二次请求、实际状态、学习电压和触限计数。
 * @param out 主循环响应缓冲。 @param size 缓冲字节数。 */
void board_foc_harmonic2_diagnostics(char *out,size_t size);
/* 仅主循环按需读取显示快照，ADC不为交互复制或排队。 */
typedef struct {
    bool tracking,closed,samples_valid;
    /* 本轮角度来自MT6835时为true，机械角才具有有效含义。 */
    bool encoder;
    /* 独立Iq使用有符号mA目标，不能将其当作TARGET_RPM。 */
    bool current_mode;
    int32_t target_iq_ma;
    int32_t mechanical_angle_decideg;
    foc_drive_stage_t stage;
    uint32_t target_rpm;
    int32_t estimated_rpm,forced_millihz,relative_angle_decideg;
    int32_t phase_ma[3],id_ma,iq_ma,ud_mv,uq_mv;
    uint16_t bus_mv;
} board_foc_runtime_t;
/** @brief 主循环读取实时控制拥有的最新值，换算在退出短临界区后完成。
 * @param runtime 输出；停机时有效标志全部清除。 */
void board_foc_runtime(board_foc_runtime_t *runtime);
/** @brief 静止资格后短时分级量程验收，900ms自动滑行关断。
 * @param angle_slot 1..6对应0/60/120/180/240/300电角度。
 * @param target_ma 分档500/1000/1500/2000mA并受本档限流上限约束，G2208仅500mA。 @return 量程及配置有效且启动成功。 */
bool board_foc_range_start(uint32_t angle_slot,uint32_t target_ma);
/** @brief 读取实际闭环及速度PI历史，禁止ISR格式化。
 * @param out 缓冲。 @param size 容量。 */
void board_foc_drive_diagnostics(char *out,size_t size);
/** @brief 主循环读取反电动势模型误差界与有效裕量，不增加ADC诊断历史。
 * @param out 缓冲。 @param size 容量。 */
void board_foc_confidence_diagnostics(char *out,size_t size);
/** @brief 主循环读取真实dq电流环状态及候选模型，不表示已开启速度模式。
 * @param out 输出。 @param size 容量。 */
void board_foc_current_diagnostics(char *out, size_t size);
/** @brief 通用关断入口先撤销FOC所有未来输出；ISR可调用。
 * @param reason NULL为正常停止，其他为故障。 */
void board_foc_abort(const char *reason);
/** @brief 主循环回收测试外设，恢复原六步时序，不在ISR等待。 */
void board_foc_service(void);
/** @brief 双ADC事件处理，只有两路JEOS均完成后读配对样本。 */
void board_foc_adc_irq(void);
/** @brief 谷底UEV确认PWM已经生效，推进电压历史及1kHz检查。 */
void board_foc_update_irq(void);
/** @brief 完成慢速采样DMA并发布母线与VDDA。 */
void board_foc_dma_irq(void);
/** @brief 输出停机保留的时序/采样/电压历史诊断，不在ISR格式化。
 * @param out 输出。 @param size 容量。 @param detail true为电压历史页。 */
void board_foc_diagnostics(char *out, size_t size, bool detail);
/** @brief 主循环读取最新配对电流，不保留诊断样本历史。
 * @param out 含CRLF的电流行。 @param size 容量。
 * @param firewater true输出FOC:IA,IB,IC,ISC（mA），false输出原DATA FOC文本。
 * @return 运行且已有未打印的完整样本并成功格式化；容量不足不标记样本已打印。 */
bool board_foc_current(char *out, size_t size, bool firewater);
/** @brief 主循环原子读取同一最新样本的电流、指令、估计角及PWM请求。
 * @param out 含CRLF的FOC_TRACE CSV行。 @param size 容量。
 * @return 活跃且有新样本并完整格式化；不保留MCU诊断历史。 */
bool board_foc_trace(char *out,size_t size);
/** @brief 读取六个50Hz原始机械角/真实时间点，运行和停机均可，只读且不清缓存。
 * @param out 固定协议响应缓冲。 @param size 容量。 @param first 请求的点序号。
 * @note 若读者落后循环缓存，FIRST报告实际最早可读点，不能冒充连续记录。 */
void board_foc_motion(char *out,size_t size,uint32_t first);
#endif
