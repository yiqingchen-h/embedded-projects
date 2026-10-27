#ifndef FOC_APP_MOTOR_BACKEND_H
#define FOC_APP_MOTOR_BACKEND_H
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "encoder_haptic.h"
/** @brief 完整更新手感，运行中只写RAM、停机安全窗口自动保存。
 * @param config 四项完整配置。 @return NULL成功，否则拒绝原因。 */
const char *motor_backend_haptic_configure(const encoder_haptic_config_t *config);
/** @brief 查询真实手感状态及已接收配置。 @param out 响应。 @param size 容量。 */
void motor_backend_haptic(char *out,size_t size);
/** @brief 初始化真实六步后端，配置由 settings_init 提前装载。 */
void motor_backend_init(void);
/** @brief REMOTE且选择ENCODER_6STEP时开始七点有界方向/电角零点标定。
 * @return NULL表示已开始；其他为拒绝原因，不自动清故障。 */
const char *motor_backend_encoder_calibrate(void);
/** @brief 停机REMOTE且选择有感六步时设置低幅脉冲分散；默认ON，仅本会话有效。
 * @param enabled true分散脉冲时刻，false使用原密度序列；均保留采样宽度及限流。
 * @return NULL成功，其他为拒绝原因；不启动电机、不写Flash、不撤销标定。 */
const char *motor_backend_encoder_quiet(bool enabled);
/** @brief REMOTE、无待处理请求、功率关闭且轴静止时设置编码器迟滞易失影子。
 * @param code 4..7，退出对照后须恢复已备份原值；不写参数Flash或传感器EEPROM。
 * @return NULL成功，其余为拒绝原因；不启动、不修改零点/方向或标定资格。 */
const char *motor_backend_encoder_hysteresis(uint8_t code);
/** @brief 查询编码器六步、校准有效性及最近实际轴速；不读取SPI。
 * @param out 输出。 @param size 容量。 */
void motor_backend_encoder_diagnostics(char *out, size_t size);
/** @brief 处理模型请求和一致快照，不在 ISR 操作 UI。 @param now 当前 ms。 */
void motor_backend_service(uint32_t now);
/** @brief 输出 ADC/DRV 诊断供上板确认。 @param out 输出。 @param size 容量。 */
void motor_backend_diagnostics(char *out, size_t size);
/** @brief 只读输出最近一次 RUN 的采样/限流/密度统计，停机保留、启动清零，仅主循环调用。
 * @param out 输出；三相 PHASE_SAMPLES 顺序为 A/B/C，PEAK_MA 为有效低侧样本绝对峰值，未采到为 NA。
 * @param size 容量；LOW/RAW/DELTA 为最近有效样本，HZ 是电频率指令，DUTY 为当前幅度指令百分比，STAGE 为指令阶段。 */
void motor_backend_run_diagnostics(char *out, size_t size);
/** @brief 查询最近一次无感运行的观测资格、估速及过零统计，停机保留。
 * @param out 输出；MODE明确区分仅观察构建与闭环实验构建。
 * @param size 容量；估速来自反电动势，未取得资格或过期时输出NA。 */
void motor_backend_bemf_diagnostics(char *out, size_t size);
/** @brief 输出ADC/TIM/速度分段最坏周期数及采样服务延迟。 @param out 输出。 @param size 容量。 */
void motor_backend_timing_diagnostics(char *out, size_t size);
/** @brief 停机读取独立原始/事件诊断快照，不作控制队列。
 * @param out 输出。 @param size 容量。 @param page 从0起。 @param events 选择事件快照。 */
void motor_backend_bemf_trace_diagnostics(char *out, size_t size, unsigned page, bool events);
/** @brief 保留旧诊断命令入口；正常控制构建关闭原始样本折叠，返回CAPTURE DISABLED。
 * @param out 输出。 @param size 容量。 @param sector 0..5。 @param page 0..3。 */
void motor_backend_bemf_sector_diagnostics(char *out, size_t size, unsigned sector, unsigned page);
/** @brief 只读查询最近运行的独立六步测试状态，停止后保留，不表示栅极实测状态。
 * @param out 输出；HIGH/LOW/FLOAT 为指令相，COMPLETED 为已完整保持的扇区数。
 * @param size 容量；DWELL_MS/ELAPSED_MS 单位 ms，DUTY 单位百分比。 */
void motor_backend_step_diagnostics(char *out, size_t size);
/** @brief 输出 Flash 布局和保存失败诊断，只读且仅主循环调用。 @param out 输出。 @param size 容量。 */
void motor_backend_storage_diagnostics(char *out, size_t size);
/** @brief 有界FOC等占空比时序测试；不改变保存设置。
 * @param duty 千分比。 @param duration_ms 运行时限。 @return 已启动。 */
bool motor_backend_foc_bench(uint32_t duty, uint32_t duration_ms);
/** @brief 启动20ms/0.5A保护的静态矢量检查，不改变保存设置。
 * @param sector 1..6。 @param spread 比较值偏移1..200 tick。 @return 已启动。 */
bool motor_backend_foc_vector(uint32_t sector, uint32_t spread);
/** @brief 独立高占空比采样扫描，不修改保存设置。
 * @param trigger CCR4=8200..8490。 @return 500ms/0.5A保护测试已启动。 */
bool motor_backend_foc_sample(uint32_t trigger);
/** @brief 独立正负小矢量切换，不修改保存参数。
 * @param spread 比较值偏移1..200 tick。 @param duration_ms 20..500ms。 @return 已启动。 */
bool motor_backend_foc_switch(uint32_t spread, uint32_t duration_ms);
/** @brief 独立低电流dq闭环阶跃，不修改保存参数。
 * @param q_axis true为Iq、false为Id。 @param target_ma 50..300mA。
 * @param duration_ms 20..500ms。 @return 已启动。 */
bool motor_backend_foc_current_start(bool q_axis, uint32_t target_ma, uint32_t duration_ms);
/** @brief 独立有界低电流I/f，不伪造估速或启用速度闭环。
 * @param frequency_millihz 1000..50000mHz。 @param duration_ms 500..5000ms。 @return 已启动。 */
bool motor_backend_foc_if(uint32_t frequency_millihz, uint32_t duration_ms);
/** @brief 停机REMOTE/SHUNT50条件下进行仅观察I/f测试。
 * @param frequency_millihz 目标电频率mHz。 @param duration_ms 时限ms。 @return 启动成功。 */
bool motor_backend_foc_observe(uint32_t frequency_millihz,uint32_t duration_ms);
/** @brief 仅在停止REMOTE/SHUNT50下开始有界低电流接管试验。
 * @param target_rpm 机械目标。 @param duration_ms 时限。 @return 启动成功。 */
bool motor_backend_foc_closed(uint32_t target_rpm,uint32_t duration_ms);
/** @brief REMOTE停机下分级验收三路正负电流量程。
 * @param angle_slot 固定电角序号1..6。 @param target_ma 分级mA。 @return 已启动。 */
bool motor_backend_foc_range(uint32_t angle_slot,uint32_t target_ma);
/** @brief REMOTE停机下验收六扇区最大CCR边界的采样位置。
 * @param trigger CCR4。 @param sector 1..6。 @return 已启动。 */
bool motor_backend_foc_sample_vector(uint32_t trigger,uint32_t sector);
/** @brief 输出FOC板卡诊断。
 * @param out 输出。 @param size 容量。 @param page -3为电流环、-2为状态、-1为电压。 */
void motor_backend_foc_diagnostics(char *out, size_t size, int page);
/** @brief 空闲主循环读取最新配对电流；不缓存、不等待串口。
 * @param out 含CRLF的电流行。 @param size 容量。
 * @param firewater true输出FOC:IA,IB,IC,ISC（mA），false输出原DATA FOC文本。
 * @return 已生成有效电流行。 */
bool motor_backend_foc_current(char *out, size_t size, bool firewater);
/** @brief 主循环读取基准日志CSV，字段及单位见串口协议。
 * @param out 行缓冲。 @param size 容量。 @return 有新样本且完整生成。 */
bool motor_backend_foc_trace(char *out,size_t size);
/** @brief 主循环分批读取独立机械测量，不影响控制或保护。
 * @param out 响应。 @param size 容量。 @param first 首个请求序号。 */
void motor_backend_foc_motion(char *out,size_t size,uint32_t first);
/** @brief 正式闭环的会话固定Iq试验。@param current_ma 0恢复速度PI。
 * @return 实际运行及限幅允许设置；不改变保存参数。 */
bool motor_backend_foc_hold_iq(uint32_t current_ma);
/** @brief REMOTE正式FOC会话切换负序五次抑制，不改保存参数。
 * @param enabled 是否请求启用。 @return 算法/来源/运行阶段允许设置。 */
bool motor_backend_foc_harmonic(bool enabled);

/** @brief 设置负序二次会话请求，只允许REMOTE且所选无感FOC。
 * @param enabled 是否启用。 @return 板卡阶段允许且已接受。 */
bool motor_backend_foc_harmonic2(bool enabled);
/** @brief REMOTE停机零点试验。@param duration_ms 20..500ms。@return 已启动禁止输出的采样。 */
bool motor_backend_foc_zero(uint32_t duration_ms);
/** @brief REMOTE停机安全窗口内设置采样位置。@param trigger TIM1 tick。@return 已接受。 */
bool motor_backend_foc_trigger(uint32_t trigger);
/** @brief 查询会话采样位置。@return TIM1 tick，未启用FOC时0。 */
uint32_t motor_backend_foc_trigger_ticks(void);
/** @brief 远程停机设置FOC接管速度。 @param rpm 机械RPM，1..1000。 @return 已应用。 */
bool motor_backend_foc_handoff(uint32_t rpm);
/** @brief 查询接管会话参数。 @return 机械RPM。 */
uint32_t motor_backend_foc_handoff_rpm(void);
/** @brief 独立位置快照；只读原始机械坐标，不产生SPI事务。 */
void motor_backend_foc_position(char *out,size_t size);
/** @brief 输出关闭/轴静止/电角标定有效时建立机械零点，目标归零。 */
const char *motor_backend_position_zero(void);
/** @brief 停机设置位置验收容差100或500毫度，不改变电角零点。 */
const char *motor_backend_position_tolerance(int32_t mdeg);
#endif
