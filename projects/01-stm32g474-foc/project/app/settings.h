#ifndef FOC_APP_SETTINGS_H
#define FOC_APP_SETTINGS_H
#include "control_model.h"
#include "encoder_haptic.h"

/* FOC独立物理参数；速度增益为微安/RPM及微安/(RPM·s)，当前R/L来自G2208图纸，待实测。 */
typedef struct {
    int32_t target_rpm, current_limit_ma, resistance_mohm, inductance_uh, bandwidth_hz;
    int32_t speed_kp_micro, speed_ki_micro, speed_ramp_millirpm_s, run_ms;
    /* V4使用旧V3保留槽保存独立Iq目标mA；速度目标单独保留，不冒充Nm。 */
    int32_t target_iq_ma;
} settings_foc_t;
/* V5追加独立位置参数；机械坐标和目标不持久化。 */
typedef struct {
    int32_t kp_milli, speed_limit_rpm, acceleration_millirpm_s;
    int32_t speed_kp_micro, speed_ki_micro, tolerance_mdeg;
} settings_position_t;
/* V6手感配置与限流/时限独立，不覆盖位置和速度模式的保存值。 */
typedef struct {
    encoder_haptic_config_t config;
    int32_t current_limit_ma, run_ms;
} settings_haptic_t;
/* 固定宽度的持久化载荷；前160字节保持V1/V2布局，运行资格不写入Flash。 */
typedef struct {
    /* 所有已确认的控制参数，尺度由描述表定义。 */
    int32_t parameter[CONTROL_PARAM_COUNT];
    /* 旧显示兼容字段、算法、模式和来源；固定偏移保持，含义由记录版本区分。 */
    uint32_t brightness, animation, algorithm, mode, source;
    settings_foc_t foc;
    settings_position_t position;
    settings_haptic_t haptic;
} settings_data_t;

/** @brief 装载两槽最新有效V1至V6，旧参数前缀保持，缺少的新字段补默认值。 */
void settings_init(void);
/** @brief 读取已提交配置。 @return 主循环只读引用。 */
const settings_data_t *settings_get(void);
/** @brief 按当前算法读取独立参数值；六步原值不受FOC编辑影响。
 * @param id 参数编号。 @return 对应定点参数值，无效编号返回0。 */
int32_t settings_value(control_parameter_id_t id);
/** @brief 发布串口参数描述。 */
void settings_publish(void);
/** @brief 提交参数并排队自动保存。 @param id 参数。 @param value 定点值。 @param now 当前 ms。 @return 合法并已接收。 */
bool settings_set(control_parameter_id_t id, int32_t value, uint32_t now);
/** @brief 收集算法、模式和旧显示兼容值。 @param brightness 1..100，不驱动屏幕。 @param animation 旧动画兼容值。 @param now 当前 ms。 */
void settings_capture(uint32_t brightness, bool animation, uint32_t now);
/** @brief 更新已确认的控制来源。 @param source 来源。 @param now 当前 ms。 */
void settings_source(control_source_t source, uint32_t now);
/** @brief 合并写入，只允许在功率禁用且 ADC 停止时调用保存。 @param safe 停机确认。 @param now 当前 ms。 */
void settings_service(bool safe, uint32_t now);
/** @brief 查询保存状态。 @return DEFAULT / SAVED / PENDING / ERROR。 */
const char *settings_status(void);
/** @brief 机械置零后清除本次上电的位置目标，不写Flash。 */
void settings_position_reset_target(void);
/** @brief 停机设置位置容差100或500毫度，保存到V5配置。 */
bool settings_position_tolerance(int32_t mdeg, uint32_t now);
/** @brief 主循环完整提交手感配置；运行期间只排队，Flash仍由安全窗口保存。
 * @param config 完整四项配置。 @param now 当前ms。 @return 合法且已接收。 */
bool settings_haptic_configure(const encoder_haptic_config_t *config,uint32_t now);
#endif
