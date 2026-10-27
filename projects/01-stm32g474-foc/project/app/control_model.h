#ifndef FOC_CONTROL_MODEL_H
#define FOC_CONTROL_MODEL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* 固定资源容量；所有接口仅在主循环调用，后端应先获取一致快照再发布。 */
#define CONTROL_ID_RESULTS_MAX 8U
#define CONTROL_DATA_TTL_MS 1000U
/* 算法、模式和状态枚举用于显示与请求，不操作硬件。 */
typedef enum { CONTROL_OPEN_6STEP, CONTROL_SENSORLESS_6STEP, CONTROL_ENCODER_6STEP, CONTROL_ENCODER_FOC, CONTROL_SENSORLESS_FOC, CONTROL_ALGORITHM_COUNT } control_algorithm_t;
/* 新模式追加枚举，保持旧SPEED/TORQUE/POSITION的持久化编号不变。 */
typedef enum { CONTROL_SPEED, CONTROL_TORQUE, CONTROL_POSITION, CONTROL_CURRENT, CONTROL_HAPTIC, CONTROL_MODE_COUNT } control_mode_t;
typedef enum { CONTROL_DISABLED, CONTROL_STOPPED, CONTROL_RUNNING, CONTROL_FAULT } control_run_t;
typedef enum { CONTROL_LOCAL, CONTROL_REMOTE } control_source_t;
typedef enum {
    CONTROL_TARGET, CONTROL_SPEED_VALUE, CONTROL_TORQUE_VALUE, CONTROL_POSITION_VALUE,
    CONTROL_BUS_VOLTAGE, CONTROL_IA, CONTROL_IB, CONTROL_IC, CONTROL_ID, CONTROL_IQ,
    CONTROL_UD, CONTROL_UQ, CONTROL_MECH_ANGLE, CONTROL_ELEC_ANGLE, CONTROL_COMM_HZ,
    CONTROL_DUTY, CONTROL_VALUE_COUNT
} control_value_id_t;
typedef enum {
    CONTROL_PARAM_TARGET, CONTROL_PARAM_POLES, CONTROL_PARAM_CURRENT_LIMIT,
    CONTROL_PARAM_SPEED_LIMIT, CONTROL_PARAM_POSITION_MIN, CONTROL_PARAM_POSITION_MAX,
    CONTROL_PARAM_D_KP, CONTROL_PARAM_D_KI, CONTROL_PARAM_Q_KP, CONTROL_PARAM_Q_KI,
    CONTROL_PARAM_SPEED_KP, CONTROL_PARAM_SPEED_KI, CONTROL_PARAM_POSITION_KP,
    CONTROL_PARAM_POSITION_KI, CONTROL_PARAM_POSITION_KD, CONTROL_PARAM_START_HZ,
    CONTROL_PARAM_RAMP, CONTROL_PARAM_DUTY, CONTROL_PARAM_HANDOFF,
    CONTROL_PARAM_COMM_TIMING, CONTROL_PARAM_OFFSET_A, CONTROL_PARAM_OFFSET_B,
    CONTROL_PARAM_OFFSET_C, CONTROL_PARAM_ENCODER_DIR, CONTROL_PARAM_ELEC_ZERO,
    CONTROL_PARAM_RESISTANCE, CONTROL_PARAM_LD, CONTROL_PARAM_LQ, CONTROL_PARAM_FLUX,
    CONTROL_PARAM_SHUNT, CONTROL_PARAM_RATED_VOLTAGE, CONTROL_PARAM_RATED_CURRENT,
    CONTROL_PARAM_RATED_SPEED, CONTROL_PARAM_ALIGN_MS, CONTROL_PARAM_RUN_MS,
    CONTROL_PARAM_COUNT
} control_parameter_id_t;
typedef struct {
    int32_t value; /* 定点值，真实值=value/10^decimals。 */
    uint8_t decimals; /* 0..3，无浮点格式化。 */
    bool valid; /* false显示--。 */
} control_value_t;
typedef struct {
    control_parameter_id_t id; /* 固定参数标识。 */
    char name[16]; /* 大写ASCII短名称，必须零结尾。 */
    char unit[8]; /* 单位文本，必须零结尾。 */
    int32_t value; /* 与范围/步长同一固定小数尺度。 */
    int32_t minimum; /* 后端提供的下限。 */
    int32_t maximum; /* 后端提供的上限。 */
    uint32_t step; /* 后端提供的正步长。 */
    uint8_t decimals; /* 0..3。 */
    bool valid; /* 参数是否有效。 */
    bool editable; /* 是否允许修改。 */
} control_parameter_t;
/* 各操作必须由后端明确声明支持，默认全部关闭。 */
enum {
    CONTROL_CAP_START=1U, CONTROL_CAP_STOP=2U, CONTROL_CAP_PARAMETERS=4U,
    CONTROL_CAP_CALIBRATE=8U, CONTROL_CAP_CLEAR_FAULT=16U, CONTROL_CAP_IDENTIFY=32U,
    CONTROL_CAP_SOURCE=64U, CONTROL_CAP_STEP_TEST=128U
};
typedef struct {
    control_algorithm_t algorithm; /* 数据所属算法，必须匹配当前选择。 */
    control_mode_t mode; /* 数据所属模式，防止切换后旧快照混入。 */
    bool ready; /* 后端是否就绪。 */
    bool encoder_valid; /* 编码器状态是否已采集。 */
    bool encoder_online; /* 磁编码器在线。 */
    bool calibrated; /* 标定是否完成。 */
    bool power_disabled; /* 后端确认的功率级禁用状态。 */
    control_run_t run; /* 运行状态。 */
    control_source_t source; /* 实际控制来源。 */
    uint32_t capabilities; /* CONTROL_CAP位集合。 */
    uint32_t updated_ms; /* 快照采样时间。 */
    control_value_t values[CONTROL_VALUE_COUNT]; /* 运行测量/目标数据。 */
    char startup[20]; /* 六步启动阶段。 */
    char commutation[20]; /* 换相状态。 */
    char fault[24]; /* 当前故障描述。 */
    char driver_fault[24]; /* 驱动器故障描述。 */
    char last_fault[24]; /* 最近一次故障。 */
} control_snapshot_t;
typedef enum {
    CONTROL_ID_UNAVAILABLE, CONTROL_ID_READY, CONTROL_ID_PREPARING, CONTROL_ID_ROTATING,
    CONTROL_ID_PROCESSING, CONTROL_ID_SUCCESS, CONTROL_ID_FAILED, CONTROL_ID_CANCELLED,
    CONTROL_ID_APPLIED
} control_id_stage_t;
typedef struct {
    uint32_t task_id; /* 每次开始/重试不同，防止迟到结果串入新任务。 */
    control_id_stage_t stage; /* 阶段，不由圈数自动推导成功。 */
    uint32_t updated_ms; /* 后端报告时间。 */
    uint32_t turns_milli; /* 已完成圈数，千分之一圈。 */
    uint16_t progress; /* 0..1000，由后端报告。 */
    char direction[16]; /* 后端提供的方向提示。 */
    char reason[24]; /* 失败/取消等原因。 */
    uint8_t result_count; /* 0..8，超限转显式失败。 */
    control_parameter_t results[CONTROL_ID_RESULTS_MAX]; /* 候选值，尚未应用。 */
} control_id_report_t;
typedef enum {
    CONTROL_REQUEST_START, CONTROL_REQUEST_STOP, CONTROL_REQUEST_PARAMETER,
    CONTROL_REQUEST_CALIBRATE, CONTROL_REQUEST_CLEAR_FAULT, CONTROL_REQUEST_SOURCE,
    CONTROL_REQUEST_ID_START, CONTROL_REQUEST_ID_CANCEL, CONTROL_REQUEST_ID_APPLY,
    CONTROL_REQUEST_ID_DISCARD, CONTROL_REQUEST_STEP_TEST
} control_request_kind_t;
typedef struct {
    /* 单扇区索引0..5；整轮测试固定从0（AB）开始。 */
    uint8_t sector;
    /* true为只走一轮六扇区，false为单扇区。 */
    bool cycle;
    /* 整轮反向；单扇区必须为false。 */
    bool reverse;
    /* 每扇区保持时长20..1000 ms；限流期间可能跳过功率脉冲。 */
    uint32_t dwell_ms;
} control_step_test_t;
typedef struct {
    uint32_t sequence; /* 非零请求编号，后端应带回。 */
    uint32_t task_id; /* 自识别请求所属任务。 */
    control_request_kind_t kind; /* 操作类型。 */
    control_parameter_id_t parameter; /* 参数修改的ID。 */
    int32_t value; /* 参数定点值或来源枚举。 */
    control_algorithm_t algorithm; /* 提交时选择的算法。 */
    control_mode_t mode; /* 提交时选择的模式。 */
    /* 独立六步测试配置，不修改保存参数。 */
    control_step_test_t step_test;
} control_request_t;
typedef enum { CONTROL_OK, CONTROL_NOT_READY, CONTROL_BUSY, CONTROL_RANGE, CONTROL_INVALID } control_result_t;
/** @brief 初始化占位后端；无真实测量和可用操作。 */
void control_model_init(void);
/** @brief 发布一致快照，仅主循环。 @param snapshot 数据。 @return 合法并复制为true。 */
bool control_model_publish(const control_snapshot_t *snapshot);
/** @brief 发布一个参数描述。 @param parameter 参数。 @return 合法并复制为true。 */
bool control_model_parameter_publish(const control_parameter_t *parameter);
/** @brief 获取当前快照，只读至下次主循环更新。 @return 内部只读指针。 */
const control_snapshot_t *control_model_snapshot(void);
/** @brief 获取参数。 @param id 参数ID。 @return 描述，非法ID返回NULL。 */
const control_parameter_t *control_model_parameter(control_parameter_id_t id);
/** @brief 获取自识别报告。 @return 只读报告。 */
const control_id_report_t *control_model_identification(void);
/** @brief 获取选择算法。 @return 算法。 */
control_algorithm_t control_model_algorithm(void);
/** @brief 获取选择模式。 @return 模式。 */
control_mode_t control_model_mode(void);
/** @brief 选择算法/模式并清旧目标。 @param algorithm 算法。 @param mode 模式。 @return 结果。 */
control_result_t control_model_select(control_algorithm_t algorithm, control_mode_t mode);
/** @brief 查询自识别是否锁定配置/导航。 @return 未结束或候选待处理为true。 */
bool control_model_id_locked(void);
/** @brief 检查快照过期。 @param now 当前ms。 @return 已就绪但超时为true。 */
bool control_model_stale(uint32_t now);
/** @brief 提交单个操作，不等待；只有一个在途请求。
 * @param kind 操作。 @param parameter 参数ID。 @param value 值。 @param now 当前ms。 @return 接受结果。 */
control_result_t control_model_request(control_request_kind_t kind, control_parameter_id_t parameter, int32_t value, uint32_t now);
/** @brief 排队独立六步测试；要求就绪、未过期、停机并禁用功率及专属能力。
 * @param test 单扇区或整轮配置；不包含保存参数，后端在启动前重新校准。
 * @param now 当前ms。 @return 接受结果；配置为空返回ARGUMENT，范围错误返回RANGE。 */
control_result_t control_model_request_step(const control_step_test_t *test, uint32_t now);
/** @brief 后端取一次请求；取走后仍在途，直到complete。 @param request 输出。 @return 有新请求为true。 */
bool control_model_take_request(control_request_t *request);
/** @brief 后端返回操作结果。 @param sequence 请求编号。 @param success 成功标志。
 * @param reason 失败原因，可为空。 @return 匹配当前请求为true。 */
bool control_model_complete(uint32_t sequence, bool success, const char *reason);
/** @brief 发布识别阶段/候选，拒绝旧任务和取消中的迟到结果。
 * @param report 报告。 @return 接受为true，超限报告转失败并返回false。 */
bool control_model_id_publish(const control_id_report_t *report);
/** @brief 查询在途请求。 @return 无请求时NULL。 */
const control_request_t *control_model_pending(void);
/** @brief 获取状态变化序号，用于立即刷新阶段/结果。 @return 修订号。 */
uint32_t control_model_revision(void);
/** @brief 获取最近请求结果文本。 @return 长期有效短文本。 */
const char *control_model_notice(void);
/** @brief 获取显示名称。 @param algorithm 算法。 @return 短英文名称。 */
const char *control_algorithm_name(control_algorithm_t algorithm);
/** @brief 获取协议枚举文本。 @param algorithm 算法。 @return ASCII名称。 */
const char *control_algorithm_token(control_algorithm_t algorithm);
/** @brief 查询算法是否FOC。 @param algorithm 算法。 @return 是否FOC。 */
bool control_algorithm_foc(control_algorithm_t algorithm);
/** @brief 获取模式文本。 @param mode 模式。 @return 英文名称。 */
const char *control_mode_name(control_mode_t mode);
/** @brief 获取结果文本。 @param result 结果。 @return 协议错误码/OK。 */
const char *control_result_name(control_result_t result);
/** @brief 定点格式化，不引入浮点printf。 @param out 输出。 @param size 容量。
 * @param value 值。 @param decimals 小数位0..3。 @param unit 单位。 */
void control_format(char *out, size_t size, int32_t value, uint8_t decimals, const char *unit);
#endif
