#include "control_model.h"
#include <stdio.h>
#include <string.h>
/* 全部状态仅由主循环访问；不接受ISR直接调用以避免撕裂快照。 */
static control_snapshot_t g_control_snapshot;
static control_parameter_t g_control_parameters[CONTROL_PARAM_COUNT];
static control_id_report_t g_control_id;
static control_request_t g_control_request;
/* 配置选择与在途操作状态。 */
static control_algorithm_t g_control_algorithm;
static control_mode_t g_control_mode;
static bool g_control_pending, g_control_taken, g_control_id_lock;
/* 请求/任务编号不复用；0保留为无编号。 */
static uint32_t g_control_sequence, g_control_task, g_control_revision;
/* 最近操作的可见结果，固定缓冲。 */
static char g_control_notice[24];
/* 占位名称和单位；有效值、范围与步长必须由后端提供。 */
static const char *const g_control_names[CONTROL_PARAM_COUNT] = {
    "TARGET","POLE PAIRS","CURRENT LIMIT","SPEED LIMIT","POSITION MIN","POSITION MAX",
    "D KP","D KI","Q KP","Q KI","SPEED KP","SPEED KI","POSITION KP","POSITION KI",
    "POSITION KD","START HZ","RAMP","DUTY","HANDOFF","COMM TIMING",
    "OFFSET A","OFFSET B","OFFSET C","ENCODER DIR","ELEC ZERO","RESISTANCE","LD","LQ","FLUX",
    "SHUNT","RATED VOLTAGE","RATED CURRENT","RATED SPEED","ALIGN TIME","RUN TIME"
};
static const char *const g_control_units[CONTROL_PARAM_COUNT] = {
    /* D 为占空千分比，D/RPMS 表示 D/(RPM·s)；COMM TIMING 的 DEG 为电角度。 */
    "RPM","","A","RPM","DEG","DEG","","","","","D/RPM","D/RPMS","","","","HZ","","%","RPM","DEG","A","A","A","","DEG","OHM","MH","MH","MWB","MOHM","V","A","RPM","MS","MS"
};
/** @brief 复制有界提示文本。 @param text 提示。 */
static void
notice(const char *text)
{
    snprintf(g_control_notice,sizeof(g_control_notice),"%s",text != NULL ? text : "FAILED");
    ++g_control_revision;
}
/** @brief 检查参数描述契约。 @param p 描述。 @return 合法为true。 */
static bool
parameter_valid(const control_parameter_t *p)
{
    return p != NULL && (unsigned)p->id < CONTROL_PARAM_COUNT && p->decimals <= 3U &&
        memchr(p->name,0,sizeof(p->name)) != NULL && memchr(p->unit,0,sizeof(p->unit)) != NULL &&
        (!p->valid || (p->minimum <= p->value && p->value <= p->maximum)) &&
        (!p->editable || (p->valid && p->step != 0U && p->step <= INT32_MAX));
}
/** @brief 初始化占位后端；无真实测量和可用操作。 */
void
control_model_init(void)
{
    memset(&g_control_snapshot,0,sizeof(g_control_snapshot));
    memset(&g_control_id,0,sizeof(g_control_id));
    memset(g_control_parameters,0,sizeof(g_control_parameters));
    g_control_algorithm=CONTROL_ENCODER_FOC;
    g_control_mode=CONTROL_SPEED;
    g_control_snapshot.algorithm=g_control_algorithm;
    g_control_snapshot.mode=g_control_mode;
    g_control_pending=g_control_taken=g_control_id_lock=false;
    g_control_revision=1U;
    /* 请求与任务编号在重复初始化时仍不倒退。 */
    notice("NOT READY");
    for (unsigned i=0U;i<CONTROL_PARAM_COUNT;++i)
    {
        g_control_parameters[i].id=(control_parameter_id_t)i;
        snprintf(g_control_parameters[i].name,16,"%s",g_control_names[i]);
        snprintf(g_control_parameters[i].unit,8,"%s",g_control_units[i]);
    }
}
/** @brief 发布一致快照，仅主循环。 */
bool
control_model_publish(const control_snapshot_t *s)
{
    if (s == NULL || s->algorithm!=g_control_algorithm || s->mode!=g_control_mode || (unsigned)s->run > CONTROL_FAULT || (unsigned)s->source > CONTROL_REMOTE ||
        memchr(s->startup,0,sizeof(s->startup)) == NULL || memchr(s->commutation,0,sizeof(s->commutation)) == NULL ||
        memchr(s->fault,0,sizeof(s->fault)) == NULL || memchr(s->driver_fault,0,sizeof(s->driver_fault)) == NULL ||
        memchr(s->last_fault,0,sizeof(s->last_fault)) == NULL) { return false; }
    for (unsigned i=0;i<CONTROL_VALUE_COUNT;++i) { if (s->values[i].decimals>3U) { return false; } }
    if (s->ready != g_control_snapshot.ready || s->run != g_control_snapshot.run ||
        s->source != g_control_snapshot.source || s->calibrated != g_control_snapshot.calibrated ||
        s->capabilities != g_control_snapshot.capabilities ||
        strcmp(s->fault,g_control_snapshot.fault) != 0 || strcmp(s->driver_fault,g_control_snapshot.driver_fault) != 0 ||
        strcmp(s->last_fault,g_control_snapshot.last_fault) != 0) { ++g_control_revision; }
    g_control_snapshot=*s;
    if (!g_control_id_lock && !g_control_pending &&
        (g_control_id.stage==CONTROL_ID_READY || g_control_id.stage==CONTROL_ID_UNAVAILABLE))
    {
        g_control_id.stage=(s->ready && (s->capabilities & CONTROL_CAP_IDENTIFY)) ? CONTROL_ID_READY : CONTROL_ID_UNAVAILABLE;
    }
    return true;
}
/** @brief 发布一个参数描述。 */
bool
control_model_parameter_publish(const control_parameter_t *p)
{
    if (!parameter_valid(p) || g_control_id_lock) { return false; }
    g_control_parameters[p->id]=*p;
    ++g_control_revision;
    return true;
}
/** @brief 获取当前快照，只读至下次主循环更新。 */
const control_snapshot_t *
control_model_snapshot(void) { return &g_control_snapshot; }
/** @brief 获取参数。 */
const control_parameter_t *
control_model_parameter(control_parameter_id_t id) { return (unsigned)id < CONTROL_PARAM_COUNT ? &g_control_parameters[id] : NULL; }
/** @brief 获取自识别报告。 */
const control_id_report_t *
control_model_identification(void) { return &g_control_id; }
/** @brief 获取选择算法。 */
control_algorithm_t
control_model_algorithm(void) { return g_control_algorithm; }
/** @brief 获取选择模式。 */
control_mode_t
control_model_mode(void) { return g_control_mode; }
/** @brief 查询自识别是否锁定配置/导航。 */
bool
control_model_id_locked(void) { return g_control_id_lock; }
/** @brief 检查快照过期。 */
bool
control_model_stale(uint32_t now)
{
    return g_control_snapshot.ready && (uint32_t)(now-g_control_snapshot.updated_ms)>CONTROL_DATA_TTL_MS;
}
/** @brief 选择算法/模式并清旧目标。 */
control_result_t
control_model_select(control_algorithm_t algorithm, control_mode_t mode)
{
    if ((unsigned)algorithm>=CONTROL_ALGORITHM_COUNT || (unsigned)mode>=CONTROL_MODE_COUNT ||
        (!control_algorithm_foc(algorithm) && mode != CONTROL_SPEED) ||
        ((mode==CONTROL_CURRENT || mode==CONTROL_POSITION || mode==CONTROL_HAPTIC) && algorithm!=CONTROL_ENCODER_FOC)) { return CONTROL_INVALID; }
    if (g_control_pending || g_control_id_lock || (g_control_snapshot.run != CONTROL_DISABLED &&
        g_control_snapshot.run != CONTROL_STOPPED)) { return CONTROL_BUSY; }
    if (algorithm != g_control_algorithm || mode != g_control_mode)
    {
        g_control_algorithm=algorithm;
        g_control_mode=mode;
        /* 后端必须针对新组合重新发布，不能沿用旧算法的就绪与测量。 */
        g_control_snapshot.algorithm=algorithm;
        g_control_snapshot.mode=mode;
        g_control_snapshot.ready=false;
        g_control_snapshot.capabilities=0;
        g_control_snapshot.calibrated=false;
        g_control_snapshot.encoder_valid=false;
        for(unsigned i=0;i<CONTROL_VALUE_COUNT;i++) { g_control_snapshot.values[i].valid=false; }

        g_control_parameters[CONTROL_PARAM_TARGET].valid=false;
        g_control_parameters[CONTROL_PARAM_TARGET].editable=false;
        g_control_parameters[CONTROL_PARAM_TARGET].value=0;
        g_control_snapshot.values[CONTROL_TARGET].valid=false;
        snprintf(g_control_parameters[CONTROL_PARAM_TARGET].unit,8,"%s",mode==CONTROL_SPEED ? "RPM" : mode==CONTROL_CURRENT ? "A" : mode==CONTROL_TORQUE ? "NM" : "DEG");
        notice("SELECTED");
    }
    return CONTROL_OK;
}
/** @brief 共用互锁校验和请求槽；单扇区或整轮六步测试必须带专用配置。
 * @param kind 操作。 @param parameter 参数ID。 @param value 参数值。
 * @param test 六步测试配置，其余操作为空。 @param now 当前ms。 @return 排队结果。 */
static control_result_t
request(control_request_kind_t kind, control_parameter_id_t parameter, int32_t value,
        const control_step_test_t *test, uint32_t now)
{
    if ((unsigned)kind>CONTROL_REQUEST_STEP_TEST ||
        (kind==CONTROL_REQUEST_STEP_TEST && NULL==test)) { return CONTROL_INVALID; }
    if (kind==CONTROL_REQUEST_STEP_TEST && (test->sector>=6U || test->dwell_ms<20U ||
        test->dwell_ms>1000U || (test->cycle && test->sector!=0U) ||
        (!test->cycle && test->reverse))) { return CONTROL_RANGE; }
    /* STOP 可覆盖尚未执行的启动/参数请求，不能被一行连发的 START 挡住。
     * 模型和后端都在主循环，不存在两个执行器同时消费请求。 */
    if (g_control_pending && kind != CONTROL_REQUEST_STOP) { return CONTROL_BUSY; }
    if (kind == CONTROL_REQUEST_STOP && g_control_pending && !g_control_taken)
    {
        g_control_pending = false;
    }
    if (g_control_pending) { return CONTROL_BUSY; }
    bool cancel=kind==CONTROL_REQUEST_ID_CANCEL || kind==CONTROL_REQUEST_ID_DISCARD;
    if (!cancel && kind != CONTROL_REQUEST_PARAMETER && kind != CONTROL_REQUEST_CLEAR_FAULT &&
        kind != CONTROL_REQUEST_STOP && (!g_control_snapshot.ready || control_model_stale(now))) { return CONTROL_NOT_READY; }
    if (g_control_id_lock && (kind<CONTROL_REQUEST_ID_CANCEL ||
        kind==CONTROL_REQUEST_STEP_TEST)) { return CONTROL_BUSY; }
    uint32_t capability=0U;
    switch (kind)
    {
    case CONTROL_REQUEST_START:
        capability=CONTROL_CAP_START;
        if (g_control_snapshot.run!=CONTROL_STOPPED && g_control_snapshot.run!=CONTROL_DISABLED) { return CONTROL_BUSY; }
        break;
    case CONTROL_REQUEST_STOP: capability=CONTROL_CAP_STOP; break;
    case CONTROL_REQUEST_CLEAR_FAULT:
        capability=CONTROL_CAP_CLEAR_FAULT;
        if (g_control_snapshot.run==CONTROL_RUNNING) { return CONTROL_BUSY; }
        break;
    case CONTROL_REQUEST_STEP_TEST:
        capability=CONTROL_CAP_STEP_TEST;
        if (!g_control_snapshot.power_disabled || (g_control_snapshot.run!=CONTROL_STOPPED &&
            g_control_snapshot.run!=CONTROL_DISABLED)) { return CONTROL_BUSY; }
        /* 停机会使板卡校准状态失效；与START一样交给后端在通电前重新校准。 */
        break;
    case CONTROL_REQUEST_CALIBRATE:
        capability=CONTROL_CAP_CALIBRATE;
        if (g_control_snapshot.run!=CONTROL_DISABLED && g_control_snapshot.run!=CONTROL_STOPPED) { return CONTROL_BUSY; }
        break;
    case CONTROL_REQUEST_SOURCE:
        capability=CONTROL_CAP_SOURCE;
        if (value<0 || value>CONTROL_REMOTE) { return CONTROL_RANGE; }
        if (g_control_snapshot.run!=CONTROL_DISABLED && g_control_snapshot.run!=CONTROL_STOPPED) { return CONTROL_BUSY; }
        break;
    case CONTROL_REQUEST_PARAMETER:
    {
        capability=CONTROL_CAP_PARAMETERS;
        const control_parameter_t *p=control_model_parameter(parameter);
        if (p==NULL || !p->valid || !p->editable) { return CONTROL_NOT_READY; }
        if (value<p->minimum || value>p->maximum) { return CONTROL_RANGE; }
        if (parameter != CONTROL_PARAM_TARGET && g_control_snapshot.run==CONTROL_RUNNING) { return CONTROL_BUSY; }
        break;
    }
    case CONTROL_REQUEST_ID_START:
        capability=CONTROL_CAP_IDENTIFY;
        if (g_control_id_lock || !g_control_snapshot.power_disabled ||
            (g_control_snapshot.run!=CONTROL_DISABLED && g_control_snapshot.run!=CONTROL_STOPPED)) { return CONTROL_BUSY; }
        break;
    case CONTROL_REQUEST_ID_CANCEL:
        if (!g_control_id_lock || g_control_id.stage==CONTROL_ID_SUCCESS) { return CONTROL_INVALID; }
        break;
    case CONTROL_REQUEST_ID_APPLY:
    case CONTROL_REQUEST_ID_DISCARD:
        if (!g_control_id_lock || g_control_id.stage!=CONTROL_ID_SUCCESS) { return CONTROL_INVALID; }
        break;
    default: return CONTROL_INVALID;
    }
    if (capability!=0U && (g_control_snapshot.capabilities & capability)==0U) { return CONTROL_NOT_READY; }
    if (kind==CONTROL_REQUEST_ID_START)
    {
        memset(&g_control_id,0,sizeof(g_control_id));
        if (++g_control_task==0U) { ++g_control_task; }
        g_control_id.task_id=g_control_task;
        g_control_id.stage=CONTROL_ID_PREPARING;
        g_control_id.updated_ms=now;
        g_control_id_lock=true;
    }
    if (++g_control_sequence==0U) { ++g_control_sequence; }
    g_control_request=(control_request_t){.sequence=g_control_sequence,.task_id=g_control_id.task_id,
        .kind=kind,.parameter=parameter,.value=value,.algorithm=g_control_algorithm,.mode=g_control_mode};
    if (kind==CONTROL_REQUEST_STEP_TEST) { g_control_request.step_test=*test; }
    g_control_pending=true;
    g_control_taken=false;
    notice("PENDING");
    return CONTROL_OK;
}
/** @brief 提交普通操作或无负载直启测试；单扇区或整轮六步测试使用带配置接口。 */
control_result_t
control_model_request(control_request_kind_t kind, control_parameter_id_t parameter, int32_t value, uint32_t now)
{
    return request(kind,parameter,value,NULL,now);
}
/** @brief 提交独立六步测试，使用同一个在途请求槽。 */
control_result_t
control_model_request_step(const control_step_test_t *test, uint32_t now)
{
    return request(CONTROL_REQUEST_STEP_TEST,CONTROL_PARAM_TARGET,0,test,now);
}
/** @brief 后端取一次请求；取走后仍在途，直到complete。 */
bool
control_model_take_request(control_request_t *request)
{
    if (request==NULL || !g_control_pending || g_control_taken) { return false; }
    *request=g_control_request;
    g_control_taken=true;
    return true;
}
/** @brief 后端返回操作结果。 */
bool
control_model_complete(uint32_t sequence, bool success, const char *reason)
{
    if (!g_control_pending || !g_control_taken || sequence!=g_control_request.sequence) { return false; }
    control_request_kind_t kind=g_control_request.kind;
    g_control_pending=false;
    if (!success)
    {
        if (kind==CONTROL_REQUEST_ID_START)
        {
            g_control_id.stage=CONTROL_ID_FAILED;
            g_control_id_lock=false;
        }
        if (kind>=CONTROL_REQUEST_ID_START && kind<=CONTROL_REQUEST_ID_DISCARD) { snprintf(g_control_id.reason,24,"%s",reason!=NULL ? reason : "FAILED"); }
        notice(reason!=NULL ? reason : "FAILED");
        return true;
    }
    if (kind>=CONTROL_REQUEST_ID_START && kind<=CONTROL_REQUEST_ID_DISCARD) { g_control_id.reason[0]=0; }
    switch (kind)
    {
    case CONTROL_REQUEST_PARAMETER: g_control_parameters[g_control_request.parameter].value=g_control_request.value; break;
    case CONTROL_REQUEST_SOURCE: g_control_snapshot.source=(control_source_t)g_control_request.value; break;
    case CONTROL_REQUEST_ID_CANCEL: g_control_id.stage=CONTROL_ID_CANCELLED; g_control_id_lock=false; break;
    case CONTROL_REQUEST_ID_DISCARD: g_control_id.stage=CONTROL_ID_READY; g_control_id.result_count=0; g_control_id_lock=false; break;
    case CONTROL_REQUEST_ID_APPLY:
        for (unsigned i=0;i<g_control_id.result_count;++i) { g_control_parameters[g_control_id.results[i].id]=g_control_id.results[i]; }
        g_control_id.stage=CONTROL_ID_APPLIED;
        g_control_id_lock=false;
        break;
    default: break;
    }
    notice(kind==CONTROL_REQUEST_ID_APPLY ? "APPLIED" : kind==CONTROL_REQUEST_ID_CANCEL ? "CANCELLED" : "OK");
    return true;
}
/** @brief 发布识别阶段/候选，拒绝旧任务和取消中的迟到结果。
 * */
bool
control_model_id_publish(const control_id_report_t *r)
{
    if (r==NULL || !g_control_id_lock || r->task_id!=g_control_id.task_id ||
        g_control_pending || g_control_id.stage==CONTROL_ID_SUCCESS ||
        (r->stage!=CONTROL_ID_PREPARING && r->stage!=CONTROL_ID_ROTATING &&
         r->stage!=CONTROL_ID_PROCESSING && r->stage!=CONTROL_ID_SUCCESS && r->stage!=CONTROL_ID_FAILED)) { return false; }
    if (r->result_count>CONTROL_ID_RESULTS_MAX)
    {
        g_control_id.stage=CONTROL_ID_FAILED;
        g_control_id.result_count=0;
        g_control_id_lock=false;
        snprintf(g_control_id.reason,24,"TOO MANY RESULTS");
        notice("TOO MANY RESULTS");
        return false;
    }
    if (r->progress>1000U || memchr(r->direction,0,sizeof(r->direction))==NULL ||
        memchr(r->reason,0,sizeof(r->reason))==NULL) { return false; }
    if (r->stage==CONTROL_ID_SUCCESS && r->result_count==0U) { return false; }
    for (unsigned i=0;i<r->result_count;++i)
    {
        if (!parameter_valid(&r->results[i]) || !r->results[i].valid) { return false; }
        for (unsigned j=0;j<i;++j) { if (r->results[i].id==r->results[j].id) { return false; } }
    }
    if (r->stage != g_control_id.stage || strcmp(r->reason,g_control_id.reason)!=0) { ++g_control_revision; }
    g_control_id=*r;
    if (r->stage==CONTROL_ID_FAILED) { g_control_id_lock=false; notice(r->reason); }
    return true;
}
/** @brief 查询在途请求。 */
const control_request_t *
control_model_pending(void) { return g_control_pending ? &g_control_request : NULL; }
/** @brief 获取状态变化序号，用于立即刷新阶段/结果。 */
uint32_t
control_model_revision(void) { return g_control_revision; }
/** @brief 获取最近请求结果文本。 */
const char *
control_model_notice(void) { return g_control_notice; }
/** @brief 获取显示名称。 */
const char *
control_algorithm_name(control_algorithm_t a)
{
    static const char *const names[]={"OPEN 6STEP","SENSORLESS 6STEP","MAG 6STEP","MAG FOC","SENSORLESS FOC"};
    return (unsigned)a<CONTROL_ALGORITHM_COUNT ? names[a] : "--";
}
/** @brief 获取协议枚举文本。 */
const char *
control_algorithm_token(control_algorithm_t a)
{
    static const char *const names[]={"OPEN_6STEP","SENSORLESS_6STEP","ENCODER_6STEP","ENCODER_FOC","SENSORLESS_FOC"};
    return (unsigned)a<CONTROL_ALGORITHM_COUNT ? names[a] : "--";
}
/** @brief 获取模式文本。 */
const char *
control_mode_name(control_mode_t m)
{
    static const char *const names[]={"SPEED","TORQUE","POSITION","CURRENT","HAPTIC"};
    return (unsigned)m<CONTROL_MODE_COUNT ? names[m] : "--";
}
/** @brief 获取结果文本。 */
const char *
control_result_name(control_result_t r)
{
    static const char *const names[]={"OK","NOT_READY","BUSY","RANGE","ARGUMENT"};
    return (unsigned)r<=CONTROL_INVALID ? names[r] : "ARGUMENT";
}
/** @brief 定点格式化，不引入浮点printf。 */
void
control_format(char *out, size_t size, int32_t value, uint8_t decimals, const char *unit)
{
    static const uint32_t powers[]={1,10,100,1000};
    if (decimals>3U) { snprintf(out,size,"--"); return; }
    uint32_t magnitude=value<0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
    uint32_t scale=powers[decimals];
    if (decimals==0U) { snprintf(out,size,"%s%lu%s%s",value<0 ? "-" : "",(unsigned long)magnitude,*unit ? " " : "",unit); }
    else { snprintf(out,size,"%s%lu.%0*lu%s%s",value<0 ? "-" : "",(unsigned long)(magnitude/scale),decimals,(unsigned long)(magnitude%scale),*unit ? " " : "",unit); }
}

/** @brief 查询算法是否FOC。 */
bool control_algorithm_foc(control_algorithm_t a) { return a==CONTROL_ENCODER_FOC || a==CONTROL_SENSORLESS_FOC; }
