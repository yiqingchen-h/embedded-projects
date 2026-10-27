/* 复用既有Flash/六步板卡替身，编译真实应用后端；不复制应用的START或保存逻辑。 */
#define main six_step_regression_main
#define board_motor_stop six_step_mock_stop
#include "test_motor.c"
#undef board_motor_stop
#undef main
#include "board_foc.h"
#include <math.h>

/* 替身只模拟板卡接受配置及实际提交标志；SMO/电流/融合在实际纯算法测试中验证。 */
static bool foc_active,commit_confirmed;
static foc_drive_config_t received_config;
static uint32_t received_duration;
/* 有感入口独立锁存方向/零点，既有无感夹具仍默认false。 */
static bool foc_encoder;
static encoder_foc_config_t received_encoder_config;

/** @brief 与实物关断契约一致，STOP同时撤销未来FOC命令。 */
void board_motor_stop(const char *reason)
{ six_step_mock_stop(reason); foc_active=false; commit_confirmed=false; }
bool board_motor_standstill(void) { return !hardware.running; }
bool board_foc_active(void) { return foc_active; }
bool board_foc_speed_active(void) { return foc_active; }
void board_foc_service(void) { if(!hardware.running) { foc_active=false; } }
bool board_foc_speed_start(const foc_drive_config_t *config,uint32_t duration)
{
    CHECK(!hardware.running);
    received_config=*config;
    received_duration=duration;
    foc_active=hardware.running=true;
    foc_encoder=false;
    commit_confirmed=false;
    return true;
}
/** @brief 仅模拟板卡接受已校准有感配置，实际dq/反馈控制由纯算法独立回归。
 * @param config 有感配置。 @param duration 有界时限。 @return 替身已启动。 */
bool board_foc_encoder_start(const encoder_foc_config_t *config,uint32_t duration)
{
    CHECK(!hardware.running);
    received_encoder_config=*config;
    received_config=config->drive;
    received_duration=duration;
    foc_encoder=true;
    foc_active=hardware.running=true;
    commit_confirmed=false;
    return true;
}
bool board_foc_speed_target(uint32_t rpm)
{
    if(!foc_active || !commit_confirmed) { return false; }
    received_config.target_rpm=rpm;
    return true;
}
/** @brief 电流模式板卡替身只验证资格和目标传递，实际斜坡由正式算法测试。
 * @param current_ma 有符号mA。 @return 独立Iq实际提交后接受。 */
bool board_foc_iq_target(int32_t current_ma)
{
    if(!foc_active || !foc_encoder || !received_encoder_config.current_mode || !commit_confirmed ||
        current_ma<-500 || current_ma>500) { return false; }
    received_encoder_config.target_iq_a=current_ma*.001f;
    return true;
}
bool board_foc_position_target(int32_t target_mdeg)
{
    if(!foc_active || !commit_confirmed || !received_encoder_config.position_mode) { return false; }
    received_encoder_config.target_position_mdeg=target_mdeg;return true;
}
void board_foc_position(char *out,size_t size,int32_t target,int32_t tolerance)
{ snprintf(out,size,"OK FOC POSITION TARGET_MDEG=%ld TOL_MDEG=%ld",(long)target,(long)tolerance); }
void board_foc_runtime(board_foc_runtime_t *out)
{
    memset(out,0,sizeof(*out));
    out->tracking=out->samples_valid=foc_active;
    out->closed=commit_confirmed;
    out->encoder=foc_encoder;
    out->current_mode=foc_encoder&&received_encoder_config.current_mode;
    out->target_iq_ma=(int32_t)lroundf(received_encoder_config.target_iq_a*1000.0f);
    out->stage=commit_confirmed?FOC_CLOSED:foc_encoder?FOC_CALIBRATE:FOC_BLEND;
    out->target_rpm=received_config.target_rpm;
    out->estimated_rpm=1000;
    out->bus_mv=12000;
    out->id_ma=10; out->iq_ma=100;
}
bool board_foc_current(char *out,size_t size,bool firewater) { (void)out;(void)size;(void)firewater;return false; }
bool board_foc_trace(char *out,size_t size) { (void)out;(void)size;return false; }
bool board_foc_hold_iq(uint32_t current_ma) { (void)current_ma;return false; }
bool board_foc_harmonic(bool enabled) { (void)enabled;return true; }
bool board_foc_harmonic2(bool enabled) { (void)enabled;return true; }
void board_foc_harmonic_diagnostics(char *out,size_t size) { snprintf(out,size,"OK MOCK HARMONIC"); }
void board_foc_harmonic2_diagnostics(char *out,size_t size) { snprintf(out,size,"OK MOCK HARMONIC2"); }
bool board_foc_zero_start(uint32_t duration_ms) { (void)duration_ms;return false; }
bool board_foc_test_trigger(uint32_t trigger) { (void)trigger;return false; }
uint32_t board_foc_test_trigger_ticks(void) { return 8400; }
bool board_foc_test_handoff(uint32_t rpm) { return !foc_active && rpm>=1U && rpm<=1000U; }
uint32_t board_foc_test_handoff_rpm(void) { return 1000U; }
void board_foc_diagnostics(char *out,size_t size,bool detail) { (void)detail;snprintf(out,size,"OK MOCK"); }
void board_foc_current_diagnostics(char *out,size_t size) { snprintf(out,size,"OK MOCK"); }
void board_foc_observer_diagnostics(char *out,size_t size) { snprintf(out,size,"OK MOCK"); }
void board_foc_drive_diagnostics(char *out,size_t size) { snprintf(out,size,"OK MOCK"); }
/** @brief 测量协议替身，本测试不生成或伪造真实机械数据。 */
void board_foc_motion(char *out,size_t size,uint32_t first)
{ (void)first; snprintf(out,size,"OK MOCK MOTION"); }
void board_foc_confidence_diagnostics(char *out,size_t size) { snprintf(out,size,"OK MOCK"); }
/* 有界实验入口在本测试不启动，仅满足真实后端的链接接口。 */
#define MOCK_PAIR(name) bool name(uint32_t a,uint32_t b) { (void)a;(void)b;return false; }
MOCK_PAIR(board_foc_bench_start)
MOCK_PAIR(board_foc_vector_start)
MOCK_PAIR(board_foc_switch_start)
MOCK_PAIR(board_foc_if_start)
MOCK_PAIR(board_foc_observe_start)
MOCK_PAIR(board_foc_closed_start)
MOCK_PAIR(board_foc_range_start)
MOCK_PAIR(board_foc_sample_vector_start)
bool board_foc_sample_start(uint32_t a) { (void)a;return false; }
bool board_foc_current_start(bool q,uint32_t a,uint32_t b) { (void)q;(void)a;(void)b;return false; }

#ifndef MOTOR_FOC_BACKEND_FIXTURE_ONLY
/** @brief 真实请求/后端验证参数隔离、提交资格、在线目标、停机保存及未实现模式。 */
int main(void)
{
    reset_run_backend();
    int32_t old_target=settings_get()->parameter[CONTROL_PARAM_TARGET];
    int32_t old_limit=settings_get()->parameter[CONTROL_PARAM_CURRENT_LIMIT];
    CHECK(control_model_select(CONTROL_SENSORLESS_FOC,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(now+=20);
    CHECK(control_model_snapshot()->ready);
    CHECK(!motor_backend_foc_harmonic(true));
    CHECK(!motor_backend_foc_harmonic2(true));
    request(CONTROL_REQUEST_SOURCE,CONTROL_PARAM_TARGET,CONTROL_REMOTE);
    CHECK(motor_backend_foc_harmonic(true));
    CHECK(motor_backend_foc_harmonic2(true));
    CHECK(motor_backend_foc_harmonic(false));
    CHECK(motor_backend_foc_harmonic2(false));
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1500);
    CHECK(settings_get()->foc.target_rpm==1500 && settings_get()->parameter[CONTROL_PARAM_TARGET]==old_target);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(foc_active && received_duration==20000);
    CHECK(fabsf(received_config.resistance_ohm-.79f)<1e-6f);
    CHECK(fabsf(received_config.inductance_h-.00024f)<1e-9f);
    CHECK(fabsf(received_config.current_limit_a-.5f)<1e-6f);
    CHECK(fabsf(received_config.voltage_error_bound_v-.12f)<1e-6f);
    CHECK(fabsf(received_config.speed_kp-.0002f)<1e-8f && fabsf(received_config.speed_ki-.0005f)<1e-8f);
    CHECK(received_config.target_rpm==1500 && received_config.pole_pairs==3);
    CHECK(received_config.extended_speed_range);
    CHECK(!motor_backend_foc_handoff(600));
    CHECK(strcmp(control_model_snapshot()->startup,"BLEND")==0);
    CHECK(strcmp(control_model_snapshot()->commutation,"STARTUP ANGLE")==0);
    CHECK(!control_model_snapshot()->values[CONTROL_MECH_ANGLE].valid);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,2000);
    CHECK(strcmp(control_model_notice(),"FOC WAIT CLOSED")==0 && settings_get()->foc.target_rpm==1500);
    commit_confirmed=true;
    motor_backend_service(now+=20);
    CHECK(strcmp(control_model_snapshot()->startup,"CLOSED")==0);
    CHECK(strcmp(control_model_snapshot()->commutation,"SMO PLL")==0);
    unsigned before=writes;
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,2000);
    CHECK(received_config.target_rpm==2000 && settings_get()->foc.target_rpm==2000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,500);
    CHECK(received_config.target_rpm==500 && settings_get()->foc.target_rpm==500);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,5400);
    CHECK(received_config.target_rpm==5400 && settings_get()->foc.target_rpm==5400);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,2000);
    CHECK(settings_get()->parameter[CONTROL_PARAM_CURRENT_LIMIT]==old_limit);
    motor_backend_service(now+=2000);
    CHECK(writes==before && strcmp(settings_status(),"PENDING")==0);
    CHECK(control_model_request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_CURRENT_LIMIT,2000,now)==CONTROL_BUSY);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    CHECK(!foc_active && !hardware.running && control_model_snapshot()->power_disabled);
    CHECK(motor_backend_foc_handoff(600));
    CHECK(!motor_backend_foc_handoff(0) && !motor_backend_foc_handoff(1001));
    motor_backend_service(now+=2000);
    CHECK(writes==before+1U && strcmp(settings_status(),"SAVED")==0);
    CHECK(control_model_select(CONTROL_SENSORLESS_FOC,CONTROL_TORQUE)==CONTROL_OK);
    motor_backend_service(now+=20);
    CHECK(!control_model_snapshot()->ready && !control_model_parameter(CONTROL_PARAM_TARGET)->valid);
    CHECK(control_model_select(CONTROL_SENSORLESS_6STEP,CONTROL_SPEED)==CONTROL_OK);
    CHECK(!motor_backend_foc_harmonic(true));
    CHECK(!motor_backend_foc_harmonic2(true));
    motor_backend_service(now+=20);
    CHECK(settings_value(CONTROL_PARAM_TARGET)==old_target && settings_value(CONTROL_PARAM_CURRENT_LIMIT)==old_limit);
    printf("FOC application routing: %u checks passed\n",checks);
    return 0;
}
#endif

/* 记录真正发往实时控制的次数，不能用RAM保存成功替代ISR接收。 */
static unsigned g_haptic_publishes;
/** @brief 模拟独立FOC运行标志及完整发布；六步running可保持false。 */
bool board_foc_haptic_configure(const encoder_haptic_config_t *c)
{
    if(!foc_active || !foc_encoder || !received_encoder_config.haptic_mode || !encoder_haptic_valid(c)) { return false; }
    ++g_haptic_publishes;received_encoder_config.haptic=*c;return true;
}
/** @brief 返回后端协议夹具。 */
void board_foc_haptic(char *out,size_t size,const encoder_haptic_config_t *c,int32_t limit,int32_t run)
{ (void)c;(void)limit;(void)run;snprintf(out,size,"OK FOC HAPTIC"); }
