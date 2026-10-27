#include "interaction.h"
#include "board_storage.h"
#include "motor_backend.h"
#include "motor_config.h"
#include "control_model.h"
#include "settings.h"
#include "board_io.h"
#include "board_encoder.h"
#include "serial_buffer.h"
#include "console.h"
#include "wave_stream.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
/* 串口 回归使用空 Flash；掉电恢复另由 motor 回归覆盖。 */
/** @brief 串口测试使用匹配的 Flash 布局。 */
bool
board_storage_layout_valid(void) { return true; }
/** @brief 串口测试模拟首次上电的擦除态记录。 */
bool
board_storage_read(unsigned slot, void *data, size_t size) { (void)slot; memset(data,255,size); return true; }
/** @brief 串口测试的保存接口替身，掉电验证由motor测试覆盖。 */
bool
board_storage_write(unsigned slot, const void *data, size_t size) { (void)slot; (void)data; (void)size; return true; }
/** @brief 串口协议测试按共用频率宏模拟诊断字段。 @param out 输出。 @param size 容量。 */
void
motor_backend_diagnostics(char *out, size_t size)
{
    snprintf(out,size,"OK MOTOR MOCK PWM_HZ=%lu CTRL_HZ=%lu",
        (unsigned long)MOTOR_CONTROL_HZ,(unsigned long)MOTOR_CONTROL_HZ);
}
/** @brief 标定命令分派替身；本测试不输出功率。 @return 明确拒绝原因。 */
const char *motor_backend_encoder_calibrate(void) { return "SELECT ENCODER_6STEP"; }
/* 会话开关替身，用于验证分派、参数及后端拒绝原因的透传。 */
void motor_backend_foc_position(char *out,size_t size) { snprintf(out,size,"OK FOC POSITION VALID=0"); }
const char *motor_backend_position_zero(void) { return "ENCODER CAL REQUIRED"; }
const char *motor_backend_position_tolerance(int32_t mdeg) { return mdeg==100 || mdeg==500?NULL:"RANGE"; }
static bool g_encoder_quiet;
static const char *g_encoder_quiet_reason;
/** @brief 模拟停机会话选项。 @param enabled 目标状态。 @return 拒绝原因或NULL。 */
const char *motor_backend_encoder_quiet(bool enabled)
{
    if (g_encoder_quiet_reason!=NULL) { return g_encoder_quiet_reason; }
    g_encoder_quiet=enabled;return NULL;
}
/* 仅验证分派和拒绝传播，真实停机/静止守卫由后端回归覆盖。 */
static unsigned g_encoder_hyst=7U;
/** @brief 模拟有界影子迟滞。 @param code 4..7。 @return 后端拒绝原因或NULL。 */
const char *motor_backend_encoder_hysteresis(uint8_t code)
{
    if(NULL!=g_encoder_quiet_reason) { return g_encoder_quiet_reason; }
    g_encoder_hyst=code;return NULL;
}
/** @brief 编码器有感状态查询替身。 @param out 输出。 @param size 容量。 */
void motor_backend_encoder_diagnostics(char *out,size_t size) { snprintf(out,size,"OK MOTOR ENCODER CALIBRATION=REQUIRED"); }
/* 使用较长计数和最长阶段名，覆盖新增字段后的整行响应容量。 */
static const char g_run_diagnostics[] =
    "OK MOTOR RUN TICKS=4294967295 VALID=4294967295 INVALID=4294967295 "
    "LIMIT_HITS=4294967295 DENSITY_SKIP=4294967295 "
    "PHASE_SAMPLES=4294967295,4294967295,4294967295 "
    "PEAK_MA=4294967295,4294967295,4294967295 LOW=C RAW=4095 DELTA=-4095 "
    "SECTOR=6 HZ=3333.333 DUTY=25.0 STAGE=ALIGN_RAMP";
/** @brief 串口协议测试的最近RUN诊断替身，保留阶段字段。 @param out 输出。 @param size 容量。 */
void
motor_backend_run_diagnostics(char *out, size_t size) { snprintf(out,size,"%s",g_run_diagnostics); }
/** @brief 串口协议测试的只读存储诊断替身。 @param out 输出。 @param size 容量。 */
void
motor_backend_storage_diagnostics(char *out, size_t size) { snprintf(out,size,"OK STORAGE MOCK"); }
/** @brief 模拟无感只读诊断，验证命令分派及响应。 */
void
motor_backend_bemf_diagnostics(char *out, size_t size) { snprintf(out,size,"OK MOTOR BEMF MOCK"); }
/** @brief 分段时序与曲线协议替身，不访问板卡。 */
void motor_backend_timing_diagnostics(char *out,size_t size) { snprintf(out,size,"OK MOTOR TIMING MOCK"); }
/** @brief 交互层协作消费入口替身，真实队列由电机测试覆盖。 */
void motor_backend_bemf_trace_diagnostics(char *out,size_t size,unsigned page,bool events)
{ (void)page; (void)events; snprintf(out,size,"OK MOTOR BEMF TRACE MOCK"); }
void motor_backend_bemf_sector_diagnostics(char *out,size_t size,unsigned sector,unsigned page)
{ snprintf(out,size,"OK MOTOR BEMF SECTOR=%u PAGE=%u MOCK",sector+1U,page); }
/** @brief 独立六步状态只读查询替身。 @param out 输出。 @param size 容量。 */
void
motor_backend_step_diagnostics(char *out, size_t size) { snprintf(out,size,"OK STEP STATUS MOCK"); }
/** @brief 时序测试协议替身；串口回归不启动实物。 */
bool motor_backend_foc_bench(uint32_t duty,uint32_t duration_ms)
{ (void)duty; (void)duration_ms; return false; }
/** @brief 机械测量协议替身，仅检查序号透传，不冒充实测记录。 */
void motor_backend_foc_motion(char *out,size_t size,uint32_t first)
{ snprintf(out,size,"OK FOC MOTION MOCK FIRST=%lu",(unsigned long)first); }
/** @brief 静态矢量命令替身，不启动电机。 */
bool motor_backend_foc_vector(uint32_t sector,uint32_t spread)
{ (void)sector; (void)spread; return false; }
/** @brief 采样扫描命令替身，不访问实物。 */
bool motor_backend_foc_sample(uint32_t trigger)
{ (void)trigger; return false; }
/** @brief 动态提交测试替身，不启动实物。 */
bool motor_backend_foc_switch(uint32_t spread,uint32_t duration_ms)
{ (void)spread; (void)duration_ms; return false; }
/** @brief dq有界测试替身，不访问实物。 */
bool motor_backend_foc_current_start(bool q_axis,uint32_t target_ma,uint32_t duration_ms)
{ (void)q_axis; (void)target_ma; (void)duration_ms; return false; }
/** @brief 强拖试验替身，不访问实物或返回虚假估速。 */
bool motor_backend_foc_observe(uint32_t frequency_millihz,uint32_t duration_ms)
{ return frequency_millihz>=1000U&&frequency_millihz<=50000U&&duration_ms>=500U&&duration_ms<=5000U; }
bool motor_backend_foc_closed(uint32_t target_rpm,uint32_t duration_ms)
{ return target_rpm>=1U&&target_rpm<=5400U&&duration_ms>=500U&&duration_ms<=32000U; }
/** @brief 量程协议替身，实物验收由独立有界工具完成。 */
bool motor_backend_foc_range(uint32_t angle_slot,uint32_t target_ma)
{ return angle_slot>=1U&&angle_slot<=6U&&(target_ma==500U||target_ma==1000U||target_ma==1500U||target_ma==2000U); }
/** @brief 最大CCR六扇区扫描协议替身。 */
bool motor_backend_foc_sample_vector(uint32_t trigger,uint32_t sector)
{ return trigger>=8200U&&trigger<=8490U&&sector>=1U&&sector<=6U; }
bool motor_backend_foc_if(uint32_t frequency_millihz,uint32_t duration_ms)
{ (void)frequency_millihz; (void)duration_ms; return false; }
/** @brief 新增诊断协议替身；算法及硬件分别验收。 */
void motor_backend_foc_diagnostics(char *out,size_t size,int page)
{ (void)page; snprintf(out,size,"ERR FOC UNAVAILABLE"); }
/* 测试替身只生成最新行，检查主循环发送与STOP优先，不模拟ADC采集。 */
static bool g_mock_foc_current;
static unsigned g_mock_foc_reads;
/* 最新模拟采样序号及最近成功打印序号，不使用时间节流。 */
static unsigned g_mock_foc_sample, g_mock_foc_sent;
/** @brief 模拟最新电流的两种格式，不重复已打印样本。
 * @param out 输出缓冲。 @param size 容量。 @param firewater true为四通道波形。
 * @return 已生成新样本。 */
bool motor_backend_foc_current(char *out,size_t size,bool firewater)
{
    if (!g_mock_foc_current || g_mock_foc_sample == g_mock_foc_sent) { return false; }
    ++g_mock_foc_reads;
    g_mock_foc_sent = g_mock_foc_sample;
    if (firewater) { snprintf(out,size,"FOC:1,2,-3,-4\r\n"); }
    else { snprintf(out,size,"DATA FOC SAMPLE=%u IA_MA=1 IB_MA=2 IC_MA=-3\r\n",g_mock_foc_sample); }
    return true;
}
/** @brief 基准日志分派替身，验证TRACE模式仍只消费最新样本。 */
bool motor_backend_foc_trace(char *out,size_t size)
{
    if(!motor_backend_foc_current(out,size,false)) { return false; }
    snprintf(out,size,"FOC_TRACE:%u,8436,1,6\r\n",g_mock_foc_sample);
    return true;
}
/** @brief 固定Iq命令替身；实际状态与限幅由电机回归检查。 */
bool motor_backend_foc_hold_iq(uint32_t current_ma) { return current_ma<=500U; }
bool motor_backend_foc_harmonic(bool enabled) { (void)enabled;return true; }
bool motor_backend_foc_harmonic2(bool enabled) { (void)enabled;return true; }
bool motor_backend_foc_zero(uint32_t duration_ms) { return duration_ms>=20U && duration_ms<=500U; }
bool motor_backend_foc_trigger(uint32_t trigger) { return trigger>=8300U && trigger<=8470U; }
uint32_t motor_backend_foc_trigger_ticks(void) { return 8400; }
bool motor_backend_foc_handoff(uint32_t rpm) { return rpm>=1U && rpm<=1000U; }
uint32_t motor_backend_foc_handoff_rpm(void) { return 1000U; }
/* 只模拟输入电平、时钟与串口；没有OLED/I²C替身，残留显示依赖会直接链接失败。 */
static uint32_t g_now;
static uint8_t g_raw = 7U;
static unsigned g_checks;
static board_uart_stats_t g_uart;
/* 速率替身只在成功SET时变化，不模拟真实线路切换时序。 */
static uint32_t g_mock_uart_baud=921600U;
/* 实板静止检查约175ms，测试源不能沿用检查前的循环时刻。 */
static uint32_t g_mock_still_delay;
/* 编码器读取由独立模块回归覆盖；此处只注入快照检查真实协议格式。 */
static encoder_snapshot_t g_mock_encoder;
static char g_output[8192];
static size_t g_output_length;
#define CHECK(x) do { ++g_checks; assert(x); } while (0)
/** @brief 主机时基。 @return 模拟ms。 */
uint32_t board_millis(void) { return g_now; }
/** @brief 模拟开关电平。 @return 三路位。 */
uint8_t board_switch_raw(void) { return g_raw; }
/** @brief 模拟发送中断启动，测试循环主动取字节。 */
void board_uart_kick(void) { }
/** @brief 读取模拟串口统计。 @return 统计。 */
board_uart_stats_t board_uart_stats(void) { return g_uart; }
/** @brief 返回模拟编码器快照。 @param now_ms 当前毫秒。 @return 测试快照。 */
encoder_snapshot_t board_encoder_snapshot(uint32_t now_ms) { (void)now_ms; return g_mock_encoder; }
/** @brief 编码器详细命令的只读分派替身。 @param out 输出。 @param size 容量。 @param now_ms 当前毫秒。 */
void board_encoder_diagnostics(char *out, size_t size, uint32_t now_ms)
{ (void)now_ms; snprintf(out,size,"OK ENCODER MOCK"); }
/** @brief 运行完整应用，模拟每毫秒一个接收/发送服务机会。 @param ms 时间。 */
static void
run(unsigned ms)
{
    for (unsigned i=0;i<ms;++i)
    {
        interaction_service();
        uint8_t byte;
        for (unsigned j=0;j<12 && serial_buffer_transmit(&byte);++j)
        {
            CHECK(g_output_length+1 < sizeof(g_output));
            g_output[g_output_length++] = (char)byte;
            g_output[g_output_length] = '\0';
            ++g_uart.tx;
        }
        ++g_now;
    }
}
/** @brief 注入串口文本。 @param text 命令文本。 */
static void
inject(const char *text)
{
    for (;*text;++text) { serial_buffer_receive((uint8_t)*text,false); ++g_uart.rx; }
}
/** @brief 请求并检查唯一响应。 @param text 完整命令。 @param expected 期望响应内容。 */
static void
request(const char *text, const char *expected)
{
    g_output_length=0;
    g_output[0]='\0';
    inject(text);
    run(50);
    if(strstr(g_output,expected)!=g_output)
    { fprintf(stderr,"request %s expected %s actual %s\n",text,expected,g_output); }
    CHECK(strstr(g_output,expected) == g_output);
    unsigned lines=0;
    for (char *p=g_output;*p;++p) { if (*p=='\n') { ++lines; } }
    CHECK(lines==1);
}
/** @brief 验证无屏固件的既有串口命令、兼容字段及本地输入无控制副作用。 */
static void
test_application(void)
{
    serial_buffer_init();
    interaction_init(true);
    request("ping\r\n","OK PING\r\n");
    request("VERSION\n","OK VERSION FW=" INTERACTION_VERSION " PROTOCOL=1.5");
    request("GET HAPTIC_CAPS\n","OK GET HAPTIC_CAPS=HAP1");
    request("MOTOR FOC HAPTIC\n","OK FOC HAPTIC");
    request("MOTOR FOC HAPTIC CONFIG DETENT 24 100 2000\n","OK FOC HAPTIC");
    request("MOTOR FOC HAPTIC CONFIG DAMPED 256 800 10000\n","OK FOC HAPTIC");
    request("MOTOR FOC HAPTIC CONFIG BAD 24 100 2000\n","ERR FOC RANGE");
    request("MOTOR FOC HAPTIC CONFIG DETENT 1 100 2000\n","ERR FOC RANGE");
    request("MOTOR FOC HAPTIC CONFIG DETENT 24 801 2000\n","ERR FOC RANGE");
    request("MOTOR FOC HAPTIC CONFIG DETENT 24 100\n","ERR FOC ARGUMENT");
    request("MOTOR\n","OK MOTOR MOCK PWM_HZ=20000 CTRL_HZ=20000");
    request("MOTOR FOC CONFIDENCE\n","ERR FOC UNAVAILABLE");
    request("MOTOR FOC CONFIDENCE EXTRA\n","ERR FOC ARGUMENT");
    request("MOTOR RUN\n",g_run_diagnostics);
    CHECK(strlen(g_output)==strlen(g_run_diagnostics)+2U);
    CHECK(strcmp(g_output+strlen(g_run_diagnostics),"\r\n")==0);
    request("MOTOR BEMF\n","OK MOTOR BEMF MOCK");
    request("MOTOR ENCODER\n","OK MOTOR ENCODER CALIBRATION=REQUIRED");
    request("MOTOR ENCODER CAL\n","ERR ENCODER SELECT ENCODER_6STEP");
    request("MOTOR ENCODER BAD\n","ERR ENCODER ARGUMENT");
    request("MOTOR ENCODER CAL EXTRA\n","ERR ENCODER ARGUMENT");
    request("MOTOR ENCODER QUIET ON\n","OK ENCODER QUIET=ON\r\n");
    CHECK(g_encoder_quiet);
    request("MOTOR ENCODER QUIET OFF\n","OK ENCODER QUIET=OFF\r\n");
    CHECK(!g_encoder_quiet);
    request("MOTOR ENCODER QUIET\n","ERR ENCODER ARGUMENT");
    request("MOTOR ENCODER QUIET BAD\n","ERR ENCODER ARGUMENT");
    request("MOTOR ENCODER QUIET ON EXTRA\n","ERR ENCODER ARGUMENT");
    g_encoder_quiet_reason="BUSY";
    request("MOTOR ENCODER QUIET ON\n","ERR ENCODER BUSY\r\n");
    CHECK(!g_encoder_quiet);
    g_encoder_quiet_reason=NULL;
    request("MOTOR ENCODER HYST 5\n","OK ENCODER HYST=5 VOLATILE=1\r\n");
    CHECK(g_encoder_hyst==5U);
    request("MOTOR ENCODER HYST 7\n","OK ENCODER HYST=7 VOLATILE=1\r\n");
    CHECK(g_encoder_hyst==7U);
    request("MOTOR ENCODER HYST 3\n","ERR ENCODER RANGE");
    request("MOTOR ENCODER HYST 8\n","ERR ENCODER RANGE");
    request("MOTOR ENCODER HYST -1\n","ERR ENCODER RANGE");
    request("MOTOR ENCODER HYST BAD\n","ERR ENCODER RANGE");
    request("MOTOR ENCODER HYST\n","ERR ENCODER ARGUMENT");
    g_encoder_quiet_reason="BUSY";
    request("MOTOR ENCODER HYST 4\n","ERR ENCODER BUSY\r\n");
    CHECK(g_encoder_hyst==7U);
    g_encoder_quiet_reason=NULL;
    request("MOTOR RUN EXTRA\n","ERR MOTOR ARGUMENT");
    request("MOTOR UNKNOWN\n","ERR MOTOR ARGUMENT");
    request("STORAGE\n","OK STORAGE MOCK");
    request("START\n","ERR START SOURCE");
    request("SET BRIGHTNESS 0\n","ERR SET RANGE");
    request("SET BRIGHTNESS 101\n","ERR SET RANGE");
    request("SET BRIGHTNESS 9999999999999999999999999\n","ERR SET RANGE");
    request("SET BRIGHTNESS 8X\n","ERR SET ARGUMENT");
    request("SET ANIMATION -1\n","ERR SET ARGUMENT");
    request("SET ANIMATION 2\n","ERR SET RANGE");
    request("GET BRIGHTNESS EXTRA\n","ERR GET ARGUMENT");
    request("PING X\n","ERR PING ARGUMENT");
    request("GET MISSING\n","ERR GET ARGUMENT");
    request("UI WRONG\n","ERR UI UNAVAILABLE");
    request("SET BRIGHTNESS 51\n","OK SET BRIGHTNESS=51");
    request("SET ANIMATION 0\n","OK SET ANIMATION=0");
    request("GET BRIGHTNESS\n","OK GET BRIGHTNESS=51");
    request("GET ANIMATION\n","OK GET ANIMATION=0");
    CHECK(settings_get()->brightness==51U && settings_get()->animation==0U);
    request("STATUS\n","OK STATUS UPTIME_MS=");
    CHECK(strstr(g_output,"OLED=DISABLED OLED_ERRORS=0")!=NULL);
    CHECK(strstr(g_output,"ENC_ANGLE_MDEG=NA ENC_RPM=NA ENC_ANGLE_VALID=0 ENC_SPEED_VALID=0")!=NULL);
    request("ENCODER\n","OK ENCODER MOCK");
    request("MOTOR FOC MOTION 4294967295\n","OK FOC MOTION MOCK FIRST=4294967295");
    request("MOTOR FOC MOTION 4294967296\n","ERR FOC RANGE");
    request("ENCODER EXTRA\n","ERR ENCODER ARGUMENT");
    g_mock_encoder=(encoder_snapshot_t){.state=ENCODER_OK,.angle_valid=true,.speed_valid=true,.angle_mdeg=359999U,.rpm=-5400};
    request("STATUS\n","OK STATUS UPTIME_MS=");
    CHECK(strstr(g_output,"ENCODER=OK ENC_ANGLE_MDEG=359999 ENC_RPM=-5400 ENC_ANGLE_VALID=1 ENC_SPEED_VALID=1")!=NULL);
    g_mock_encoder=(encoder_snapshot_t){.state=ENCODER_WEAK_FIELD,.status=2U,.angle_mdeg=123456U,.rpm=999};
    request("STATUS\n","OK STATUS UPTIME_MS=");
    CHECK(strstr(g_output,"ENCODER=WEAK_FIELD ENC_ANGLE_MDEG=NA ENC_RPM=NA ENC_ANGLE_VALID=0 ENC_SPEED_VALID=0")!=NULL);
    request("HELP\n","OK HELP COMMANDS=");
    CHECK(strstr(g_output,",UI,")==NULL);
    const char *removed[]={"UI\n","UI HOME\n","UI CLICK\n","UI LONG\n","UI UP\n","UI DOWN\n"};
    for(unsigned i=0;i<sizeof(removed)/sizeof(removed[0]);++i)
    {
        uint32_t revision=control_model_revision();
        request(removed[i],"ERR UI UNAVAILABLE");
        CHECK(control_model_pending()==NULL && control_model_revision()==revision);
    }
    /* 按压只记诊断，不能留下启动、停止或参数请求。 */
    g_raw=6U; run(25); g_raw=7U; run(25);
    CHECK(control_model_pending()==NULL);
    request("INPUT\n","OK INPUT RAW=7");
    CHECK(strstr(g_output,"LOCAL=1,0,0,0")!=NULL);
    CHECK(strstr(g_output,"REMOTE=0,0,0,0")!=NULL);
    CHECK(!interaction_pending());
    /* 去掉显示后仍必须捕获选择，供原Flash保存服务使用。 */
    request("SET ALGORITHM SENSORLESS_FOC\n","OK SET ALGORITHM=SENSORLESS_FOC");
    CHECK(settings_get()->algorithm==CONTROL_SENSORLESS_FOC);
    request("SET MODE TORQUE\n","OK SET MODE=TORQUE");
    CHECK(settings_get()->mode==CONTROL_TORQUE);
    request("SET MODE SPEED\n","OK SET MODE=SPEED");
    request("SET MODE CURRENT\n","ERR SET ARGUMENT");
    request("SET ALGORITHM ENCODER_FOC\n","OK SET ALGORITHM=ENCODER_FOC");
    request("SET MODE CURRENT\n","OK SET MODE=CURRENT");
    settings_publish();
    request("GET TARGET\n","OK GET TARGET=0 DECIMALS=3 UNIT=A MIN=-500 MAX=500 EDITABLE=1");
    request("SET TARGET -501\n","ERR SET RANGE");
    request("SET TARGET 501\n","ERR SET RANGE");
    request("GET SPEED_KP\n","ERR GET NOT_READY");
    request("GET CURRENT_LIMIT\n","OK GET CURRENT_LIMIT=500 DECIMALS=3 UNIT=A");
    request("SET CURRENT_LIMIT 800\n","ERR SET NOT_READY");
    request("SET ALGORITHM SENSORLESS_FOC\n","OK SET ALGORITHM=SENSORLESS_FOC");
    request("GET MODE\n","OK GET MODE=SPEED");
}
/** @brief 检查串口排队的测试配置并模拟后端结果。
 * @param sector 零基扇区。 @param cycle 整轮标志。 @param reverse 反向标志。 @param dwell_ms 通电ms。 */
static void
complete_step(uint8_t sector, bool cycle, bool reverse, uint32_t dwell_ms)
{
    control_request_t operation;
    CHECK(control_model_take_request(&operation)); CHECK(operation.kind==CONTROL_REQUEST_STEP_TEST);
    CHECK(operation.step_test.sector==sector && operation.step_test.cycle==cycle &&
        operation.step_test.reverse==reverse && operation.step_test.dwell_ms==dwell_ms);
    CHECK(control_model_complete(operation.sequence,true,NULL));
}
/** @brief 验证独立六步串口语法、来源、状态和紧急停止覆盖。 */
static void
test_step_protocol(void)
{
    interaction_init(true);
    request("HELP\n","OK HELP COMMANDS="); CHECK(strstr(g_output,",STEP FOC_PRINT=OFF,TEXT,FIREWATER,TRACE,BINARY\r\n")!=NULL);
    request("STEP STATUS\n","OK STEP STATUS MOCK");
    request("STEP\n","ERR STEP ARGUMENT");
    request("STEP STATUS EXTRA\n","ERR STEP ARGUMENT");
    request("STEP 1\n","ERR STEP ARGUMENT");
    request("STEP RUN\n","ERR STEP ARGUMENT");
    request("STEP 1 200 FWD\n","ERR STEP ARGUMENT");
    request("STEP RUN 200 FWD EXTRA\n","ERR STEP ARGUMENT");
    request("STEP RUN 200 OTHER\n","ERR STEP ARGUMENT");
    request("STEP 0 200\n","ERR STEP RANGE");
    request("STEP 7 200\n","ERR STEP RANGE");
    request("STEP 1 19\n","ERR STEP RANGE");
    request("STEP RUN 1001\n","ERR STEP RANGE");
    request("STEP RUN 999999999999999999999\n","ERR STEP RANGE");
    request("STEP 1 -200\n","ERR STEP ARGUMENT");
    request("STEP RUN 20X\n","ERR STEP ARGUMENT");
    request("STEP 1 200\n","ERR STEP SOURCE");
    control_snapshot_t s=*control_model_snapshot();
    s.source=CONTROL_REMOTE; s.updated_ms=g_now; CHECK(control_model_publish(&s));
    request("STEP 1 200\n","ERR STEP NOT_READY");
    s.ready=true; s.run=CONTROL_STOPPED; s.power_disabled=true;
    s.capabilities=CONTROL_CAP_STEP_TEST|CONTROL_CAP_START|CONTROL_CAP_STOP|CONTROL_CAP_SOURCE|CONTROL_CAP_IDENTIFY;
    s.updated_ms=g_now; CHECK(control_model_publish(&s));
    for (unsigned sector=1;sector<=6U;++sector)
    {
        char text[32]; snprintf(text,sizeof(text),"STEP %u 20\n",sector);
        request(text,"OK STEP PENDING"); complete_step((uint8_t)(sector-1U),false,false,20);
    }
    request("STEP RUN 1000\n","OK STEP PENDING"); complete_step(0,true,false,1000);
    request("STEP RUN 200 FWD\n","OK STEP PENDING"); complete_step(0,true,false,200);
    request("step run 200 rev\n","OK STEP PENDING"); complete_step(0,true,true,200);
    request("GET RESULT\n","OK GET RESULT=OK");
    request("STEP 6 200\n","OK STEP PENDING");
    request("STEP 2 200\n","ERR STEP BUSY");
    request("STOP\n","OK STOP PENDING");
    control_request_t operation; CHECK(control_model_take_request(&operation));
    CHECK(operation.kind==CONTROL_REQUEST_STOP && operation.step_test.dwell_ms==0U);
    CHECK(control_model_complete(operation.sequence,true,NULL));
    s.updated_ms=g_now; CHECK(control_model_publish(&s));
    request("STEP RUN 200\n","OK STEP PENDING");
    request("!\n","OK STOP PENDING"); CHECK(control_model_take_request(&operation));
    CHECK(operation.kind==CONTROL_REQUEST_STOP); CHECK(control_model_complete(operation.sequence,true,NULL));
    s.updated_ms=g_now; CHECK(control_model_publish(&s));
    request("STEP 1 20\n","OK STEP PENDING"); CHECK(control_model_take_request(&operation));
    CHECK(control_model_complete(operation.sequence,false,"DRV/ADC CAL"));
    request("GET RESULT\n","OK GET RESULT=DRV/ADC CAL");
    s.run=CONTROL_RUNNING; s.updated_ms=g_now; CHECK(control_model_publish(&s));
    request("STEP RUN 200\n","ERR STEP BUSY");
    s.run=CONTROL_STOPPED; s.power_disabled=false; s.updated_ms=g_now; CHECK(control_model_publish(&s));
    request("STEP RUN 200\n","ERR STEP BUSY");
    s.power_disabled=true; s.capabilities&=~CONTROL_CAP_STEP_TEST; s.updated_ms=g_now; CHECK(control_model_publish(&s));
    request("STEP RUN 200\n","ERR STEP NOT_READY");
    s.capabilities|=CONTROL_CAP_STEP_TEST; s.updated_ms=g_now-CONTROL_DATA_TTL_MS-1U; CHECK(control_model_publish(&s));
    request("STEP RUN 200\n","ERR STEP NOT_READY");
    s.updated_ms=g_now; CHECK(control_model_publish(&s));
    request("SET SOURCE LOCAL\n","OK SET PENDING"); CHECK(control_model_take_request(&operation));
    CHECK(control_model_complete(operation.sequence,true,NULL)); request("STEP 1 20\n","ERR STEP SOURCE");
    request("STEP STATUS\n","OK STEP STATUS MOCK");
}
/** @brief 旧直启命令统一拒绝，不排队、不改变功率状态，也不覆盖已有普通启动。 */
static void
test_removed_direct_protocol(void)
{
    interaction_init(true);
    request("HELP\n","OK HELP COMMANDS=");
    CHECK(strstr(g_output,",STEP FOC_PRINT=OFF,TEXT,FIREWATER,TRACE,BINARY\r\n")!=NULL && strstr(g_output,"DIRECT")==NULL);
    control_snapshot_t s=*control_model_snapshot();
    request("DIRECT RUN\n","ERR DIRECT UNKNOWN");
    CHECK(control_model_pending()==NULL && memcmp(&s,control_model_snapshot(),sizeof(s))==0);
    s.ready=true; s.run=CONTROL_STOPPED; s.power_disabled=true; s.source=CONTROL_REMOTE;
    s.capabilities=CONTROL_CAP_START|CONTROL_CAP_STEP_TEST|CONTROL_CAP_STOP|CONTROL_CAP_SOURCE;
    s.updated_ms=g_now; CHECK(control_model_publish(&s));
    const char *commands[]={"DIRECT\n","DIRECT STATUS\n","DIRECT RUN\n","direct\trun\r\n","DIRECT RUN EXTRA\n"};
    for(unsigned i=0;i<sizeof(commands)/sizeof(commands[0]);++i)
    {
        uint32_t revision=control_model_revision();
        request(commands[i],"ERR DIRECT UNKNOWN");
        control_request_t operation;
        CHECK(control_model_pending()==NULL && !control_model_take_request(&operation));
        CHECK(control_model_revision()==revision);
        CHECK(control_model_snapshot()->run==CONTROL_STOPPED && control_model_snapshot()->power_disabled);
    }
    /* 普通 START 的请求槽已占用时，旧命令也不能修改或替换它。 */
    CHECK(control_model_request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0,g_now)==CONTROL_OK);
    control_request_t queued=*control_model_pending();
    request("DIRECT RUN\n","ERR DIRECT UNKNOWN");
    CHECK(control_model_pending()!=NULL && memcmp(&queued,control_model_pending(),sizeof(queued))==0);
    CHECK(control_model_snapshot()->run==CONTROL_STOPPED && control_model_snapshot()->power_disabled);
    control_request_t operation; CHECK(control_model_take_request(&operation));
    CHECK(operation.kind==CONTROL_REQUEST_START && control_model_complete(operation.sequence,true,NULL));
}
/** @brief 验证真实参数表发布的50 Hz初频端点与串口范围检查，越界不得留下请求。 */
static void
test_start_frequency_protocol_range(void)
{
    interaction_init(true);
    const control_parameter_t *parameter=control_model_parameter(CONTROL_PARAM_START_HZ);
    CHECK(parameter->valid && parameter->editable && parameter->maximum==50000 && parameter->decimals==3);
    CHECK(parameter->value==MOTOR_START_DEFAULT_MHZ);
    control_snapshot_t s=*control_model_snapshot();
    s.capabilities=CONTROL_CAP_PARAMETERS; s.run=CONTROL_STOPPED; s.updated_ms=g_now;
    CHECK(control_model_publish(&s));
    request("SET START_HZ 50001\n","ERR SET RANGE"); CHECK(control_model_pending()==NULL);
    request("SET START_HZ 50000\n","OK SET PENDING");
    control_request_t operation; CHECK(control_model_take_request(&operation));
    CHECK(operation.kind==CONTROL_REQUEST_PARAMETER && operation.parameter==CONTROL_PARAM_START_HZ && operation.value==50000);
    CHECK(control_model_complete(operation.sequence,true,NULL));
    request("GET START_HZ\n","OK GET START_HZ=50000 DECIMALS=3 UNIT=HZ");
    request("SET START_HZ 99\n","ERR SET RANGE"); CHECK(control_model_pending()==NULL);
    request("SET START_HZ 50001\n","ERR SET RANGE"); CHECK(control_model_pending()==NULL);
    CHECK(control_model_parameter(CONTROL_PARAM_START_HZ)->value==50000);
}
/** @brief RUN串口单命令预算、STOP及新增诊断分派回归。 */
static void
test_running_service_budget(void)
{
    interaction_init(true);
    CHECK(control_model_select(CONTROL_SENSORLESS_6STEP,CONTROL_SPEED)==CONTROL_OK);
    control_snapshot_t s=*control_model_snapshot();
    s.ready=true; s.run=CONTROL_RUNNING; s.power_disabled=false; s.updated_ms=g_now;
    s.capabilities=CONTROL_CAP_STOP|CONTROL_CAP_PARAMETERS;
    CHECK(control_model_publish(&s));
    for (unsigned i=0U;i<200U;++i) { ++g_now; interaction_service(); }
    request("MOTOR TIMING\n","ERR RUNNING STOP TO READ");
    request("MOTOR BEMF SECTOR 6 3\n","ERR RUNNING STOP TO READ");
    request("STATUS\n","ERR RUNNING STOP TO READ");
    request("ENCODER\n","ERR RUNNING STOP TO READ");
    request("STORAGE\n","ERR RUNNING STOP TO READ");
    g_output_length=0U; g_output[0]='\0';
    const char *two="PING\nPING\n";
    for (const char *p=two;*p;++p) { serial_buffer_receive((uint8_t)*p,false); }
    interaction_service();
    uint8_t byte;
    while (serial_buffer_transmit(&byte)) { g_output[g_output_length++]=(char)byte; }
    g_output[g_output_length]='\0';
    CHECK(strcmp(g_output,"OK PING\r\n")==0 && serial_buffer_rx_pending());
    interaction_service();
    while (serial_buffer_transmit(&byte)) { g_output[g_output_length++]=(char)byte; }
    g_output[g_output_length]='\0';
    CHECK(strcmp(g_output,"OK PING\r\nOK PING\r\n")==0 && !serial_buffer_rx_pending());
    request("STOP\n","OK STOP PENDING");
    control_request_t operation;
    CHECK(control_model_take_request(&operation) && operation.kind==CONTROL_REQUEST_STOP);
    CHECK(control_model_complete(operation.sequence,true,NULL));
    s.run=CONTROL_STOPPED; s.power_disabled=true; s.updated_ms=g_now;
    CHECK(control_model_publish(&s));
    request("MOTOR TIMING\n","OK MOTOR TIMING MOCK");
    request("MOTOR BEMF SECTOR 6 3\n","OK MOTOR BEMF SECTOR=6 PAGE=3 MOCK");
    request("MOTOR BEMF SECTOR 0 0\n","ERR MOTOR RANGE");
    request("MOTOR BEMF SECTOR 6 4\n","ERR MOTOR RANGE");
    for (unsigned i=0U;i<30U;++i) { ++g_now; interaction_service(); }
    CHECK(!interaction_pending());
}

/** @brief 最大计数与最长编码器字段组合下，整行状态必须保留最后的有效性字段。 */
static void
test_encoder_status_capacity(void)
{
    interaction_init(true);
    uint32_t saved_now = g_now;
    board_uart_stats_t saved_uart = g_uart;
    g_now = UINT32_MAX - 100U;
    g_uart = (board_uart_stats_t){UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX};
    g_mock_encoder = (encoder_snapshot_t){.state=ENCODER_SENSOR_FAULT,.angle_mdeg=UINT32_MAX,.rpm=INT32_MIN,.angle_valid=true,.speed_valid=true};
    request("STATUS\n","OK STATUS UPTIME_MS=");
    CHECK(strstr(g_output,"ENC_ANGLE_MDEG=4294967295 ENC_RPM=-2147483648 ENC_ANGLE_VALID=1 ENC_SPEED_VALID=1\r\n")!=NULL);
    CHECK(strlen(g_output)<CONSOLE_RESPONSE_MAX);
    g_mock_encoder=(encoder_snapshot_t){0};
    g_now=saved_now;
    g_uart=saved_uart;
}

/** @brief 验证打印默认关闭、运行中切换、非法参数、复位恢复及命令优先级。 */
static void
test_foc_idle_current(void)
{
    interaction_init(true);
    CHECK(control_model_select(CONTROL_SENSORLESS_FOC,CONTROL_SPEED)==CONTROL_OK);
    control_snapshot_t s=*control_model_snapshot();
    s.run=CONTROL_RUNNING; s.power_disabled=false; s.updated_ms=g_now;
    s.capabilities=CONTROL_CAP_STOP;
    CHECK(control_model_publish(&s));
    g_mock_foc_current=true; g_mock_foc_reads=0U;
    g_mock_foc_sample=1U; g_mock_foc_sent=0U;
    interaction_service();
    interaction_service();
    CHECK(g_mock_foc_reads==0U);
    request("GET FOC_PRINT\n","OK GET FOC_PRINT=OFF\r\n");
    request("SET FOC_PRINT UNKNOWN\n","ERR SET ARGUMENT\r\n");
    CHECK(NULL==control_model_pending());
    /* 启用响应与第一条样本分开检查，避免把实时行算作额外命令应答。 */
    g_mock_foc_current=false;
    request("set foc_print text\n","OK SET FOC_PRINT=TEXT\r\n");
    g_mock_foc_current=true;
    interaction_service();
    interaction_service();
    CHECK(g_mock_foc_reads==1U);
    uint8_t byte;
    while (serial_buffer_transmit(&byte)) { }
    /* 同一毫秒最新值改变就发送下一组，不等待40ms。 */
    g_mock_foc_sample=2U;
    interaction_service();
    CHECK(g_mock_foc_reads==2U);
    /* 剩余空间不能容纳电流行加完整命令响应时，不读取或等待。 */
    while (serial_buffer_transmit(&byte)) { }
    char busy[600];
    memset(busy,'B',sizeof(busy));
    CHECK(serial_buffer_write(busy,sizeof(busy)));
    g_mock_foc_sample=3U;
    interaction_service();
    CHECK(g_mock_foc_reads==2U);
    while (serial_buffer_transmit(&byte)) { }
    /* OFF在本轮处理后即停止产生电流行，不消费期间跳过的样本。 */
    request("SET FOC_PRINT OFF\n","OK SET FOC_PRINT=OFF\r\n");
    interaction_service();
    CHECK(g_mock_foc_reads==2U);
    g_mock_foc_sample=100U;
    request("GET FOC_PRINT\n","OK GET FOC_PRINT=OFF\r\n");
    request("SET FOC_PRINT\n","ERR SET ARGUMENT\r\n");
    request("SET FOC_PRINT FIREWATER EXTRA\n","ERR SET ARGUMENT\r\n");
    CHECK(g_mock_foc_reads==2U && NULL==control_model_pending());
    g_mock_foc_current=false;
    request("SET FOC_PRINT FIREWATER\n","OK SET FOC_PRINT=FIREWATER\r\n");
    request("GET FOC_PRINT\n","OK GET FOC_PRINT=FIREWATER\r\n");
    request("SET FOC_PRINT INVALID\n","ERR SET ARGUMENT\r\n");
    request("GET FOC_PRINT\n","OK GET FOC_PRINT=FIREWATER\r\n");
    g_mock_foc_current=true;
    interaction_service();
    g_output_length=0U;
    while (serial_buffer_transmit(&byte)) { g_output[g_output_length++]=(char)byte; }
    g_output[g_output_length]='\0';
    CHECK(strcmp(g_output,"FOC:1,2,-3,-4\r\n")==0 && g_mock_foc_reads==3U);
    /* 新模式有独立协议名，切换和查询不产生旧样本重发。 */
    request("SET FOC_PRINT TRACE\n","OK SET FOC_PRINT=TRACE\r\n");
    request("GET FOC_PRINT\n","OK GET FOC_PRINT=TRACE\r\n");
    request("MOTOR FOC HOLD_IQ 120\n","OK FOC HOLD_IQ\r\n");
    request("MOTOR FOC HOLD_IQ 501\n","ERR FOC RANGE\r\n");
    request("MOTOR FOC HOLD_IQ 0\n","OK FOC HOLD_IQ\r\n");
    request("MOTOR FOC HARMONIC ON\n","OK FOC HARMONIC\r\n");
    request("MOTOR FOC HARMONIC OFF\n","OK FOC HARMONIC\r\n");
    request("MOTOR FOC HARMONIC INVALID\n","ERR FOC ARGUMENT\r\n");
    request("MOTOR FOC HARMONIC2 ON\n","OK FOC HARMONIC2\r\n");
    request("MOTOR FOC HARMONIC2 OFF\n","OK FOC HARMONIC2\r\n");
    request("MOTOR FOC HARMONIC2 INVALID\n","ERR FOC ARGUMENT\r\n");
    request("MOTOR FOC HARMONIC2 ON EXTRA\n","ERR FOC ARGUMENT\r\n");
    request("MOTOR FOC ZERO 500\n","OK FOC ZERO NO_OUTPUT\r\n");
    request("MOTOR FOC ZERO 501\n","ERR FOC RANGE\r\n");
    request("MOTOR FOC TRIGGER\n","OK FOC TRIGGER=8400\r\n");
    request("MOTOR FOC TRIGGER 8300\n","OK FOC TRIGGER\r\n");
    request("MOTOR FOC TRIGGER 8299\n","ERR FOC RANGE\r\n");
    request("MOTOR FOC HANDOFF\n","OK FOC HANDOFF_RPM=1000\r\n");
    request("MOTOR FOC HANDOFF 600\n","OK FOC HANDOFF\r\n");
    request("MOTOR FOC HANDOFF 0\n","ERR FOC RANGE\r\n");
    request("MOTOR FOC HANDOFF 1001\n","ERR FOC RANGE\r\n");
    g_mock_foc_sample=101U;
    interaction_service();
    g_output_length=0U;
    while(serial_buffer_transmit(&byte)) { g_output[g_output_length++]=(char)byte; }
    g_output[g_output_length]='\0';
    CHECK(strcmp(g_output,"FOC_TRACE:101,8436,1,6\r\n")==0 && g_mock_foc_reads==4U);
    g_mock_foc_reads=3U;
    g_mock_foc_sample=100U;g_mock_foc_sent=100U;
    request("SET FOC_PRINT FIREWATER\n","OK SET FOC_PRINT=FIREWATER\r\n");
    /* 波形模式也只发送最新值，没有补发OFF期间的历史。 */
    interaction_service();
    CHECK(g_mock_foc_reads==3U && !serial_buffer_transmit(&byte));
    g_mock_foc_sample=101U;
    inject("PI");
    interaction_service();
    CHECK(g_mock_foc_reads==3U && !serial_buffer_transmit(&byte));
    inject("NG\n");
    /* 本轮命令响应先于最新波形进入TX，二者均保留完整换行。 */
    interaction_service();
    g_output_length=0U;
    while (serial_buffer_transmit(&byte)) { g_output[g_output_length++]=(char)byte; }
    g_output[g_output_length]='\0';
    CHECK(strcmp(g_output,"OK PING\r\nFOC:1,2,-3,-4\r\n")==0 && g_mock_foc_reads==4U);
    g_mock_foc_sample=102U;
    const char *stop="STOP\n";
    for (const char *p=stop;*p;++p) { serial_buffer_receive((uint8_t)*p,false); }
    interaction_service();
    CHECK(g_mock_foc_reads==4U);
    g_output_length=0U;
    while (serial_buffer_transmit(&byte)) { g_output[g_output_length++]=(char)byte; }
    g_output[g_output_length]='\0';
    CHECK(strcmp(g_output,"OK STOP PENDING\r\n")==0);
    control_request_t operation;
    CHECK(control_model_take_request(&operation) && operation.kind==CONTROL_REQUEST_STOP);
    CHECK(control_model_complete(operation.sequence,true,NULL));
    g_mock_foc_current=false;
    s.run=CONTROL_STOPPED; s.power_disabled=true; CHECK(control_model_publish(&s));
    interaction_init(true);
    request("GET FOC_PRINT\n","OK GET FOC_PRINT=OFF\r\n");
}

/** @brief 验证易失高速接口、带宽拒绝和测试期间START守卫。 */
static void test_wave_configuration(void)
{
    serial_buffer_init(); interaction_init(true); g_mock_uart_baud=921600U;
    control_snapshot_t snapshot=*control_model_snapshot();
    snapshot.run=CONTROL_STOPPED; snapshot.source=CONTROL_REMOTE;
    snapshot.power_disabled=true; snapshot.ready=true; snapshot.updated_ms=g_now;
    snapshot.capabilities=CONTROL_CAP_STOP;
    CHECK(control_model_publish(&snapshot));
    request("GET WAVE_CAPS\n","OK GET WAVE_CAPS=BIN1 RATES=5000,10000");
    request("GET WAVE_RATE\n","OK GET WAVE_RATE=5000\r\n");
    request("SET WAVE_RATE 10000\n","ERR SET BANDWIDTH\r\n");
    request("SET UART_BAUD 2000000\n","OK SET UART_BAUD=2000000\r\n");
    request("GET UART_BAUD\n","OK GET UART_BAUD=2000000\r\n");
    request("SET WAVE_RATE 10000\n","OK SET WAVE_RATE=10000\r\n");
    request("SET UART_BAUD 921600\n","ERR SET BANDWIDTH\r\n");
    request("SET FOC_PRINT BINARY\n","OK SET FOC_PRINT=BINARY\r\n");
    request("SET WAVE_RATE 5000\n","ERR SET BUSY\r\n");
    request("SET FOC_PRINT OFF\n","OK SET FOC_PRINT=OFF\r\n");
    request("MOTOR UART TEST 0\n","ERR MOTOR RANGE\r\n");
    request("MOTOR UART TEST 60001\n","ERR MOTOR RANGE\r\n");
    g_mock_still_delay=175U;
    request("MOTOR UART TEST 10000\n","OK MOTOR UART TEST MS=10000 SOURCE=TEST\r\n");
    g_mock_still_delay=0U;
    CHECK(wave_stream_stats().captured+wave_stream_stats().dropped<=500U);
    CHECK(wave_stream_testing());
    request("START\n","ERR START BUSY\r\n");
    request("STOP\n","OK STOP PENDING\r\n");
    CHECK(!wave_stream_testing());
    control_request_t operation;
    CHECK(control_model_take_request(&operation));
    CHECK(control_model_complete(operation.sequence,true,NULL));
    request("SET FOC_PRINT OFF\n","OK SET FOC_PRINT=OFF\r\n");
    request("SET WAVE_RATE 5000\n","OK SET WAVE_RATE=5000\r\n");
    request("SET UART_BAUD 921600\n","OK SET UART_BAUD=921600\r\n");
}

/** @brief 无显示依赖的串口集成测试入口。 @return 成功为0。 */
int
main(void)
{
    test_application();
    test_step_protocol();
    test_removed_direct_protocol();
    test_start_frequency_protocol_range();
    test_running_service_budget();
    test_foc_idle_current();
    test_encoder_status_capacity();
    test_wave_configuration();
    printf("serial protocol: %u checks passed\n",g_checks);
    return 0;
}

/* 通信硬件替身只验证命令路由；DMA寄存器由独立测试覆盖。 */
/** @brief 返回速率替身。 @return 当前波特率。 */
uint32_t board_uart_baud(void) { return g_mock_uart_baud; }
/** @brief 模拟170MHz计数。 @return 当前tick。 */
uint32_t board_uart_ticks(void) { return g_now*170000U; }
/** @brief 时序由独立DMA测试覆盖。 @return 当前无切换。 */
bool board_uart_baud_pending(void) { return false; }
/** @brief 确认入口替身，不改动控制。 */
void board_uart_confirm_baud(void) { }
/** @brief 验证允许的速率范围。 @param baud 请求值。 @return 已接受。 */
bool board_uart_request_baud(uint32_t baud)
{
    bool valid=baud==921600U || baud==1000000U || baud==2000000U || baud==3000000U;
    if(valid) { g_mock_uart_baud=baud; } return valid;
}
/** @brief 不在交互回归中发送波形。 @return false。 */
bool board_uart_wave_ready(void) { return false; }
/** @brief 返回固定替身缓冲。 @return 512字节缓冲。 */
uint8_t *board_uart_wave_buffer(void) { static uint8_t bytes[512]; return bytes; }
/** @brief 拒绝模拟DMA发送。 @param size 长度。 @return false。 */
bool board_uart_wave_submit(size_t size) { (void)size; return false; }
/** @brief 测试时功率始终关闭。 @return true。 */
bool board_motor_storage_safe(void) { return true; }
/** @brief 测试时转轴静止。 @return true。 */
bool board_motor_standstill(void) { g_now+=g_mock_still_delay; return true; }

/** @brief 手感协议完整配置替身。 */
const char *motor_backend_haptic_configure(const encoder_haptic_config_t *c)
{ return encoder_haptic_valid(c)?NULL:"RANGE"; }
/** @brief 手感协议只读状态替身。 */
void motor_backend_haptic(char *out,size_t size) { snprintf(out,size,"OK FOC HAPTIC"); }
