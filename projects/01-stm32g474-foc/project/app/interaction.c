#include "interaction.h"
#include "board_io.h"
#include "board_motor.h"
#include "wave_stream.h"
#include "board_encoder.h"
#include "switch_input.h"
#include "console.h"
#include "serial_buffer.h"
#include "control_model.h"
#include "settings.h"
#include "motor_backend.h"
#include <stdio.h>
#include <string.h>

/* 保留旧显示字段的协议与Flash兼容性，仅作为元数据，不驱动硬件。 */
static int32_t g_app_brightness = 50;
static bool g_app_animation = true;
/* 输入和命令解析均只由主循环持有。 */
static switch_input_t g_app_switch;
static console_t g_app_console;
/* 固定响应缓冲；只有发送容量充足才解析下一条命令。 */
static char g_app_response[CONSOLE_RESPONSE_MAX];
/* 空闲打印独立小缓冲；保留命令响应空间后立即发送最新电流。 */
static char g_app_foc_current[320];
/* 打印模式仅由主循环持有，不写Flash；复位后默认关闭。 */
typedef enum {
    /* 关闭实时电流打印，保留命令响应。 */
    FOC_PRINT_OFF,
    /* 原有DATA FOC详细文本。 */
    FOC_PRINT_TEXT,
    /* VOFA+ FireWater四通道电流波形。 */
    FOC_PRINT_FIREWATER,
    /* 同一最新样本的基准电流、角度和电压指令CSV。 */
    FOC_PRINT_TRACE,
    /* 固定节奏的COBS二进制块，与文本模式互斥。 */
    FOC_PRINT_BINARY
} foc_print_mode_t;
/* 当前会话的输出模式，初始化和串口命令均由主循环修改。 */
static foc_print_mode_t g_app_foc_print;
/* 模式查询与设置响应使用相同的协议名称。 */
static const char *const g_app_foc_print_names[] = {"OFF", "TEXT", "FIREWATER", "TRACE", "BINARY"};
/* 本地/串口事件分开累计，列顺序为CLICK、LONG、DOWN、UP。 */
static uint32_t g_app_events[2][4];
/* 诊断状态、更新周期和最大主循环间隔。 */
static uint32_t g_app_now, g_app_last_loop, g_app_gap_max;
static bool g_app_uart_online;
static const char *g_app_last_event = "NONE";
static const char *g_app_last_source = "NONE";
/** @brief 捕获已确认算法、模式及旧显示元数据，沿用原停机保存流程。 */
static void
capture_settings(void)
{
    settings_capture((uint32_t)g_app_brightness, g_app_animation, g_app_now);
}

/** @brief 记录四事件诊断，不产生菜单动作或电机请求。
 * @param event 四事件。 @param remote true表示串口来源。 */
static void
handle_event(switch_event_t event, bool remote)
{
    static const char *const names[] = {"NONE","CLICK","LONG","DOWN","UP"};
    if (event < SWITCH_CLICK || event > SWITCH_UP) { return; }
    ++g_app_events[remote ? 1U : 0U][event - 1U];
    g_app_last_event = names[event];
    g_app_last_source = remote ? "REMOTE" : "LOCAL";
}

/** @brief 严格解析无符号十进制设置。
 * @param text 参数。 @param maximum 最大值。 @param value 输出。
 * @return NULL为成功，否则为协议错误码。 */
static const char *
parse_value(const char *text, unsigned maximum, unsigned *value)
{
    unsigned result = 0U;
    bool range = false;
    if (*text == '\0') { return "ARGUMENT"; }
    for (; *text != '\0'; ++text)
    {
        if (*text < '0' || *text > '9') { return "ARGUMENT"; }
        if (!range)
        {
            unsigned digit=(unsigned)(*text-'0');
            /* 放开固定时长后可用完整整数范围，必须在乘十前判断溢出。 */
            if(result>maximum/10U || (result==maximum/10U && digit>maximum%10U)) { range=true; }
            else { result=result*10U+digit; }
        }
    }
    if (range) { return "RANGE"; }
    *value = result;
    return NULL;
}

/** @brief 执行文本协议V1，控制及诊断动作沿用现有后端接口。
 * @param argc 参数数。 @param argv 大写参数。 @param out 响应缓冲。 @param cap 容量。 */
static void
command(unsigned argc, char **argv, char *out, size_t cap)
{
    const char *name = argv[0];
    const char *error = "ARGUMENT";
    /* RUN只接收控制及短查询，完整诊断留在停机后；不让打印占用逐样本检测预算。 */
    if (control_model_snapshot()->run == CONTROL_RUNNING &&
        control_model_algorithm() == CONTROL_SENSORLESS_6STEP &&
        (strcmp(name,"MOTOR") == 0 || strcmp(name,"STATUS") == 0 ||
         strcmp(name,"ENCODER") == 0 || strcmp(name,"INPUT") == 0 || strcmp(name,"STORAGE") == 0 || strcmp(name,"HELP") == 0))
    {
        const char reply[] = "ERR RUNNING STOP TO READ";
        size_t count = sizeof(reply) < cap ? sizeof(reply) : cap;
        if (count != 0U) { memcpy(out, reply, count); out[count-1U] = '\0'; }
        return;
    }
    if (strcmp(name,"PING") == 0 && argc == 1U) { board_uart_confirm_baud(); snprintf(out,cap,"OK PING"); return; }
    if (strcmp(name,"VERSION") == 0 && argc == 1U)
    {
        snprintf(out,cap,"OK VERSION FW=%s PROTOCOL=1.5 ENCODER=MT6835",INTERACTION_VERSION); return;
    }
    if (strcmp(name,"HELP") == 0 && argc == 1U)
    {
        snprintf(out,cap,"OK HELP COMMANDS=PING,VERSION,HELP,STATUS,ENCODER,INPUT,GET,SET,START,STOP,CLEAR,MOTOR,STORAGE,STEP FOC_PRINT=OFF,TEXT,FIREWATER,TRACE,BINARY"); return;
    }
    if (strcmp(name,"STATUS") == 0 && argc == 1U)
    {
        board_uart_stats_t uart = board_uart_stats();
        encoder_snapshot_t encoder = board_encoder_snapshot(g_app_now);
        char angle[16] = "NA", speed[16] = "NA";
        if (encoder.angle_valid) { snprintf(angle,sizeof(angle),"%lu",(unsigned long)encoder.angle_mdeg); }
        if (encoder.speed_valid) { snprintf(speed,sizeof(speed),"%ld",(long)encoder.rpm); }
        /* 旧字段仍可被串口工具读取，明确表示本固件未启用显示支持。 */
        snprintf(out,cap,"OK STATUS UPTIME_MS=%lu OLED=DISABLED OLED_ERRORS=0 UART=%s RX=%lu TX=%lu UART_ERRORS=%lu RX_OVERFLOW=%lu GAP_MAX_MS=%lu IRQ_MAX_CYCLES=%lu MOTOR=%s ALGORITHM=%s MODE=%s READY=%u ENCODER=%s ENC_ANGLE_MDEG=%s ENC_RPM=%s ENC_ANGLE_VALID=%u ENC_SPEED_VALID=%u",
            (unsigned long)g_app_now,g_app_uart_online ? "ONLINE" : "OFFLINE",
            (unsigned long)uart.rx,(unsigned long)uart.tx,(unsigned long)uart.errors,
            (unsigned long)serial_buffer_overflows(),(unsigned long)g_app_gap_max,(unsigned long)uart.irq_max_cycles,control_model_snapshot()->run==CONTROL_RUNNING?"RUNNING":control_model_snapshot()->run==CONTROL_FAULT?"FAULT":"STOPPED",control_algorithm_token(control_model_algorithm()),control_mode_name(control_model_mode()),control_model_snapshot()->ready?1U:0U,
            encoder_state_name(encoder.state),angle,speed,encoder.angle_valid?1U:0U,encoder.speed_valid?1U:0U);
        return;
    }
    if (strcmp(name,"ENCODER") == 0 && argc == 1U)
    {
        board_encoder_diagnostics(out,cap,g_app_now);
        return;
    }
    if (strcmp(name,"INPUT") == 0 && argc == 1U)
    {
        snprintf(out,cap,"OK INPUT RAW=%u ACTIVE=%u LAST=%s SOURCE=%s LOCAL=%lu,%lu,%lu,%lu REMOTE=%lu,%lu,%lu,%lu ILLEGAL=%lu",
            g_app_switch.raw,g_app_switch.stable,g_app_last_event,g_app_last_source,
            (unsigned long)g_app_events[0][0],(unsigned long)g_app_events[0][1],
            (unsigned long)g_app_events[0][2],(unsigned long)g_app_events[0][3],
            (unsigned long)g_app_events[1][0],(unsigned long)g_app_events[1][1],
            (unsigned long)g_app_events[1][2],(unsigned long)g_app_events[1][3],(unsigned long)g_app_switch.invalid_transitions);
        return;
    }
    if (strcmp(name,"MOTOR") == 0)
    {
        if(argc>=3U && strcmp(argv[1],"UART")==0 && strcmp(argv[2],"TEST")==0)
        {
            unsigned duration=10000U;
            const char *reason=argc==4U?parse_value(argv[3],60000U,&duration):NULL;
            if(argc>4U || NULL!=reason || duration==0U)
            { snprintf(out,cap,"ERR MOTOR RANGE"); return; }
            if(control_model_snapshot()->run!=CONTROL_STOPPED || control_model_pending()!=NULL ||
                g_app_foc_print!=FOC_PRINT_OFF || board_uart_baud_pending() ||
                !board_motor_storage_safe() || !board_motor_standstill())
            { snprintf(out,cap,"ERR MOTOR UART BUSY OR NOT STILL"); return; }
            /* 静止ADC检查会等待，测试起点须在检查完成后重新读取。 */
            g_app_now=board_millis();
            wave_stream_test_start(board_uart_ticks(),g_app_now,duration);
            g_app_foc_print=FOC_PRINT_BINARY;
            snprintf(out,cap,"OK MOTOR UART TEST MS=%u SOURCE=TEST",duration); return;
        }
        if (argc>=2U && strcmp(argv[1],"ENCODER")==0)
        {
            if (argc==2U) { motor_backend_encoder_diagnostics(out,cap); return; }
            if(argc==4U && strcmp(argv[2],"HYST")==0)
            {
                unsigned code;
                const char *err=parse_value(argv[3],7U,&code);
                if(NULL!=err || code<4U) { snprintf(out,cap,"ERR ENCODER RANGE"); return; }
                const char *reason=motor_backend_encoder_hysteresis((uint8_t)code);
                if(NULL==reason) { snprintf(out,cap,"OK ENCODER HYST=%u VOLATILE=1",code); }
                else { snprintf(out,cap,"ERR ENCODER %s",reason); }
                return;
            }
            if (argc==4U && strcmp(argv[2],"QUIET")==0)
            {
                bool enabled=strcmp(argv[3],"ON")==0;
                if (!enabled && strcmp(argv[3],"OFF")!=0) { snprintf(out,cap,"ERR ENCODER ARGUMENT"); return; }
                const char *reason=motor_backend_encoder_quiet(enabled);
                if (reason==NULL) { snprintf(out,cap,"OK ENCODER QUIET=%s",enabled?"ON":"OFF"); }
                else { snprintf(out,cap,"ERR ENCODER %s",reason); }
                return;
            }
            if (argc==3U && strcmp(argv[2],"CAL")==0)
            {
                const char *reason=motor_backend_encoder_calibrate();
                if (reason==NULL) { snprintf(out,cap,"OK ENCODER CAL RUNNING"); }
                else { snprintf(out,cap,"ERR ENCODER %s",reason); }
                return;
            }
            snprintf(out,cap,"ERR ENCODER ARGUMENT"); return;
        }
        if (argc >= 2U && strcmp(argv[1],"FOC") == 0)
        {
            if (argc == 2U) { motor_backend_foc_diagnostics(out,cap,-2); return; }
            if(argc>=3U && strcmp(argv[2],"HAPTIC")==0)
            {
                if(argc==3U) { motor_backend_haptic(out,cap); return; }
                const char *reason="ARGUMENT";
                if(argc==8U && strcmp(argv[3],"CONFIG")==0)
                {
                    encoder_haptic_config_t c={0}; unsigned n,strength,damping;
                    c.kind=strcmp(argv[4],"DETENT")==0?0:strcmp(argv[4],"DAMPED")==0?1:-1;
                    reason=parse_value(argv[5],256U,&n);
                    if(NULL==reason) { reason=parse_value(argv[6],800U,&strength); }
                    if(NULL==reason) { reason=parse_value(argv[7],10000U,&damping); }
                    if(NULL==reason)
                    { c.detents=(int32_t)n;c.detent_ma=(int32_t)strength;c.damping_ua_rpm=(int32_t)damping;reason=motor_backend_haptic_configure(&c); }
                }
                if(NULL==reason) { motor_backend_haptic(out,cap); }
                else { snprintf(out,cap,"ERR FOC %s",reason); }
                return;
            }
            if(argc>=3U && strcmp(argv[2],"POSITION")==0)
            {
                if(argc==3U) { motor_backend_foc_position(out,cap); return; }
                const char *reason="ARGUMENT";
                if(argc==4U && strcmp(argv[3],"ZERO")==0) { reason=motor_backend_position_zero(); }
                if(argc==5U && strcmp(argv[3],"TOLERANCE")==0)
                {
                    unsigned value;
                    reason=parse_value(argv[4],500U,&value);
                    if(NULL==reason) { reason=motor_backend_position_tolerance((int32_t)value); }
                }
                if(NULL==reason) { snprintf(out,cap,"OK FOC POSITION %s",argv[3]); }
                else { snprintf(out,cap,"ERR FOC %s",reason); }
                return;
            }
            if(argc==4U && strcmp(argv[2],"MOTION")==0)
            {
                unsigned first;
                const char *err=parse_value(argv[3],UINT32_MAX,&first);
                if(NULL!=err) { snprintf(out,cap,"ERR FOC %s",err); return; }
                motor_backend_foc_motion(out,cap,first); return;
            }
            if(argc==3U && strcmp(argv[2],"OBSERVER")==0) { motor_backend_foc_diagnostics(out,cap,-4); return; }
            if(argc==3U && strcmp(argv[2],"DRIVE")==0) { motor_backend_foc_diagnostics(out,cap,-5); return; }
            if(argc==3U && strcmp(argv[2],"REST")==0) { motor_backend_foc_diagnostics(out,cap,-6); return; }
            if(argc==3U && strcmp(argv[2],"CONFIDENCE")==0) { motor_backend_foc_diagnostics(out,cap,-7); return; }
            if(argc>=3U && (strcmp(argv[2],"HARMONIC")==0 || strcmp(argv[2],"HARMONIC2")==0))
            {
                bool second=strcmp(argv[2],"HARMONIC2")==0;
                if (argc==3U) { motor_backend_foc_diagnostics(out,cap,second?-9:-8); return; }
                if (argc!=4U) { snprintf(out,cap,"ERR FOC ARGUMENT"); return; }
                bool enabled=strcmp(argv[3],"ON")==0;
                if(!enabled && strcmp(argv[3],"OFF")!=0) { snprintf(out,cap,"ERR FOC ARGUMENT"); return; }
                bool accepted=second?motor_backend_foc_harmonic2(enabled):motor_backend_foc_harmonic(enabled);
                snprintf(out,cap,"%s FOC %s",accepted?"OK":"ERR",argv[2]); return;
            }
            if(argc==3U && strcmp(argv[2],"HANDOFF")==0)
            {
                snprintf(out,cap,"OK FOC HANDOFF_RPM=%lu",(unsigned long)motor_backend_foc_handoff_rpm()); return;
            }
            if(argc==4U && strcmp(argv[2],"HANDOFF")==0)
            {
                unsigned rpm;
                const char *err=parse_value(argv[3],1000U,&rpm);
                if(NULL!=err || rpm<1U) { snprintf(out,cap,"ERR FOC RANGE"); return; }
                snprintf(out,cap,"%s FOC HANDOFF",motor_backend_foc_handoff(rpm)?"OK":"ERR"); return;
            }
            if(argc==3U && strcmp(argv[2],"TRIGGER")==0)
            {
                snprintf(out,cap,"OK FOC TRIGGER=%lu",(unsigned long)motor_backend_foc_trigger_ticks()); return;
            }
            if(argc==4U && strcmp(argv[2],"TRIGGER")==0)
            {
                unsigned trigger;
                const char *err=parse_value(argv[3],8470U,&trigger);
                if(NULL!=err || trigger<8300U) { snprintf(out,cap,"ERR FOC RANGE"); return; }
                snprintf(out,cap,"%s FOC TRIGGER",motor_backend_foc_trigger(trigger)?"OK":"ERR"); return;
            }
            if(argc==4U && strcmp(argv[2],"ZERO")==0)
            {
                unsigned duration;
                const char *err=parse_value(argv[3],500U,&duration);
                if(NULL!=err || duration<20U) { snprintf(out,cap,"ERR FOC RANGE"); return; }
                snprintf(out,cap,"%s FOC ZERO NO_OUTPUT",motor_backend_foc_zero(duration)?"OK":"ERR"); return;
            }
            if(argc==4U && strcmp(argv[2],"HOLD_IQ")==0)
            {
                unsigned current;
                const char *err=parse_value(argv[3],500U,&current);
                if(NULL!=err) { snprintf(out,cap,"ERR FOC RANGE"); return; }
                snprintf(out,cap,"%s FOC HOLD_IQ",motor_backend_foc_hold_iq(current)?"OK":"ERR"); return;
            }
            if(argc==5U && strcmp(argv[2],"CLOSED")==0)
            {
                unsigned target,duration;
                const char *err=parse_value(argv[3],5400U,&target);
                if(NULL==err) { err=parse_value(argv[4],32000U,&duration); }
                if(NULL!=err || target<1U || duration<500U) { snprintf(out,cap,"ERR FOC RANGE"); return; }
                snprintf(out,cap,"%s FOC CLOSED BENCH",motor_backend_foc_closed(target,duration)?"OK":"ERR"); return;
            }
            if (argc == 3U && strcmp(argv[2],"VOLTAGE") == 0)
            {
                motor_backend_foc_diagnostics(out,cap,-1); return;
            }
            if (argc == 3U && strcmp(argv[2],"CURRENT") == 0)
            {
                motor_backend_foc_diagnostics(out,cap,-3); return;
            }
            if (argc == 5U && (strcmp(argv[2],"CURRENT") == 0 || strcmp(argv[2],"CURRENTQ") == 0))
            {
                unsigned target, duration;
                const char *err=parse_value(argv[3],300U,&target);
                if (NULL == err) { err=parse_value(argv[4],500U,&duration); }
                if (NULL != err || target<50U || duration<20U) { snprintf(out,cap,"ERR FOC RANGE"); return; }
                snprintf(out,cap,"%s FOC CURRENT BENCH",
                    motor_backend_foc_current_start(strcmp(argv[2],"CURRENTQ")==0,target,duration)?"OK":"ERR"); return;
            }
            if (argc == 5U && strcmp(argv[2],"RANGE") == 0)
            {
                unsigned slot,target;
                const char *err=parse_value(argv[3],6U,&slot);
                if(NULL==err) { err=parse_value(argv[4],2000U,&target); }
                if(NULL!=err || slot<1U || (target!=500U && target!=1000U && target!=1500U && target!=2000U))
                { snprintf(out,cap,"ERR FOC RANGE"); return; }
                snprintf(out,cap,"%s FOC RANGE BENCH",motor_backend_foc_range(slot,target)?"OK":"ERR"); return;
            }
            if (argc == 5U && strcmp(argv[2],"SAMPLE") == 0)
            {
                unsigned trigger,sector;
                const char *err=parse_value(argv[3],8490U,&trigger);
                if(NULL==err) { err=parse_value(argv[4],6U,&sector); }
                if(NULL!=err || trigger<8200U || sector<1U) { snprintf(out,cap,"ERR FOC RANGE"); return; }
                snprintf(out,cap,"%s FOC SAMPLE BENCH",motor_backend_foc_sample_vector(trigger,sector)?"OK":"ERR"); return;
            }
            if (argc == 4U && strcmp(argv[2],"SAMPLE") == 0)
            {
                unsigned trigger;
                const char *err=parse_value(argv[3],8490U,&trigger);
                if (NULL != err || trigger < 8200U) { snprintf(out,cap,"ERR FOC RANGE"); return; }
                snprintf(out,cap,"%s FOC SAMPLE BENCH",motor_backend_foc_sample(trigger)?"OK":"ERR"); return;
            }
            if (argc == 5U && strcmp(argv[2],"VECTOR") == 0)
            {
                unsigned sector, spread;
                const char *err=parse_value(argv[3],6U,&sector);
                if (NULL == err) { err=parse_value(argv[4],200U,&spread); }
                if (NULL != err || sector < 1U || spread < 1U) { snprintf(out,cap,"ERR FOC RANGE"); return; }
                snprintf(out,cap,"%s FOC VECTOR BENCH",motor_backend_foc_vector(sector,spread)?"OK":"ERR"); return;
            }
            if (argc == 5U && strcmp(argv[2],"BENCH") == 0)
            {
                unsigned duty, duration;
                const char *err = parse_value(argv[3],950U,&duty);
                if (NULL == err) { err = parse_value(argv[4],2000U,&duration); }
                if (NULL != err || duty < 50U || duration < 20U)
                {
                    snprintf(out,cap,"ERR FOC RANGE"); return;
                }
                snprintf(out,cap,"%s FOC TIMING BENCH",motor_backend_foc_bench(duty,duration)?"OK":"ERR"); return;
            }
            if (argc == 5U && strcmp(argv[2],"SWITCH") == 0)
            {
                unsigned spread, duration;
                const char *err=parse_value(argv[3],200U,&spread);
                if (NULL == err) { err=parse_value(argv[4],500U,&duration); }
                if (NULL != err || spread < 1U || duration < 20U) { snprintf(out,cap,"ERR FOC RANGE"); return; }
                snprintf(out,cap,"%s FOC SWITCH BENCH",motor_backend_foc_switch(spread,duration)?"OK":"ERR"); return;
            }
            if (argc == 5U && (strcmp(argv[2],"IF") == 0 || strcmp(argv[2],"OBSERVE") == 0))
            {
                unsigned frequency, duration;
                const char *err=parse_value(argv[3],50000U,&frequency);
                if (NULL == err) { err=parse_value(argv[4],5000U,&duration); }
                if (NULL != err || frequency<1000U || duration<500U) { snprintf(out,cap,"ERR FOC RANGE"); return; }
                bool observe=strcmp(argv[2],"OBSERVE")==0;
                bool started=observe?motor_backend_foc_observe(frequency,duration):motor_backend_foc_if(frequency,duration);
                snprintf(out,cap,"%s FOC %s BENCH",started?"OK":"ERR",observe?"OBSERVE":"IF"); return;
            }
            snprintf(out,cap,"ERR FOC ARGUMENT"); return;
        }
        if (argc == 1U) { motor_backend_diagnostics(out,cap); return; }
        if (argc == 2U && strcmp(argv[1],"RUN") == 0)
        {
            motor_backend_run_diagnostics(out,cap); return;
        }
        if (argc == 2U && strcmp(argv[1],"TIMING") == 0)
        {
            motor_backend_timing_diagnostics(out,cap); return;
        }
        if (argc == 4U && strcmp(argv[1],"BEMF") == 0 &&
            (strcmp(argv[2],"TRACE") == 0 || strcmp(argv[2],"EVENTS") == 0))
        {
            unsigned page;
            const char *err = parse_value(argv[3],256U,&page);
            if (err != NULL) { snprintf(out,cap,"ERR MOTOR %s",err); return; }
            motor_backend_bemf_trace_diagnostics(out,cap,page,strcmp(argv[2],"EVENTS")==0); return;
        }
        if (argc == 5U && strcmp(argv[1],"BEMF") == 0 && strcmp(argv[2],"SECTOR") == 0)
        {
            unsigned sector, page;
            const char *err = parse_value(argv[3],6U,&sector);
            if (err == NULL && sector == 0U) { err = "RANGE"; }
            if (err == NULL) { err = parse_value(argv[4],3U,&page); }
            if (err != NULL) { snprintf(out,cap,"ERR MOTOR %s",err); return; }
            motor_backend_bemf_sector_diagnostics(out,cap,sector-1U,page); return;
        }
        if (argc == 2U && strcmp(argv[1],"BEMF") == 0)
        {
            motor_backend_bemf_diagnostics(out,cap); return;
        }
        snprintf(out,cap,"ERR MOTOR ARGUMENT"); return;
    }
    if (strcmp(name,"STORAGE") == 0 && argc == 1U)
    {
        motor_backend_storage_diagnostics(out,cap); return;
    }
    if (strcmp(name,"STEP") == 0)
    {
        if (argc==2U && strcmp(argv[1],"STATUS")==0)
        {
            motor_backend_step_diagnostics(out,cap); return;
        }
        control_step_test_t test={0};
        unsigned sector=0U, dwell_ms=0U;
        const char *err="ARGUMENT";
        if ((argc==3U || argc==4U) && strcmp(argv[1],"RUN")==0)
        {
            test.cycle=true;
            err=parse_value(argv[2],1000U,&dwell_ms);
            if (argc==4U)
            {
                if (strcmp(argv[3],"REV")==0) { test.reverse=true; }
                else if (strcmp(argv[3],"FWD")!=0) { err="ARGUMENT"; }
            }
        }
        else if (argc==3U)
        {
            err=parse_value(argv[1],6U,&sector);
            if (NULL==err && sector==0U) { err="RANGE"; }
            if (NULL==err) { err=parse_value(argv[2],1000U,&dwell_ms); }
            if (NULL==err) { test.sector=(uint8_t)(sector-1U); }
        }
        if (NULL==err && dwell_ms<20U) { err="RANGE"; }
        if (NULL!=err) { snprintf(out,cap,"ERR STEP %s",err); return; }
        if (control_model_snapshot()->source!=CONTROL_REMOTE)
        {
            snprintf(out,cap,"ERR STEP SOURCE"); return;
        }
        test.dwell_ms=dwell_ms;
        control_result_t result=control_model_request_step(&test,g_app_now);
        snprintf(out,cap,"%s STEP %s",result==CONTROL_OK?"OK":"ERR",
            result==CONTROL_OK?"PENDING":control_result_name(result)); return;
    }
    /* 感叹号整行沿用STOP的来源豁免与未取走请求覆盖规则。 */
    bool stop=strcmp(name,"STOP")==0 || strcmp(name,"!")==0;
    if ((strcmp(name,"START") == 0 || stop || strcmp(name,"CLEAR") == 0) && argc == 1U)
    {
        if(stop && wave_stream_testing()) { wave_stream_end(); }
        if(strcmp(name,"START")==0 && (wave_stream_testing() || board_uart_baud_pending()))
        { snprintf(out,cap,"ERR START BUSY"); return; }
        control_request_kind_t kind = strcmp(name,"START") == 0 ? CONTROL_REQUEST_START :
            stop ? CONTROL_REQUEST_STOP : CONTROL_REQUEST_CLEAR_FAULT;
        if (kind == CONTROL_REQUEST_START && control_model_snapshot()->source != CONTROL_REMOTE)
        {
            snprintf(out,cap,"ERR START SOURCE"); return;
        }
        control_result_t result = control_model_request(kind,CONTROL_PARAM_TARGET,0,g_app_now);
        snprintf(out,cap,"%s %s %s",result == CONTROL_OK ? "OK" : "ERR",stop?"STOP":name,
            result == CONTROL_OK ? "PENDING" : control_result_name(result)); return;
    }
    if (strcmp(name,"GET") == 0 || strcmp(name,"SET") == 0)
    {
        bool set = strcmp(name,"SET") == 0;
        if (argc == (set ? 3U : 2U))
        {
            if(!set && strcmp(argv[1],"HAPTIC_CAPS")==0)
            {
                snprintf(out,cap,"OK GET HAPTIC_CAPS=HAP1 TYPES=DETENT,DAMPED DETENTS_MIN=2 DETENTS_MAX=256 IQ_MAX_MA=800 DAMPING_MAX=10000 TRANSITION_MS=200 CLOCK=170000000 OUTER_HZ=1000 FEEDBACK_HZ=5000"); return;
            }
            if(!set && strcmp(argv[1],"POSITION_CAPS")==0)
            {
                snprintf(out,cap,"OK GET POSITION_CAPS=POS1 UNIT=DEG SCALE=1000 MULTITURN=1 HOLD=1 CLOCK=170000000"); return;
            }
            if(!set && strcmp(argv[1],"WAVE_CAPS")==0)
            {
                snprintf(out,cap,"OK GET WAVE_CAPS=BIN1 RATES=5000,10000 BAUDS=921600,1000000,2000000,3000000 BLOCK=32 Q=13 CLOCK=170000000"); return;
            }
            if(!set && strcmp(argv[1],"WAVE_STATS")==0)
            {
                wave_stats_t s=wave_stream_stats(); board_uart_stats_t u=board_uart_stats();
                snprintf(out,cap,"OK GET WAVE_STATS=BIN1 CAPTURED=%lu SENT=%lu DROPPED=%lu CANCELLED=%lu CLIPPED=%lu PACKETS=%lu BACKLOG_MAX=%lu COPY_CYCLES=%lu DMA_ERRORS=%lu RX_OVERRUNS=%lu TEST=%u UART_ERRORS=%lu UART_FLAGS=%lX",
                    (unsigned long)s.captured,(unsigned long)s.sent,(unsigned long)s.dropped,
                    (unsigned long)s.cancelled,(unsigned long)s.clipped,(unsigned long)s.packets,
                    (unsigned long)s.backlog_max,(unsigned long)s.capture_max_cycles,
                    (unsigned long)u.dma_errors,(unsigned long)u.rx_overruns,wave_stream_testing()?1U:0U,
                    (unsigned long)u.errors,(unsigned long)u.error_flags); return;
            }
            if(strcmp(argv[1],"UART_BAUD")==0 || strcmp(argv[1],"WAVE_RATE")==0)
            {
                bool baud=strcmp(argv[1],"UART_BAUD")==0;
                if(set)
                {
                    unsigned value;
                    const char *reason=parse_value(argv[2],3000000U,&value);
                    if(NULL!=reason) { snprintf(out,cap,"ERR SET %s",reason); return; }
                    if(control_model_snapshot()->run!=CONTROL_STOPPED || control_model_pending()!=NULL ||
                        g_app_foc_print!=FOC_PRINT_OFF || wave_stream_testing() || board_uart_baud_pending())
                    { snprintf(out,cap,"ERR SET BUSY"); return; }
                    if((!baud && value==10000U && board_uart_baud()<2000000U) ||
                        (baud && value<2000000U && wave_stream_rate()==10000U))
                    { snprintf(out,cap,"ERR SET BANDWIDTH"); return; }
                    if(!(baud?board_uart_request_baud(value):wave_stream_set_rate(value)))
                    { snprintf(out,cap,"ERR SET RANGE"); return; }
                    snprintf(out,cap,"OK SET %s=%u",argv[1],value); return;
                }
                snprintf(out,cap,"OK GET %s=%lu",argv[1],(unsigned long)(baud?board_uart_baud():wave_stream_rate())); return;
            }
            if (strcmp(argv[1],"FOC_PRINT") == 0)
            {
                if (set)
                {
                    unsigned mode;
                    for (mode = 0U; mode < 5U; ++mode)
                    {
                        if (strcmp(argv[2],g_app_foc_print_names[mode]) == 0) { break; }
                    }
                    if (mode == 5U) { snprintf(out,cap,"ERR SET ARGUMENT"); return; }
                    if(board_uart_baud_pending()) { snprintf(out,cap,"ERR SET BUSY"); return; }
                    if(mode!=FOC_PRINT_BINARY && wave_stream_testing()) { wave_stream_end(); }
                    wave_stream_enable(mode==FOC_PRINT_BINARY);
                    g_app_foc_print = (foc_print_mode_t)mode;
                }
                snprintf(out,cap,"OK %s FOC_PRINT=%s",name,g_app_foc_print_names[g_app_foc_print]);
                return;
            }
            if (!set && strcmp(argv[1],"SAVE")==0) { snprintf(out,cap,"OK GET SAVE=%s",settings_status()); return; }
            if (!set && strcmp(argv[1],"RESULT")==0) { snprintf(out,cap,"OK GET RESULT=%s",control_model_notice()); return; }
            if (strcmp(argv[1],"SOURCE")==0)
            {
                if (set)
                {
                    int32_t source = strcmp(argv[2],"LOCAL")==0 ? CONTROL_LOCAL : strcmp(argv[2],"REMOTE")==0 ? CONTROL_REMOTE : -1;
                    control_result_t r = source < 0 ? CONTROL_INVALID : control_model_request(CONTROL_REQUEST_SOURCE,CONTROL_PARAM_TARGET,source,g_app_now);
                    snprintf(out,cap,"%s SET %s",r==CONTROL_OK?"OK":"ERR",r==CONTROL_OK?"PENDING":control_result_name(r)); return;
                }
                snprintf(out,cap,"OK GET SOURCE=%s",control_model_snapshot()->source==CONTROL_LOCAL?"LOCAL":"REMOTE"); return;
            }
            /* 参数名使用模型名称中的空格替换为下划线，例如 CURRENT_LIMIT。 */
            for (unsigned i=0; i<CONTROL_PARAM_COUNT; ++i)
            {
                const control_parameter_t *p=control_model_parameter((control_parameter_id_t)i);
                char token[sizeof(p->name)]; memcpy(token,p->name,sizeof(token));
                for (char *c=token; *c; ++c) { if (*c==' ') { *c='_'; } }
                if (strcmp(argv[1],token)!=0) { continue; }
                if (!set)
                {
                    if (!p->valid) { snprintf(out,cap,"ERR GET NOT_READY"); return; }
                    snprintf(out,cap,"OK GET %s=%ld DECIMALS=%u UNIT=%s",token,(long)p->value,p->decimals,p->unit);
                    if(p->id==CONTROL_PARAM_RUN_MS || p->id==CONTROL_PARAM_TARGET || p->id==CONTROL_PARAM_CURRENT_LIMIT)
                    {
                        /* 发布真实参数范围，支持上位机能力适配，避免沿用旧32秒本地限制。 */
                        size_t used=strlen(out);
                        snprintf(out+used,cap-used," MIN=%ld MAX=%ld EDITABLE=%u",(long)p->minimum,(long)p->maximum,p->editable?1U:0U);
                    }
                    return;
                }
                const char *digits=argv[2]; bool negative=*digits=='-'; if(negative) { ++digits; }
                unsigned magnitude;
                const char *err=parse_value(digits,INT32_MAX,&magnitude);
                if(err!=NULL) { snprintf(out,cap,"ERR SET %s",err); return; }
                int32_t value=negative?-(int32_t)magnitude:(int32_t)magnitude;
                control_result_t r=control_model_request(CONTROL_REQUEST_PARAMETER,p->id,value,g_app_now);
                snprintf(out,cap,"%s SET %s",r==CONTROL_OK?"OK":"ERR",r==CONTROL_OK?"PENDING":control_result_name(r)); return;
            }
            bool algorithm = strcmp(argv[1],"ALGORITHM")==0;
            bool mode = strcmp(argv[1],"MODE")==0;
            if (algorithm || mode) {
                control_algorithm_t a=control_model_algorithm(); control_mode_t m=control_model_mode();
                if(set) {
                    unsigned i;
                    for(i=0;i<(algorithm?CONTROL_ALGORITHM_COUNT:CONTROL_MODE_COUNT);++i) {
                        if(strcmp(argv[2],algorithm?control_algorithm_token((control_algorithm_t)i):control_mode_name((control_mode_t)i))==0) { break; }
                    }
                    if(i==(algorithm?CONTROL_ALGORITHM_COUNT:CONTROL_MODE_COUNT)) { snprintf(out,cap,"ERR SET ARGUMENT"); return; }
                    if(algorithm) { a=(control_algorithm_t)i; if(!control_algorithm_foc(a) || ((m==CONTROL_CURRENT || m==CONTROL_HAPTIC) && a!=CONTROL_ENCODER_FOC)) { m=CONTROL_SPEED; } } else { m=(control_mode_t)i; }
                    control_result_t r=control_model_select(a,m);
                    if(r!=CONTROL_OK) { snprintf(out,cap,"ERR SET %s",control_result_name(r)); return; }
                }
                snprintf(out,cap,"OK %s %s=%s",name,argv[1],algorithm?control_algorithm_token(a):control_mode_name(m)); return;
            }
            bool brightness = strcmp(argv[1],"BRIGHTNESS") == 0;
            bool animation = strcmp(argv[1],"ANIMATION") == 0;
            if (brightness || animation)
            {
                unsigned value = brightness ? (unsigned)g_app_brightness : (g_app_animation ? 1U : 0U);
                if (set)
                {
                    error = parse_value(argv[2],brightness ? 100U : 1U,&value);
                    if (error == NULL && brightness && value == 0U) { error = "RANGE"; }
                    if (error != NULL) { snprintf(out,cap,"ERR %s %s",name,error); return; }
                    if (brightness) { g_app_brightness = (int32_t)value; }
                    else { g_app_animation = value != 0U; }
                    capture_settings();
                }
                snprintf(out,cap,"OK %s %s=%u",name,argv[1],value); return;
            }
        }
    }
    else if (strcmp(name,"UI") == 0)
    {
        snprintf(out,cap,"ERR UI UNAVAILABLE"); return;
    }
    else if (strcmp(name,"PING") != 0 && strcmp(name,"VERSION") != 0 && strcmp(name,"HELP") != 0 &&
             strcmp(name,"STATUS") != 0 && strcmp(name,"ENCODER") != 0 && strcmp(name,"INPUT") != 0) { error = "UNKNOWN"; }
    snprintf(out,cap,"ERR %s %s",name,error);
}

/** @brief 恢复旧版兼容参数并初始化输入诊断及串口控制模型。 */
void
interaction_init(bool uart_online)
{
    wave_stream_init();
    g_app_now = board_millis();
    g_app_last_loop = g_app_now;
    g_app_uart_online = uart_online;
    g_app_gap_max = 0U;
    g_app_brightness = 50;
    g_app_animation = true;
    g_app_foc_print = FOC_PRINT_OFF;
    memset(g_app_events,0,sizeof(g_app_events));
    g_app_last_event="NONE";
    g_app_last_source="NONE";
    console_init(&g_app_console);
    switch_input_init(&g_app_switch,board_switch_raw(),g_app_now);
    control_model_init();
    settings_init();
    (void)control_model_select((control_algorithm_t)settings_get()->algorithm,(control_mode_t)settings_get()->mode);
    g_app_brightness=(int32_t)settings_get()->brightness;
    g_app_animation=settings_get()->animation!=0;
    settings_publish();
    capture_settings();
}

/** @brief 推进有界输入诊断和串口服务，确认后的设置交给原保存模块。 */
void
interaction_service(void)
{
    switch_event_t events[2];
    g_app_now = board_millis();
    uint32_t gap = g_app_now - g_app_last_loop;
    if (gap > g_app_gap_max) { g_app_gap_max = gap; }
    g_app_last_loop = g_app_now;
    uint8_t count = switch_input_update(&g_app_switch,board_switch_raw(),g_app_now,events);
    for (uint8_t i = 0U; i < count; ++i) { handle_event(events[i],false); }
    /* 每轮至多处理32字节；下一条命令开始前必须能放下最坏情况完整响应。 */
    for (unsigned i = 0U; i < 32U && serial_buffer_tx_free() >= CONSOLE_RESPONSE_MAX; ++i)
    {
        uint16_t token;
        if (!serial_buffer_read(&token)) { break; }
        if (console_feed(&g_app_console,token,command,g_app_response))
        {
            (void)serial_buffer_write(g_app_response,strlen(g_app_response));
            board_uart_kick();
            /* 每次主循环至多执行一条完整命令，让同步样本尽快获得后台CPU。 */
            break;
        }
    }
    capture_settings();
    wave_stream_test_service(board_uart_ticks(),g_app_now);
    if(g_app_foc_print==FOC_PRINT_BINARY && !serial_buffer_rx_pending() &&
        g_app_console.length==0U && control_model_pending()==NULL && board_uart_wave_ready())
    {
        size_t bytes=wave_stream_next(board_uart_wave_buffer(),WAVE_PACKET_CAPACITY);
        if(bytes>0U) { (void)board_uart_wave_submit(bytes); }
    }
    /* 运行时先处理输入；TX有空间就发送新样本，始终给控制响应保留容量。 */
    if (control_model_snapshot()->run == CONTROL_RUNNING)
    {
        if (g_app_foc_print != FOC_PRINT_OFF && g_app_foc_print != FOC_PRINT_BINARY && !serial_buffer_rx_pending() && g_app_console.length == 0U &&
            NULL == control_model_pending() &&
            serial_buffer_tx_free() >= CONSOLE_RESPONSE_MAX+sizeof(g_app_foc_current))
        {
            bool generated=g_app_foc_print==FOC_PRINT_TRACE?
                motor_backend_foc_trace(g_app_foc_current,sizeof(g_app_foc_current)):
                motor_backend_foc_current(g_app_foc_current,sizeof(g_app_foc_current),g_app_foc_print == FOC_PRINT_FIREWATER);
            if (generated &&
                serial_buffer_write(g_app_foc_current,strlen(g_app_foc_current))) { board_uart_kick(); }
        }
    }
}

/** @brief 串口仍有输入且能容纳完整响应时继续服务，否则等待下一次中断。 */
bool
interaction_pending(void)
{
    return serial_buffer_rx_pending() && serial_buffer_tx_free() >= CONSOLE_RESPONSE_MAX;
}
