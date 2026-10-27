/* 复用Flash/DRV替身，START、校准和电流/编码器保护均运行正式后端。 */
#define MOTOR_FOC_BACKEND_FIXTURE_ONLY
#define board_motor_cycles unused_legacy_cycles
#include "../motor/test_foc_backend.c"
#undef board_motor_cycles
#include "board_encoder.h"

/* 编码器只模拟真实机械轨迹及DMA完整帧，控制算法来自正式模块。 */
static uint32_t g_cycles,g_frame_divider;
static bool g_fast_active,g_online=true,g_freeze,g_corrupt;
static encoder_six_feedback_t g_feedback;
static double g_mechanical=123456.0,g_target_mechanical=123456.0;
static uint8_t g_previous_sector;
static encoder_position_t g_test_position;
static int g_physical_direction=1;

/** @brief 返回自由运行DWT替身。 @return 周期计数。 */
uint32_t board_motor_cycles(void) { return g_cycles; }
/** @brief 停机预检使用机械位置有效状态，不伪造控制速度。 @param now_ms 当前ms。 @return 预检快照。 */
encoder_snapshot_t board_encoder_snapshot(uint32_t now_ms)
{
    (void)now_ms;
    return (encoder_snapshot_t){.state=g_online?ENCODER_OK:ENCODER_IO_ERROR,.angle_valid=g_online,
        .speed_valid=g_online,.rpm=0,.raw=(uint32_t)g_mechanical&(ENCODER_SIX_TURN-1U)};
}
/** @brief 模拟SPI1控制采集独占。 @return 可启用。 */
bool board_encoder_control_begin(void)
{
    if(!g_online || g_fast_active) { return false; }
    g_fast_active=true;g_frame_divider=0U;g_feedback=(encoder_six_feedback_t){0};return true;
}
/** @brief 模拟每4个PWM周期完成一帧。 @return 最新完整反馈。 */
encoder_six_feedback_t board_encoder_control_tick(void)
{
    if(g_fast_active && !g_freeze && ++g_frame_divider>=4U)
    {
        g_frame_divider=0U;
        g_feedback=(encoder_six_feedback_t){(uint32_t)g_mechanical&(ENCODER_SIX_TURN-1U),g_cycles,g_feedback.sequence+1U,!g_corrupt};
    }
    return g_feedback;
}
/** @brief 停机释放控制采集。 */
void board_encoder_control_end(void) { g_fast_active=false; }
/* SPI影子设置替身，不输出功率；真实后端仍负责REMOTE/运行/静止守卫。 */
static uint8_t g_hyst_code=7U;
/** @brief 模拟停止后的影子配置。 @param code 目标。 @return 总线空闲且在线。 */
encoder_position_t board_encoder_position(void) { return g_test_position; }
bool board_encoder_position_zero(int8_t direction) { return encoder_position_zero(&g_test_position,direction); }
void board_encoder_position_invalidate(void) { g_test_position.zero_valid=false; }
bool board_encoder_hysteresis(uint8_t code)
{
    if(g_fast_active || !g_online) { return false; }
    g_hyst_code=code;return true;
}

/** @brief 将模拟桥臂指令转成独立的六扇区编号。 @return 0..5。 */
static unsigned output_sector(void)
{
    static const uint8_t high[6]={0,0,1,1,2,2},low[6]={1,2,2,0,0,1};
    for(unsigned i=0;i<6U;++i) { if(output.high==high[i] && output.low==low[i]) { return i; } }
    CHECK(false);return 0U;
}

/** @brief 推进一周期真实回调，机械轨迹独立于目标转速/无感观测器。
 * @param calibration 校准时向真实桥臂位置收敛。 @param rpm 正常测试机械RPM。 */
static void pump(bool calibration,double rpm)
{
    unsigned sector=output_sector();
    if(calibration)
    {
        if(sector!=g_previous_sector)
        {
            CHECK(sector==(unsigned)((g_previous_sector+1U)%6U));
            g_target_mechanical+=g_physical_direction*(double)ENCODER_SIX_TURN/42.0;
            g_previous_sector=(uint8_t)sector;
        }
        if(output.pulse)
        {
            double delta=g_target_mechanical-g_mechanical;
            g_mechanical+=delta>300?300:delta< -300?-300:delta;
        }
    }
    else { g_mechanical+=g_physical_direction*rpm*ENCODER_SIX_TURN/(60.0*20000.0); }
    while(g_mechanical<0.0) { g_mechanical+=ENCODER_SIX_TURN;g_target_mechanical+=ENCODER_SIX_TURN; }
    while(g_mechanical>=ENCODER_SIX_TURN) { g_mechanical-=ENCODER_SIX_TURN;g_target_mechanical-=ENCODER_SIX_TURN; }
    g_cycles+=8500U;
    current_sample(2048U,output.low,output.pulse,&output);
    if(hardware.running) { peak_callback(false,0U,0U,&output); }
    if(hardware.running && ++pwm_speed_count>=20U)
    {
        pwm_speed_count=0U;
        if(speed_callback) { speed_callback(); }
    }
    if(g_cycles%3400000U==0U) { motor_backend_service(now+=20U); }
}

/** @brief 建立新电机空白Flash、真实后端和未校准编码器。 */
static void reset_encoder(void)
{
    memset(flash,255,sizeof(flash));layout=calibration_ok=true;calibration_failures_left=0;
    calibrated_bus_mv=0;start_failure_reason=NULL;
    g_cycles=0U;g_online=true;g_freeze=g_corrupt=g_fast_active=false;
    g_test_position=(encoder_position_t){0};
    g_mechanical=g_target_mechanical=123456.0;g_previous_sector=0U;
    control_model_init();settings_init();
    CHECK(control_model_select(CONTROL_ENCODER_6STEP,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_init();motor_backend_service(now=20U);
    CHECK(control_model_snapshot()->ready && !control_model_snapshot()->calibrated);
    request(CONTROL_REQUEST_SOURCE,CONTROL_PARAM_TARGET,CONTROL_REMOTE);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_DUTY,500);
}

/** @brief 完整执行七点标定并核对Flash候选和闭环入口。 */
static void calibrate(void)
{
    CHECK(motor_backend_encoder_calibrate()==NULL && hardware.running && !bemf_enabled);
    for(unsigned i=0U;i<43000U && hardware.running;++i) { pump(true,0.0); }
    CHECK(!hardware.running && strcmp(hardware.fault,"NONE")==0);
    motor_backend_service(now+=20U);
    char line[512];motor_backend_encoder_diagnostics(line,sizeof(line));
    if(strstr(line,"CALIBRATION=VALID")==NULL) { fprintf(stderr,"calibration result: %s\n",line); }
    CHECK(strstr(line,"CALIBRATION=VALID")!=NULL && !g_fast_active);
    CHECK(strstr(line,"STARTUP=CAL")!=NULL && diagnostic_count(line,"STARTUP_END_MS=")==0U);
    CHECK(settings_get()->parameter[CONTROL_PARAM_ENCODER_DIR]==g_physical_direction);
    CHECK(control_model_snapshot()->calibrated);
}

/** @brief 实际START拒绝默认零点、正确标定后运行完整5秒，STOP保留结果。 */
static void test_normal(void)
{
    reset_encoder();
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"ENCODER CAL REQUIRED")==0);
    CHECK(!control_model_parameter(CONTROL_PARAM_START_HZ)->editable);
    calibrate();
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && !bemf_enabled && bemf_callback==NULL && peak_callback!=NULL);
    for(unsigned i=0U;i<100100U && hardware.running;++i) { pump(false,600.0); }
    CHECK(!hardware.running && strcmp(hardware.fault,"NONE")==0);
    motor_backend_service(now+=20U);
    char line[512];motor_backend_encoder_diagnostics(line,sizeof(line));
    uint32_t measured=diagnostic_count(line,"RPM=");
    CHECK(strstr(line,"TICKS=100000")!=NULL && measured>=599U && measured<=600U);
    CHECK(strstr(line,"STARTUP=DONE")!=NULL && diagnostic_count(line,"STARTUP_END_MS=")<300U);
    CHECK(strstr(line,"CALIBRATION=VALID")!=NULL && !g_fast_active);
    /* 手改零点后资格失效，不能凭预设绕过本次上电校准。 */
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_ELEC_ZERO,0);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"ENCODER CAL REQUIRED")==0);
}

/** @brief 运行中冻结/破坏反馈或过流立即关断，不回退无感。 */
static void test_shutdown(void)
{
    for(unsigned fault=0;fault<3U;++fault)
    {
        reset_encoder();calibrate();
        request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,100);
        request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
        CHECK(hardware.running);
        for(unsigned i=0;i<500U;++i) { pump(false,100.0); }
        if(fault==0U) { g_freeze=true; }
        if(fault==1U) { g_corrupt=true; }
        if(fault==2U) { current_sample(3400U,output.low,true,&output); }
        for(unsigned i=0;i<12U && hardware.running;++i) { pump(false,100.0); }
        CHECK(!hardware.running);
        CHECK(strcmp(hardware.fault,fault==0U?"ENCODER STALE":fault==1U?"ENCODER DATA":"PHASE OVERCURRENT")==0);
        motor_backend_service(now+=20U);CHECK(!g_fast_active);
    }
}

/** @brief 95%只对有感开放，可保存恢复，切回原六步恢复其幅度范围。 */
static void test_duty_range(void)
{
    reset_encoder();
    CHECK(control_model_parameter(CONTROL_PARAM_DUTY)->maximum==950);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_DUTY,950);
    CHECK(settings_get()->parameter[CONTROL_PARAM_DUTY]==950);
    CHECK(!settings_set(CONTROL_PARAM_DUTY,951,now));
    settings_capture(50U,true,now);
    settings_service(true,now+1200U);
    CHECK(strcmp(settings_status(),"SAVED")==0);
    settings_init();settings_publish();
    CHECK(strcmp(settings_status(),"SAVED")==0 && settings_get()->parameter[CONTROL_PARAM_DUTY]==950);
    CHECK(control_model_select(CONTROL_OPEN_6STEP,CONTROL_SPEED)==CONTROL_OK);
    settings_capture(50U,true,now+1400U);settings_publish();
    CHECK(settings_get()->parameter[CONTROL_PARAM_DUTY]==600);
    CHECK(control_model_parameter(CONTROL_PARAM_DUTY)->maximum==600);
    CHECK(!settings_set(CONTROL_PARAM_DUTY,601,now+1400U));
}

/** @brief 同一机械轨迹下仅正常RUN的低幅序列改变，标定及累计输出不变。 */
static void test_quiet(void)
{
    uint64_t calibration_signature[2]={0U,0U},run_signature[2]={0U,0U};
    unsigned run_ticks[2]={0U,0U};
    for (unsigned quiet=0U;quiet<2U;++quiet)
    {
        reset_encoder();
        char line[512];motor_backend_encoder_diagnostics(line,sizeof(line));
        CHECK(strstr(line,"QUIET=ON")!=NULL);
        settings_data_t saved=*settings_get();
        CHECK(motor_backend_encoder_quiet(quiet!=0U)==NULL);
        CHECK(memcmp(&saved,settings_get(),sizeof(saved))==0);
        CHECK(motor_backend_encoder_calibrate()==NULL);
        CHECK(strcmp(motor_backend_encoder_quiet(quiet==0U),"BUSY")==0);
        for (unsigned i=0U;i<43000U && hardware.running;++i)
        {
            calibration_signature[quiet]=calibration_signature[quiet]*31U+output.pulse;
            pump(true,0.0);
        }
        CHECK(!hardware.running && strcmp(hardware.fault,"NONE")==0);
        motor_backend_service(now+=20U);
        /* 改会话选项不使刚建立的方向/零点资格失效，也不排队Flash写入。 */
        saved=*settings_get();
        CHECK(motor_backend_encoder_quiet(quiet==0U)==NULL);
        CHECK(motor_backend_encoder_quiet(quiet!=0U)==NULL);
        CHECK(memcmp(&saved,settings_get(),sizeof(saved))==0);
        motor_backend_encoder_diagnostics(line,sizeof(line));
        CHECK(strstr(line,"CALIBRATION=VALID")!=NULL);
        request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
        CHECK(hardware.running && strcmp(motor_backend_encoder_quiet(quiet==0U),"BUSY")==0);
        for (unsigned i=0U;i<2000U && hardware.running;++i)
        {
            run_signature[quiet]=run_signature[quiet]*31U+output.pulse;
            if (output.pulse) { run_ticks[quiet]+=output.duty_ticks; }
            pump(false,0.0);
        }
        CHECK(hardware.running);
        request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
        CHECK(!hardware.running);
    }
    CHECK(calibration_signature[0]==calibration_signature[1]);
    CHECK(run_signature[0]!=run_signature[1]);
    CHECK(run_ticks[0]>=run_ticks[1] && run_ticks[0]-run_ticks[1]<2U*391U);
    CHECK(control_model_select(CONTROL_OPEN_6STEP,CONTROL_SPEED)==CONTROL_OK);
    CHECK(strcmp(motor_backend_encoder_quiet(true),"SELECT ENCODER_6STEP")==0);
    CHECK(control_model_select(CONTROL_ENCODER_6STEP,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(now+=20U);
    request(CONTROL_REQUEST_SOURCE,CONTROL_PARAM_TARGET,CONTROL_LOCAL);
    CHECK(strcmp(motor_backend_encoder_quiet(true),"REMOTE ONLY")==0);
}

/** @brief 新启动补偿仍不能把堵住的转子当成功，300ms通过真实后端关闭功率。 */
static void test_startup_failure(void)
{
    reset_encoder();calibrate();
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running);
    for(unsigned i=0U;i<6100U && hardware.running;++i) { pump(false,0.0); }
    CHECK(!hardware.running && strcmp(hardware.fault,"ENCODER START")==0);
    motor_backend_service(now+=20U);
    CHECK(!g_fast_active);
    char line[512];motor_backend_encoder_diagnostics(line,sizeof(line));
    CHECK(diagnostic_count(line,"MOTION_MS=")==0U);
    CHECK(diagnostic_count(line,"TICKS=")<=6000U);
}

/** @brief 实际应用路由拒绝未校准FOC，校准后直接调用编码器FOC板卡入口。 */
static void test_encoder_foc_backend(void)
{
    reset_encoder();
    CHECK(control_model_select(CONTROL_ENCODER_FOC,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(now+=20U);
    CHECK(control_model_snapshot()->ready && !control_model_snapshot()->calibrated);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"ENCODER CAL REQUIRED")==0);
    CHECK(control_model_select(CONTROL_ENCODER_6STEP,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(now+=20U);calibrate();
    uint16_t zero=(uint16_t)settings_get()->parameter[CONTROL_PARAM_ELEC_ZERO];
    int32_t six_target=settings_get()->parameter[CONTROL_PARAM_TARGET];
    CHECK(control_model_select(CONTROL_ENCODER_FOC,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(now+=20U);
    CHECK(control_model_snapshot()->ready && control_model_snapshot()->calibrated);
    CHECK(!control_model_parameter(CONTROL_PARAM_DUTY)->valid);
    CHECK(control_model_parameter(CONTROL_PARAM_ELEC_ZERO)->valid && !control_model_parameter(CONTROL_PARAM_ELEC_ZERO)->editable);
    CHECK(strcmp(control_model_parameter(CONTROL_PARAM_SPEED_KP)->unit,"MA/RPM")==0);
    /* 正常FOC没有32秒固定上限，长轮及自然整数范围可保存后重新装载。 */
    CHECK(control_model_parameter(CONTROL_PARAM_RUN_MS)->maximum==INT32_MAX);
    CHECK(settings_set(CONTROL_PARAM_RUN_MS,90000,now));
    CHECK(settings_value(CONTROL_PARAM_RUN_MS)==90000);
    CHECK(settings_set(CONTROL_PARAM_RUN_MS,INT32_MAX,now));
    settings_service(true,now+2000U);
    settings_init();
    CHECK(settings_get()->foc.run_ms==INT32_MAX);
    settings_publish();
    CHECK(settings_set(CONTROL_PARAM_RUN_MS,5000,now));
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && foc_active && foc_encoder && !g_fast_active);
    CHECK(received_duration==5000U && received_encoder_config.direction==1 && received_encoder_config.zero_decideg==zero);
    CHECK(received_encoder_config.drive.pole_pairs==7U && received_encoder_config.drive.target_rpm==600U);
    CHECK(fabsf(received_encoder_config.drive.current_limit_a-.5f)<1e-6f);
    CHECK(strcmp(control_model_snapshot()->commutation,"MT6835")==0);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,650);
    CHECK(strcmp(control_model_notice(),"FOC WAIT CLOSED")==0);
    commit_confirmed=true;motor_backend_service(now+=20U);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,650);
    CHECK(settings_get()->foc.target_rpm==650 && settings_get()->parameter[CONTROL_PARAM_TARGET]==six_target);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && !foc_active);
    /* CURRENT是独立有符号mA，不借用保存的650RPM或六步目标。 */
    CHECK(control_model_select(CONTROL_ENCODER_FOC,CONTROL_CURRENT)==CONTROL_OK);
    motor_backend_service(now+=20U);
    const control_parameter_t *iq=control_model_parameter(CONTROL_PARAM_TARGET);
    CHECK(iq->valid && iq->editable && iq->decimals==3U && strcmp(iq->unit,"A")==0);
    CHECK(iq->minimum==-500 && iq->maximum==500 && iq->value==0);
    CHECK(control_model_parameter(CONTROL_PARAM_CURRENT_LIMIT)->value==500 &&
        !control_model_parameter(CONTROL_PARAM_CURRENT_LIMIT)->editable);
    CHECK(!control_model_parameter(CONTROL_PARAM_SPEED_KP)->valid && !control_model_parameter(CONTROL_PARAM_RAMP)->valid);
    CHECK(!settings_set(CONTROL_PARAM_TARGET,-501,now) && !settings_set(CONTROL_PARAM_TARGET,501,now));
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,-100);
    CHECK(settings_get()->foc.target_iq_ma==-100 && settings_get()->foc.target_rpm==650);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(foc_active && received_encoder_config.current_mode && fabsf(received_encoder_config.target_iq_a+.1f)<1e-6f);
    CHECK(fabsf(received_config.current_limit_a-.5f)<1e-6f);
    CHECK(control_model_select(CONTROL_ENCODER_FOC,CONTROL_SPEED)==CONTROL_BUSY);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,100);
    CHECK(strcmp(control_model_notice(),"FOC WAIT CLOSED")==0 && settings_get()->foc.target_iq_ma==-100);
    commit_confirmed=true;motor_backend_service(now+=20U);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,500);
    CHECK(settings_get()->foc.target_iq_ma==500 && fabsf(received_encoder_config.target_iq_a-.5f)<1e-6f);
    CHECK(control_model_snapshot()->values[CONTROL_TARGET].value==500 && control_model_snapshot()->values[CONTROL_TARGET].decimals==3U);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,-500);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,0);
    CHECK(foc_active && settings_get()->foc.target_iq_ma==0 && settings_get()->foc.target_rpm==650);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,-123);
    settings_capture(settings_get()->brightness,settings_get()->animation!=0U,now);
    motor_backend_service(now+=2000U);
    CHECK(strcmp(settings_status(),"SAVED")==0);
    settings_data_t expected=*settings_get();
    settings_init();CHECK(memcmp(&expected,settings_get(),sizeof(expected))==0);
    CHECK(settings_get()->mode==CONTROL_CURRENT && settings_get()->foc.target_iq_ma==-123);
    CHECK(control_model_select(CONTROL_SENSORLESS_FOC,CONTROL_CURRENT)==CONTROL_INVALID);
    CHECK(control_model_select(CONTROL_ENCODER_FOC,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(now+=20U);
    CHECK(settings_value(CONTROL_PARAM_TARGET)==650 && settings_get()->foc.target_iq_ma==-123);
    CHECK(control_model_select(CONTROL_ENCODER_6STEP,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(now+=20U);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_ELEC_ZERO,(zero+1U)%3600U);
    CHECK(control_model_select(CONTROL_ENCODER_FOC,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(now+=20U);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"ENCODER CAL REQUIRED")==0);
}

/** @brief 校验影子迟滞的REMOTE/运行/总线守卫及参数、标定资格保持。 */
static void
test_hysteresis(void)
{
    reset_encoder();g_hyst_code=7U;
    CHECK(motor_backend_encoder_hysteresis(5U)==NULL && g_hyst_code==5U);
    CHECK(strcmp(motor_backend_encoder_hysteresis(3U),"RANGE")==0 && g_hyst_code==5U);
    g_online=false;
    CHECK(strcmp(motor_backend_encoder_hysteresis(4U),"HYSTERESIS IO")==0 && g_hyst_code==5U);
    g_online=true;
    request(CONTROL_REQUEST_SOURCE,CONTROL_PARAM_TARGET,CONTROL_LOCAL);
    CHECK(strcmp(motor_backend_encoder_hysteresis(4U),"REMOTE ONLY")==0 && g_hyst_code==5U);
    request(CONTROL_REQUEST_SOURCE,CONTROL_PARAM_TARGET,CONTROL_REMOTE);
    calibrate();
    int32_t zero=settings_get()->parameter[CONTROL_PARAM_ELEC_ZERO];
    CHECK(motor_backend_encoder_hysteresis(6U)==NULL && g_hyst_code==6U);
    CHECK(control_model_snapshot()->calibrated && settings_get()->parameter[CONTROL_PARAM_ELEC_ZERO]==zero);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && strcmp(motor_backend_encoder_hysteresis(4U),"BUSY")==0 && g_hyst_code==6U);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    /* STOP消费与下一轮主循环DMA回收有先后；回收前设置仍须拒绝。 */
    CHECK(strcmp(motor_backend_encoder_hysteresis(7U),"HYSTERESIS IO")==0 && g_hyst_code==6U);
    motor_backend_service(now+=20U);
    CHECK(motor_backend_encoder_hysteresis(7U)==NULL && g_hyst_code==7U && !hardware.running);
}

/** @brief 位置零点、独立参数、目标路由和运行中拒绝均执行正式后端。 */
static void
test_position_backend(void)
{
    reset_encoder();calibrate();
    CHECK(control_model_select(CONTROL_ENCODER_FOC,CONTROL_POSITION)==CONTROL_OK);
    motor_backend_service(now+=20U);
    CHECK(control_model_parameter(CONTROL_PARAM_TARGET)->decimals==3U);
    CHECK(strcmp(control_model_parameter(CONTROL_PARAM_TARGET)->unit,"DEG")==0);
    CHECK(settings_value(CONTROL_PARAM_POSITION_KP)==8000 && settings_value(CONTROL_PARAM_SPEED_LIMIT)==60);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"POSITION ZERO REQUIRED")==0);
    g_test_position=(encoder_position_t){.raw=1234,.count=1234,.seen=true,.valid=true};
    CHECK(motor_backend_position_zero()==NULL && g_test_position.zero_valid);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,-90000);
    CHECK(settings_get()->foc.target_rpm==600 && settings_get()->foc.target_iq_ma==0);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && received_encoder_config.position_mode && !received_encoder_config.current_mode);
    CHECK(received_encoder_config.target_position_mdeg==-90000 && fabsf(received_config.current_limit_a-.5f)<1e-6f);
    CHECK(fabsf(received_encoder_config.position_kp-8.0f)<1e-6f && received_encoder_config.position_speed_limit_rpm==60);
    CHECK(control_model_snapshot()->values[CONTROL_TARGET].value==-90000);
    CHECK(strcmp(motor_backend_position_zero(),"POWER ACTIVE")==0);
    CHECK(motor_backend_encoder_calibrate()!=NULL && g_test_position.zero_valid);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,720000);
    CHECK(strcmp(control_model_notice(),"FOC WAIT CLOSED")==0 && settings_value(CONTROL_PARAM_TARGET)==-90000);
    commit_confirmed=true;motor_backend_service(now+=20U);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,720000);
    CHECK(received_encoder_config.target_position_mdeg==720000 && settings_value(CONTROL_PARAM_TARGET)==720000);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    CHECK(motor_backend_position_tolerance(500)==NULL && settings_get()->position.tolerance_mdeg==500);
    CHECK(motor_backend_position_tolerance(101)!=NULL);
    CHECK(g_test_position.zero_valid);
    /* 位置电流使用已有FOC参数块，运行中拒绝写入，CURRENT仍固定0.5A。 */
    CHECK(control_model_parameter(CONTROL_PARAM_CURRENT_LIMIT)->editable);
    CHECK(!settings_set(CONTROL_PARAM_CURRENT_LIMIT,801,now));
    CHECK(settings_set(CONTROL_PARAM_CURRENT_LIMIT,600,now));
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && fabsf(received_config.current_limit_a-.6f)<1e-6f);
    CHECK(control_model_request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_CURRENT_LIMIT,800,now)==CONTROL_BUSY);
    CHECK(settings_value(CONTROL_PARAM_CURRENT_LIMIT)==600);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    CHECK(settings_set(CONTROL_PARAM_CURRENT_LIMIT,800,now));
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && fabsf(received_config.current_limit_a-.8f)<1e-6f);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    CHECK(control_model_select(CONTROL_ENCODER_FOC,CONTROL_CURRENT)==CONTROL_OK);
    motor_backend_service(now+=20U);
    CHECK(settings_value(CONTROL_PARAM_CURRENT_LIMIT)==500 && settings_get()->foc.current_limit_ma==800);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && received_encoder_config.current_mode && fabsf(received_config.current_limit_a-.5f)<1e-6f);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    CHECK(control_model_select(CONTROL_ENCODER_FOC,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(now+=20U);
    CHECK(settings_value(CONTROL_PARAM_TARGET)==600 && settings_get()->foc.speed_kp_micro==400);
    CHECK(settings_set(CONTROL_PARAM_CURRENT_LIMIT,500,now));
}

/** @brief 独立FOC与六步运行标志分离时，在线配置仍必须实际发布给控制环。 */
static void
test_haptic_live_publish(void)
{
    reset_encoder();calibrate();
    CHECK(control_model_select(CONTROL_ENCODER_FOC,CONTROL_HAPTIC)==CONTROL_OK);
    motor_backend_service(now+=20U);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(foc_active && received_encoder_config.haptic_mode && received_duration==60000U);
    /* 实板独立FOC不写六步status.running，原替身曾把两标志合并而漏掉该缺陷。 */
    hardware.running=false;
    encoder_haptic_config_t c={ENCODER_HAPTIC_DAMPED,256,100,3000};
    unsigned before=g_haptic_publishes;
    CHECK(motor_backend_haptic_configure(&c)==NULL);
    CHECK(g_haptic_publishes==before+1U && memcmp(&received_encoder_config.haptic,&c,sizeof(c))==0);
    CHECK(memcmp(&settings_get()->haptic.config,&c,sizeof(c))==0 && strcmp(settings_status(),"PENDING")==0);
    c.detents=1;CHECK(strcmp(motor_backend_haptic_configure(&c),"RANGE")==0);
    CHECK(g_haptic_publishes==before+1U && settings_get()->haptic.config.detents==256);
    c.detents=24;received_encoder_config.haptic_mode=false;
    CHECK(strcmp(motor_backend_haptic_configure(&c),"POWER ACTIVE")==0);
    CHECK(settings_get()->haptic.config.detents==256);
    board_motor_stop(NULL);motor_backend_service(now+=20U);
    CHECK(motor_backend_haptic_configure(&c)==NULL && g_haptic_publishes==before+1U);
}

/** @brief 运行真实请求/控制集成，包含两种编码器接向。 @return 成功返回0。 */
int main(void)
{
    test_duty_range();
    test_quiet();
    test_haptic_live_publish();
    for(g_physical_direction=-1;g_physical_direction<=1;g_physical_direction+=2) { test_normal(); }
    g_physical_direction=1;test_shutdown();
    test_startup_failure();
    test_encoder_foc_backend();
    test_hysteresis();
    test_position_backend();
    printf("encoder six-step backend: %u checks passed\n",checks);
    return 0;
}
