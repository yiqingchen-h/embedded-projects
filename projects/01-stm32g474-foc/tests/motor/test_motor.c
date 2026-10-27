#include "settings.h"
#include "motor_backend.h"
#include "motor_config.h"
#include "six_step.h"
#include "board_motor.h"
#include "board_storage.h"
#include "drv8353f.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

/* Flash 替身可在任意编程双字后掉电，不连接真实板卡。 */
static unsigned char flash[2][BOARD_STORAGE_SLOT_SIZE];
static unsigned writes, cut_after = UINT_MAX, checks;
static size_t record_size;
static bool layout = true;
/* 板卡替身只注入电气状态，运行回调来自真实后端。 */
static board_motor_status_t hardware;
static board_motor_sample_fn current_sample;
static board_motor_peak_fn peak_callback;
static bool controlled, scheduled;
/* 模拟ADC即时检测及TIM精确1ms分频，不调用主循环推进算法。 */
static board_motor_bemf_fn bemf_callback;
static board_motor_speed_fn speed_callback;
static uint32_t pwm_speed_count, speed_calls;
static unsigned stop_pi_locks;
static uint8_t scheduled_sector;
static uint16_t control_duty;
static uint32_t scheduled_us;
static board_motor_output_t output;
/* 同步相电压替身与采样配置，仅供无感后端集成测试。 */
static board_motor_observation_t observation;
static bool bemf_enabled;
static bool calibration_ok = true;
/* 注入冷启动瞬态失败，检查重试次数和持续故障的停机状态。 */
static unsigned calibration_calls, calibration_failures_left;
/* 校准等待期间母线可能变化，启动只能使用等待后的实测结果。 */
static uint16_t calibrated_bus_mv;
/* 板卡启动失败原因必须原样送到请求结果，便于实物定位。 */
static const char *start_failure_reason;
static uint32_t now;
/* 驱动器寄存器模型及事务失败位置。 */
static uint16_t regs[7];
static unsigned transactions, fail_at;
#define CHECK(x) do { ++checks; assert(x); } while (0)

/** @brief 返回模拟的 DBANK 布局检查结果。 */
bool
board_storage_layout_valid(void) { return layout; }
/** @brief 模拟单 Bank 固件与双 Bank 芯片不匹配的存储诊断。 @return 布局状态。 */
board_storage_status_t
board_storage_status(void)
{
    return (board_storage_status_t){.flash_kb=128, .actual_dual_bank=!layout,
        .result=layout?"NONE":"LAYOUT", .page_error=UINT32_MAX};
}
/** @brief 从 Flash 替身读取记录，布局错误时拒绝。 */
bool
board_storage_read(unsigned slot, void *data, size_t size)
{
    if (!layout) { return false; }
    memcpy(data,flash[slot],size); return true;
}
/** @brief 模拟擦写另一个槽并在指定字节处掉电。 */
bool
board_storage_write(unsigned slot, const void *data, size_t size)
{
    CHECK(slot<2 && size%8==0 && size<BOARD_STORAGE_SLOT_SIZE);
    if (!layout) { return false; }
    ++writes; record_size=size;
    memset(flash[slot],255,BOARD_STORAGE_SLOT_SIZE);
    size_t count=cut_after<size?cut_after:size;
    memcpy(flash[slot],data,count);
    return count==size;
}
/** @brief 建立 12 V 母线和 3.3 V 参考的测试场景。 */
bool
board_motor_init(void)
{
    memset(&hardware,0,sizeof(hardware)); hardware.bus_mv=12000; hardware.vdda_mv=3300;
    memset(&observation,0,sizeof(observation)); bemf_enabled=false;
    hardware.fault="NONE"; hardware.regular_count=1; return true;
}
/** @brief 停机选择电流/相电压采样配置。 @param enabled 开启无感同步采样。 @return 已停止时成功。 */
bool
board_motor_set_bemf(bool enabled)
{
    if(hardware.running) { return false; }
    bemf_enabled=enabled;
    return true;
}
/** @brief 返回本周期合成的同步相电压快照。 @return 与实际扇区关联的样本。 */
board_motor_observation_t
board_motor_observation(void) { return observation; }
/** @brief 后台/峰顶接口的板卡替身，实际保护和算法仍编译正式源码。 */
void board_motor_set_peak_callback(board_motor_peak_fn callback) { peak_callback=callback; }
bool board_motor_running(void) { return hardware.running; }
void board_motor_set_control_callbacks(board_motor_bemf_fn bemf,board_motor_speed_fn speed)
{
    bemf_callback=bemf; speed_callback=speed;
}
uint32_t board_motor_control_lock(void)
{
    if (stop_pi_locks>0U && --stop_pi_locks==0U)
    {
        hardware.running=false; hardware.fault="DRV NFAULT";
    }
    return 0U;
}
void board_motor_control_unlock(uint32_t mask) { (void)mask; }
bool board_motor_schedule(uint8_t sector,uint32_t deadline,uint16_t duty,uint32_t *commit)
{
    CHECK(!scheduled && deadline>observation.now_us);
    scheduled=controlled=true; scheduled_sector=sector; control_duty=duty;
    uint32_t period=1000000U/MOTOR_CONTROL_HZ;
    scheduled_us=(deadline+period-1U)/period*period; *commit=scheduled_us; return true;
}
void board_motor_set_control_duty(uint16_t duty) { control_duty=duty; }
uint32_t board_motor_cycles(void) { return 0U; }
/** @brief 真实电流保护后即时处理浮相，再模拟下一硬件峰顶及低频PI；OPEN保持原入口。 */
static void sample(uint16_t raw,uint8_t low,bool valid,board_motor_output_t *next)
{
    current_sample(raw,low,valid,next);
    if (!bemf_enabled || !hardware.running) { return; }
    if (bemf_callback!=NULL) { bemf_callback(&observation); }
    if (!hardware.running) { return; }
    uint8_t sector=observation.sector;
    if (scheduled && (int32_t)(observation.next_commit_us-scheduled_us)>=0)
    {
        sector=scheduled_sector; scheduled=false;
    }
    peak_callback(controlled,sector,control_duty,next);
    if (++pwm_speed_count >= MOTOR_CONTROL_HZ/MOTOR_SPEED_LOOP_HZ)
    {
        pwm_speed_count=0U;
        if (speed_callback!=NULL) { ++speed_calls; speed_callback(); }
    }
}
/** @brief 注入正常零点或校准失败。 */
bool
board_motor_calibrate(uint32_t shunt)
{
    CHECK(shunt==1 || shunt==50);
    ++calibration_calls;
    bool ok=calibration_ok && calibration_failures_left==0;
    if(calibration_failures_left>0) { --calibration_failures_left; }
    hardware.running=false; hardware.calibrated=ok;
    hardware.fault=ok?"NONE":"DRV READBACK";
    if(calibrated_bus_mv!=0) { hardware.bus_mv=calibrated_bus_mv; }
    for(unsigned i=0;i<3;i++) { hardware.zero[i]=2048; }
    return ok;
}
/** @brief 保存真实后端的高速回调并标记模拟运行。 */
bool
board_motor_start(const board_motor_output_t *initial, board_motor_sample_fn callback)
{
    if(start_failure_reason!=NULL) { hardware.fault=start_failure_reason; return false; }
    CHECK(hardware.calibrated); output=*initial; current_sample=callback; controlled=scheduled=false; pwm_speed_count=speed_calls=0U; hardware.running=true; return true;
}
/** @brief 同步模拟运行状态并保留故障原因。 */
void
board_motor_stop(const char *reason)
{
    hardware.running=false; hardware.calibrated=false;
    if(reason) { hardware.fault=reason; }
}
/** @brief 更新时间戳，避免主机测试伪造母线过期。 */
void
board_motor_service(uint32_t tick) { hardware.slow_ms=tick; }
/** @brief 返回板卡替身状态副本。 */
board_motor_status_t
board_motor_status(void) { return hardware; }
/** @brief 模拟仅停机时允许保存。 */
bool
board_motor_storage_safe(void) { return !hardware.running; }
/** @brief 提供两档已计算的最小脉宽供后端测试。 */
uint16_t
board_motor_min_duty(uint32_t shunt) { return shunt==50?391:646; }
/** @brief 按170 MHz中心对齐计数模拟占空比换算，随控制时基同步缩放。
 * @param duty 千分比占空比。 @return 比较值tick。 */
uint16_t
board_motor_duty_ticks(uint32_t duty) { return (uint16_t)((170000000U/(2U*MOTOR_CONTROL_HZ))*duty/1000U); }

/** @brief 模拟 DRV 写入/读取及指定事务失败。 @param tx 帧。 @param rx 返回。 @return 事务成功。 */
static bool transfer(uint16_t tx,uint16_t *rx)
{
    if(++transactions==fail_at) { return false; }
    unsigned address=(tx>>11)&15U;
    CHECK(address<7);
    if(!(tx&0x8000U)) { regs[address]=tx&0x7FFU; }
    *rx=regs[address]; return true;
}
/** @brief 每一笔初始化事务失败都不能报告成功，配置损坏必须被回读发现。 */
static void test_driver(void)
{
    transactions=fail_at=0;
    CHECK(drv8353f_init_gain(transfer,40));
    unsigned total=transactions;
    CHECK(regs[5]==0x0110 && regs[6]==0x02C0);
    for(unsigned failure=1;failure<=total;failure++)
    {
        transactions=0; fail_at=failure;
        CHECK(!drv8353f_init_gain(transfer,40));
    }
    transactions=fail_at=0;
    CHECK(drv8353f_init_gain(transfer,10)); CHECK(regs[6]==0x0240);
    CHECK(drv8353f_init_gain(transfer,20)); CHECK(regs[6]==0x0280);
    CHECK(drv8353f_calibrate(true)); CHECK((regs[6]&0x1C)==0x1C);
    CHECK(drv8353f_calibrate(false)); CHECK((regs[6]&0x1C)==0);
    CHECK(regs[6]==0x0280 && drv8353f_verify(true));
    unsigned before_invalid=transactions;
    CHECK(!drv8353f_init_gain(transfer,15)); CHECK(transactions==before_invalid);
    CHECK(drv8353f_coast(false)); CHECK(drv8353f_verify(false));
    regs[5]^=0x100; CHECK(!drv8353f_verify(false));
}
/** @brief 保存合并、运行延迟、两个槽的掉电边界与 CRC 损坏恢复。 */
static void test_persistence(void)
{
    memset(flash,255,sizeof(flash)); writes=0;
    control_model_init(); settings_init();
    CHECK(settings_get()->parameter[CONTROL_PARAM_POLES]==0);
    CHECK(settings_get()->parameter[CONTROL_PARAM_CURRENT_LIMIT]==1000);
    CHECK(settings_get()->parameter[CONTROL_PARAM_START_HZ]==MOTOR_START_DEFAULT_MHZ);
    settings_publish();
    CHECK(control_model_parameter(CONTROL_PARAM_START_HZ)->maximum==50000);
    /* 扩大的端点走真实参数保存和记录解码，越界提交不能污染已接受值。 */
    CHECK(settings_set(CONTROL_PARAM_START_HZ,50000,10));
    CHECK(!settings_set(CONTROL_PARAM_START_HZ,50001,10));
    CHECK(!settings_set(CONTROL_PARAM_START_HZ,99,10));
    CHECK(settings_get()->parameter[CONTROL_PARAM_START_HZ]==50000);
    /* 改默认值不能覆盖用户已经保存的0.5 A参数。 */
    CHECK(settings_set(CONTROL_PARAM_CURRENT_LIMIT,500,10));
    CHECK(settings_set(CONTROL_PARAM_POLES,7,10));
    CHECK(!settings_set(CONTROL_PARAM_SHUNT,2,10));
    settings_service(false,2000); CHECK(writes==0);
    settings_service(true,1009); CHECK(writes==0);
    settings_service(true,1010); CHECK(writes==1);
    CHECK(strcmp(settings_status(),"SAVED")==0);
    unsigned char stable[sizeof(flash)]; memcpy(stable,flash,sizeof(flash));
    for(unsigned cut=0;cut<record_size;cut+=8)
    {
        memcpy(flash,stable,sizeof(flash)); settings_init();
        CHECK(settings_set(CONTROL_PARAM_POLES,9,2000));
        cut_after=cut; settings_service(true,3000);
        CHECK(strcmp(settings_status(),"ERROR")==0);
        settings_init(); CHECK(settings_get()->parameter[CONTROL_PARAM_POLES]==7);
        CHECK(settings_get()->parameter[CONTROL_PARAM_START_HZ]==50000);
    }
    cut_after=UINT_MAX; memcpy(flash,stable,sizeof(flash)); settings_init();
    CHECK(settings_set(CONTROL_PARAM_POLES,9,4000));
    settings_source(CONTROL_REMOTE,4000);
    settings_capture(77,false,4000); settings_service(true,5000);
    settings_init(); CHECK(settings_get()->parameter[CONTROL_PARAM_POLES]==9);
    CHECK(settings_get()->parameter[CONTROL_PARAM_START_HZ]==50000);
    CHECK(settings_get()->parameter[CONTROL_PARAM_CURRENT_LIMIT]==500);
    CHECK(settings_get()->brightness==77 && !settings_get()->animation && settings_get()->source==CONTROL_REMOTE);
    flash[0][16]^=1; settings_init(); CHECK(settings_get()->parameter[CONTROL_PARAM_POLES]==7);
    layout=false; settings_init(); CHECK(strcmp(settings_status(),"ERROR")==0); layout=true;
}
/** @brief 核查不同控制时基下相同物理时长的电角积分、加减速和运行截止。 */
static void
test_six_step(void)
{
    six_step_t state;
    six_step_config_t config={1000,1000,5000,MOTOR_CONTROL_HZ/10U,2U*MOTOR_CONTROL_HZ,100,0};
    six_step_start(&state,&config);
    for(unsigned i=0;i<MOTOR_CONTROL_HZ/10U;i++) { six_step_tick(&state); }
    CHECK(state.sector==0);
    for(unsigned i=0;i<MOTOR_CONTROL_HZ;i++) { six_step_tick(&state); }
    CHECK(state.sector==0 && state.phase==0);
    state.config.target_mhz=2000;
    for(unsigned i=0;i<MOTOR_CONTROL_HZ/5U;i++) { six_step_tick(&state); }
    CHECK(state.frequency_mhz==2000);
    state.config.target_mhz=1000;
    for(unsigned i=0;i<MOTOR_CONTROL_HZ/5U;i++) { six_step_tick(&state); }
    CHECK(state.frequency_mhz==1000);
    while(state.running) { six_step_tick(&state); }
    CHECK(state.ticks==config.run_ticks);
    for(uint8_t sector=0;sector<6;sector++)
    {
        uint8_t high,low; six_step_phases(sector,&high,&low);
        CHECK(high<3 && low<3 && high!=low);
    }
    CHECK(six_step_regular_fits(17000,10562,340));
    CHECK(!six_step_regular_fits(10902,10562,340));
    CHECK(!six_step_regular_fits(100,10562,340));
}
/** @brief 验证200 ms定位渐增/保持、5 Hz整电周期牵引和原20 Hz/s加速的物理边界。 */
static void
test_startup_stages(void)
{
    six_step_t state;
    const uint32_t align=MOTOR_CONTROL_HZ/5U, ramp=align/2U;
    six_step_config_t config={50000,5000,20000,align,4U*MOTOR_CONTROL_HZ,200,6};
    six_step_start(&state,&config);
    CHECK(state.stage==SIX_STEP_ALIGN_RAMP && state.duty==0 && state.sector==0 && state.phase==0);
    for(uint32_t tick=1;tick<=align;tick++)
    {
        six_step_tick(&state);
        uint16_t expected=(uint16_t)(tick<ramp?(uint64_t)tick*200U/ramp:200U);
        CHECK(state.duty==expected && state.frequency_mhz==5000 && state.sector==0 && state.phase==0);
        CHECK(state.stage==(tick<ramp?SIX_STEP_ALIGN_RAMP:SIX_STEP_ALIGN_HOLD));
    }
    for(uint32_t tick=1;tick<=MOTOR_CONTROL_HZ/5U;tick++)
    {
        six_step_tick(&state);
        uint32_t total=tick*30000U;
        uint32_t transitions=total/(MOTOR_CONTROL_HZ*1000U);
        CHECK(state.stage==SIX_STEP_PULLIN && state.frequency_mhz==5000 && state.duty==200);
        CHECK(state.sector==transitions%6U && state.pullin_remaining==6U-transitions);
        CHECK(state.phase==total%(MOTOR_CONTROL_HZ*1000U));
    }
    CHECK(state.sector==0 && state.phase==0 && state.pullin_remaining==0);
    six_step_tick(&state);
    CHECK(state.stage==SIX_STEP_RAMP && state.frequency_mhz==5000U+20000U/MOTOR_CONTROL_HZ);
    CHECK(state.phase==6U*state.frequency_mhz && state.sector==0);
    const uint32_t acceleration_ticks=MOTOR_CONTROL_HZ*9U/4U;
    for(uint32_t tick=1;tick<acceleration_ticks;tick++) { six_step_tick(&state); }
    CHECK(state.frequency_mhz==50000 && state.stage==SIX_STEP_RUN);
    CHECK(state.ticks==align+MOTOR_CONTROL_HZ/5U+acceleration_ticks);
    while(state.running) { six_step_tick(&state); }
    CHECK(state.ticks==config.run_ticks && state.stage==SIX_STEP_RUN && state.duty==200);
}

/** @brief 非整数初频的第六次转移保留相位余数，在线目标修改不重做定位或牵引。 */
static void
test_fractional_pullin_and_target(void)
{
    six_step_t state;
    six_step_config_t config={20000,7333,20000,0,2U*MOTOR_CONTROL_HZ,200,6};
    six_step_start(&state,&config);
    CHECK(state.stage==SIX_STEP_PULLIN && state.duty==200);
    const uint32_t period_ticks=(MOTOR_CONTROL_HZ*1000U+7332U)/7333U;
    unsigned transitions=0;
    for(uint32_t tick=1;tick<=period_ticks;tick++)
    {
        uint8_t prior=state.sector;
        if(tick==period_ticks/2U) { state.config.target_mhz=25000; }
        six_step_tick(&state);
        if(prior!=state.sector) { ++transitions; CHECK(state.sector==(prior+1U)%6U); }
        CHECK(state.frequency_mhz==7333 && state.stage==SIX_STEP_PULLIN);
        CHECK(state.phase==(uint64_t)tick*7333U*6U%(MOTOR_CONTROL_HZ*1000U));
    }
    CHECK(transitions==6 && state.pullin_remaining==0 && state.phase!=0 && state.sector==0);
    uint32_t prior_phase=state.phase;
    six_step_tick(&state);
    CHECK(state.stage==SIX_STEP_RAMP && state.frequency_mhz==7333U+20000U/MOTOR_CONTROL_HZ);
    CHECK(state.phase==prior_phase+state.frequency_mhz*6U && state.sector==0);
    CHECK(state.config.target_mhz==25000 && state.duty==200);
    while(state.frequency_mhz<25000) { six_step_tick(&state); }
    CHECK(state.stage==SIX_STEP_RUN);
    state.config.target_mhz=10000;
    uint8_t prior_sector=state.sector;
    prior_phase=state.phase;
    six_step_tick(&state);
    CHECK(state.stage==SIX_STEP_RAMP && state.frequency_mhz<25000 && state.pullin_remaining==0);
    CHECK(state.phase==(prior_phase+state.frequency_mhz*6U)%(MOTOR_CONTROL_HZ*1000U));
    CHECK(state.sector==(prior_sector+(prior_phase+state.frequency_mhz*6U>=MOTOR_CONTROL_HZ*1000U))%6U);
}

/** @brief 定位零/单/奇数周期正确选阶段，重新启动清除旧相位，总时限可在加速中结束。 */
static void
test_startup_alignment_edges(void)
{
    six_step_t state;
    six_step_config_t config={50000,5000,20000,0,MOTOR_CONTROL_HZ/2U,250,6};
    for(uint32_t align=0;align<=3;align++)
    {
        config.align_ticks=align;
        six_step_start(&state,&config);
        CHECK(state.sector==0 && state.phase==0 && state.ticks==0 && state.pullin_remaining==6);
        CHECK(state.duty==(align>=2?0:250));
        CHECK(state.stage==(align>=2?SIX_STEP_ALIGN_RAMP:align?SIX_STEP_ALIGN_HOLD:SIX_STEP_PULLIN));
        for(uint32_t tick=1;tick<=align;tick++)
        {
            six_step_tick(&state);
            CHECK(state.duty==250 && state.stage==SIX_STEP_ALIGN_HOLD);
            CHECK(state.sector==0 && state.phase==0);
        }
        six_step_tick(&state);
        CHECK(state.stage==SIX_STEP_PULLIN && state.frequency_mhz==5000 && state.phase==30000);
    }
    config.align_ticks=MOTOR_CONTROL_HZ/5U;
    six_step_start(&state,&config);
    while(state.running) { six_step_tick(&state); }
    CHECK(state.ticks==config.run_ticks && state.stage==SIX_STEP_RAMP && state.frequency_mhz<50000);
    six_step_t stopped=state;
    six_step_tick(&state);
    CHECK(memcmp(&stopped,&state,sizeof(state))==0);
}
/** @brief 提交真实模型请求并让真实后端处理。 @param kind 操作。 @param id 参数。 @param value 值。 */
static void request(control_request_kind_t kind,control_parameter_id_t id,int32_t value)
{
    control_result_t result=control_model_request(kind,id,value,now);
    /* 失败时同时报告请求和模型快照，避免所有前置状态错误只停在同一断言行。 */
    if(result!=CONTROL_OK)
    {
        fprintf(stderr,"request denied: kind=%u parameter=%u value=%ld result=%s run=%u notice=%s\n",
            (unsigned)kind,(unsigned)id,(long)value,control_result_name(result),
            (unsigned)control_model_snapshot()->run,control_model_notice());
    }
    CHECK(result==CONTROL_OK);
    motor_backend_service(++now);
    CHECK(control_model_pending()==NULL);
}
/** @brief 从诊断读取一个无符号计数，用增量核对保护，避免渐增过程污染原统计场景。
 * @param line 完整诊断。 @param field 带等号的字段名。 @return 字段整数值。 */
static uint32_t
diagnostic_count(const char *line,const char *field)
{
    const char *value=strstr(line,field);
    CHECK(value!=NULL);
    return (uint32_t)strtoul(value+strlen(field),NULL,10);
}
/** @brief 让既有保护场景到达完整定位幅度；无效大码值不应触发保护。
 * @param align_ms 本场景保存的定位时间，单位ms。 */
static void
finish_alignment_ramp(unsigned align_ms)
{
    for(unsigned tick=0;tick<align_ms*(MOTOR_CONTROL_HZ/1000U)/2U;tick++)
    {
        sample(4095,output.low,false,&output);
        CHECK(hardware.running);
    }
    CHECK(output.pulse);
}
/** @brief 冷启动瞬态失败允许一次重试；持续故障保持停机，不在主循环无限重试。 */
static void test_boot_calibration(void)
{
    memset(flash,255,sizeof(flash)); control_model_init(); settings_init();
    CHECK(control_model_select(CONTROL_OPEN_6STEP,CONTROL_SPEED)==CONTROL_OK);
    calibration_ok=true; calibration_calls=0; calibration_failures_left=1;
    motor_backend_init(); motor_backend_service(now=10);
    CHECK(calibration_calls==2 && strcmp(hardware.fault,"NONE")==0);
    CHECK(!hardware.running && control_model_snapshot()->run==CONTROL_STOPPED);
    calibration_calls=0; calibration_ok=false;
    motor_backend_init(); motor_backend_service(now+=20);
    CHECK(calibration_calls==2 && control_model_snapshot()->run==CONTROL_FAULT);
    CHECK(!hardware.running);
    motor_backend_service(now+=20); CHECK(calibration_calls==2);
    CHECK(control_model_request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0,now)==CONTROL_BUSY);
    CHECK(!hardware.running && calibration_calls==2);
    calibration_ok=true; calibration_calls=0;
    motor_backend_init(); motor_backend_service(now+=20);
    CHECK(calibration_calls==1 && strcmp(hardware.fault,"NONE")==0 && !hardware.running);
}
/** @brief 缺失参数拒绝启动、有效电流限幅、过流和驱动失败。 */
static void test_backend(void)
{
    memset(flash,255,sizeof(flash)); control_model_init(); settings_init();
    CHECK(control_model_select(CONTROL_OPEN_6STEP,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_init(); motor_backend_service(now=10);
    /* 复现用户日志：DRV 无故障但布局不匹配，不能保存或接受远程来源。 */
    layout=false; settings_init(); motor_backend_service(now+=20);
    CHECK(strcmp(hardware.fault,"NONE")==0 && !control_model_snapshot()->ready);
    CHECK(strcmp(settings_status(),"ERROR")==0);
    CHECK(control_model_request(CONTROL_REQUEST_SOURCE,CONTROL_PARAM_TARGET,CONTROL_REMOTE,now)==CONTROL_NOT_READY);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SHUNT,50);
    motor_backend_service(now+=1000);
    CHECK(strcmp(settings_status(),"ERROR")==0 && settings_get()->parameter[CONTROL_PARAM_SHUNT]==50);
    char storage[384]; motor_backend_storage_diagnostics(storage,sizeof(storage));
    CHECK(strstr(storage,"SAVE=ERROR RESULT=LAYOUT ACTUAL_DBANK=1 BUILD_DBANK=0 MATCH=0")!=NULL);
    CHECK(strstr(storage,"FLASH_KB=128")!=NULL && strlen(storage)<sizeof(storage)-1);
    layout=true; settings_init(); settings_publish(); motor_backend_service(now+=20);
    CHECK(control_model_snapshot()->ready);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"SET POLE PAIRS")==0);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_POLES,7);
    /* 1 mΩ档20 kHz下10%需要密度调制；此保护场景使用20%完整可采样脉冲。 */
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_DUTY,200);
    /* 保留原0.5 A保护场景，避免测试条件随默认值变化。 */
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_CURRENT_LIMIT,500);
    CHECK(control_model_request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0,now)==CONTROL_OK);
    CHECK(control_model_request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0,now)==CONTROL_OK);
    motor_backend_service(++now); CHECK(!hardware.running);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0); CHECK(hardware.running);
    CHECK(output.high==0 && output.low==1 && !output.pulse);
    finish_alignment_ramp(MOTOR_ALIGN_DEFAULT_MS);
    CHECK(control_model_request(CONTROL_REQUEST_CLEAR_FAULT,CONTROL_PARAM_TARGET,0,now)==CONTROL_BUSY);
    CHECK(control_model_request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SHUNT,50,now)==CONTROL_BUSY);
    /* 1 mΩ/40倍：0.5 A约25码、2 A约99码；无效窗口不能冒充电流样本。 */
    sample(2048+30,1,true,&output); CHECK(!output.pulse && hardware.running);
    sample(4095,1,false,&output); CHECK(hardware.running);
    sample(2048+105,1,true,&output); CHECK(!hardware.running);
    motor_backend_service(++now); CHECK(control_model_snapshot()->run==CONTROL_FAULT);
    CHECK(!control_model_snapshot()->values[CONTROL_SPEED_VALUE].valid);
    request(CONTROL_REQUEST_CLEAR_FAULT,CONTROL_PARAM_TARGET,0);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SHUNT,50);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0); CHECK(hardware.running);
    finish_alignment_ramp(MOTOR_ALIGN_DEFAULT_MS);
    sample(2048+1200,1,true,&output); CHECK(hardware.running);
    /* 测量快照为 50 Hz，保护已在上面的真实快速回调中执行。 */
    now+=20; motor_backend_service(now); CHECK(control_model_snapshot()->values[CONTROL_IB].value>1900);
    sample(2048+1250,1,true,&output); CHECK(!hardware.running);
    motor_backend_service(++now);
    /* START前旧值为12 V，校准后降至7 V时必须拒绝输出。 */
    request(CONTROL_REQUEST_CLEAR_FAULT,CONTROL_PARAM_TARGET,0);
    calibrated_bus_mv=7000;
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"BUS VOLTAGE")==0);
    calibrated_bus_mv=12000;
    request(CONTROL_REQUEST_CLEAR_FAULT,CONTROL_PARAM_TARGET,0);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0); CHECK(hardware.running);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    calibrated_bus_mv=0;
    start_failure_reason="TIM PHASE";
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"TIM PHASE")==0);
    start_failure_reason=NULL;
    request(CONTROL_REQUEST_CLEAR_FAULT,CONTROL_PARAM_TARGET,0);
    calibration_ok=false;
    request(CONTROL_REQUEST_CLEAR_FAULT,CONTROL_PARAM_TARGET,0); CHECK(!hardware.running);
    CHECK(strcmp(control_model_notice(),"DRV/ADC CAL")==0);
    calibration_ok=true;
    request(CONTROL_REQUEST_CLEAR_FAULT,CONTROL_PARAM_TARGET,0);
    CHECK(control_model_select(CONTROL_ENCODER_FOC,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(++now); CHECK(!control_model_snapshot()->ready);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_D_KP,1234);
    CHECK(settings_get()->parameter[CONTROL_PARAM_D_KP]==1234);
    CHECK(control_model_request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0,now)==CONTROL_NOT_READY);
}

/** @brief 建立每次RUN独立的50 mΩ测试场景，后端与请求模型仍使用正式源码。 */
static void reset_run_backend(void)
{
    memset(flash,255,sizeof(flash));
    layout=calibration_ok=true; calibration_failures_left=0;
    calibrated_bus_mv=0; start_failure_reason=NULL;
    control_model_init(); settings_init();
    CHECK(control_model_select(CONTROL_OPEN_6STEP,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_init(); motor_backend_service(now=10);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_POLES,3);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SHUNT,50);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_ALIGN_MS,10);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_DUTY,200);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_CURRENT_LIMIT,500);
}

/** @brief 有效/无效窗口分别计数，负电流峰值保留，过流和正常停机均不丢诊断，重启清零。 */
static void test_run_statistics(void)
{
    reset_run_backend();
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    char line[384], stopped[384];
    motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"TICKS=0 VALID=0 INVALID=0 LIMIT_HITS=0 DENSITY_SKIP=1")!=NULL);
    CHECK(strstr(line,"PHASE_SAMPLES=0,0,0 PEAK_MA=NA,NA,NA LOW=NA RAW=NA DELTA=NA")!=NULL);
    finish_alignment_ramp(10);
    motor_backend_run_diagnostics(line,sizeof(line));
    uint32_t warmup=diagnostic_count(line,"TICKS="), density=diagnostic_count(line,"DENSITY_SKIP=");
    sample(2048+100,0,true,&output); CHECK(output.pulse);
    sample(2048-400,1,true,&output); CHECK(!output.pulse);
    sample(4095,2,false,&output); CHECK(output.pulse && hardware.running);
    sample(2048+300,2,true,&output);
    motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(diagnostic_count(line,"TICKS=")==warmup+4U && diagnostic_count(line,"VALID=")==3);
    CHECK(diagnostic_count(line,"INVALID=")==warmup+1U && diagnostic_count(line,"LIMIT_HITS=")==1);
    CHECK(diagnostic_count(line,"DENSITY_SKIP=")==density && strstr(line,"STAGE=ALIGN_HOLD")!=NULL);
    CHECK(strstr(line,"PHASE_SAMPLES=1,1,1 PEAK_MA=161,644,483")!=NULL);
    CHECK(strstr(line,"LOW=C RAW=2348 DELTA=300")!=NULL);
    CHECK(strstr(line,"DUTY=20.0")!=NULL);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    motor_backend_run_diagnostics(stopped,sizeof(stopped));
    CHECK(strcmp(stopped,line)==0);
    /* 后续换档且启动校准失败时，仍按上次RUN的量程解释历史峰值。 */
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SHUNT,1);
    calibration_ok=false;
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running);
    motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strcmp(stopped,line)==0);
    calibration_ok=true;
    request(CONTROL_REQUEST_CLEAR_FAULT,CONTROL_PARAM_TARGET,0);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SHUNT,50);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"TICKS=0 VALID=0 INVALID=0 LIMIT_HITS=0 DENSITY_SKIP=1")!=NULL);
    CHECK(strstr(line,"PEAK_MA=NA,NA,NA LOW=NA RAW=NA DELTA=NA")!=NULL);
    sample(2048+1300,1,true,&output);
    CHECK(!hardware.running && strcmp(hardware.fault,"PHASE OVERCURRENT")==0);
    motor_backend_service(++now);
    motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"TICKS=1 VALID=1 INVALID=0 LIMIT_HITS=1")!=NULL);
    CHECK(strstr(line,"PHASE_SAMPLES=0,1,0 PEAK_MA=NA,2095,NA LOW=B RAW=3348 DELTA=1300")!=NULL);
}

/** @brief 200 ms低占空比渐增后牵引，从首周期遵循累计密度预算及总运行时限。 */
static void
test_first_density_pulse(void)
{
    reset_run_backend();
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_DUTY,10);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,10000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,200);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!output.pulse && output.duty_ticks==391);
    unsigned pulses=0;
    uint32_t budget=0;
    const unsigned ramp_ticks=MOTOR_CONTROL_HZ/200U, ticks=MOTOR_CONTROL_HZ/5U;
    for(unsigned cycle=0;cycle<ticks;cycle++)
    {
        if(output.pulse) { ++pulses; }
        uint32_t duty=cycle<ramp_ticks?10U*cycle/ramp_ticks:10U;
        budget+=board_motor_duty_ticks(duty);
        /* 用时间线积分指令宽度，检查早段不会额外赠送最小脉冲。 */
        CHECK(pulses*391U<=budget && budget-pulses*391U<391U);
        sample(2048,output.low,output.pulse,&output);
        if(cycle==0)
        {
            char first[384]; motor_backend_run_diagnostics(first,sizeof(first));
            CHECK(strstr(first,"TICKS=1 VALID=0 INVALID=1 LIMIT_HITS=0 DENSITY_SKIP=2")!=NULL);
            CHECK(strstr(first,"PEAK_MA=NA,NA,NA LOW=NA")!=NULL);
        }
    }
    CHECK(!hardware.running && pulses==budget/391U);
    CHECK(budget-pulses*391U<391U);
    char line[384], expected[96];
    motor_backend_run_diagnostics(line,sizeof(line));
    snprintf(expected,sizeof(expected),"TICKS=%u VALID=%u INVALID=%u LIMIT_HITS=0 DENSITY_SKIP=%u",ticks,pulses,ticks-pulses,ticks-pulses);
    CHECK(strstr(line,expected)!=NULL);
}

/** @brief 1 A软门限允许原0.5 A以上电流，超1 A跳脉冲，2 A过流停机仍有效。 */
static void test_one_amp_limit(void)
{
    reset_run_backend();
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_CURRENT_LIMIT,1000);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    finish_alignment_ramp(10);
    /* 50 mΩ/10倍/3.3 V：500码约0.806 A，650码约1.048 A。 */
    sample(2548,1,true,&output);
    CHECK(hardware.running && output.pulse);
    sample(2698,1,true,&output);
    CHECK(hardware.running && !output.pulse);
    sample(4095,1,false,&output);
    CHECK(hardware.running && output.pulse);
    /* 极性反向也使用绝对电流判断，仍不把无效样本用作保护依据。 */
    sample(1398,1,true,&output);
    CHECK(hardware.running && !output.pulse);
    sample(3348,1,true,&output);
    CHECK(!hardware.running && strcmp(hardware.fault,"PHASE OVERCURRENT")==0);
    char line[384]; motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(diagnostic_count(line,"TICKS=")==MOTOR_CONTROL_HZ/200U+5U);
    CHECK(diagnostic_count(line,"VALID=")==4 && diagnostic_count(line,"INVALID=")==MOTOR_CONTROL_HZ/200U+1U);
    CHECK(diagnostic_count(line,"LIMIT_HITS=")==3);
}

/** @brief 无有效电流样本时仍显示当前渐增幅度与阶段，不能用保存满幅代替启动指令。 */
static void
test_startup_snapshot(void)
{
    reset_run_backend();
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    const control_snapshot_t *snapshot=control_model_snapshot();
    CHECK(hardware.running && !output.pulse && snapshot->run==CONTROL_RUNNING);
    CHECK(snapshot->values[CONTROL_DUTY].valid && snapshot->values[CONTROL_DUTY].value==0);
    CHECK(snapshot->values[CONTROL_DUTY].decimals==1 && strcmp(snapshot->startup,"ALIGN_RAMP")==0);
    CHECK(snapshot->values[CONTROL_COMM_HZ].valid && snapshot->values[CONTROL_COMM_HZ].value==1000);
    CHECK(!snapshot->values[CONTROL_IA].valid && !snapshot->values[CONTROL_IB].valid && !snapshot->values[CONTROL_IC].valid);
    sample(4095,output.low,false,&output);
    motor_backend_service(now+=20);
    snapshot=control_model_snapshot();
    CHECK(snapshot->values[CONTROL_DUTY].valid && snapshot->values[CONTROL_DUTY].value==200U/(MOTOR_CONTROL_HZ/200U));
    CHECK(snapshot->values[CONTROL_DUTY].value<settings_get()->parameter[CONTROL_PARAM_DUTY]);
    CHECK(strcmp(snapshot->startup,"ALIGN_RAMP")==0 && !snapshot->values[CONTROL_IB].valid);
    for(unsigned tick=1;tick<MOTOR_CONTROL_HZ/200U;tick++) { sample(4095,output.low,false,&output); }
    motor_backend_service(now+=20);
    snapshot=control_model_snapshot();
    CHECK(snapshot->values[CONTROL_DUTY].valid && snapshot->values[CONTROL_DUTY].value==200);
    CHECK(strcmp(snapshot->startup,"ALIGN_HOLD")==0 && output.pulse);
    sample(2048,output.low,true,&output);
    motor_backend_service(now+=20);
    CHECK(control_model_snapshot()->values[CONTROL_IB].valid && control_model_snapshot()->values[CONTROL_IB].value==0);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
}

/** @brief 20%定位跨过最小脉宽时累计真实导通不超过渐增预算，随后满幅保持不再跳周期。 */
static void
test_alignment_pulse_budget(void)
{
    reset_run_backend();
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_ALIGN_MS,200);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    const unsigned ramp_ticks=MOTOR_CONTROL_HZ/10U, align_ticks=MOTOR_CONTROL_HZ/5U;
    uint32_t budget=0, delivered=0;
    for(unsigned tick=0;tick<align_ticks;tick++)
    {
        uint32_t duty=tick<ramp_ticks?200U*tick/ramp_ticks:200U;
        uint32_t requested=board_motor_duty_ticks(duty);
        budget+=requested;
        if(output.pulse) { delivered+=output.duty_ticks; }
        CHECK(delivered<=budget && budget-delivered<391U);
        CHECK(output.high==0 && output.low==1 && output.duty_ticks>=391U);
        if(requested>=391U) { CHECK(output.pulse && output.duty_ticks==requested); }
        sample(output.pulse?2048:4095,output.low,output.pulse,&output);
        CHECK(hardware.running && strcmp(hardware.fault,"NONE")==0);
    }
    char line[384]; motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"DUTY=20.0 STAGE=ALIGN_HOLD")!=NULL);
    CHECK(diagnostic_count(line,"INVALID=")==diagnostic_count(line,"DENSITY_SKIP="));
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
}

/** @brief 普通启动拒绝装不下整电周期牵引的RUN；整数、非整数与最低初频均在校准前检查。 */
static void
test_startup_time_budget(void)
{
    reset_run_backend();
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SPEED_LIMIT,3000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_ALIGN_MS,200);
    const unsigned frequencies[3]={5000,7333,100};
    const unsigned boundary_ms[3]={400,336,10200};
    for(unsigned scenario=0;scenario<3;scenario++)
    {
        request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,(int32_t)frequencies[scenario]);
        request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,(int32_t)boundary_ms[scenario]);
        unsigned before=calibration_calls;
        request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
        CHECK(!hardware.running && calibration_calls==before);
        CHECK(strcmp(control_model_notice(),"RUN/START TIME")==0);
        request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,(int32_t)boundary_ms[scenario]+1);
        request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
        CHECK(hardware.running && calibration_calls==before+1U && !output.pulse);
        request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    }
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,200);
    unsigned before=calibration_calls;
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && calibration_calls==before && strcmp(control_model_notice(),"RUN/ALIGN TIME")==0);
}

/** @brief 在各启动阶段注入有效过流与无效大码值，保护始终优先于定时换相。 */
static void
test_startup_stage_protection(void)
{
    const char *stages[5]={"ALIGN_RAMP","ALIGN_HOLD","PULLIN","RAMP","RUN"};
    const unsigned points[5]={0,MOTOR_CONTROL_HZ/200U,MOTOR_CONTROL_HZ/100U+1U,
        MOTOR_CONTROL_HZ*11U/100U+1U,MOTOR_CONTROL_HZ*51U/100U};
    for(unsigned scenario=0;scenario<5;scenario++)
    {
        reset_run_backend();
        request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SPEED_LIMIT,3000);
        request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1000);
        request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,10000);
        request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RAMP,100000);
        request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,1000);
        request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
        for(unsigned tick=0;tick<points[scenario];tick++)
        {
            sample(output.pulse?2048:4095,output.low,output.pulse,&output);
            CHECK(hardware.running && strcmp(hardware.fault,"NONE")==0);
        }
        char line[384], expected[40];
        motor_backend_run_diagnostics(line,sizeof(line));
        snprintf(expected,sizeof(expected),"STAGE=%s",stages[scenario]);
        CHECK(strstr(line,expected)!=NULL);
        uint32_t ticks=diagnostic_count(line,"TICKS="), hits=diagnostic_count(line,"LIMIT_HITS=");
        sample(2698,output.low,true,&output);
        CHECK(hardware.running && !output.pulse);
        sample(4095,output.low,false,&output);
        CHECK(hardware.running && strcmp(hardware.fault,"NONE")==0);
        sample(3348,output.low,true,&output);
        CHECK(!hardware.running && strcmp(hardware.fault,"PHASE OVERCURRENT")==0);
        motor_backend_run_diagnostics(line,sizeof(line));
        CHECK(diagnostic_count(line,"TICKS=")==ticks+3U && diagnostic_count(line,"LIMIT_HITS=")==hits+2U);
        CHECK(strstr(line,expected)!=NULL);
        motor_backend_service(++now);
        CHECK(control_model_snapshot()->run==CONTROL_FAULT);
    }
}

/** @brief 运行中修改目标仅改变后续频率轨迹，既不重新校准也不重复定位/牵引。 */
static void
test_startup_live_target(void)
{
    reset_run_backend();
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SPEED_LIMIT,3000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,5000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RAMP,20000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_ALIGN_MS,200);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,3000);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    for(unsigned tick=0;tick<MOTOR_CONTROL_HZ/4U;tick++)
    {
        sample(2048,output.low,output.pulse,&output);
    }
    char before[384], after[384];
    motor_backend_run_diagnostics(before,sizeof(before));
    CHECK(strstr(before,"HZ=5.000 DUTY=20.0 STAGE=PULLIN")!=NULL);
    unsigned calibration_before=calibration_calls;
    board_motor_output_t previous=output;
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,900);
    CHECK(hardware.running && calibration_calls==calibration_before && settings_get()->parameter[CONTROL_PARAM_TARGET]==900);
    CHECK(output.high==previous.high && output.low==previous.low && output.duty_ticks==previous.duty_ticks && output.pulse==previous.pulse);
    motor_backend_run_diagnostics(after,sizeof(after));
    CHECK(strcmp(before,after)==0);
    for(unsigned tick=MOTOR_CONTROL_HZ/4U;tick<12U*MOTOR_CONTROL_HZ/5U;tick++)
    {
        sample(2048,output.low,output.pulse,&output);
        CHECK(hardware.running);
    }
    motor_backend_run_diagnostics(after,sizeof(after));
    CHECK(strstr(after,"HZ=45.000 DUTY=20.0 STAGE=RUN")!=NULL);
    CHECK(diagnostic_count(after,"TICKS=")==12U*MOTOR_CONTROL_HZ/5U);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,600);
    sample(2048,output.low,output.pulse,&output);
    motor_backend_run_diagnostics(after,sizeof(after));
    CHECK(strstr(after,"STAGE=RAMP")!=NULL && strstr(after,"DUTY=20.0")!=NULL);
    CHECK(calibration_calls==calibration_before);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
}

/** @brief 推进真实后端零电流运行，仅汇总扇区错序，避免逐周期重复实现积分算法。
 * @param cycles 待完成的控制周期数。 @return 本段相邻扇区转移次数。 */
static unsigned
run_zero_current_cycles(unsigned cycles)
{
    /* 高侧/低侧相对直接映射AB、AC、BC、BA、CA、CB；对角无有效扇区。 */
    static const uint8_t sector[9]={255,0,1,3,255,2,4,5,255};
    unsigned completed=0, transitions=0, errors=0;
    for(;completed<cycles && hardware.running;completed++)
    {
        uint8_t before=sector[output.high*3U+output.low];
        sample(2048,output.low,output.pulse,&output);
        uint8_t after=sector[output.high*3U+output.low];
        if(hardware.running && before!=after)
        {
            ++transitions;
            if(before>=6 || after!=(before+1U)%6U) { ++errors; }
        }
    }
    CHECK(completed==cycles && errors==0);
    return transitions;
}

/** @brief 建立用户当前20%/5 Hz/20 Hz每秒/3极对基线并进入50 Hz稳定指令。
 * @param run_ms 整次RUN的总时限，单位ms。 */
static void
start_running_target_baseline(unsigned run_ms)
{
    reset_run_backend();
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SPEED_LIMIT,3000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,5000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RAMP,20000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_ALIGN_MS,200);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,(int32_t)run_ms);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_CURRENT_LIMIT,1000);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running);
    run_zero_current_cycles(53U*MOTOR_CONTROL_HZ/20U);
    char line[384]; motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"HZ=50.000 DUTY=20.0 STAGE=RUN")!=NULL);
}

/** @brief 在低于初频的目标修改边界直接核对相位余数，低频一秒仍连续积分而非静态定位。 */
static void
test_running_near_stop_phase(void)
{
    six_step_t state;
    six_step_config_t config={50000,50000,20000,0,10U*MOTOR_CONTROL_HZ,200,0};
    six_step_start(&state,&config);
    six_step_tick(&state);
    uint32_t phase=state.phase;
    uint8_t sector=state.sector;
    state.config.target_mhz=50;
    six_step_tick(&state);
    uint32_t frequency=50000U-20000U/MOTOR_CONTROL_HZ;
    CHECK(state.frequency_mhz==frequency && state.stage==SIX_STEP_RAMP && state.duty==200);
    CHECK(state.phase==(phase+frequency*6U)%(MOTOR_CONTROL_HZ*1000U));
    CHECK(state.sector==(sector+(phase+frequency*6U>=MOTOR_CONTROL_HZ*1000U))%6U);
    for(unsigned tick=1;tick<999U*MOTOR_CONTROL_HZ/400U;tick++) { six_step_tick(&state); }
    CHECK(state.frequency_mhz==50 && state.stage==SIX_STEP_RUN);
    phase=state.phase; sector=state.sector;
    for(unsigned tick=0;tick<MOTOR_CONTROL_HZ;tick++) { six_step_tick(&state); }
    uint32_t total=phase+300U*MOTOR_CONTROL_HZ;
    CHECK(state.phase==total%(MOTOR_CONTROL_HZ*1000U));
    CHECK(state.sector==(sector+total/(MOTOR_CONTROL_HZ*1000U))%6U);
    phase=state.phase; sector=state.sector;
    state.config.target_mhz=50000;
    six_step_tick(&state);
    frequency=50U+20000U/MOTOR_CONTROL_HZ;
    CHECK(state.frequency_mhz==frequency && state.stage==SIX_STEP_RAMP && state.pullin_remaining==0);
    CHECK(state.phase==(phase+frequency*6U)%(MOTOR_CONTROL_HZ*1000U));
    CHECK(state.sector==(sector+(phase+frequency*6U>=MOTOR_CONTROL_HZ*1000U))%6U);
}

/** @brief 在线1000→1→1000 RPM按原斜率经过近停，保持换相连续且不重置10秒总时限。 */
static void
test_running_near_stop_target(void)
{
    start_running_target_baseline(10000);
    unsigned calibration_before=calibration_calls;
    const unsigned ramp_ticks=999U*MOTOR_CONTROL_HZ/400U;
    char before[384], after[384];
    motor_backend_run_diagnostics(before,sizeof(before));
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,0);
    CHECK(strcmp(control_model_notice(),"TARGET/SPEED LIMIT")==0 && hardware.running);
    CHECK(settings_get()->parameter[CONTROL_PARAM_TARGET]==1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1);
    CHECK(strcmp(control_model_notice(),"OK")==0 && settings_get()->parameter[CONTROL_PARAM_TARGET]==1);
    motor_backend_run_diagnostics(after,sizeof(after));
    CHECK(strcmp(before,after)==0 && calibration_calls==calibration_before);
    run_zero_current_cycles(1);
    motor_backend_run_diagnostics(after,sizeof(after));
    CHECK(strstr(after,"STAGE=RAMP")!=NULL && strstr(after,"DUTY=20.0")!=NULL);
    CHECK(diagnostic_count(after,"HZ=")==49);
    run_zero_current_cycles(ramp_ticks-2U);
    motor_backend_run_diagnostics(after,sizeof(after));
    CHECK(strstr(after,"STAGE=RAMP")!=NULL);
    run_zero_current_cycles(1);
    motor_backend_run_diagnostics(after,sizeof(after));
    CHECK(strstr(after,"HZ=0.050 DUTY=20.0 STAGE=RUN")!=NULL);
    CHECK(run_zero_current_cycles(MOTOR_CONTROL_HZ)<=1U);
    motor_backend_run_diagnostics(after,sizeof(after));
    CHECK(strstr(after,"HZ=0.050 DUTY=20.0 STAGE=RUN")!=NULL && hardware.running);
    uint32_t elapsed=diagnostic_count(after,"TICKS=");
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1000);
    run_zero_current_cycles(1);
    motor_backend_run_diagnostics(after,sizeof(after));
    CHECK(strstr(after,"STAGE=RAMP")!=NULL && strstr(after,"DUTY=20.0")!=NULL);
    run_zero_current_cycles(ramp_ticks-1U);
    motor_backend_run_diagnostics(after,sizeof(after));
    CHECK(strstr(after,"HZ=50.000 DUTY=20.0 STAGE=RUN")!=NULL);
    CHECK(diagnostic_count(after,"TICKS=")==elapsed+ramp_ticks && calibration_calls==calibration_before);
    elapsed+=ramp_ticks;
    run_zero_current_cycles(10U*MOTOR_CONTROL_HZ-elapsed-1U);
    CHECK(hardware.running);
    run_zero_current_cycles(1);
    CHECK(!hardware.running && strcmp(hardware.fault,"NONE")==0);
    motor_backend_run_diagnostics(after,sizeof(after));
    CHECK(diagnostic_count(after,"TICKS=")==10U*MOTOR_CONTROL_HZ);
    CHECK(strstr(after,"HZ=50.000 DUTY=20.0 STAGE=RUN")!=NULL && calibration_calls==calibration_before);
    motor_backend_service(++now);
    CHECK(control_model_snapshot()->run==CONTROL_STOPPED);
}

/** @brief 低频在线目标保留软限流、无效样本与STOP/过流保护，停机START仍检查初频下界。 */
static void
test_running_low_target_protection(void)
{
    start_running_target_baseline(20000);
    unsigned calibration_before=calibration_calls;
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1);
    run_zero_current_cycles(999U*MOTOR_CONTROL_HZ/400U);
    char line[384]; motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"HZ=0.050 DUTY=20.0 STAGE=RUN")!=NULL);
    sample(2698,output.low,true,&output);
    CHECK(hardware.running && !output.pulse);
    sample(4095,output.low,false,&output);
    CHECK(hardware.running && output.pulse && strcmp(hardware.fault,"NONE")==0);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && calibration_calls==calibration_before);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"ELECTRICAL SPEED")==0);
    CHECK(calibration_calls==calibration_before && settings_get()->parameter[CONTROL_PARAM_TARGET]==1);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1000);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && calibration_calls==calibration_before+1U);
    run_zero_current_cycles(53U*MOTOR_CONTROL_HZ/20U);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1);
    run_zero_current_cycles(999U*MOTOR_CONTROL_HZ/400U);
    sample(3348,output.low,true,&output);
    CHECK(!hardware.running && strcmp(hardware.fault,"PHASE OVERCURRENT")==0);
    motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"HZ=0.050 DUTY=20.0 STAGE=RUN")!=NULL);
    motor_backend_service(++now);
    CHECK(control_model_snapshot()->run==CONTROL_FAULT);
}

/** @brief 建立独立步进测试场景，普通开环参数故意不满足启动条件。 */
static void
reset_step_backend(void)
{
    reset_run_backend();
    request(CONTROL_REQUEST_SOURCE,CONTROL_PARAM_TARGET,CONTROL_REMOTE);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_POLES,0);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,0);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_ALIGN_MS,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,100);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,10000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RAMP,100);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_DUTY,100);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_CURRENT_LIMIT,1000);
}

/** @brief 提交独立六步请求并等待主循环完成，期望文本可包含启动失败原因。
 * @param test 单步或一圈配置。 @param result 期望的最终请求结果。 */
static void
request_step(const control_step_test_t *test,const char *result)
{
    CHECK(control_model_request_step(test,now)==CONTROL_OK);
    motor_backend_service(++now);
    CHECK(control_model_pending()==NULL);
    CHECK(strcmp(control_model_notice(),result)==0);
}

/** @brief 用独立相对表核对输出与诊断，避免实现和期望共用同一张表。
 * @param sector 期望扇区0..5。 */
static void
check_step_phases(uint8_t sector)
{
    /* A/B/C相索引和未通电相；该表同时检出相交换及低侧绑定错误。 */
    static const uint8_t highs[6]={0,0,1,1,2,2};
    static const uint8_t lows[6]={1,2,2,0,0,1};
    static const uint8_t floats[6]={2,1,0,2,1,0};
    CHECK(output.high==highs[sector] && output.low==lows[sector]);
    char line[384], expected[64];
    motor_backend_step_diagnostics(line,sizeof(line));
    snprintf(expected,sizeof(expected),"SECTOR=%u HIGH=%c LOW=%c FLOAT=%c",
        sector+1U,'A'+highs[sector],'A'+lows[sector],'A'+floats[sector]);
    CHECK(strstr(line,expected)!=NULL);
}

/** @brief 六个单步按最短边界保持，重新校准，完成后保留诊断且不修改保存配置。 */
static void
test_step_hold(void)
{
    reset_step_backend();
    settings_data_t saved=*settings_get();
    char line[384];
    motor_backend_step_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"MODE=NONE STATE=STOPPED")!=NULL);
    const unsigned ticks=20U*(MOTOR_CONTROL_HZ/1000U);
    for(uint8_t sector=0;sector<6;sector++)
    {
        control_step_test_t test={.sector=sector,.cycle=false,.reverse=false,.dwell_ms=20};
        unsigned before=calibration_calls;
        request_step(&test,"OK");
        CHECK(hardware.running && calibration_calls==before+1U);
        CHECK(output.pulse && output.duty_ticks==170000000U/(2U*MOTOR_CONTROL_HZ)/10U);
        check_step_phases(sector);
        motor_backend_step_diagnostics(line,sizeof(line));
        CHECK(strstr(line,"MODE=HOLD STATE=RUNNING DIRECTION=FWD")!=NULL);
        CHECK(strstr(line,"DWELL_MS=20 ELAPSED_MS=0 COMPLETED=0/1 DUTY=10.0 FAULT=NONE")!=NULL);
        for(unsigned tick=1;tick<ticks;tick++)
        {
            CHECK(hardware.running);
            sample(2048,output.low,output.pulse,&output);
            CHECK(hardware.running);
        }
        check_step_phases(sector);
        motor_backend_run_diagnostics(line,sizeof(line));
        char expected[64];
        snprintf(expected,sizeof(expected),"LOW=%c RAW=2048 DELTA=0",'A'+output.low);
        CHECK(strstr(line,expected)!=NULL);
        sample(2048,output.low,output.pulse,&output);
        CHECK(!hardware.running && strcmp(hardware.fault,"NONE")==0);
        motor_backend_service(++now);
        CHECK(control_model_snapshot()->run==CONTROL_STOPPED);
        motor_backend_step_diagnostics(line,sizeof(line));
        CHECK(strstr(line,"MODE=HOLD STATE=STOPPED")!=NULL);
        CHECK(strstr(line,"ELAPSED_MS=20 COMPLETED=1/1 DUTY=10.0 FAULT=NONE")!=NULL);
        CHECK(memcmp(&saved,settings_get(),sizeof(saved))==0);
    }
    /* 最大驻留也必须以自身时长结束，不能沿用故意设为100 ms的普通RUN时限。 */
    control_step_test_t maximum={.sector=0,.cycle=false,.reverse=false,.dwell_ms=1000};
    request_step(&maximum,"OK");
    for(unsigned tick=1;tick<1000U*(MOTOR_CONTROL_HZ/1000U);tick++)
    {
        sample(2048,output.low,output.pulse,&output);
        CHECK(hardware.running);
    }
    sample(2048,output.low,output.pulse,&output);
    CHECK(!hardware.running);
    motor_backend_step_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"DWELL_MS=1000 ELAPSED_MS=1000 COMPLETED=1/1")!=NULL);
    CHECK(memcmp(&saved,settings_get(),sizeof(saved))==0);
}

/** @brief 正反各六步严格按固定驻留边界推进，不进入第七步，不改变普通开环参数。 */
static void
test_step_cycle(void)
{
    reset_step_backend();
    settings_data_t saved=*settings_get();
    const unsigned ticks=20U*(MOTOR_CONTROL_HZ/1000U);
    for(unsigned reverse=0;reverse<2;reverse++)
    {
        control_step_test_t test={.sector=0,.cycle=true,.reverse=reverse!=0,.dwell_ms=20};
        request_step(&test,"OK");
        for(unsigned step=0;step<6;step++)
        {
            uint8_t sector=(uint8_t)(reverse?(6U-step)%6U:step);
            CHECK(hardware.running);
            check_step_phases(sector);
            for(unsigned tick=1;tick<ticks;tick++)
            {
                sample(2048,output.low,output.pulse,&output);
                CHECK(hardware.running);
            }
            check_step_phases(sector);
            sample(2048,output.low,output.pulse,&output);
            CHECK(hardware.running==(step<5));
            char line[384], expected[80];
            motor_backend_step_diagnostics(line,sizeof(line));
            snprintf(expected,sizeof(expected),"ELAPSED_MS=%u COMPLETED=%u/6",(step+1U)*20U,step+1U);
            CHECK(strstr(line,expected)!=NULL);
            CHECK(strstr(line,reverse?"DIRECTION=REV":"DIRECTION=FWD")!=NULL);
        }
        motor_backend_service(++now);
        CHECK(control_model_snapshot()->run==CONTROL_STOPPED);
        char line[384], prior[384];
        motor_backend_run_diagnostics(prior,sizeof(prior));
        char expected[96];
        snprintf(expected,sizeof(expected),"TICKS=%u VALID=%u INVALID=0",6U*ticks,6U*ticks);
        CHECK(strstr(prior,expected)!=NULL);
        snprintf(expected,sizeof(expected),"PHASE_SAMPLES=%u,%u,%u",2U*ticks,2U*ticks,2U*ticks);
        CHECK(strstr(prior,expected)!=NULL);
        motor_backend_service(now+=20);
        motor_backend_run_diagnostics(line,sizeof(line));
        CHECK(strcmp(line,prior)==0);
        CHECK(memcmp(&saved,settings_get(),sizeof(saved))==0);
    }
}

/** @brief STOP覆盖未执行测试，运行TARGET拒绝，人工停机和下一次普通START恢复原路径。 */
static void
test_step_stop_and_normal_start(void)
{
    reset_step_backend();
    control_step_test_t test={.sector=4,.cycle=false,.reverse=false,.dwell_ms=20};
    unsigned before=calibration_calls;
    CHECK(control_model_request_step(&test,now)==CONTROL_OK);
    CHECK(control_model_request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0,now)==CONTROL_OK);
    motor_backend_service(++now);
    CHECK(!hardware.running && calibration_calls==before);
    CHECK(control_model_pending()==NULL && strcmp(control_model_notice(),"OK")==0);
    request_step(&test,"OK");
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,100);
    CHECK(strcmp(control_model_notice(),"STEP TEST ACTIVE")==0);
    CHECK(settings_get()->parameter[CONTROL_PARAM_TARGET]==0);
    CHECK(hardware.running);
    sample(2048,output.low,output.pulse,&output);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running);
    char line[384];
    motor_backend_step_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"MODE=HOLD STATE=STOPPED")!=NULL);
    CHECK(strstr(line,"COMPLETED=0/1")!=NULL);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_POLES,3);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,100);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_ALIGN_MS,10);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,1200);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && output.high==0 && output.low==1);
    motor_backend_step_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"MODE=NORMAL STATE=RUNNING")!=NULL);
    const unsigned ticks=6U*MOTOR_CONTROL_HZ/5U;
    for(unsigned tick=0;tick<MOTOR_CONTROL_HZ/50U;tick++) { sample(2048,output.low,output.pulse,&output); }
    CHECK(hardware.running);
    for(unsigned tick=MOTOR_CONTROL_HZ/50U;tick<ticks;tick++) { sample(2048,output.low,output.pulse,&output); }
    CHECK(!hardware.running && strcmp(hardware.fault,"NONE")==0);
}

/** @brief 独立测试继续使用有效低侧限流、无效样本处理和绝对过流保护，诊断安全截断。 */
static void
test_step_protection_and_diagnostics(void)
{
    reset_step_backend();
    control_step_test_t test={.sector=4,.cycle=false,.reverse=false,.dwell_ms=1000};
    request_step(&test,"OK");
    sample(2698,output.low,true,&output);
    CHECK(hardware.running && !output.pulse && output.high==2 && output.low==0);
    sample(4095,output.low,false,&output);
    CHECK(hardware.running && output.pulse);
    sample(1398,output.low,true,&output);
    CHECK(hardware.running && !output.pulse);
    sample(3348,output.low,true,&output);
    CHECK(!hardware.running && strcmp(hardware.fault,"PHASE OVERCURRENT")==0);
    motor_backend_service(++now);
    char line[384];
    motor_backend_step_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"MODE=HOLD STATE=FAULT")!=NULL);
    CHECK(strstr(line,"DWELL_MS=1000")!=NULL);
    CHECK(strstr(line,"COMPLETED=0/1 DUTY=10.0 FAULT=PHASE OVERCURRENT")!=NULL);
    CHECK(strlen(line)<sizeof(line)-3);
    motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"TICKS=4 VALID=3 INVALID=1 LIMIT_HITS=3")!=NULL);
    CHECK(strstr(line,"PHASE_SAMPLES=3,0,0")!=NULL);
    /* 限定容量后的写入必须零结尾，且不能越过调用者给定的缓冲区。 */
    char short_line[9];
    memset(short_line,'X',sizeof(short_line));
    motor_backend_step_diagnostics(short_line,8);
    CHECK(short_line[7]=='\0' && short_line[8]=='X');
    request(CONTROL_REQUEST_CLEAR_FAULT,CONTROL_PARAM_TARGET,0);
    calibrated_bus_mv=7000;
    request_step(&test,"BUS VOLTAGE");
    CHECK(!hardware.running);
    calibrated_bus_mv=12000;
    request(CONTROL_REQUEST_CLEAR_FAULT,CONTROL_PARAM_TARGET,0);
    calibration_ok=false;
    request_step(&test,"DRV/ADC CAL");
    CHECK(!hardware.running);
    calibration_ok=true;
    request(CONTROL_REQUEST_CLEAR_FAULT,CONTROL_PARAM_TARGET,0);
    start_failure_reason="TIM PHASE";
    request_step(&test,"TIM PHASE");
    CHECK(!hardware.running && strcmp(hardware.fault,"TIM PHASE")==0);
    start_failure_reason=NULL; calibrated_bus_mv=0;
}

/** @brief 单步低占空比首周期仍遵循整脉冲密度预算，无效大码值不能触发过流。 */
static void
test_step_density(void)
{
    reset_step_backend();
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_DUTY,10);
    control_step_test_t test={.sector=5,.cycle=false,.reverse=false,.dwell_ms=20};
    request_step(&test,"OK");
    CHECK(!output.pulse && output.duty_ticks==391);
    unsigned pulses=0;
    const unsigned ticks=20U*(MOTOR_CONTROL_HZ/1000U);
    const unsigned requested=board_motor_duty_ticks(10);
    for(unsigned tick=0;tick<ticks;tick++)
    {
        CHECK(hardware.running && output.high==2 && output.low==1);
        if(output.pulse) { ++pulses; }
        CHECK(pulses*391U<=(tick+1U)*requested);
        sample(output.pulse?2048:4095,output.low,output.pulse,&output);
    }
    CHECK(!hardware.running && strcmp(hardware.fault,"NONE")==0);
    CHECK(pulses==ticks*requested/391U);
    char line[384], expected[128];
    motor_backend_run_diagnostics(line,sizeof(line));
    snprintf(expected,sizeof(expected),"TICKS=%u VALID=%u INVALID=%u LIMIT_HITS=0 DENSITY_SKIP=%u",
        ticks,pulses,ticks-pulses,ticks-pulses);
    CHECK(strstr(line,expected)!=NULL);
}

/** @brief 保留旧混合场景中的普通启动预算、低目标拒绝及 STOP 覆盖检查。 */
static void
test_normal_start_budget_and_stop(void)
{
    reset_run_backend();
    request(CONTROL_REQUEST_SOURCE,CONTROL_PARAM_TARGET,CONTROL_REMOTE);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SPEED_LIMIT,3000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,100);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_ALIGN_MS,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_CURRENT_LIMIT,1000);
    unsigned calibration_before=calibration_calls;
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"RUN/ALIGN TIME")==0);
    CHECK(calibration_calls==calibration_before);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_ALIGN_MS,10);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"RUN/START TIME")==0);
    CHECK(calibration_calls==calibration_before);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,5000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"ELECTRICAL SPEED")==0);
    CHECK(calibration_calls==calibration_before);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_ALIGN_MS,200);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,5000);
    CHECK(control_model_request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0,now)==CONTROL_OK);
    CHECK(control_model_request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0,now)==CONTROL_OK);
    motor_backend_service(++now);
    CHECK(!hardware.running && calibration_calls==calibration_before && control_model_pending()==NULL);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && !output.pulse && output.high==0 && output.low==1);
    char line[384]; motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"HZ=5.000 DUTY=0.0 STAGE=ALIGN_RAMP")!=NULL);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running);
}

/** @brief 建立保留200 ms定位、以50 Hz直接牵引到同频目标的真实后端场景。 */
static void
reset_high_start_backend(void)
{
    reset_run_backend();
    request(CONTROL_REQUEST_SOURCE,CONTROL_PARAM_TARGET,CONTROL_REMOTE);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SPEED_LIMIT,3000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,50000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RAMP,20000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_ALIGN_MS,200);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,20000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_CURRENT_LIMIT,1500);
}

/** @brief 删除的请求编号不能排队、校准或调用功率启动，普通 START 的编号保持不变。 */
static void
test_removed_request_no_output(void)
{
    reset_high_start_backend();
    settings_data_t saved=*settings_get();
    board_motor_output_t before=output;
    unsigned calibration_before=calibration_calls;
    CHECK(CONTROL_REQUEST_START==0 && CONTROL_REQUEST_STEP_TEST==10);
    CHECK(control_model_request((control_request_kind_t)(CONTROL_REQUEST_STEP_TEST+1U),CONTROL_PARAM_TARGET,0,now)==CONTROL_INVALID);
    CHECK(control_model_pending()==NULL);
    motor_backend_service(++now);
    CHECK(!hardware.running && calibration_calls==calibration_before);
    CHECK(memcmp(&output,&before,sizeof(output))==0 && memcmp(&saved,settings_get(),sizeof(saved))==0);
    char line[384]; motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"TICKS=0 VALID=0 INVALID=0")!=NULL && strstr(line,"STAGE=NONE")!=NULL);
}

/** @brief 50 Hz端点保留定位和六次牵引，随后同频连续换相，到20秒准确停机。 */
static void
test_high_start_frequency_normal(void)
{
    reset_high_start_backend();
    CHECK(control_model_request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,50001,now)==CONTROL_RANGE);
    CHECK(control_model_pending()==NULL && settings_get()->parameter[CONTROL_PARAM_START_HZ]==50000);
    settings_data_t saved=*settings_get();
    unsigned calibration_before=calibration_calls;
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && calibration_calls==calibration_before+1U && !output.pulse);
    char line[384]; motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"HZ=50.000 DUTY=0.0 STAGE=ALIGN_RAMP")!=NULL);
    CHECK(run_zero_current_cycles(MOTOR_CONTROL_HZ/10U)==0U);
    motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"HZ=50.000 DUTY=20.0 STAGE=ALIGN_HOLD")!=NULL);
    CHECK(output.pulse && output.high==0 && output.low==1 && output.duty_ticks==board_motor_duty_ticks(200));
    CHECK(run_zero_current_cycles(MOTOR_CONTROL_HZ/10U)==0U);
    const unsigned align_ticks=MOTOR_CONTROL_HZ/5U, total_ticks=20U*MOTOR_CONTROL_HZ;
    unsigned current_tick=align_ticks;
    motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(diagnostic_count(line,"TICKS=")==align_ticks && strstr(line,"STAGE=ALIGN_HOLD")!=NULL);
    /* 用50 Hz对应300次每秒的物理换相时刻核对两个电周，覆盖牵引结束后的相位连续性。 */
    for(unsigned transition=1;transition<=12U;transition++)
    {
        unsigned at=align_ticks+(transition*MOTOR_CONTROL_HZ+299U)/300U;
        CHECK(run_zero_current_cycles(at-current_tick-1U)==0U);
        CHECK(run_zero_current_cycles(1U)==1U);
        current_tick=at;
        motor_backend_run_diagnostics(line,sizeof(line));
        CHECK(diagnostic_count(line,"TICKS=")==at && diagnostic_count(line,"SECTOR=")==transition%6U+1U);
        CHECK(strstr(line,"HZ=50.000 DUTY=20.0")!=NULL);
        CHECK(strstr(line,transition<=6U?"STAGE=PULLIN":"STAGE=RUN")!=NULL);
        if(transition==6U)
        {
            CHECK(current_tick==align_ticks+MOTOR_CONTROL_HZ/50U);
            CHECK(run_zero_current_cycles(1U)==0U); ++current_tick;
            motor_backend_run_diagnostics(line,sizeof(line));
            CHECK(strstr(line,"SECTOR=1 HZ=50.000 DUTY=20.0 STAGE=RUN")!=NULL);
        }
    }
    /* 首200 ms不换相；随后19.8秒截止前应完成5939次相邻换相。 */
    CHECK(run_zero_current_cycles(total_ticks-current_tick-1U)==5939U-12U);
    CHECK(hardware.running && output.pulse && strcmp(hardware.fault,"NONE")==0);
    CHECK(run_zero_current_cycles(1U)==0U && !hardware.running);
    motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(diagnostic_count(line,"TICKS=")==total_ticks);
    CHECK(diagnostic_count(line,"VALID=")+diagnostic_count(line,"INVALID=")==total_ticks);
    CHECK(diagnostic_count(line,"INVALID=")==diagnostic_count(line,"DENSITY_SKIP="));
    CHECK(diagnostic_count(line,"LIMIT_HITS=")==0U && strstr(line,"HZ=50.000 DUTY=20.0 STAGE=RUN")!=NULL);
    motor_backend_service(now+=20);
    CHECK(control_model_snapshot()->run==CONTROL_STOPPED && memcmp(&saved,settings_get(),sizeof(saved))==0);
}

/** @brief 扩初频范围后仍拒绝低目标与不足牵引预算，不改变保存值。 */
static void
test_high_start_frequency_guards(void)
{
    reset_high_start_backend();
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,999);
    settings_data_t saved=*settings_get();
    unsigned calibration_before=calibration_calls;
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && calibration_calls==calibration_before && strcmp(control_model_notice(),"ELECTRICAL SPEED")==0);
    CHECK(memcmp(&saved,settings_get(),sizeof(saved))==0);
    char line[384];
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,220);
    calibration_before=calibration_calls;
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && calibration_calls==calibration_before && strcmp(control_model_notice(),"RUN/START TIME")==0);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,221);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && calibration_calls==calibration_before+1U && !output.pulse);
    run_zero_current_cycles(221U*(MOTOR_CONTROL_HZ/1000U));
    motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(!hardware.running && diagnostic_count(line,"TICKS=")==221U*(MOTOR_CONTROL_HZ/1000U));
    CHECK(strstr(line,"HZ=50.000 DUTY=20.0 STAGE=RUN")!=NULL);
}

/** @brief 30秒RUN随控制频率累计采样，六位计数和最高电频率响应完整保留CRLF空间。 */
static void
test_run_response_capacity(void)
{
    reset_run_backend();
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_POLES,9);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SPEED_LIMIT,5400);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,5400);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,10000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RAMP,100000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_CURRENT_LIMIT,1500);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_DUTY,250);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_RUN_MS,30000);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    const unsigned ticks=30U*MOTOR_CONTROL_HZ;
    for(unsigned cycle=0;cycle<ticks;cycle++)
    {
        sample(cycle==ticks-1U?0:2948,(uint8_t)(cycle%3U),true,&output);
    }
    CHECK(!hardware.running && strcmp(hardware.fault,"PHASE OVERCURRENT")==0);
    char line[384]; motor_backend_run_diagnostics(line,sizeof(line));
    CHECK(strlen(line)<sizeof(line)-3);
    char expected[160];
    snprintf(expected,sizeof(expected),"TICKS=%u VALID=%u INVALID=0 LIMIT_HITS=1",ticks,ticks);
    CHECK(strstr(line,expected)!=NULL);
    CHECK(diagnostic_count(line,"DENSITY_SKIP=")>0 && strstr(line,"STAGE=RUN")!=NULL);
    snprintf(expected,sizeof(expected),"PHASE_SAMPLES=%u,%u,%u PEAK_MA=1450,1450,3300 LOW=C RAW=0 DELTA=-2048",ticks/3U,ticks/3U,ticks/3U);
    CHECK(strstr(line,expected)!=NULL);
    CHECK(strstr(line,"HZ=810.000 DUTY=25.0")!=NULL);
    CHECK(strcmp(strrchr(line,' ')," STAGE=RUN")==0);
}

/** @brief 诊断字段的最大长度仍须保留末尾故障，不能截断实际串口响应。 */
static void test_diagnostics(void)
{
    hardware.fast_count=hardware.regular_count=hardware.regular_skipped=hardware.irq_max_cycles=UINT32_MAX;
    hardware.update_count=hardware.peak_count=hardware.bus_age_ms=UINT32_MAX;
    hardware.bus_mv=hardware.vdda_mv=UINT16_MAX;
    for(unsigned i=0;i<3;i++) { hardware.zero[i]=UINT16_MAX; }
    for(unsigned i=0;i<5;i++) { hardware.driver_registers[i]=0x7FF; }
    hardware.fault="PHASE OVERCURRENT";
    char line[384]; motor_backend_diagnostics(line,sizeof(line));
    CHECK(strlen(line)<sizeof(line)-3);
    CHECK(strstr(line,"UP=4294967295 PEAK=4294967295 BUS_AGE_MS=4294967295")!=NULL);
    char expected[64];
    snprintf(expected,sizeof(expected),"PWM_HZ=%u CTRL_HZ=%u",MOTOR_CONTROL_HZ,MOTOR_CONTROL_HZ);
    CHECK(strstr(line,expected)!=NULL);
    CHECK(strstr(line,"FAULT=PHASE OVERCURRENT")!=NULL);
}
/** @brief 注入中点过阈波形及换相首周期无效窗口，模拟实际扇区先提交后采样。
 * @param ticks 从一次新启动开始模拟的控制周期数。 */
static void
run_sensorless_waveform(uint32_t ticks)
{
    uint8_t committed_sector=0;
    uint32_t committed_us=0;
    const uint32_t period=1000000U/MOTOR_CONTROL_HZ;
    /* 合成六扇区中点的上/下阈值事件，时钟模拟实际输出先提交后采样。 */
    for(uint32_t tick=0;tick<ticks;tick++)
    {
        uint8_t sector=0;
        for(uint8_t candidate=0;candidate<6;candidate++)
        {
            uint8_t high,low;
            six_step_phases(candidate,&high,&low);
            if(high==output.high && low==output.low) { sector=candidate; break; }
        }
        uint32_t stamp=tick*period;
        bool changed=sector!=committed_sector;
        if(changed) { committed_sector=sector; committed_us=stamp; }
        bool rising=(sector&1U)!=0;
        uint32_t elapsed=stamp+period/2U-committed_us;
        bool after=elapsed>=1667U;
        /* 复现下降沿先经过290码、后到扇区中点的形状：旧300码会过早触发，
         * 与上升沿形成交替长短间隔，不能取得连续资格；不把波形绑到当前门限。 */
        uint16_t falling_raw=after?250U:(elapsed>=733U?290U:350U);
        observation=(board_motor_observation_t){
            .raw=rising ? (after?80U:10U) : falling_raw, .floating=(uint8_t)(3U-output.high-output.low),
            .sector=sector, .valid=output.pulse && !changed, .sequence=tick+1U,
            .sample_us=stamp+period/2U, .committed_us=committed_us,
            .next_commit_us=stamp+period, .now_us=stamp+3U*period/4U
        };
        sample(2048,output.low,output.pulse,&output);
        CHECK(hardware.running);
    }
}

/** @brief 观察与默认闭环模式复用启动，零增益使用默认值但不改写保存参数，保护优先。 */
static void
test_sensorless_backend(void)
{
    reset_run_backend();
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SPEED_LIMIT,3000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,50000);
    CHECK(control_model_select(CONTROL_SENSORLESS_6STEP,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(++now);
    CHECK(control_model_snapshot()->ready);
    CHECK(!(control_model_snapshot()->capabilities&CONTROL_CAP_STEP_TEST));
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && bemf_enabled);
    run_sensorless_waveform(MOTOR_CONTROL_HZ/5U);
    CHECK(speed_calls==MOTOR_SPEED_LOOP_HZ/5U);
    /* 200ms没有任何主循环服务，控制仍取得事件资格和真实COM确认。 */
    motor_backend_service(now+=200);
    char line[384];
    motor_backend_bemf_diagnostics(line,sizeof(line));
#if MOTOR_SENSORLESS_CLOSED_LOOP
    CHECK(strstr(line,"MODE=CLOSED_LOOP STATE=CLOSED")!=NULL);
    CHECK(strstr(line,"QUALIFIED=1 LOCKED=1")!=NULL);
#else
    CHECK(strstr(line,"MODE=OBSERVE_ONLY")!=NULL);
    CHECK(strstr(line,"QUALIFIED=1 LOCKED=0")!=NULL);
#endif
    CHECK(diagnostic_count(line,"EVENTS=")>=12U);
    CHECK(settings_get()->parameter[CONTROL_PARAM_SPEED_KP]==0 && settings_get()->parameter[CONTROL_PARAM_SPEED_KI]==0);
    CHECK(control_model_snapshot()->values[CONTROL_SPEED_VALUE].valid);
    CHECK(control_model_snapshot()->values[CONTROL_SPEED_VALUE].value>=950 &&
        control_model_snapshot()->values[CONTROL_SPEED_VALUE].value<=1050);
    CHECK(strlen(line)<sizeof(line)-3U && strstr(line,"NEXT_US=")!=NULL);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1);
    CHECK(strcmp(control_model_notice(),"BEMF SPEED RANGE")==0 && hardware.running);
    CHECK(settings_get()->parameter[CONTROL_PARAM_TARGET]==1000);
    /* 正常停机和板层故障都必须屏蔽冻结历史的当前锁定、估速与样本有效位。 */
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    motor_backend_bemf_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"STATE=STOPPED QUALIFIED=0 LOCKED=0 EST_RPM=NA")!=NULL);
    CHECK(strstr(line," VALID=0 ")!=NULL && diagnostic_count(line,"EVENTS=")>=12U);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    run_sensorless_waveform(MOTOR_CONTROL_HZ/5U);
    board_motor_stop("BUS STALE");
    motor_backend_service(++now);
    motor_backend_bemf_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"STATE=FAULT QUALIFIED=0 LOCKED=0 EST_RPM=NA")!=NULL);
    CHECK(strstr(line," VALID=0 ")!=NULL && diagnostic_count(line,"EVENTS=")>=12U);
    request(CONTROL_REQUEST_CLEAR_FAULT,CONTROL_PARAM_TARGET,0);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    run_sensorless_waveform(MOTOR_CONTROL_HZ/5U);
    /* 电流过流始终先于观察器处理，故障不能因无感状态被吞掉。 */
    sample(2048+1250,output.low,true,&output);
    CHECK(!hardware.running && strcmp(hardware.fault,"PHASE OVERCURRENT")==0);
    motor_backend_service(++now);
    motor_backend_bemf_diagnostics(line,sizeof(line));
    CHECK(strstr(line,"STATE=FAULT QUALIFIED=0 LOCKED=0 EST_RPM=NA")!=NULL);
    CHECK(strstr(line," VALID=0 ")!=NULL && diagnostic_count(line,"EVENTS=")>=12U);
    CHECK(!control_model_snapshot()->values[CONTROL_SPEED_VALUE].valid);
    request(CONTROL_REQUEST_CLEAR_FAULT,CONTROL_PARAM_TARGET,0);
    CHECK(control_model_select(CONTROL_OPEN_6STEP,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(++now);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && !bemf_enabled);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
}

/** @brief 无感低速目标和不足采样脉宽不得产生功率输出。 */
static void
test_sensorless_guards(void)
{
    reset_run_backend();
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SPEED_LIMIT,3000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,900);
    CHECK(control_model_select(CONTROL_SENSORLESS_6STEP,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(++now);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"BEMF SPEED RANGE")==0);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_HANDOFF,2000);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"HANDOFF/TARGET")==0);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_HANDOFF,0);
#if MOTOR_SENSORLESS_CLOSED_LOOP
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SPEED_KP,50);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SPEED_KI,100);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_COMM_TIMING,300);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"BEMF TIMING RANGE")==0);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_COMM_TIMING,0);
#endif
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_DUTY,1);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(!hardware.running && strcmp(control_model_notice(),"BEMF DUTY RANGE")==0);
}

/** @brief ADC/BKIN在PI计算期间关断时，TIM不得提交已过期的占空比。 */
static void test_pi_preempted_by_stop(void)
{
#if MOTOR_SENSORLESS_CLOSED_LOOP
    reset_run_backend();
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_SPEED_LIMIT,3000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1000);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_START_HZ,50000);
    CHECK(control_model_select(CONTROL_SENSORLESS_6STEP,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_service(++now);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    run_sensorless_waveform(MOTOR_CONTROL_HZ/5U);
    char line[384]; motor_backend_timing_diagnostics(line,sizeof(line));
    CHECK(diagnostic_count(line,"PI_TICKS=")>0U);
    uint32_t updates=diagnostic_count(line,"PI_TICKS=");
    uint16_t before=control_duty;
    /* 第一次短锁读取，第二次短锁模拟计算期间发生的关断。 */
    stop_pi_locks=2U;
    speed_callback();
    CHECK(!hardware.running && strcmp(hardware.fault,"DRV NFAULT")==0);
    CHECK(control_duty==before);
    motor_backend_timing_diagnostics(line,sizeof(line));
    CHECK(diagnostic_count(line,"PI_TICKS=")==updates);
#endif
}

/** @brief 主机回归入口。 @return 全部通过返回0。 */
int main(void)
{
    test_driver(); test_persistence(); test_six_step(); test_startup_stages();
    test_fractional_pullin_and_target(); test_startup_alignment_edges(); test_boot_calibration(); test_backend();
    test_run_statistics(); test_first_density_pulse(); test_one_amp_limit();
    test_startup_snapshot(); test_alignment_pulse_budget(); test_startup_time_budget();
    test_startup_stage_protection(); test_startup_live_target();
    test_running_near_stop_phase(); test_running_near_stop_target(); test_running_low_target_protection();
    test_step_hold(); test_step_cycle(); test_step_stop_and_normal_start(); test_step_protection_and_diagnostics();
    test_step_density();
    test_normal_start_budget_and_stop();
    test_removed_request_no_output(); test_high_start_frequency_normal(); test_high_start_frequency_guards();
    test_run_response_capacity(); test_diagnostics();
    test_sensorless_backend(); test_sensorless_guards();
    test_pi_preempted_by_stop();
    printf("motor/settings/driver: %u checks passed\n",checks); return 0;
}
