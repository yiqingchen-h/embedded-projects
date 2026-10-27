/* 将实际应用编译到此测试翻译单元，直接覆盖命令和状态转移。 */
#include "../../project-test/app/selftest.c"
#include "../../project-test/app/load_test.c"
#include <stdlib.h>

GPIO_TypeDef test_gpio_b;
TIM_TypeDef test_tim1;
RCC_TypeDef test_rcc;
/* 主机注入的输入、ADC及驱动影子寄存器。 */
static board_inputs_t t_inputs;
static board_adc_t t_adc;
static uint16_t t_registers[7];
static uint8_t t_user_id;
static bool t_stop, t_enabled, t_adc_fail, t_drv_fail, t_enc_fail;
/* 脉冲过程中注入故障，监测故障后是否继续发脉冲。 */
static bool t_pulse_fault, t_step_fault;
static unsigned t_power_pulses, t_logic_pulses, t_step_pulses, t_unsafe;
/* 模拟串口输入和时基。 */
static const char *t_rx;
static uint32_t t_ticks;
/* 长脉冲模型及B相电流变化，正负符号都必须保留。 */
static bool t_load_active, t_load_stuck;
static uint32_t t_load_started;
static int32_t t_load_delta;
static unsigned t_load_starts;
/* 捕获报告，验证单位和有效性。 */
static char t_log[32768];
/* 检查统计。 */
static unsigned t_checks, t_failures;

/** @brief 记录断言。 @param ok 条件。 @param label 标题。 */
static void
expect(bool ok,const char *label)
{
    ++t_checks;
    if (!ok) { ++t_failures; printf("FAIL %s\n",label); }
}
/** @brief 模拟初始化HAL后外设输入基准。 */
static void
fixture(void)
{
    memset((void *)&g_selftest,0,sizeof(g_selftest));
    memset(&t_inputs,0,sizeof(t_inputs));
    memset(&t_adc,0,sizeof(t_adc));
    memset(t_registers,0,sizeof(t_registers));
    load_test_init();
    t_load_active=t_load_stuck=false; t_load_starts=0U; t_load_delta=-24;
    t_log[0]='\0';
    t_adc.vref_mv=3300U;
    t_adc.raw[0]=1145U;
    t_adc.raw[4]=t_adc.raw[5]=t_adc.raw[6]=2048U;
    t_adc.raw[7]=1500U;
    t_ticks=0U; t_user_id=0x7BU;
    t_stop=false; t_enabled=false; t_adc_fail=false; t_drv_fail=false; t_enc_fail=false;
    t_pulse_fault=false; t_step_fault=false;
    t_power_pulses=0U; t_logic_pulses=0U; t_step_pulses=0U; t_unsafe=0U;
    t_rx="";
    test_gpio_b.IDR=GPIO_PIN_2|GPIO_PIN_12;
    test_tim1.BDTR=0U;
    g_initialized=(board_test_init_t){true,true,true};
    g_command_size=0U; g_command_discard=false; g_last_rx_errors=0U;
    g_encoder_probed=false;
    g_page=0U; g_power_choice=0U; g_last_up=0U; g_last_down=0U; g_last_presses=0U; g_last_held=0U;
    g_long_consumed=false; g_pattern_active=false;
    g_sample_time=g_draw_time=g_log_time=g_probe_time=0U;
}

/** @brief 模拟毫秒时钟。 @return ms。 */
uint32_t HAL_GetTick(void) { return t_ticks; }
/** @brief 模拟时钟频率。 @return Hz。 */
uint32_t HAL_RCC_GetSysClockFreq(void) { return 170000000U; }
/** @brief 模拟芯片识别。 @return ID。 */
uint32_t HAL_GetDEVID(void) { return 0x469U; }
/** @brief 模拟延时。 @param ms 时间。 */
void HAL_Delay(uint32_t ms) { t_ticks+=ms; }
/** @brief 模拟引脚采样。 @param port 端口。 @param pin 引脚。 @return 电平。 */
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port,uint16_t pin) { return (port->IDR&pin)?GPIO_PIN_SET:GPIO_PIN_RESET; }
/** @brief 模拟ADC。 @param sample 返回值。 @return 状态。 */
bool board_adc_read(board_adc_t *sample)
{
    *sample=t_adc;
    if (t_load_active && t_enabled)
    {
        sample->raw[1]=sample->raw[0]; sample->raw[2]=0U;
        sample->raw[5]=(uint16_t)((int32_t)t_adc.raw[5]+t_load_delta);
    }
    return !t_adc_fail;
}
/** @brief 模拟关断。 */
void board_trip(void) { t_stop=true; t_enabled=false; test_gpio_b.IDR&=~GPIO_PIN_9; }
/** @brief 模拟正常禁用，不清除锁存。 */
void board_driver_disable(void) { t_enabled=false; t_load_active=false; test_gpio_b.IDR&=~GPIO_PIN_9; }
/** @brief 模拟固定AH+BL启动。 @return 满足驱动条件。 */
bool board_load_start(void)
{
    if (!t_enabled || t_stop || (t_registers[2]&4U)!=0U) { ++t_unsafe; return false; }
    ++t_load_starts; t_load_active=true; t_load_started=t_ticks; return true;
}
/** @brief 模拟硬件截止，不由应用主动结束脉冲。 @return 仍运行。 */
bool board_load_running(void)
{
    if (t_load_active && !t_load_stuck && (uint32_t)(t_ticks-t_load_started)>=1001U) { board_driver_disable(); }
    return t_load_active;
}
/** @brief 模拟停机锁存。 @return 状态。 */
bool board_tripped(void) { return t_stop; }
/** @brief 模拟显式复位。 */
void board_clear_trip(void) { t_stop=false; }
/** @brief 模拟唤醒并重置寄存器。 */
void board_driver_wake(void)
{
    if (!t_stop) { t_enabled=true; test_gpio_b.IDR|=GPIO_PIN_9; memset(t_registers,0,sizeof(t_registers)); }
}
/** @brief 模拟故障监控开启。 @return 状态。 */
bool board_driver_arm(void) { if ((test_gpio_b.IDR&GPIO_PIN_12)==0U) { board_trip(); } return !t_stop; }
/** @brief 模拟电流前置条件。 @param zero 零点。 @param mv 电压。 @return 合理范围为true。 */
bool board_current_guard(const uint16_t zero[3],uint32_t mv)
{
    return zero[0]>=1433U && zero[1]>=1433U && zero[2]>=1433U && mv>=2700U && mv<=3600U;
}
/** @brief 模拟DRV总线。 @param tx 请求。 @param rx 应答。 @return 状态。 */
bool board_drv_transfer(uint16_t tx,uint16_t *rx)
{
    if (t_drv_fail || !t_enabled) { return false; }
    unsigned addr=(tx>>11)&15U;
    if (addr>6U) { return false; }
    *rx=t_registers[addr];
    if ((tx&0x8000U)==0U) { t_registers[addr]=tx&0x7FFU; }
    return true;
}
/** @brief 模拟MT总线。 @param tx 请求。 @param rx 响应。 @param count 字节数。 @return 状态。 */
bool board_enc_transfer(const uint8_t *tx,uint8_t *rx,uint16_t count)
{
    if (t_enc_fail) { memset(rx,0,count); return true; }
    memset(rx,0,count);
    if (count==3U)
    {
        if (tx[0]==0x60U) { t_user_id=tx[2]; }
        else { rx[2]=t_user_id; }
    }
    else { rx[4]=2U; rx[5]=0x0EU; }
    return true;
}
/** @brief 模拟脉冲并核查互锁。 @param gate 引脚。 @param phase_raw 相电压。 @param enabled 驱动应开启。 @return 状态。 */
bool board_gate_pulse(uint8_t gate,uint16_t *phase_raw,bool enabled)
{
    if (enabled)
    {
        ++t_power_pulses;
        if (!t_enabled || t_stop || (t_registers[2]&4U)!=0U) { ++t_unsafe; return false; }
        if (t_pulse_fault) { board_trip(); return false; }
        *phase_raw=(gate&1U)?0U:t_adc.raw[0];
    }
    else
    {
        ++t_logic_pulses;
        if (t_enabled) { ++t_unsafe; return false; }
        *phase_raw=0U;
    }
    return true;
}
/** @brief 模拟异相上下管六步脉冲及驱动器高阻门禁。 @param step 六步状态。 @param sample 三相采样。 @return 模拟结果。 */
bool board_six_step_pulse(uint8_t step,board_six_step_sample_t *sample)
{
    if (step != t_step_pulses % 6U || !t_enabled || t_stop || (t_registers[2] & 4U) != 0U)
    {
        ++t_unsafe; return false;
    }
    ++t_step_pulses;
    if (t_step_fault && t_step_pulses == 4U) { board_trip(); return false; }
    sample->high_raw=t_adc.raw[0]; sample->low_raw=0U; sample->floating_raw=0U;
    return true;
}
/** @brief 模拟UART发送。 @param text 文本。 */
void board_uart_write(const char *text)
{
    size_t used=strlen(t_log);
    (void)snprintf(t_log+used,sizeof(t_log)-used,"%s",text);
}
/** @brief 模拟TXE发送，无阻塞也不改变时基。 @param text 文本。 @return 已接受。 */
bool board_uart_try_write(const char *text) { board_uart_write(text); return true; }
/** @brief 模拟UART接收。 @param value 字节。 @return 有数据为true。 */
bool board_uart_get(char *value) { if (*t_rx=='\0') { return false; } *value=*t_rx++; return true; }
/** @brief 模拟输入快照。 @param inputs 输出。 */
void board_inputs_read(board_inputs_t *inputs) { *inputs=t_inputs; }
/** @brief 模拟OLED未接。 @param a 地址。 @param c 控制。 @param d 数据。 @param n 长度。 @return false。 */
bool board_i2c_write(uint8_t a,uint8_t c,const uint8_t *d,uint16_t n) { (void)a;(void)c;(void)d;(void)n;return false; }

/** @brief 每毫秒推进实际应用而非复制状态机。 @param ms 时间。 */
static void
advance(uint32_t ms)
{
    for (uint32_t i=0U;i<ms;++i) { ++t_ticks; selftest_poll(); }
}

/** @brief 通过正式命令进入已校准负载状态。 */
static void
prepare_load(void)
{
    fixture(); run_checks(); execute("LOAD CHECK"); advance(600U);
    expect(g_load.state==LOAD_READY && t_registers[6]==0x2C0U && t_load_starts==0U,
        "LOAD CHECK calibrates gain40 without bridge output");
}

/** @brief 检查负载模式的正常流程、通信、保护和冷却。 */
static void
test_load(void)
{
    prepare_load(); execute("LOAD RUN"); advance(300U);
    expect(t_load_active && t_load_starts==1U && t_unsafe==0U,"single AH BL start after fresh baseline");
    execute("RETEST"); execute("POWER NOLOAD"); execute("LOAD CHECK"); execute("LOAD RUN");
    expect(t_load_starts==1U && t_load_active && t_registers[6]==0x2C0U,"busy requests cannot reenter or reset gain");
    execute("LOAD STATUS");
    expect(strstr(t_log,"LOAD STATE=OUTPUT")!=NULL,"active status query has nonblocking phase response");
    t_log[0]='\0'; execute("STATUS");
    expect(strstr(t_log,"LOAD STATE=OUTPUT")!=NULL,"legacy STATUS cannot expose stale gain20 data during load");
    t_inputs.held_ms=3000U; t_inputs.presses=1U; g_selftest.display=true; g_page=1U;
    advance(1300U);
    expect(g_load.state==LOAD_DONE && !t_enabled && t_load_starts==1U,"hardware end then normal-route zero and disabled");
    expect(g_load.before.count==256U && g_load.after.count==256U && g_load.during.count>=900U,"all three measurement windows");
    expect(stats_mean(&g_load.during)-stats_mean(&g_load.before)==(uint32_t)-24000,
        "raw fractional mean preserves negative B signal");
    expect(strstr(t_log,"B_MA=-483")!=NULL && strstr(t_log,"VALID=1")!=NULL,
        "gain40 current conversion and validity reported");
    expect(t_step_pulses==0U && t_power_pulses==0U,"load mode isolates local power menu");
    execute("LOAD RUN");
    expect(t_load_starts==1U,"completed run requires another check");
    uint32_t stopped=g_load.stopped_at;
    execute("RETEST");
    expect(t_registers[6]==0x280U && !load_test_mode(),"RETEST restores gain20 ordinary selftest");
    execute("LOAD CHECK"); advance(600U); execute("LOAD RUN");
    expect(g_load.state==LOAD_READY && t_load_starts==1U,"cooldown survives RETEST and CHECK");
    t_ticks=stopped+10000U; selftest_poll(); execute("LOAD RUN"); advance(300U);
    expect(t_load_starts==2U,"cooldown boundary allows explicit second request");
    execute("STOP"); advance(1U);
    expect(g_load.state==LOAD_FAULT && !t_enabled,"STOP aborts output and locks fault");

    prepare_load(); execute("LOAD RUN"); advance(300U); t_adc_fail=true; advance(1U);
    expect(g_load.state==LOAD_FAULT && !t_enabled && strcmp(g_load.reason,"ADC")==0,"ADC failure shuts down first");
    prepare_load(); execute("LOAD RUN"); advance(300U); t_load_delta=41; advance(1U);
    expect(g_load.state==LOAD_FAULT && strcmp(g_load.reason,"CURRENT_LIMIT")==0,"gain40 0.8A guard uses 32mV");
    prepare_load(); t_load_delta=41; execute("LOAD RUN"); advance(300U);
    expect(g_load.state==LOAD_FAULT && g_load.during.count==0U && !t_enabled,
        "first output sample can trip before ten millisecond statistics window");
    expect(strstr(t_log,"LOAD TRIP SAMPLE_VALID=1 STAGE=OUTPUT ELAPSED_MS=1 OVER_MASK=2")!=NULL &&
        strstr(t_log,"DELTA=0,41,0")!=NULL && strstr(t_log,"EST_MA=0,826,0")!=NULL,
        "first sample fault retains phase raw offset current and time after shutdown");
    prepare_load(); execute("LOAD RUN"); advance(300U); t_load_delta=39; advance(1U);
    expect(g_load.state==LOAD_OUTPUT,"39 codes remains below nominal 0.8A guard");
    t_load_delta=2047; advance(1U);
    expect(g_load.state==LOAD_FAULT && strcmp(g_load.reason,"SATURATION")==0,"saturated sample stops");
    prepare_load(); execute("LOAD RUN"); advance(300U); t_adc.raw[0]=900U; advance(1U);
    expect(g_load.state==LOAD_FAULT && strcmp(g_load.reason,"SUPPLY")==0,"live supply change stops");
    prepare_load(); execute("LOAD RUN"); advance(300U); t_registers[5]^=1U; advance(11U);
    expect(g_load.state==LOAD_FAULT && !t_enabled,"periodic configuration corruption stops");
    prepare_load(); execute("LOAD RUN"); advance(300U); t_registers[0]=1U; advance(11U);
    expect(g_load.state==LOAD_FAULT && !t_enabled,"DRV latched fault stops");
    prepare_load(); execute("LOAD RUN"); advance(300U); t_ticks+=6U; selftest_poll();
    expect(g_load.state==LOAD_FAULT && strcmp(g_load.reason,"SAMPLE_TIMEOUT")==0,"sample freshness deadline");
    prepare_load(); execute("LOAD RUN"); t_load_stuck=true; advance(1400U);
    expect(g_load.state==LOAD_FAULT && !t_enabled,"timer end timeout has software shutdown");
    prepare_load(); execute("LOAD RUN"); advance(300U); t_load_active=false; advance(1U);
    expect(g_load.state==LOAD_FAULT && !t_enabled,"early hardware break is not normal completion");
    prepare_load(); execute("LOAD RUN"); advance(300U); t_inputs.rx_errors++; advance(1U);
    expect(g_load.state==LOAD_FAULT && !t_enabled,"UART error cannot leave output enabled");
    prepare_load(); execute("LOAD RUN"); advance(300U); t_rx="LOAD !RUN\nLOAD RUN\n"; advance(1U);
    expect(g_load.state==LOAD_FAULT && !t_enabled && t_load_starts==1U,"emergency byte discards residual command");
    fixture(); run_checks(); execute("LOAD CHECK"); t_drv_fail=true; advance(20U);
    expect(g_load.state==LOAD_FAULT && t_load_starts==0U,"calibration driver failure prevents output");
    fixture(); run_checks(); execute("LOAD CHECK"); t_adc.raw[5]=1200U; advance(400U);
    expect(g_load.state==LOAD_FAULT && strcmp(g_load.reason,"ZERO_NOISE")==0,"bad calibration baseline blocked");
    prepare_load(); t_ticks=UINT32_MAX-500U; g_load.last_tick=t_ticks; execute("LOAD RUN"); advance(1600U);
    expect(g_load.state==LOAD_DONE && cooldown(t_ticks)>0U,"measurement and cooldown across tick wrap");
}

/** @brief 对实际应用执行正常与异常状态序列。 @return 退出码。 */
int
main(void)
{
    test_load();
    fixture(); run_checks();
    expect(t_power_pulses==0U && t_logic_pulses==6U && t_unsafe==0U,"startup has no bridge pulses");
    expect(g_selftest.driver==RESULT_PASS && g_selftest.csa==RESULT_PASS,"driver and CSA startup");
    expect(g_selftest.encoder==RESULT_WARN,"missing magnet does not fail communication");
    run_power();
    expect(t_power_pulses==6U && t_unsafe==0U && !t_stop,"six interlocked pulses");
    expect(g_selftest.power==RESULT_MANUAL,"never claim low sides pass from zero volts");
    expect((t_registers[2]&4U)!=0U,"restore COAST after test");
    run_sequence();
    expect(t_step_pulses==120U && t_unsafe==0U && !t_stop,"bounded six-step burst has ordered pairs");
    expect(g_selftest.sequence==RESULT_MANUAL && (t_registers[2]&4U)!=0U,
        "six-step burst remains manual and restores COAST");
    fixture(); run_checks(); run_sequence();
    expect(t_step_pulses==0U && g_selftest.sequence==RESULT_BLOCKED && !t_stop,
        "six-step requires conservative test first");
    fixture(); run_checks(); run_power(); t_step_fault=true; run_sequence();
    expect(t_step_pulses==4U && t_stop && g_selftest.sequence==RESULT_FAIL,
        "six-step fault stops remaining states");
    fixture(); t_adc_fail=true; run_checks(); run_power();
    expect(t_power_pulses==0U && t_stop,"ADC failure blocks all power");
    fixture(); t_drv_fail=true; run_checks(); run_power();
    expect(t_power_pulses==0U && t_stop,"driver missing blocks power");
    fixture(); t_adc.raw[4]=0U; run_checks(); run_power();
    expect(t_power_pulses==0U && g_selftest.csa==RESULT_FAIL,"CSA failure blocks power");
    fixture(); run_checks(); test_gpio_b.IDR&=~GPIO_PIN_2; run_power();
    expect(t_power_pulses==0U && t_stop,"PG loss blocks power");
    fixture(); run_checks(); t_adc.raw[0]=800U; run_power();
    expect(t_power_pulses==0U && t_stop,"live undervoltage blocks power");
    fixture(); run_checks(); t_adc.vref_mv=2800U; run_power();
    expect(t_power_pulses==0U && t_stop && g_selftest.supply==RESULT_WARN,"3V3 rail deviation blocks power");
    fixture(); run_checks(); t_adc.raw[1]=1185U; t_adc.raw[2]=1185U; t_adc.raw[3]=1185U; run_power();
    expect(t_power_pulses==6U && !t_stop && t_unsafe==0U,"prehigh phases do not block manual pulses");
    expect(g_selftest.gate_result[0]==RESULT_MANUAL && g_selftest.gate_result[2]==RESULT_MANUAL &&
        g_selftest.gate_result[4]==RESULT_MANUAL,"prehigh high-side ADC is not reported as pass");
    expect((t_registers[2]&4U)!=0U,"prehigh test restores COAST");
    fixture(); run_checks(); t_pulse_fault=true; run_power();
    expect(t_power_pulses==1U && t_stop && g_selftest.power==RESULT_FAIL,"fault stops remaining pulses");
    fixture(); run_checks(); execute("STOP"); run_power();
    expect(t_power_pulses==0U && t_stop,"STOP remains latched");
    execute("RETEST");
    expect(!t_stop && g_selftest.driver==RESULT_PASS,"explicit retest recovers");
    fixture(); t_enc_fail=true; run_checks(); check_encoder(false);
    expect(g_selftest.encoder==RESULT_MISSING && g_selftest.encoder_frames==0U,"stuck-zero CRC cannot bypass probe");
    fixture(); run_checks(); t_rx="POWER\n"; receive();
    expect(t_power_pulses==0U,"incomplete power command ignored");
    t_rx="POWER NOLOAD\n"; t_inputs.rx_errors=1U; receive();
    expect(t_power_pulses==0U && t_stop,"UART loss discards command and stops");
    char oversized[100]; memset(oversized,'X',sizeof(oversized));
    memcpy(oversized+85,"POWER NOLOAD\n",13U); oversized[98]='\0';
    t_rx=oversized; receive(); receive();
    expect(t_power_pulses==0U,"overlong command never runs suffix");
    execute("RETEST");
    t_rx="power noload\r\n"; receive();
    expect(t_power_pulses==6U,"explicit complete command executes once");
    fixture();
    t_inputs.up_presses=1U; selftest_poll();
    expect(g_page==2U,"up selects previous page with wrap");
    selftest_poll();
    expect(g_page==2U,"holding direction does not repeat page");
    t_inputs.down_presses=2U; selftest_poll();
    expect(g_page==1U,"multiple down events between polls are retained");
    t_inputs.held_ms=100U; t_inputs.up_presses=2U; selftest_poll();
    expect(g_page==1U,"holding select suppresses direction page changes");
    g_last_held=0U; t_inputs.held_ms=0U; selftest_poll();
    expect(g_page==1U,"suppressed direction is not replayed on release");
    g_last_up=UINT32_MAX; t_inputs.up_presses=0U; selftest_poll();
    expect(g_page==0U,"direction counter wrap produces one event");
    fixture(); t_inputs.presses=1U; t_inputs.held_ms=20U; selftest_poll();
    expect(g_page==0U,"select does not act before release");
    t_inputs.held_ms=0U; selftest_poll();
    expect(g_page==1U && t_power_pulses==0U,"short select advances page without power output");
    t_inputs.presses=2U; t_inputs.held_ms=20U; selftest_poll();
    t_inputs.held_ms=0U; selftest_poll();
    expect(g_page==1U && g_power_choice==1U && t_step_pulses==0U,
        "power menu short press selects six-step without output");
    fixture(); run_checks(); run_power();
    g_selftest.display=true; g_page=1U; g_power_choice=1U;
    t_inputs.presses=1U; t_inputs.held_ms=3000U; selftest_poll();
    expect(t_step_pulses==120U && g_selftest.sequence==RESULT_MANUAL,
        "power menu long press runs selected six-step once");
    selftest_poll();
    expect(t_step_pulses==120U,"holding key does not repeat six-step");
    t_inputs.held_ms=0U; selftest_poll();
    expect(g_power_choice==1U && t_step_pulses==120U,
        "long-press release does not change selection or retrigger output");
    printf("Selftest application checks=%u failures=%u\n",t_checks,t_failures);
    return t_failures?1:0;
}
