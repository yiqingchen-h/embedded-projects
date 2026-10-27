#include "board_motor.h"
#include "motor_config.h"
#include "drv8353f.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* 仅替换校准函数直接访问的寄存器，不使用真实外设地址。 */
typedef struct { uint32_t BSRR, IDR; } gpio_t;
typedef struct { uint32_t CFGR; } adc_t;
static gpio_t gpio;
static adc_t adc, adc2;
#define GPIOB (&gpio)
#define ADC1 (&adc)
#define ADC2 (&adc2)
#define GPIO_PIN_9 (1U << 9)
#define GPIO_PIN_12 (1U << 12)
#define ADC_CFGR_JQDIS (1U << 31)
/* 与正式模块同名，供提取出的实际函数访问。 */
static board_motor_status_t g_motor_status;
static bool g_motor_initialized, g_motor_adc_active;
/* 双 ADC 拓扑仅改变电流来源及慢速发布范围。 */
static bool g_motor_bemf_enabled;
static uint32_t g_motor_idle_ms;
/* 使用可控的工厂标定值，真实发布函数不访问芯片地址。 */
static const uint16_t calibration_reference=1500;
#define VREFINT_CAL_ADDRESS ((uintptr_t)&calibration_reference)
#define VREFINT_CAL_MV 3000U
/* 模拟电源上升过程中的驱动就绪时间，单位 us。 */
static uint32_t elapsed_us, driver_ready_us, first_config_us;
static bool driver_failure, adc_failure, monitor_failure, zero_reference, outputs_off;
/* 注入任一路ADC停止或使能失败。 */
static bool adc_stop_failure, adc_enable_failure;
/* 停机扫描替身：1为固定通道偏置，2/3为50/100Hz旋转电压。 */
static unsigned rest_mode;
static bool rest_unsafe,rest_finish_stop_failure;
static unsigned config_calls, checks;
static uint8_t configured_gain;
static unsigned channel_reads[19];
#define CHECK(x) do { ++checks; assert(x); } while(0)

/** @brief 提供等待和转换结束后的实际毫秒时钟。 @return 模拟当前毫秒。 */
static uint32_t HAL_GetTick(void) { return elapsed_us/1000U; }
/** @brief 累积停机等待时间。 @param us 微秒。 */
static void delay_us(uint32_t us) { elapsed_us+=us; }
/** @brief 替换板卡 SPI 入口，校准测试的驱动替身不实际发送。 @param tx 帧。 @param rx 返回。 @return 不发送，返回false。 */
static bool driver_transfer(uint16_t tx,uint16_t *rx) { (void)tx; (void)rx; return false; }
/** @brief 模拟 ADC 停止。 @return 成功。 */
static bool adc_quiesce(void) { return !adc_stop_failure && !(rest_finish_stop_failure&&g_motor_status.rest_scans>=128U); }
/** @brief 替换功率/ADC前提，真实储存安全判断另由采样回归覆盖。 @return 停机且可采。 */
bool board_motor_storage_safe(void) { return !rest_unsafe&&!g_motor_status.running; }
/** @brief 扫描只切回原软件ADC拓扑，不允许使能无感注入。 @param enabled 必须false。 @return ADC回收成功。 */
bool board_motor_set_bemf(bool enabled)
{
    CHECK(!enabled);
    g_motor_bemf_enabled=false;
    return adc_quiesce();
}
/** @brief 模拟 ADC 就绪。 @return 成功。 */
static bool adc_enable(void) { return !adc_enable_failure; }
/** @brief 记录六路输出是否全部禁用。 @param output 校准前的输出配置。 */
static void apply_output(const board_motor_output_t *output) { outputs_off=!output->pulse; }
/** @brief 提供实际零点和慢速量，并注入零点/母线采样失败。 @param channel ADC输入。 @param raw 结果。 @return 采样成功。 */
static bool adc_read(uint32_t channel,uint16_t *raw)
{
    CHECK((channel>=6 && channel<=8) || (channel>=1 && channel<=4) || channel==18);
    ++channel_reads[channel];
    ++elapsed_us;
    if(channel>=6 && channel<=8) { *raw=(uint16_t)(2048+channel-6); }
    else if(channel==1) { *raw=1154; }
    else if(channel==18) { *raw=zero_reference?0:1373; }
    else if(rest_mode)
    {
        static const int offset[3]={18,27,16};
        unsigned phase=channel-2U;
        int value=offset[phase]+(int)((g_motor_status.regular_count+phase)%3U)-1;
        if(rest_mode==4U && g_motor_status.regular_count%19U==phase*5U) { value+=12; }
        if(rest_mode==2U||rest_mode==3U)
        {
            unsigned period=rest_mode==2U?20000U:10000U;
            unsigned t=(elapsed_us+phase*period/3U)%period;
            /* 三相三角波供拒绝测试，不用同一个静止判断的实现副本。 */
            unsigned triangle=t<period/2U?t*80U/(period/2U):(period-t)*80U/(period/2U);
            value=offset[phase]+(int)triangle;
        }
        *raw=(uint16_t)value;
    }
    else { *raw=0; }
    return !adc_failure && !(monitor_failure && channel==1);
}
/** @brief 保存配置回读成功状态。 @return 成功。 */
static bool driver_capture(void) { g_motor_status.driver_verified=true; return true; }
/** @brief 模拟校准失败后的硬件关断并保留原因。 @param reason 故障原因，NULL保留原原因。 */
void board_motor_stop(const char *reason)
{
    gpio.BSRR=GPIO_PIN_9<<16;
    g_motor_status.running=false; g_motor_status.calibrated=false;
    if(reason) { g_motor_status.fault=reason; }
}
/** @brief 就绪前拒绝配置，记录首次访问时间；检查功率输出保持禁用。 @param transfer 板卡入口。 @param gain 增益。 @return 电源已就绪且无注入故障。 */
bool drv8353f_init_gain(drv8353f_transfer_t transfer,uint8_t gain)
{
    CHECK(transfer!=NULL && (gain==10 || gain==20 || gain==40));
    CHECK(outputs_off && !g_motor_status.running);
    if(config_calls++==0) { first_config_us=elapsed_us; }
    configured_gain=gain;
    return !driver_failure && elapsed_us>=driver_ready_us;
}
/** @brief 模拟 CSA 校准位切换。 @param enabled 短接状态。 @return 成功。 */
bool drv8353f_calibrate(bool enabled) { (void)enabled; return true; }
/** @brief 检查校准仍为COAST。 @param coast 高阻状态。 @return 成功。 */
bool drv8353f_verify(bool coast) { CHECK(coast); return true; }
/** @brief 模拟无驱动故障。 @param f1 第一组。 @param f2 第二组。 @return 成功。 */
bool drv8353f_faults(uint16_t *f1,uint16_t *f2) { *f1=*f2=0; return true; }
#if MOTOR_ENCODER_ENABLED
/* 编码器只模拟采集服务耗时，不改变原ADC或静止判断输入。 */
static unsigned encoder_polls;
static uint32_t encoder_last_us,encoder_gap_max;
static void board_encoder_service(void)
{
    if(encoder_polls!=0U && elapsed_us-encoder_last_us>encoder_gap_max) { encoder_gap_max=elapsed_us-encoder_last_us; }
    encoder_last_us=elapsed_us; ++encoder_polls;
}
#endif
#include "calibration_under_test.inc"

/** @brief 重置替身，使4ms后才就绪的驱动复现首次上电瞬态。 */
static void reset(void)
{
    memset(&g_motor_status,0,sizeof(g_motor_status));
    memset(&gpio,0,sizeof(gpio)); memset(&adc,0,sizeof(adc)); memset(&adc2,0,sizeof(adc2));
    memset(channel_reads,0,sizeof(channel_reads)); g_motor_bemf_enabled=false;
    g_motor_initialized=true; gpio.IDR=GPIO_PIN_12;
    elapsed_us=first_config_us=config_calls=0; driver_ready_us=4000;
    driver_failure=adc_failure=monitor_failure=zero_reference=outputs_off=false;
    adc_stop_failure=adc_enable_failure=false;
    rest_mode=0;rest_unsafe=rest_finish_stop_failure=false;
    g_motor_adc_active=false; g_motor_idle_ms=0;
#if MOTOR_ENCODER_ENABLED
    encoder_polls=0U;encoder_last_us=encoder_gap_max=0U;
#endif
}

/** @brief 新实测时间戳不受旧调度时间影响；停止更新超过门限时仍须关断。 */
static void test_bus_watchdog(void)
{
    reset(); CHECK(board_motor_calibrate(50));
    uint32_t completed=HAL_GetTick();
    CHECK(g_motor_status.bus_mv>=11900 && g_motor_status.bus_mv<=12100);
    CHECK(g_motor_status.vdda_mv>=3200 && g_motor_status.vdda_mv<=3400);
    g_motor_status.running=true;
    /* DMA可能在主循环取得调度时间后跨过一个毫秒完成，不能无符号下溢。 */
    board_motor_service(completed-1U);
    CHECK(g_motor_status.running && strcmp(g_motor_status.fault,"NONE")==0);
    elapsed_us+=MOTOR_BUS_STALE_MS*1000U;
    board_motor_service(completed-1U);
    CHECK(g_motor_status.running);
    CHECK(g_motor_status.bus_age_ms==MOTOR_BUS_STALE_MS);
    elapsed_us+=1000U;
    board_motor_service(completed-1U);
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"BUS STALE")==0);
    CHECK(g_motor_status.bus_age_ms==MOTOR_BUS_STALE_MS+1U);
    /* 校准后采样没有实际完成时不能伪造新时间戳或继续启动。 */
    reset(); g_motor_status.slow_ms=7; monitor_failure=true;
    CHECK(!board_motor_calibrate(50));
    CHECK(g_motor_status.slow_ms==7 && g_motor_status.regular_count==0);
    CHECK(!g_motor_status.running && !g_motor_status.calibrated && gpio.BSRR==(GPIO_PIN_9<<16));
    reset(); zero_reference=true; CHECK(!board_motor_calibrate(50));
    CHECK(g_motor_status.regular_count==0 && !g_motor_status.calibrated);
    /* 停机服务也使用真实结束时间发布，不继承传入的旧调度时间。 */
    reset(); elapsed_us=20000;
    board_motor_service(10);
    CHECK(g_motor_status.slow_ms==HAL_GetTick() && g_motor_status.regular_count==1);
}

/** @brief 按共用快速时基推进超过母线超时长度，实际慢速发布每毫秒完成时不能误停。 */
static void
test_regular_keeps_bus_fresh(void)
{
    reset(); CHECK(board_motor_calibrate(50));
    g_motor_status.running=true;
    /* 完整慢速序列：SOC诊断、母线、三相电压和有效参考；使用真实发布与看门函数。 */
    const uint16_t values[6]={2048,1154,0,0,0,1373};
    const unsigned ticks_per_ms=MOTOR_CONTROL_HZ/1000U;
    const unsigned duration_ms=MOTOR_BUS_STALE_MS+20U;
    for(unsigned tick=1;tick<=duration_ms*ticks_per_ms;tick++)
    {
        elapsed_us+=1000000U/MOTOR_CONTROL_HZ;
        if(tick%ticks_per_ms==0U) { CHECK(publish_regular(values,HAL_GetTick())); }
        board_motor_service(HAL_GetTick()-1U);
        CHECK(g_motor_status.running && strcmp(g_motor_status.fault,"NONE")==0);
        CHECK(g_motor_status.bus_age_ms<=1U);
    }
    CHECK(g_motor_status.regular_count==1U+duration_ms);
}

/** @brief 双ADC校准仍实采三零点，慢速量仅更新母线/参考，失败均保持功率禁用。 */
static void test_bemf_calibration(void)
{
    reset(); g_motor_bemf_enabled=true;
    g_motor_status.phase_mv[0]=1234; g_motor_status.phase_sample_us[0]=5678;
    CHECK(board_motor_calibrate(50));
    CHECK(g_motor_status.zero[0]==2048U && g_motor_status.zero[1]==2049U && g_motor_status.zero[2]==2050U);
    CHECK(channel_reads[6]==MOTOR_ZERO_SAMPLES && channel_reads[7]==MOTOR_ZERO_SAMPLES &&
        channel_reads[8]==MOTOR_ZERO_SAMPLES);
    CHECK(channel_reads[1]==1U && channel_reads[18]==1U);
    CHECK(channel_reads[2]==0U && channel_reads[3]==0U && channel_reads[4]==0U);
    CHECK(adc.CFGR==ADC_CFGR_JQDIS && adc2.CFGR==ADC_CFGR_JQDIS);
    CHECK(g_motor_status.regular_count==1U && g_motor_status.slow_ms==HAL_GetTick());
    CHECK(g_motor_status.phase_mv[0]==1234U && g_motor_status.phase_sample_us[0]==5678U);
    CHECK(!g_motor_status.phase_valid[0] && !g_motor_status.phase_valid[1] && !g_motor_status.phase_valid[2]);
    const uint16_t monitor[6]={0,1154,0,0,0,1373};
    CHECK(publish_regular(monitor,HAL_GetTick()));
    CHECK(g_motor_status.phase_mv[0]==1234U && !g_motor_status.phase_valid[0]);
    /* 运行母线越界和参考无效不可被双ADC精简扫描隐藏。 */
    g_motor_status.running=true;
    const uint16_t undervoltage[6]={0,1,0,0,0,1373};
    CHECK(!publish_regular(undervoltage,HAL_GetTick()));
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"BUS VOLTAGE")==0);
    for (unsigned invalid=0U;invalid<2U;++invalid)
    {
        reset(); g_motor_status.running=true;
        uint16_t bad_vref[6]={0,1154,0,0,0,invalid?4095U:1U};
        CHECK(!publish_regular(bad_vref,HAL_GetTick()));
        CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"VDDA")==0);
    }
    reset(); g_motor_bemf_enabled=true; zero_reference=true;
    CHECK(!board_motor_calibrate(50) && gpio.BSRR==(GPIO_PIN_9<<16U));
    reset(); g_motor_bemf_enabled=true; adc_stop_failure=true;
    CHECK(!board_motor_calibrate(50) && strcmp(g_motor_status.fault,"ADC STOP")==0);
    CHECK(config_calls==0U && gpio.BSRR==(GPIO_PIN_9<<16U));
    reset(); g_motor_bemf_enabled=true; adc_enable_failure=true;
    CHECK(!board_motor_calibrate(50) && strcmp(g_motor_status.fault,"ADC ENABLE")==0);
    CHECK(config_calls==0U && gpio.BSRR==(GPIO_PIN_9<<16U));
}

/** @brief 静态偏置不当作旋转，50/100Hz变化、转换故障和不安全前提全部拒绝。 */
static void
test_standstill(void)
{
    reset();rest_mode=1;
    CHECK(board_motor_standstill());
    CHECK(g_motor_status.rest_scans==128U&&g_motor_status.rest_line_ptp_mv<=80U);
    CHECK(elapsed_us>=128000U);
#if MOTOR_ENCODER_ENABLED
    CHECK(encoder_polls==128U && encoder_gap_max<=2000U);
#endif
    CHECK(g_motor_status.phase_mv[1]-g_motor_status.phase_mv[2]>80);
    reset();rest_mode=4;
    CHECK(board_motor_standstill());
    CHECK(g_motor_status.rest_line_raw_ptp_mv>80U&&g_motor_status.rest_line_ptp_mv<=80U);
    for(unsigned mode=2U;mode<=3U;++mode)
    {
        reset();rest_mode=mode;
        CHECK(!board_motor_standstill());
        CHECK(g_motor_status.rest_scans<128U&&g_motor_status.rest_line_ptp_mv>80U);
        CHECK(!g_motor_status.running&&config_calls==0U);
    }
    reset();rest_unsafe=true;CHECK(!board_motor_standstill()&&g_motor_status.rest_scans==0U);
    reset();g_motor_status.running=true;CHECK(!board_motor_standstill()&&g_motor_status.rest_scans==0U);
    reset();adc_enable_failure=true;CHECK(!board_motor_standstill()&&g_motor_status.rest_scans==0U);
    reset();monitor_failure=true;CHECK(!board_motor_standstill()&&g_motor_status.rest_scans==0U);
    reset();zero_reference=true;CHECK(!board_motor_standstill()&&g_motor_status.rest_scans==0U);
    reset();rest_mode=1;rest_finish_stop_failure=true;
    CHECK(!board_motor_standstill()&&g_motor_status.rest_scans==128U);
}

/** @brief 校准等待覆盖电源上升窗口；驱动、ADC或nFAULT失败均不能继续输出。 @return 全部通过为0。 */
int main(void)
{
    reset(); CHECK(board_motor_calibrate(50));
    CHECK(first_config_us>=driver_ready_us && config_calls==1);
    CHECK(first_config_us==MOTOR_DRIVER_SLEEP_US+MOTOR_DRIVER_WAKE_US);
    CHECK(MOTOR_DRIVER_SLEEP_US==1000U && MOTOR_DRIVER_WAKE_US==100000U);
    CHECK(g_motor_status.driver_verified && strcmp(g_motor_status.fault,"NONE")==0);
    CHECK(g_motor_status.zero[0]==2048 && g_motor_status.zero[2]==2050);
    /* 复现100 ms唤醒后继承旧母线时间戳：成功校准必须发布实际新采样。 */
    CHECK(g_motor_status.slow_ms==HAL_GetTick());
    CHECK(g_motor_status.regular_count==1);
    CHECK(configured_gain==10U);
    reset(); CHECK(board_motor_calibrate(1)); CHECK(configured_gain==40U);
    /* 小电流增益完成实际校准；不合法分流电阻/增益组合不得接触驱动。 */
    reset(); CHECK(board_motor_calibrate_gain(50,20)); CHECK(configured_gain==20U);
    CHECK(g_motor_status.driver_verified && !g_motor_status.running && outputs_off);
    reset(); CHECK(!board_motor_calibrate_gain(1,20)); CHECK(config_calls==0U);
    CHECK(!board_motor_calibrate_gain(50,40)); CHECK(config_calls==0U);
    reset(); driver_failure=true; CHECK(!board_motor_calibrate(50));
    CHECK(gpio.BSRR==(GPIO_PIN_9<<16) && !g_motor_status.calibrated);
    reset(); adc_failure=true; CHECK(!board_motor_calibrate(50));
    CHECK(gpio.BSRR==(GPIO_PIN_9<<16) && !g_motor_status.calibrated);
    reset(); gpio.IDR=0; CHECK(!board_motor_calibrate(50));
    CHECK(gpio.BSRR==(GPIO_PIN_9<<16) && !g_motor_status.calibrated);
    test_bus_watchdog();
    test_regular_keeps_bus_fresh();
    test_bemf_calibration();
    test_standstill();
    printf("board calibration: %u checks passed\n",checks); return 0;
}
