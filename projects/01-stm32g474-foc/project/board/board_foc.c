#include "board_foc.h"
#include "wave_stream.h"
#include "board_motor.h"
#include "foc_voltage.h"
#include "foc_current.h"
#include "foc_smo.h"
#include "foc_math.h"
#include "foc_drive.h"
#include "motor_config.h"
#if MOTOR_ENCODER_ENABLED
#include "board_encoder.h"
#include "encoder_motion.h"
#endif
#include "drv8353f.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* 已核对的板卡时钟及独立FOC载波；六步仍使用原宏。 */
#define FOC_TIMER_HZ 170000000U
#define FOC_ARR 8500U
#define FOC_PERIOD_TICKS (2U * FOC_ARR)
/* 正式PWM每50us载波，RCR=3每100us谷底转移预装载；算法仍用8500归一化比较值。 */
#define FOC_PWM20_ARR (FOC_ARR / 2U)
/* 正式20kHz采样仍在峰顶前100tick；独立台架保留原10kHz触发。 */
#define FOC_PWM20_TRIGGER 4150U
#define FOC_TRIGGER_TICKS 8400U
/* 保持发生在6.5周期采样结束；加最坏2.5周期同步。初始估计仍待实测校核。 */
#define FOC_HOLD_TICKS 36U
/* 预装载写入距下次100us谷底UEV至少5us；20kHz的中间谷底不转移CCR。 */
#define FOC_COMMIT_GUARD 850U
/* 启动等待仅用于停机配置，ISR不能调用。 */
#define FOC_WAIT_TICKS 340000U
/* 当前时序测试三相均值相等；异常电流0.5A即关闭，正常FOC另行验收量程。 */
#define FOC_BENCH_TRIP_MA 500U
/* 慢速SOC、母线、三相电压和参考六项，ADC1 DMA请求5。 */
#define FOC_SLOW_ITEMS 6U
#define FOC_DMA_REQUEST 5U
/* 3.1ms正负小电压交替，避开串口约1ms打印间隔的固定相位。 */
#define FOC_SWITCH_HOLD_CYCLES 31U
/* 在实际同占空比PWM下校准零点；只积累控制校准总和，不保存诊断样本。 */
#define FOC_ZERO_SAMPLES 128U
/* 有界dq阶跃5.3ms保持，避免与低带宽打印固定相位；尚不启用正常FOC START。 */
#define FOC_CURRENT_HOLD_CYCLES 53U
/* 200Hz初值在旋转反电动势下偏差过大；800Hz候选需通过延迟/谐波及上板验证。 */
#define FOC_CURRENT_BANDWIDTH_HZ ((float)MOTOR_FOC_BANDWIDTH_HZ)
/* 小电流正式入口及禁止输出试验候选增益；高于0.5A指令仍采用原10倍量程。 */
#define FOC_LOW_CURRENT_GAIN 20U
/* 主循环配置/诊断及实时SMO、双速率ADC按尺寸优化，执行预算须实际测量。 */
#if defined(__ARMCC_VERSION)
#define FOC_DIAGNOSTIC_CODE __attribute__((minsize))
#else
#define FOC_DIAGNOSTIC_CODE
#endif

typedef enum {
    FOC_TEST_STATIC,
    FOC_TEST_SWITCH,
    FOC_TEST_ID,
    FOC_TEST_IQ,
    FOC_TEST_RANGE,
    FOC_TEST_IF,
    FOC_TEST_OBSERVE,
    FOC_TEST_CLOSED,
    /* 仅运行同步双ADC和定时器，COAST保持、MOE禁止，不切换MOS。 */
    FOC_TEST_ZERO
} foc_test_mode_t;

typedef struct {
    /* 运行、回收与退出原因；中断只写静态文本。 */
    bool active, restore, production;
    /* 编码器FOC与原SMO路径互斥，停机后保留本轮诊断。 */
    bool encoder_mode;
    const char *fault;
    /* 同域采样与提交编号，UEV只在谷底推进完整周期。 */
    uint32_t cycle, samples, updates, speed_checks, intervals;
    /* 最近硬件采样保持、ISR服务与截止裕量，单位TIM1 tick。 */
    uint32_t sample_ticks, age_max, margin_min, adc_max, update_max;
    /* 首末原始电流、绝对峰值和校准量程。 */
    uint16_t raw[3], zero[3], vdda_mv, bus_mv, trip_raw;
    /* 50mΩ换算系数：CSA10为2、CSA20为1；原始码和各处mA/A换算必须一致。 */
    uint32_t current_units;
    uint32_t peak_raw, bus_ms, last_update_cycle;
    /* 递减64位周期数，按用户设置到时停机；不再人为限制正常FOC单轮秒数。 */
    uint64_t remaining_cycles;
    /* 双ADC完成计数和慢速采样状态。 */
    uint32_t adc1_count, adc2_count, slow_count;
    /* 测试配置保持不变，ADC保持点取第一组同时采样的SOA/SOB。 */
    uint32_t trigger_ticks;
    /* 动态测试幅度及开关策略，启动后保持配置不变。 */
    uint16_t center, spread;
    foc_test_mode_t test_mode;
    /* 有界电流环指令幅度，单位A。 */
    float current_target_a;
    /* 有界量程试验固定参考角和静止资格；这是运行保护状态，不是打印历史。 */
    float range_cos, range_sin;
    uint32_t range_still_cycles;
    uint16_t range_trip_raw;
    /* 强拖电角度/电角速度及目标；只表示外部轨迹，不是估计转子角度。 */
    float force_theta, force_omega, force_target_omega;
    /* 零电压PWM校准中测得滤波EMF噪声；只保存控制资格所需的结果。 */
    float emf_noise_v;
    bool slow_busy;
    /* 首个故障现场寄存器，回收后仍可查询。 */
    uint32_t fault_cnt, fault_cr1, fault_adc1, fault_adc2, fault_jsqr, fault_adc_cr;
} foc_board_state_t;

/* 中断拥有运行状态，后台只在短临界区取快照。 */
static volatile foc_board_state_t g_foc={.current_units=2U};
/* 已生效历史与待提交命令完全分离，仅实时中断修改。 */
static foc_voltage_history_t g_foc_history;
static foc_voltage_command_t g_foc_preloaded;
static foc_voltage_interval_t g_foc_interval;
/* ADC拥有dq控制状态，主循环只读取诊断；真实参数尚为低电流候选值。 */
static foc_current_t g_foc_current;
/* 仅观察模式启用；与请求电压及强拖轨迹没有输入关联。 */
static foc_smo_t g_foc_smo;
/* 完整模式的实时状态，电流/观察在ADC、速度/实际提交确认在TIM。 */
static foc_drive_t g_foc_drive;
#if MOTOR_ENCODER_ENABLED
/* 独立有感状态，不借用无感定位/强拖或SMO资格。 */
static encoder_foc_t g_foc_encoder;
/* 独立的原始机械测量，不参与电流、速度或任何保护反馈。 */
static encoder_motion_t g_foc_motion;
/* 每次正常有感START递增，主机据此分开各轮记录。 */
static uint32_t g_foc_motion_run;
#endif
/* 慢速DMA缓冲固定大小，关闭后不再交给六步DMA处理。 */
static volatile uint16_t g_foc_slow[FOC_SLOW_ITEMS];
/* 仅主循环使用的已打印序号，避免空闲循环重复同一份最新样本；ADC不访问。 */
static uint32_t g_foc_printed_sample;
/* 主循环状态快照按20ms取样，与高速电流序列分开。 */
static uint32_t g_foc_wave_status_ms;
/* 停机设置的正式速度采样位置；G2208默认留出CSA20建立时间，不写Flash。 */
static uint32_t g_foc_run_trigger=MOTOR_FOC_TRIGGER_DEFAULT_TICKS;
/* 停机设置的接管试验速度RPM；重启回本档约50Hz对应机械转速，不改变Flash和六步HANDOFF。 */
static uint32_t g_foc_handoff_rpm=MOTOR_FOC_STARTUP_RPM;
/* 主循环会话选择，默认启用；仅正式闭环的ADC电流路径实际采用，不写Flash。 */
static bool g_foc_harmonic_requested=true;
/* 二次与五次独立会话请求，阶段试验不启用。 */
static bool g_foc_harmonic2_requested=true;

/** @brief 停机等待硬件标志，超时返回；禁止ISR使用。
 * @param reg 标志寄存器。 @param mask 位组。 @param set 期望置位。
 * @return 在限定时间内满足。 */
static bool
foc_wait(volatile uint32_t *reg, uint32_t mask, bool set)
{
    uint32_t begun = DWT->CYCCNT;
    while (((*reg & mask) != 0U) != set)
    {
        if (DWT->CYCCNT - begun > FOC_WAIT_TICKS) { return false; }
    }
    return true;
}

/** @brief 功率禁用后停止并禁用双ADC，才能改变DUAL模式。
 * @return 两路转换停止且ADEN清零。 */
static bool
foc_quiesce(void)
{
    ADC_TypeDef *converters[2] = {ADC1, ADC2};
    for (unsigned i=0U; i<2U; ++i)
    {
        ADC_TypeDef *adc = converters[i];
        if (adc->CR & ADC_CR_JADSTART) { adc->CR |= ADC_CR_JADSTP; }
        if (adc->CR & ADC_CR_ADSTART) { adc->CR |= ADC_CR_ADSTP; }
        if (!foc_wait(&adc->CR, ADC_CR_JADSTART | ADC_CR_ADSTART, false)) { return false; }
        if (adc->CR & ADC_CR_ADEN)
        {
            adc->CR |= ADC_CR_ADDIS;
            if (!foc_wait(&adc->CR, ADC_CR_ADEN, false)) { return false; }
        }
    }
    return true;
}

/** @brief 功率禁用时先实际经过峰顶，再在峰顶后写RCR以对齐谷底UEV。
 * @return 已确认谷底且距离采样触发有充足时间；不猜测奇数RCR首事件。 */
static bool
foc_prime_valley(void)
{
    TIM1->RCR = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CNT = 0U;
    TIM1->SR = 0U;
    TIM1->CR1 |= TIM_CR1_CEN;
    if (!foc_wait(&TIM1->SR,TIM_SR_UIF,true) || !(TIM1->CR1 & TIM_CR1_DIR)) { return false; }
    TIM1->SR = 0U;
    TIM1->RCR = g_foc.production ? 3U : 1U;
    if (!foc_wait(&TIM1->SR,TIM_SR_UIF,true) || (TIM1->CR1 & TIM_CR1_DIR) ||
        TIM1->CNT > FOC_COMMIT_GUARD) { return false; }
    TIM1->SR = 0U;
    return true;
}

/** @brief 主循环在零电压PWM下求电流零点，消除功率关闭与开关状态的偏差。
 * @return 128组稳定配对样本有效且输出已重新关闭；失败不能进入带电压测试。 */
static bool
foc_calibrate_pwm_zero(void)
{
    uint32_t sum[3] = {0U,0U,0U};
    uint16_t minimum[3] = {4095U,4095U,4095U}, maximum[3] = {0U,0U,0U};
    float emf_sum_a=0,emf_sum_b=0,emf_square_a=0,emf_square_b=0;
    const foc_voltage_interval_t zero_voltage={.duration_ticks=FOC_PERIOD_TICKS};
    ADC1->IER = ADC2->IER = 0U;
    ADC1->ISR = ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC | ADC_ISR_OVR | ADC_ISR_JQOVF;
    ADC1->CR |= ADC_CR_JADSTART;
    TIM1->BDTR |= TIM_BDTR_MOE;
    uint32_t last = DWT->CYCCNT;
    for (unsigned n=0U; n<FOC_ZERO_SAMPLES; ++n)
    {
        if (!foc_wait(&ADC1->ISR,ADC_ISR_JEOS,true) || !foc_wait(&ADC2->ISR,ADC_ISR_JEOS,true) ||
            ! (TIM1->BDTR & TIM_BDTR_MOE) || !(GPIOB->IDR & GPIO_PIN_12) ||
            ((ADC1->ISR | ADC2->ISR) & (ADC_ISR_OVR | ADC_ISR_JQOVF))) { return false; }
        uint32_t now = DWT->CYCCNT;
        if (now-last > FOC_PERIOD_TICKS+FOC_PERIOD_TICKS/2U) { return false; }
        last = now;
        uint16_t raw[3] = {(uint16_t)ADC1->JDR1,(uint16_t)ADC2->JDR1,(uint16_t)ADC1->JDR2};
        ADC1->ISR = ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
        for (unsigned i=0U; i<3U; ++i)
        {
            int32_t delta = (int32_t)raw[i]-g_foc.zero[i];
            if (delta < -64 || delta > 64 || raw[i]<16U || raw[i]>4079U) { return false; }
            sum[i] += raw[i];
            if (raw[i]<minimum[i]) { minimum[i]=raw[i]; }
            if (raw[i]>maximum[i]) { maximum[i]=raw[i]; }
        }
        if(g_foc.test_mode>=FOC_TEST_OBSERVE || g_foc.test_mode==FOC_TEST_RANGE)
        {
            /* 这是主循环校准，不新增ADC诊断写入；同占空比对应真实零电压。 */
            float scale=g_foc.vdda_mv*(.001f/4095.0f)*g_foc.current_units;
            float a=((int32_t)raw[0]-g_foc.zero[0])*scale;
            float b=(a+2*((int32_t)raw[1]-g_foc.zero[1])*scale)*.57735026919f;
            if(!foc_smo_step(&g_foc_smo,n==0?NULL:&zero_voltage,a,b)) { return false; }
            if(n>=32U)
            {
                float ea=g_foc_smo.filtered_alpha_v,eb=g_foc_smo.filtered_beta_v;
                emf_sum_a+=ea;
                emf_sum_b+=eb;
                emf_square_a+=ea*ea;
                emf_square_b+=eb*eb;
            }
        }
        if (g_foc.production)
        {
            /* 校准SMO也保持100us；中间50us样本只丢弃于此主循环，不伪造采样周期。 */
            if (!foc_wait(&ADC1->ISR,ADC_ISR_JEOS,true) || !foc_wait(&ADC2->ISR,ADC_ISR_JEOS,true) ||
                ((ADC1->ISR | ADC2->ISR) & (ADC_ISR_OVR | ADC_ISR_JQOVF))) { return false; }
            ADC1->ISR = ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
        }
    }
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    ADC1->CR |= ADC_CR_JADSTP;
    if (!foc_wait(&ADC1->CR,ADC_CR_JADSTART,false)) { return false; }
    TIM1->CR1 &= ~TIM_CR1_CEN;
    for (unsigned i=0U; i<3U; ++i)
    {
        if (maximum[i]-minimum[i]>48U) { return false; }
        uint16_t zero = (uint16_t)((sum[i]+FOC_ZERO_SAMPLES/2U)/FOC_ZERO_SAMPLES);
        int32_t shift = (int32_t)zero-g_foc.zero[i];
        if (shift < -32 || shift > 32) { return false; }
        g_foc.zero[i] = zero;
    }
    if(g_foc.test_mode>=FOC_TEST_OBSERVE || g_foc.test_mode==FOC_TEST_RANGE)
    {
        float count=FOC_ZERO_SAMPLES-32U;
        float variance=(emf_square_a+emf_square_b)/count-
            (emf_sum_a*emf_sum_a+emf_sum_b*emf_sum_b)/(count*count);
        g_foc.emf_noise_v=sqrtf(fmaxf(0,variance*.5f));
        /* 正式首电流仍只初始化，不携带校准期间的滤波/速度/相位资格。 */
        if(!foc_smo_init(&g_foc_smo,MOTOR_FOC_RESISTANCE_OHM,MOTOR_FOC_INDUCTANCE_H,.0001f,6,.03f,500,20,160,.05f,FOC_PERIOD_TICKS)) { return false; }
    }
    return true;
}

/** @brief 读取独立硬件占用标志，参见头文件。 */
bool
board_foc_active(void) { return g_foc.active; }

/** @brief 通用关断前撤销FOC，保留最后的实测统计。 */
void MOTOR_FAST_CODE
board_foc_abort(const char *reason)
{
    if (!g_foc.active && !g_foc.restore) { return; }
    g_foc.active = false;
    wave_stream_end();
    g_foc.restore = true;
    g_foc.fault = NULL == reason ? "NONE" : reason;
    g_foc_history.count = 0U;
    g_foc_preloaded.valid = false;
#if MOTOR_ENCODER_ENABLED
    if(g_foc.encoder_mode) { encoder_foc_stop(&g_foc_encoder,reason); }
    else
#endif
    if(g_foc.test_mode==FOC_TEST_CLOSED) { foc_drive_stop(&g_foc_drive,reason); }
    DMA1_Channel1->CCR = 0U;
    DMA1->IFCR = DMA_IFCR_CGIF1;
}

/** @brief 停机回收，正常结束后六步需沿原流程重新校准。 */
void
board_foc_service(void)
{
    uint32_t wave_now=HAL_GetTick();
    if(g_foc.active && wave_stream_enabled() && wave_now-g_foc_wave_status_ms>=20U)
    {
        g_foc_wave_status_ms=wave_now;
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        uint32_t ticks=g_foc.sample_ticks;
        uint16_t bus=g_foc.bus_mv;
        float rpm=g_foc_drive.observer.omega_rad_s*(60.0f/(6.28318530718f*MOTOR_POLE_PAIRS));
        uint32_t target=g_foc_drive.config.target_rpm;
        uint8_t stage=(uint8_t)g_foc_drive.stage;
        bool valid=stage==(uint8_t)FOC_CLOSED;
#if MOTOR_ENCODER_ENABLED
        if(g_foc.encoder_mode)
        {
            rpm=g_foc_encoder.rpm; target=(g_foc_encoder.config.current_mode||g_foc_encoder.config.position_mode||g_foc_encoder.config.haptic_mode)?0U:g_foc_encoder.config.drive.target_rpm;
            stage=g_foc_encoder.closed_seen?(uint8_t)FOC_CLOSED:(uint8_t)FOC_CALIBRATE;
            valid=g_foc_encoder.speed_valid;
        }
#endif
        __set_PRIMASK(mask);
        wave_stream_status(ticks,rpm,(int32_t)(target*1000U),bus,true,stage,valid);
    }
    if (!g_foc.active && g_foc.restore)
    {
#if MOTOR_ENCODER_ENABLED
        if(g_foc.encoder_mode) { board_encoder_control_end(); }
#endif
        if (!foc_quiesce() || !board_motor_restore_profile())
        {
            board_motor_stop("FOC ADC STOP");
            g_foc.fault = "FOC ADC STOP";
            return;
        }
        g_foc.restore = false;
    }
}

/** @brief 有界时序/静态矢量测试，所有等待位于停机阶段。
 * @param duty 三相比较值中心，千分比。 @param duration_ms 时限ms。
 * @param sector 0为等占空比，1..6为固定矢量排序。 @param spread 比较值偏移tick。
 * @param trigger 上数侧采样触发tick。 @param mode 有界测试方式。
 * @param target Id/Iq方式为mA，I/f方式为目标电频率mHz。
 * @return 配置并启用成功；异常电流立即关断。 */
static bool FOC_DIAGNOSTIC_CODE
foc_bench_configure(uint32_t duty, uint32_t duration_ms, uint32_t sector, uint32_t spread,
    uint32_t trigger, foc_test_mode_t mode, uint32_t target,const foc_drive_config_t *drive_config,
    const encoder_foc_config_t *encoder_config)
{
    if (board_motor_running() || g_foc.restore || duty < 50U || duty > 950U ||
        duration_ms < 20U || (NULL==drive_config && duration_ms > (mode==FOC_TEST_CLOSED?32000U:mode>=FOC_TEST_IF?5000U:2000U)) ||
        sector > 6U || spread > 200U || trigger < 8200U || trigger > 8490U)
    {
        return false;
    }
    if(!board_motor_standstill()) { return false; }
    /* 位置0.5..0.8A仍用原CSA20及1.25A异常关断，不随调大指令放宽量程保护。 */
    uint32_t units=(mode==FOC_TEST_ZERO || (encoder_config && (encoder_config->position_mode || encoder_config->haptic_mode)) ||
        (drive_config && drive_config->current_limit_a<=.5f))?
        20U/FOC_LOW_CURRENT_GAIN:2U;
    if (!board_motor_calibrate_gain(50U,(uint8_t)(20U/units))) { return false; }
    board_motor_status_t hw = board_motor_status();
    if ((hw.revision != 0x2002U && hw.revision != 0x2003U) ||
        hw.bus_mv < MOTOR_BUS_MIN_MV || hw.bus_mv > MOTOR_BUS_MAX_MV)
    {
        board_motor_stop("FOC REV/BUS");
        return false;
    }
    if (!foc_quiesce()) { board_motor_stop("FOC ADC STOP"); return false; }
    memset((void *)&g_foc, 0, sizeof(g_foc));
    g_foc.current_units=units;
    memset(&g_foc_history, 0, sizeof(g_foc_history));
    memset(&g_foc_interval, 0, sizeof(g_foc_interval));
    if (!foc_current_init(&g_foc_current,MOTOR_FOC_RESISTANCE_OHM,MOTOR_FOC_INDUCTANCE_H,FOC_CURRENT_BANDWIDTH_HZ,.0001f,FOC_ARR,425U,8075U))
    {
        board_motor_stop("FOC MODEL CONFIG");
        return false;
    }
    if(!foc_smo_init(&g_foc_smo,MOTOR_FOC_RESISTANCE_OHM,MOTOR_FOC_INDUCTANCE_H,.0001f,6,.03f,500,20,160,.05f,FOC_PERIOD_TICKS))
    {
        board_motor_stop("FOC SMO CONFIG");
        return false;
    }
    g_foc_printed_sample = 0U;
    g_foc.fault = "NONE";
    g_foc.restore = true;
    g_foc.vdda_mv = hw.vdda_mv;
    g_foc.bus_mv = hw.bus_mv;
    g_foc.bus_ms = HAL_GetTick();
    g_foc.trigger_ticks = trigger;
    g_foc.test_mode = mode;
    g_foc.production=NULL!=drive_config;
    g_foc.encoder_mode=NULL!=encoder_config;
    uint32_t pwm_arr=g_foc.production?FOC_PWM20_ARR:FOC_ARR;
    if (g_foc.production)
    {
        /* 会话位置保留原10kHz坐标；保持峰顶前的绝对提前量，诊断报告实际tick。 */
        g_foc.trigger_ticks=FOC_PWM20_TRIGGER-(FOC_TRIGGER_TICKS-trigger);
    }
    g_foc.current_target_a = mode>=FOC_TEST_IF?.3f:target*0.001f;
    g_foc.force_target_omega = mode>=FOC_TEST_IF&&mode!=FOC_TEST_CLOSED?target*0.00628318530718f:0.0f;
    g_foc.spread = (uint16_t)spread;
    g_foc.center = (uint16_t)(FOC_ARR * duty / 1000U);
    for (unsigned i=0U; i<3U; ++i) { g_foc.zero[i] = hw.zero[i]; }
    g_foc.trip_raw = (uint16_t)(FOC_BENCH_TRIP_MA * 4095U / (hw.vdda_mv*units));
    if(mode==FOC_TEST_RANGE || drive_config)
    {
        /* 先验证实际零点两侧至少有0.25V余量，再允许当前量程的关断上限；定位仍以0.5A关断。 */
        /* CSA20的线性量程减半，关断提前至1.25A；不能沿用不可测的2.5A门限。 */
        uint32_t trip_ma=drive_config?1250U*units:target+target/4U;
        uint32_t trip_raw=trip_ma*4095U/(hw.vdda_mv*units);
        uint32_t rail=250U*4095U/hw.vdda_mv;
        for(unsigned i=0;i<3;++i)
        {
            if(g_foc.zero[i]<=trip_raw+rail || g_foc.zero[i]+trip_raw+rail>=4095U)
            { board_motor_stop("FOC RANGE HEADROOM"); return false; }
        }
        g_foc.range_trip_raw=(uint16_t)trip_raw;
        if(drive_config) { g_foc.trip_raw=(uint16_t)trip_raw; }
    }
    if(mode==FOC_TEST_RANGE)
    {
        float cosine,sine;
        foc_sincos((sector-1U)*1.0471975512f,&cosine,&sine);
        g_foc.range_cos=cosine;
        g_foc.range_sin=sine;
        /* sector在此只指定电流参考角，首PWM仍是真实零电压。 */
        sector=0U;
    }
    /* 按完整载波周期停机，允许>25.3s而不受170MHz时间戳自然回绕影响。 */
    g_foc.remaining_cycles = (uint64_t)duration_ms * 10U;
    g_foc.margin_min = FOC_PERIOD_TICKS;
    /* RM0440：DUAL=00101为注入同时模式，两路序列长度相同，只有主ADC外部触发。 */
    MODIFY_REG(ADC12_COMMON->CCR, ADC_CCR_DUAL, ADC_CCR_DUAL_0 | ADC_CCR_DUAL_2);
    ADC1->CFGR = ADC_CFGR_JQDIS | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;
    ADC2->CFGR = ADC_CFGR_JQDIS;
    /* 第二组读取SOC作独立一致性保护；只保留最新控制值，不保存诊断历史。
     * 两ADC序列长度相同；Id/Iq和SMO仍使用第一组同时SOA/SOB的时刻。 */
    ADC1->JSQR = ADC_JSQR_JL_0 | (6U << ADC_JSQR_JSQ1_Pos) | (8U << ADC_JSQR_JSQ2_Pos);
    ADC2->JSQR = ADC_JSQR_JL_0 | (7U << ADC_JSQR_JSQ1_Pos) | (7U << ADC_JSQR_JSQ2_Pos);
    ADC1->ISR = 0xFFFFFFFFU;
    ADC2->ISR = 0xFFFFFFFFU;
    ADC1->CR |= ADC_CR_ADEN;
    ADC2->CR |= ADC_CR_ADEN;
    if (!foc_wait(&ADC1->ISR, ADC_ISR_ADRDY, true) || !foc_wait(&ADC2->ISR, ADC_ISR_ADRDY, true))
    {
        board_motor_stop("FOC ADC ENABLE");
        return false;
    }
    /* 正常输入零点采集已预热两路ADC；配置耗时受1ms空闲勘误约束。 */
    ADC1->SQR1 = 5U | (8U << ADC_SQR1_SQ1_Pos) | (1U << ADC_SQR1_SQ2_Pos) |
        (2U << ADC_SQR1_SQ3_Pos) | (3U << ADC_SQR1_SQ4_Pos);
    ADC1->SQR2 = (4U << ADC_SQR2_SQ5_Pos) | (18U << ADC_SQR2_SQ6_Pos);
    ADC1->SQR3 = ADC1->SQR4 = 0U;
    DMA1_Channel1->CCR = 0U;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    DMAMUX1_Channel0->CCR = FOC_DMA_REQUEST;
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)g_foc_slow;
    DMA1_Channel1->CNDTR = FOC_SLOW_ITEMS;
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |
        DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_EN;
    uint16_t compare = (uint16_t)(pwm_arr * duty / 1000U);
    /* 固定电角30/90/150/210/270/330度，只用于低能量相序及电流极性检查。 */
    static const uint8_t order[6][3]={{2,1,0},{1,2,0},{0,2,1},{0,1,2},{1,0,2},{2,0,1}};
    g_foc_preloaded = (foc_voltage_command_t){.sequence=0U, .applied_ticks=0U,
        .ccr={compare,compare,compare}, .bus_v=hw.bus_mv * 0.001f};
    if (sector != 0U)
    {
        for (unsigned i=0U;i<3U;++i)
        {
            int32_t ccr = (int32_t)compare + ((int32_t)order[sector-1U][i]-1) * (int32_t)spread;
            if (ccr < 425 || ccr > 8075) { board_motor_stop("FOC BENCH RANGE"); return false; }
            g_foc_preloaded.ccr[i]=(uint16_t)ccr;
        }
    }
    if (!foc_voltage_prepare(&g_foc_preloaded, (uint16_t)pwm_arr) ||
        !drv8353f_coast(mode==FOC_TEST_ZERO) || !drv8353f_verify(mode==FOC_TEST_ZERO))
    {
        board_motor_stop("FOC DRV/PWM");
        return false;
    }
    /* 功率禁用时丢弃双ADC首个软件注入，覆盖超过1ms空闲后的预热勘误。 */
    ADC1->CR |= ADC_CR_JADSTART;
    if (!foc_wait(&ADC1->ISR, ADC_ISR_JEOS, true) || !foc_wait(&ADC2->ISR, ADC_ISR_JEOS, true) ||
        !foc_wait(&ADC1->CR, ADC_CR_JADSTART, false))
    {
        board_motor_stop("FOC ADC WARMUP");
        return false;
    }
    ADC1->JSQR |= ADC_JSQR_JEXTEN_0 | (8U << ADC_JSQR_JEXTSEL_Pos);
    TIM1->CR1 = TIM_CR1_CMS_0 | TIM_CR1_ARPE;
    TIM1->CR2 = 7U << TIM_CR2_MMS2_Pos;
    TIM1->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
        (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (7U << TIM_CCMR2_OC4M_Pos);
    TIM1->CCMR3 = 0U;
    TIM1->ARR = pwm_arr;
    /* 先在本轮中心占空比校准开关状态下零点；此时不提交目标电压历史。 */
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = compare;
    TIM1->CCR4 = g_foc.trigger_ticks;
    /* 实际首事件由下面的无功率预同步确认，不能根据RCR静态值推断。 */
    TIM1->RCR = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CNT = 0U;
    TIM1->SR = 0U;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |
        TIM_CCER_CC3E | TIM_CCER_CC3NE;
#if MOTOR_ADC_TRACE_ENC_CAL
    TIM1->CCER |= TIM_CCER_CC4NE;
#endif
    if (!foc_prime_valley()) { board_motor_stop("FOC PRIME VALLEY"); return false; }
    if (mode!=FOC_TEST_ZERO && !foc_calibrate_pwm_zero()) { board_motor_stop("FOC PWM ZERO"); return false; }
    if(mode==FOC_TEST_CLOSED)
    {
        foc_drive_config_t config={.resistance_ohm=MOTOR_FOC_RESISTANCE_OHM,.inductance_h=MOTOR_FOC_INDUCTANCE_H,.current_limit_a=.3f,
            .voltage_error_bound_v=.12f,
            .emf_noise_v=g_foc.emf_noise_v,.current_bandwidth_hz=FOC_CURRENT_BANDWIDTH_HZ,
            .target_rpm=target,.pole_pairs=MOTOR_POLE_PAIRS,.speed_kp=.0002f,.speed_ki=.0005f,.speed_ramp_rpm_s=200};
        if(drive_config) { config=*drive_config; config.emf_noise_v=g_foc.emf_noise_v; }
#if MOTOR_ENCODER_ENABLED
        if(encoder_config)
        {
            if(!encoder_foc_init(&g_foc_encoder,encoder_config) || !board_encoder_control_begin())
            { board_motor_stop("ENCODER FOC CONFIG"); return false; }
            encoder_motion_init(&g_foc_motion,encoder_config->direction,FOC_TIMER_HZ);
            ++g_foc_motion_run;
        }
        else
#else
        (void)encoder_config;
#endif
        {
        config.startup_rpm=g_foc_handoff_rpm;
        config.extended_speed_range=true;
        if(!foc_drive_init(&g_foc_drive,&config)) { board_motor_stop("FOC DRIVE CONFIG"); return false; }
        g_foc_drive.harmonic_requested=g_foc.production&&g_foc_harmonic_requested;
        g_foc_drive.harmonic2_requested=g_foc.production&&g_foc_harmonic2_requested;
        foc_drive_start(&g_foc_drive);
        }
    }
    /* 正式计时前关断、停采、加载目标CCR并重新确认真实谷底。 */
    TIM1->CCR1 = g_foc_preloaded.ccr[0];
    TIM1->CCR2 = g_foc_preloaded.ccr[1];
    TIM1->CCR3 = g_foc_preloaded.ccr[2];
    if (!foc_prime_valley()) { board_motor_stop("FOC PRIME VALLEY"); return false; }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if ((TIM1->CR1 & TIM_CR1_DIR) || TIM1->CNT > FOC_COMMIT_GUARD)
    {
        board_motor_stop("FOC START PHASE");
        __set_PRIMASK(mask);
        return false;
    }
    ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC | ADC_ISR_OVR | ADC_ISR_JQOVF;
    ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC | ADC_ISR_OVR | ADC_ISR_JQOVF;
    ADC1->IER = ADC_IER_JEOSIE | ADC_IER_OVRIE | ADC_IER_JQOVFIE;
    ADC2->IER = ADC_IER_JEOSIE | ADC_IER_OVRIE | ADC_IER_JQOVFIE;
    /* 校准已完成，建立独立观测会话；不改变控制配置。 */
    uint16_t wave_zero[3]={g_foc.zero[0],g_foc.zero[1],g_foc.zero[2]};
    wave_stream_begin(wave_zero,g_foc.current_units,false);
    g_foc.active = true;
    g_foc_preloaded.applied_ticks = TIM1->CNT;
    (void)foc_voltage_commit(&g_foc_history, &g_foc_preloaded);
    ADC1->CR |= ADC_CR_JADSTART;
    TIM1->DIER = TIM_DIER_UIE | TIM_DIER_BIE;
    if(mode!=FOC_TEST_ZERO) { TIM1->BDTR |= TIM_BDTR_MOE; }
    g_foc.last_update_cycle = DWT->CYCCNT - TIM1->CNT;
    __set_PRIMASK(mask);
    return true;
}

/** @brief 原有等占空比入口保持独立，三相平均电压为零。 */
bool
board_foc_bench_start(uint32_t duty, uint32_t duration_ms)
{
    return foc_bench_configure(duty,duration_ms,0U,0U,FOC_TRIGGER_TICKS,FOC_TEST_STATIC,0U,NULL,NULL);
}

/** @brief 见board_foc.h；零点试验从配置到结束保持COAST且从不开启MOE。 */
bool
board_foc_zero_start(uint32_t duration_ms)
{
    if(duration_ms<20U || duration_ms>500U) { return false; }
    return foc_bench_configure(500U,duration_ms,0U,0U,FOC_TRIGGER_TICKS,FOC_TEST_ZERO,0U,NULL,NULL);
}

/** @brief 见board_foc.h；安全窗口内的采样平移仅在停机设置。 */
bool
board_foc_test_trigger(uint32_t trigger)
{
    if(trigger<8300U || trigger>8470U || board_motor_running() || g_foc.restore) { return false; }
    g_foc_run_trigger=trigger;
    return true;
}

/** @brief 主循环读取会话采样设置，不访问外设。 */
uint32_t
board_foc_test_trigger_ticks(void) { return g_foc_run_trigger; }

/** @brief 见board_foc.h；只能在功率和采样均停止时设置会话接管速度。 */
bool
board_foc_test_handoff(uint32_t rpm)
{
    if(rpm<1U || rpm>1000U || board_motor_running() || g_foc.active || g_foc.restore ||
        !board_motor_storage_safe()) { return false; }
    g_foc_handoff_rpm=rpm;
    return true;
}

/** @brief 查询已有会话设置，不访问运行中观察器。 */
uint32_t
board_foc_test_handoff_rpm(void) { return g_foc_handoff_rpm; }

/** @brief 20ms固定低电压矢量；沿用0.5A异常电流关断。 */
bool
board_foc_vector_start(uint32_t sector, uint32_t spread)
{
    if (sector < 1U || sector > 6U || spread < 1U || spread > 200U) { return false; }
    return foc_bench_configure(500U,20U,sector,spread,FOC_TRIGGER_TICKS,FOC_TEST_STATIC,0U,NULL,NULL);
}

/** @brief 在高占空比中央全低边界扫描保持位置，电流异常仍直接关断。 */
bool
board_foc_sample_start(uint32_t trigger)
{
    return foc_bench_configure(926U,500U,1U,200U,trigger,FOC_TEST_STATIC,0U,NULL,NULL);
}

/** @brief 927千分比中心7879加196恰为8075，实测最大调制的中央有效零矢量。 */
bool
board_foc_sample_vector_start(uint32_t trigger,uint32_t sector)
{
    if(sector<1U || sector>6U) { return false; }
    return foc_bench_configure(927U,500U,sector,196U,trigger,FOC_TEST_STATIC,0U,NULL,NULL);
}

/** @brief 正负小电压动态提交检查；不运行电流闭环或观测器。 */
bool
board_foc_switch_start(uint32_t spread, uint32_t duration_ms)
{
    if (spread < 1U || spread > 200U || duration_ms < 20U || duration_ms > 500U) { return false; }
    return foc_bench_configure(500U,duration_ms,1U,spread,FOC_TRIGGER_TICKS,FOC_TEST_SWITCH,0U,NULL,NULL);
}

/** @brief 低电流dq闭环阶跃，固定参考电角30度，不宣称实际转子角度。 */
bool
board_foc_current_start(bool q_axis, uint32_t target_ma, uint32_t duration_ms)
{
    if (target_ma<50U || target_ma>300U || duration_ms<20U || duration_ms>500U) { return false; }
    return foc_bench_configure(500U,duration_ms,0U,0U,FOC_TRIGGER_TICKS,q_axis?FOC_TEST_IQ:FOC_TEST_ID,target_ma,NULL,NULL);
}

/** @brief 候选模型下的有界低电流I/f测试，不启用角度接管或速度PI。 */
bool
board_foc_if_start(uint32_t frequency_millihz, uint32_t duration_ms)
{
    if (frequency_millihz<1000U || frequency_millihz>50000U || duration_ms<500U || duration_ms>5000U) { return false; }
    return foc_bench_configure(500U,duration_ms,0U,0U,FOC_TRIGGER_TICKS,FOC_TEST_IF,frequency_millihz,NULL,NULL);
}

/** @brief 已有低电流模型范围内独立观察SMO；强拖仍负责输出，不接管速度。 */
bool
board_foc_observe_start(uint32_t frequency_millihz,uint32_t duration_ms)
{
    if(frequency_millihz<1000U || frequency_millihz>50000U || duration_ms<500U || duration_ms>5000U) { return false; }
    return foc_bench_configure(500U,duration_ms,0U,0U,FOC_TRIGGER_TICKS,FOC_TEST_OBSERVE,frequency_millihz,NULL,NULL);
}

/** @brief 有界0.3A接管试验，仍保留0.5A异常关断及独立六步配置。 */
bool
board_foc_closed_start(uint32_t target_rpm,uint32_t duration_ms)
{
    if(target_rpm<1U || target_rpm>MOTOR_SPEED_MAX_RPM || duration_ms<500U || duration_ms>32000U) { return false; }
    return foc_bench_configure(500U,duration_ms,0U,0U,FOC_TRIGGER_TICKS,FOC_TEST_CLOSED,target_rpm,NULL,NULL);
}

/** @brief 800ms低电流定位后20ms升流、60ms保持、20ms退流；不得用空载转动代替量程试验。 */
bool
board_foc_range_start(uint32_t angle_slot,uint32_t target_ma)
{
    if(angle_slot<1U || angle_slot>6U || target_ma>MOTOR_FOC_LIMIT_MAX_MA ||
        (target_ma!=500U && target_ma!=1000U && target_ma!=1500U && target_ma!=2000U)) { return false; }
    return foc_bench_configure(500U,900U,angle_slot,0U,FOC_TRIGGER_TICKS,FOC_TEST_RANGE,target_ma,NULL,NULL);
}

/** @brief 正常START使用独立FOC参数；0.5A指令使用20倍/1.25A保护，其余10倍/2.5A。 */
bool
board_foc_speed_start(const foc_drive_config_t *config,uint32_t duration_ms)
{
    if(!config || config->pole_pairs!=MOTOR_POLE_PAIRS || config->current_limit_a<.5f || config->current_limit_a>MOTOR_FOC_LIMIT_MAX_MA*.001f ||
        fabsf(config->resistance_ohm-MOTOR_FOC_RESISTANCE_OHM)>.000001f || fabsf(config->inductance_h-MOTOR_FOC_INDUCTANCE_H)>.000000001f || config->current_bandwidth_hz!=FOC_CURRENT_BANDWIDTH_HZ ||
        config->voltage_error_bound_v<.12f || duration_ms<1000U) { return false; }
    return foc_bench_configure(500U,duration_ms,0U,0U,g_foc_run_trigger,FOC_TEST_CLOSED,config->target_rpm,config,NULL);
}

/** @brief 见board_foc.h；复用正式双ADC和连续SVPWM，首帧前保持零矢量。 */
bool
board_foc_encoder_start(const encoder_foc_config_t *config,uint32_t duration_ms)
{
#if MOTOR_ENCODER_ENABLED
    if(!config || config->drive.pole_pairs!=MOTOR_POLE_PAIRS ||
        config->drive.current_limit_a<.5f || config->drive.current_limit_a>MOTOR_FOC_LIMIT_MAX_MA*.001f ||
        fabsf(config->drive.resistance_ohm-MOTOR_FOC_RESISTANCE_OHM)>.000001f ||
        fabsf(config->drive.inductance_h-MOTOR_FOC_INDUCTANCE_H)>.000000001f ||
        config->drive.current_bandwidth_hz!=FOC_CURRENT_BANDWIDTH_HZ ||
        duration_ms<1000U) { return false; }
    return foc_bench_configure(500U,duration_ms,0U,0U,g_foc_run_trigger,FOC_TEST_CLOSED,
        config->drive.target_rpm,&config->drive,config);
#else
    (void)config; (void)duration_ms;
    return false;
#endif
}

/** @brief 目标单次对齐写；只有实际闭环后允许在线变化。 */
bool
board_foc_speed_target(uint32_t rpm)
{
    bool closed=g_foc_drive.stage==FOC_CLOSED;
#if MOTOR_ENCODER_ENABLED
    if(g_foc.encoder_mode) { closed=g_foc_encoder.closed_seen&&!g_foc_encoder.config.current_mode&&!g_foc_encoder.config.position_mode&&!g_foc_encoder.config.haptic_mode; }
#endif
    if(rpm<1U || rpm>MOTOR_SPEED_MAX_RPM || !board_foc_speed_active() || !closed) { return false; }
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    if(!g_foc.active) { __set_PRIMASK(mask); return false; }
#if MOTOR_ENCODER_ENABLED
    if(g_foc.encoder_mode) { g_foc_encoder.config.drive.target_rpm=rpm; }
    else
#endif
    { g_foc_drive.config.target_rpm=rpm; }
    __set_PRIMASK(mask);
    return true;
}

/** @brief 见board_foc.h；单一短临界区同时验证运行模式并写入有符号Iq目标。 */
bool
board_foc_iq_target(int32_t current_ma)
{
#if MOTOR_ENCODER_ENABLED
    if(current_ma<-500 || current_ma>500) { return false; }
    uint32_t mask=__get_PRIMASK();__disable_irq();
    bool accepted=g_foc.active&&g_foc.production&&g_foc.encoder_mode&&
        g_foc_encoder.config.current_mode&&g_foc_encoder.closed_seen;
    if(accepted) { g_foc_encoder.config.target_iq_a=current_ma*.001f; }
    __set_PRIMASK(mask);
    return accepted;
#else
    (void)current_ma;
    return false;
#endif
}

/** @brief 正常入口标志与功率占用同时有效，不把历史CLOSED当作运行。 */
bool
board_foc_speed_active(void) { return g_foc.active&&g_foc.production; }

/** @brief 见board_foc.h；固定Iq只在主循环短临界区内改变速度环开关。 */
bool
board_foc_hold_iq(uint32_t current_ma)
{
    if(current_ma>500U || !board_foc_speed_active()) { return false; }
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    bool accepted=g_foc.active&&!g_foc.encoder_mode&&foc_drive_hold_iq(&g_foc_drive,current_ma*.001f);
    __set_PRIMASK(mask);
    return accepted;
}

/** @brief 在同一短临界区切换请求，二/五次共用原阶段门控。
 * @param second 是否选择二次，否则五次。 @param enabled 启用请求。
 * @return 阶段允许并已设置。 */
static bool
foc_harmonic_select(bool second,bool enabled)
{
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    bool accepted=!g_foc.active || (g_foc.production&&!g_foc.encoder_mode&&g_foc_drive.stage==FOC_CLOSED);
    if (accepted)
    {
        if (second)
        {
            g_foc_harmonic2_requested=enabled;
            if (g_foc.production) { g_foc_drive.harmonic2_requested=enabled; }
        }
        else
        {
            g_foc_harmonic_requested=enabled;
            if (g_foc.production) { g_foc_drive.harmonic_requested=enabled; }
        }
    }
    __set_PRIMASK(mask);
    return accepted;
}

/** @brief 见board_foc.h；保持原五次接口。 */
bool
board_foc_harmonic(bool enabled)
{
    return foc_harmonic_select(false,enabled);
}

/** @brief 见board_foc.h；只更新二次请求。 */
bool
board_foc_harmonic2(bool enabled)
{
    return foc_harmonic_select(true,enabled);
}

/** @brief 主循环快照及共用格式化，实际ACTIVE来自实时支路状态。
 * @param out 响应缓冲。 @param size 字节数。 @param second 是否二次。 */
static void
foc_harmonic_diagnostics(char *out,size_t size,bool second)
{
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    bool requested=second?g_foc_harmonic2_requested:g_foc_harmonic_requested;
    bool active=g_foc.active&&g_foc.production&&!g_foc.encoder_mode&&
        (second?g_foc_drive.current.harmonic2.enabled:g_foc_drive.current.harmonic_enabled);
    float x=second?g_foc_drive.current.harmonic2.x_v:g_foc_drive.current.harmonic_x_v;
    float y=second?g_foc_drive.current.harmonic2.y_v:g_foc_drive.current.harmonic_y_v;
    uint32_t limited=second?g_foc_drive.current.harmonic2.limited:g_foc_drive.current.harmonic_limited;
    bool seventh=g_foc.active&&g_foc.production&&!g_foc.encoder_mode&&g_foc_drive.current.harmonic7.enabled;
    __set_PRIMASK(mask);
    snprintf(out,size,"OK MOTOR FOC %s REQUESTED=%u ACTIVE=%u ORDER=%d KI_V_AS=20 LIMIT_MV=%u X_MV=%ld Y_MV=%ld LIMITED=%lu H7_ACTIVE=%u",
        second?"HARMONIC2":"HARMONIC",requested?1U:0U,active?1U:0U,second?-2:-5,
        second?100U:200U,(long)(x*1000),(long)(y*1000),(unsigned long)limited,seventh?1U:0U);
}

/** @brief 见board_foc.h；保持五次查询字段与单位。 */
void
board_foc_harmonic_diagnostics(char *out,size_t size)
{
    foc_harmonic_diagnostics(out,size,false);
}

/** @brief 见board_foc.h；二次使用相同快照和主循环格式化。 */
void
board_foc_harmonic2_diagnostics(char *out,size_t size)
{
    foc_harmonic_diagnostics(out,size,true);
}

/** @brief 显示只读最新状态；电压是上一采样区间真实平均值，不是新请求。 */
void FOC_DIAGNOSTIC_CODE
board_foc_runtime(board_foc_runtime_t *out)
{
    memset(out,0,sizeof(*out));
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    bool active=board_foc_speed_active();
#if MOTOR_ENCODER_ENABLED
    if(g_foc.encoder_mode)
    {
        out->encoder=true;
        out->current_mode=g_foc_encoder.config.current_mode;
        out->target_iq_ma=(int32_t)lroundf(g_foc_encoder.config.target_iq_a*1000.0f);
        out->closed=active&&g_foc_encoder.closed_seen;
        out->stage=!active?FOC_STOPPED:out->closed?FOC_CLOSED:FOC_CALIBRATE;
        out->target_rpm=(out->current_mode||g_foc_encoder.config.position_mode||g_foc_encoder.config.haptic_mode)?0U:g_foc_encoder.config.drive.target_rpm;
        out->samples_valid=active&&g_foc.samples>0U;
        out->tracking=out->samples_valid&&g_foc_encoder.speed_valid;
        out->bus_mv=g_foc.bus_mv;
        uint32_t raw=g_foc_encoder.raw,vdda=g_foc.vdda_mv,units=g_foc.current_units;
        int32_t a=(int32_t)g_foc.raw[0]-g_foc.zero[0],b=(int32_t)g_foc.raw[1]-g_foc.zero[1];
        float id=g_foc_encoder.current.id_a,iq=g_foc_encoder.current.iq_a;
        float rpm=g_foc_encoder.rpm,theta=g_foc_encoder.sample_theta_rad;
        float d=g_foc_encoder.current.command_d_v,q=g_foc_encoder.current.command_q_v;
        __set_PRIMASK(mask);
        out->phase_ma[0]=a*(int32_t)vdda*(int32_t)units/4095;
        out->phase_ma[1]=b*(int32_t)vdda*(int32_t)units/4095;
        out->phase_ma[2]=-out->phase_ma[0]-out->phase_ma[1];
        out->mechanical_angle_decideg=(int32_t)((uint64_t)raw*3600U/ENCODER_SIX_TURN);
        out->relative_angle_decideg=(int32_t)(theta*572.95779513f);
        if(out->relative_angle_decideg<0) { out->relative_angle_decideg+=3600; }
        out->estimated_rpm=(int32_t)rpm;
        out->id_ma=(int32_t)(id*1000);out->iq_ma=(int32_t)(iq*1000);
        out->ud_mv=(int32_t)(d*1000);out->uq_mv=(int32_t)(q*1000);
        return;
    }
#endif
    out->stage=g_foc_drive.stage;
    out->target_rpm=g_foc_drive.config.target_rpm;
    out->closed=active&&out->stage==FOC_CLOSED;
    out->samples_valid=active&&g_foc.samples>0U;
    out->tracking=out->samples_valid&&g_foc_drive.observer.pll_initialized&&g_foc_drive.observer.emf_magnitude_v>=.1f;
    out->bus_mv=g_foc.bus_mv;
    uint32_t vdda=g_foc.vdda_mv,units=g_foc.current_units;
    int32_t a=(int32_t)g_foc.raw[0]-g_foc.zero[0],b=(int32_t)g_foc.raw[1]-g_foc.zero[1];
    float id=g_foc_drive.current.id_a,iq=g_foc_drive.current.iq_a;
    float omega=g_foc_drive.observer.omega_rad_s,force=g_foc_drive.force_omega_rad_s;
    float theta=g_foc_drive.observer.sample_theta_rad,control_theta=g_foc_drive.control_theta_rad;
    float alpha_v=g_foc_interval.alpha_v,beta_v=g_foc_interval.beta_v;
    __set_PRIMASK(mask);
    out->phase_ma[0]=a*(int32_t)vdda*(int32_t)units/4095;
    out->phase_ma[1]=b*(int32_t)vdda*(int32_t)units/4095;
    out->phase_ma[2]=-out->phase_ma[0]-out->phase_ma[1];
    out->id_ma=(int32_t)(id*1000);
    out->iq_ma=(int32_t)(iq*1000);
    out->estimated_rpm=(int32_t)(omega*MOTOR_FOC_RPM_PER_RAD_S);
    out->forced_millihz=(int32_t)(force*159.154943092f);
    out->relative_angle_decideg=(int32_t)(theta*572.95779513f);
    if(out->relative_angle_decideg<0) { out->relative_angle_decideg+=3600; }
    if(out->relative_angle_decideg>=3600) { out->relative_angle_decideg-=3600; }
    float cosine,sine;
    foc_sincos(control_theta,&cosine,&sine);
    out->ud_mv=(int32_t)((alpha_v*cosine+beta_v*sine)*1000);
    out->uq_mv=(int32_t)((-alpha_v*sine+beta_v*cosine)*1000);
}

/** @brief 生成定位和强拖指令并预测下一PWM作用角度；不从强拖频率伪造估速。
 * @param c 采样时刻余弦。 @param s 采样时刻正弦。
 * @param future_c 未来PWM中心余弦。 @param future_s 未来PWM中心正弦。
 * @param reference_d Id指令A。 @param reference_q Iq指令A。 */
static void MOTOR_FAST_CODE
foc_if_reference(float *c, float *s, float *future_c, float *future_s, float *reference_d, float *reference_q)
{
    uint32_t cycle=g_foc.cycle;
    if (cycle<2000U)
    {
        *reference_d=cycle<1000U?.2f*cycle/1000.0f:.2f;
        *reference_q=0.0f;
    }
    else
    {
        float previous=g_foc.force_omega;
        float omega=previous+0.0125663706144f;
        if (omega>g_foc.force_target_omega) { omega=g_foc.force_target_omega; }
        float theta=g_foc.force_theta+(previous+omega)*.00005f;
        if (theta>=6.28318530718f) { theta-=6.28318530718f; }
        g_foc.force_theta=theta;
        g_foc.force_omega=omega;
        /* 50ms转换电流矢量，避免定位结束瞬间将磁场跳转90电角。 */
        float fraction=cycle-2000U<500U?(cycle-2000U)/500.0f:1.0f;
        *reference_d=.2f*(1.0f-fraction);
        *reference_q=g_foc.current_target_a*fraction;
    }
    foc_sincos(g_foc.force_theta,c,s);
    float delta=g_foc.force_omega*(1.5f*FOC_PERIOD_TICKS-g_foc.trigger_ticks-FOC_HOLD_TICKS)/FOC_TIMER_HZ;
    /* 本轮0～50Hz令|delta|<=0.032rad，六阶小角展开误差远小于单精度舍入。 */
    float delta2=delta*delta;
    float ds=delta*(1.0f-delta2/6.0f+delta2*delta2/120.0f);
    float dc=1.0f-delta2/2.0f+delta2*delta2/24.0f;
    *future_c=*c*dc-*s*ds;
    *future_s=*s*dc+*c*ds;
}

/** @brief 谷底确认命令生效，时间采用硬件周期位置而不是ISR进入时刻。 */
void MOTOR_FAST_CODE
board_foc_update_irq(void)
{
    uint32_t begun = DWT->CYCCNT;
    TIM1->SR = ~TIM_SR_UIF;
    ++g_foc.updates;
    const char *reason = (TIM1->CR1 & TIM_CR1_DIR) ? "FOC TIM PHASE" :
        TIM1->CNT > FOC_COMMIT_GUARD ? "FOC TIM LATE" :
        (g_foc.samples != g_foc.updates ||
         g_foc.adc1_count != g_foc.updates*(g_foc.production?2U:1U) ||
         g_foc.adc2_count != g_foc.adc1_count) ? "FOC ADC MISSING" :
        DWT->CYCCNT - g_foc.last_update_cycle > FOC_PERIOD_TICKS + FOC_PERIOD_TICKS/2U ? "FOC TIM GAP" : NULL;
    if (NULL != reason)
    {
        g_foc.fault_cnt = TIM1->CNT;
        g_foc.fault_cr1 = TIM1->CR1;
        g_foc.fault_adc1 = ADC1->ISR;
        g_foc.fault_adc2 = ADC2->ISR;
        g_foc.fault_jsqr = ADC1->JSQR;
        g_foc.fault_adc_cr = ADC1->CR;
        board_motor_stop(reason);
        return;
    }
    /* DWT与TIM1同为170MHz；锚定真实谷底，避免把ISR进入延迟加入截止预算。 */
    g_foc.last_update_cycle = begun - TIM1->CNT;
    ++g_foc.cycle;
    g_foc_preloaded.applied_ticks = g_foc.cycle * FOC_PERIOD_TICKS;
    if (!foc_voltage_commit(&g_foc_history, &g_foc_preloaded))
    {
        board_motor_stop("FOC PWM HISTORY");
        return;
    }
    if(g_foc.test_mode==FOC_TEST_CLOSED)
    {
#if MOTOR_ENCODER_ENABLED
        if(g_foc.encoder_mode) { encoder_foc_commit(&g_foc_encoder,g_foc_history.current.sequence); }
        else
#endif
        { foc_drive_commit(&g_foc_drive,g_foc_history.current.sequence,g_foc_history.current.estimated_angle); }
    }
    if (--g_foc.remaining_cycles == 0U)
    {
        board_motor_stop(NULL);
        return;
    }
    if (g_foc.cycle % 10U == 0U)
    {
        ++g_foc.speed_checks;
        if(g_foc.test_mode==FOC_TEST_CLOSED)
        {
#if MOTOR_ENCODER_ENABLED
            if(g_foc.encoder_mode)
            {
                /* ADC优先级高于TIM，只有位移/时间的交换需要屏蔽抢占；浮点PI在临界区外。 */
                uint32_t mask=__get_PRIMASK();__disable_irq();
                encoder_foc_speed_window_t window=encoder_foc_take_speed(&g_foc_encoder);
                uint32_t raw=g_foc_encoder.raw,tick=g_foc_encoder.feedback_tick;
                bool feedback_seen=g_foc_encoder.feedback_seen;
                if(g_foc_encoder.config.position_mode)
                {
                    encoder_position_t position=board_encoder_position();
                    g_foc_encoder.position_counts=encoder_position_counts(&position);
                    g_foc_encoder.position_valid=position.valid&&position.zero_valid;
                }
                __set_PRIMASK(mask);
                if(feedback_seen) { encoder_motion_update(&g_foc_motion,raw,tick); }
                encoder_foc_speed_tick(&g_foc_encoder,g_foc_encoder.config.drive.target_rpm,window);
                if(!g_foc_encoder.running) { board_motor_stop(g_foc_encoder.fault); return; }
            }
            else
#endif
            { foc_drive_speed_tick(&g_foc_drive,g_foc_drive.config.target_rpm); }
        }
        if (HAL_GetTick() - g_foc.bus_ms > MOTOR_BUS_STALE_MS)
        {
            board_motor_stop("FOC BUS STALE");
            return;
        }
    }
    uint32_t elapsed = DWT->CYCCNT - begun;
    if (elapsed > g_foc.update_max) { g_foc.update_max = elapsed; }
}

/** @brief 同时读双路电流，先加权实际电压，再准备未来命令。 */
void MOTOR_FAST_CODE FOC_DIAGNOSTIC_CODE
board_foc_adc_irq(void)
{
    uint32_t begun = DWT->CYCCNT;
    uint32_t flags1 = ADC1->ISR, flags2 = ADC2->ISR;
    if ((flags1 | flags2) & (ADC_ISR_OVR | ADC_ISR_JQOVF))
    {
        board_motor_stop("FOC ADC OVERRUN");
        return;
    }
    if (!(flags1 & ADC_ISR_JEOS) || !(flags2 & ADC_ISR_JEOS)) { return; }
    uint16_t raw[3]={(uint16_t)ADC1->JDR1,(uint16_t)ADC2->JDR1,(uint16_t)ADC1->JDR2};
    ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    ++g_foc.adc1_count;
    ++g_foc.adc2_count;
    if ((TIM1->CR1 & TIM_CR1_DIR) == 0U && TIM1->CNT < g_foc.trigger_ticks)
    {
        board_motor_stop("FOC ADC PHASE");
        return;
    }
    uint32_t carrier_ticks=g_foc.production?FOC_PERIOD_TICKS/2U:FOC_PERIOD_TICKS;
    uint32_t elapsed_ticks = (TIM1->CR1 & TIM_CR1_DIR) ? carrier_ticks - TIM1->CNT : TIM1->CNT;
    if (elapsed_ticks < g_foc.trigger_ticks || elapsed_ticks >= carrier_ticks - FOC_COMMIT_GUARD)
    {
        board_motor_stop("FOC ADC DEADLINE");
        return;
    }
    uint32_t age = elapsed_ticks - g_foc.trigger_ticks - FOC_HOLD_TICKS;
    if (age > g_foc.age_max) { g_foc.age_max = age; }
    int32_t delta[3] = {(int32_t)raw[0] - g_foc.zero[0], (int32_t)raw[1] - g_foc.zero[1], 0};
    delta[2] = -delta[0] - delta[1];
    int32_t dc = (int32_t)raw[2] - g_foc.zero[2];
    int32_t sum = delta[0]+delta[1]+dc;
    /* 第三保持点比第一组晚19个ADC周期；0.2A级误差不能当作正常零矢量样本。 */
    if (sum < -128 || sum > 128 || raw[2] < 16U || raw[2] > 4079U ||
        dc < -(int32_t)g_foc.trip_raw || dc > (int32_t)g_foc.trip_raw)
    {
        board_motor_stop("FOC CURRENT SUM");
        return;
    }
    for (unsigned i=0U; i<3U; ++i)
    {
        uint32_t magnitude = (uint32_t)(delta[i] < 0 ? -delta[i] : delta[i]);
        if (magnitude > g_foc.peak_raw) { g_foc.peak_raw = magnitude; }
        if (magnitude > g_foc.trip_raw || (i < 2U && (raw[i] < 16U || raw[i] > 4079U)))
        {
            board_motor_stop("FOC BENCH CURRENT");
            return;
        }
    }
#if MOTOR_ENCODER_ENABLED
    /* 每个50us保护样本推进SPI1 DMA，四相流水线仍为5kHz；无等待或额外中断。 */
    encoder_six_feedback_t encoder_feedback={0};
    if(g_foc.encoder_mode) { encoder_feedback=board_encoder_control_tick(); }
#endif
    if (g_foc.production)
    {
        uint32_t phase=(g_foc.adc1_count-1U)&1U;
        /* 两路ADC每50us都检查量程/过流；奇偶相位由真实谷底和计数互相校验。 */
        if (g_foc.adc1_count != 2U*g_foc.cycle+1U+phase ||
            (begun-g_foc.last_update_cycle)/carrier_ticks != phase)
        { board_motor_stop("FOC ADC PAIR"); return; }
        if (phase)
        {
            uint32_t elapsed=DWT->CYCCNT-begun;
            if(elapsed>g_foc.adc_max) { g_foc.adc_max=elapsed; }
            return;
        }
    }
    /* 只发布与10kHz观察器配对的主样本；中间保护样本不覆盖打印/电压配对状态。 */
    g_foc.raw[0]=raw[0]; g_foc.raw[1]=raw[1]; g_foc.raw[2]=raw[2];
    uint32_t sample = g_foc.cycle * FOC_PERIOD_TICKS + g_foc.trigger_ticks + FOC_HOLD_TICKS;
    if (g_foc.samples != 0U)
    {
        if (sample - g_foc.sample_ticks != FOC_PERIOD_TICKS ||
            !foc_voltage_average(&g_foc_history, g_foc.sample_ticks, sample, &g_foc_interval))
        {
            board_motor_stop("FOC SAMPLE HISTORY");
            return;
        }
        ++g_foc.intervals;
    }
    g_foc.sample_ticks = sample;
    ++g_foc.samples;
    if (g_foc.samples != g_foc.cycle + 1U) { board_motor_stop("FOC ADC PAIR"); return; }
    /* 禁止输出的零点试验不运行电流PI或SMO，保留同样的配对、缺样和提交时序检查。 */
    if(g_foc.test_mode==FOC_TEST_ZERO)
    {
        g_foc_preloaded.sequence=g_foc_history.current.sequence+1U;
        /* 禁止输出仍须更新真实母线/参考，不能因跳过电流环而丢掉慢速采样和过期保护。 */
        if (!g_foc.slow_busy && g_foc.samples % 10U == 1U)
        {
            g_foc.slow_busy=true;
            ADC1->ISR=ADC_ISR_EOS | ADC_ISR_OVR;
            ADC1->CR |= ADC_CR_ADSTART;
        }
        uint32_t elapsed=DWT->CYCCNT-begun;
        if(elapsed>g_foc.adc_max) { g_foc.adc_max=elapsed; }
        return;
    }
    float scale=g_foc.vdda_mv*(.001f/4095.0f)*g_foc.current_units;
    float alpha_a=delta[0]*scale;
    float beta_a=(delta[0]+2*delta[1])*scale*0.57735026919f;
    if(g_foc.test_mode==FOC_TEST_OBSERVE &&
        !foc_smo_step(&g_foc_smo,g_foc.samples==1U?NULL:&g_foc_interval,alpha_a,beta_a))
    {
        board_motor_stop("FOC SMO INVALID");
        return;
    }
    if(g_foc.test_mode==FOC_TEST_RANGE && g_foc.cycle<8000U)
    {
        /* 只在已识别的低电流范围观察静止；高电流段不借用未验收的观察器模型。 */
        if(!foc_smo_step(&g_foc_smo,g_foc.samples==1U?NULL:&g_foc_interval,alpha_a,beta_a))
        { board_motor_stop("FOC RANGE OBSERVER"); return; }
        if(g_foc.cycle>=4000U && g_foc_smo.emf_magnitude_v<.1f && g_foc_smo.residual_a<.05f)
        { ++g_foc.range_still_cycles; }
        else { g_foc.range_still_cycles=0U; }
    }
    /* 先完成上一电流区间的电压加权，之后才生成未来请求。 */
    g_foc_preloaded.sequence = g_foc_history.current.sequence + 1U;
    if (g_foc.test_mode == FOC_TEST_SWITCH)
    {
        int32_t signed_spread = (g_foc_preloaded.sequence / FOC_SWITCH_HOLD_CYCLES) % 2U ?
            -(int32_t)g_foc.spread : (int32_t)g_foc.spread;
        g_foc_preloaded.ccr[0] = (uint16_t)((int32_t)g_foc.center + signed_spread);
        g_foc_preloaded.ccr[1] = g_foc.center;
        g_foc_preloaded.ccr[2] = (uint16_t)((int32_t)g_foc.center - signed_spread);
    }
    g_foc_preloaded.bus_v = g_foc.bus_mv * 0.001f;
    if(g_foc.test_mode==FOC_TEST_CLOSED)
    {
        float future_s=(1.5f*FOC_PERIOD_TICKS-g_foc.trigger_ticks-FOC_HOLD_TICKS)/FOC_TIMER_HZ;
#if MOTOR_ENCODER_ENABLED
        if(g_foc.encoder_mode)
        {
            uint32_t sample_dwt=g_foc.last_update_cycle+g_foc.trigger_ticks+FOC_HOLD_TICKS;
            if(!encoder_foc_sample(&g_foc_encoder,&encoder_feedback,DWT->CYCCNT,sample_dwt,alpha_a,beta_a,
                g_foc_preloaded.bus_v,g_foc_preloaded.sequence,future_s,g_foc_preloaded.ccr))
            { board_motor_stop(g_foc_encoder.fault); return; }
            g_foc_preloaded.estimated_angle=false;
        }
        else
#endif
        {
        if(!foc_drive_sample(&g_foc_drive,g_foc.samples==1U?NULL:&g_foc_interval,alpha_a,beta_a,
            g_foc_preloaded.bus_v,g_foc_preloaded.sequence,future_s,g_foc_preloaded.ccr))
        { board_motor_stop(g_foc_drive.fault); return; }
        g_foc_preloaded.estimated_angle=g_foc_drive.request_estimated_angle;
        }
    }
    else if (g_foc.test_mode >= FOC_TEST_ID)
    {
        float reference=g_foc.cycle/FOC_CURRENT_HOLD_CYCLES%2U ? 0.0f : g_foc.current_target_a;
        float reference_d=g_foc.test_mode==FOC_TEST_ID?reference:0.0f;
        float reference_q=g_foc.test_mode==FOC_TEST_IQ?reference:0.0f;
        float c=.86602540378f,s=.5f,future_c=c,future_s=s;
        if(g_foc.test_mode==FOC_TEST_RANGE)
        {
            c=future_c=g_foc.range_cos;
            s=future_s=g_foc.range_sin;
            reference_q=0;
            if(g_foc.cycle<8000U)
            { reference_d=.2f*fminf(1.0f,g_foc.cycle*.001f); }
            else
            {
                if(g_foc.range_still_cycles<1000U)
                { board_motor_stop("FOC RANGE MOVING"); return; }
                g_foc.trip_raw=g_foc.range_trip_raw;
                uint32_t n=g_foc.cycle-8000U;
                float fraction=n<200U?n*.005f:n<800U?1.0f:(1000U-n)*.005f;
                reference_d=.2f+(g_foc.current_target_a-.2f)*fraction;
            }
        }
        if (g_foc.test_mode>=FOC_TEST_IF) { foc_if_reference(&c,&s,&future_c,&future_s,&reference_d,&reference_q); }
        if (!foc_current_step(&g_foc_current,alpha_a,beta_a,c,s,future_c,future_s,
            g_foc_preloaded.bus_v,reference_d,reference_q,g_foc_preloaded.ccr))
        {
            board_motor_stop("FOC CURRENT VALUE");
            return;
        }
    }
    if (g_foc.production)
    {
        /* 算法的归一化8500比较值映射到4250载波；模型只用量化后实际CCR。 */
        for (unsigned i=0U;i<3U;++i) { g_foc_preloaded.ccr[i]/=2U; }
    }
    if (!foc_voltage_prepare(&g_foc_preloaded, g_foc.production?FOC_PWM20_ARR:FOC_ARR))
    { board_motor_stop("FOC PWM VALUE"); return; }
    uint32_t elapsed=DWT->CYCCNT-g_foc.last_update_cycle;
    uint32_t margin = elapsed<FOC_PERIOD_TICKS?FOC_PERIOD_TICKS-elapsed:0U;
    if (margin < FOC_COMMIT_GUARD) { board_motor_stop("FOC CCR DEADLINE"); return; }
    TIM1->CCR1 = g_foc_preloaded.ccr[0];
    TIM1->CCR2 = g_foc_preloaded.ccr[1];
    TIM1->CCR3 = g_foc_preloaded.ccr[2];
    elapsed=DWT->CYCCNT-g_foc.last_update_cycle;
    margin = elapsed<FOC_PERIOD_TICKS?FOC_PERIOD_TICKS-elapsed:0U;
    if (margin < FOC_COMMIT_GUARD) { board_motor_stop("FOC CCR DEADLINE"); return; }
    if (margin < g_foc.margin_min) { g_foc.margin_min = margin; }
    /* 控制计算后启动31us慢速序列；20kHz中间注入可抢占，双ADC保护仍优先。 */
    if (!g_foc.slow_busy && g_foc.samples % 10U == 1U)
    {
        g_foc.slow_busy = true;
        ADC1->ISR = ADC_ISR_EOS | ADC_ISR_OVR;
        ADC1->CR |= ADC_CR_ADSTART;
    }
    if(wave_stream_enabled())
    {
        uint32_t capture_started=DWT->CYCCNT;
        const foc_current_t *observed=&g_foc_current;
        if(g_foc.test_mode==FOC_TEST_CLOSED) { observed=&g_foc_drive.current; }
#if MOTOR_ENCODER_ENABLED
        if(g_foc.encoder_mode) { observed=&g_foc_encoder.current; }
#endif
        wave_sample_t row={.sequence=g_foc.samples,.ticks=g_foc.sample_ticks,
            .raw={raw[0],raw[1],raw[2]},.vdda_mv=g_foc.vdda_mv,
            .id_a=observed->id_a,.iq_a=observed->iq_a,
            .reference_d_a=observed->reference_d,.reference_q_a=observed->reference_q};
        wave_stream_capture(&row);
        wave_stream_capture_timing(DWT->CYCCNT-capture_started);
    }
    elapsed = DWT->CYCCNT - begun;
    if (elapsed > g_foc.adc_max) { g_foc.adc_max = elapsed; }
}

/** @brief 慢速DMA完成后校核参考和母线，避免继续使用过期电压。 */
void
board_foc_dma_irq(void)
{
    uint32_t flags = DMA1->ISR;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    if (flags & DMA_ISR_TEIF1) { board_motor_stop("FOC ADC DMA"); return; }
    if (flags & DMA_ISR_TCIF1)
    {
        if (g_foc_slow[5] == 0U) { board_motor_stop("FOC VREF"); return; }
        uint32_t vdda = 3000U * *(const uint16_t *)0x1FFF75AAUL / g_foc_slow[5];
        uint32_t bus = (uint32_t)g_foc_slow[1] * vdda * 13U / 4095U;
        if (vdda < 2700U || vdda > 3600U || bus < MOTOR_BUS_MIN_MV || bus > MOTOR_BUS_MAX_MV)
        {
            board_motor_stop("FOC VDDA/BUS");
            return;
        }
        g_foc.vdda_mv = (uint16_t)vdda;
        g_foc.bus_mv = (uint16_t)bus;
        g_foc.bus_ms = HAL_GetTick();
        ++g_foc.slow_count;
        g_foc.slow_busy = false;
    }
}

/** @brief 主循环短快照后格式化，输出均为实际测试数据或明确未验收标志。 */
void FOC_DIAGNOSTIC_CODE
board_foc_diagnostics(char *out, size_t size, bool detail)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    foc_board_state_t s = g_foc;
    bool current_mode=false;
#if MOTOR_ENCODER_ENABLED
    current_mode=s.encoder_mode&&g_foc_encoder.config.current_mode;
#endif
    foc_voltage_interval_t interval = g_foc_interval;
    foc_voltage_command_t command = g_foc_history.current;
    __set_PRIMASK(mask);
    if (detail)
    {
        snprintf(out,size,"OK MOTOR FOC VOLTAGE OLD=%lu NEW=%lu DT_TICKS=%lu NEW_TICKS=%lu SAMPLE_TICKS=%lu LATEST_SEQ=%lu LATEST_APPLIED_TICKS=%lu LATEST_CCR=%u,%u,%u UA_MV=%ld UB_MV=%ld MODEL=PWM_COMMAND_AVERAGE",
            (unsigned long)interval.old_sequence,(unsigned long)interval.new_sequence,
            (unsigned long)interval.duration_ticks,(unsigned long)interval.new_ticks,
            (unsigned long)s.sample_ticks,(unsigned long)command.sequence,(unsigned long)command.applied_ticks,
            command.ccr[0],command.ccr[1],command.ccr[2],
            (long)(interval.alpha_v*1000.0f),(long)(interval.beta_v*1000.0f));
    }
    else
    {
        uint32_t peak_ma = s.vdda_mv == 0U ? 0U : (uint32_t)((uint64_t)s.peak_raw * s.vdda_mv * s.current_units / 4095U);
        snprintf(out,size,"OK MOTOR FOC STAGE=%s ACTIVE=%u PWM_HZ=%lu CTRL_HZ=10000 SAMPLE_HZ=%lu SAMPLES=%lu ADC1=%lu ADC2=%lu UEV=%lu INTERVALS=%lu SLOW=%lu BUS_MV=%u RAW=%u,%u,%u ZERO=%u,%u,%u PEAK_MA=%lu ADC_CYCLES=%lu TIM_CYCLES=%lu AGE_TICKS=%lu MARGIN_TICKS=%lu TRIGGER=%lu CSA_GAIN=%lu CSA_VERIFIED=0 FAULT=%s",
            s.encoder_mode?(current_mode?"ENCODER_IQ":"ENCODER_SPEED"):s.production?"SENSORLESS_SPEED":s.test_mode==FOC_TEST_RANGE?"CURRENT_RANGE":"TIMING_BENCH",s.active?1U:0U,(unsigned long)(s.production?20000U:10000U),(unsigned long)(s.production?20000U:10000U),(unsigned long)s.samples,(unsigned long)s.adc1_count,(unsigned long)s.adc2_count,
            (unsigned long)s.updates,(unsigned long)s.intervals,(unsigned long)s.slow_count,s.bus_mv,s.raw[0],s.raw[1],s.raw[2],s.zero[0],s.zero[1],s.zero[2],
            (unsigned long)peak_ma,(unsigned long)s.adc_max,(unsigned long)s.update_max,
            (unsigned long)s.age_max,(unsigned long)s.margin_min,(unsigned long)s.trigger_ticks,
            (unsigned long)(20U/s.current_units),NULL==s.fault?"NONE":s.fault);
    }
}

/** @brief 主循环复制真实电流环状态，固定角度基准与模型候选标志明确输出。 */
void FOC_DIAGNOSTIC_CODE
board_foc_current_diagnostics(char *out, size_t size)
{
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    foc_current_t s=g_foc.test_mode==FOC_TEST_CLOSED?g_foc_drive.current:g_foc_current;
    foc_test_mode_t mode=g_foc.test_mode;
    float force_omega=g_foc.test_mode==FOC_TEST_CLOSED?g_foc_drive.force_omega_rad_s:g_foc.force_omega;
    /* 同一短临界区取得速度偏置与扰动；它们不是电流PI积分电压。 */
    float speed_integral=0,disturbance_rpm_s=0,load_a=0,velocity=0;
    bool low_position=false;
#if MOTOR_ENCODER_ENABLED
    if(g_foc.encoder_mode)
    {
        s=g_foc_encoder.current; force_omega=0;
        speed_integral=g_foc_encoder.integral_a;
        disturbance_rpm_s=g_foc_encoder.low_disturbance_rpm_s;
        load_a=g_foc_encoder.low_load_previous_a;
        velocity=g_foc_encoder.low_velocity_rpm;
        low_position=g_foc_encoder.low_position_active;
    }
#endif
    bool encoder=g_foc.encoder_mode;
    __set_PRIMASK(mask);
    snprintf(out,size,"OK MOTOR FOC CURRENT AXIS=%s ANGLE=%s FORCE_MHZ=%ld FORCE_RPM=%ld MODEL=CANDIDATE RS_MOHM=%lu LS_UH=%lu BW_HZ=%lu ID_MA=%ld IQ_MA=%ld REF_D_MA=%ld REF_Q_MA=%ld INT_D_MV=%ld INT_Q_MV=%ld LIMITED=%lu",
        mode==FOC_TEST_CLOSED?"DQ":mode==FOC_TEST_RANGE?"RANGE":mode==FOC_TEST_ID?"D":mode==FOC_TEST_IQ?"Q":mode>=FOC_TEST_IF?"IF":"OFF",
        encoder?"MT6835":mode==FOC_TEST_RANGE?"FIXED_RANGE":mode==FOC_TEST_CLOSED?"DRIVE_STATE":mode>=FOC_TEST_IF?"IF_FORCED":"FIXED_30E",(long)(force_omega*159.154943092f),(long)(force_omega*MOTOR_FOC_RPM_PER_RAD_S),
        (unsigned long)MOTOR_FOC_RESISTANCE_MOHM,(unsigned long)MOTOR_FOC_INDUCTANCE_UH,(unsigned long)MOTOR_FOC_BANDWIDTH_HZ,
        (long)(s.id_a*1000),(long)(s.iq_a*1000),(long)(s.reference_d*1000),(long)(s.reference_q*1000),
        (long)(s.integral_d*1000),(long)(s.integral_q*1000),(unsigned long)s.limited);
    if(encoder)
    {
        size_t used=strlen(out);
        if(used<size)
        { snprintf(out+used,size-used," LOW_POSITION=%u SPEED_I_UA=%ld DIST_RPM_S=%ld LOAD_UA=%ld OBS_RPM_MILLI=%ld FF=%u FLUX_UWB=%u FF_D_MV=%ld FF_Q_MV=%ld",
            low_position?1U:0U,(long)(speed_integral*1000000),(long)disturbance_rpm_s,
            (long)(load_a*1000000),(long)(velocity*1000),(unsigned)MOTOR_FOC_FEEDFORWARD_ENABLED,
            (unsigned)MOTOR_FOC_FLUX_UWB,(long)(-s.speed_lq_v_per_a*s.iq_a*1000),
            (long)((s.speed_ld_v_per_a*s.id_a+s.bemf_q_v)*1000)); }
    }
}

/** @brief 观察结果独立报告；TRACKING只说明PLL，不表示接管或速度闭环。 */
void FOC_DIAGNOSTIC_CODE
board_foc_observer_diagnostics(char *out,size_t size)
{
    if(g_foc.encoder_mode)
    { snprintf(out,size,"OK MOTOR FOC OBSERVER MODE=ENCODER_FOC ACTIVE=0 SOURCE=MT6835 NOT_USED=1"); return; }
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    foc_smo_t s=g_foc.test_mode==FOC_TEST_CLOSED?g_foc_drive.observer:g_foc_smo;
    bool drive_mode=g_foc.test_mode==FOC_TEST_CLOSED;
    float noise=g_foc.emf_noise_v,force=drive_mode?g_foc_drive.force_omega_rad_s:g_foc.force_omega;
    /* 正式运行报告实际生效的额外裕量门槛，不能仍显示旧电机固定100mV。 */
    float threshold=drive_mode?g_foc_drive.emf_floor_v:fmaxf(.1f,6*noise);
    bool active=g_foc.active;
    bool closed=active&&drive_mode&&g_foc_drive.stage==FOC_CLOSED;
    bool seen=drive_mode&&g_foc_drive.closed_seen;
    __set_PRIMASK(mask);
    snprintf(out,size,"OK MOTOR FOC OBSERVER MODE=%s UPDATES=%lu ACTIVE=%u TRACKING=%u CLOSED=%u CLOSED_SEEN=%u EST_RPM=%ld FORCE_RPM=%ld EMF_MV=%ld EA_MV=%ld EB_MV=%ld NOISE_MV=%ld THRESHOLD_MV=%ld PHASE_MRAD=%ld RESIDUAL_MA=%ld LPF_HZ=500 PLL_HZ=20 COMPENSATION=CAUSAL_ESTIMATED_OMEGA VOLTAGE=APPLIED_INTERVAL",
        drive_mode?"SENSORLESS_SPEED":"OBSERVE_ONLY",(unsigned long)s.updates,active?1U:0U,active&&s.pll_initialized?1U:0U,closed?1U:0U,seen?1U:0U,
        (long)(s.omega_rad_s*MOTOR_FOC_RPM_PER_RAD_S),(long)(force*MOTOR_FOC_RPM_PER_RAD_S),
        (long)(s.emf_magnitude_v*1000),(long)(s.emf_alpha_v*1000),(long)(s.emf_beta_v*1000),
        (long)(noise*1000),(long)(threshold*1000),(long)(s.phase_error*1000),(long)(s.residual_a*1000));
}

/** @brief 主循环报告实际提交后才产生的CLOSED历史及速度环计数。 */
void FOC_DIAGNOSTIC_CODE
board_foc_drive_diagnostics(char *out,size_t size)
{
#if MOTOR_ENCODER_ENABLED
    if(g_foc.encoder_mode)
    {
        uint32_t mask=__get_PRIMASK();__disable_irq();
        encoder_foc_t s=g_foc_encoder;
        bool active=g_foc.active;
        __set_PRIMASK(mask);
        snprintf(out,size,"OK MOTOR FOC DRIVE STATE=%s ACTIVE=%u CLOSED_SEEN=%u SAMPLES=%lu SPEED_TICKS=%lu AWAITING_COMMIT=%u ANGLE=MT6835 EST_RPM=%ld RPM_MILLI=%ld TARGET_RPM=%lu ID_MA=%ld IQ_MA=%ld REF_D_MA=0 REF_Q_MA=%ld I_LIMIT_MA=%ld RAW=%lu AGE_MAX_US=%lu DIR=%d ZERO_DECIDEG=%u MOTION_MS=%lu STARTUP_END_MS=%lu STARTUP=%s FAULT=%s MODE=%s TARGET_IQ_MA=%ld IQ_SLEW_MA_S=2000",
            !active?"STOPPED":s.closed_seen?"CLOSED":"WAIT_ENCODER",active?1U:0U,s.closed_seen?1U:0U,
            (unsigned long)s.samples,(unsigned long)s.speed_ticks,s.awaiting_commit?1U:0U,
            (long)s.rpm,(long)(s.rpm*1000),(unsigned long)((s.config.current_mode||s.config.position_mode||s.config.haptic_mode)?0U:s.config.drive.target_rpm),(long)(s.current.id_a*1000),
            (long)(s.current.iq_a*1000),(long)(s.reference_q_a*1000),(long)(s.config.drive.current_limit_a*1000),
            (unsigned long)s.raw,(unsigned long)(s.age_max_ticks/170U),s.config.direction,s.config.zero_decideg,
            (unsigned long)s.motion_ms,(unsigned long)s.startup_end_ms,
            !active?"OFF":s.config.current_mode?"IQ_RAMP":s.startup_precharge?"PRECHARGE":s.startup_end_ms?"DONE":"PI",s.fault,
            s.config.haptic_mode?"HAPTIC":s.config.position_mode?"POSITION":s.config.current_mode?"CURRENT":"SPEED",(long)lroundf(s.config.target_iq_a*1000.0f));
        return;
    }
#endif
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    foc_drive_t s=g_foc_drive;
    bool active=g_foc.active&&g_foc.test_mode==FOC_TEST_CLOSED;
    __set_PRIMASK(mask);
    snprintf(out,size,"OK MOTOR FOC DRIVE STATE=%s ACTIVE=%u CLOSED_SEEN=%u CLOSED_SAMPLES=%lu SPEED_TICKS=%lu QUALIFY_CYCLES=%lu AWAITING_COMMIT=%u ANGLE=%s EST_RPM=%ld TARGET_RPM=%lu ID_MA=%ld IQ_MA=%ld REF_D_MA=%ld REF_Q_MA=%ld I_LIMIT_MA=%ld FAULT=%s",
        foc_drive_stage_name(s.stage),active?1U:0U,s.closed_seen?1U:0U,(unsigned long)s.closed_samples,(unsigned long)s.speed_ticks,
        (unsigned long)s.qualify_cycles,s.awaiting_commit?1U:0U,active&&s.stage==FOC_CLOSED?"SMO_PLL":active?"STARTUP":"OFF",
        (long)(s.observer.omega_rad_s*MOTOR_FOC_RPM_PER_RAD_S),(unsigned long)s.config.target_rpm,
        (long)(s.current.id_a*1000),(long)(s.current.iq_a*1000),(long)(s.reference_d_a*1000),(long)(s.reference_q_a*1000),
        (long)(s.config.current_limit_a*1000),NULL==s.fault?"NONE":s.fault);
}

/** @brief 主循环报告现有资格计算量，独立小响应避免超出384字节。 */
void FOC_DIAGNOSTIC_CODE
board_foc_confidence_diagnostics(char *out,size_t size)
{
    if(g_foc.encoder_mode)
    { snprintf(out,size,"OK MOTOR FOC CONFIDENCE ACTIVE=0 SOURCE=MT6835 NOT_USED=1"); return; }
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    bool active=g_foc.active&&g_foc.test_mode==FOC_TEST_CLOSED;
    float emf=g_foc_drive.observer.emf_magnitude_v;
    float bound=g_foc_drive.model_bound_v,margin=g_foc_drive.emf_margin_v;
    float floor=g_foc_drive.emf_floor_v,error=g_foc_drive.config.voltage_error_bound_v;
    __set_PRIMASK(mask);
    snprintf(out,size,"OK MOTOR FOC CONFIDENCE ACTIVE=%u EMF_MV=%ld MODEL_BOUND_MV=%ld MARGIN_MV=%ld FLOOR_MV=%ld VOLTAGE_ERROR_MV=%ld MODEL_RL_PERCENT=20",
        active?1U:0U,(long)(emf*1000),(long)(bound*1000),(long)(margin*1000),(long)(floor*1000),(long)(error*1000));
}

/** @brief 见board_foc.h；主循环读取已有最新样本并换算，ADC不增加打印记录。 */
bool
board_foc_current(char *out, size_t size, bool firewater)
{
    /* 六步正常运行不复制FOC状态，也不进入临界区。 */
    if (!g_foc.active || g_foc.samples == 0U || g_foc.samples == g_foc_printed_sample) { return false; }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool active = g_foc.active;
    uint32_t sequence = g_foc.samples;
    uint32_t ticks = g_foc.sample_ticks;
    uint32_t pwm_sequence=g_foc_history.current.sequence;
    uint16_t a = g_foc.raw[0], b = g_foc.raw[1], c = g_foc.raw[2];
    int32_t da = (int32_t)a - g_foc.zero[0];
    int32_t db = (int32_t)b - g_foc.zero[1];
    int32_t dc = (int32_t)c - g_foc.zero[2];
    uint32_t vdda = g_foc.vdda_mv,units=g_foc.current_units;
    /* 复制控制已经算好的区间值；不在ADC增加任何串口诊断记录。 */
    float alpha_v = g_foc_interval.alpha_v, beta_v = g_foc_interval.beta_v;
    bool observe=g_foc.test_mode>=FOC_TEST_OBSERVE&&!g_foc.encoder_mode;
    const foc_smo_t *observer=g_foc.test_mode==FOC_TEST_CLOSED?&g_foc_drive.observer:&g_foc_smo;
    float omega=observer->omega_rad_s,theta=observer->sample_theta_rad;
    float forced=g_foc.test_mode==FOC_TEST_CLOSED?g_foc_drive.force_theta_rad:g_foc.force_theta;
    float emf=observer->emf_magnitude_v,residual=observer->residual_a;
    unsigned stage=g_foc.test_mode==FOC_TEST_CLOSED?(unsigned)g_foc_drive.stage:0U;
    __set_PRIMASK(mask);
    if (!active || sequence == 0U) { return false; }
    /* 50mΩ、按本轮真实CSA增益；整数mA换算在主循环完成，Ic由两相重建。 */
    /* current_units为10倍时2、20倍时1；最大乘积仍在int32范围内。 */
    int32_t ia = da * (int32_t)vdda * (int32_t)units / 4095;
    int32_t ib = db * (int32_t)vdda * (int32_t)units / 4095;
    int32_t isc = dc * (int32_t)vdda * (int32_t)units / 4095;
    if (firewater)
    {
        /* 官方FireWater帧为名称:CSV数据+换行；仅输出四路mA，缩短串口占用。 */
        int count = snprintf(out,size,"FOC:%ld,%ld,%ld,%ld\r\n",(long)ia,(long)ib,(long)(-ia-ib),(long)isc);
        if (count <= 0 || (size_t)count >= size) { return false; }
        g_foc_printed_sample = sequence;
        return true;
    }
    int count = snprintf(out,size,"DATA FOC SAMPLE=%lu PWM_SEQ=%lu TICKS=%lu RAW=%u,%u,%u IA_MA=%ld IB_MA=%ld IC_MA=%ld ISC_MA=%ld UA_MV=%ld UB_MV=%ld UVALID=%u",
        (unsigned long)sequence,(unsigned long)pwm_sequence,(unsigned long)ticks,a,b,c,(long)ia,(long)ib,(long)(-ia-ib),(long)isc,
        (long)(alpha_v*1000.0f),(long)(beta_v*1000.0f),sequence>1U?1U:0U);
    if (count <= 0 || (size_t)count >= size) { return false; }
    int extra=observe?snprintf(out+count,size-(size_t)count," EST_RPM=%ld THETA_MRAD=%ld FORCE_MRAD=%ld EMF_MV=%ld RES_MA=%ld STAGE_ID=%u\r\n",
        (long)(omega*MOTOR_FOC_RPM_PER_RAD_S),(long)(theta*1000),(long)(forced*1000),(long)(emf*1000),(long)(residual*1000),stage):
        snprintf(out+count,size-(size_t)count,"\r\n");
    if(extra<=0 || (size_t)(count+extra)>=size) { return false; }
    g_foc_printed_sample = sequence;
    return true;
}

/** @brief 见board_foc.h；只在短临界区复制原记录，字符串格式化在主循环完成。 */
void
board_foc_motion(char *out,size_t size,uint32_t first)
{
#if MOTOR_ENCODER_ENABLED
    encoder_motion_sample_t rows[ENCODER_MOTION_BATCH];
    uint32_t mask=__get_PRIMASK();__disable_irq();
    uint32_t count=encoder_motion_read(&g_foc_motion,&first,rows);
    uint32_t total=g_foc_motion.total,hz=g_foc_motion.tick_hz,gaps=g_foc_motion.gaps;
    uint32_t run=g_foc_motion_run;
    __set_PRIMASK(mask);
    size_t written=(size_t)snprintf(out,size,
        "OK FOC MOTION RUN=%lu FIRST=%lu NEXT=%lu TOTAL=%lu HZ=%lu TURN=%lu GAPS=%lu COUNT=%lu DATA=",
        (unsigned long)run,(unsigned long)first,(unsigned long)(first+count),(unsigned long)total,
        (unsigned long)hz,(unsigned long)ENCODER_SIX_TURN,(unsigned long)gaps,(unsigned long)count);
    for(uint32_t i=0;i<count && written<size;++i)
    {
        const encoder_motion_sample_t *p=&rows[i];
        written+=(size_t)snprintf(out+written,size-written,"%s%08lX%08lX%08lX%08lX%08lX%08lX%08lX",
            i?",":"",(unsigned long)p->time_hi,(unsigned long)p->time_lo,
            (unsigned long)p->position_hi,(unsigned long)p->position_lo,(unsigned long)p->raw,
            (unsigned long)(uint32_t)p->min_delta,(unsigned long)(uint32_t)p->max_delta);
    }
#else
    (void)first;
    snprintf(out,size,"ERR FOC ENCODER DISABLED");
#endif
}

/** @brief 见board_foc.h；计算、单位换算及格式化全部在主循环完成。 */
bool
board_foc_trace(char *out,size_t size)
{
    if(!g_foc.active || g_foc.samples==0U || g_foc.samples==g_foc_printed_sample) { return false; }
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    bool active=g_foc.active;
    uint32_t sequence=g_foc.samples,ticks=g_foc.sample_ticks,request_sequence=g_foc_preloaded.sequence;
    uint16_t raw_a=g_foc.raw[0],raw_b=g_foc.raw[1],raw_c=g_foc.raw[2];
    int32_t da=(int32_t)raw_a-g_foc.zero[0],db=(int32_t)raw_b-g_foc.zero[1],dc=(int32_t)raw_c-g_foc.zero[2];
    uint32_t vdda=g_foc.vdda_mv,bus=g_foc.bus_mv,trigger=g_foc.trigger_ticks,units=g_foc.current_units;
    bool closed_test=g_foc.test_mode==FOC_TEST_CLOSED;
    const foc_current_t *loop=closed_test?&g_foc_drive.current:&g_foc_current;
#if MOTOR_ENCODER_ENABLED
    if(g_foc.encoder_mode) { loop=&g_foc_encoder.current; }
#endif
    const foc_smo_t *observer=closed_test?&g_foc_drive.observer:&g_foc_smo;
    float id=loop->id_a,iq=loop->iq_a,rd=loop->reference_d,rq=loop->reference_q;
    float vd=loop->command_d_v,vq=loop->command_q_v,va=loop->request_alpha_v,vb=loop->request_beta_v;
    float theta=observer->sample_theta_rad,omega=observer->omega_rad_s;
    float control_theta=closed_test?g_foc_drive.control_theta_rad:g_foc.force_theta;
    unsigned stage=closed_test?(unsigned)g_foc_drive.stage:0U;
    unsigned hold=closed_test&&g_foc_drive.hold_iq?1U:0U;
#if MOTOR_ENCODER_ENABLED
    if(g_foc.encoder_mode)
    {
        theta=control_theta=g_foc_encoder.sample_theta_rad;
        omega=g_foc_encoder.omega_rad_s;
        stage=g_foc_encoder.closed_seen?(unsigned)FOC_CLOSED:(unsigned)FOC_CALIBRATE;
        hold=0U;
    }
#endif
    __set_PRIMASK(mask);
    if(!active) { return false; }
    int32_t ia=da*(int32_t)vdda*(int32_t)units/4095,ib=db*(int32_t)vdda*(int32_t)units/4095,isc=dc*(int32_t)vdda*(int32_t)units/4095;
    /* 扇区按新请求alpha/beta电压的0..360度分区；不是转子角扇区或已施加电压。 */
    float angle=atan2f(vb,va);
    if(angle<0) { angle+=6.28318530718f; }
    unsigned sector=(unsigned)(angle*.954929658551f)+1U;
    if(sector>6U) { sector=6U; }
    int count=snprintf(out,size,
        "FOC_TRACE:%lu,%lu,%lu,%u,%u,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%lu,%u,%lu,%ld,%lu\r\n",
        (unsigned long)sequence,(unsigned long)ticks,(unsigned long)request_sequence,stage,raw_a,raw_b,raw_c,
        (long)ia,(long)ib,(long)(-ia-ib),(long)isc,(long)(id*1000),(long)(iq*1000),(long)(rd*1000),(long)(rq*1000),
        (long)(theta*1000),(long)(omega*1000),(long)(vd*1000),(long)(vq*1000),(long)(va*1000),(long)(vb*1000),sector,
        (unsigned long)bus,hold,(unsigned long)trigger,(long)(control_theta*1000),(unsigned long)vdda);
    if(count<=0 || (size_t)count>=size) { return false; }
    g_foc_printed_sample=sequence;
    return true;
}

/** @brief 见board_foc.h。 */
bool FOC_DIAGNOSTIC_CODE
board_foc_position_target(int32_t target_mdeg)
{
#if MOTOR_ENCODER_ENABLED
    int64_t counts=encoder_position_target_counts(target_mdeg);
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool accepted=g_foc.active&&g_foc.encoder_mode&&g_foc_encoder.config.position_mode&&g_foc_encoder.closed_seen;
    if(accepted)
    {
        g_foc_encoder.config.target_position_mdeg=target_mdeg;
        g_foc_encoder.target_position_counts=counts;
        /* 在线发布同样重建逃离基准，保持与停机初始化/模型目标更新一致。 */
        g_foc_encoder.position_best_error_counts=INT64_MAX;
        g_foc_encoder.position_settle_ms=0U;
    }
    __set_PRIMASK(mask);
    return accepted;
#else
    (void)target_mdeg; return false;
#endif
}

/** @brief 见board_foc.h；64位原码拆成两字，不引入浮点printf或改变原电流包。 */
void FOC_DIAGNOSTIC_CODE
board_foc_position(char *out,size_t size,int32_t target_mdeg,int32_t tolerance_mdeg)
{
#if MOTOR_ENCODER_ENABLED
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    encoder_position_t p=board_encoder_position();
    bool running=g_foc.active&&g_foc.encoder_mode&&g_foc_encoder.config.position_mode;
    uint32_t settle=g_foc_encoder.position_settle_ms;
    float reference=g_foc_encoder.reference_rpm;
    __set_PRIMASK(mask);
    int64_t counts=encoder_position_counts(&p);
    int32_t error=encoder_position_mdeg(encoder_position_target_counts(target_mdeg)-counts);
    snprintf(out,size,"OK FOC POSITION VALID=%u ZERO_ID=%lu RAW=%lu POS_LO=%lu POS_HI=%ld TICKS=%lu TARGET_MDEG=%ld REF_MRPM=%ld ERROR_MDEG=%ld ARRIVED=%u TOL_MDEG=%ld RUN=%u",
        p.valid&&p.zero_valid?1U:0U,(unsigned long)p.epoch,(unsigned long)p.raw,
        (unsigned long)(uint32_t)counts,(long)(int32_t)((uint64_t)counts>>32U),(unsigned long)p.tick,
        (long)target_mdeg,(long)(running?lroundf(reference*1000.0f):0),(long)error,
        running&&settle>=200U?1U:0U,(long)tolerance_mdeg,running?1U:0U);
#else
    (void)target_mdeg;(void)tolerance_mdeg; snprintf(out,size,"ERR FOC DISABLED");
#endif
}

/** @brief 见board_foc.h；短临界区只复制固定16字节，ISR在下一机械拍消费。 */
bool FOC_DIAGNOSTIC_CODE
board_foc_haptic_configure(const encoder_haptic_config_t *config)
{
#if MOTOR_ENCODER_ENABLED
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool accepted=g_foc.active&&g_foc.encoder_mode&&g_foc_encoder.config.haptic_mode;
    if(accepted) { g_foc_encoder.config.haptic=*config; }
    __set_PRIMASK(mask);
    return accepted;
#else
    (void)config; return false;
#endif
}

/** @brief 见board_foc.h；64位计数拆字打印，避免浮点printf及更改电流协议。 */
void FOC_DIAGNOSTIC_CODE
board_foc_haptic(char *out,size_t size,const encoder_haptic_config_t *config,
    int32_t current_limit_ma,int32_t run_ms)
{
#if MOTOR_ENCODER_ENABLED
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    encoder_haptic_t h=g_foc_encoder.haptic;
    encoder_position_t p=board_encoder_position();
    bool running=g_foc.active&&g_foc.encoder_mode&&g_foc_encoder.config.haptic_mode;
    bool valid=running?g_foc_encoder.feedback_seen&&h.anchored:p.valid;
    float rpm=h.rpm,iq=g_foc_encoder.current.iq_a;
    uint32_t session=g_foc_motion_run;
    __set_PRIMASK(mask);
    bool transition=running&&(memcmp(config,&h.applied,sizeof(*config))!=0 || h.transition_s<.2f);
    snprintf(out,size,"OK FOC HAPTIC KIND=%s DETENTS=%ld DETENT_MA=%ld DAMPING_UA_RPM=%ld LIMIT_MA=%ld RUN_MS=%ld VALID=%u RUN=%u TRANSITION=%u SESSION=%lu RAW=%lu TICKS=%lu POS_LO=%lu POS_HI=%ld CENTER_LO=%lu CENTER_HI=%ld INDEX_LO=%lu INDEX_HI=%ld REF_MA=%ld IQ_MA=%ld RPM_MILLI=%ld",
        config->kind==ENCODER_HAPTIC_DETENT?"DETENT":"DAMPED",(long)config->detents,
        (long)config->detent_ma,(long)config->damping_ua_rpm,(long)current_limit_ma,(long)run_ms,
        valid?1U:0U,running?1U:0U,transition?1U:0U,(unsigned long)session,
        (unsigned long)(running?h.raw:p.raw),(unsigned long)(running?h.tick:p.tick),
        (unsigned long)(uint32_t)h.counts,(long)(int32_t)((uint64_t)h.counts>>32U),
        (unsigned long)(uint32_t)h.center_counts,(long)(int32_t)((uint64_t)h.center_counts>>32U),
        (unsigned long)(uint32_t)h.index,(long)(int32_t)((uint64_t)h.index>>32U),
        (long)(running?lroundf(h.iq_a*1000.0f):0),(long)(running?lroundf(iq*1000.0f):0),
        (long)(running?lroundf(rpm*1000.0f):0));
#else
    (void)config;(void)current_limit_ma;(void)run_ms;snprintf(out,size,"ERR FOC DISABLED");
#endif
}
