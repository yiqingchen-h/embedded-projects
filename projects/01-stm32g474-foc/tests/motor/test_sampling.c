#include "board_motor.h"
#include "six_step.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* 只模拟本测试需要的寄存器；被测函数从正式源码提取，不复制其逻辑。 */
typedef struct { uint32_t CR1, CR2, CNT, CCR1, CCR2, CCR3, CCR5, CCMR1, CCMR2, CCMR3, CCER, EGR, SR, RCR, BDTR, DIER; } timer_t;
typedef struct { uint32_t CR, ISR, IER, CFGR, JSQR, SQR1, SQR2, SQR3, SQR4, DR, JDR1; } adc_t;
typedef adc_t ADC_TypeDef;
typedef struct { uint32_t CYCCNT; } dwt_t;
typedef struct { uint32_t CCR, CPAR, CMAR, CNDTR; } dma_channel_t;
typedef struct { uint32_t IFCR; } dma_t;
static timer_t timer;
static adc_t adc, adc2;
static dwt_t dwt;
static dma_channel_t channel, mux;
static dma_t dma;
#define TIM1 (&timer)
#define ADC1 (&adc)
#define ADC2 (&adc2)
#define DWT (&dwt)
#define DMA1_Channel1 (&channel)
#define DMAMUX1_Channel0 (&mux)
#define DMA1 (&dma)
/* 保存互锁只读取使能电平，不访问真实 GPIO。 */
static struct { uint32_t ODR, BSRR; } gpio;
#define GPIOB (&gpio)
#define GPIO_PIN_9 (1U<<9)
#define DMA_IFCR_CGIF1 1U
#define DMA_CCR_EN 1U
#define DMA_CCR_TCIE 2U
#define DMA_CCR_TEIE 8U
#define DMA_CCR_CIRC 32U
#define DMA_CCR_MINC 128U
#define DMA_CCR_PSIZE_0 256U
#define DMA_CCR_MSIZE_0 1024U
/* 位号来自 STM32G474 CMSIS，测试运行时不使用物理外设地址。 */
#define TIM_CR1_DIR (1U<<4)
#define TIM_CR1_CEN 1U
#define TIM_CR1_CMS_0 (1U<<5)
#define TIM_CR2_CCPC 1U
#define TIM_CR2_MMS_Pos 4U
#define TIM_CR2_MMS2_Pos 20U
#define TIM_CCMR3_OC5M_Pos 4U
#define TIM_BDTR_MOE (1U<<15)
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC1NE 4U
#define TIM_CCER_CC4E (1U<<12)
#define TIM_CCER_CC4NE (1U<<14)
#define TIM_CCER_CC4NP (1U<<15)
#define TIM_CCMR1_OC1M_Pos 4U
#define TIM_CCMR1_OC2M_Pos 12U
#define TIM_CCMR2_OC3M_Pos 4U
#define TIM_CCMR2_OC4M_Pos 12U
#define TIM_EGR_COMG (1U<<5)
#define TIM_EGR_UG 1U
#define TIM_SR_UIF 1U
#define TIM_DIER_UIE 1U
#define TIM_DIER_BIE (1U<<7)
#define ADC_CFGR_JQM (1U<<21)
#define ADC_CFGR_DMAEN 1U
#define ADC_CFGR_DMACFG 2U
#define ADC_CR_ADSTART (1U<<2)
#define ADC_CR_JADSTART (1U<<3)
#define ADC_ISR_EOS (1U<<3)
#define ADC_ISR_OVR (1U<<4)
#define ADC_ISR_JEOC (1U<<5)
#define ADC_ISR_JEOS (1U<<6)
#define ADC_ISR_JQOVF (1U<<10)
#define ADC_IER_JEOSIE (1U<<6)
#define ADC_IER_OVRIE (1U<<4)
#define ADC_IER_JQOVFIE (1U<<10)
#define ADC_JSQR_JEXTSEL_Pos 2U
#define ADC_JSQR_JEXTEN_0 (1U<<7)
#define ADC_JSQR_JSQ1_Pos 9U
#define ADC_JSQR_JSQ2_Pos 15U
/* 与源码同名的状态，供提取函数访问。 */
static board_motor_status_t g_motor_status;
static board_motor_output_t g_motor_output, g_motor_next;
static bool g_motor_regular_busy, g_motor_need_context;
static uint32_t g_motor_next_regular, g_motor_regular_ticks, g_motor_last_fast, g_motor_jsqr[3];
static unsigned checks;
static unsigned g_motor_repeat;
static volatile uint16_t g_motor_dma[12];
/* 无感模式独立转换和硬件时间模型，字段与实际模块一致。 */
static bool g_motor_bemf_enabled, g_motor_bemf_window, g_motor_bemf_need_context;
static uint32_t g_motor_bemf_jsqr[3], g_motor_last_bemf, g_motor_last_current_conversion, g_motor_last_conversion;
static uint32_t g_motor_clock_cycle, g_motor_clock_us, g_motor_clock_remainder;
static uint32_t g_motor_peak_us, g_motor_bemf_sample_us, g_motor_committed_us;
static uint8_t g_motor_sector;
static board_motor_observation_t g_motor_observation;
static board_motor_sample_fn g_motor_sample;
/* 同步ADC直接回调，速度环在COM完成后按整数分频执行。 */
static board_motor_bemf_fn g_motor_bemf_callback;
static board_motor_speed_fn g_motor_speed_callback;
static uint32_t g_motor_speed_divider;
static board_motor_peak_fn g_motor_peak_callback;
/* 模拟功率侧主动全关留窗的实际时间，调度逻辑提取正式源码。 */
static bool g_motor_encoder_blanked;
static uint32_t g_motor_encoder_off_cycle;
static volatile bool g_motor_controlled,g_motor_schedule_pending;
static uint8_t g_motor_scheduled_sector;
static uint16_t g_motor_control_duty;
static uint32_t g_motor_schedule_us;

/* 临界区模型用于检查 getter 没有破坏既有 PRIMASK。 */
static uint32_t interrupt_mask;
static unsigned current_callbacks;
static bool trip_on_current;
/* 同时JEOS时记录回调看到的最新帧，并模拟回调安排下一扇区。 */
static uint32_t callback_seen_sequence;
static bool change_next_in_callback;
/* 注入真实控制耗时，检查ADC执行不能越过下一峰顶的提交预算。 */
static uint32_t callback_delay_cycles;
/* 模拟慢速毫秒时钟，独立于主循环执行。 */
static uint32_t mock_tick_ms;
/* 注入首次UEV的实际方向或等待超时；不以主机模型宣称芯片相位已实测。 */
static bool prime_at_valley, prime_timeout;
/* 启动测试可以独立推进硬件峰顶，并注入超过OFF触发余量的延迟。 */
static uint32_t prime_latency_ticks;
static unsigned prime_calls;
static bool mock_read_failure, mock_enable_failure, mock_stop_failure, mock_driver_failure;
#define CHECK(x) do { ++checks; assert(x); } while(0)

/** @brief 返回模拟中断屏蔽位。 @return 当前 PRIMASK。 */
static void __DMB(void) { }
static uint32_t HAL_GetTick(void) { return mock_tick_ms; }
static uint32_t __get_PRIMASK(void) { return interrupt_mask; }
/** @brief 模拟短临界区入口。 */
static void __disable_irq(void) { interrupt_mask=1U; }
/** @brief 恢复此前屏蔽位。 @param mask 此前 PRIMASK。 */
static void __set_PRIMASK(uint32_t mask) { interrupt_mask=mask; }

/** @brief 电流保护优先的回调替身，可模拟过流关断。
 * @param raw ADC2 电流。 @param low 当前低相。 @param valid 导通有效。
 * @param next 下周期输出。 */
static void sample_callback(uint16_t raw,uint8_t low,bool valid,board_motor_output_t *next)
{
    CHECK(raw==2222U && low==g_motor_output.low && valid==g_motor_output.pulse);
    CHECK(next==&g_motor_next);
    callback_seen_sequence=g_motor_observation.sequence;
    if(change_next_in_callback) { next->high=1U; next->low=2U; }
    ++current_callbacks;
    dwt.CYCCNT += callback_delay_cycles;
    if(trip_on_current) { board_motor_stop("CURRENT TRIP"); }
}
/** @brief 在功率仍禁用时注入首个更新事件。 @param reg 更新状态寄存器。 @param mask UIF。 @param set 等待置位。 @return 模拟等待是否完成。 */
static bool wait_bits(volatile uint32_t *reg,uint32_t mask,bool set)
{
    CHECK(reg==&timer.SR && mask==TIM_SR_UIF && set);
    CHECK((timer.CR1&TIM_CR1_CEN) && timer.RCR==1);
    CHECK(!(timer.BDTR&TIM_BDTR_MOE) && timer.DIER==0 && !g_motor_status.running);
    if(prime_timeout) { return false; }
    if(!prime_at_valley) { timer.CR1|=TIM_CR1_DIR; }
    uint32_t peak=170000000U/(2U*MOTOR_CONTROL_HZ);
    timer.CNT=peak-prime_latency_ticks;
    dwt.CYCCNT+=peak+prime_latency_ticks; ++prime_calls;
    timer.SR|=TIM_SR_UIF;
    return true;
}
/** @brief 模拟停机软件预热，更新对应转换完成时刻。 @param input ADC输入。 @param raw 结果。 @return 未注入失败。 */
static bool adc_read(uint32_t input,uint16_t *raw)
{
    dwt.CYCCNT+=1040U; *raw=0;
    if(input>=6U && input<=8U && g_motor_bemf_enabled) { g_motor_last_current_conversion=dwt.CYCCNT; }
    else { g_motor_last_conversion=dwt.CYCCNT; }
    return !mock_read_failure;
}
/** @brief 替换停机回收并保留队列未就绪失败。 @return 是否回收成功。 */
static bool adc_quiesce(void) { adc.CR=adc2.CR=0; return !mock_stop_failure; }
/** @brief 模拟两ADC就绪。 @return 未注入就绪失败。 */
static bool adc_enable(void) { return !mock_enable_failure; }
/** @brief 模拟DRV启动前回读。 @param coast 预期高阻状态。 @return 未注入驱动失败。 */
static bool drv8353f_verify(bool coast) { (void)coast; return !mock_driver_failure; }
/** @brief 替换主循环SPI解除COAST，不产生硬件访问。 @param coast 高阻状态。 @return 成功。 */
static bool drv8353f_coast(bool coast) { CHECK(!coast); return true; }
/** @brief 模拟已完成的配置回读。 @return 成功。 */
static bool driver_capture(void) { return true; }
#include "sampling_under_test.inc"

/** @brief 从一个合法 JEOS 后的时间点建立场景。 @param count 当前计数值。 @param down 下降方向。 */
static void reset(uint32_t count,bool down)
{
    memset(&timer,0,sizeof(timer)); memset(&adc,0,sizeof(adc)); memset(&adc2,0,sizeof(adc2));
    memset(&g_motor_status,0,sizeof(g_motor_status));
    g_motor_status.running=true; g_motor_status.fast_count=100;
    g_motor_regular_busy=false; g_motor_next_regular=100; g_motor_regular_ticks=5286;
    timer.CNT=count; timer.CR1=down?TIM_CR1_DIR:0;
    g_motor_bemf_enabled=false; g_motor_repeat=1U;
    g_motor_bemf_window=g_motor_bemf_need_context=false;
    memset(&g_motor_observation,0,sizeof(g_motor_observation));
    g_motor_clock_cycle=dwt.CYCCNT; g_motor_clock_us=g_motor_clock_remainder=0;
    g_motor_peak_us=g_motor_bemf_sample_us=g_motor_committed_us=0;
    g_motor_sector=0U;
    g_motor_peak_callback=NULL; g_motor_bemf_callback=NULL; g_motor_speed_callback=NULL; g_motor_speed_divider=0U;
    g_motor_encoder_blanked=false;g_motor_encoder_off_cycle=0U;
    g_motor_controlled=g_motor_schedule_pending=false;
    g_motor_last_bemf=dwt.CYCCNT;
    interrupt_mask=0; gpio.ODR=0; gpio.BSRR=0;
    memset(&g_motor_output,0,sizeof(g_motor_output)); memset(&g_motor_next,0,sizeof(g_motor_next));
    current_callbacks=0; trip_on_current=false; g_motor_sample=sample_callback;
    callback_seen_sequence=0; change_next_in_callback=false;
    callback_delay_cycles=0;
    mock_tick_ms=0U;
    prime_latency_ticks=prime_calls=0; prime_at_valley=prime_timeout=false;
    mock_read_failure=mock_enable_failure=mock_stop_failure=mock_driver_failure=false;
}

/** @brief 10/20 kHz共享绝对采样位置和死区；比较值随周期改变，最小稳定脉宽不缩短。 */
static void test_timing_parameters(void)
{
    CHECK(TIMER_HZ==170000000U && SAMPLE_COMPARE==170U);
    CHECK(2U*PWM_ARR/TIMER_TICKS_US==1000000U/MOTOR_CONTROL_HZ);
    CHECK(TIMER_DEADTIME_NS==200U && DRIVER_DEADTIME_NS==100U);
    CHECK(TIMER_DEADTIME_NS*TIMER_TICKS_US/1000U==34U);
    CHECK(board_motor_min_duty(50)==391U && board_motor_min_duty(1)==646U);
    CHECK(board_motor_duty_ticks(100)==PWM_ARR/10U);
    CHECK(board_motor_duty_ticks(100)>=board_motor_min_duty(50));
    if(MOTOR_CONTROL_HZ==20000U)
    {
        CHECK(PWM_ARR==4250U && board_motor_duty_ticks(100)==425U);
        CHECK(board_motor_duty_ticks(100)<board_motor_min_duty(1));
    }
    else
    {
        CHECK(MOTOR_CONTROL_HZ==10000U && PWM_ARR==8500U);
        CHECK(board_motor_duty_ticks(100)>=board_motor_min_duty(1));
    }
}

/** @brief 最大周期不能容纳配对Regular扫描时，真实启动守卫必须先关断且给出ADC TIMING。 */
static void test_start_timing(void)
{
    reset(90,true);
    CHECK(regular_timing_ready());
    CHECK(g_motor_status.running);
    g_motor_regular_ticks=10562U;
    timer.CR1|=TIM_CR1_CEN; timer.BDTR|=TIM_BDTR_MOE; timer.DIER=1;
    if(MOTOR_CONTROL_HZ==20000U)
    {
        CHECK(!regular_timing_ready());
        CHECK(!g_motor_status.running && !g_motor_status.calibrated);
        CHECK(!(timer.CR1&TIM_CR1_CEN) && !(timer.BDTR&TIM_BDTR_MOE) && timer.DIER==0);
        CHECK(strcmp(g_motor_status.fault,"ADC TIMING")==0);
    }
    else
    {
        CHECK(regular_timing_ready());
        CHECK(g_motor_status.running && (timer.BDTR&TIM_BDTR_MOE));
    }
}

/** @brief 不追赶 1 kHz 时刻：窗口不足跳过，下一周期有余量才启动。 */
static void test_opportunities(void)
{
    reset(90,true); regular_opportunity();
    CHECK(adc.CR&ADC_CR_ADSTART); CHECK(g_motor_regular_busy);
    CHECK(g_motor_next_regular==100U+REGULAR_INTERVAL_TICKS);
    reset(SAMPLE_COMPARE+100U,true); regular_opportunity();
    CHECK(!(adc.CR&ADC_CR_ADSTART) && !g_motor_regular_busy);
    CHECK(g_motor_status.regular_skipped==1 && g_motor_next_regular==100);
    timer.CNT=80; ++g_motor_status.fast_count; regular_opportunity();
    CHECK(adc.CR&ADC_CR_ADSTART); CHECK(g_motor_next_regular==101U+REGULAR_INTERVAL_TICKS);
    reset(100,false); g_motor_regular_busy=true; regular_opportunity(); CHECK(adc.CR==0);
    reset(100,false); g_motor_next_regular=101; regular_opportunity(); CHECK(adc.CR==0);
    /* 最坏扫描耗时改变后仍严格检查临界边界，而非比较一个固定 CNT。 */
    reset(100,false); g_motor_regular_ticks=2*PWM_ARR-100-SAMPLE_COMPARE-REGULAR_LAUNCH_TICKS;
    regular_opportunity(); CHECK(adc.CR==0);
    --g_motor_regular_ticks; regular_opportunity(); CHECK(adc.CR&ADC_CR_ADSTART);
    /* 两种修订版扫描预算扫过全周期，任一实际启动都必须在下次fast触发前完成。 */
    for(unsigned repeat=1;repeat<=2;repeat++)
    {
        const uint32_t scan=repeat==1U?5286U:10562U;
        unsigned launches=0;
        for(unsigned down=0;down<2;down++)
        {
            for(unsigned cnt=0;cnt<=PWM_ARR;cnt++)
            {
                reset(cnt,down!=0); g_motor_regular_ticks=scan;
                regular_opportunity();
                if(adc.CR&ADC_CR_ADSTART)
                {
                    uint32_t next=down?(cnt>=SAMPLE_COMPARE?cnt-SAMPLE_COMPARE:cnt+2*PWM_ARR-SAMPLE_COMPARE):2*PWM_ARR-cnt-SAMPLE_COMPARE;
                    CHECK(next>scan+REGULAR_LAUNCH_TICKS);
                    ++launches;
                }
            }
        }
        CHECK((launches!=0)==(repeat==1U || MOTOR_CONTROL_HZ==10000U));
    }
}

/** @brief 六扇区输出不同时使能同一桥臂上下管；仅在峰顶换相且一周期补一个上下文。 */
static void test_commutation(void)
{
    /* 标记通道独立于三相功率使能，两种构建均不得启用 CH4 主输出或反相。 */
    const uint32_t marker = MOTOR_ADC_TRACE_ENC_CAL ? TIM_CCER_CC4NE : 0U;
    for(uint8_t sector=0;sector<6;sector++)
    {
        board_motor_output_t output={0};
        six_step_phases(sector,&output.high,&output.low);
        output.pulse=true; output.duty_ticks=board_motor_duty_ticks(100);
        apply_output(&output);
        CHECK(timer.CCER==((1U<<(4*output.high))|(4U<<(4*output.low))|marker));
        CHECK(!(timer.CCER&(TIM_CCER_CC4E|TIM_CCER_CC4NP)));
        for(unsigned phase=0;phase<3;phase++) { CHECK(((timer.CCER>>(phase*4))&5U)!=5U); }
        CHECK(timer.CCR1==output.duty_ticks && timer.CCR2==output.duty_ticks && timer.CCR3==output.duty_ticks);
        CHECK(timer.EGR==TIM_EGR_COMG);
        output.pulse=false; apply_output(&output); CHECK(timer.CCER==marker);
        /* 所有扇区的峰顶提交都须选择新输出的低侧上下文，而非固定某个电流相。 */
        reset(PWM_ARR,true); output.pulse=true; g_motor_next=output;
        for(unsigned phase=0;phase<3;phase++) { g_motor_jsqr[phase]=0x123450U+phase; }
        g_motor_last_fast=dwt.CYCCNT=1000;
        g_motor_need_context=true; TIM1_UP_TIM16_IRQHandler();
        CHECK(adc.JSQR==g_motor_jsqr[output.low] && g_motor_output.low==output.low);
        CHECK(!g_motor_need_context);
    }
    reset(PWM_ARR,true); g_motor_next=(board_motor_output_t){1,2,board_motor_duty_ticks(100),true};
    g_motor_jsqr[2]=0x123456; g_motor_last_fast=dwt.CYCCNT=1000;
    g_motor_need_context=true; TIM1_UP_TIM16_IRQHandler(); CHECK(adc.JSQR==0x123456 && !g_motor_need_context);
    CHECK(g_motor_status.update_count==1 && g_motor_status.peak_count==1);
    adc.JSQR=0; TIM1_UP_TIM16_IRQHandler(); CHECK(adc.JSQR==0);
    g_motor_need_context=true; timer.CR1=0; TIM1_UP_TIM16_IRQHandler(); CHECK(adc.JSQR==0);
    /* 复现只有首个JEOS：更新落在谷底时，旧实现静默跳过，不能继续留在RUN。 */
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"TIM PHASE")==0);
    CHECK(g_motor_status.update_count==3 && g_motor_status.peak_count==2);
    g_motor_status.running=true;
    timer.CR1=TIM_CR1_DIR; dwt.CYCCNT+=ADC_IDLE_WARMUP_CYCLES+1;
    TIM1_UP_TIM16_IRQHandler(); CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"ADC TIMEOUT")==0);
}

/** @brief 首次峰顶同步仅在功率禁用时进行，谷底相位或超时都拒绝启动。 */
static void test_start_phase(void)
{
    reset(0,false); g_motor_status.running=false; prime_at_valley=prime_timeout=false;
    CHECK(timer_prime_peak());
    CHECK((timer.CR1&(TIM_CR1_CEN|TIM_CR1_DIR))==(TIM_CR1_CEN|TIM_CR1_DIR) && timer.SR==0);
    reset(0,false); g_motor_status.running=false; prime_at_valley=true;
    CHECK(!timer_prime_peak());
    CHECK(!(timer.CR1&TIM_CR1_CEN) && strcmp(g_motor_status.fault,"TIM PHASE")==0);
    reset(0,false); g_motor_status.running=false; prime_at_valley=false; prime_timeout=true;
    CHECK(!timer_prime_peak());
    CHECK(!(timer.CR1&TIM_CR1_CEN) && strcmp(g_motor_status.fault,"TIM PHASE")==0);
}
/** @brief Rev Z 配对保持第一结果所在索引，Y/X 单次；DMA 始终 circular。 */
static void test_regular_sequence(void)
{
    static const unsigned expected[6]={8,1,2,3,4,18};
    for(g_motor_repeat=1;g_motor_repeat<=2;g_motor_repeat++)
    {
        regular_configure();
        CHECK((adc.SQR1&15U)==6*g_motor_repeat-1);
        CHECK(channel.CNDTR==6*g_motor_repeat && (channel.CCR&DMA_CCR_CIRC));
        for(unsigned rank=0;rank<6*g_motor_repeat;rank++)
        {
            uint32_t reg=rank<4?adc.SQR1:rank<9?adc.SQR2:adc.SQR3;
            unsigned position=rank<4?6+rank*6:rank<9?(rank-4)*6:(rank-9)*6;
            CHECK(((reg>>position)&31U)==expected[rank/g_motor_repeat]);
        }
    }
}

/** @brief 真实拓扑配置必须保持电流触发，并为三浮相选正确电压通道和独立慢速预算。 */
static void test_bemf_configuration(void)
{
    reset(PWM_ARR,true); sampling_configure();
    CHECK(timer.CR2==(TIM_CR2_CCPC|(7U<<TIM_CR2_MMS2_Pos)) && timer.CCMR3==0);
    CHECK(g_motor_regular_ticks==5286U);
    for(unsigned phase=0;phase<3;phase++)
    {
        CHECK(((g_motor_jsqr[phase]>>ADC_JSQR_JEXTSEL_Pos)&15U)==8U);
        CHECK(((g_motor_jsqr[phase]>>ADC_JSQR_JSQ1_Pos)&31U)==phase+6U);
    }
    g_motor_bemf_enabled=true; sampling_configure();
    CHECK(timer.CR2==(TIM_CR2_CCPC|(7U<<TIM_CR2_MMS_Pos)|(7U<<TIM_CR2_MMS2_Pos)));
    CHECK(timer.CCMR3==(6U<<TIM_CCMR3_OC5M_Pos) && g_motor_regular_ticks==2090U);
    for(unsigned phase=0;phase<3;phase++)
    {
        CHECK(((g_motor_jsqr[phase]>>ADC_JSQR_JEXTSEL_Pos)&15U)==0U);
        CHECK(((g_motor_jsqr[phase]>>ADC_JSQR_JSQ1_Pos)&31U)==phase+6U);
        CHECK(((g_motor_bemf_jsqr[phase]>>ADC_JSQR_JEXTSEL_Pos)&15U)==0U);
        CHECK(((g_motor_bemf_jsqr[phase]>>ADC_JSQR_JSQ1_Pos)&31U)==phase+2U);
        CHECK((g_motor_bemf_jsqr[phase]&3U)==0U);
    }
    CHECK(regular_timing_ready());
    regular_configure();
    CHECK((adc.SQR1&15U)==1U && channel.CNDTR==2U);
    CHECK(((adc.SQR1>>6U)&31U)==1U && ((adc.SQR1>>12U)&31U)==18U);
    CHECK(mux.CCR==ADC1_DMA_REQUEST && (channel.CCR&DMA_CCR_CIRC));
    /* Rev Z/未知修订不能静默用单次BEMF序列继续运行。 */
    g_motor_repeat=2U; sampling_configure();
    CHECK(!regular_timing_ready() && strcmp(g_motor_status.fault,"BEMF REV")==0);
}

/** @brief 六个实际扇区的浮相上下文、时间及跳脉冲窗口均从真实峰顶提交建立。 */
static void test_bemf_commutation(void)
{
    for(uint8_t sector=0;sector<6;sector++)
    {
        reset(PWM_ARR-2U*TIMER_TICKS_US,true);
        g_motor_bemf_enabled=true; sampling_configure();
        g_motor_clock_us=500U; g_motor_clock_cycle=dwt.CYCCNT;
        dwt.CYCCNT+=2U*TIMER_TICKS_US; g_motor_last_fast=g_motor_last_bemf=dwt.CYCCNT;
        six_step_phases(sector,&g_motor_next.high,&g_motor_next.low);
        g_motor_next.duty_ticks=board_motor_duty_ticks(200U); g_motor_next.pulse=true;
        g_motor_need_context=g_motor_bemf_need_context=true;
        TIM1_UP_TIM16_IRQHandler();
        uint8_t floating=(uint8_t)(3U-g_motor_next.high-g_motor_next.low);
        CHECK(g_motor_sector==sector && g_motor_committed_us==502U && g_motor_peak_us==500U);
        CHECK(adc2.JSQR==g_motor_jsqr[g_motor_next.low] && adc.JSQR==g_motor_bemf_jsqr[floating]);
        CHECK(!g_motor_need_context && !g_motor_bemf_need_context && g_motor_bemf_window);
        CHECK(timer.CCR5==SAMPLE_COMPARE);
        CHECK(g_motor_bemf_sample_us==500U+(PWM_ARR-timer.CCR5+BEMF_HOLD_TICKS)/TIMER_TICKS_US);
        /* 占空比变化及重复COM不能重置真实换相时间。 */
        ++g_motor_next.duty_ticks;
        dwt.CYCCNT+=PWM_PERIOD_US*TIMER_TICKS_US;
        g_motor_last_fast=g_motor_last_bemf=dwt.CYCCNT;
        TIM1_UP_TIM16_IRQHandler();
        CHECK(g_motor_committed_us==502U && g_motor_bemf_window);
        g_motor_next.pulse=false; apply_output(&g_motor_next);
        CHECK(g_motor_committed_us==502U && !g_motor_bemf_window);
        g_motor_next.pulse=true; apply_output(&g_motor_next); CHECK(g_motor_bemf_window);
        apply_output(&g_motor_next); CHECK(g_motor_bemf_window && g_motor_committed_us==502U);
    }
}

/** @brief 同时JEOS先发布最新BEMF原始帧，快速回调仍先保护；其后才更新诊断与慢速扫描。 */
static void test_bemf_interrupts(void)
{
    reset(0,true); g_motor_bemf_enabled=true; g_motor_status.vdda_mv=3300;
    g_motor_output=(board_motor_output_t){0,1,board_motor_duty_ticks(200),true};
    g_motor_bemf_window=true; g_motor_bemf_sample_us=37; g_motor_committed_us=11;
    adc2.JDR1=2222; adc2.ISR=ADC_ISR_JEOS; adc.JDR1=321; adc.ISR=ADC_ISR_JEOS;
    change_next_in_callback=true;
    ADC1_2_IRQHandler();
    CHECK(current_callbacks==1U && g_motor_status.fast_count==101U);
    CHECK(g_motor_need_context && g_motor_bemf_need_context);
    CHECK(g_motor_observation.sequence==1U && g_motor_observation.raw==321U);
    CHECK(callback_seen_sequence==0U && g_motor_next.high==1U && g_motor_next.low==2U);
    CHECK(g_motor_observation.sector==0U && g_motor_output.high==0U && g_motor_output.low==1U);
    CHECK(g_motor_observation.valid && g_motor_observation.floating==2U);
    CHECK(g_motor_observation.sample_us==37U && g_motor_observation.committed_us==11U);
    CHECK(g_motor_status.phase_valid[2]);
    g_motor_clock_us=40U;
    board_motor_observation_t frame=board_motor_observation();
    CHECK(frame.sequence==1U && frame.sector==0U);
    CHECK(g_motor_status.phase_valid[2] && !g_motor_status.phase_valid[0]);
    /* 正常控制不逐样本换算mV，诊断保留原始码和时间。 */
    CHECK(g_motor_status.phase_sample_us[2]==37U);
    /* 无效OFF样本也递增序号，但不得覆盖上次有效相电压。 */
    g_motor_bemf_window=false; adc2.ISR=0; adc.ISR=ADC_ISR_JEOS; adc.JDR1=0;
    ADC1_2_IRQHandler();
    CHECK(current_callbacks==1U && g_motor_observation.sequence==2U && !g_motor_observation.valid);
    frame=board_motor_observation(); CHECK(!frame.valid);
    CHECK(!g_motor_status.phase_valid[2] && g_motor_status.phase_sample_us[2]==37U);
    /* 正常控制不逐样本换算mV，诊断保留原始码和时间。 */
    /* 原始帧已经发布，但电流保护停机后禁止转换诊断或开慢速扫描。 */
    reset(0,true); g_motor_bemf_enabled=true; trip_on_current=true;
    adc2.ISR=adc.ISR=ADC_ISR_JEOS; adc2.JDR1=2222;
    adc.IER=adc2.IER=1U; timer.CR1|=TIM_CR1_CEN; timer.BDTR|=TIM_BDTR_MOE;
    ADC1_2_IRQHandler();
    CHECK(current_callbacks==1U && !g_motor_status.running && g_motor_observation.sequence==0U);
    CHECK(!(adc.CR&ADC_CR_ADSTART) && !g_motor_status.phase_valid[2]);
    CHECK(adc.IER==0U && adc2.IER==0U && !(timer.CR1&TIM_CR1_CEN));
    CHECK(!(timer.BDTR&TIM_BDTR_MOE) && gpio.BSRR==(GPIO_PIN_9<<16U));
    reset(0,true); g_motor_bemf_enabled=true; adc2.ISR=ADC_ISR_OVR; adc.ISR=ADC_ISR_JEOS;
    ADC1_2_IRQHandler();
    CHECK(current_callbacks==0U && strcmp(g_motor_status.fault,"ADC OVERRUN")==0);
    reset(0,true); g_motor_bemf_enabled=true; adc2.ISR=ADC_ISR_JEOS; adc2.JDR1=2222;
    adc.ISR=ADC_ISR_JQOVF; ADC1_2_IRQHandler();
    CHECK(current_callbacks==0U && strcmp(g_motor_status.fault,"BEMF ADC")==0);
    /* 候选写入后超过提交余量必须同步撤销功率，不等下一峰顶误提交。 */
    reset(0,true); g_motor_bemf_enabled=true; adc2.ISR=ADC_ISR_JEOS; adc2.JDR1=2222;
    /* 当前JEOS已在载波中段，不能把整个PWM周期当成控制器剩余执行时间。 */
    g_motor_clock_us=PWM_PERIOD_US/2U;
    callback_delay_cycles=(PWM_PERIOD_US/2U)*TIMER_TICKS_US;
    timer.BDTR=TIM_BDTR_MOE; change_next_in_callback=true;
    ADC1_2_IRQHandler();
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"CONTROL DEADLINE")==0);
    CHECK(!(timer.BDTR&TIM_BDTR_MOE) && !g_motor_next.pulse);
    CHECK(g_motor_status.irq_max_cycles==callback_delay_cycles);
    reset(0,true); g_motor_bemf_enabled=true; adc2.ISR=ADC_ISR_JEOS; adc2.JDR1=2222;
    g_motor_clock_us=PWM_PERIOD_US/2U;
    callback_delay_cycles=(PWM_PERIOD_US/2U-3U)*TIMER_TICKS_US;
    ADC1_2_IRQHandler(); CHECK(g_motor_status.running);
    /* 原开环ADC1仍只承担电流回调。 */
    reset(0,true); adc.ISR=ADC_ISR_JEOS; adc.JDR1=2222; ADC1_2_IRQHandler();
    CHECK(current_callbacks==1U && g_motor_observation.sequence==0U);
}

/** @brief BEMF慢速机会按OC5触发位置预算，两路看门分别约束实际完成。 */
static void test_bemf_opportunities_and_timeout(void)
{
    reset(0U,true); g_motor_bemf_enabled=true; sampling_configure();
    timer.CCR5=board_motor_duty_ticks(200U)+BEMF_CONVERSION_TICKS+BEMF_EDGE_GUARD_TICKS;
    regular_opportunity();
    CHECK((adc.CR&ADC_CR_ADSTART) && !(adc2.CR&ADC_CR_ADSTART));
    reset(0,true); g_motor_bemf_enabled=true; sampling_configure(); timer.CCR5=2000U;
    timer.CNT=SAMPLE_COMPARE+g_motor_regular_ticks+REGULAR_LAUNCH_TICKS;
    regular_opportunity(); CHECK(!(adc.CR&ADC_CR_ADSTART));
    ++timer.CNT; regular_opportunity(); CHECK(adc.CR&ADC_CR_ADSTART);
    reset(PWM_ARR,true); g_motor_bemf_enabled=true;
    g_motor_last_bemf=dwt.CYCCNT-ADC_IDLE_WARMUP_CYCLES-1U; g_motor_last_fast=dwt.CYCCNT;
    TIM1_UP_TIM16_IRQHandler();
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"BEMF TIMEOUT")==0);
    reset(PWM_ARR,true); g_motor_bemf_enabled=true;
    g_motor_last_fast=dwt.CYCCNT-ADC_IDLE_WARMUP_CYCLES-1U;
    TIM1_UP_TIM16_IRQHandler();
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"ADC TIMEOUT")==0);
}

/** @brief DWT回绕和不足1 us余数不丢失；一致getter保留PRIMASK，Flash互锁覆盖两ADC。 */
static void test_bemf_time_and_storage(void)
{
    reset(0,true); g_motor_bemf_enabled=true;
    g_motor_clock_cycle=UINT32_MAX-84U; dwt.CYCCNT=85U;
    motor_clock_update(); CHECK(motor_time_us()==1U && g_motor_clock_remainder==0U);
    dwt.CYCCNT+=85U; motor_clock_update(); CHECK(motor_time_us()==1U && g_motor_clock_remainder==85U);
    dwt.CYCCNT+=85U; motor_clock_update(); CHECK(motor_time_us()==2U && g_motor_clock_remainder==0U);
    g_motor_observation.valid=true; g_motor_observation.sample_us=1; g_motor_peak_us=10;
    for(unsigned mask=0;mask<=1;mask++)
    {
        interrupt_mask=mask; board_motor_observation_t observation=board_motor_observation();
        CHECK(interrupt_mask==mask && observation.valid && observation.now_us==2U);
        CHECK(observation.sample_us==1U && observation.next_commit_us==10U+PWM_PERIOD_US);
    }
    g_motor_status.running=false; CHECK(!board_motor_observation().valid && board_motor_storage_safe());
    adc.CR=ADC_CR_JADSTART; CHECK(!board_motor_storage_safe()); adc.CR=ADC_CR_ADSTART;
    CHECK(!board_motor_storage_safe()); adc.CR=0;
    adc2.CR=ADC_CR_JADSTART; CHECK(!board_motor_storage_safe()); adc2.CR=ADC_CR_ADSTART;
    CHECK(!board_motor_storage_safe()); adc2.CR=0;
    gpio.ODR=GPIO_PIN_9; CHECK(!board_motor_storage_safe()); gpio.ODR=0;
    timer.BDTR=TIM_BDTR_MOE; CHECK(!board_motor_storage_safe()); timer.BDTR=0;
    CHECK(board_motor_storage_safe());
}

/** @brief 真实启动必须重建运行时间域、同步首个峰顶并同时开放两队列；失败保持关断。 */
static void test_bemf_start(void)
{
    const board_motor_output_t output={0,1,board_motor_duty_ticks(200U),true};
    reset(0,false); g_motor_status.running=false; g_motor_status.calibrated=true;
    g_motor_bemf_enabled=true; sampling_configure(); prime_latency_ticks=2U*TIMER_TICKS_US;
    g_motor_clock_us=123456U; g_motor_clock_remainder=85U;
    interrupt_mask=1U;
    CHECK(board_motor_start(&output,sample_callback));
    CHECK(prime_calls==2U && interrupt_mask==1U);
    CHECK(g_motor_status.running && (timer.BDTR&TIM_BDTR_MOE));
    CHECK((adc.CR&ADC_CR_JADSTART) && (adc2.CR&ADC_CR_JADSTART));
    CHECK(adc.JSQR==g_motor_bemf_jsqr[2] && adc2.JSQR==g_motor_jsqr[1]);
    CHECK(adc.IER==adc2.IER && (adc.IER&ADC_IER_JEOSIE));
    CHECK(g_motor_peak_us==0U && g_motor_clock_us==2U && g_motor_clock_remainder==0U);
    CHECK(g_motor_committed_us==2U && g_motor_bemf_window);
    CHECK(g_motor_bemf_sample_us==(PWM_ARR-timer.CCR5+BEMF_HOLD_TICKS)/TIMER_TICKS_US);
    board_motor_observation_t observation=board_motor_observation();
    CHECK(!observation.valid && observation.now_us==2U && observation.next_commit_us==PWM_PERIOD_US);
    dwt.CYCCNT+=PWM_PERIOD_US*TIMER_TICKS_US;
    TIM1_UP_TIM16_IRQHandler();
    CHECK(g_motor_peak_us==PWM_PERIOD_US && g_motor_committed_us==2U && g_motor_bemf_window);
    board_motor_stop(NULL);
    CHECK(!g_motor_status.running && adc.IER==0U && adc2.IER==0U && !g_motor_bemf_window);
    /* 异常启动延迟跨过首OFF机会时不能使用过时峰顶预测。 */
    reset(0,false); g_motor_status.running=false; g_motor_status.calibrated=true;
    g_motor_bemf_enabled=true; sampling_configure();
    prime_latency_ticks=PWM_ARR-(SAMPLE_COMPARE+REGULAR_LAUNCH_TICKS);
    CHECK(!board_motor_start(&output,sample_callback));
    CHECK(strcmp(g_motor_status.fault,"TIM START")==0 && !(timer.BDTR&TIM_BDTR_MOE));
    CHECK(adc.IER==0U && adc2.IER==0U && interrupt_mask==0U);
    for(unsigned failure=0;failure<4;failure++)
    {
        reset(0,false); g_motor_status.running=false; g_motor_status.calibrated=true;
        g_motor_bemf_enabled=true; sampling_configure();
        mock_read_failure=failure==0U; mock_stop_failure=failure==1U;
        mock_enable_failure=failure==2U; mock_driver_failure=failure==3U;
        CHECK(!board_motor_start(&output,sample_callback));
        CHECK(!g_motor_status.running && !g_motor_status.calibrated && !(timer.BDTR&TIM_BDTR_MOE));
        CHECK(gpio.BSRR==(GPIO_PIN_9<<16U) && adc.IER==0U && adc2.IER==0U);
    }
    reset(0,false); CHECK(!board_motor_start(NULL,sample_callback));
    CHECK(strcmp(g_motor_status.fault,"OUTPUT CONFIG")==0);
}
/* 诊断读取晚于100us不应影响控制；ADC仍立即收到每次原始样本。 */
static unsigned bemf_callbacks,speed_callbacks;
/* 模拟慢速毫秒时钟，独立于主循环执行。 */
static bool preempt_speed_with_trip;
static board_motor_observation_t received_frame;
/** @brief 捕获ADC回调参数，验证其独立于后台读取。 @param frame 真实板卡帧。 */
static void capture_bemf(const board_motor_observation_t *frame)
{
    ++bemf_callbacks; received_frame=*frame;
}
/** @brief 模拟ADC在速度计算期间抢占并过流关断，确认后续TIM不会重新开启输出。 */
static void capture_speed(void)
{
    ++speed_callbacks;
    if (preempt_speed_with_trip)
    {
        trip_on_current=true;
        ADC2->JDR1=2222U;
        ADC2->ISR=ADC_ISR_JEOS;
        ADC1_2_IRQHandler();
    }
}
/** @brief 真实中断即时回调、序号回绕、主循环停摆、1ms分频及实际COM期限。 */
static void test_irq_callbacks_and_schedule(void)
{
    reset(PWM_ARR,true); g_motor_bemf_enabled=true;
    g_motor_output=(board_motor_output_t){0,1,board_motor_duty_ticks(200U),true};
    g_motor_next=g_motor_output; g_motor_bemf_window=true;
    bemf_callbacks=0U;
    board_motor_set_control_callbacks(capture_bemf,capture_speed);
    /* RUN期间不能替换回调，配置仅在停机装入。 */
    CHECK(g_motor_bemf_callback==NULL);
    g_motor_status.running=false;
    board_motor_set_control_callbacks(capture_bemf,capture_speed);
    g_motor_status.running=true;
    g_motor_observation.sequence=UINT32_MAX-18U;
    uint32_t expected=g_motor_observation.sequence;
    for (unsigned i=0U;i<300U;++i)
    {
        g_motor_clock_us=i*PWM_PERIOD_US+30U; g_motor_clock_cycle=dwt.CYCCNT;
        g_motor_bemf_sample_us=i*PWM_PERIOD_US+24U; ADC1->JDR1=i;
        bemf_publish(dwt.CYCCNT);
        CHECK(received_frame.sequence==++expected && received_frame.raw==i);
        CHECK(received_frame.sector==0U && received_frame.floating==2U);
        CHECK(received_frame.now_us-received_frame.sample_us==6U);
        CHECK(g_motor_status.running && bemf_callbacks==i+1U);
    }
    /* 没有出队操作、没有32项积压停机；读诊断晚于100us也不会触发故障。 */
    g_motor_clock_us+=5000U;
    CHECK(board_motor_observation().raw==299U && g_motor_status.running);
    reset(PWM_ARR,true); g_motor_bemf_enabled=true;
    g_motor_output=(board_motor_output_t){0,1,board_motor_duty_ticks(200U),true};
    g_motor_next=g_motor_output;
    speed_callbacks=0U; preempt_speed_with_trip=false;
    g_motor_speed_callback=capture_speed;
    for (unsigned i=1U;i<=2U*SPEED_LOOP_DIVIDER;++i)
    {
        DWT->CYCCNT+=PWM_PERIOD_US*TIMER_TICKS_US;
        g_motor_last_fast=g_motor_last_bemf=DWT->CYCCNT;
        TIM1_UP_TIM16_IRQHandler();
        CHECK(speed_callbacks==i/SPEED_LOOP_DIVIDER);
        CHECK(g_motor_status.running);
    }
    CHECK(g_motor_status.speed_count==2U);
    preempt_speed_with_trip=true;
    for (unsigned i=0U;i<SPEED_LOOP_DIVIDER;++i)
    {
        DWT->CYCCNT+=PWM_PERIOD_US*TIMER_TICKS_US;
        g_motor_last_fast=g_motor_last_bemf=DWT->CYCCNT;
        TIM1_UP_TIM16_IRQHandler();
    }
    CHECK(!g_motor_status.running && !(TIM1->BDTR&TIM_BDTR_MOE));
    CHECK(strcmp(g_motor_status.fault,"CURRENT TRIP")==0);
    preempt_speed_with_trip=false;
    /* 母线停采保护由TIM执行：100ms边界允许，超过边界即关断。 */
    reset(PWM_ARR,true); g_motor_bemf_enabled=true;
    g_motor_output=(board_motor_output_t){0,1,850,true}; g_motor_next=g_motor_output;
    mock_tick_ms=MOTOR_BUS_STALE_MS; g_motor_speed_divider=SPEED_LOOP_DIVIDER-1U;
    g_motor_last_fast=g_motor_last_bemf=DWT->CYCCNT;
    TIM1_UP_TIM16_IRQHandler(); CHECK(g_motor_status.running);
    mock_tick_ms=MOTOR_BUS_STALE_MS+1U; g_motor_speed_divider=SPEED_LOOP_DIVIDER-1U;
    DWT->CYCCNT+=PWM_PERIOD_US*TIMER_TICKS_US;
    g_motor_last_fast=g_motor_last_bemf=DWT->CYCCNT;
    TIM1_UP_TIM16_IRQHandler();
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"BUS STALE")==0);
    reset(PWM_ARR,true); g_motor_bemf_enabled=true;
    g_motor_clock_us=20U; g_motor_output=(board_motor_output_t){0,1,board_motor_duty_ticks(200U),true};
    g_motor_next=g_motor_output;
    uint32_t commit=0U;
    CHECK(board_motor_schedule(1U,2U*PWM_PERIOD_US+25U,200U,&commit));
    CHECK(commit==3U*PWM_PERIOD_US && g_motor_controlled && g_motor_schedule_pending);
    for (unsigned i=1U;i<=3U;++i)
    {
        dwt.CYCCNT+=(i==1U?PWM_PERIOD_US-20U:PWM_PERIOD_US)*TIMER_TICKS_US;
        g_motor_last_fast=g_motor_last_bemf=dwt.CYCCNT;
        TIM1_UP_TIM16_IRQHandler();
        CHECK(g_motor_status.running && g_motor_sector==(i<3U?0U:1U));
        CHECK(g_motor_schedule_pending==(i<3U));
    }
    CHECK(g_motor_committed_us==commit && g_motor_status.peak_count==3U);
    CHECK(!board_motor_schedule(2U,commit,200U,&commit));
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"BEMF SCHEDULE")==0);
    /* 跳过应提交的峰顶，不能到下一峰顶补做；也不能在ON窗口迟到COM。 */
    reset(PWM_ARR,true); g_motor_bemf_enabled=true;
    g_motor_output=(board_motor_output_t){0,1,850,true}; g_motor_next=g_motor_output;
    CHECK(board_motor_schedule(1U,PWM_PERIOD_US,200U,&commit));
    dwt.CYCCNT+=2U*PWM_PERIOD_US*TIMER_TICKS_US;
    g_motor_last_fast=g_motor_last_bemf=dwt.CYCCNT;
    TIM1_UP_TIM16_IRQHandler();
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"BEMF LATE")==0 && g_motor_sector==0U);
    reset(850U,true); g_motor_bemf_enabled=true;
    g_motor_next=(board_motor_output_t){0,1,850,true}; g_motor_last_fast=g_motor_last_bemf=dwt.CYCCNT;
    TIM1_UP_TIM16_IRQHandler();
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"COM DEADLINE")==0);
    /* 后台停摆不触发BEMF WORKER LATE；真实转换超时仍由TIM保护。 */
    reset(PWM_ARR,true); g_motor_bemf_enabled=true;
    g_motor_output=(board_motor_output_t){0,1,850,true}; g_motor_next=g_motor_output;
    bemf_publish(DWT->CYCCNT);
    DWT->CYCCNT+=(2U*PWM_PERIOD_US+1U)*TIMER_TICKS_US;
    g_motor_last_fast=g_motor_last_bemf=DWT->CYCCNT;
    TIM1_UP_TIM16_IRQHandler();
    CHECK(g_motor_status.running);
}

/* 有感路径使用原ADC1电流拓扑；峰顶回调仍必须受COM截止和母线保护约束。 */
static unsigned encoder_peak_calls;
static bool encoder_stop_in_callback;
/* 每个峰顶可独立改变反馈决定的输出，验证留窗不重放旧扇区。 */
static board_motor_output_t encoder_requested;
/** @brief 模拟编码器给出的扇区和限幅，不访问SPI。
 * @param controlled 无感标记。 @param sector 旧扇区。 @param duty 旧幅度。 @param next 新输出。 */
static void encoder_test_peak(bool controlled,uint8_t sector,uint16_t duty,board_motor_output_t *next)
{
    (void)sector;(void)duty;CHECK(!controlled);++encoder_peak_calls;
    *next=encoder_requested;
    if(encoder_stop_in_callback) { board_motor_stop("ENCODER DATA"); }
}

/** @brief 直接执行正式TIM中断，覆盖非BEMF回调、1kHz分频、迟提交与停止后不得再COM。 */
static void test_encoder_peak_path(void)
{
    reset(PWM_ARR,true);
    encoder_peak_calls=0U;encoder_stop_in_callback=false;speed_callbacks=0U;
    encoder_requested=(board_motor_output_t){1,2,board_motor_duty_ticks(500U),true};
    g_motor_peak_callback=encoder_test_peak;g_motor_speed_callback=capture_speed;
    for(unsigned i=0U;i<2U*SPEED_LOOP_DIVIDER;++i)
    {
        dwt.CYCCNT+=PWM_PERIOD_US*TIMER_TICKS_US;g_motor_last_fast=dwt.CYCCNT;
        TIM1_UP_TIM16_IRQHandler();
        CHECK(g_motor_status.running && g_motor_output.high==1U && g_motor_output.low==2U);
    }
    CHECK(encoder_peak_calls==2U*SPEED_LOOP_DIVIDER && speed_callbacks==2U);
    CHECK(!g_motor_bemf_enabled && TIM1->CCER!=0U);
    reset(board_motor_duty_ticks(500U),true);g_motor_peak_callback=encoder_test_peak;
    g_motor_last_fast=dwt.CYCCNT;TIM1_UP_TIM16_IRQHandler();
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"COM DEADLINE")==0);
    CHECK(TIM1->EGR!=TIM_EGR_COMG);
    reset(PWM_ARR,true);g_motor_peak_callback=encoder_test_peak;encoder_stop_in_callback=true;
    g_motor_last_fast=dwt.CYCCNT;TIM1_UP_TIM16_IRQHandler();
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"ENCODER DATA")==0);
    CHECK(TIM1->CCER==0U && TIM1->EGR!=TIM_EGR_COMG);
    encoder_stop_in_callback=false;
    reset(PWM_ARR,true);g_motor_peak_callback=encoder_test_peak;
    mock_tick_ms=MOTOR_BUS_STALE_MS+1U;g_motor_speed_divider=SPEED_LOOP_DIVIDER-1U;
    g_motor_last_fast=dwt.CYCCNT;TIM1_UP_TIM16_IRQHandler();
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"BUS STALE")==0);
}

/** @brief 95%同相不COM，换相先全关闭一拍，采用新反馈且错误绝不重开功率。 */
static void test_encoder_high_duty(void)
{
    uint32_t count=PWM_ARR-3U*TIMER_TICKS_US;
    reset(count,true);g_motor_peak_callback=encoder_test_peak;encoder_stop_in_callback=false;
    g_motor_output=(board_motor_output_t){0,1,board_motor_duty_ticks(950U),true};
    encoder_requested=(board_motor_output_t){1,2,board_motor_duty_ticks(950U),true};
    g_motor_last_fast=dwt.CYCCNT;TIM1_UP_TIM16_IRQHandler();
    CHECK(g_motor_status.running && g_motor_encoder_blanked && !g_motor_output.pulse);
    CHECK((TIM1->CCER&0x555U)==0U && g_motor_status.encoder_blank_count==1U);
    /* 下一峰顶角度已经变化，应采用这次反馈的新扇区。 */
    encoder_requested.high=2U;encoder_requested.low=0U;
    dwt.CYCCNT+=PWM_PERIOD_US*TIMER_TICKS_US;g_motor_last_fast=dwt.CYCCNT;
    TIM1_UP_TIM16_IRQHandler();
    CHECK(g_motor_status.running && !g_motor_encoder_blanked && g_motor_output.pulse);
    CHECK(g_motor_output.high==2U && g_motor_output.low==0U && (TIM1->CCER&0x555U)==0x104U);
    TIM1->EGR=0U;
    dwt.CYCCNT+=PWM_PERIOD_US*TIMER_TICKS_US;g_motor_last_fast=dwt.CYCCNT;
    TIM1_UP_TIM16_IRQHandler();
    CHECK(g_motor_status.running && TIM1->EGR==0U && TIM1->CCR1==board_motor_duty_ticks(950U));
    CHECK(g_motor_status.encoder_blank_count==1U);
    /* 从95%降为低幅并换相，仍检查旧脉冲宽度，不能只比较新占空比。 */
    encoder_requested=(board_motor_output_t){1,2,board_motor_duty_ticks(200U),true};
    dwt.CYCCNT+=PWM_PERIOD_US*TIMER_TICKS_US;g_motor_last_fast=dwt.CYCCNT;
    TIM1_UP_TIM16_IRQHandler();CHECK(g_motor_encoder_blanked && (TIM1->CCER&0x555U)==0U);
    encoder_stop_in_callback=true;TIM1->EGR=0U;
    dwt.CYCCNT+=PWM_PERIOD_US*TIMER_TICKS_US;g_motor_last_fast=dwt.CYCCNT;
    TIM1_UP_TIM16_IRQHandler();
    CHECK(!g_motor_status.running && (TIM1->BDTR&TIM_BDTR_MOE)==0U && (TIM1->CCER&0x555U)==0U);
    CHECK(TIM1->EGR==0U);
    encoder_stop_in_callback=false;
    /* 关闭未满原2us保护余量或CPU本身迟到，都必须拒绝开启。 */
    reset(count,true);g_motor_peak_callback=encoder_test_peak;
    encoder_requested=(board_motor_output_t){1,2,board_motor_duty_ticks(950U),true};
    g_motor_encoder_blanked=true;g_motor_encoder_off_cycle=dwt.CYCCNT-(BEMF_EDGE_GUARD_TICKS-1U);
    g_motor_last_fast=dwt.CYCCNT;TIM1_UP_TIM16_IRQHandler();
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"COM DEADLINE")==0 && (TIM1->CCER&0x555U)==0U);
    reset(PWM_ARR/2U+BEMF_EDGE_GUARD_TICKS,true);g_motor_peak_callback=encoder_test_peak;
    g_motor_last_fast=dwt.CYCCNT;TIM1_UP_TIM16_IRQHandler();
    CHECK(!g_motor_status.running && strcmp(g_motor_status.fault,"COM DEADLINE")==0);
}

/** @brief 寄存器模型入口，不代替真实电气波形验证。 @return 全部通过为0。 */
int main(void)
{
    test_timing_parameters(); test_start_timing(); test_opportunities();
    test_commutation(); test_regular_sequence(); test_start_phase();
    test_bemf_configuration(); test_bemf_commutation(); test_bemf_interrupts();
    test_bemf_opportunities_and_timeout(); test_bemf_time_and_storage();
    test_bemf_start();
    test_irq_callbacks_and_schedule();
    test_encoder_peak_path();
    test_encoder_high_duty();
    printf("sampling/commutation at %u Hz: %u checks passed\n",MOTOR_CONTROL_HZ,checks); return 0;
}
