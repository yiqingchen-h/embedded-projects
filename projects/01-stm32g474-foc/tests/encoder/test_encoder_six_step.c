#include "encoder_six_step.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* 只模拟机械位置和时间，运行正式算法，不访问板卡。 */
static unsigned g_checks;
#define CHECK(value) do { ++g_checks; assert(value); } while (0)
/* 复用原方向/零点间填充，不改变已归档七点SRAM字段的起始偏移。 */
_Static_assert(sizeof(encoder_six_config_t)==44U,"encoder config layout");
_Static_assert(offsetof(encoder_six_config_t,zero_decideg)==38U,"encoder zero layout");

/** @brief 提供7极对、600RPM、5秒及60%幅度上限的测试配置。 @return 配置。 */
static encoder_six_config_t config(void)
{
    return (encoder_six_config_t){.tick_hz=170000000U,.pwm_hz=20000U,.pole_pairs=7U,.run_ticks=100000U,
        .target_rpm=600U,.ramp_millirpm_s=171428U,.kp_milli=200U,.ki_milli=1000U,
        .max_duty=600U,.calibration_duty=300U,.direction=1,.zero_decideg=1234U,.hard_rpm=2000U};
}

/** @brief 电角度归一化。 @param degrees 角度。 @return 0..360。 */
static double normalize(double degrees)
{
    degrees = fmod(degrees, 360.0);
    return degrees < 0.0 ? degrees + 360.0 : degrees;
}

/** @brief 用独立机械轨迹验证两方向、0/360回绕及电压矢量领先角。 */
static void test_commutation(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
    {
        encoder_six_config_t c=config(); c.direction=(int8_t)direction;
        encoder_six_step_t s; CHECK(encoder_six_step_start(&s,&c,false));
        uint32_t tick=UINT32_MAX-20000U;
        double mechanical=2097000.0;
        encoder_six_feedback_t input={0};
        unsigned changes=0U;
        uint8_t previous=0U;
        for (unsigned i=0U;i<10000U;++i)
        {
            tick+=8500U;
            mechanical+=direction*(double)ENCODER_SIX_TURN*10.0/20000.0;
            if (i%4U==0U)
            {
                double wrapped=fmod(mechanical,ENCODER_SIX_TURN);
                if(wrapped<0) { wrapped+=ENCODER_SIX_TURN; }
                input=(encoder_six_feedback_t){(uint32_t)wrapped,tick,input.sequence+1U,true};
            }
            encoder_six_step_tick(&s,&input,tick);
            CHECK(s.running && s.sector<6U);
            double rotor=normalize(direction*input.raw*360.0*7.0/ENCODER_SIX_TURN-123.4);
            double lead=normalize(-30.0+60.0*s.sector-rotor);
            CHECK(lead>=57.8 && lead<=122.2);
            if(i>0U && s.sector!=previous) { ++changes; }
            previous=s.sector;
            if(i%20U==19U) { encoder_six_step_speed(&s,600U,false); }
            CHECK(s.duty<=600U);
        }
        CHECK(changes>180U && s.speed_valid && s.rpm>=599 && s.rpm<=600);
    }
}

/** @brief 验证陈旧、损坏、超速、时限和未取得首帧时全关闭。 */
static void test_protection(void)
{
    encoder_six_config_t c=config();
    encoder_six_step_t s;
    encoder_six_feedback_t input={100000U,0U,1U,true};
    CHECK(encoder_six_step_start(&s,&c,false));
    encoder_six_step_tick(&s,&input,0U);
    encoder_six_step_tick(&s,&input,51001U);
    CHECK(!s.running && strcmp(s.fault,"ENCODER STALE")==0 && s.duty==0U);
    CHECK(encoder_six_step_start(&s,&c,false));
    input.valid=false;
    encoder_six_step_tick(&s,&input,0U);
    CHECK(!s.running && strcmp(s.fault,"ENCODER DATA")==0);
    CHECK(encoder_six_step_start(&s,&c,false));
    input=(encoder_six_feedback_t){0};
    for(unsigned i=0U;i<20U;++i) { encoder_six_step_tick(&s,&input,i*8500U); CHECK(s.duty==0U); }
    CHECK(!s.running && strcmp(s.fault,"ENCODER STALE")==0);
    CHECK(encoder_six_step_start(&s,&c,false));
    input=(encoder_six_feedback_t){100000U,0U,1U,true};
    encoder_six_step_tick(&s,&input,0U);
    input=(encoder_six_feedback_t){120000U,34000U,2U,true};
    encoder_six_step_tick(&s,&input,34000U);
    CHECK(!s.running && strcmp(s.fault,"ENCODER OVERSPEED")==0);
    c.run_ticks=40U;
    CHECK(encoder_six_step_start(&s,&c,false));
    for(unsigned i=0U;i<40U;++i)
    {
        input=(encoder_six_feedback_t){100000U,i*8500U,i+1U,true};
        encoder_six_step_tick(&s,&input,i*8500U);
    }
    CHECK(!s.running && s.fault==NULL && s.ticks==40U);
    c=config();c.target_rpm=10U;
    CHECK(encoder_six_step_start(&s,&c,false));
    c.direction=0;CHECK(!encoder_six_step_start(&s,&c,false));
    c=config();c.pole_pairs=0U;CHECK(!encoder_six_step_start(&s,&c,false));
    c=config();c.max_duty=951U;CHECK(!encoder_six_step_start(&s,&c,false));
}

/** @brief 模拟七点定位，覆盖方向反向、跨零及完全不动的假标定拒绝。 */
static void test_calibration(void)
{
    for(int direction=-1;direction<=1;++direction)
    {
        encoder_six_config_t c=config();
        encoder_six_step_t s;CHECK(encoder_six_step_start(&s,&c,true));
        int64_t position=2096000;
        encoder_six_feedback_t input={0};
        for(unsigned i=0U;i<43000U && s.running;++i)
        {
            unsigned index=i/6000U;
            if(index>6U) { index=6U; }
            int64_t target=2096000+(int64_t)direction*index*ENCODER_SIX_TURN/42;
            int64_t difference=target-position;
            position+=difference>300?300:difference< -300?-300:difference;
            if(i%4U==0U)
            {
                input=(encoder_six_feedback_t){(uint32_t)position&(ENCODER_SIX_TURN-1U),i*8500U,input.sequence+1U,true};
            }
            encoder_six_step_tick(&s,&input,i*8500U);
            CHECK(s.duty<=300U);
        }
        CHECK(!s.running && s.calibration_complete && s.fault==NULL);
        bool valid=encoder_six_step_calibration_result(&s);
        if(direction==0) { CHECK(!valid && strcmp(s.fault,"ENC CAL DIRECTION")==0); }
        else
        {
            CHECK(valid && s.config.direction==direction);
            double expected=normalize(direction*2096000.0*7.0*360.0/ENCODER_SIX_TURN+30.0);
            double error=normalize(s.config.zero_decideg*.1-expected+180.0)-180.0;
            CHECK(fabs(error)<.06);
        }
    }
}

/** @brief 软限流冻结积分，反转及堵转及时关闭。 */
static void test_speed(void)
{
    encoder_six_config_t c=config();encoder_six_step_t s;
    CHECK(encoder_six_step_start(&s,&c,false));
    s.feedback_seen=s.speed_valid=true;s.rpm=100;
    for(unsigned i=0;i<2000U;++i) { encoder_six_step_speed(&s,600U,true);CHECK(s.duty<=600U); }
    CHECK(s.running && s.integral_microduty==0);
    s.rpm=-50;
    for(unsigned i=0;i<10U;++i) { encoder_six_step_speed(&s,600U,false); }
    CHECK(!s.running && strcmp(s.fault,"ENCODER REVERSE")==0);
    CHECK(encoder_six_step_start(&s,&c,false));s.feedback_seen=true;
    for(unsigned i=0;i<2000U && s.running;++i) { encoder_six_step_speed(&s,600U,false); }
    CHECK(!s.running && strcmp(s.fault,"ENCODER STALL")==0);
}

/** @brief 真实20ms数据仅累加；速度宽除法必须等COM之后1kHz服务才发生。 */
static void test_speed_after_commit(void)
{
    encoder_six_config_t c=config();encoder_six_step_t s;
    CHECK(encoder_six_step_start(&s,&c,false));
    for(unsigned i=0U;i<=400U;++i)
    {
        encoder_six_feedback_t input={(uint32_t)((uint64_t)i*ENCODER_SIX_TURN/2000U),i*8500U,i+1U,true};
        encoder_six_step_tick(&s,&input,i*8500U);
    }
    CHECK(s.running && !s.speed_valid && s.speed_ticks>=c.tick_hz/50U);
    encoder_six_step_speed(&s,600U,false);
    CHECK(s.speed_valid && s.rpm>=599 && s.rpm<=600 && s.speed_ticks==0U);
}

/** @brief 95%参数有效且PI严格受其限制，超过95%仍拒绝。 */
static void test_high_duty_limit(void)
{
    encoder_six_config_t c=config();encoder_six_step_t s;
    c.max_duty=c.calibration_duty=950U;
    CHECK(encoder_six_step_start(&s,&c,true));
    encoder_six_feedback_t input={12345U,0U,1U,true};
    for(unsigned i=0U;i<600U;++i)
    {
        input.tick=i*8500U;++input.sequence;
        encoder_six_step_tick(&s,&input,input.tick);
    }
    CHECK(s.running && s.duty==950U);
    CHECK(encoder_six_step_start(&s,&c,false));
    s.feedback_seen=s.speed_valid=true;s.rpm=100;
    for(unsigned i=0U;i<10000U;++i)
    {
        encoder_six_step_speed(&s,1800U,false);
        CHECK(s.duty<=950U && s.running);
    }
    CHECK(s.duty==950U);
}

/** @brief 使用独立静摩擦/惯性模型复现旧启动等待，验证补偿缩短起转时间且无幅度跳变。
 * @param boost 是否启用新启动补偿。 @return 模型首次达到30RPM的毫秒数。 */
static unsigned simulate_startup(bool boost)
{
    encoder_six_config_t c=config();c.startup_boost=boost;
    encoder_six_step_t s;CHECK(encoder_six_step_start(&s,&c,false));
    encoder_six_feedback_t input={0};
    double rpm=0.0,position=123456.0;
    unsigned first_motion=0U,first_continuous=0U;
    for(unsigned i=0U;i<26000U;++i)
    {
        /* 静止时幅度不足10%不转；运动后为有阻尼的一阶机械模型，非真实电机辨识。 */
        double equilibrium=2.0*s.duty-100.0;
        if(equilibrium<0.0) { equilibrium=0.0; }
        if(rpm>0.01 || s.duty>=100U) { rpm+=(equilibrium-rpm)/1200.0; }
        position+=rpm*ENCODER_SIX_TURN/(60.0*20000.0);
        if(first_motion==0U && rpm>=30.0) { first_motion=(i+1U)/20U; }
        if(first_continuous==0U && s.duty>=92U) { first_continuous=(i+1U)/20U; }
        if(i%4U==0U) { input=(encoder_six_feedback_t){(uint32_t)position&(ENCODER_SIX_TURN-1U),i*8500U,input.sequence+1U,true}; }
        encoder_six_step_tick(&s,&input,i*8500U);
        if(i%20U==19U)
        {
            bool starting=s.startup_active;uint16_t previous=s.duty;
            encoder_six_step_speed(&s,600U,false);
            CHECK(s.duty<=previous+2U && s.duty<=c.max_duty);
            if(starting && !s.startup_active)
            {
                CHECK(s.duty+2U>=previous && s.startup_motion_windows==2U);
                CHECK(s.startup_end_ticks>s.motion_ticks && s.startup_end_ticks-s.motion_ticks<=420U);
            }
        }
        CHECK(s.running && s.fault==NULL);
    }
    CHECK(first_motion!=0U);
    if(boost)
    {
        CHECK(first_continuous<=50U && first_motion<160U);
        CHECK(!s.startup_active && s.startup_end_ticks<4000U);
    }
    printf("encoder startup model boost=%u: >=9.2%% at %ums, >=30RPM at %ums\n",boost?1U:0U,first_continuous,first_motion);
    return first_motion;
}

/** @brief 无反馈、限流、单个运动窗口、幅度上限与启动超时都不能被补偿绕过。 */
static void test_startup_guards(void)
{
    encoder_six_config_t c=config();c.startup_boost=true;
    encoder_six_step_t s;
    CHECK(encoder_six_step_start(&s,&c,false));
    encoder_six_step_speed(&s,600U,false);
    CHECK(s.duty==0U);
    for(unsigned i=0U;i<6000U && s.running;++i)
    {
        encoder_six_feedback_t input={100000U,i*8500U,i+1U,true};
        encoder_six_step_tick(&s,&input,input.tick);
        if(i%20U==19U) { encoder_six_step_speed(&s,600U,false); }
    }
    CHECK(!s.running && strcmp(s.fault,"ENCODER START")==0 && s.duty==0U);
    CHECK(encoder_six_step_start(&s,&c,false));s.feedback_seen=true;
    for(unsigned i=0U;i<60U;++i)
    {
        encoder_six_step_speed(&s,600U,true);
        CHECK(s.integral_microduty==0 && s.duty<ENCODER_SIX_START_DUTY);
    }
    CHECK(encoder_six_step_start(&s,&c,false));s.feedback_seen=s.speed_valid=true;
    for(unsigned i=0U;i<60U;++i) { encoder_six_step_speed(&s,600U,false); }
    CHECK(s.duty==ENCODER_SIX_START_DUTY && s.startup_active);
    /* 一次正向速度窗不能认定连续转动；下一静止窗必须撤销候选。 */
    s.speed_ticks=c.tick_hz/50U;s.speed_delta=20972;s.ticks=1600U;
    encoder_six_step_speed(&s,600U,false);
    CHECK(s.startup_active && s.startup_motion_windows==1U && s.motion_ticks==1600U);
    s.speed_ticks=c.tick_hz/50U;s.speed_delta=0;s.ticks=2000U;
    encoder_six_step_speed(&s,600U,false);
    CHECK(s.startup_active && s.startup_motion_windows==0U && s.motion_ticks==0U);
    /* 运行中降低目标立即取消补偿，不向小目标持续推入12%幅度。 */
    encoder_six_step_speed(&s,10U,false);
    CHECK(!s.startup_active && s.duty<ENCODER_SIX_START_DUTY);
    c.max_duty=c.calibration_duty=50U;
    CHECK(encoder_six_step_start(&s,&c,false));s.feedback_seen=true;
    for(unsigned i=0U;i<100U;++i) { encoder_six_step_speed(&s,600U,false);CHECK(s.duty<=50U); }
    c=config();c.startup_boost=true;c.target_rpm=10U;
    CHECK(encoder_six_step_start(&s,&c,false) && !s.startup_active);
    c.target_rpm=600U;
    CHECK(encoder_six_step_start(&s,&c,true) && !s.startup_active);
    CHECK(simulate_startup(true)<simulate_startup(false)/3U);
}

/** @brief 运行真实有感换相、标定和保护回归。 @return 成功返回0。 */
int main(void)
{
    test_commutation();test_protection();test_calibration();test_speed();test_speed_after_commit();test_high_duty_limit();
    test_startup_guards();
    printf("encoder six-step: %u checks passed\n",g_checks);
    return 0;
}
