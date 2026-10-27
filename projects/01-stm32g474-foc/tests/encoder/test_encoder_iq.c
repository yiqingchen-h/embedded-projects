#include "encoder_foc.h"
#include "motor_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 正式控制算法与独立RL/机械模型；模型不代表实板辨识。 */
static unsigned g_checks;
#define CHECK(value) do { ++g_checks; if(!(value)) { fprintf(stderr,"line %d: %s\n",__LINE__,#value); exit(1); } } while(0)

/** @brief 未使用的速度目标保留1RPM，检查独立Iq不会误入低速算法。
 * @param iq 有符号A。 @return 本板测试配置。 */
static encoder_foc_config_t
config(float iq)
{
    return (encoder_foc_config_t){.drive={.resistance_ohm=10.1f,.inductance_h=.0015f,
        .current_limit_a=.5f,.current_bandwidth_hz=800,.target_rpm=1,.pole_pairs=7,
        .speed_kp=.0001f,.speed_ki=.0002f,.speed_ramp_rpm_s=500},
        .direction=1,.zero_decideg=504,.current_mode=true,.target_iq_a=iq};
}

/** @brief 非法符号范围、非数及超过0.5A的配置不能初始化。 */
static void
test_config(void)
{
    encoder_foc_t s;
    static const float valid[]={-.5f,-.1f,0,.1f,.5f};
    for(unsigned i=0;i<sizeof(valid)/sizeof(valid[0]);++i)
    {
        encoder_foc_config_t c=config(valid[i]);
        CHECK(encoder_foc_init(&s,&c));
        CHECK(s.reference_q_a==0 && !s.closed_seen && !s.low_position_active && s.integral_a==0);
    }
    static const float invalid[]={-.50001f,.50001f,NAN,INFINITY};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);++i)
    {
        encoder_foc_config_t c=config(invalid[i]);CHECK(!encoder_foc_init(&s,&c));
    }
    encoder_foc_config_t c=config(.1f);c.drive.current_limit_a=.8f;CHECK(!encoder_foc_init(&s,&c));
    c=config(.5f);c.drive.current_limit_a=.4f;CHECK(!encoder_foc_init(&s,&c));
}

/** @brief 静止RL实算PWM延迟模型，检查正负半安培、直接换向、过零和无速度积分。
 * @param raw 初始机械原码。 @param direction 标定方向。 */
static void
test_locked_current(uint32_t raw,int direction)
{
    static const float targets[]={0,.1f,.5f,-.5f,0,-.1f,.1f,0};
    encoder_foc_t s;encoder_foc_config_t c=config(0);c.direction=(int8_t)direction;
    CHECK(encoder_foc_init(&s,&c));
    uint16_t ccr[3]={4250,4250,4250};
    float alpha=0,beta=0,voltage_alpha=0,voltage_beta=0;
    for(unsigned k=1;k<=48000U;++k)
    {
        /* 保留一拍硬件作用延迟，不将新CCR提前施加。 */
        for(unsigned sub=0;sub<10U;++sub)
        {
            alpha+=(voltage_alpha-10.1f*alpha)*(.00001f/.0015f);
            beta+=(voltage_beta-10.1f*beta)*(.00001f/.0015f);
        }
        uint32_t tick=k*17000U;
        encoder_six_feedback_t f={raw,tick-100U,k,true};
        s.config.target_iq_a=targets[(k-1U)/6000U];
        CHECK(encoder_foc_sample(&s,&f,tick,tick-100U,roundf(alpha/.0008f)*.0008f,
            roundf(beta/.0008f)*.0008f,12,k,.000125f,ccr));
        CHECK(!s.current.trapezoidal && s.current.reference_d==0 && !s.startup_precharge && !s.low_position_active);
        encoder_foc_commit(&s,k);
        if(k%10U==0U)
        {
            float before=s.reference_q_a;
            encoder_foc_speed_tick(&s,1,encoder_foc_take_speed(&s));
            CHECK(s.running && fabsf(s.reference_q_a)<=.500001f);
            CHECK(fabsf(s.reference_q_a-before)<=.002001f && s.integral_a==0);
            CHECK(!s.low_position_active && s.low_trajectory_error==0 && s.reverse_ms==0 && s.stall_ms==0);
        }
        /* 由实际SVPWM比较值反算电压，避免只检查内部PI输出。 */
        float a=ccr[0]*(12.0f/8500U),b=ccr[1]*(12.0f/8500U),d=ccr[2]*(12.0f/8500U);
        voltage_alpha=(2*a-b-d)/3.0f;voltage_beta=(b-d)*.57735026919f;
        if(k%6000U>5500U)
        {
            CHECK(fabsf(s.current.iq_a-s.config.target_iq_a)<.004f);
            CHECK(fabsf(s.current.id_a)<.004f);
        }
    }
    CHECK(s.running && s.startup_end_ms==0 && s.speed_ticks==4800U);
    encoder_foc_stop(&s,NULL);CHECK(!s.running && !s.awaiting_commit && s.reference_q_a==0);
    c=config(.1f);CHECK(encoder_foc_init(&s,&c));
    CHECK(s.current.integral_q==0 && s.integral_a==0 && s.reference_q_a==0);
}

/** @brief 双向位移允许，双向原软/硬超速以及反馈损坏仍关断。
 * @param rpm 独立机械轨迹RPM。 @param fault NULL表示应正常。 */
static void
test_motion(double rpm,const char *fault)
{
    encoder_foc_t s;encoder_foc_config_t c=config(0);CHECK(encoder_foc_init(&s,&c));
    double raw=123456;encoder_six_feedback_t f={0};uint16_t out[3];
    for(unsigned k=1;k<=2000U && s.running;++k)
    {
        raw+=rpm*ENCODER_SIX_TURN/600000.0;
        while(raw<0) { raw+=ENCODER_SIX_TURN; }
        while(raw>=ENCODER_SIX_TURN) { raw-=ENCODER_SIX_TURN; }
        uint32_t tick=k*17000U;
        if(k%2U==1U) { f=(encoder_six_feedback_t){(uint32_t)raw,tick-100U,(k+1U)/2U,true}; }
        bool sample=encoder_foc_sample(&s,&f,tick,tick-100U,0,0,12,k,.000125f,out);
        if(!sample) { CHECK(fault!=NULL);break; }
        encoder_foc_commit(&s,k);
        if(k%10U==0U) { encoder_foc_speed_tick(&s,1,encoder_foc_take_speed(&s)); }
    }
    if(NULL==fault)
    {
        CHECK(s.running && fabsf(s.rpm-(float)rpm)<1);
        f.valid=false;f.sequence++;
        CHECK(!encoder_foc_sample(&s,&f,f.tick+100U,f.tick,0,0,12,2001,.000125f,out));
        CHECK(strcmp(s.fault,"ENCODER DATA")==0 && s.reference_q_a==0);
    }
    else { CHECK(!s.running && strcmp(s.fault,fault)==0 && s.reference_q_a==0); }
}

/** @brief 电压不足时检查条件抗饱和，恢复母线及反号给定后不能保留正积分。 */
static void
test_saturation_recovery(void)
{
    encoder_foc_t s;encoder_foc_config_t c=config(.5f);CHECK(encoder_foc_init(&s,&c));
    float iq=0;uint16_t out[3];
    for(unsigned k=1;k<=20000U;++k)
    {
        uint32_t tick=k*17000U;
        encoder_six_feedback_t f={s.zero_raw/7U,tick-100U,k,true};
        float bus=k<8000U?1.0f:12.0f;
        if(k==8000U) { s.config.target_iq_a=-.1f; }
        float theta=((f.raw*7U-s.zero_raw)&(ENCODER_SIX_TURN-1U))*(6.28318530718f/ENCODER_SIX_TURN);
        CHECK(encoder_foc_sample(&s,&f,tick,tick-100U,-iq*sinf(theta),iq*cosf(theta),bus,k,.000125f,out));
        encoder_foc_commit(&s,k);
        if(k%10U==0U) { encoder_foc_speed_tick(&s,1,encoder_foc_take_speed(&s)); }
        /* 低压只注入纯算法模型；实际板卡母线异常仍由原保护关断。 */
        float decay=expf(-10.1f*.0001f/.0015f);
        iq=iq*decay+s.current.command_q_v/10.1f*(1-decay);
        CHECK(fabsf(s.current.integral_q)<7 && s.integral_a==0);
    }
    CHECK(s.current.limited>0 && fabsf(iq+.1f)<.003f && s.running);
}

/** @brief 独立Iq模型测试入口。 @return 全部通过为0。 */
int main(void)
{
    test_config();
    for(int direction=-1;direction<=1;direction+=2)
    {
        test_locked_current(123456U,direction);test_locked_current(1999999U,direction);
    }
    test_motion(0,NULL);test_motion(100,NULL);test_motion(-100,NULL);
    test_motion(1950,"ENCODER SPEED RANGE");test_motion(-1950,"ENCODER SPEED RANGE");
    test_motion(2100,"ENCODER OVERSPEED");test_motion(-2100,"ENCODER OVERSPEED");
    test_saturation_recovery();
    printf("encoder independent Iq: %u checks passed\n",g_checks);
    return 0;
}
