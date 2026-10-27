#include "encoder_foc.h"
#include "motor_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 独立轨迹/RL模型直接运行正式控制函数，不访问板卡。 */
static unsigned g_checks;
#define CHECK(v) do { ++g_checks; if(!(v)) { fprintf(stderr,"haptic line %d: %s\n",__LINE__,#v); exit(1); } } while(0)
#define TURN 2097152LL

/** @brief 运行固定输入的1kHz控制，跨配置渐变后读取稳态转矩。
 * @param s 状态。 @param c 完整配置。 @param delta 首拍实际位移。 @param rpm 机械速度。 @return 最终Iq。 */
static float
settle(encoder_haptic_t *s,encoder_haptic_config_t *c,int32_t delta,float rpm)
{
    float iq=0;
    for(unsigned k=0;k<600;++k) { iq=encoder_haptic_tick(s,c,k==0?delta:0,k*170000U,.001f,rpm,.5f); }
    return iq;
}

/** @brief 检查阻尼双向无源性、静止撤销、限幅及快速反向。 */
static void
test_damping(void)
{
    encoder_haptic_t s={0};encoder_haptic_config_t c={ENCODER_HAPTIC_DAMPED,24,100,2000};
    CHECK(encoder_haptic_valid(&c));
    CHECK(settle(&s,&c,0,0)==0);
    CHECK(fabsf(settle(&s,&c,10000,20)+.0396f)<.00001f);
    CHECK(s.counts==10000 && s.index==0);
    float iq=encoder_haptic_tick(&s,&c,-100,5,.001f,-20,.5f);
    CHECK(iq>=0 && iq<=.00201f);
    CHECK(fabsf(settle(&s,&c,-10000,-20)-.0396f)<.00001f);
    CHECK(settle(&s,&c,0,0)==0);
    CHECK(fabsf(settle(&s,&c,0,1000)+.5f)<.00001f);
    c.damping_ua_rpm=0;CHECK(settle(&s,&c,0,100)==0);
    c.detents=1;CHECK(!encoder_haptic_valid(&c));c.detents=257;CHECK(!encoder_haptic_valid(&c));
    c.detents=24;c.detent_ma=801;CHECK(!encoder_haptic_valid(&c));
}

/** @brief 检查真实整数多圈、半档回差、无积分死区和在线重新对齐。
 * @param n 每圈档数，覆盖极端和默认值。 */
static void
test_detents(int n)
{
    encoder_haptic_t s={0};encoder_haptic_config_t c={ENCODER_HAPTIC_DETENT,n,100,0};
    CHECK(settle(&s,&c,77777,0)==0 && s.counts==0);
    int32_t pitch=(int32_t)(TURN/n);
    float iq=settle(&s,&c,pitch/4,0);
    CHECK(iq<-.01f && iq>-.06f && s.index==0);
    CHECK(settle(&s,&c,-pitch/4,0)==0);
    (void)settle(&s,&c,(int32_t)(.54f*pitch),0);CHECK(s.index==0);
    (void)settle(&s,&c,(int32_t)(.03f*pitch),0);CHECK(s.index==1);
    (void)settle(&s,&c,(int32_t)(-.08f*pitch),0);CHECK(s.index==1);
    (void)settle(&s,&c,(int32_t)(-.06f*pitch),0);CHECK(s.index==0);
    /* 每拍跨至多半圈原码，原始回绕由采集模块展开；累计跨多圈不丢档。 */
    for(unsigned k=0;k<10000;++k) { (void)encoder_haptic_tick(&s,&c,(int32_t)(TURN/8),k,.001f,100,.5f); }
    CHECK(llabs(s.center_counts-s.counts)<(int64_t)(.55*TURN/n)+2);
    CHECK(s.center_counts==s.anchor_counts+s.index*TURN/n);
    for(unsigned k=0;k<20000;++k) { (void)encoder_haptic_tick(&s,&c,(int32_t)(-TURN/8),k,.001f,-100,.5f); }
    CHECK(s.counts<0 && s.index<0 && s.center_counts==s.anchor_counts+s.index*TURN/n);
    c.detents=n==24?256:24;
    float previous=s.iq_a;
    iq=encoder_haptic_tick(&s,&c,0,7,.001f,0,.5f);
    CHECK(s.center_counts==s.counts && fabsf(iq-previous)<=.00201f);
    CHECK(settle(&s,&c,0,0)==0);
    c.kind=ENCODER_HAPTIC_DAMPED;CHECK(settle(&s,&c,pitch,0)==0);
    c.kind=ENCODER_HAPTIC_DETENT;CHECK(settle(&s,&c,0,0)==0 && s.center_counts==s.counts);
}

/** @brief 用真实编码器/PWM延迟和RL植物检查手拨双向/静止、实际Iq及原保护。
 * @param direction 电角标定方向。 @param kind 连续阻尼或档位吸附。 */
static void
test_foc(int direction,int kind)
{
    encoder_foc_config_t c={.drive={.resistance_ohm=10.1f,.inductance_h=.0015f,
        .current_limit_a=.5f,.current_bandwidth_hz=800,.target_rpm=1,.pole_pairs=7,
        .speed_kp=.0001f,.speed_ki=.0002f,.speed_ramp_rpm_s=500},
        .direction=(int8_t)direction,.zero_decideg=498,.haptic_mode=true,
        .haptic={kind,24,100,2000}};
    encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));
    float alpha=0,beta=0,va=0,vb=0;double raw=2000000;
    uint16_t ccr[3]={4250,4250,4250};
    float error_peak=0,error_square=0;unsigned error_count=0;
    for(unsigned k=1;k<=90000;++k)
    {
        float rpm=k<30000?30:k<60000?-30:0;
        raw+=rpm*direction*TURN/600000.0;
        if(raw<0) { raw+=TURN; } if(raw>=TURN) { raw-=TURN; }
        for(unsigned sub=0;sub<10;++sub)
        { alpha+=(va-10.1f*alpha)*(.00001f/.0015f);beta+=(vb-10.1f*beta)*(.00001f/.0015f); }
        uint32_t tick=k*17000U;
        encoder_six_feedback_t f={(uint32_t)raw,tick-100U,k,true};
        CHECK(encoder_foc_sample(&s,&f,tick,tick-100U,alpha,beta,12,k,.000125f,ccr));
        encoder_foc_commit(&s,k);
        if(k%10==0)
        {
            encoder_foc_speed_tick(&s,1,encoder_foc_take_speed(&s));
            CHECK(s.running && s.integral_a==0 && s.current.reference_d==0 && s.current.trapezoidal);
            CHECK(s.reverse_ms==0 && s.stall_ms==0 && !s.low_position_active && !s.startup_precharge);
            CHECK(fabsf(s.reference_q_a)<=.50001f);
            if(k%30000>2000)
            {
                if(kind==ENCODER_HAPTIC_DAMPED) { CHECK(s.reference_q_a*rpm<=.00001f); }
                /* 机械环刚发布的是下一拍目标；电流只与已送入dq环的参考比较。 */
                float error=fabsf(s.current.iq_a-s.current.reference_q);
                error_peak=fmaxf(error_peak,error);error_square+=error*error;++error_count;
                if(kind==ENCODER_HAPTIC_DAMPED) { CHECK(error<.008f); }
                CHECK(fabsf(s.current.id_a)<.008f);
            }
        }
        float a=ccr[0]*(12.0f/8500),b=ccr[1]*(12.0f/8500),d=ccr[2]*(12.0f/8500);
        va=(2*a-b-d)/3;vb=(b-d)*.57735026919f;
    }
    printf("haptic dq kind=%d dir=%d tracking peak=%.6f rms=%.6f A\n",kind,direction,error_peak,sqrtf(error_square/error_count));
    CHECK(error_peak<.065f && sqrtf(error_square/error_count)<.012f);
    CHECK(fabsf(s.current.iq_a)<.004f && s.haptic.counts!=0);
    encoder_six_feedback_t bad={0,90001U*17000U-100U,90001U,false};
    CHECK(!encoder_foc_sample(&s,&bad,bad.tick+100U,bad.tick,0,0,12,90001U,.000125f,ccr));
    CHECK(strcmp(s.fault,"ENCODER DATA")==0 && s.reference_q_a==0);
    c.current_mode=true;CHECK(!encoder_foc_init(&s,&c));
    c.current_mode=false;c.position_mode=true;CHECK(!encoder_foc_init(&s,&c));
}

/** @brief 正常手拨必须形成完整正反吸附力，不能被启动用慢斜坡磨平。
 * @param n 档数。 @param rpm 实际手拨速度。 */
static void
test_click_bandwidth(int n,float rpm)
{
    encoder_haptic_t s={0};encoder_haptic_config_t c={ENCODER_HAPTIC_DETENT,n,100,0};
    (void)settle(&s,&c,0,0);
    float low=0,high=0,spring_low=0,spring_high=0,previous=0;double counts=0;int64_t last=0;
    for(unsigned k=0;k<5000;++k)
    {
        counts+=rpm*TURN/60000.0;int64_t raw=(int64_t)llround(counts);
        float iq=encoder_haptic_tick(&s,&c,(int32_t)(raw-last),k,.001f,rpm,.5f);last=raw;
        CHECK(fabsf(iq-previous)<=.200001f);previous=iq;
        if(k>1000)
        {
            low=fminf(low,iq);high=fmaxf(high,iq);
            float elastic=iq-s.damping_iq_a;
            spring_low=fminf(spring_low,elastic);spring_high=fmaxf(spring_high,elastic);
        }
    }
    printf("haptic click N=%d rpm=%.1f range=%.4f..%.4f A\n",n,rpm,low,high);
    /* 内部弹性D项产生真实速度相关偏置；单独检查吸附力形仍有双向明显幅度。 */
    CHECK(spring_low<-.07f && spring_high>.07f && low<-.07f && high>.03f);
}

/** @brief 独立机械惯量/摩擦模型检查推离档中心后松手的吸附和末段稳定性。
 * @param n 档数。 @param inertia 模型转动惯量kg*m2，不能冒充实板辨识。 */
static void
test_release_plant(int n,float inertia)
{
    encoder_haptic_t s={0};encoder_haptic_config_t c={ENCODER_HAPTIC_DETENT,n,100,2000};
    (void)settle(&s,&c,0,0);
    float theta=.4f*6.28318530718f/n,omega=0,iq=0,low=1000,high=-1000,peak_rpm=0;
    int64_t last=0;
    for(unsigned k=0;k<10000;++k)
    {
        int64_t counts=(int64_t)llround(theta*(TURN/6.28318530718));
        float request=encoder_haptic_tick(&s,&c,(int32_t)(counts-last),k,.001f,omega*9.54929658551f,.5f);last=counts;
        /* 电流并非理想即时执行，保留1ms转矩建立滞后；机械植物不使用目标角。 */
        iq+=(request-iq)*.63212056f;
        float friction=.00002f*omega+.00005f*tanhf(omega/.1f);
        omega+=(.08f*iq-friction)*(.001f/inertia);theta+=omega*.001f;
        CHECK(fabsf(iq)<=.50001f && fabsf(omega)<200 && s.transition_s>=.199f);
        if(k>9000) { low=fminf(low,theta);high=fmaxf(high,theta);peak_rpm=fmaxf(peak_rpm,fabsf(omega)*9.54929658551f); }
    }
    printf("haptic release N=%d J=%.7f pp=%.6fdeg peak=%.5fRPM\n",n,inertia,(high-low)*57.2957795f,peak_rpm);
    CHECK((high-low)*57.2957795f<.15f && peak_rpm<.3f);
}

/** @brief 真实反馈略快于5kHz时仍保留1kHz外环相位，不能每次等六帧才计算。 */
static void
test_outer_rate(void)
{
    encoder_haptic_t s={0};encoder_haptic_config_t c={ENCODER_HAPTIC_DETENT,24,100,2000};
    unsigned updates=0;float previous=0;
    for(unsigned k=0;k<1000;++k)
    {
        (void)encoder_haptic_tick(&s,&c,0,k*33830U,.000199f,0,.5f);
        if(s.transition_s!=previous) { ++updates;previous=s.transition_s; }
    }
    CHECK(updates>=197 && updates<=200 && s.transition_s>.198f && s.iq_a==0);
}

/** @brief 以真实5kHz的64码阶梯检查慢转阻尼、长驻点重启与双向停转。
 * @param target 独立机械速度RPM，不作为算法输入。 */
static void
test_quantized_damping(float target)
{
    encoder_haptic_t s={0};encoder_haptic_config_t c={ENCODER_HAPTIC_DAMPED,24,100,10000};
    double position=0,sum=0,square=0;int64_t previous=0;unsigned count=0;
    for(unsigned k=0;k<45000;++k)
    {
        if(k>=25000) { position+=target*TURN/300000.0; }
        int64_t raw=(int64_t)floor(position/64)*64;
        int32_t delta=(int32_t)(raw-previous);previous=raw;
        float rpm=encoder_haptic_velocity(&s,delta,.0002f);
        float iq=encoder_haptic_tick(&s,&c,delta,k,.0002f,rpm,.5f);
        CHECK(iq*target<=.000001f && s.index==0);
        if(k<25000) { CHECK(iq==0); }
        if(k>30000) { sum+=iq;square+=iq*iq;++count; }
    }
    double expected=-copysign(fmax(0,fabs(target)-.2)*.01,target);
    double mean=sum/count,rms=sqrt(fmax(0,square/count-mean*mean));
    printf("haptic quantized %.2fRPM mean=%.7fA ac_rms=%.7fA\n",target,mean,rms);
    CHECK(fabs(mean-expected)<fmax(.0001,fabs(expected)*.025));
    CHECK(rms<(fabsf(target)<=5?.0015:.0025));
    /* 停转保留原噪声范围，不能把上一轮速度长时间带到驻点。 */
    int64_t last=previous;
    for(unsigned k=0;k<500;++k)
    {
        int64_t raw=last+(k%2?32:-32);
        int32_t delta=(int32_t)(raw-previous);previous=raw;
        float rpm=encoder_haptic_velocity(&s,delta,.0002f);
        (void)encoder_haptic_tick(&s,&c,delta,k,.0002f,rpm,.5f);
        if(k>100) { CHECK(s.iq_a==0); }
    }
}

/** @brief 包含双相ADC/PWM量化及一拍延迟的零请求电流，不以滤波后显示作为断言。
 * @param theta 独立物理电角rad。 */
static void
test_zero_current_noise(float theta)
{
    encoder_foc_config_t c={.drive={.resistance_ohm=10.1f,.inductance_h=.0015f,
        .current_limit_a=.5f,.current_bandwidth_hz=800,.target_rpm=600,.pole_pairs=7,
        .speed_kp=.0001f,.speed_ki=.0002f,.speed_ramp_rpm_s=500},
        .direction=1,.haptic_mode=true,.haptic={ENCODER_HAPTIC_DAMPED,24,100,10000}};
    encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));
    double ia=0,ib=0,va=0,vb=0,square=0;unsigned count=0;uint16_t out[3];
    for(unsigned k=1;k<=30000;++k)
    {
        uint32_t tick=k*17000U;
        encoder_six_feedback_t f={(uint32_t)(theta*TURN/(6.28318530718*7)),tick-100,k,true};
        double b=(-ia+sqrt(3)*ib)*.5;
        double a_measured=round((ia+.0005*sin(k*1.0826))/.000805)*.000805;
        double b_measured=round((b+.0005*sin(k*.8817))/.000805)*.000805;
        CHECK(encoder_foc_sample(&s,&f,tick,tick-100,(float)a_measured,
            (float)((a_measured+2*b_measured)/sqrt(3)),12,k,.000125f,out));
        encoder_foc_commit(&s,k);
        for(unsigned sub=0;sub<10;++sub)
        { ia+=(va-10.1*ia)/.0015*.00001;ib+=(vb-10.1*ib)/.0015*.00001; }
        va=12.0*(2*out[0]-out[1]-out[2])/(3*8500);
        vb=12.0*(out[1]-out[2])/(sqrt(3)*8500);
        if(k>10000) { double q=-ia*sin(theta)+ib*cos(theta);square+=q*q;++count; }
    }
    double rms=sqrt(square/count);
    printf("haptic zero current theta=%.3f actual_q_rms=%.9fA\n",theta,rms);
    CHECK(rms<.0008 && s.reference_q_a==0);
}

/** @brief 新模式边界及实际电流模型入口。 @return 全通过为0。 */
int
main(void)
{
    test_damping();test_detents(2);test_detents(24);test_detents(256);test_outer_rate();
    test_foc(1,ENCODER_HAPTIC_DAMPED);test_foc(-1,ENCODER_HAPTIC_DAMPED);
    test_foc(1,ENCODER_HAPTIC_DETENT);test_foc(-1,ENCODER_HAPTIC_DETENT);
    test_click_bandwidth(24,60);test_click_bandwidth(256,10);
    for(int sign=-1;sign<=1;sign+=2)
    {
        const float speeds[]={.5f,1,2,5,10,30};
        for(unsigned i=0;i<sizeof(speeds)/sizeof(*speeds);++i) { test_quantized_damping(sign*speeds[i]); }
    }
    test_zero_current_noise(0);test_zero_current_noise(.3f);test_zero_current_noise(1.1f);
    for(int n=2;n<=256;n=n==2?24:256)
    {
        test_release_plant(n,.00001f);test_release_plant(n,.00006f);
        if(n==256) { break; }
    }
    printf("encoder haptic: %u checks passed\n",g_checks);return 0;
}
