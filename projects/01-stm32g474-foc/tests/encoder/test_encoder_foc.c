#include "encoder_foc.h"
#include "foc_voltage.h"
#include "motor_config.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* 回归仅运行正式纯算法；机械模型不代表实板电机参数已辨识。 */
static unsigned g_checks;
#define CHECK(condition) do { ++g_checks; if(!(condition)) { fprintf(stderr,"line %d: %s\n",__LINE__,#condition); exit(1); } } while(0)

/** @brief 独立检查Iq约束：升流仍原斜率，快速变化只能撤销已有转矩且不越过零。
 * @param s 本拍结束状态。 @param previous 上拍Iq指令A。 */
static void check_low_iq_change(const encoder_foc_t *s,float previous)
{
    float step=fabsf(s->reference_q_a-previous);
    float original=s->startup_end_ms!=0U?.008001f:.002001f;
    CHECK(fabsf(s->reference_q_a)<=s->config.drive.current_limit_a+.000001f);
    if(step>original)
    {
        CHECK(s->startup_end_ms!=0U && step<=.064001f);
        CHECK(previous*s->reference_q_a>=0.0f && fabsf(s->reference_q_a)<fabsf(previous));
    }
}

/** @brief 构造本电机档的保守0.5A、600RPM配置。 @return 独立有感FOC配置。 */
static encoder_foc_config_t config(void)
{
    return (encoder_foc_config_t){.drive={.resistance_ohm=10.1f,.inductance_h=.0015f,
        .current_limit_a=.5f,.current_bandwidth_hz=800,.target_rpm=600,.pole_pairs=7,
        .speed_kp=.0004f,.speed_ki=.0005f,.speed_ramp_rpm_s=200},.direction=1,.zero_decideg=498};
}

/** @brief 检查无首帧不输出、真实提交资格、CRC/磁场和反馈时间保护。 */
static void test_feedback(void)
{
    encoder_foc_t s;encoder_foc_config_t c=config();uint16_t out[3];
    CHECK(encoder_foc_init(&s,&c));
    encoder_six_feedback_t f={0};
    for(unsigned i=1;i<10;++i)
    {
        CHECK(encoder_foc_sample(&s,&f,i*17000U,i*17000U-100U,0,0,12,i,.000125f,out));
        CHECK(out[0]==4250 && out[1]==4250 && out[2]==4250 && !s.closed_seen);
    }
    CHECK(!encoder_foc_sample(&s,&f,170000U,169900U,0,0,12,10,.000125f,out));
    CHECK(strcmp(s.fault,"ENCODER STALE")==0);
    CHECK(encoder_foc_init(&s,&c));
    f=(encoder_six_feedback_t){123456U,UINT32_MAX-10000U,1U,true};
    CHECK(encoder_foc_sample(&s,&f,1000U,900U,0,0,12,7,.000125f,out));
    CHECK(s.awaiting_commit && !s.closed_seen);
    encoder_foc_commit(&s,6);CHECK(!s.closed_seen);
    encoder_foc_commit(&s,7);CHECK(s.closed_seen);
    f.valid=false;f.sequence=2;
    CHECK(!encoder_foc_sample(&s,&f,2000U,1900U,0,0,12,8,.000125f,out));
    CHECK(strcmp(s.fault,"ENCODER DATA")==0 && !s.running && s.reference_q_a==0);
    CHECK(encoder_foc_init(&s,&c));f=(encoder_six_feedback_t){100U,1000U,1U,true};
    CHECK(!encoder_foc_sample(&s,&f,53000U,52900U,0,0,12,1,.000125f,out));
    CHECK(strcmp(s.fault,"ENCODER STALE")==0);
    CHECK(encoder_foc_init(&s,&c));f=(encoder_six_feedback_t){100U,1000U,1U,true};
    CHECK(encoder_foc_sample(&s,&f,2000U,1900U,0,0,12,1,.000125f,out));
    f.sequence=2;f.raw+=100000U;f.tick+=34000U;
    CHECK(!encoder_foc_sample(&s,&f,36000U,35900U,0,0,12,2,.000125f,out));
    CHECK(strcmp(s.fault,"ENCODER OVERSPEED")==0);
}

/** @brief 多电角/双方向确认Park零点及预测不混入六步90度提前。 */
static void test_angles(void)
{
    encoder_foc_config_t c=config();encoder_foc_t s;uint16_t out[3];
    for(int direction=-1;direction<=1;direction+=2)
    {
        c.direction=(int8_t)direction;
        for(unsigned raw=0;raw<ENCODER_SIX_TURN;raw+=13751U)
        {
            CHECK(encoder_foc_init(&s,&c));
            encoder_six_feedback_t f={raw,1000U,1U,true};
            uint32_t phase=raw*7U;
            if(direction<0) { phase=0U-phase; }
            phase=(phase-s.zero_raw)&(ENCODER_SIX_TURN-1U);
            float theta=phase*(6.28318530718f/ENCODER_SIX_TURN);
            CHECK(encoder_foc_sample(&s,&f,2000U,1900U,.1f*cosf(theta),.1f*sinf(theta),12,1,.000125f,out));
            CHECK(fabsf(s.current.id_a-.1f)<.00001f && fabsf(s.current.iq_a)<.00001f);
            CHECK(s.current.reference_d==0 && s.current.reference_q==0);
            CHECK(!s.current.harmonic_enabled && !s.current.harmonic2.enabled);
            CHECK(out[0]>=425 && out[0]<=8075 && out[1]>=425 && out[1]<=8075 && out[2]>=425 && out[2]<=8075);
        }
    }
}

/** @brief 无机械运动时保持电流/斜率边界，并在1秒关断而不是无限强拉。 */
static void test_stall(void)
{
    encoder_foc_config_t c=config();encoder_foc_t s;uint16_t out[3];
    CHECK(encoder_foc_init(&s,&c));
    for(unsigned k=1;k<=10010U && s.running;++k)
    {
        uint32_t tick=k*17000U;
        encoder_six_feedback_t f={123456U,tick-8500U,k,true};
        CHECK(encoder_foc_sample(&s,&f,tick,tick-100U,0,0,12,k,.000125f,out));
        encoder_foc_commit(&s,k);
        if(k%10U==0U)
        {
            float old=s.reference_q_a;
            encoder_foc_speed_tick(&s,600,encoder_foc_take_speed(&s));
            CHECK(fabsf(s.reference_q_a)<=.500001f);
            if(s.running) { CHECK(fabsf(s.reference_q_a-old)<=.002001f); }
        }
    }
    CHECK(!s.running && strcmp(s.fault,"ENCODER START")==0 && s.reference_q_a==0);
}

/** @brief 高速预充限60mA/80ms、首合格窗退出；低速快反馈PI不注入固定偏置。 */
static void test_precharge(void)
{
    encoder_foc_config_t c=config();c.drive.speed_kp=.0001f;c.drive.speed_ki=.0002f;
    encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;
    for(unsigned ms=1;ms<=81U;++ms)
    {
        float old=s.reference_q_a;
        encoder_foc_speed_tick(&s,600,(encoder_foc_speed_window_t){0,170000U});
        CHECK(s.startup_precharge==(ms<=80U));
        CHECK(s.reference_q_a-old<=.002001f && s.reference_q_a-old>=-.002001f);
        if(ms<=80U) { CHECK(s.integral_a<=.060001f); }
    }
    CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;s.motion_windows=1;
    encoder_foc_speed_tick(&s,600,(encoder_foc_speed_window_t){0,170000U});
    CHECK(!s.startup_precharge && s.integral_a<.00001f);
    CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;
    encoder_foc_speed_tick(&s,99,(encoder_foc_speed_window_t){0,170000U});
    CHECK(!s.startup_precharge && s.integral_a<.0001f);
    encoder_foc_stop(&s,"TEST");CHECK(!s.startup_precharge && s.reference_q_a==0);
}

/** @brief 快反馈不改变20ms保护速度、启动时限及原Iq斜率/相限流。 */
static void test_low_feedback(void)
{
    encoder_foc_config_t c=config();c.drive.speed_kp=.0001f;c.drive.speed_ki=.0002f;
    encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;
    encoder_foc_speed_tick(&s,100,(encoder_foc_speed_window_t){350,170000U});
    CHECK(s.control_rpm>4 && s.rpm==0 && !s.speed_valid && !s.startup_precharge);
    for(unsigned target=1;target<=200U;target=target==1U?20U:target==20U?200U:201U)
    {
        CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;
        unsigned timeout=target>=100U?1000U:5000U;
        for(unsigned ms=1;ms<=timeout;++ms)
        {
            float old=s.reference_q_a;
            encoder_foc_speed_tick(&s,target,(encoder_foc_speed_window_t){0,170000U});
            CHECK(fabsf(s.reference_q_a)<=.500001f && !s.startup_precharge);
            if(s.running) { CHECK(fabsf(s.reference_q_a-old)<=.002001f); }
            if(ms<timeout) { CHECK(s.running); }
        }
        CHECK(!s.running && strcmp(s.fault,"ENCODER START")==0 && s.reference_q_a==0);
    }
}

/** @brief 实板反转轨迹暴露的Iq斜率饱和：速度恢复后不能继续累积反向转矩。 */
static void test_iq_slew_windup(void)
{
    encoder_foc_config_t c=config();c.drive.speed_kp=.0001f;c.drive.speed_ki=.0002f;
    encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;
    s.reference_rpm=100;s.reference_q_a=.08f;s.integral_a=.08f;s.startup_end_ms=1;
    for(unsigned ms=0;ms<30U;++ms)
    {
        float old=s.reference_q_a;
        encoder_foc_speed_tick(&s,100,(encoder_foc_speed_window_t){20972,170000U});
        CHECK(s.running && fabsf(s.reference_q_a-old)<=.008001f);
    }
    float braking=s.reference_q_a;
    for(unsigned ms=0;ms<12U;++ms)
    {
        encoder_foc_speed_tick(&s,100,(encoder_foc_speed_window_t){0,170000U});
        CHECK(s.running && fabsf(s.reference_q_a)<=.500001f);
    }
    CHECK(s.reference_q_a>braking && s.integral_a>-.1f);
}

/** @brief PLL只跟踪真实位移，整数原码量化下的低速不漂移，保护仍独立取20ms速度。 */
static void test_tracking(void)
{
    static const unsigned targets[]={1,5,20,100,200};
    for(unsigned index=0;index<sizeof(targets)/sizeof(targets[0]);++index)
    {
        encoder_foc_config_t c=config();c.drive.speed_kp=.0001f;c.drive.speed_ki=.0002f;
        encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;
        double raw=0;int32_t previous=0;
        for(unsigned ms=1;ms<=1000U;++ms)
        {
            raw+=targets[index]*(ENCODER_SIX_TURN/60000.0);
            int32_t sample=(int32_t)llround(raw)+(int32_t)(ms%3U)-1;
            encoder_foc_speed_tick(&s,targets[index],(encoder_foc_speed_window_t){sample-previous,170000U});
            previous=sample;
            CHECK(s.running);
            if(ms>500U) { CHECK(fabsf(s.control_rpm-targets[index])<.06f); }
        }
        CHECK(s.startup_end_ms!=0U && fabsf(s.rpm-targets[index])<.01f);
    }
}

/** @brief 外部匀速轨迹具有0.8/1.2ms采样间隔时，PLL仍须跟踪真实速度。
 * 位置使用真实间隔独立积分并量化；原实现把上次间隔用于本次预测会产生约20%假波动。 */
static void
test_tracking_variable_interval(void)
{
    static const unsigned targets[]={1,5,20,100,200};
    for(unsigned index=0;index<sizeof(targets)/sizeof(targets[0]);++index)
    {
        encoder_foc_config_t c=config();c.drive.speed_kp=.0001f;c.drive.speed_ki=.0002f;
        encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;
        double raw=0;int32_t previous=0;
        for(unsigned ms=1;ms<=1000U;++ms)
        {
            uint32_t ticks=(ms&1U)?136000U:204000U;
            raw+=targets[index]*(ENCODER_SIX_TURN/60.0)*(ticks/170000000.0);
            int32_t sample=(int32_t)llround(raw)+(int32_t)(ms%3U)-1;
            encoder_foc_speed_tick(&s,targets[index],(encoder_foc_speed_window_t){sample-previous,ticks});
            previous=sample;
            CHECK(s.running);
            if(ms>500U)
            {
                if(fabsf(s.control_rpm-targets[index])>=.1f)
                { fprintf(stderr,"variable interval target=%u tick=%u rpm=%f\n",targets[index],ms,s.control_rpm); }
                /* 变间隔叠加±1原码噪声，绝对估速误差须小于0.1RPM。 */
                CHECK(fabsf(s.control_rpm-targets[index])<.1f);
            }
        }
        CHECK(s.startup_end_ms!=0U && fabsf(s.rpm-targets[index])<.01f);
    }
}

/** @brief 角同步学习必须在运动后、每实例独立，并同时受系数、相限流及原Iq变化率约束。 */
static void
test_ripple_bounds(void)
{
    encoder_foc_config_t c=config();c.drive.speed_kp=.000001f;c.drive.speed_ki=.000001f;
    for(unsigned low=0;low<2;++low)
    {
        c.drive.current_limit_a=low?.05f:.5f;
        encoder_foc_t s,other;CHECK(encoder_foc_init(&s,&c));CHECK(encoder_foc_init(&other,&c));
        s.feedback_seen=true;
        for(unsigned ms=0;ms<100;++ms)
        {
            encoder_foc_speed_tick(&s,100,(encoder_foc_speed_window_t){0,170000});
            CHECK(s.ripple_c[0]==0 && s.ripple_s[1]==0);
        }
        s.startup_end_ms=1;s.reference_rpm=100;
        for(unsigned ms=0;ms<200;++ms)
        {
            float old=s.reference_q_a;
            encoder_foc_speed_tick(&s,100,(encoder_foc_speed_window_t){0,170000});
            CHECK(s.running && fabsf(s.reference_q_a)<=c.drive.current_limit_a+.000001f);
            CHECK(fabsf(s.reference_q_a-old)<=.008001f);
            for(unsigned h=0;h<2;++h)
            { CHECK(hypotf(s.ripple_c[h],s.ripple_s[h])<=fminf(.04f,c.drive.current_limit_a*.08f)+.000001f); }
            CHECK(other.ripple_c[0]==0 && other.ripple_s[1]==0);
        }
        CHECK(s.ripple_c[0]>.001f);
        float saved=s.ripple_c[0];++s.current.limited;
        encoder_foc_speed_tick(&s,100,(encoder_foc_speed_window_t){0,170000});
        CHECK(s.ripple_c[0]==saved);
        CHECK(encoder_foc_init(&s,&c));CHECK(s.ripple_c[0]==0 && s.ripple_s[1]==0);
    }
}

/** @brief 独立转矩扰动模型比较角同步补偿，不冒充实板或完整电压模型。
 * @param learning 是否允许学习。 @return 末5秒真实机械速度峰峰差RPM。 */
static double
test_ripple_disturbance_model(bool learning)
{
    /* 正式分档调度以保存100/200为基准，100RPM有效增益对应诊断版30/20。 */
    encoder_foc_config_t c=config();c.drive.speed_kp=.0001f;c.drive.speed_ki=.0002f;c.drive.speed_ramp_rpm_s=500;
    encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;
    double theta=0,rpm=0,iq=0,low=1e9,high=-1e9;int previous=0;
    for(unsigned k=1;k<=120000;++k)
    {
        iq+=(s.reference_q_a-iq)*.3951;
        double force=iq-.012*sin(14*theta)-.006*sin(12*theta)-.00002*rpm;
        if(fabs(rpm)<.0001 && fabs(force)<=.003) { rpm=0; }
        else
        {
            force-=.003*(rpm>.0001?1:rpm<-.0001?-1:force>=0?1:-1);
            rpm+=392366*force*.0001;
        }
        theta+=rpm*(6.283185307179586/60)*.0001;
        if(k%10U==0U)
        {
            int raw=(int)llround(theta*ENCODER_SIX_TURN/6.283185307179586);
            s.raw=(uint32_t)raw&(ENCODER_SIX_TURN-1U);
            if(!learning) { s.ripple_frozen=true; }
            encoder_foc_speed_tick(&s,100,(encoder_foc_speed_window_t){raw-previous,170000});
            previous=raw;s.current.iq_a=(float)iq;
            CHECK(s.running && fabsf(s.reference_q_a)<=.500001f);
        }
        if(k>70000U) { if(rpm<low) { low=rpm; } if(rpm>high) { high=rpm; } }
    }
    printf("angle disturbance learning=%u tail actual rpm %.3f..%.3f\n",learning,low,high);
    return high-low;
}

/** @brief 分档节点/过渡与高档原增益，必须保持保存100/200的尺度和无突跳插值。 */
static void
test_low_gain_schedule(void)
{
    /* 1..20使用新位置LESO候选，原增益节点只检查其余段。 */
    static const float cases[][3]={{35,9,375},{50,9,250},{75,7.2f,150},{100,5.4f,50},{150,5.4f,50},
        {200,5.4f,50},{250,3.2f,25.5f},{300,1,1},{600,1,1},{1100,1,1}};
    encoder_foc_config_t c=config();c.drive.speed_kp=.0001f;c.drive.speed_ki=.0002f;c.drive.speed_ramp_rpm_s=500;
    for(unsigned i=0;i<sizeof(cases)/sizeof(cases[0]);++i)
    {
        encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;s.motion_windows=1;
        encoder_foc_speed_tick(&s,(unsigned)cases[i][0],(encoder_foc_speed_window_t){0,0});
        CHECK(fabsf(s.reference_q_a-(c.drive.speed_kp*cases[i][1]+c.drive.speed_ki*cases[i][2]*.001f)*.5f)<.0000001f);
        CHECK(fabsf(s.integral_a-c.drive.speed_ki*cases[i][2]*.0005f)<.0000001f);
    }
    encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;s.startup_end_ms=1;
    s.reference_rpm=200;
    encoder_foc_speed_tick(&s,200,(encoder_foc_speed_window_t){0,0});
    CHECK(s.ripple_c[0]==0 && s.ripple_s[1]==0);
}

/** @brief 真实SVPWM与两相ADC量化的RL模型，比较积分离散法而不使用滤波遥测。
 * @param trapezoidal 选择1RPM新入口，关闭时600RPM原电流路径。
 * @return 末1秒独立植物实际Iq相对55mA的RMS误差A。 */
static double
test_quantized_current(bool trapezoidal)
{
    encoder_foc_config_t c=config();c.drive.target_rpm=trapezoidal?1U:600U;c.zero_decideg=0;
    encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));
    double ia=0,ib=0,va=0,vb=0,error_square=0;unsigned count=0;
    encoder_six_feedback_t feedback={0};
    for(unsigned k=1;k<=20000;++k)
    {
        uint32_t tick=k*17000U;
        if(k%2U==1U) { feedback=(encoder_six_feedback_t){0U,tick-100U,(k+1U)/2U,true}; }
        s.reference_q_a=.055f;
        /* ADC先量化两相原电流，再做Clarke；约0.805mA/码不使用早期粗量化假设。 */
        double phase_b=(-ia+1.7320508075688772*ib)*.5;
        double measured_a=round((ia+.0005*sin(k*1.0826))/.000805)*.000805;
        double measured_b=round((phase_b+.0005*sin(k*.8817))/.000805)*.000805;
        float beta=(float)((measured_a+2*measured_b)/1.7320508075688772);
        uint16_t out[3];
        CHECK(encoder_foc_sample(&s,&feedback,tick,tick-100U,(float)measured_a,beta,12,k,.000125f,out));
        encoder_foc_commit(&s,k);
        CHECK(s.current.trapezoidal==trapezoidal && fabsf(s.current.iq_a-beta)<.0000001f);
        /* 独立推进上一拍实际CCR对应的电压，包含一拍延迟、PWM量化及R/L动态。 */
        for(unsigned sub=0;sub<10;++sub)
        { ia+=(va-10.1*ia)/.0015*.00001;ib+=(vb-10.1*ib)/.0015*.00001; }
        va=12.0*(2.0*out[0]-out[1]-out[2])/(3.0*8500);
        vb=12.0*(out[1]-out[2])/(1.7320508075688772*8500);
        CHECK(hypot(ia,ib)<.1);
        for(unsigned p=0;p<3;++p) { CHECK(out[p]>=425U && out[p]<=8075U); }
        if(k>10000) { error_square+=(ib-.055)*(ib-.055);++count; }
    }
    double rms=sqrt(error_square/count);
    printf("quantized current trapezoidal=%u rms=%.6fA\n",trapezoidal,rms);
    CHECK(rms<.002);
    return rms;
}

/** @brief 1RPM以外回原路径，积分模式切换不改变原带宽或积分电压。 */
static void
test_current_integrator_mode(void)
{
    encoder_foc_config_t c=config();c.drive.target_rpm=1;
    encoder_foc_t s;uint16_t out[3];CHECK(encoder_foc_init(&s,&c));
    encoder_six_feedback_t f={0U,16900U,1U,true};
    CHECK(encoder_foc_sample(&s,&f,17000U,16900U,0,0,12,1,.000125f,out));
    CHECK(s.current.trapezoidal);
    CHECK(fabsf(s.current.kp-.0015f*6.28318530718f*800)<.00001f);
    s.config.drive.target_rpm=2;
    CHECK(encoder_foc_sample(&s,&f,34000U,33900U,0,0,12,2,.000125f,out));
    CHECK(!s.current.trapezoidal);
    s.current.integral_d=.13f;s.current.integral_q=.27f;
    foc_current_trapezoidal(&s.current,true);
    CHECK(s.current.integral_d==.13f && s.current.integral_q==.27f);
    CHECK(encoder_foc_init(&s,&c));
    CHECK(!s.current.trapezoidal && s.current.previous_error_d==0 && s.current.previous_error_q==0);
}

/** @brief 独立机械/电流延迟模型检查新位置LESO、真实20ms速度与全部Iq约束。
 * @param target 真实机械目标RPM。 @param gain 未独立辨识的机械增益RPM/(A*s)。 */
static void
test_low_position_model(unsigned target,double gain)
{
    encoder_foc_config_t c=config();c.drive.speed_kp=.0001f;c.drive.speed_ki=.0002f;
    c.drive.speed_ramp_rpm_s=500;
    encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;
    double angle=.63,rpm=0,iq=0,delayed=0,raw=angle*ENCODER_SIX_TURN/6.283185307179586;
    int previous=(int)llround(raw);float low=1e9f,high=-1e9f;
    for(unsigned k=1U;k<=120000U;++k)
    {
        /* 放慢到500us电流响应，覆盖实板新记录暴露的模型/建立时间不确定性。 */
        iq+=(delayed-iq)*.181269246922;
        delayed=s.reference_q_a;
        double force=iq-.018*sin(14*angle)-.009*sin(28*angle+.3)-.004*cos(12*angle);
        if(fabs(rpm)<.001 && fabs(force)<.006) { rpm=0; }
        else
        {
            force-=.006*(rpm>.001?1:rpm<-.001?-1:force>0?1:-1);
            rpm+=gain*force*.0001;
        }
        angle+=rpm*(6.283185307179586/60)*.0001;
        if(k%10U==0U)
        {
            int sample=(int)llround(angle*ENCODER_SIX_TURN/6.283185307179586)+((k/10U)%3U)-1;
            float old=s.reference_q_a;
            s.raw=(uint32_t)sample&(ENCODER_SIX_TURN-1U);
            encoder_foc_speed_tick(&s,target,(encoder_foc_speed_window_t){sample-previous,170000U});
            previous=sample;
            CHECK(s.running && s.low_position_active && fabsf(s.reference_q_a)<=.500001f);
            check_low_iq_change(&s,old);
            CHECK(fabsf(s.low_disturbance_rpm_s)<=48000.01f);
            for(unsigned h=0;h<3;++h) { CHECK(hypotf(s.low_load_c[h],s.low_load_s[h])<=.026668f); }
            CHECK(fabsf(s.low_load_previous_a)<=.080001f);
            CHECK(fabs(rpm)<1900);
            if(k>70000U) { low=fminf(low,s.rpm);high=fmaxf(high,s.rpm); }
        }
    }
    CHECK(s.startup_end_ms>0U);
    printf("low position model target=%u gain=%.0f tail=%.3f..%.3f\n",target,gain,low,high);
    if(target==1U && gain<=900000) { CHECK(low>.94f && high<1.06f); }
    /* 每实例状态必须清零，切回高档仍走原速度环；这是模型验证，不是实板全档验收。 */
    CHECK(encoder_foc_init(&s,&c));CHECK(!s.low_position_active && s.low_disturbance_rpm_s==0);
    for(unsigned h=0;h<3;++h) { CHECK(s.low_load_c[h]==0 && s.low_load_s[h]==0); }
    CHECK(s.low_load_previous_a==0);
    s.feedback_seen=true;
    encoder_foc_speed_tick(&s,1,(encoder_foc_speed_window_t){0,170000U});
    CHECK(s.low_position_active);
    encoder_foc_speed_tick(&s,600,(encoder_foc_speed_window_t){0,170000U});
    CHECK(!s.low_position_active && s.running && fabsf(s.reference_q_a)<=.500001f);
}

/** @brief 三次增加静摩擦后突然卸载，检查原电流界限、退流方向及额外机械位移。
 * @param target 1/5/20RPM。 @param gain 独立机械增益RPM/(A*s)，覆盖模型不确定性。
 * @note 150mA等效外加摩擦与500us电流建立延迟只作回归激励，不冒充已测实板负载。
 * 卸载累计多走不得超过0.3机械圈；旧冻结/双向8mA斜率会在此模型多走一圈以上。 */
static void test_low_unload(unsigned target,double gain)
{
    encoder_foc_config_t c=config();c.drive.target_rpm=target;c.drive.speed_kp=.0001f;
    c.drive.speed_ki=.0002f;c.drive.speed_ramp_rpm_s=500;c.zero_decideg=0;
    encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;
    double angle=.63,rpm=0,iq=0,delayed=0,excess=0,peak=0;
    int previous=(int)llround(angle*ENCODER_SIX_TURN/6.283185307179586);
    unsigned fast_returns=0;
    for(unsigned k=1;k<=70000;++k)
    {
        double time=k*.0001;
        bool loading=(time>=1 && time<2.2)||(time>=3 && time<3.5)||(time>=4 && time<4.7);
        iq+=(delayed-iq)*.181269246922;delayed=s.reference_q_a;
        double force=iq-.018*sin(14*angle)-.009*sin(28*angle+.3)-.004*cos(12*angle)
            -.013*sin(36*angle+.2)-.0046*sin(84*angle+.4);
        double friction=.006+(loading?.15:0);
        if(fabs(rpm)<.001 && fabs(force)<friction) { rpm=0; }
        else
        {
            double sign=rpm>.001?1:rpm<-.001?-1:force>0?1:-1;
            double next=rpm+gain*(force-friction*sign)*.0001;
            rpm=next*rpm<0 && fabs(force)<=friction?0:next;
        }
        angle+=rpm*(6.283185307179586/60)*.0001;
        if(time>=2.2 && !loading)
        { peak=fmax(peak,rpm);excess+=fmax(0,rpm-target)*.0001; }
        if(k%10U==0U)
        {
            int sample=(int)llround(angle*ENCODER_SIX_TURN/6.283185307179586)+(int)(k/10U%3U)-1;
            s.raw=(uint32_t)sample&(ENCODER_SIX_TURN-1U);s.current.iq_a=(float)iq;
            float old=s.reference_q_a;
            encoder_foc_speed_tick(&s,target,(encoder_foc_speed_window_t){sample-previous,170000U});
            previous=sample;
            CHECK(s.running && fabsf(s.integral_a)<=.500001f);
            check_low_iq_change(&s,old);
            if(fabsf(s.reference_q_a-old)>.008001f) { ++fast_returns; }
        }
    }
    printf("unload target=%u gain=%.0f peak=%.3f extra_turns=%.6f fast_returns=%u\n",
        target,gain,peak,excess/60,fast_returns);
    CHECK(excess/60<.3 && fast_returns>0);
    CHECK(s.startup_end_ms!=0U && fabsf(s.reference_q_a)<.08f);
}

/** @brief 用独立周期负载对比同一正式控制器的纯LESO与学习预测；原速度窗不滤波。
 * @param target RPM。 @param gain 未独立辨识的机械增益RPM/(A*s)。
 * @param learning 允许学习；关闭时每拍冻结且初始系数为零。 @return 末5秒原20ms速度峰峰值。 */
static float test_low_periodic_model(unsigned target,double gain,bool learning)
{
    encoder_foc_config_t c=config();c.drive.speed_kp=.0001f;c.drive.speed_ki=.0002f;
    c.drive.speed_ramp_rpm_s=500;
    encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));s.feedback_seen=true;
    double angle=.63,rpm=0,iq=0;int previous=(int)llround(angle*ENCODER_SIX_TURN/6.283185307179586);
    float low=1e9f,high=-1e9f;
    for(unsigned k=1U;k<=120000U;++k)
    {
        iq+=(s.reference_q_a-iq)*.181269246922;
        double force=iq-.018*sin(14*angle)-.009*sin(28*angle+.3)-.004*cos(12*angle)
            -.013*sin(36*angle+.2)-.0046*sin(84*angle+.4);
        if(fabs(rpm)<.001 && fabs(force)<.006) { rpm=0; }
        else
        {
            force-=.006*(rpm>.001?1:rpm<-.001?-1:force>0?1:-1);
            rpm+=gain*force*.0001;
        }
        angle+=rpm*(6.283185307179586/60)*.0001;
        if(k%10U==0U)
        {
            int sample=(int)llround(angle*ENCODER_SIX_TURN/6.283185307179586)+((k/10U)%3U)-1;
            s.raw=(uint32_t)sample&(ENCODER_SIX_TURN-1U);
            if(!learning) { s.ripple_frozen=true; }
            float old=s.reference_q_a;
            encoder_foc_speed_tick(&s,target,(encoder_foc_speed_window_t){sample-previous,170000U});
            previous=sample;
            CHECK(s.running && fabsf(s.reference_q_a)<=.500001f);
            check_low_iq_change(&s,old);
            for(unsigned h=0;h<3;++h) { CHECK(hypotf(s.low_load_c[h],s.low_load_s[h])<=.026668f); }
            CHECK(fabsf(s.low_load_previous_a)<=.080001f);
            CHECK(fabsf(s.low_disturbance_rpm_s)<=48000.01f);
            if(k>70000U) { low=fminf(low,s.rpm);high=fmaxf(high,s.rpm); }
        }
    }
    CHECK(s.startup_end_ms>0U);
    printf("periodic load target=%u gain=%.0f learning=%u tail=%.3f..%.3f\n",target,gain,learning,low,high);
    return high-low;
}

/** @brief 验证反转/软超速延时保护，以及ADC与速度中断交换后不丢后续增量。
 * @param rpm 外部给定的真实机械轨迹。 @param fault 应触发的故障。 */
static void test_speed_fault(double rpm, const char *fault)
{
    encoder_foc_config_t c=config();encoder_foc_t s;uint16_t out[3];
    CHECK(encoder_foc_init(&s,&c));
    encoder_six_feedback_t f={0};double raw=123456;
    for(unsigned k=1;k<=2000U && s.running;++k)
    {
        uint32_t tick=k*17000U;
        raw+=rpm*ENCODER_SIX_TURN/600000.0;
        while(raw<0) { raw+=ENCODER_SIX_TURN; }
        while(raw>=ENCODER_SIX_TURN) { raw-=ENCODER_SIX_TURN; }
        if(k%2U==1U) { f=(encoder_six_feedback_t){(uint32_t)raw,tick-100U,(k+1U)/2U,true}; }
        CHECK(encoder_foc_sample(&s,&f,tick,tick-100U,0,0,12,k,.000125f,out));
        if(k%10U==0U) { encoder_foc_speed_tick(&s,600,encoder_foc_take_speed(&s)); }
    }
    CHECK(!s.running && strcmp(s.fault,fault)==0);
    CHECK(encoder_foc_init(&s,&c));
    s.speed_delta=100;s.speed_elapsed=170000;
    encoder_foc_speed_window_t window=encoder_foc_take_speed(&s);
    CHECK(window.delta==100 && window.ticks==170000U);
    s.speed_delta=27;s.speed_elapsed=34000;
    s.feedback_seen=true;
    encoder_foc_speed_tick(&s,600,window);
    CHECK(s.speed_delta==27 && s.speed_elapsed==34000U);
    CHECK(s.window_delta==100 && s.window_elapsed==170000U);
}

/** @brief 在独立机械模型下验证闭环从静止直接启动，无六步/强拖/SMO阶段。
 * @param direction 编码器方向。 @param initial 初始机械角rad。
 * @param inertia 独立机械模型惯量kg*m²。 @param kp 速度比例A/RPM。 @param ki 积分A/(RPM*s)。
 * @param ramp 机械速度参考斜坡RPM/s。 @param friction_nm 未实物辨识的静摩擦模型N*m。 */
static void test_plant(int direction, double initial, double inertia, float kp, float ki, float ramp, double friction_nm)
{
    encoder_foc_config_t c=config();c.direction=(int8_t)direction;c.zero_decideg=513;
    c.drive.speed_kp=kp;c.drive.speed_ki=ki;
    c.drive.speed_ramp_rpm_s=ramp;
    encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));
    double angle=initial,omega=0,ia=0,ib=0,peak=0;
    encoder_six_feedback_t frame={0};
    float tail_min=10000,tail_max=-10000;
    for(unsigned k=1;k<=120000;++k)
    {
        uint32_t tick=k*17000U;
        if(k%2U==1U)
        {
            double mechanical=fmod(direction*(angle+c.zero_decideg*3.141592653589793/1800.0/7),6.283185307179586);
            if(mechanical<0) { mechanical+=6.283185307179586; }
            frame=(encoder_six_feedback_t){(uint32_t)(mechanical*ENCODER_SIX_TURN/6.283185307179586),tick-100U,(k+1U)/2U,true};
        }
        uint16_t out[3];
        bool ok=encoder_foc_sample(&s,&frame,tick,tick-100U,(float)ia,(float)ib,12,k,.000125f,out);
        if(!ok) { fprintf(stderr,"plant dir=%d k=%u fault=%s rpm=%f\n",direction,k,s.fault,s.rpm); }
        CHECK(ok);
        encoder_foc_commit(&s,k);
        if(k%10U==0U) { encoder_foc_speed_tick(&s,600,encoder_foc_take_speed(&s)); }
        CHECK(s.running && s.current.reference_d==0 && fabsf(s.reference_q_a)<=.500001f);
        /* 电压使用实际SVPWM最终CCR，不用参考电流直接推动机械模型。 */
        double va=12.0*(2.0*out[0]-out[1]-out[2])/(3.0*8500.0);
        double vb=12.0*(out[1]-out[2])/(1.7320508075688772*8500.0);
        for(unsigned sub=0;sub<10;++sub)
        {
            double theta=angle*7.0,ea=-omega*7*.007619047619*sin(theta),eb=omega*7*.007619047619*cos(theta);
            ia+=(va-10.1*ia-ea)/.0015*.00001;
            ib+=(vb-10.1*ib-eb)/.0015*.00001;
            double iq=-ia*sin(theta)+ib*cos(theta);
            double torque=.08*iq-4e-5*omega;
            double friction=friction_nm;
            if(fabs(omega)<.01 && fabs(torque)<=friction) { omega=0; }
            else { torque-=friction*(omega>.01?1:omega<-.01?-1:torque>=0?1:-1);omega+=torque/inertia*.00001; }
            angle+=omega*.00001;
            if(hypot(ia,ib)>peak) { peak=hypot(ia,ib); }
        }
        if(k>=110000U) { tail_min=fminf(tail_min,s.rpm);tail_max=fmaxf(tail_max,s.rpm); }
    }
    CHECK(s.closed_seen && s.startup_end_ms>0 && s.startup_end_ms<1000U);
    CHECK(tail_min>570 && tail_max<630 && peak<.5);
    printf("plant direction=%d angle=%.2f inertia=%g kp=%g ki=%g motion=%lu rpm=%.2f..%.2f peak=%.3fA\n",
        direction,initial,inertia,kp,ki,(unsigned long)s.motion_ms,tail_min,tail_max,peak);
}

/** @brief 真实算法回归入口，不连接任何硬件。 @return 成功0。 */
int main(void)
{
    test_feedback();test_angles();test_stall();test_precharge();test_low_feedback();test_iq_slew_windup();test_tracking();
    test_tracking_variable_interval();
    test_ripple_bounds();
    test_low_gain_schedule();
    test_current_integrator_mode();
    double rectangular_noise=test_quantized_current(false);
    double trapezoidal_noise=test_quantized_current(true);
    CHECK(trapezoidal_noise<rectangular_noise);
    for(unsigned i=0U;i<4U;++i)
    {
        /* 按约537k的惯性近似选择测试范围；保留50k..400k模型作为失败候选。 */
        static const double gains[4]={300000,600000,900000,1200000};
        test_low_position_model(1,gains[i]);test_low_position_model(5,gains[i]);test_low_position_model(10,gains[i]);
        test_low_position_model(11,gains[i]);test_low_position_model(15,gains[i]);test_low_position_model(20,gains[i]);
    }
    for(unsigned i=0;i<3;++i)
    {
        static const double gains[3]={300000,600000,900000};
        test_low_unload(1,gains[i]);test_low_unload(5,gains[i]);test_low_unload(20,gains[i]);
        for(unsigned target=5;target<=20;target*=4)
        {
            float plain=test_low_periodic_model(target,gains[i],false);
            float learned=test_low_periodic_model(target,gains[i],true);
            CHECK(learned<plain*.6f);
        }
    }
    CHECK(test_ripple_disturbance_model(false)>20);
    CHECK(test_ripple_disturbance_model(true)<5);
    test_speed_fault(-100,"ENCODER REVERSE");
    test_speed_fault(1950,"ENCODER SPEED RANGE");
    /* 原400/500实板已反转，不再作为编码器正常模型的合格参数；原失败快照长期保留。 */
    test_plant(1,.3,3e-6,.0001f,.0002f,500,.0002);test_plant(1,3.1,3e-6,.0001f,.0002f,500,.0002);test_plant(-1,5.9,3e-6,.0001f,.0002f,500,.0002);
    /* 首轮实板出现反转，补查20ms速度窗与低惯量下增益过强的风险；模型惯量未作实物辨识。 */
    for(unsigned i=0;i<4;++i)
    {
        static const double inertia[4]={3e-7,5e-7,1e-6,3e-6};
        test_plant(1,.3,inertia[i],.0001f,.0002f,500,.003);
        test_plant(-1,5.9,inertia[i],.0001f,.0002f,500,.003);
    }
    printf("encoder FOC checks=%u\n",g_checks);
    return 0;
}
