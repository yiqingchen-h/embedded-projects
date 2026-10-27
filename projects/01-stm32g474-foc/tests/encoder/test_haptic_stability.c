#include "encoder_foc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* 同时闭合真实编码器、速度估计、dq、PWM延迟和独立机械植物，避免理想速度漏测自激。 */
#define TURN 2097152U
#define PI 3.14159265358979323846
static unsigned g_checks;
#define CHECK(v) do { ++g_checks; if(!(v)) { fprintf(stderr,"haptic stability line %d: %s\n",__LINE__,#v); exit(1); } } while(0)

/** @brief 推离档中心后释放，检查持续振荡和电流；模型不等于实板惯量辨识。
 * @param n 档数。 @param inertia 惯量kg*m2。 @param damping 粘性微安/RPM。
 * @param strength 弹性mA。 @param kind 手感。 @param noise 编码器噪声峰值原码。
 * @param limit 实例电流上限A。 @param sign 手拨方向正负1。
 * @param adc_noise 是否叠加独立ADC噪声及量化，另按实际末段速度检查振动。 */
static void
run_plant_case(int n,double inertia,int damping,int strength,int kind,int noise,float limit,int sign,bool adc_noise)
{
    encoder_foc_config_t c={.drive={.resistance_ohm=10.1f,.inductance_h=.0015f,
        .current_limit_a=limit,.current_bandwidth_hz=800,.target_rpm=600,.pole_pairs=7,
        .speed_kp=.0001f,.speed_ki=.0002f,.speed_ramp_rpm_s=500},
        .direction=1,.zero_decideg=0,.haptic_mode=true,
        .haptic={kind,n,strength,damping}};
    encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));
    double theta=0,omega=0,ia=0,ib=0,va=0,vb=0;
    double low=10000,high=-10000,peak=0,current_peak=0;
    uint16_t ccr[3]={4250,4250,4250};
    uint32_t random=72471U;
    encoder_six_feedback_t f={0};
    for(unsigned k=1;k<=100000;++k)
    {
        /* 5us独立积分RL、反电势与力矩；指令在下一次电流采样才作用于植物。 */
        for(unsigned sub=0;sub<20;++sub)
        {
            double time=((k-1)*20+sub+1)*.000005;
            double angle=7*theta,co=cos(angle),si=sin(angle);
            double bemf=.0533333333333*omega;
            ia+=(va-10.1*ia+bemf*si)*(.000005/.0015);
            ib+=(vb-10.1*ib-bemf*co)*(.000005/.0015);
            double iq=-ia*si+ib*co;
            if(time>=.8 && time<=1.0)
            {
                double u=(time-.8)/.2;
                theta=sign*.175*(2*PI/n)*(1-cos(PI*u));
                omega=sign*.175*(2*PI/n)*PI/.2*sin(PI*u);
            }
            else
            {
                double friction=.00002*omega+.00005*tanh(omega/.1);
                omega+=(.08*iq-friction)*(.000005/inertia);theta+=omega*.000005;
            }
        }
        uint32_t tick=k*17000U;
        if(k%2==0 || !f.valid)
        {
            random=random*1664525U+1013904223U;
            int32_t jitter=noise?(int32_t)(random%(unsigned)(noise*2+1))-noise:0;
            int64_t raw=llround(theta*(TURN/(2*PI)))+jitter;
            f.raw=(uint32_t)raw&(TURN-1U);f.tick=tick-100U;f.sequence=k;f.valid=true;
        }
        float measured_a=(float)ia,measured_beta=(float)ib;
        if(adc_noise)
        {
            /* 噪声场景同时量化真实两相ADC，不能用理想连续电流掩盖驻点振荡。 */
            double b=(-ia+sqrt(3)*ib)*.5;
            double a_adc=round((ia+.0005*sin(k*1.0826))/.000805)*.000805;
            double b_adc=round((b+.0005*sin(k*.8817))/.000805)*.000805;
            measured_a=(float)a_adc;measured_beta=(float)((a_adc+2*b_adc)/sqrt(3));
        }
        CHECK(encoder_foc_sample(&s,&f,tick,tick-100U,measured_a,measured_beta,12,k,.000125f,ccr));
        encoder_foc_commit(&s,k);
        CHECK(fabsf(s.reference_q_a)<=limit+.00001f);
        current_peak=fmax(current_peak,fabs(s.current.iq_a));
        if(k%10==0)
        {
            encoder_foc_speed_tick(&s,600,encoder_foc_take_speed(&s));
            CHECK(s.running && s.integral_a==0);
            if(k>80000)
            { low=fmin(low,theta);high=fmax(high,theta);peak=fmax(peak,fabs(omega)*60/(2*PI)); }
        }
        double a=ccr[0]*(12.0/8500),b=ccr[1]*(12.0/8500),d=ccr[2]*(12.0/8500);
        va=(2*a-b-d)/3;vb=(b-d)/sqrt(3);
    }
    double pp=(high-low)*180/PI;
    printf("haptic coupled N=%d J=%.8g D=%d S=%d kind=%d noise=%d adc=%u limit=%.1f dir=%d pp=%.6fdeg rpm=%.5f iq=%.6fA\n",
        n,inertia,damping,strength,kind,noise,adc_noise?1U:0U,limit,sign,pp,peak,current_peak);
    /* 原25组位移/速度判据完整保留。新增ADC噪声场景检查真实末段振速，
     * 连续阻尼不锁角，档内还有死区，不能把自由缓慢漂移当成位置跟踪失败。 */
    if(adc_noise) { CHECK(peak<.3); }
    else { CHECK(pp<(noise?.1:.15) && peak<(noise?3:.3)); }
    CHECK(current_peak<limit+.05f && s.reverse_ms==0 && s.stall_ms==0);
}

/** @brief 保留原25组植物与判据，不叠加新ADC扰动。
 * @param n 档数。 @param inertia 惯量。 @param damping 阻尼。 @param strength 力度。
 * @param kind 类型。 @param noise 编码器噪声。 @param limit 限流。 @param sign 方向。 */
static void
run_plant(int n,double inertia,int damping,int strength,int kind,int noise,float limit,int sign)
{
    run_plant_case(n,inertia,damping,strength,kind,noise,limit,sign,false);
}

/** @brief 默认、强阻尼、细档、零粘性、双向及传感噪声下松手回归。 @return 全通过为0。 */
int
main(void)
{
    for(int n=2;n<=256;n=n==2?24:256)
    {
        for(int d=0;d<=10000;d=d==0?2000:10000)
        {
            run_plant(n,.000001, d,100,ENCODER_HAPTIC_DETENT,0,.5f,1);
            run_plant(n,.00006, d,100,ENCODER_HAPTIC_DETENT,0,.5f,-1);
            if(d==10000) { break; }
        }
        if(n==256) { break; }
    }
    run_plant(24,.000001273239545,2000,100,ENCODER_HAPTIC_DETENT,32,.5f,1);
    run_plant(24,.000001,10000,100,ENCODER_HAPTIC_DETENT,32,.5f,-1);
    run_plant(256,.000001,0,800,ENCODER_HAPTIC_DETENT,32,.5f,1);
    run_plant(256,.000001,10000,800,ENCODER_HAPTIC_DETENT,0,.8f,-1);
    run_plant(256,.000001,0,800,ENCODER_HAPTIC_DETENT,0,.8f,1);
    run_plant(24,.000001,2000,100,ENCODER_HAPTIC_DAMPED,32,.5f,1);
    run_plant(24,.000001,10000,100,ENCODER_HAPTIC_DAMPED,32,.5f,-1);
    run_plant_case(24,.000001273239545,2000,100,ENCODER_HAPTIC_DETENT,32,.5f,1,true);
    run_plant_case(24,.000001,10000,100,ENCODER_HAPTIC_DETENT,32,.5f,-1,true);
    run_plant_case(256,.000001,0,800,ENCODER_HAPTIC_DETENT,32,.5f,1,true);
    run_plant_case(24,.000001,2000,100,ENCODER_HAPTIC_DAMPED,32,.5f,1,true);
    run_plant_case(24,.000001,10000,100,ENCODER_HAPTIC_DAMPED,32,.5f,-1,true);
    printf("haptic coupled stability: %u checks passed\n",g_checks);return 0;
}
