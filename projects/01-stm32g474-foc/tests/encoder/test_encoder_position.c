#include "encoder_foc.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned checks;
#define CHECK(x) do { ++checks; assert(x); } while(0)

/** @brief 直接输入跨圈原码，核对机械坐标、方向、掉线与零点重建。 */
static void coordinates(void)
{
    encoder_position_t p={0};
    encoder_position_sample(&p,2097100U,0xffff0000U,true,false);
    CHECK(encoder_position_zero(&p,1));
    encoder_position_sample(&p,48U,0x00019810U,true,false);
    CHECK(encoder_position_counts(&p)==100 && p.zero_valid);
    for(unsigned k=1U;k<=6000U;++k)
    {
        encoder_position_sample(&p,(48U+k*1000U)&2097151U,0x19810U+k*170000U,true,false);
    }
    CHECK(encoder_position_counts(&p)==6000100);
    CHECK(encoder_position_zero(&p,-1));
    encoder_position_sample(&p,(p.raw-1000U)&2097151U,p.tick+170000U,true,false);
    CHECK(encoder_position_counts(&p)==1000);
    encoder_position_sample(&p,p.raw,p.tick+3400000U,true,false);
    CHECK(!p.zero_valid);
    CHECK(encoder_position_zero(&p,1));
    encoder_position_sample(&p,0,p.tick+170000U,false,false);
    CHECK(!p.valid && !p.zero_valid);
    CHECK(encoder_position_target_counts(720000)==4194304);
    CHECK(encoder_position_target_counts(-90000)==-524288);
    CHECK(encoder_position_mdeg(-4194304)==-720000);
}

/** @brief 位置开放原G2208的0.8A上限，越界拒绝；CURRENT仍保持独立0.5A契约。 */
static void current_limits(void)
{
    encoder_foc_config_t c={.drive={.resistance_ohm=10.1f,.inductance_h=.0015f,
        .current_limit_a=.8f,.current_bandwidth_hz=800,.target_rpm=600,.pole_pairs=7,
        .speed_kp=.001333f,.speed_ki=.266667f,.speed_ramp_rpm_s=300},
        .direction=1,.zero_decideg=513,.position_mode=true,
        .position_kp=8,.position_speed_limit_rpm=60,.position_tolerance_mdeg=100};
    encoder_foc_t s;
    CHECK(encoder_foc_init(&s,&c));
    c.drive.current_limit_a=.801f;
    CHECK(!encoder_foc_init(&s,&c));
    c.drive.current_limit_a=.8f;c.position_mode=false;c.current_mode=true;
    CHECK(!encoder_foc_init(&s,&c));
    c.drive.current_limit_a=.5f;
    CHECK(encoder_foc_init(&s,&c));
}

/** @brief 原始坐标控制完整dq/RL/机械模型，电流来自最终PWM电压而非参考指令。 */
static void plant(int direction)
{
    encoder_foc_config_t c={.drive={.resistance_ohm=10.1f,.inductance_h=.0015f,
        .current_limit_a=.5f,.current_bandwidth_hz=800,.target_rpm=600,.pole_pairs=7,
        .speed_kp=.001333f,.speed_ki=.266667f,.speed_ramp_rpm_s=300},
        .direction=(int8_t)direction,.zero_decideg=513,.position_mode=true,
        .position_kp=8,.position_speed_limit_rpm=60,.position_tolerance_mdeg=100};
    encoder_foc_t s;
    CHECK(encoder_foc_init(&s,&c));
    double angle=0,omega=0,ia=0,ib=0;
    encoder_six_feedback_t frame={0};
    const int32_t targets[]={0,90000,-90000,720000,-1000,0};
    for(unsigned segment=0;segment<6;++segment)
    {
        encoder_foc_position_target(&s,targets[segment]);
        double peak_error=0;
        for(unsigned j=0;j<70000U;++j)
        {
            unsigned k=segment*70000U+j+1U;
            uint32_t tick=k*17000U;
            if(k%2U==1U)
            {
                double raw=fmod(direction*(angle+c.zero_decideg*3.141592653589793/1800.0/7),6.283185307179586);
                if(raw<0) { raw+=6.283185307179586; }
                frame=(encoder_six_feedback_t){(uint32_t)(raw*ENCODER_SIX_TURN/6.283185307179586),tick-100U,(k+1U)/2U,true};
            }
            uint16_t out[3];
            CHECK(encoder_foc_sample(&s,&frame,tick,tick-100U,(float)ia,(float)ib,12,k,.000125f,out));
            encoder_foc_commit(&s,k);
            if(k%10U==0U)
            {
                s.position_counts=(int64_t)llround(angle*ENCODER_SIX_TURN/6.283185307179586);
                s.position_valid=true;
                encoder_foc_speed_tick(&s,600,encoder_foc_take_speed(&s));
            }
            CHECK(s.running && fabsf(s.reference_q_a)<=.500001f && s.current.reference_d==0);
            CHECK(fabsf(s.reference_rpm)<=60.0001f);
            double va=12.0*(2*out[0]-out[1]-out[2])/(3*8500.0);
            double vb=12.0*(out[1]-out[2])/(1.7320508075688772*8500.0);
            for(unsigned sub=0;sub<10;++sub)
            {
                double theta=angle*7,ea=-omega*7*.007619047619*sin(theta),eb=omega*7*.007619047619*cos(theta);
                ia+=(va-10.1*ia-ea)/.0015*.00001;
                ib+=(vb-10.1*ib-eb)/.0015*.00001;
                double iq=-ia*sin(theta)+ib*cos(theta);
                /* 末段加入恒定外载，微死区不能靠清空承载电流获得安静。 */
                double torque=.08*iq-4e-5*omega-(segment==5?.0015:0);
                double friction=.00002;
                if(fabs(omega)<.01 && fabs(torque)<=friction) { omega=0; }
                else { torque-=friction*(omega>.01?1:omega<-.01?-1:torque>=0?1:-1);omega+=torque/1e-6*.00001; }
                angle+=omega*.00001;
            }
            if(j>=60000U) { peak_error=fmax(peak_error,fabs(angle*180/3.141592653589793-targets[segment]*.001)); }
        }
        printf("position plant dir=%d target=%ld peak_error=%.6f arrived=%lu\n",direction,(long)targets[segment],peak_error,(unsigned long)s.position_settle_ms);
        CHECK(peak_error<.1 && s.position_settle_ms==200U);
        if(segment==5) { CHECK(s.reference_q_a>.01f); }
    }
    s.position_valid=false;
    encoder_foc_speed_tick(&s,600,(encoder_foc_speed_window_t){0,170000});
    CHECK(!s.running && strcmp(s.fault,"POSITION LOST")==0);
}

/** @brief 无运动仍关断，持续远离目标的双向实际位移也必须关断。 */
static void protections(void)
{
    encoder_foc_config_t c={.drive={.resistance_ohm=10.1f,.inductance_h=.0015f,
        .current_limit_a=.5f,.current_bandwidth_hz=800,.target_rpm=600,.pole_pairs=7,
        .speed_kp=.001333f,.speed_ki=.266667f,.speed_ramp_rpm_s=300},
        .direction=1,.zero_decideg=513,.position_mode=true,.target_position_mdeg=90000,
        .position_kp=8,.position_speed_limit_rpm=60,.position_tolerance_mdeg=100};
    for(unsigned scenario=0;scenario<3;++scenario)
    {
        encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));
        s.feedback_seen=s.closed_seen=s.position_valid=true;
        for(unsigned k=0;k<6000U&&s.running;++k)
        {
            int32_t delta=scenario==0?0:scenario==1?-2097:2097;
            s.position_counts+=delta;
            encoder_foc_speed_tick(&s,600,(encoder_foc_speed_window_t){delta,170000});
            CHECK(fabsf(s.reference_q_a)<=.500001f && fabsf(s.integral_a)<=.500001f);
        }
        CHECK(!s.running && strcmp(s.fault,scenario==0?"ENCODER START":"POSITION ESCAPE")==0);
    }
}

/** @brief 模拟手动快速正反拖至150度并返回；原50ms方向判据会误停，有限偏离应保持通电。 */
static void bounded_backdrive(void)
{
    encoder_foc_config_t c={.drive={.resistance_ohm=10.1f,.inductance_h=.0015f,
        .current_limit_a=.5f,.current_bandwidth_hz=800,.target_rpm=600,.pole_pairs=7,
        .speed_kp=.001333f,.speed_ki=.266667f,.speed_ramp_rpm_s=300},
        .direction=1,.zero_decideg=513,.position_mode=true,
        .position_kp=8,.position_speed_limit_rpm=60,.position_tolerance_mdeg=100};
    for(int sign=-1;sign<=1;sign+=2)
    {
        encoder_foc_t s;CHECK(encoder_foc_init(&s,&c));
        s.feedback_seen=s.closed_seen=s.position_valid=true;
        int64_t previous=0;
        for(unsigned k=0;k<6000U;++k)
        {
            /* 60RPM持续反拖至150度、保持、复位，三次完整动作不重置目标。 */
            unsigned phase=k%2000U;
            int64_t position=phase<417U?(int64_t)phase*2097:
                phase<1000U?416LL*2097:phase<1417U?(1416LL-phase)*2097:0;
            position*=sign;
            /* 正式板卡在速度回调之前提供本次原始机械坐标。 */
            s.position_counts=position;
            encoder_foc_speed_tick(&s,600,(encoder_foc_speed_window_t){(int32_t)(position-previous),170000});
            previous=position;
            CHECK(s.running && fabsf(s.reference_q_a)<=.500001f && fabsf(s.integral_a)<=.500001f);
        }
        CHECK(s.reverse_ms==0U);
    }
}

int main(void)
{
    coordinates();current_limits();plant(1);plant(-1);protections();bounded_backdrive();
    printf("position: %u checks passed\n",checks);
    return 0;
}
