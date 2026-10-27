#include "foc_smo.h"
#include "foc_math.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief 返回[-pi,pi]角度差，用于跨回绕相位验收。 */
static float
phase_difference(float a,float b) { return atan2f(sinf(a-b),cosf(a-b)); }

/** @brief 连续扫过启动、允许运行及额外裕量频段；不是三个RPM验收点的固定补偿。 */
static void
check_filter(void)
{
    float pole=expf(-6.28318530718f*500*.0001f),maximum=0;
    for(unsigned point=0;point<=3000;++point)
    {
        float omega=6.28318530718f*point*.1f;
        float a=0,b=0,theta=0;
        for(unsigned k=0;k<600;++k)
        {
            theta+=omega*.0001f;
            a=pole*a+(1-pole)*cosf(theta);
            b=pole*b+(1-pole)*sinf(theta);
        }
        assert(foc_smo_compensate(pole,omega,.0001f,&a,&b));
        assert(fabsf(sqrtf(a*a+b*b)-1.0f)<.001f);
        float error=fabsf(phase_difference(atan2f(b,a),theta));
        if(error>maximum) { maximum=error; }
    }
    fprintf(stderr,"LPF sweep maximum phase error %.6f deg\n",(double)(maximum*57.2957795131f));
    assert(maximum<.01745329252f);
    /* 动态加减速用上一周期速度，单独验收因果补偿暂态。 */
    float a=0,b=0,theta=0,omega_previous=0,peak=0;
    for(unsigned k=0;k<160000;++k)
    {
        float hz=k<80000?k*.0015f:(160000-k)*.0015f;
        float omega=hz*6.28318530718f;
        theta+=omega*.0001f;
        a=pole*a+(1-pole)*cosf(theta);
        b=pole*b+(1-pole)*sinf(theta);
        float ca=a,cb=b;
        assert(foc_smo_compensate(pole,omega_previous,.0001f,&ca,&cb));
        if(k>1000)
        {
            float error=fabsf(phase_difference(atan2f(cb,ca),theta));
            if(error>peak) { peak=error; }
        }
        omega_previous=omega;
    }
    fprintf(stderr,"LPF causal ramp phase error %.6f deg\n",(double)(peak*57.2957795131f));
    assert(peak<.01745329252f);
}

/** @brief 已施加电压驱动含真实反电动势的RL模型，独立PLL从零速度估计。 */
static void
check_observer(float frequency)
{
    foc_smo_t observer;
    assert(foc_smo_init(&observer,.79f,.00024f,.0001f,6,.03f,500,20,300,.05f,17000));
    float alpha=0,beta=0,theta=0,omega=frequency*6.28318530718f;
    assert(foc_smo_step(&observer,NULL,alpha,beta));
    float peak_phase=0,peak_speed=0;
    for(unsigned k=0;k<30000;++k)
    {
        float middle=theta+omega*.00005f;
        float ea=-1.5f*sinf(middle),eb=1.5f*cosf(middle);
        foc_voltage_interval_t applied={.duration_ticks=17000,.alpha_v=.25f*cosf(middle),.beta_v=.25f*sinf(middle)};
        alpha=observer.plant_a*alpha+observer.plant_b*(applied.alpha_v-ea);
        beta=observer.plant_a*beta+observer.plant_b*(applied.beta_v-eb);
        theta+=omega*.0001f;
        if(theta>3.14159265f) { theta-=6.28318531f; }
        if(theta<-3.14159265f) { theta+=6.28318531f; }
        assert(foc_smo_step(&observer,&applied,alpha,beta));
        if(k>20000)
        {
            float phase=fabsf(phase_difference(observer.sample_theta_rad,theta));
            float speed=fabsf(observer.omega_rad_s-omega);
            if(phase>peak_phase) { peak_phase=phase; }
            if(speed>peak_speed) { peak_speed=speed; }
        }
    }
    fprintf(stderr,"SMO %.1f Hz phase %.4f deg speed error %.5f rad/s residual %.5f A\n",
        (double)frequency,(double)(peak_phase*57.29577951f),(double)peak_speed,(double)observer.residual_a);
    assert(peak_phase<.035f);
    assert(peak_speed<fabsf(omega)*.01f+.1f);
    assert(fabsf(observer.emf_magnitude_v-1.5f)<.02f);
    assert(observer.residual_a<.03f);
}

/** @brief 静止转子、旋转电压命令不具有转子EMF；检查伪锁定及区间错误复位。 */
static void
check_stationary(void)
{
    foc_smo_t observer;
    assert(foc_smo_init(&observer,.79f,.00024f,.0001f,6,.03f,500,20,130,.05f,17000));
    float alpha=0,beta=0;
    assert(foc_smo_step(&observer,NULL,alpha,beta));
    float peak=0;
    for(unsigned k=0;k<10000;++k)
    {
        float theta=k*.0314159265f;
        foc_voltage_interval_t applied={.duration_ticks=17000,.alpha_v=.3f*cosf(theta),.beta_v=.3f*sinf(theta)};
        /* 真实R/L偏差10%，测量含量程内噪声；不会把强拖频率传给SMO。 */
        float decay=expf(-.85f*.0001f/.00022f),scale=(1-decay)/.85f;
        alpha=decay*alpha+scale*applied.alpha_v;
        beta=decay*beta+scale*applied.beta_v;
        assert(foc_smo_step(&observer,&applied,alpha+.004f*sinf(k*1.37f),beta+.004f*cosf(k*1.91f)));
        if(observer.emf_magnitude_v>peak) { peak=observer.emf_magnitude_v; }
    }
    fprintf(stderr,"stationary rotor maximum false EMF %.5f V\n",(double)peak);
    assert(peak<.1f);
    foc_smo_t copy=observer;
    foc_voltage_interval_t last={.duration_ticks=17000,.alpha_v=.1f,.beta_v=.2f};
    assert(foc_smo_step(&observer,&last,alpha,beta));
    /* 请求电压只是无关的未来变量；改变它不影响上一电流区间结果。 */
    foc_voltage_command_t future={.bus_v=12,.ccr={425,8075,425}};
    assert(foc_voltage_prepare(&future,8500));
    assert(foc_smo_step(&copy,&last,alpha,beta));
    assert(memcmp(&observer,&copy,sizeof(observer))==0);
    last.duration_ticks=34000;
    assert(!foc_smo_step(&observer,&last,alpha,beta));
    assert(!observer.sample_initialized&&!observer.pll_initialized&&observer.emf_magnitude_v==0);
    assert(!foc_smo_step(&observer,&last,alpha,beta));
    assert(foc_smo_step(&observer,NULL,alpha,beta));
    assert(!foc_smo_step(&observer,NULL,alpha,beta));
}

/** @brief 连续启动/加减速覆盖R/L误差和采样噪声，滤波补偿只能使用独立PLL速度。 */
static void
check_ramp(float resistance,float inductance)
{
    foc_smo_t observer;
    assert(foc_smo_init(&observer,.79f,.00024f,.0001f,6,.03f,500,20,300,.05f,17000));
    float alpha=0,beta=0,theta=0,peak_phase=0,peak_speed=0,peak_residual=0;
    assert(foc_smo_step(&observer,NULL,alpha,beta));
    float decay=expf(-resistance*.0001f/inductance),b=(1-decay)/resistance;
    for(unsigned k=0;k<95000;++k)
    {
        float hz=k<50000?k*.002f:k<60000?100.0f:100.0f-(k-60000)*.002f;
        float omega=hz*6.28318530718f;
        float middle=theta+omega*.00005f;
        theta+=omega*.0001f;
        if(theta>3.14159265f) { theta-=6.28318531f; }
        float next_a=.3f*cosf(theta+1.4f),next_b=.3f*sinf(theta+1.4f);
        float ea=-omega*.0045f*sinf(middle),eb=omega*.0045f*cosf(middle);
        foc_voltage_interval_t applied={.duration_ticks=17000,
            .alpha_v=(next_a-decay*alpha)/b+ea,.beta_v=(next_b-decay*beta)/b+eb};
        alpha=next_a;
        beta=next_b;
        assert(foc_smo_step(&observer,&applied,alpha+.003f*sinf(k*1.37f),beta+.003f*cosf(k*1.91f)));
        if(k>20000)
        {
            float phase=fabsf(phase_difference(observer.sample_theta_rad,theta));
            float speed=fabsf(observer.omega_rad_s-omega)/omega;
            if(phase>peak_phase) { peak_phase=phase; }
            if(speed>peak_speed) { peak_speed=speed; }
            if(observer.residual_a>peak_residual) { peak_residual=observer.residual_a; }
        }
    }
    fprintf(stderr,"ramp R=%.3f L=%.6f phase %.3f deg speed %.2f%% residual %.4f A\n",
        (double)resistance,(double)inductance,(double)(peak_phase*57.29577951f),(double)(peak_speed*100),(double)peak_residual);
    assert(peak_phase<.0872665f);
    assert(peak_speed<.05f);
    assert(peak_residual<.05f);
}

/** @brief 独立谐波RL模型覆盖实测二/五次，学习后突加未建模扰动仍必须超过原保护门限。
 * @param hz 基波电频率Hz；对应低速和高运行档。 */
static void
check_harmonic_prediction(float hz)
{
    foc_smo_t o;
    assert(foc_smo_init(&o,.79f,.00024f,.0001f,6,.03f,500,20,300,.05f,17000));
    float ia=0,ib=0,theta=0,omega=hz*6.28318530718f;
    float amplitude=fminf(4.5f,omega*.00405f),peak_residual=0,peak_phase=0,peak_speed=0;
    assert(foc_smo_step(&o,NULL,ia,ib));
    for(unsigned n=0;n<40000;++n)
    {
        float middle=theta+omega*.00005f;
        float ea=-amplitude*sinf(middle)+.014f*amplitude*cosf(-2*middle)+.03f*amplitude*cosf(-5*middle);
        float eb=amplitude*cosf(middle)+.014f*amplitude*sinf(-2*middle)+.03f*amplitude*sinf(-5*middle);
        foc_voltage_interval_t u={.duration_ticks=17000,.alpha_v=ea+.12f*cosf(middle),.beta_v=eb+.12f*sinf(middle)};
        ia=o.plant_a*ia+o.plant_b*(u.alpha_v-ea);
        ib=o.plant_a*ib+o.plant_b*(u.beta_v-eb);
        theta=phase_difference(theta+omega*.0001f,0);
        assert(foc_smo_step(&o,&u,ia,ib));
        if(n>30000)
        {
            peak_residual=fmaxf(peak_residual,o.residual_a);
            peak_phase=fmaxf(peak_phase,fabsf(phase_difference(o.sample_theta_rad,theta)));
            peak_speed=fmaxf(peak_speed,fabsf(o.omega_rad_s-omega)/fabsf(omega));
        }
    }
    fprintf(stderr,"harmonic %.1fHz residual %.2fmA phase %.3fdeg speed %.3f%%\n",(double)hz,
        (double)(peak_residual*1000),(double)(peak_phase*57.29577951f),(double)(peak_speed*100));
    /* 谐波低速速度纹波按既有动态模型5%验收，基波相位与残差仍独立约束。 */
    assert(peak_residual<.025f && peak_phase<.035f && peak_speed<.05f);
    assert(hypotf(o.harmonic_x_v[0],o.harmonic_y_v[0])<=fminf(.25f,o.emf_magnitude_v*.03f)+.00001f);
    assert(hypotf(o.harmonic_x_v[1],o.harmonic_y_v[1])<=fminf(.25f,o.emf_magnitude_v*.06f)+.00001f);
    /* 突然增添1V直流反电动势误差；预测不得使用本周期样本立即消掉真实残差。 */
    bool tripped=false;
    for(unsigned n=0;n<100;++n)
    {
        foc_voltage_interval_t u={.duration_ticks=17000,.alpha_v=2,.beta_v=0};
        ia=o.plant_a*ia+o.plant_b*(u.alpha_v-1);
        ib=o.plant_a*ib+o.plant_b*u.beta_v;
        if(!foc_smo_step(&o,&u,ia,ib) || o.residual_a>.06f || fabsf(o.phase_error)>.35f) { tripped=true;break; }
    }
    assert(tripped);
}

/** @brief 动态滤波、估速、静止伪锁定和缺样/未来电压因果回归入口。 */
int
main(void)
{
    for(unsigned k=0;k<=100000;++k)
    {
        float angle=-6.28318530718f+k*(12.56637061436f/100000),c,s;
        foc_sincos(angle,&c,&s);
        assert(fabsf(c-cosf(angle))<.000001f&&fabsf(s-sinf(angle))<.000001f);
        assert(fabsf(c*c+s*s-1)<.000001f);
    }
    check_filter();
    /* 5400RPM对应270Hz；300Hz仅为PLL运行上界，不能保证从零瞬间捕获边界。 */
    for(unsigned hz=5;hz<=270;hz+=5) { check_observer((float)hz); }
    check_observer(-50);
    check_stationary();
    check_harmonic_prediction(25);
    check_harmonic_prediction(160);
    check_harmonic_prediction(230);
    for(unsigned r=0;r<3;++r)
    {
        for(unsigned l=0;l<3;++l) { check_ramp(.79f*(.8f+.2f*r),.00024f*(.8f+.2f*l)); }
    }
    puts("FOC SMO: implicit stability, continuous LPF compensation, independent PLL and voltage causality PASS");
    return 0;
}
