#include "foc_smo.h"
#include "motor_config.h"
#include "foc_math.h"
#include <math.h>
#include <stddef.h>

/* 谐波模型使用有界循环按尺寸优化，避免展开重复支路挤入参数页；执行预算须上板复核。 */
#if defined(__ARMCC_VERSION)
#define SMO_COMPACT_CODE __attribute__((minsize))
#else
#define SMO_COMPACT_CODE
#endif

/** @brief 把运行角度折回[-pi,pi]，本模块每步角增量已经有限。 */
static float MOTOR_FAST_CODE
smo_wrap(float theta)
{
    if(theta>3.14159265359f) { theta-=6.28318530718f; }
    if(theta<-3.14159265359f) { theta+=6.28318530718f; }
    return theta;
}

/** @brief 固定控制频率下初始化离散模型，接口约束见头文件。 */
bool
foc_smo_init(foc_smo_t *o,float r,float l,float dt,float gain,float boundary,
    float filter_hz,float pll_hz,float maximum_hz,float minimum_v,uint32_t sample_ticks)
{
    if(NULL==o || !isfinite(r)||!isfinite(l)||!isfinite(dt)||!isfinite(gain)||!isfinite(boundary)||
        !isfinite(filter_hz)||!isfinite(pll_hz)||!isfinite(maximum_hz)||!isfinite(minimum_v)||
        r<=0 || l<=0 || dt<=0 || gain<=0 || boundary<=0 || filter_hz<=0 || pll_hz<=0 ||
        maximum_hz<=0 || minimum_v<=0 || 6.28318530718f*maximum_hz*dt>.2f ||
        pll_hz*dt>=.02f || sample_ticks==0U || sample_ticks>=0x80000000U) { return false; }
    float a=expf(-r*dt/l), b=(1.0f-a)/r;
    if(b<=0 || !isfinite(b)) { return false; }
    *o=(foc_smo_t){.sample_s=dt,.plant_a=a,.plant_b=b,.gain_v=gain,.boundary_a=boundary,
        .lpf_a=expf(-6.28318530718f*filter_hz*dt),
        .observer_pole=a*boundary/(boundary+b*gain),
        .emf_gain_inverse=1.0f+r*boundary/gain,
        .harmonic_step=1.0f-expf(-6.28318530718f*2.0f*dt),
        .pll_kp=2.0f*.70710678118f*6.28318530718f*pll_hz,
        .pll_ki_step=39.4784176044f*pll_hz*pll_hz*dt,
        .omega_limit=6.28318530718f*maximum_hz,.bemf_min_v=minimum_v,.sample_ticks=sample_ticks};
    return true;
}

/** @brief 以旋转矩阵直接实现atan2(a*sinΩ,1-a*cosΩ)补偿，避免重复求三角函数。 */
bool MOTOR_FAST_CODE
foc_smo_compensate(float pole,float omega,float dt,float *alpha,float *beta)
{
    float x=omega*dt;
    if(NULL==alpha || NULL==beta || !isfinite(x) || fabsf(x)>.2f) { return false; }
    /* 初始化限定整个允许频段，七阶正弦/六阶余弦误差小于单精度舍入量。 */
    float x2=x*x;
    float s=x*(1.0f-x2/6.0f+x2*x2/120.0f-x2*x2*x2/5040.0f);
    float c=1.0f-x2/2.0f+x2*x2/24.0f-x2*x2*x2/720.0f;
    float imaginary=pole*s, real=1.0f-pole*c;
    /* H(z)=(1-pole)/(1-pole*z^-1)：同时恢复幅值和相位，不降低残差门限。 */
    float scale=1.0f-pole;
    float a=*alpha,b=*beta;
    *alpha=(real*a-imaginary*b)/scale;
    *beta=(imaginary*a+real*b)/scale;
    return isfinite(*alpha) && isfinite(*beta);
}

/** @brief 解隐式边界层，避免显式高滑模增益使10kHz离散电流预测振荡。
 * @param o 固定参数。 @param prediction 施加电压下预测电流。 @param measured 本次电流。
 * @param estimate 输出校正电流。 @return 本区间滑模电压。 */
static float MOTOR_FAST_CODE
smo_axis(const foc_smo_t *o,float prediction,float measured,float *estimate)
{
    float error=prediction-measured;
    float voltage=o->gain_v*error/(o->boundary_a+o->plant_b*o->gain_v);
    if(voltage>o->gain_v) { voltage=o->gain_v; }
    if(voltage<-o->gain_v) { voltage=-o->gain_v; }
    *estimate=prediction-o->plant_b*voltage;
    return voltage;
}

/** @brief 无效区间撤销资格，不以目标或强拖角补全缺失数据。 */
static bool MOTOR_FAST_CODE
smo_invalid(foc_smo_t *o)
{
    o->sample_initialized=false;
    o->pll_initialized=false;
    o->omega_rad_s=o->omega_integral=0;
    o->filtered_alpha_v=o->filtered_beta_v=o->emf_alpha_v=o->emf_beta_v=0;
    o->emf_magnitude_v=o->phase_error=o->residual_a=0;
    o->pll_theta_rad=o->sample_theta_rad=0;
    for(unsigned k=0;k<2U;++k) { o->harmonic_x_v[k]=o->harmonic_y_v[k]=0; }
    return false;
}

/** @brief 求负序二/五次基底，避免对五倍角重复调用三角函数。
 * @param c 基波余弦。 @param s 基波正弦。 @param k 0为二次、1为五次。
 * @param hc 输出谐波余弦。 @param hs 输出谐波正弦。 */
static void MOTOR_FAST_CODE
smo_harmonic_basis(float c,float s,unsigned k,float *hc,float *hs)
{
    float c2=c*c,s2=s*s;
    *hc=k?c*(16*c2*c2-20*c2+5):c2-s2;
    *hs=k?-s*(16*s2*s2-20*s2+5):-2*c*s;
}

/** @brief 用基波与谐波各自的离散滤波响应比，预测谐波在总SMO输出中的贡献。
 * @param pole 离散极点。 @param c 基波单步余弦。 @param s 基波单步正弦。
 * @param hc 谐波单步余弦。 @param hs 谐波单步正弦。
 * @param a 输入/输出矢量α。 @param b 输入/输出矢量β。 */
static void MOTOR_FAST_CODE __attribute__((noinline))
smo_filter_ratio(float pole,float c,float s,float hc,float hs,float *a,float *b)
{
    float nr=1-pole*c,ni=pole*s,dr=1-pole*hc,di=pole*hs;
    float inverse=1/(dr*dr+di*di);
    float r=(nr*dr+ni*di)*inverse,i=(ni*dr-nr*di)*inverse;
    float old=*a;
    *a=old*r-*b*i;
    *b=old*i+*b*r;
}

/** @brief 电流预测→隐式滑模→动态滤波补偿→独立PLL→采样时刻外推。 */
bool MOTOR_FAST_CODE SMO_COMPACT_CODE
foc_smo_step(foc_smo_t *o,const foc_voltage_interval_t *u,float ia,float ib)
{
    if(NULL==o) { return false; }
    if(!isfinite(ia)||!isfinite(ib)) { return smo_invalid(o); }
    if(!o->sample_initialized)
    {
        if(NULL!=u) { return smo_invalid(o); }
        o->estimated_alpha_a=o->previous_alpha_a=ia;
        o->estimated_beta_a=o->previous_beta_a=ib;
        o->sample_initialized=true;
        return true;
    }
    if(NULL==u || u->duration_ticks!=o->sample_ticks || !isfinite(u->alpha_v)||!isfinite(u->beta_v))
    { return smo_invalid(o); }
    float omega_before=o->omega_rad_s;
    /* 相邻区间中点相距完整Ts；没有使用本次电流反求本次残差。 */
    float advance=omega_before*o->sample_s;
    float hs=advance-advance*advance*advance/6.0f, hc=1.0f-advance*advance*.5f;
    float prior_emf_a=o->emf_alpha_v*hc-o->emf_beta_v*hs;
    float prior_emf_b=o->emf_beta_v*hc+o->emf_alpha_v*hs;
    float fundamental_a=prior_emf_a,fundamental_b=prior_emf_b;
    float harmonic_a[2],harmonic_b[2],basis_c[2],basis_s[2];
    float c,s;
    foc_sincos(smo_wrap(o->pll_theta_rad+advance),&c,&s);
    for(unsigned k=0;k<2U;++k)
    {
        smo_harmonic_basis(c,s,k,&basis_c[k],&basis_s[k]);
        harmonic_a[k]=o->harmonic_x_v[k]*basis_c[k]-o->harmonic_y_v[k]*basis_s[k];
        harmonic_b[k]=o->harmonic_x_v[k]*basis_s[k]+o->harmonic_y_v[k]*basis_c[k];
        prior_emf_a+=harmonic_a[k];
        prior_emf_b+=harmonic_b[k];
    }
    float residual_a=o->plant_a*o->previous_alpha_a+o->plant_b*(u->alpha_v-prior_emf_a)-ia;
    float residual_b=o->plant_a*o->previous_beta_a+o->plant_b*(u->beta_v-prior_emf_b)-ib;
    float residual=sqrtf(residual_a*residual_a+residual_b*residual_b);
    if(!isfinite(residual)) { return smo_invalid(o); }
    o->residual_a+=.02f*(residual-o->residual_a);
    /* 系数来自上一周期，当前独立残差已计算；当前样本只能学习下一次预测。
     * 原始RL反电动势减去上次基波预测后做同步学习，避免低速基波泄漏进谐波。 */
    float innovation_a=prior_emf_a+residual_a/o->plant_b-fundamental_a;
    float innovation_b=prior_emf_b+residual_b/o->plant_b-fundamental_b;
    float za=smo_axis(o,o->plant_a*o->estimated_alpha_a+o->plant_b*u->alpha_v,ia,&o->estimated_alpha_a);
    float zb=smo_axis(o,o->plant_a*o->estimated_beta_a+o->plant_b*u->beta_v,ib,&o->estimated_beta_a);
    o->previous_alpha_a=ia;
    o->previous_beta_a=ib;
    o->filtered_alpha_v=o->lpf_a*o->filtered_alpha_v+(1.0f-o->lpf_a)*za;
    o->filtered_beta_v=o->lpf_a*o->filtered_beta_v+(1.0f-o->lpf_a)*zb;
    float ea=o->filtered_alpha_v,eb=o->filtered_beta_v;
    /* 两个离散极点分别补偿；只用上次PLL估计速度，不能用强拖或目标频率。 */
    if(!foc_smo_compensate(o->lpf_a,omega_before,o->sample_s,&ea,&eb) ||
        !foc_smo_compensate(o->observer_pole,omega_before,o->sample_s,&ea,&eb)) { return smo_invalid(o); }
    ea*=o->emf_gain_inverse;
    eb*=o->emf_gain_inverse;
    for(unsigned k=0;k<2U;++k)
    {
        float ca,sa;
        foc_sincos((k?-5.0f:-2.0f)*advance,&ca,&sa);
        smo_filter_ratio(o->observer_pole,hc,hs,ca,sa,&harmonic_a[k],&harmonic_b[k]);
        smo_filter_ratio(o->lpf_a,hc,hs,ca,sa,&harmonic_a[k],&harmonic_b[k]);
        ea-=harmonic_a[k];
        eb-=harmonic_b[k];
    }
    o->emf_alpha_v=ea;
    o->emf_beta_v=eb;
    o->emf_magnitude_v=sqrtf(ea*ea+eb*eb);
    if(!isfinite(o->emf_magnitude_v)) { return smo_invalid(o); }
    ++o->updates;
    if(o->emf_magnitude_v<o->bemf_min_v)
    {
        o->pll_initialized=false;
        o->omega_rad_s=o->omega_integral=0;
        o->phase_error=0;
        for(unsigned k=0;k<2U;++k) { o->harmonic_x_v[k]=o->harmonic_y_v[k]=0; }
        return true;
    }
    if(!o->pll_initialized)
    {
        o->pll_theta_rad=atan2f(-ea,eb);
        o->omega_rad_s=o->omega_integral=0;
        o->pll_initialized=true;
    }
    float theta=smo_wrap(o->pll_theta_rad+o->omega_integral*o->sample_s);
    float cosine,sine;
    foc_sincos(theta,&cosine,&sine);
    float error=-(ea*cosine+eb*sine)/o->emf_magnitude_v;
    o->phase_error=error;
    /* 独立估速低于25Hz时PLL自然频率减半，25..50Hz连续恢复；不对显示RPM做平滑。
     * 低速反电动势小，原固定20Hz把采样噪声放大为速度纹波；积分/比例同步缩放保留阻尼。
     * 只使用上次独立PLL速度，禁止以目标或强拖频率设置跟踪状态。 */
    float bandwidth=fmaxf(.5f,fminf(1.0f,fabsf(omega_before)*.00318309886184f));
    float kp=o->pll_kp*bandwidth;
    float integrated=o->omega_integral+o->pll_ki_step*bandwidth*bandwidth*error;
    if(integrated>o->omega_limit) { integrated=o->omega_limit; }
    if(integrated<-o->omega_limit) { integrated=-o->omega_limit; }
    o->omega_integral=integrated;
    o->omega_rad_s=integrated+kp*error;
    if(fabsf(o->omega_rad_s)>o->omega_limit) { return smo_invalid(o); }
    o->pll_theta_rad=smo_wrap(theta+kp*error*o->sample_s);
    /* LPF补偿到区间中点，另加半周期才到采样时刻；不重复补偿低通相位。 */
    o->sample_theta_rad=smo_wrap(o->pll_theta_rad+o->omega_rad_s*o->sample_s*.5f);
    if(fabsf(error)<.35f && fabsf(o->omega_rad_s)>125.663706144f)
    {
        for(unsigned k=0;k<2U;++k)
        {
            float x=o->harmonic_x_v[k]+o->harmonic_step*(innovation_a*basis_c[k]+innovation_b*basis_s[k]-o->harmonic_x_v[k]);
            float y=o->harmonic_y_v[k]+o->harmonic_step*(-innovation_a*basis_s[k]+innovation_b*basis_c[k]-o->harmonic_y_v[k]);
            /* 二次最多基波3%、五次6%，各自硬封顶0.25V；不能用任意模型误差伪造基波。
             * 2Hz慢学习，保持60mA残差/0.35相位与突变失锁保护原门限。 */
            float limit=fminf(.25f,o->emf_magnitude_v*(k?.06f:.03f));
            float norm=x*x+y*y;
            if(norm>limit*limit)
            {
                float scale=limit/sqrtf(norm);
                x*=scale; y*=scale;
            }
            o->harmonic_x_v[k]=x;
            o->harmonic_y_v[k]=y;
        }
    }
    else
    {
        /* 20Hz以下不学习分离支路；低速观测不足时逐步退去历史系数，避免伪谐波拖动PLL。 */
        for(unsigned k=0;k<2U;++k)
        {
            o->harmonic_x_v[k]*=1-o->harmonic_step;
            o->harmonic_y_v[k]*=1-o->harmonic_step;
        }
    }
    return isfinite(o->sample_theta_rad);
}
