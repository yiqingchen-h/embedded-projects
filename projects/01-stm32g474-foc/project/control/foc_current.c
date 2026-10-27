#include "foc_current.h"
#include "motor_config.h"
#include <math.h>
#include <stddef.h>

/* 二/五/七次同步积分增益V/(A*s)；保守学习速度不提升原PI带宽。 */
#define FOC_HARMONIC_KI 20.0f
/* 当前实测五次BEMF约0.072V，留出模型余量并独立限制新增电压。 */
#define FOC_HARMONIC_LIMIT_V .2f
/* 三轮滑行二次BEMF约0.035V，留出余量但不扩大总调制范围。 */
#define FOC_HARMONIC2_LIMIT_V .1f
/* 编译期开关用于固定二/五次及CSA增益的独立实机A/B/A；不写入保存参数。 */
#define FOC_HARMONIC7_ENABLED 1

/** @brief 从已有正余弦求三倍角，不调用三角库。
 * @param c 原角余弦。 @param s 原角正弦。
 * @param c3 三倍角余弦输出。 @param s3 三倍角正弦输出。 */
static inline void
current_three_angle(float c, float s, float *c3, float *s3)
{
    *c3=c*(c*c-3.0f*s*s);
    *s3=s*(3.0f*c*c-s*s);
}

/** @brief 更新独立0.1V谐波积分支路，共用二/七次限幅和未来输出。
 * @param h 支路状态。 @param ex 旋转后X误差A。 @param ey 旋转后Y误差A。
 * @param c 未来支路角余弦。 @param s 未来支路角正弦。 @param step 积分步长V/A。 */
/* 保留共享实现以节省Flash，实机验证此调用仍有足够提交裕量。 */
static void MOTOR_FAST_CODE __attribute__((noinline))
current_harmonic_step(foc_harmonic2_t *h,float ex,float ey,float c,float s,float step)
{
    float x=h->x_v+step*ex,y=h->y_v+step*ey;
    float magnitude=x*x+y*y;
    if (magnitude>FOC_HARMONIC2_LIMIT_V*FOC_HARMONIC2_LIMIT_V)
    {
        float fraction=FOC_HARMONIC2_LIMIT_V/sqrtf(magnitude);
        x*=fraction; y*=fraction; ++h->limited;
    }
    h->x_v=x; h->y_v=y;
    h->d_v=x*c-y*s;
    h->q_v=x*s+y*c;
}

/** @brief 以实测模型初始化固定PI参数，契约见头文件。 */
bool
foc_current_init(foc_current_t *loop, float resistance_ohm, float inductance_h,
    float bandwidth_hz, float sample_s, uint16_t arr, uint16_t minimum_ccr, uint16_t maximum_ccr)
{
    if (NULL == loop || !isfinite(resistance_ohm) || !isfinite(inductance_h) ||
        !isfinite(bandwidth_hz) || !isfinite(sample_s) || resistance_ohm<=0.0f ||
        inductance_h<=0.0f || bandwidth_hz<=0.0f || sample_s<=0.0f ||
        bandwidth_hz*sample_s>=0.1f || arr==0U || minimum_ccr>=arr/2U ||
        maximum_ccr<=arr/2U || maximum_ccr>arr) { return false; }
    float reserve_low=1.0f-2.0f*minimum_ccr/arr;
    float reserve_high=2.0f*maximum_ccr/arr-1.0f;
    *loop=(foc_current_t){.kp=inductance_h*6.28318530718f*bandwidth_hz,
        .ki_step=resistance_ohm*6.28318530718f*bandwidth_hz*sample_s,
        .harmonic_step=FOC_HARMONIC_KI*sample_s,.harmonic_limit_v=FOC_HARMONIC_LIMIT_V,
        .modulation=(reserve_low<reserve_high?reserve_low:reserve_high)*0.57735026919f,
        .arr=arr,.minimum_ccr=minimum_ccr,.maximum_ccr=maximum_ccr};
    return isfinite(loop->kp) && isfinite(loop->ki_step);
}

/** @brief 见foc_current.h；历史误差逐拍跟踪，模式切换不重置积分电压。 */
void MOTOR_FAST_CODE
foc_current_trapezoidal(foc_current_t *loop, bool enabled)
{
    loop->trapezoidal=enabled;
}

/** @brief 见头文件；只更新旋转模型系数，静态R/L已在启动阶段校验。 */
void MOTOR_FAST_CODE
foc_current_feedforward(foc_current_t *loop, float omega_rad_s,
    float ld_h, float lq_h, float flux_wb)
{
    loop->speed_ld_v_per_a=omega_rad_s*ld_h;
    loop->speed_lq_v_per_a=omega_rad_s*lq_h;
    loop->bemf_q_v=omega_rad_s*flux_wb;
}

/** @brief 见头文件；只有状态变化才复位，避免每周期重复清零。 */
void MOTOR_FAST_CODE
foc_current_harmonic(foc_current_t *loop, bool enabled)
{
    if (loop->harmonic_enabled==enabled) { return; }
    loop->harmonic_x_v=loop->harmonic_y_v=0.0f;
    loop->harmonic_d_v=loop->harmonic_q_v=0.0f;
    loop->harmonic_enabled=enabled;
    loop->harmonic7=(foc_harmonic2_t){.enabled=enabled && FOC_HARMONIC7_ENABLED};
}

/** @brief 见头文件；状态改变才清零，五次支路和原PI不变。 */
void MOTOR_FAST_CODE
foc_current_harmonic2(foc_current_t *loop, bool enabled)
{
    if (loop->harmonic2.enabled==enabled) { return; }
    loop->harmonic2.x_v=loop->harmonic2.y_v=0;
    loop->harmonic2.d_v=loop->harmonic2.q_v=0;
    loop->harmonic2.enabled=enabled;
}

/** @brief 核对动态值，PI矢量饱和时禁止继续向饱和方向积累，静态参数只在初始化验证。 */
bool MOTOR_FAST_CODE
foc_current_step(foc_current_t *loop, float alpha_a, float beta_a,
    float sample_cos, float sample_sin, float future_cos, float future_sin,
    float bus_v, float reference_d, float reference_q, uint16_t ccr[3])
{
    if (NULL == loop || NULL == ccr || !isfinite(bus_v) || bus_v<=0.0f ||
        !isfinite(alpha_a) || !isfinite(beta_a)) { return false; }
    loop->id_a=alpha_a*sample_cos+beta_a*sample_sin;
    loop->iq_a=-alpha_a*sample_sin+beta_a*sample_cos;
    loop->reference_d=reference_d;
    loop->reference_q=reference_q;
    float error_d=reference_d-loop->id_a, error_q=reference_q-loop->iq_a;
    /* Sguan_PID以相邻误差梯形积分；只由指定低速有感入口启用，原矩形路径保持。 */
    float step_d=loop->trapezoidal?.5f*(error_d+loop->previous_error_d):error_d;
    float step_q=loop->trapezoidal?.5f*(error_q+loop->previous_error_q):error_q;
    float integral_d=loop->integral_d+loop->ki_step*step_d;
    float integral_q=loop->integral_q+loop->ki_step*step_q;
    bool integral_frozen=false;
    float harmonic_x=loop->harmonic_x_v, harmonic_y=loop->harmonic_y_v;
    float second_x=loop->harmonic2.x_v, second_y=loop->harmonic2.y_v;
    float seventh_x=loop->harmonic7.x_v, seventh_y=loop->harmonic7.y_v;
    float c3=0,s3=0,fc3=0,fs3=0;
    if (loop->harmonic_enabled || loop->harmonic2.enabled)
    {
        /* 支路共用三倍角，五/七次所需六倍角再由倍角得到，降低中断开销。 */
        current_three_angle(sample_cos,sample_sin,&c3,&s3);
        current_three_angle(future_cos,future_sin,&fc3,&fs3);
    }
    if (loop->harmonic_enabled)
    {
        float c6=c3*c3-s3*s3, s6=2.0f*c3*s3;
        float fc6=fc3*fc3-fs3*fs3, fs6=2.0f*fc3*fs3;
        /* dq负六次误差旋转为直流；基波参考仍由原PI负责，不从采样电流中删谐波。 */
        harmonic_x+=loop->harmonic_step*(error_d*c6-error_q*s6);
        harmonic_y+=loop->harmonic_step*(error_d*s6+error_q*c6);
        float harmonic_magnitude2=harmonic_x*harmonic_x+harmonic_y*harmonic_y;
        if (harmonic_magnitude2>loop->harmonic_limit_v*loop->harmonic_limit_v)
        {
            float fraction=loop->harmonic_limit_v/sqrtf(harmonic_magnitude2);
            harmonic_x*=fraction;
            harmonic_y*=fraction;
            ++loop->harmonic_limited;
        }
        /* 负序五次需按未来实际作用角反变换，不能只按基波延时预测补偿。 */
        loop->harmonic_d_v=harmonic_x*fc6+harmonic_y*fs6;
        loop->harmonic_q_v=-harmonic_x*fs6+harmonic_y*fc6;
        if (loop->harmonic7.enabled)
        {
            /* 正序七次在dq为正六次，学习和未来输出的旋转方向与负序五次相反。 */
            current_harmonic_step(&loop->harmonic7,error_d*c6+error_q*s6,
                -error_d*s6+error_q*c6,fc6,fs6,loop->harmonic_step);
        }
    }
    if (loop->harmonic2.enabled)
    {
        /* 负序二次在dq中为负三次；按采样角学习，按实际未来角输出。 */
        current_harmonic_step(&loop->harmonic2,error_d*c3-error_q*s3,
            error_d*s3+error_q*c3,fc3,-fs3,loop->harmonic_step);
    }
    float requested_d=loop->kp*error_d+integral_d;
    float requested_q=loop->kp*error_q+integral_q;
    requested_d+=loop->harmonic_d_v+loop->harmonic2.d_v+loop->harmonic7.d_v;
    requested_q+=loop->harmonic_q_v+loop->harmonic2.q_v+loop->harmonic7.q_v;
    /* 同拍实际电流：-ωLqIq、+ωLdId及+ωψ；共同限幅后原抗饱和仍约束全部电压。 */
    requested_d-=loop->speed_lq_v_per_a*loop->iq_a;
    requested_q+=loop->speed_ld_v_per_a*loop->id_a+loop->bemf_q_v;
    float d=requested_d, q=requested_q;
    float radius=bus_v*loop->modulation;
    float magnitude2=d*d+q*q;
    if (!isfinite(magnitude2)) { return false; }
    if (magnitude2>radius*radius)
    {
        float fraction=radius/sqrtf(magnitude2);
        d*=fraction;
        q*=fraction;
        ++loop->limited;
        if ((requested_d-d)*error_d+(requested_q-q)*error_q>0.0f)
        {
            integral_d=loop->integral_d;
            integral_q=loop->integral_q;
            integral_frozen=true;
        }
        /* 总调制饱和时冻结额外学习，避免谐波支路积累不可施加的补偿。 */
        harmonic_x=loop->harmonic_x_v;
        harmonic_y=loop->harmonic_y_v;
        loop->harmonic2.x_v=second_x;
        loop->harmonic2.y_v=second_y;
        loop->harmonic7.x_v=seventh_x;
        loop->harmonic7.y_v=seventh_y;
    }
    loop->harmonic_x_v=harmonic_x;
    loop->harmonic_y_v=harmonic_y;
    loop->integral_d=integral_d;
    loop->integral_q=integral_q;
    /* 向外饱和已冻结的误差不能留到下一拍零给定，避免旧大误差再次注入积分。 */
    loop->previous_error_d=integral_frozen?0.0f:error_d;
    loop->previous_error_q=integral_frozen?0.0f:error_q;
    loop->command_d_v=d;
    loop->command_q_v=q;
    loop->request_alpha_v=d*future_cos-q*future_sin;
    loop->request_beta_v=d*future_sin+q*future_cos;
    float phase[3]={loop->request_alpha_v,
        -0.5f*loop->request_alpha_v+0.86602540378f*loop->request_beta_v,
        -0.5f*loop->request_alpha_v-0.86602540378f*loop->request_beta_v};
    float high=phase[0], low=phase[0];
    for (unsigned i=1U;i<3U;++i)
    {
        if (phase[i]>high) { high=phase[i]; }
        if (phase[i]<low) { low=phase[i]; }
    }
    float common=-0.5f*(high+low);
    for (unsigned i=0U;i<3U;++i)
    {
        float value=(0.5f+(phase[i]+common)/bus_v)*loop->arr;
        if (!isfinite(value)) { return false; }
        uint16_t compare=(uint16_t)(value+0.5f);
        if (compare<loop->minimum_ccr) { compare=loop->minimum_ccr; }
        if (compare>loop->maximum_ccr) { compare=loop->maximum_ccr; }
        ccr[i]=compare;
    }
    return true;
}

/** @brief 只补偿接管坐标修正，积分矢量保持定子坐标连续。 */
void MOTOR_FAST_CODE
foc_current_rotate(foc_current_t *loop, float delta_cos, float delta_sin)
{
    float d=loop->integral_d, q=loop->integral_q;
    loop->integral_d=d*delta_cos+q*delta_sin;
    loop->integral_q=-d*delta_sin+q*delta_cos;
}
