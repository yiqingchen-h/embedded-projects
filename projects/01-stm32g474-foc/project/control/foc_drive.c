#include "foc_drive.h"
#include "motor_config.h"
#include "foc_math.h"
#include <math.h>
#include <stddef.h>

/* 独立FOC时基：电流/观察10kHz，速度1kHz；六步宏不参与。 */
#define DRIVE_TS .0001f
#define DRIVE_PERIOD_TICKS 17000U
/* 200ms升流、200ms预定位、300ms转向及500ms最终保持，共1.2s。 */
#define DRIVE_ALIGN_CYCLES 12000U
#define DRIVE_QUALIFY_CYCLES 2000U
#define DRIVE_BLEND_CYCLES 1000U
/* 强拖上限A；实际幅度取用户电流限值与本上限的较小值。 */
#define DRIVE_START_CURRENT_A MOTOR_FOC_START_CURRENT_A
/* 定位采用强拖幅度的80%，0.5A配置保持原0.4A定位轨迹。 */
#define DRIVE_ALIGN_CURRENT_FRACTION .8f
/* 可靠角度下速度软越限连续100ms才关断；反转及硬越限立即关断（普通3000RPM，扩展6000RPM）。 */
#define DRIVE_SPEED_RANGE_CYCLES 1000U
/* 0→会话启动频率用1秒，起始两倍平均升频率连续减到0，减少到达资格频率时的加速度突变。 */
#define DRIVE_RAMP_CYCLES 10000U
/* 功率阻尼增益(rad/s)/W；65Hz快速平滑及3.2Hz均值，不使用低速估计角。 */
#define DRIVE_POWER_DAMPING_GAIN 150.0f

/** @brief 折回相对电角差，避免跨pi清除正常资格。 */
static float MOTOR_FAST_CODE
drive_wrap(float angle)
{
    if(angle>3.14159265359f) { angle-=6.28318530718f; }
    if(angle<-3.14159265359f) { angle+=6.28318530718f; }
    return angle;
}

/** @brief 配置在停机一次验证；运行循环不重复验证已知R/L/频率。 */
bool
foc_drive_init(foc_drive_t *d,const foc_drive_config_t *c)
{
    if(NULL==d || NULL==c || !isfinite(c->current_limit_a)||!isfinite(c->emf_noise_v)||
        !isfinite(c->speed_kp)||!isfinite(c->speed_ki)||!isfinite(c->speed_ramp_rpm_s)||
        !isfinite(c->voltage_error_bound_v)||c->voltage_error_bound_v<0 || c->voltage_error_bound_v>1 ||
        c->current_limit_a<.05f || c->current_limit_a>2.0f || c->emf_noise_v<0 ||
        c->speed_kp<=0 || c->speed_ki<=0 || c->speed_ramp_rpm_s<=0 ||
        c->target_rpm<(c->extended_speed_range?1U:1000U) ||
        c->target_rpm>(c->extended_speed_range?MOTOR_SPEED_MAX_RPM:2000U) ||
        c->startup_rpm>1000U || c->pole_pairs!=MOTOR_POLE_PAIRS ||
        (MOTOR_PROFILE_G2208 && (c->target_rpm>MOTOR_SPEED_MAX_RPM || c->current_limit_a>MOTOR_FOC_LIMIT_MAX_MA*.001f))) { return false; }
    *d=(foc_drive_t){.config=*c,.stage=FOC_CALIBRATE,.harmonic_requested=true,.harmonic2_requested=true,.fault="NONE"};
    d->startup_omega_rad_s=(float)(c->startup_rpm?c->startup_rpm:MOTOR_FOC_STARTUP_RPM)*
        ((float)c->pole_pairs*.10471975512f);
    d->emf_floor_v=fmaxf(MOTOR_FOC_EMF_FLOOR_V,6*c->emf_noise_v);
    d->resistance_error_ohm=.2f*c->resistance_ohm;
    d->inductance_error_v_per_a=.2f*c->inductance_h/DRIVE_TS;
    return foc_current_init(&d->current,c->resistance_ohm,c->inductance_h,c->current_bandwidth_hz,
        DRIVE_TS,8500U,425U,8075U) &&
        foc_smo_init(&d->observer,c->resistance_ohm,c->inductance_h,DRIVE_TS,6,.03f,500,20,c->extended_speed_range?300:160,.05f,DRIVE_PERIOD_TICKS);
}

/** @brief 硬件完成安全校准后进入定位，首样本仍不更新观察器。 */
void
foc_drive_start(foc_drive_t *d) { d->stage=FOC_ALIGN; }

/** @brief 公共停机资格复位，不抹除真实接管历史。 */
void MOTOR_FAST_CODE
foc_drive_stop(foc_drive_t *d,const char *reason)
{
    d->stage=FOC_STOPPED;
    d->awaiting_commit=false;
    d->request_estimated_angle=false;
    d->qualify_cycles=0;
    d->hold_iq=false;
    foc_current_harmonic(&d->current,false);
    foc_current_harmonic2(&d->current,false);
    d->fault=NULL==reason?"NONE":reason;
}

/** @brief 任何估计/接管故障都只返回关断请求，不重回强拖。 */
static bool MOTOR_FAST_CODE
drive_fail(foc_drive_t *d,const char *reason)
{
    foc_drive_stop(d,reason);
    return false;
}

/**
 * @brief 先固定定子电流定位，再连续旋转同一电流矢量强拖。
 * @param d ADC路径拥有的启动状态；强拖角为定子电流方向，不是假定转子角。
 */
static void MOTOR_FAST_CODE
drive_force(foc_drive_t *d)
{
    float startup_current=fminf(DRIVE_START_CURRENT_A,d->config.current_limit_a);
    float alignment_current=fminf(d->config.current_limit_a,fmaxf(.4f,DRIVE_ALIGN_CURRENT_FRACTION*startup_current));
    if(d->samples<DRIVE_ALIGN_CYCLES)
    {
        /* 单一方向定位在反向死点附近可能不动；预定位后缓慢旋转90电角度。
         * 这是开环定子电流方向，没有转子位置反馈，也不宣称定位已被量测确认。 */
        float current_fraction=fminf(1.0f,d->samples/2000.0f);
        float angle_fraction=fmaxf(0,fminf(1.0f,((float)d->samples-4000)/3000.0f));
        d->force_theta_rad=-1.57079632679f*(1-angle_fraction*angle_fraction*(3-2*angle_fraction));
        /* 转向两端速度为零，未来PWM角度预测使用这条实际定子轨迹的电角速度。 */
        d->force_omega_rad_s=1.57079632679f*6*angle_fraction*(1-angle_fraction)/.3f;
        d->reference_d_a=alignment_current*current_fraction*current_fraction*(3-2*current_fraction);
        d->reference_q_a=0;
        return;
    }
    if(d->stage==FOC_ALIGN) { d->stage=FOC_I_F_RAMP; }
    if(d->stage==FOC_BLEND)
    {
        /* 沿通过资格时的真实强拖频率连续推进，不能在融合入口跳回名义启动频率。 */
        d->force_theta_rad=drive_wrap(d->force_theta_rad+d->force_omega_rad_s*DRIVE_TS);
        return;
    }
    float old=d->force_omega_rad_s;
    float speed_fraction=fminf(1.0f,(d->samples-DRIVE_ALIGN_CYCLES+1U)/(float)DRIVE_RAMP_CYCLES);
    float nominal_omega=d->startup_omega_rad_s*speed_fraction*(2-speed_fraction);
    /* 已施加区间功率的高通分量反馈到定子电频率，给I/f同步摆动增加阻尼。
     * 前100ms平滑加入、最后200ms平滑退出；校正最多名义频率的30%。
     * 资格阶段另用100ms平滑恢复阻尼，防止带载时结束加速便失去同步阻尼。
     * 电流幅度沿名义轨迹，不把目标或强拖频率送入SMO。 */
    float enter=fminf(1.0f,speed_fraction/.1f);
    float leave=fminf(1.0f,(1-speed_fraction)/.2f);
    float weight=enter*enter*(3-2*enter)*leave*leave*(3-2*leave);
    if(d->stage==FOC_QUALIFY)
    {
        float settle=fminf(1.0f,(d->samples-d->qualify_begun)/1000.0f);
        weight=settle*settle*(3-2*settle);
    }
    float correction=-DRIVE_POWER_DAMPING_GAIN*(d->power_fast_w-d->power_mean_w)*weight;
    float limit=.3f*nominal_omega;
    correction=fmaxf(-limit,fminf(limit,correction));
    d->force_omega_rad_s=fmaxf(0,fminf(d->startup_omega_rad_s,nominal_omega+correction));
    d->force_theta_rad=drive_wrap(d->force_theta_rad+(old+d->force_omega_rad_s)*.00005f);
    /* 电流从最终定位幅度随强拖频率连续升至启动限值。 */
    float fraction=speed_fraction*(2-speed_fraction);
    /* 强拖沿定位电流原方向缓升至启动限值；d→q移交会额外旋转90度并激起摆动。
     * 此处d轴是强拖坐标，转矩由真实转子落后旋转电流矢量的角度产生。 */
    d->reference_d_a=alignment_current+(startup_current-alignment_current)*fraction;
    /* 高运行限流不自动变为2A强拖，1A启动上限仍受用户电流限值约束。 */
    d->reference_q_a=0;
    if(d->stage==FOC_I_F_RAMP && speed_fraction>=1.0f)
    {
        d->stage=FOC_QUALIFY;
        d->qualify_begun=d->samples;
    }
}

/** @brief 200ms连续资格包含EMF、方向、速度、相位稳定及独立模型残差。 */
static bool MOTOR_FAST_CODE
drive_qualify(foc_drive_t *d)
{
    const foc_smo_t *o=&d->observer;
    float phase=drive_wrap(o->sample_theta_rad-d->force_theta_rad);
    if(d->qualify_cycles==0U) { d->qualify_phase_rad=phase; }
    bool qualified=o->pll_initialized && d->emf_margin_v>=d->emf_floor_v &&
        o->omega_rad_s>0 && fabsf(o->omega_rad_s-d->force_omega_rad_s)<=.15f*d->force_omega_rad_s &&
        fabsf(drive_wrap(phase-d->qualify_phase_rad))<=.25f && o->residual_a<=.05f;
    d->qualify_cycles=qualified?d->qualify_cycles+1U:0U;
    if(d->qualify_cycles>=DRIVE_QUALIFY_CYCLES)
    {
        d->stage=FOC_BLEND;
        d->blend_begun=d->samples;
    }
    return d->samples-d->qualify_begun<=20000U;
}

/** @brief 真实ADC顺序固定；未来请求的seq只用于提交资格，不进入SMO。 */
bool MOTOR_FAST_CODE
foc_drive_sample(foc_drive_t *d,const foc_voltage_interval_t *u,float ia,float ib,
    float bus,uint32_t sequence,float future_s,uint16_t ccr[3])
{
    if(d->stage==FOC_STOPPED || d->stage==FOC_CALIBRATE) { return false; }
    float da=ia-d->observer.previous_alpha_a,db=ib-d->observer.previous_beta_a;
    if(!foc_smo_step(&d->observer,u,ia,ib)) { return drive_fail(d,"FOC SMO INVALID"); }
    /* 先扣除实际电流及其变化可能产生的R/L误差，再判断是否存在足够的转子反电动势。
     * 本板压降拟合73..89mV、逻辑平均残差约40mV，暂取120mV电压误差界，待新镜像实测。
     * 这些量测不能逐项相加当作独立误差；R/L另覆盖±20%模型验证。
     * 不使用强拖频率或速度指令；静止转子跟随旋转电流造成的伪EMF必须落在此误差区内。 */
    d->model_bound_v=d->config.voltage_error_bound_v+d->resistance_error_ohm*sqrtf(ia*ia+ib*ib)+
        (u?d->inductance_error_v_per_a*sqrtf(da*da+db*db):0);
    d->emf_margin_v=d->observer.emf_magnitude_v-d->model_bound_v;
    /* 已施加区间平均电压与首末电流梯形均值计算功率，不能读取未来PWM。 */
    if(u && d->stage<=FOC_QUALIFY)
    {
        float power=1.5f*(u->alpha_v*(ia-.5f*da)+u->beta_v*(ib-.5f*db));
        d->power_fast_w+=.04f*(power-d->power_fast_w);
        d->power_mean_w+=.002f*(d->power_fast_w-d->power_mean_w);
    }
    d->request_estimated_angle=false;
    if(d->stage!=FOC_CLOSED)
    {
        drive_force(d);
        if(d->stage==FOC_QUALIFY && !drive_qualify(d)) { return drive_fail(d,"FOC QUALIFY TIMEOUT"); }
        d->control_theta_rad=d->force_theta_rad;
        d->control_omega_rad_s=d->force_omega_rad_s;
        if(d->stage==FOC_BLEND)
        {
            /* 融合中任一EMF/方向/残差失效立即关断，不退回强拖。 */
            if(!d->observer.pll_initialized || d->emf_margin_v<d->emf_floor_v ||
                d->observer.omega_rad_s<=0 || d->observer.residual_a>.06f)
            { return drive_fail(d,"FOC BLEND LOST"); }
            float fraction=fminf(1.0f,(d->samples-d->blend_begun+1U)/(float)DRIVE_BLEND_CYCLES);
            float wrapped_phase=drive_wrap(d->observer.sample_theta_rad-d->force_theta_rad);
            /* 对资格锚点连续展开，避免相位差跨pi导致融合方向突跳。 */
            float phase=d->qualify_phase_rad+drive_wrap(wrapped_phase-d->qualify_phase_rad);
            float correction=fraction*phase;
            float delta=drive_wrap(correction-d->correction_rad);
            /* 只旋转融合坐标修正量，正常电角增量不重复旋转PI积分。 */
            float cosine,sine;
            foc_sincos(delta,&cosine,&sine);
            foc_current_rotate(&d->current,cosine,sine);
            d->control_theta_rad=drive_wrap(d->force_theta_rad+correction);
            d->control_omega_rad_s=d->force_omega_rad_s+delta/DRIVE_TS;
            foc_sincos(correction,&cosine,&sine);
            float startup_current=fminf(DRIVE_START_CURRENT_A,d->config.current_limit_a);
            /* 强拖d轴电流转换到估计转子坐标，整个融合保持同一定子电流矢量。 */
            d->reference_d_a=startup_current*cosine;
            d->reference_q_a=-startup_current*sine;
            d->correction_rad=correction;
            if(fraction>=1.0f)
            {
                d->request_estimated_angle=true;
                d->awaiting_commit=true;
                d->pending_sequence=sequence;
            }
        }
    }
    else
    {
        float rpm=d->observer.omega_rad_s*(9.54929658551f/d->config.pole_pairs);
        /* 角度可信度失败立即关断，速度暂态单独计时和报告。 */
        const char *reason=!d->observer.pll_initialized?"FOC PLL LOST":
            d->emf_margin_v<d->emf_floor_v?"FOC EMF LOW":
            fabsf(d->observer.phase_error)>.35f?"FOC PHASE LOST":
            d->observer.residual_a>.06f?"FOC MODEL ERROR":
            rpm<=0 || rpm>=(d->config.extended_speed_range?MOTOR_FOC_HARD_MAX_RPM:3000)?"FOC SPEED HARD":NULL;
        if(reason) { return drive_fail(d,reason); }
        /* 极限试验由EMF/相位/残差检测低速失锁，取消旧600RPM人为下限。 */
        bool speed_outside=d->config.extended_speed_range?rpm>MOTOR_FOC_SOFT_MAX_RPM:(rpm<600 || rpm>2200);
        d->speed_range_cycles=speed_outside?d->speed_range_cycles+1U:0U;
        if(d->speed_range_cycles>=DRIVE_SPEED_RANGE_CYCLES)
        { return drive_fail(d,rpm<600?"FOC SPEED LOW":"FOC SPEED HIGH"); }
        d->control_theta_rad=d->observer.sample_theta_rad;
        d->control_omega_rad_s=d->observer.omega_rad_s;
        /* 接管后100ms退去定位磁场，保持速度PI给出的转矩电流连续。 */
        float fade=fmaxf(0,1.0f-(d->samples-d->closed_begun)/(float)DRIVE_BLEND_CYCLES);
        d->reference_d_a=d->closed_initial_d_a*fade;
        d->reference_q_a=d->speed_q_a;
        d->request_estimated_angle=true;
        ++d->closed_samples;
    }
    float theta=d->control_theta_rad;
    float future=drive_wrap(theta+d->control_omega_rad_s*future_s);
    float c,s,fc,fs;
    foc_sincos(theta,&c,&s);
    foc_sincos(future,&fc,&fs);
    /* 接管提交和100ms Id退出均完成才加入二/五次支路，启动轨迹保持。 */
    foc_current_harmonic(&d->current,d->harmonic_requested && d->stage==FOC_CLOSED &&
        d->samples-d->closed_begun>=DRIVE_BLEND_CYCLES);
    foc_current_harmonic2(&d->current,d->harmonic2_requested && d->stage==FOC_CLOSED &&
        d->samples-d->closed_begun>=DRIVE_BLEND_CYCLES);
    if(!foc_current_step(&d->current,ia,ib,c,s,fc,fs,
        bus,d->reference_d_a,d->reference_q_a,ccr)) { return drive_fail(d,"FOC CURRENT VALUE"); }
    ++d->samples;
    return true;
}

/** @brief 精确对应待确认的估计角PWM，提前写预装载不能打开速度PI。 */
void MOTOR_FAST_CODE
foc_drive_commit(foc_drive_t *d,uint32_t sequence,bool estimated_angle)
{
    if(d->stage!=FOC_BLEND || !d->awaiting_commit || sequence!=d->pending_sequence || !estimated_angle) { return; }
    d->stage=FOC_CLOSED;
    d->awaiting_commit=false;
    d->closed_seen=true;
    d->closed_begun=d->samples;
    d->closed_initial_d_a=d->reference_d_a;
    d->speed_reference_rpm=(float)(d->config.startup_rpm?d->config.startup_rpm:MOTOR_FOC_STARTUP_RPM);
    d->speed_q_a=fmaxf(0,fminf(d->config.current_limit_a,d->reference_q_a));
    float rpm=d->observer.omega_rad_s*(9.54929658551f/d->config.pole_pairs);
    d->speed_integral_a=d->speed_q_a-d->config.speed_kp*(d->speed_reference_rpm-rpm);
}

/** @brief 1kHz正转速度PI，指令斜坡及同方向条件积分抗饱和。 */
void MOTOR_FAST_CODE
foc_drive_speed_tick(foc_drive_t *d,uint32_t target)
{
    if(d->stage!=FOC_CLOSED) { return; }
    /* 固定Iq基准期间只暂停速度PI调节，电流环、观察器及失锁保护继续运行。 */
    if(d->hold_iq) { ++d->speed_ticks; return; }
    float delta=(float)target-d->speed_reference_rpm;
    float increment=d->config.speed_ramp_rpm_s*.001f;
    d->speed_reference_rpm+=fmaxf(-increment,fminf(increment,delta));
    float rpm=d->observer.omega_rad_s*(9.54929658551f/d->config.pole_pairs);
    float error=d->speed_reference_rpm-rpm;
    float integral=d->speed_integral_a+d->config.speed_ki*.001f*error;
    float requested=d->config.speed_kp*error+integral;
    /* 退去定位Id期间仍限制指令矢量幅值，不能让Id与速度Iq叠加越过初始限流。 */
    float q_limit=sqrtf(fmaxf(0,d->config.current_limit_a*d->config.current_limit_a-
        d->reference_d_a*d->reference_d_a));
    float limited=fmaxf(0,fminf(q_limit,requested));
    if((requested-limited)*error<=0) { d->speed_integral_a=integral; }
    d->speed_q_a=limited;
    ++d->speed_ticks;
}

/** @brief 见foc_drive.h；恢复速度环时从当前转矩电流连续接回。 */
bool
foc_drive_hold_iq(foc_drive_t *d,float current_a)
{
    if(NULL==d || d->stage!=FOC_CLOSED || !isfinite(current_a) || current_a<0 ||
        current_a>d->config.current_limit_a || fabsf(d->reference_d_a)>.000001f)
    { return false; }
    if(current_a==0)
    {
        float rpm=d->observer.omega_rad_s*(9.54929658551f/d->config.pole_pairs);
        d->speed_integral_a=d->speed_q_a-d->config.speed_kp*(d->speed_reference_rpm-rpm);
        d->hold_iq=false;
    }
    else
    {
        d->speed_q_a=current_a;
        d->hold_iq=true;
    }
    return true;
}

/** @brief 阶段仅描述控制实际状态，CLOSED由硬件提交确认设置。 */
const char *
foc_drive_stage_name(foc_drive_stage_t stage)
{
    static const char *const names[]={"STOPPED","CALIBRATE","ALIGN","I_F_RAMP","QUALIFY","BLEND","CLOSED"};
    return (unsigned)stage<sizeof(names)/sizeof(names[0])?names[stage]:"INVALID";
}
