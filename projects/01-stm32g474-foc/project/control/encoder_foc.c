#include "encoder_foc.h"
#include "foc_math.h"
#include "motor_config.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/* 当前板卡DWT=170MHz，双ADC电流环10kHz；机械反馈约5kHz。 */
#define ENCODER_FOC_TICK_HZ 170000000U
#define ENCODER_FOC_SAMPLE_HZ 10000U
/* 无首帧最多1ms；已有反馈300us年龄上限沿用有感六步。 */
#define ENCODER_FOC_AGE_TICKS (170U * ENCODER_SIX_MAX_AGE_US)
/* >=300RPM静止起步：最多80ms预充至60mA；低目标由快反馈PI建立转矩。 */
#define ENCODER_FOC_PRECHARGE_MS 80U
#define ENCODER_FOC_PRECHARGE_A .06f
/* 31.25Hz实测位置/电流主频的惯性近似约537k，先取600k；不是独立惯量辨识。 */
#define ENCODER_FOC_LOW_MODEL_B0 600000.0f
/* 参照Sguan_Ladrc的wc约200rad/s、w0=4wc；按本板真实间隔匹配离散极点。 */
#define ENCODER_FOC_LOW_OBSERVER_WO 800.0f
/* 1..20RPM统一位置观测反馈，避免11RPM突然回到原高积分速度PI。 */
#define ENCODER_FOC_LOW_POSITION_MAX_RPM 20U
/* 独立Iq沿用静止起步2mA/ms，正负切换逐拍经过零，不引入速度积分。 */
#define ENCODER_FOC_IQ_STEP_A .002f
/* 独立电流模式的指令限流固定0.5A；异常过流仍由原板卡保护执行。 */
#define ENCODER_FOC_IQ_LIMIT_A .5f
/* 位置允许有限反拖；偏离本次目标的最好位置额外超过半机械圈持续50ms才关断。 */
#define ENCODER_FOC_POSITION_ESCAPE_COUNTS (ENCODER_SIX_TURN / 2U)

/** @brief 撤销未来输出，契约见头文件。 */
void MOTOR_FAST_CODE
encoder_foc_stop(encoder_foc_t *s, const char *reason)
{
    s->running=false;
    s->awaiting_commit=false;
    s->startup_precharge=false;
    s->reference_q_a=0;
    s->fault=NULL==reason?"NONE":reason;
}

/** @brief 故障只请求关断，不回退其它算法。 @param s 状态。 @param reason 原因。 @return false。 */
static bool MOTOR_FAST_CODE
encoder_foc_fail(encoder_foc_t *s, const char *reason)
{
    encoder_foc_stop(s,reason);
    return false;
}

/** @brief 见encoder_foc.h；目标转原码在主循环进行，ISR不执行64位除法。 */
void
encoder_foc_position_target(encoder_foc_t *s, int32_t target_mdeg)
{
    s->config.target_position_mdeg=target_mdeg;
    s->target_position_counts=encoder_position_target_counts(target_mdeg);
    s->position_best_error_counts=INT64_MAX;
    s->position_settle_ms=0U;
}

/** @brief 见encoder_foc.h；停机初始化按体积优化，不改变实时控制段。 */
bool MOTOR_BACKGROUND_CODE
encoder_foc_init(encoder_foc_t *s, const encoder_foc_config_t *c)
{
    if(NULL==s || NULL==c || (c->direction!=1 && c->direction!=-1) || c->zero_decideg>=3600U ||
        c->drive.pole_pairs!=MOTOR_POLE_PAIRS || c->drive.target_rpm<1U || c->drive.target_rpm>MOTOR_SPEED_MAX_RPM ||
        !isfinite(c->drive.current_limit_a) || c->drive.current_limit_a<.05f || c->drive.current_limit_a>MOTOR_FOC_LIMIT_MAX_MA*.001f ||
        !isfinite(c->drive.speed_kp) || !isfinite(c->drive.speed_ki) || !isfinite(c->drive.speed_ramp_rpm_s) ||
        c->drive.speed_kp<=0 || c->drive.speed_ki<=0 || c->drive.speed_ramp_rpm_s<=0)
    { return false; }
    if(c->current_mode && (!isfinite(c->target_iq_a) || fabsf(c->target_iq_a)>ENCODER_FOC_IQ_LIMIT_A ||
        fabsf(c->target_iq_a)>c->drive.current_limit_a || c->drive.current_limit_a>ENCODER_FOC_IQ_LIMIT_A))
    { return false; }
    if(c->position_mode && (c->current_mode ||
        !isfinite(c->position_kp) || c->position_kp<=0 ||
        !isfinite(c->position_speed_limit_rpm) || c->position_speed_limit_rpm<1 ||
        c->position_speed_limit_rpm>MOTOR_SPEED_MAX_RPM ||
        c->position_tolerance_mdeg<1 || c->position_tolerance_mdeg>500)) { return false; }
    if(c->haptic_mode && (c->current_mode || c->position_mode || !encoder_haptic_valid(&c->haptic))) { return false; }
    memset(s,0,sizeof(*s));
    s->config=*c;
    encoder_foc_position_target(s,c->target_position_mdeg);
    s->zero_raw=(uint32_t)((uint64_t)c->zero_decideg*ENCODER_SIX_TURN/3600U);
    s->rpm_per_raw_tick=60.0f*ENCODER_FOC_TICK_HZ/ENCODER_SIX_TURN;
    s->fault="NONE";
    if(!foc_current_init(&s->current,c->drive.resistance_ohm,c->drive.inductance_h,
        c->drive.current_bandwidth_hz,1.0f/ENCODER_FOC_SAMPLE_HZ,8500U,425U,8075U))
    { return false; }
    /* 首版不启用无感模型的谐波学习，直接保留真实基波dq电流环。 */
    s->running=true;
    return true;
}

/** @brief 检查动态反馈并累加机械位移，ADC内没有64位除法。
 * @param s 状态。 @param f 最新完整反馈。 @param now 当前DWT。 @return 可用。 */
static bool MOTOR_FAST_CODE
encoder_foc_feedback(encoder_foc_t *s, const encoder_six_feedback_t *f, uint32_t now)
{
    if(!f->valid || f->raw>=ENCODER_SIX_TURN)
    {
        if(f->sequence!=0U) { return encoder_foc_fail(s,"ENCODER DATA"); }
        return false;
    }
    uint32_t age=now-f->tick;
    if(age>ENCODER_FOC_AGE_TICKS) { return encoder_foc_fail(s,"ENCODER STALE"); }
    if(age>s->age_max_ticks) { s->age_max_ticks=age; }
    if(s->feedback_seen && f->sequence==s->feedback_sequence) { return true; }
    if(s->feedback_seen)
    {
        uint32_t dt=f->tick-s->feedback_tick;
        if(dt==0U || dt>ENCODER_FOC_TICK_HZ/1000U) { return encoder_foc_fail(s,"ENCODER TIME"); }
        int32_t delta=(int32_t)f->raw-(int32_t)s->raw;
        if(delta>(int32_t)(ENCODER_SIX_TURN/2U)) { delta-=(int32_t)ENCODER_SIX_TURN; }
        if(delta<-(int32_t)(ENCODER_SIX_TURN/2U)) { delta+=(int32_t)ENCODER_SIX_TURN; }
        uint32_t magnitude=(uint32_t)(delta<0?-delta:delta);
        if((uint64_t)magnitude*60U*ENCODER_FOC_TICK_HZ>=
            (uint64_t)MOTOR_FOC_HARD_MAX_RPM*ENCODER_SIX_TURN*dt)
        { return encoder_foc_fail(s,"ENCODER OVERSPEED"); }
        s->speed_delta+=delta*s->config.direction;
        s->speed_elapsed+=dt;
        if(s->config.haptic_mode && s->closed_seen)
        {
            /* 手感1kHz档位/配置外环由反馈内分频，阻尼在5kHz真实原码处校正。
             * 不复用速度PLL；全部手感动态状态归ADC所有，避免与TIM并发写。 */
            int32_t mechanical_delta=delta*s->config.direction;
            float haptic_rpm=encoder_haptic_velocity(&s->haptic,mechanical_delta,
                dt*(1.0f/ENCODER_FOC_TICK_HZ));
            s->reference_q_a=encoder_haptic_tick(&s->haptic,&s->config.haptic,mechanical_delta,
                f->tick,dt*(1.0f/ENCODER_FOC_TICK_HZ),haptic_rpm,s->config.drive.current_limit_a);
            s->haptic.raw=f->raw;
        }
    }
    s->feedback_seen=true;
    s->raw=f->raw;
    s->feedback_tick=f->tick;
    s->feedback_sequence=f->sequence;
    return true;
}

/** @brief 见encoder_foc.h；七点结果已经是转子d轴零点，不额外加减90度。 */
bool MOTOR_FAST_CODE
encoder_foc_sample(encoder_foc_t *s, const encoder_six_feedback_t *f,
    uint32_t now_tick, uint32_t sample_tick, float alpha_a, float beta_a, float bus_v,
    uint32_t sequence, float future_s, uint16_t ccr[3])
{
    if(!s->running) { return false; }
    ++s->samples;
    if(!encoder_foc_feedback(s,f,now_tick))
    {
        ccr[0]=ccr[1]=ccr[2]=4250U;
        if(s->running && s->samples>=ENCODER_FOC_SAMPLE_HZ/1000U)
        { return encoder_foc_fail(s,"ENCODER STALE"); }
        return s->running;
    }
    uint32_t phase=s->raw*s->config.drive.pole_pairs;
    if(s->config.direction<0) { phase=0U-phase; }
    phase=(phase-s->zero_raw)&(ENCODER_SIX_TURN-1U);
    /* 补偿SPI反馈年龄，再预测至下一PWM平均作用中心；角速度只取真实机械位移。 */
    float age_s=(float)(sample_tick-s->feedback_tick)/ENCODER_FOC_TICK_HZ;
    if(age_s>.0003f) { return encoder_foc_fail(s,"ENCODER SAMPLE TIME"); }
    s->sample_theta_rad=phase*(6.28318530718f/ENCODER_SIX_TURN)+s->omega_rad_s*age_s;
    if(s->sample_theta_rad>3.14159265359f) { s->sample_theta_rad-=6.28318530718f; }
    if(s->sample_theta_rad<-3.14159265359f) { s->sample_theta_rad+=6.28318530718f; }
    s->future_theta_rad=s->sample_theta_rad+s->omega_rad_s*future_s;
    if(s->future_theta_rad>3.14159265359f) { s->future_theta_rad-=6.28318530718f; }
    if(s->future_theta_rad<-3.14159265359f) { s->future_theta_rad+=6.28318530718f; }
    float c,si,fc,fs;
    foc_sincos(s->sample_theta_rad,&c,&si);
    foc_sincos(s->future_theta_rad,&fc,&fs);
    /* G档位置/手感沿用已验证的梯形积分，减少驻点量化噪声进入电压积分；
     * 原始Id/Iq不滤波，保持电流带宽、负载保持偏置及原CURRENT入口。 */
    foc_current_trapezoidal(&s->current,MOTOR_PROFILE_G2208 && (s->config.position_mode ||
        s->config.haptic_mode || (!s->config.current_mode && s->config.drive.target_rpm==1U)));
#if MOTOR_FOC_FEEDFORWARD_ENABLED
    /* 速度来自实际机械位移并已乘极对数；采样电流和未来角仍走原路径，不使用目标RPM。 */
    foc_current_feedforward(&s->current,s->omega_rad_s,s->config.drive.inductance_h,
        s->config.drive.inductance_h,MOTOR_FOC_FLUX_WB);
#endif
    if(!foc_current_step(&s->current,alpha_a,beta_a,c,si,fc,fs,bus_v,0,s->reference_q_a,ccr))
    { return encoder_foc_fail(s,"FOC CURRENT VALUE"); }
    if(!s->closed_seen)
    {
        s->awaiting_commit=true;
        s->pending_sequence=sequence;
    }
    return true;
}

/** @brief 见encoder_foc.h。 */
void MOTOR_FAST_CODE
encoder_foc_commit(encoder_foc_t *s, uint32_t sequence)
{
    if(s->running && s->feedback_seen && s->awaiting_commit && sequence==s->pending_sequence)
    {
        s->closed_seen=true;
        s->awaiting_commit=false;
    }
}

/** @brief 见encoder_foc.h；由板卡仅围绕这四次访问建立短临界区。 */
encoder_foc_speed_window_t MOTOR_FAST_CODE
encoder_foc_take_speed(encoder_foc_t *s)
{
    encoder_foc_speed_window_t window={s->speed_delta,s->speed_elapsed};
    s->speed_raw=s->raw;
    s->speed_tick=s->feedback_tick;
    s->speed_delta=0;
    s->speed_elapsed=0;
    return window;
}

/** @brief 从残差LESO学习有界角相关负载，不把同一扰动与前馈重复叠加。
 * @param s 控制实例。 @param dt 本次真实间隔秒。 @param learn 上轮输出/本轮电压均未受限。
 * @return 当前机械角的预测负载A；与残差修正之和另受原80mA约束。 */
static float MOTOR_FAST_CODE
encoder_foc_low_load(encoder_foc_t *s, float dt, bool learn)
{
    /* 28/36/84来自实板角相关性；补偿总预算保持80mA，不代表已辨识槽数。 */
    static const uint32_t orders[3]={28U,36U,84U};
    float bound=fminf(.08f/3.0f,s->config.drive.current_limit_a*(.16f/3.0f));
    float residual_a=-s->low_disturbance_rpm_s/ENCODER_FOC_LOW_MODEL_B0;
    /* ADC可能抢占速度回调，三个阶次必须使用同一帧机械原码。 */
    uint32_t raw=s->raw;
    /* 最低速放慢到28次角频率的10%，避免不足一圈时把平均摩擦快速学进谐波。 */
    float rate=fminf(4.0f,.293215314335f*s->config.drive.target_rpm);
    float before=0.0f,result=0.0f;
    for(unsigned h=0U;h<3U;++h)
    {
        uint32_t phase=(raw*orders[h])&(ENCODER_SIX_TURN-1U);
        float c,si;foc_sincos(phase*(6.28318530718f/ENCODER_SIX_TURN),&c,&si);
        before+=s->low_load_c[h]*c+s->low_load_s[h]*si;
        if(learn)
        {
            float x=s->low_load_c[h]+rate*dt*residual_a*c;
            float y=s->low_load_s[h]+rate*dt*residual_a*si;
            float magnitude=x*x+y*y;
            if(magnitude>bound*bound)
            { float scale=bound/sqrtf(magnitude);x*=scale;y*=scale; }
            s->low_load_c[h]=x;s->low_load_s[h]=y;
        }
        result+=s->low_load_c[h]*c+s->low_load_s[h]*si;
    }
    /* 学习系数变化时只平移扰动坐标，输出补偿不允许在这一拍突然叠加。 */
    float disturbance_bound=ENCODER_FOC_LOW_MODEL_B0*fminf(.08f,s->config.drive.current_limit_a);
    s->low_disturbance_rpm_s=fmaxf(-disturbance_bound,fminf(disturbance_bound,
        s->low_disturbance_rpm_s+ENCODER_FOC_LOW_MODEL_B0*(result-before)));
    return result;
}

/** @brief 低速Iq仍按原斜率升流；明显超速时更快撤掉同方向驱动转矩。
 * @param s 独立状态。 @param requested 未限幅Iq指令A。
 * @param error 参考减观测机械速度RPM。 @param limit 原相限流A。
 * @return 下一拍Iq指令A；快速回收只到零，反向制动仍按原斜率建立。
 * @note 10RPM/一倍参考误差之外才加速回收，避免把低速测量纹波变成电流跳变。
 * 64mA/ms仅撤销已存在的转矩，不提高0.5A限流或起步/制动升流速度。 */
static float MOTOR_FAST_CODE
encoder_foc_low_iq_next(const encoder_foc_t *s,float requested,float error,float limit)
{
    float bounded=fmaxf(-limit,fminf(limit,requested));
    float slew=s->startup_end_ms!=0U?.008f:.002f;
    if(s->startup_end_ms!=0U && fabsf(error)>fmaxf(10.0f,s->config.position_mode?fabsf(s->reference_rpm):s->reference_rpm) &&
        error*s->reference_q_a<0.0f && s->reference_q_a*(bounded-s->reference_q_a)<0.0f)
    { slew=fmaxf(slew,fminf(.064f,fabsf(s->reference_q_a))); }
    return s->reference_q_a+fmaxf(-slew,fminf(slew,bounded-s->reference_q_a));
}

/** @brief 借鉴Sguan的LESO/位置反馈，只在候选低速段控制真实编码器转矩。
 * @param s 独立状态。 @param window 当前真实位移/时间。 @param previous_reference 上次速度参考RPM。
 * @note 位置二阶模型为位置导数=速度、速度导数=b0*上次限幅Iq+扰动；输出仍经过原限流/斜率。
 * 用极点匹配代替参考工程固定周期Euler，避免把其50/250us系数原样放到本板1ms。 */
static void MOTOR_FAST_CODE
encoder_foc_low_position(encoder_foc_t *s, encoder_foc_speed_window_t window, float previous_reference)
{
    if(window.ticks==0U) { return; }
    float dt=window.ticks*(1.0f/ENCODER_FOC_TICK_HZ);
    float displacement=window.delta*(60.0f/ENCODER_SIX_TURN);
    /* 将实板较好的会话200/800归一到保存100/200，电流环仍原800Hz。 */
    float kp=s->config.drive.speed_kp*(s->config.position_mode?1.0f:(40.0f/3.0f));
    float ki=s->config.drive.speed_ki*(s->config.position_mode?1.0f:(4000.0f/3.0f));
    float limit=s->config.drive.current_limit_a;
    if(!s->low_position_active)
    {
        s->low_velocity_rpm=s->control_rpm;
        s->low_position_residual=s->low_disturbance_rpm_s=0.0f;
        s->low_trajectory_error=s->integral_a/ki;
        for(unsigned h=0U;h<3U;++h) { s->low_load_c[h]=s->low_load_s[h]=0.0f; }
        s->low_load_previous_a=0.0f;
        s->low_position_active=true;
    }
    /* 位置残差用增量保存，避免低速长时间绝对角累加损失单精度有效位。 */
    float acceleration=ENCODER_FOC_LOW_MODEL_B0*(s->reference_q_a-s->low_load_previous_a)+s->low_disturbance_rpm_s;
    float innovation=s->low_position_residual+s->low_velocity_rpm*dt+.5f*acceleration*dt*dt-displacement;
    float predicted=s->low_velocity_rpm+acceleration*dt;
    float pole=expf(-ENCODER_FOC_LOW_OBSERVER_WO*dt),m=1.0f-pole;
    s->low_position_residual=pole*pole*pole*innovation;
    s->low_velocity_rpm=predicted-1.5f*m*m*(1.0f+pole)/dt*innovation;
    /* 原两组角同步支路最多80mA，这里扰动模型同样只允许80mA且不超过相限流。 */
    float disturbance_bound=ENCODER_FOC_LOW_MODEL_B0*fminf(.08f,limit);
    s->low_disturbance_rpm_s=fmaxf(-disturbance_bound,fminf(disturbance_bound,
        s->low_disturbance_rpm_s-m*m*m/(dt*dt)*innovation));
    float previous=s->low_trajectory_error;
    /* Sguan位置环思路与梯形参考积分：实际位移直接进入积分，不再积分PLL速度延迟。 */
    float trajectory=previous+.5f*(previous_reference+s->reference_rpm)*dt-displacement;
    trajectory=fmaxf(-limit/ki,fminf(limit/ki,trajectory));
    float error=s->reference_rpm-s->low_velocity_rpm;
    bool voltage_limited=s->current.limited!=s->seen_voltage_limits;
    s->seen_voltage_limits=s->current.limited;
    bool learn=s->startup_end_ms!=0U && s->reference_rpm==(float)s->config.drive.target_rpm &&
        !s->ripple_frozen && !voltage_limited;
    /* 位置保持不持续学习旋转谐波，避免驻点附近系数自适应制造抖动。 */
    float load=s->config.position_mode?0.0f:encoder_foc_low_load(s,dt,learn);
    float correction_bound=fminf(.08f,limit);
    float correction=fmaxf(-correction_bound,fminf(correction_bound,
        load-s->low_disturbance_rpm_s/ENCODER_FOC_LOW_MODEL_B0));
    float requested=kp*error+ki*(trajectory-s->low_position_residual)+correction;
    float next=encoder_foc_low_iq_next(s,requested,error,limit);
    if(error*(requested-next)>0.0f || (voltage_limited && error*requested>0.0f))
    {
        /* Sguan条件冻结扩展到真实Iq斜率/电压饱和，但误差反向时允许旧偏置回到零。
         * 原先在比例项要求强制动时仍冻结正积分，会保留加载时的驱动转矩。
         * 回收至零即止，不把比例跳变反灌积分，也不继续积出反方向偏置。 */
        float old_bias=ki*(previous-s->low_position_residual)+correction;
        float new_bias=ki*(trajectory-s->low_position_residual)+correction;
        if(error*old_bias<0.0f)
        {
            if(old_bias*new_bias<0.0f) { trajectory=s->low_position_residual-correction/ki; }
        }
        else { trajectory=previous; }
        requested=kp*error+ki*(trajectory-s->low_position_residual)+correction;
        next=encoder_foc_low_iq_next(s,requested,error,limit);
    }
    s->low_trajectory_error=trajectory;
    s->integral_a=fmaxf(-limit,fminf(limit,ki*(trajectory-s->low_position_residual)+correction));
    s->reference_q_a=next;
    s->low_load_previous_a=load;
    s->startup_precharge=false;
    s->ripple_frozen=voltage_limited || fabsf(requested-next)>.000001f || fabsf(requested)>=limit;
}

/** @brief 以实测角相关性学习有界转矩补偿，不能改变保护或实际编码器角。
 * @param s 独立状态。 @param weight 原G档低速权重。 @param error 真实PLL速度误差RPM。
 * @param target 机械目标RPM。 @return 总Iq补偿A，继续受原限流和变化率约束。 */
static float MOTOR_FAST_CODE
encoder_foc_ripple(encoder_foc_t *s, float weight, float error, uint32_t target)
{
    /* 150RPM已通过，200RPM补偿曾变差；150..200平滑撤销，不将试验支路全档套用。 */
    weight*=fmaxf(0.0f,fminf(1.0f,(200.0f-target)*.02f));
    if(weight<=0.0f) { return 0.0f; }
    /* 14/12来自角度同步拟合；不把12次相关性当作已测定的槽数或唯一成因。 */
    static const uint32_t orders[2]={2U*MOTOR_POLE_PAIRS,12U};
    float bound=fminf(.04f,s->config.drive.current_limit_a*.08f);
    bool learn=s->startup_end_ms!=0U && s->reference_rpm==(float)target &&
        !s->ripple_frozen && s->current.limited==s->seen_voltage_limits;
    float result=0.0f;
    for(unsigned h=0U;h<2U;++h)
    {
        uint32_t phase=(s->raw*orders[h])&(ENCODER_SIX_TURN-1U);
        float c,si;
        foc_sincos(phase*(6.28318530718f/ENCODER_SIX_TURN),&c,&si);
        if(learn)
        {
            float step=.002f*.001f*error;
            float x=s->ripple_c[h]+step*c, y=s->ripple_s[h]+step*si;
            float magnitude=x*x+y*y;
            if(magnitude>bound*bound)
            {
                float scale=bound/sqrtf(magnitude);
                x*=scale; y*=scale;
            }
            s->ripple_c[h]=x; s->ripple_s[h]=y;
        }
        result+=s->ripple_c[h]*c+s->ripple_s[h]*si;
    }
    return result*weight;
}

/** @brief Sguan位置P外环：限速、制动、加速度连续及有限反拖监督。
 * @param s 独立位置控制状态。
 * @param window 本拍实际机械位移及DWT时间，再进入原低速抗饱和反馈。 */
static void MOTOR_FAST_CODE
encoder_foc_position_step(encoder_foc_t *s, encoder_foc_speed_window_t window)
{
    if(!s->position_valid) { (void)encoder_foc_fail(s,"POSITION LOST"); return; }
    s->overspeed_ms=s->speed_valid&&fabsf(s->rpm)>MOTOR_FOC_SOFT_MAX_RPM?s->overspeed_ms+1U:0U;
    s->stall_ms=s->speed_valid&&fabsf(s->reference_rpm)>=100.0f&&fabsf(s->rpm)<10.0f?s->stall_ms+1U:0U;
    if(s->overspeed_ms>=100U || s->stall_ms>=500U)
    { (void)encoder_foc_fail(s,s->overspeed_ms>=100U?"ENCODER SPEED RANGE":"ENCODER STALL"); return; }
    if(!s->closed_seen || window.ticks==0U) { return; }
    /* 整数先求差，长时间或多圈位置不会降低驻点附近的单精度分辨率。 */
    int64_t delta=s->target_position_counts-s->position_counts;
    int64_t distance=delta<0?-delta:delta;
    if(distance<s->position_best_error_counts) { s->position_best_error_counts=distance; }
    float error=(float)delta*(360.0f/ENCODER_SIX_TURN);
    float acceleration=s->config.drive.speed_ramp_rpm_s;
    /* 仅在目标附近使用连续微死区，最大20mdeg且小于到位容差的四分之一。
     * 不清积分/不漂移目标，已有承载转矩保留；超出后连续恢复P外环。 */
    float dead=fminf(.02f,s->config.position_tolerance_mdeg*.00025f);
    float correction=copysignf(fmaxf(0,fabsf(error)-dead),error);
    float maximum=fminf(s->config.position_speed_limit_rpm,sqrtf(acceleration*fabsf(correction)/3.0f));
    float requested=fmaxf(-maximum,fminf(maximum,correction*s->config.position_kp/6.0f));
    /* 外力反拖和控制反向无法只靠速度符号区分。按原始多圈误差增长关断，
     * 新目标重建基准，合法大行程及在线反向不占用半圈逃离预算。 */
    s->reverse_ms=distance-s->position_best_error_counts>ENCODER_FOC_POSITION_ESCAPE_COUNTS?s->reverse_ms+1U:0U;
    s->motion_windows=fabsf(s->reference_rpm)>.05f && fabsf(error)*1000.0f>s->config.position_tolerance_mdeg &&
        s->speed_valid && fabsf(s->rpm)<.05f?s->motion_windows+1U:0U;
    if(s->reverse_ms>=50U || s->motion_windows>=5000U)
    { (void)encoder_foc_fail(s,s->reverse_ms>=50U?"POSITION ESCAPE":"ENCODER START"); return; }
    float previous=s->reference_rpm;
    float step=acceleration*window.ticks*(1.0f/ENCODER_FOC_TICK_HZ);
    s->reference_rpm+=fmaxf(-step,fminf(step,requested-s->reference_rpm));
    if(s->speed_valid && fabsf(s->rpm)>.5f && s->startup_end_ms==0U) { s->startup_end_ms=s->speed_ticks; }
    encoder_foc_low_position(s,window,previous);
    bool settled=fabsf(error)*1000.0f<=(float)s->config.position_tolerance_mdeg && fabsf(s->reference_rpm)<=.1f;
    s->position_settle_ms=settled?(s->position_settle_ms<200U?s->position_settle_ms+1U:200U):0U;
}

/** @brief 见encoder_foc.h；单精度硬件除法足以支持20ms机械速度窗。 */
void MOTOR_FAST_CODE
encoder_foc_speed_tick(encoder_foc_t *s, uint32_t target_rpm, encoder_foc_speed_window_t window)
{
    if(!s->running || !s->feedback_seen) { return; }
    ++s->speed_ticks;
    if(!s->config.current_mode && !s->config.position_mode && !s->config.haptic_mode && (target_rpm<1U || target_rpm>MOTOR_SPEED_MAX_RPM))
    { (void)encoder_foc_fail(s,"ENCODER TARGET"); return; }
    s->config.drive.target_rpm=target_rpm;
    if(window.ticks>0U)
    {
        /* 借鉴SguanFOC角度PLL思路（docs/参考工程/sguan_foc_local_20261019/MIT许可），
         * 用真实位移纠正位置预测，不用短窗差分噪声直接驱动高增益PI；Park仍取原校准角。 */
        float dt=(float)window.ticks/ENCODER_FOC_TICK_HZ;
        /* 先按本窗口实际间隔推进上次速度，再纠正当前残差；不能用上次间隔预支本次位移。 */
        float previous_velocity=s->control_rpm*(ENCODER_SIX_TURN/60.0f);
        float error=s->tracking_error+(float)window.delta-previous_velocity*dt;
        float maximum=MOTOR_FOC_HARD_MAX_RPM*(ENCODER_SIX_TURN/60.0f);
        s->tracking_integral=fmaxf(-maximum,fminf(maximum,s->tracking_integral+210000.0f*dt*error));
        float velocity=fmaxf(-maximum,fminf(maximum,650.0f*error+s->tracking_integral));
        s->control_rpm=velocity*(60.0f/ENCODER_SIX_TURN);
        s->tracking_error=error;
    }
    s->window_delta+=window.delta;
    s->window_elapsed+=window.ticks;
    if(s->window_elapsed>=ENCODER_FOC_TICK_HZ/50U)
    {
        s->rpm=(float)s->window_delta*s->rpm_per_raw_tick/(float)s->window_elapsed;
        s->omega_rad_s=s->rpm*(s->config.drive.pole_pairs*.10471975512f);
        s->window_delta=0;
        s->window_elapsed=0U;
        s->speed_valid=true;
        float motion_threshold=fminf(30.0f,fmaxf(.5f,target_rpm*.5f));
        if(!s->config.current_mode && !s->config.position_mode && !s->config.haptic_mode && s->startup_end_ms==0U && s->rpm>=motion_threshold)
        {
            if(s->motion_windows==0U) { s->motion_ms=s->speed_ticks; }
            if(s->motion_windows<2U) { ++s->motion_windows; }
            if(s->motion_windows==2U && s->startup_end_ms==0U) { s->startup_end_ms=s->speed_ticks; }
        }
        else if(!s->config.current_mode && !s->config.position_mode && !s->config.haptic_mode && s->startup_end_ms==0U) { s->motion_windows=0U; }
    }
    if(s->config.position_mode) { encoder_foc_position_step(s,window); return; }
    if(s->config.haptic_mode)
    {
        /* 自由双向手拨不套速度跟踪/固定位置判据，绝对超速与采样保护仍执行。 */
        s->overspeed_ms=s->speed_valid&&fabsf(s->rpm)>MOTOR_FOC_SOFT_MAX_RPM?s->overspeed_ms+1U:0U;
        if(s->overspeed_ms>=100U) { (void)encoder_foc_fail(s,"ENCODER SPEED RANGE"); return; }
        return;
    }
    if(s->config.current_mode)
    {
        /* 参照SguanFOC的Control_Current_SINGLE：编码器Park直接闭合Id/Iq，
         * 电流模式允许静止输出及双向运动，不能套用正速度追踪的起转/反转/堵转判据。
         * 原硬超速在每帧反馈中执行；软超速对两个机械方向采用同一原1900RPM/100ms。 */
        s->overspeed_ms=s->speed_valid&&fabsf(s->rpm)>MOTOR_FOC_SOFT_MAX_RPM?s->overspeed_ms+1U:0U;
        if(s->overspeed_ms>=100U) { (void)encoder_foc_fail(s,"ENCODER SPEED RANGE"); return; }
        if(s->closed_seen)
        {
            float requested=s->config.target_iq_a;
            s->reference_q_a+=fmaxf(-ENCODER_FOC_IQ_STEP_A,
                fminf(ENCODER_FOC_IQ_STEP_A,requested-s->reference_q_a));
        }
        return;
    }
    if(s->speed_ticks>=(target_rpm>=100U?1000U:5000U) && s->startup_end_ms==0U)
    { (void)encoder_foc_fail(s,"ENCODER START"); return; }
    s->reverse_ms=s->speed_valid&&s->rpm<-30?s->reverse_ms+1U:0U;
    s->overspeed_ms=s->speed_valid&&s->rpm>MOTOR_FOC_SOFT_MAX_RPM?s->overspeed_ms+1U:0U;
    s->stall_ms=s->startup_end_ms!=0U&&s->reference_rpm>=100&&s->rpm<10?s->stall_ms+1U:0U;
    if(s->reverse_ms>=50U || s->overspeed_ms>=100U || s->stall_ms>=500U)
    { (void)encoder_foc_fail(s,s->reverse_ms>=50U?"ENCODER REVERSE":s->overspeed_ms>=100U?"ENCODER SPEED RANGE":"ENCODER STALL"); return; }
    float previous_reference=s->reference_rpm;
    float step=s->config.drive.speed_ramp_rpm_s*.001f;
    s->reference_rpm+=fmaxf(-step,fminf(step,(float)target_rpm-s->reference_rpm));
    if(MOTOR_PROFILE_G2208 && target_rpm<=ENCODER_FOC_LOW_POSITION_MAX_RPM)
    {
        encoder_foc_low_position(s,window,previous_reference);
        return;
    }
    s->low_position_active=false;
    float low_weight=MOTOR_PROFILE_G2208?fmaxf(0.0f,fminf(1.0f,(300.0f-target_rpm)*.01f)):0.0f;
    float feedback=s->rpm+low_weight*(s->control_rpm-s->rpm);
    /* 把实测分档增益归一到保存100/200：1..10为50/500，20为50/200，
     * 50为50/100，100..200为30/20；节点间连续插值，200..300回原用户增益。
     * 这些参数用于低速补偿，调节效果需结合电机与工况验证。 */
    float kp_scale=target_rpm<=50U?9.0f:target_rpm>=100U?5.4f:9.0f-(target_rpm-50U)*.072f;
    float ki_scale=50.0f;
    if(target_rpm<=10U) { ki_scale=1250.0f; }
    else if(target_rpm<=20U) { ki_scale=1250.0f-(target_rpm-10U)*75.0f; }
    else if(target_rpm<=50U) { ki_scale=500.0f-(target_rpm-20U)*(250.0f/30.0f); }
    else if(target_rpm<100U) { ki_scale=250.0f-(target_rpm-50U)*4.0f; }
    float kp=s->config.drive.speed_kp*(1.0f+low_weight*(kp_scale-1.0f));
    float ki=s->config.drive.speed_ki*(1.0f+low_weight*(ki_scale-1.0f));
    float error=s->reference_rpm-feedback;
    float previous_integral=s->integral_a;
    float integral=previous_integral+ki*.001f*error;
    float limit=s->config.drive.current_limit_a;
    /* 首个合格运动窗即停止预充，直接沿用同一PI积分，不切换电角或电压控制。 */
    s->startup_precharge=target_rpm>=(MOTOR_PROFILE_G2208?300U:100U) && s->startup_end_ms==0U &&
        s->motion_windows==0U && s->speed_ticks<=ENCODER_FOC_PRECHARGE_MS;
    float precharge_limit=fminf(ENCODER_FOC_PRECHARGE_A,limit);
    if(s->startup_precharge)
    { integral=fminf(precharge_limit,integral+.002f); }
    float ripple=encoder_foc_ripple(s,low_weight,error,target_rpm);
    float requested=kp*error+integral+ripple;
    uint32_t voltage_limits=s->current.limited;
    bool voltage_limited=voltage_limits!=s->seen_voltage_limits;
    s->seen_voltage_limits=voltage_limits;
    if((requested>limit&&error>0) || (requested<-limit&&error<0) || (voltage_limited&&error*requested>0))
    { integral=s->integral_a; }
    s->integral_a=fmaxf(-limit,fminf(limit,integral));
    requested=fmaxf(-limit,fminf(limit,kp*error+s->integral_a+ripple));
    /* 静止起步和高档仍2mA/ms；确认两窗运动后低速允许最多8mA/ms修正扰动。
     * 这只改变指令带宽，不提高相限流、异常过流或任何速度/反馈/时限关断阈值。 */
    float slew=.002f+(s->startup_end_ms!=0U?low_weight*.006f:0.0f);
    float next=s->reference_q_a+fmaxf(-slew,fminf(slew,requested-s->reference_q_a));
    if(low_weight>0 && error*(requested-next)>0)
    {
        /* 条件冻结积分；不把比例项的高速变化回灌积分，避免持续偏速。 */
        s->integral_a=previous_integral;
        requested=fmaxf(-limit,fminf(limit,kp*error+s->integral_a+ripple));
        next=s->reference_q_a+fmaxf(-slew,fminf(slew,requested-s->reference_q_a));
    }
    s->reference_q_a=next;
    s->ripple_frozen=voltage_limited || fabsf(requested-next)>.000001f || fabsf(requested)>=limit;
}
