#include "foc_drive.h"
#include "foc_voltage.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief 电气和机械耦合模型，覆盖停转角度与真实两段PWM作用。
 * @param target 机械目标RPM。
 * @param current_limit 电流指令上限A。
 * @param actual_r 植物电阻Ω。
 * @param actual_l 植物电感H。
 * @param drop 植物电流方向压降V。
 * @param viscous_drag 粘性阻尼Nm/(rad/s)，未标定的植物敏感性参数。
 * @param initial_theta 初始转子电角度rad；静止时不能从观察器获知。
 * @param disturbance 加入未标定的静摩擦/齿槽应力模型，不作为实物参数。
 */
static void
check_free_running(uint32_t target,float current_limit,float actual_r,float actual_l,float drop,float viscous_drag,float initial_theta,bool disturbance)
{
    foc_drive_t d;
    foc_drive_config_t config={.resistance_ohm=.79f,.inductance_h=.00024f,
        .current_limit_a=current_limit,.emf_noise_v=.005f,.current_bandwidth_hz=800,
        .voltage_error_bound_v=.12f,
        .target_rpm=target,.pole_pairs=3,.speed_kp=.0002f,.speed_ki=.0005f,.speed_ramp_rpm_s=200};
    assert(foc_drive_init(&d,&config));
    assert(d.stage==FOC_CALIBRATE);
    foc_drive_start(&d);
    float ia=0,ib=0,old_a=0,old_b=0,theta=initial_theta,mechanical_omega=0,last_applied_a=0,last_applied_b=0;
    float peak_current=0,blend_reference_error=0,last_rpm=0;
    /* 核对定位/强拖交界的定子指令连续性，以及强拖阶段的真实机械跟随误差。 */
    float previous_reference_a=0,previous_reference_b=0,peak_ramp_error=0;
    float startup_current=fminf(1.0f,current_limit);
    uint32_t closed_at=0,ramp_begun=0;
    for(unsigned k=0;k<180000;++k)
    {
        foc_voltage_interval_t applied={.duration_ticks=17000,
            .alpha_v=old_a,.beta_v=old_b};
        /* 本次电流已经由下方两段真实电压产生，applied存前一次区间平均值。 */
        applied.alpha_v=k==0?0:last_applied_a;
        applied.beta_v=k==0?0:last_applied_b;
        uint16_t ccr[3];
        bool ok=foc_drive_sample(&d,k==0?NULL:&applied,ia+.002f*sinf(k*1.37f),ib+.002f*cosf(k*1.91f),
            12,k+1,.00010037647f,ccr);
        if(!ok) { fprintf(stderr,"drive target %lu failed k=%u: %s\n",(unsigned long)target,k,d.fault); }
        assert(ok);
        /* 启动/融合及提交后Id退出期间禁用；真正CLOSED后支路才学习。 */
        bool harmonic_expected=d.stage==FOC_CLOSED && d.samples-1U-d.closed_begun>=1000U;
        assert(d.current.harmonic_enabled==harmonic_expected);
        assert(d.current.harmonic2.enabled==harmonic_expected);
        assert(d.current.harmonic7.enabled==harmonic_expected);
        if(d.stage==FOC_I_F_RAMP && ramp_begun==0U) { ramp_begun=k; }
        assert(hypotf(d.reference_d_a,d.reference_q_a)<=current_limit+.00001f);
        float reference_a=d.reference_d_a*cosf(d.control_theta_rad)-d.reference_q_a*sinf(d.control_theta_rad);
        float reference_b=d.reference_d_a*sinf(d.control_theta_rad)+d.reference_q_a*cosf(d.control_theta_rad);
        if(d.stage==FOC_ALIGN || d.stage==FOC_I_F_RAMP || d.stage==FOC_QUALIFY)
        {
            /* 10kHz下幅度缓升和50Hz旋转均应连续，禁止交界额外转向90度。 */
            assert(hypotf(reference_a-previous_reference_a,reference_b-previous_reference_b)<.032f*startup_current+.00001f);
        }
        previous_reference_a=reference_a;
        previous_reference_b=reference_b;
        if(d.stage==FOC_BLEND)
        {
            /* dq坐标旋转前后仍指向强拖d轴，相同定子电流矢量。 */
            float a=d.reference_d_a*cosf(d.control_theta_rad)-d.reference_q_a*sinf(d.control_theta_rad);
            float b=d.reference_d_a*sinf(d.control_theta_rad)+d.reference_q_a*cosf(d.control_theta_rad);
            float error=hypotf(a-startup_current*cosf(d.force_theta_rad),b-startup_current*sinf(d.force_theta_rad));
            if(error>blend_reference_error) { blend_reference_error=error; }
        }
        if(d.awaiting_commit)
        {
            /* 写预装载/错序/非估计角记录均不能报告CLOSED或计算速度PI。 */
            foc_drive_commit(&d,k,false);
            foc_drive_commit(&d,k+1,false);
            foc_drive_speed_tick(&d,target);
            assert(d.stage==FOC_BLEND&&d.speed_ticks==0);
        }
        foc_voltage_command_t next={.ccr={ccr[0],ccr[1],ccr[2]},.bus_v=12};
        assert(foc_voltage_prepare(&next,8500));
        for(unsigned half=0;half<2;++half)
        {
            float dt=half?.00004962353f:.00005037647f;
            float rotor=theta+mechanical_omega*3*dt*.5f;
            float emf=mechanical_omega*3*.0042f;
            float ea=-emf*sinf(rotor),eb=emf*cosf(rotor);
            /* 已施加PWM平均值仍按CCR记录；实际功率级另有沿电流方向的压降。 */
            float magnitude=hypotf(ia,ib);
            float da=magnitude>.01f?drop*ia/magnitude:0;
            float db=magnitude>.01f?drop*ib/magnitude:0;
            float decay=expf(-actual_r*dt/actual_l);
            ia=decay*ia+(1-decay)*((half?next.alpha_v:old_a)-ea-da)/actual_r;
            ib=decay*ib+(1-decay)*((half?next.beta_v:old_b)-eb-db)/actual_r;
            float iq=-ia*sinf(rotor)+ib*cosf(rotor);
            /* 1mNm静摩擦与3mNm齿槽扰动仅用于覆盖未知停转位置的敏感性。 */
            float torque=.0189f*iq-viscous_drag*mechanical_omega-(disturbance?.003f*sinf(4*rotor):0);
            float friction=disturbance?.001f:0;
            if(fabsf(mechanical_omega)<.002f && fabsf(torque)<=friction)
            {
                torque=0;
                mechanical_omega=0;
            }
            else
            {
                torque-=copysignf(friction,fabsf(mechanical_omega)<.002f?torque:mechanical_omega);
            }
            float acceleration=torque/.000003f;
            mechanical_omega+=acceleration*dt;
            theta+=mechanical_omega*3*dt;
            if(theta>3.14159265f) { theta-=6.28318531f; }
            if(theta<-3.14159265f) { theta+=6.28318531f; }
        }
        last_applied_a=old_a*.5037647f+next.alpha_v*.4962353f;
        last_applied_b=old_b*.5037647f+next.beta_v*.4962353f;
        old_a=next.alpha_v;
        old_b=next.beta_v;
        foc_drive_commit(&d,k+1,d.request_estimated_angle);
        if(d.stage==FOC_CLOSED&&closed_at==0) { closed_at=k+1; }
        if((k+1)%10==0) { foc_drive_speed_tick(&d,target); }
        float magnitude=hypotf(ia,ib);
        if(magnitude>peak_current) { peak_current=magnitude; }
        last_rpm=mechanical_omega*9.54929658551f;
        if(d.stage==FOC_I_F_RAMP && k-ramp_begun>=500U)
        {
            peak_ramp_error=fmaxf(peak_ramp_error,fabsf(last_rpm-d.force_omega_rad_s*3.18309886184f));
        }
        if(k>130000) { assert(fabsf(last_rpm-target)<target*.05f); }
    }
    fprintf(stderr,"drive target %lu limit %.2f A R %.3f L %.0f uH drop %.3f V closed %.4f s final %.2f RPM peak %.4f A blend error %.8f A\n",
        (unsigned long)target,(double)current_limit,(double)actual_r,(double)(actual_l*1e6f),(double)drop,
        closed_at*.0001,(double)last_rpm,(double)peak_current,(double)blend_reference_error);
    assert(d.closed_seen&&closed_at<47000U);
    assert(blend_reference_error<.0001f);
    /* 此上限是既有植物模型的回归界限，不能冒充实物速度/惯量验收。 */
    /* 低阻尼新增模型允许更大的初始摆动；既有名义50/扰动200RPM界限不变。 */
    assert(peak_ramp_error<(viscous_drag<.000010f?500:disturbance?200:50));
    /* 强拖取用户限流与1A上限的小值；此空载模型接管后电流不应超过该幅度。 */
    assert(peak_current<startup_current+.05f);
    assert(d.speed_ticks>10000);
    foc_voltage_interval_t missing={.duration_ticks=34000};
    uint16_t ccr[3];
    assert(!foc_drive_sample(&d,&missing,ia,ib,12,180001,.00010037647f,ccr));
    assert(d.stage==FOC_STOPPED&&!d.awaiting_commit&&!d.request_estimated_angle);
}

/** @brief 静止模型仅响应旋转电压，必须资格超时关断，不能假闭环。 */
static void
check_stalled(float actual_r,float actual_l,float drop)
{
    foc_drive_t d;
    foc_drive_config_t config={.resistance_ohm=.79f,.inductance_h=.00024f,
        .current_limit_a=.5f,.emf_noise_v=.005f,.current_bandwidth_hz=800,
        .voltage_error_bound_v=.12f,
        .target_rpm=1000,.pole_pairs=3,.speed_kp=.0001f,.speed_ki=.0005f,.speed_ramp_rpm_s=200};
    assert(foc_drive_init(&d,&config));
    foc_drive_start(&d);
    float ia=0,ib=0,old_a=0,old_b=0,last_a=0,last_b=0;
    bool failed=false;
    for(unsigned k=0;k<60000;++k)
    {
        foc_voltage_interval_t applied={.duration_ticks=17000,.alpha_v=last_a,.beta_v=last_b};
        uint16_t ccr[3];
        if(!foc_drive_sample(&d,k==0?NULL:&applied,ia+.002f*sinf(k*1.37f),ib+.002f*cosf(k*1.91f),
            12,k+1,.00010037647f,ccr)) { failed=true; break; }
        foc_voltage_command_t next={.ccr={ccr[0],ccr[1],ccr[2]},.bus_v=12};
        assert(foc_voltage_prepare(&next,8500));
        float a=expf(-actual_r*.00005037647f/actual_l),b=expf(-actual_r*.00004962353f/actual_l);
        float norm=hypotf(ia,ib);
        float da=norm>.01f?drop*ia/norm:0,db=norm>.01f?drop*ib/norm:0;
        ia=a*ia+(1-a)*(old_a-da)/actual_r;
        ib=a*ib+(1-a)*(old_b-db)/actual_r;
        ia=b*ia+(1-b)*(next.alpha_v-da)/actual_r;
        ib=b*ib+(1-b)*(next.beta_v-db)/actual_r;
        last_a=old_a*.5037647f+next.alpha_v*.4962353f;
        last_b=old_b*.5037647f+next.beta_v*.4962353f;
        old_a=next.alpha_v;
        old_b=next.beta_v;
        foc_drive_commit(&d,k+1,d.request_estimated_angle);
        if((k+1)%10==0) { foc_drive_speed_tick(&d,1000); }
        assert(!d.closed_seen&&d.speed_ticks==0);
    }
    assert(failed&&!d.closed_seen&&d.speed_ticks==0&&strcmp(d.fault,"FOC QUALIFY TIMEOUT")==0);
}

/** @brief 固定Iq只允许已接管且Id退出，暂停速度调节并验证连续恢复和停机复位。 */
static void
check_hold_iq(void)
{
    foc_drive_t d={.stage=FOC_CLOSED,.config={.current_limit_a=.5f,.pole_pairs=3,
        .speed_kp=.0002f,.speed_ki=.001f,.speed_ramp_rpm_s=1000},
        .speed_reference_rpm=2000,.speed_q_a=.12f,.speed_integral_a=.12f};
    d.observer.omega_rad_s=600;
    assert(!foc_drive_hold_iq(NULL,.1f));
    assert(!foc_drive_hold_iq(&d,NAN));
    assert(!foc_drive_hold_iq(&d,-.1f));
    assert(!foc_drive_hold_iq(&d,.501f));
    d.reference_d_a=.01f;
    assert(!foc_drive_hold_iq(&d,.12f));
    d.reference_d_a=0;d.stage=FOC_BLEND;
    assert(!foc_drive_hold_iq(&d,.12f));
    d.stage=FOC_CLOSED;
    assert(foc_drive_hold_iq(&d,.12f));
    for(unsigned k=0;k<100;++k) { foc_drive_speed_tick(&d,1000); }
    assert(d.hold_iq && d.speed_q_a==.12f && d.speed_integral_a==.12f && d.speed_reference_rpm==2000);
    assert(foc_drive_hold_iq(&d,0) && !d.hold_iq);
    foc_drive_speed_tick(&d,2000);
    assert(fabsf(d.speed_q_a-.12f)<.001f);
    assert(foc_drive_hold_iq(&d,.12f));
    foc_drive_stop(&d,NULL);
    assert(!d.hold_iq && !foc_drive_hold_iq(&d,.12f));
    assert(!d.current.harmonic7.enabled && d.current.harmonic7.x_v==0 && d.current.harmonic7.y_v==0);
}

/** @brief 为保护测试合成具有独立EMF幅度及可控相位的已初始化观察器区间。
 * @param d 已初始化的闭环状态。 @param rpm 测试机械估速。
 * @param phase EMF与PLL预测方向的电角差rad。 @param emf 独立EMF幅度V。
 * @return 正式采样入口是否允许继续输出；不替代机械带载模型。 */
static bool
guard_sample(foc_drive_t *d,float rpm,float phase,float emf)
{
    foc_smo_t *o=&d->observer;
    float omega=rpm/3.18309886184f,angle=omega*.0001f+phase;
    float ea=-emf*sinf(angle),eb=emf*cosf(angle);
    assert(foc_smo_compensate(o->lpf_a,-omega,.0001f,&ea,&eb));
    assert(foc_smo_compensate(o->observer_pole,-omega,.0001f,&ea,&eb));
    o->sample_initialized=o->pll_initialized=true;
    o->omega_rad_s=o->omega_integral=omega;
    o->pll_theta_rad=0;
    o->previous_alpha_a=o->previous_beta_a=0;
    o->estimated_alpha_a=o->estimated_beta_a=0;
    o->filtered_alpha_v=ea;o->filtered_beta_v=eb;
    float factor=(o->boundary_a+o->plant_b*o->gain_v)/(o->plant_b*o->gain_v);
    foc_voltage_interval_t u={.duration_ticks=17000,.alpha_v=ea*factor,.beta_v=eb*factor};
    /* 残差预测保留因果的前一区间EMF，避免用本轮测量重置被测试的残差状态。 */
    o->emf_alpha_v=u.alpha_v*cosf(omega*.0001f)+u.beta_v*sinf(omega*.0001f);
    o->emf_beta_v=-u.alpha_v*sinf(omega*.0001f)+u.beta_v*cosf(omega*.0001f);
    uint16_t ccr[3];
    return foc_drive_sample(d,&u,0,0,12,d->samples+1,.00010037647f,ccr);
}

/** @brief 越限恢复清计时，连续100ms关断；相位/EMF/模型错误不等待速度计时。 */
static void
check_speed_guards(void)
{
    const foc_drive_config_t c={.resistance_ohm=.79f,.inductance_h=.00024f,
        .current_limit_a=.5f,.current_bandwidth_hz=800,.voltage_error_bound_v=.12f,
        .target_rpm=2000,.pole_pairs=3,.speed_kp=.0002f,.speed_ki=.0005f,.speed_ramp_rpm_s=200};
    foc_drive_t d;
    assert(foc_drive_init(&d,&c));d.stage=FOC_CLOSED;d.samples=10000;
    for(unsigned k=0;k<999;++k) { assert(guard_sample(&d,2250,0,1)); }
    assert(d.speed_range_cycles==999);
    assert(guard_sample(&d,2000,0,1)&&d.speed_range_cycles==0);
    for(unsigned k=0;k<999;++k) { assert(guard_sample(&d,2250,0,1)); }
    assert(!guard_sample(&d,2250,0,1)&&strcmp(d.fault,"FOC SPEED HIGH")==0);
    assert(d.speed_range_cycles==1000);
    for(unsigned scenario=0;scenario<6;++scenario)
    {
        assert(foc_drive_init(&d,&c));d.stage=FOC_CLOSED;d.samples=10000;
        const char *expected;
        bool ok;
        if(scenario==0) { expected="FOC SPEED HARD";ok=guard_sample(&d,3050,0,1); }
        else if(scenario==1) { expected="FOC SPEED HARD";ok=guard_sample(&d,-100,0,1); }
        else if(scenario==2) { expected="FOC PHASE LOST";ok=guard_sample(&d,2000,.8f,1); }
        else if(scenario==3) { expected="FOC EMF LOW";ok=guard_sample(&d,2000,0,.2f); }
        else if(scenario==4) { expected="FOC PLL LOST";ok=guard_sample(&d,2000,0,0); }
        else { expected="FOC MODEL ERROR";d.observer.residual_a=.2f;ok=guard_sample(&d,2000,0,1); }
        assert(!ok && d.samples==10000 && strcmp(d.fault,expected)==0);
    }
    assert(foc_drive_init(&d,&c));d.stage=FOC_CLOSED;d.samples=10000;
    for(unsigned k=0;k<999;++k) { assert(guard_sample(&d,550,0,1)); }
    assert(!guard_sample(&d,550,0,1)&&strcmp(d.fault,"FOC SPEED LOW")==0);
}

/** @brief 扩展范围不屏蔽低EMF失锁，低速接管参考独立于最终目标且连续提交。 */
static void
check_extended_range(void)
{
    foc_drive_config_t c={.resistance_ohm=.79f,.inductance_h=.00024f,
        .current_limit_a=1,.current_bandwidth_hz=800,.voltage_error_bound_v=.12f,
        .target_rpm=5400,.pole_pairs=3,.speed_kp=.0004f,.speed_ki=.0005f,
        .speed_ramp_rpm_s=200,.startup_rpm=600,.extended_speed_range=true};
    foc_drive_t d;
    assert(foc_drive_init(&d,&c));
    assert(fabsf(d.startup_omega_rad_s-188.4955592f)<.0001f);
    d.stage=FOC_CLOSED;d.samples=10000;
    for(unsigned k=0;k<1001;++k) { assert(guard_sample(&d,100,0,1)); }
    assert(d.speed_range_cycles==0);
    assert(guard_sample(&d,5400,0,1));
    for(unsigned k=0;k<999;++k) { assert(guard_sample(&d,5850,0,1)); }
    assert(!guard_sample(&d,5850,0,1)&&strcmp(d.fault,"FOC SPEED HIGH")==0);
    assert(foc_drive_init(&d,&c));d.stage=FOC_CLOSED;d.samples=10000;
    assert(!guard_sample(&d,100,0,.2f)&&strcmp(d.fault,"FOC EMF LOW")==0);
    assert(foc_drive_init(&d,&c));
    d.stage=FOC_BLEND;d.awaiting_commit=true;d.pending_sequence=5;
    d.reference_q_a=.1f;d.observer.omega_rad_s=188.4955592f;
    foc_drive_commit(&d,5,true);
    assert(d.stage==FOC_CLOSED&&d.speed_reference_rpm==600);
    foc_drive_speed_tick(&d,5400);
    assert(fabsf(d.speed_reference_rpm-600.2f)<.0001f);
    c.target_rpm=5401;assert(!foc_drive_init(&d,&c));
    c.target_rpm=1;c.startup_rpm=1001;assert(!foc_drive_init(&d,&c));
    c.startup_rpm=1;assert(foc_drive_init(&d,&c));
}

/* 电气和机械参数沿用已有test_foc_drive模型；不能当作实物惯量/转矩标定。 */
/** @brief 沿用未标定的电气/机械植物，检查启动及加卸负载，不代表实物负载验收。
 * @param name 场景。 @param load 摩擦Nm。 @param on 施加时刻s。 @param off 卸载时刻s。
 * @param kp_scale 原Kp的倍数。 @param current_limit 用户电流限制A。
 * @param expected_fault 预期故障；NONE表示必须完成18s及真实接管。
 * @param inertia 转动惯量kg·m²。 @param drag 粘性阻尼Nm/(rad/s)。
 * @param ripple 是否叠加12Hz、15%摩擦变化，检查资格阶段的同步摆动。 */
static void
check_load_case(const char *name,float load,float on,float off,float kp_scale,float current_limit,const char *expected_fault,
    float inertia,float drag,bool ripple) {
    foc_drive_t d;
    foc_drive_config_t c={.resistance_ohm=.79f,.inductance_h=.00024f,
        .current_limit_a=current_limit,.emf_noise_v=.005f,.current_bandwidth_hz=800,
        .voltage_error_bound_v=.12f,.target_rpm=2000,.pole_pairs=3,
        .speed_kp=.0002f*kp_scale,.speed_ki=.0005f,.speed_ramp_rpm_s=200};
    assert(foc_drive_init(&d,&c)); foc_drive_start(&d);
    float ia=0,ib=0,old_a=0,old_b=0,last_a=0,last_b=0,theta=.4f,wm=0;
    float max_iq=0,max_current=0,min_loaded_rpm=10000,max_released_rpm=0;
    /* 变化摩擦的稳态允许周期速度响应，检查整段误差而非任意末点相位。 */
    double steady_squared_error=0;
    float steady_peak_error=0;
    unsigned steady_count=0;
    unsigned closed_at=0,k;
    for(k=0;k<180000;++k) {
        foc_voltage_interval_t u={.duration_ticks=17000,.alpha_v=last_a,.beta_v=last_b};
        uint16_t ccr[3];
        if(!foc_drive_sample(&d,k==0?NULL:&u,ia+.002f*sinf(k*1.37f),ib+.002f*cosf(k*1.91f),12,k+1,.00010037647f,ccr)) break;
        assert(hypotf(d.reference_d_a,d.reference_q_a)<=current_limit+.00001f);
        foc_voltage_command_t next={.ccr={ccr[0],ccr[1],ccr[2]},.bus_v=12};
        assert(foc_voltage_prepare(&next,8500));
        for(unsigned half=0;half<2;++half) {
            float dt=half?.00004962353f:.00005037647f;
            float rotor=theta+wm*3*dt*.5f;
            float emf=wm*3*.0042f,ea=-emf*sinf(rotor),eb=emf*cosf(rotor);
            float decay=expf(-.79f*dt/.00024f);
            ia=decay*ia+(1-decay)*((half?next.alpha_v:old_a)-ea)/.79f;
            ib=decay*ib+(1-decay)*((half?next.beta_v:old_b)-eb)/.79f;
            float iq=-ia*sinf(rotor)+ib*cosf(rotor);
            float torque=.0189f*iq-drag*wm;
            float elapsed=k*.0001f;
            float friction=(elapsed>=on && elapsed<off)?load:0;
            if(ripple) { friction*=1+.15f*sinf(elapsed*75.3982237f); }
            if(fabsf(wm)<.002f && fabsf(torque)<=friction) { torque=0; wm=0; }
            else { torque-=copysignf(friction,fabsf(wm)<.002f?torque:wm); }
            wm+=torque/inertia*dt;
            theta+=wm*3*dt;
            if(theta>3.14159265f) theta-=6.28318531f;
            if(theta<-3.14159265f) theta+=6.28318531f;
        }
        last_a=old_a*.5037647f+next.alpha_v*.4962353f;
        last_b=old_b*.5037647f+next.beta_v*.4962353f;
        old_a=next.alpha_v; old_b=next.beta_v;
        foc_drive_commit(&d,k+1,d.request_estimated_angle);
        if(d.stage==FOC_CLOSED && !closed_at) closed_at=k+1;
        if((k+1)%10==0) foc_drive_speed_tick(&d,2000);
        max_current=fmaxf(max_current,hypotf(ia,ib));
        max_iq=fmaxf(max_iq,d.reference_q_a);
        float rpm=wm*9.54929658551f,elapsed=k*.0001f;
        if(elapsed>=on && elapsed<off) min_loaded_rpm=fminf(min_loaded_rpm,rpm);
        if(elapsed>=off) max_released_rpm=fmaxf(max_released_rpm,rpm);
        if(elapsed>=10)
        {
            float error=fabsf(rpm-2000);
            steady_squared_error+=error*error;
            steady_peak_error=fmaxf(steady_peak_error,error);
            ++steady_count;
        }
    }
    fprintf(stderr,"{\"type\":\"foc_model\",\"name\":\"%s\",\"load_nm\":%.6f,\"on_s\":%.2f,\"off_s\":%.2f,\"kp_scale\":%.2f,\"current_limit_a\":%.2f,\"end_s\":%.4f,\"closed_s\":%.4f,\"fault\":\"%s\",\"actual_model_rpm\":%.2f,\"estimated_rpm\":%.2f,\"max_iq_ref_a\":%.5f,\"peak_current_a\":%.5f,\"min_loaded_rpm\":%.2f,\"max_released_rpm\":%.2f,\"phase_error\":%.6f,\"residual_a\":%.6f}\n",
        name,(double)load,(double)on,(double)off,(double)kp_scale,(double)current_limit,
        k*.0001,closed_at*.0001,d.fault,(double)(wm*9.54929658551f),
        (double)(d.observer.omega_rad_s*3.18309886184f),(double)max_iq,(double)max_current,
        (double)min_loaded_rpm,(double)max_released_rpm,(double)d.observer.phase_error,(double)d.observer.residual_a);
    assert(strcmp(d.fault,expected_fault)==0);
    if(strcmp(expected_fault,"NONE")==0)
    {
        assert(k==180000 && d.closed_seen && closed_at>0 && d.speed_ticks>10000);
        if(ripple)
        { assert(steady_count>79000 && sqrt(steady_squared_error/steady_count)<80 && steady_peak_error<150); }
        else { assert(fabsf(wm*9.54929658551f-2000)<30); }
    }
    else if(strcmp(expected_fault,"FOC QUALIFY TIMEOUT")==0)
    { assert(!d.closed_seen && d.speed_ticks==0); }
    else if(strcmp(expected_fault,"FOC SPEED HIGH")==0)
    { assert(d.closed_seen && d.speed_range_cycles==1000); }
}

/** @brief 全启动链覆盖整个停转电角度、真实提交门控、速度及失锁关断。
 * @return 所有模型场景通过时返回0。 */
int
main(void)
{
    check_hold_iq();
    check_speed_guards();
    check_extended_range();
    /* 强拖随限流建立余量；同一负载在0.5A时必须拒绝接管，在1/2A配置时可接管。 */
    check_load_case("loaded_start_limit_05A",.009f,0,30,1,.5f,"FOC QUALIFY TIMEOUT",.000003f,.000020f,false);
    check_load_case("loaded_start_limit_1A",.009f,0,30,1,1,"NONE",.000003f,.000020f,false);
    check_load_case("loaded_start_limit_2A",.009f,0,30,1,2,"NONE",.000003f,.000020f,false);
    /* 此低阻尼/变化摩擦模型在资格阶段退出功率阻尼时会超时，必须连续200ms合格再接管。 */
    check_load_case("qualify_damping_ripple_load",.009f,0,30,2,1,"NONE",.000003f,.000001f,true);
    /* 保护计时不能掩盖持续超调；较强Kp为模型候选，保留旧保存参数由板上逐级验证。 */
    check_load_case("load_release_old_kp",.002f,8,11,1,.5f,"FOC SPEED HIGH",.000003f,.000020f,false);
    for(unsigned load=1;load<=5;++load)
    { check_load_case("load_release_kp_1000",load*.001f,8,11,5,.5f,"NONE",.000003f,.000020f,false); }
    for(unsigned limit=0;limit<3;++limit)
    {
        float current_limit=limit==0?.3f:limit==1?.5f:2.0f;
        check_free_running(1000,current_limit,.79f,.00024f,0,.000020f,.4f,false);
        check_free_running(1500,current_limit,.79f,.00024f,0,.000020f,.4f,false);
        check_free_running(2000,current_limit,.79f,.00024f,0,.000020f,.4f,false);
    }
    /* 量测附近R、L±20%、保守功率级压降和噪声同时存在仍可正常启动。 */
    for(unsigned r=0;r<3;++r) for(unsigned l=0;l<3;++l)
    {
        float actual_r=.76f+r*.05f,actual_l=.00024f*(.8f+l*.2f);
        check_free_running(1000,.5f,actual_r,actual_l,.12f,.000020f,.4f,false);
        for(unsigned v=0;v<4;++v) { check_stalled(actual_r,actual_l,.06f+v*.02f); }
    }
    check_stalled(.79f,.00024f,0);
    /* 原测试只有0.4rad初角；新增72个初角，包含与定位电流反向的死点附近。 */
    for(unsigned angle=0;angle<72;++angle)
    {
        float initial=-3.14159265f+angle*6.28318531f/72;
        check_free_running(1000,.5f,.79f,.00024f,.08f,.000020f,initial,false);
        check_free_running(1000,.5f,.79f,.00024f,.08f,.000020f,initial,true);
        /* 低阻尼模型曾反复摆动/资格超时；仍需真实提交、稳定运行及失锁关断。 */
        check_free_running(1000,.5f,.79f,.00024f,.08f,.000001f,initial,true);
    }
    puts("FOC drive: startup, qualification, blend, real commit, speed loop and shutdown PASS");
    return 0;
}
