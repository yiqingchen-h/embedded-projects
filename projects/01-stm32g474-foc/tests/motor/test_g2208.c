/* 复用板卡/Flash替身，执行真实参数、请求后端及FOC算法，不打开硬件。 */
#define MOTOR_FOC_BACKEND_FIXTURE_ONLY
#include "test_foc_backend.c"
#include "sensorless_six_step.h"
#include <stddef.h>

/* 冻结V3记录长度及CRC位置，防止迁移时误改布局。 */
typedef struct {
    uint32_t magic,version,length,sequence;
    struct { int32_t parameter[35]; uint32_t brightness,animation,algorithm,mode,source; settings_foc_t foc; } data;
    uint32_t crc,commit;
} profile_record_t;
typedef struct { uint32_t magic,version,length,sequence; settings_data_t data; uint32_t crc,commit; } current_record_t;
_Static_assert(sizeof(profile_record_t)==224 && offsetof(profile_record_t,crc)==216,"V3 layout frozen");

/** @brief 计算独立测试记录的标准CRC32。@param bytes 记录。@param size 覆盖字节。@return CRC32。 */
static uint32_t
profile_crc(const void *bytes,size_t size)
{
    const unsigned char *p=bytes;
    uint32_t crc=UINT32_MAX;
    for(size_t i=0;i<size;++i)
    {
        crc^=p[i];
        for(unsigned bit=0;bit<8;++bit) { crc=(crc>>1)^((crc&1)?0xedb88320U:0); }
    }
    return ~crc;
}

/** @brief 建立与原电机实板配置同量级的旧记录，故意保留旧速度/大电流及编码器零点。@return CRC正确的原记录。 */
static profile_record_t
original_record(void)
{
    profile_record_t record={.magic=0x464f4332,.version=3,.length=200,.sequence=156,.commit=0x434f4d54,
        .data={.parameter={1000,3,1500,3000,-1800,1800,0,0,0,0,0,0,0,0,0,50000,20000,200,0,0,
            0,0,0,-1,1234,0,0,0,0,50,12000,0,5400,200,20000},
            .brightness=73,.animation=0,.algorithm=CONTROL_SENSORLESS_6STEP,.mode=CONTROL_SPEED,.source=CONTROL_REMOTE,
            .foc={2000,1000,790,240,800,400,500,200000,20000,0}}};
    record.crc=profile_crc(&record,offsetof(profile_record_t,crc));
    return record;
}

/** @brief 清空内存存储与通信模型，不访问实板。 */
static void
profile_reset(void)
{
    memset(flash,255,sizeof(flash));
    writes=0;cut_after=UINT_MAX;layout=calibration_ok=true;calibration_failures_left=0;
    calibrated_bus_mv=0;start_failure_reason=NULL;
    control_model_init();settings_init();
}

/** @brief 核对新电机参数没有残留旧极对数、模型、运行时长或标定值。 */
static void
check_new_values(void)
{
    const settings_data_t *s=settings_get();
    CHECK(s->parameter[CONTROL_PARAM_POLES]==7 && s->parameter[CONTROL_PARAM_SHUNT]==50);
    CHECK(s->parameter[CONTROL_PARAM_RATED_VOLTAGE]==12000 && s->parameter[CONTROL_PARAM_RATED_CURRENT]==800);
    CHECK(s->parameter[CONTROL_PARAM_RATED_SPEED]==1800 && s->parameter[CONTROL_PARAM_SPEED_LIMIT]==1800);
    CHECK(s->parameter[CONTROL_PARAM_TARGET]==600 && s->parameter[CONTROL_PARAM_CURRENT_LIMIT]==500);
    CHECK(s->parameter[CONTROL_PARAM_RESISTANCE]==10100 && s->parameter[CONTROL_PARAM_LD]==1500);
    CHECK(s->parameter[CONTROL_PARAM_ENCODER_DIR]==1 && s->parameter[CONTROL_PARAM_ELEC_ZERO]==0);
    CHECK(s->foc.resistance_mohm==10100 && s->foc.inductance_uh==1500 && s->foc.bandwidth_hz==800);
    CHECK(s->foc.target_rpm==600 && s->foc.current_limit_ma==500 && s->foc.run_ms==5000);
    CHECK(s->foc.speed_kp_micro==400 && s->foc.speed_ki_micro==500);
}

/** @brief 同电机V3升级V4保留用户速度、50秒时限和零点，旧保留槽仍只接受零。 */
static void
test_current_v3_migration(void)
{
    profile_reset();
    profile_record_t old=original_record();
    memcpy(&old.data,settings_get(),sizeof(old.data));old.version=3U;old.sequence=1333U;
    old.data.algorithm=CONTROL_ENCODER_FOC;old.data.mode=CONTROL_SPEED;old.data.source=CONTROL_REMOTE;
    old.data.parameter[CONTROL_PARAM_ELEC_ZERO]=504;
    old.data.foc.target_rpm=1000;old.data.foc.run_ms=50000;
    old.data.foc.speed_kp_micro=100;old.data.foc.speed_ki_micro=200;
    old.crc=profile_crc(&old,offsetof(profile_record_t,crc));
    for(unsigned cut=0;cut<=sizeof(current_record_t);cut+=8)
    {
        profile_reset();memcpy(flash[0],&old,sizeof(old));settings_init();
        CHECK(memcmp(settings_get(),&old.data,sizeof(old.data))==0);
        CHECK(strcmp(settings_status(),"PENDING")==0 && settings_get()->foc.target_iq_ma==0);
        cut_after=cut;settings_service(true,2000);
        CHECK(memcmp(flash[0],&old,sizeof(old))==0);
        settings_init();CHECK(memcmp(settings_get(),&old.data,sizeof(old.data))==0);
        CHECK(strcmp(settings_status(),cut<sizeof(current_record_t)?"PENDING":"SAVED")==0);
    }
    profile_reset();old.data.foc.target_iq_ma=1;
    old.crc=profile_crc(&old,offsetof(profile_record_t,crc));memcpy(flash[0],&old,sizeof(old));settings_init();
    CHECK(strcmp(settings_status(),"DEFAULT")==0);
}

/** @brief 冻结V4负Iq记录升级V5，保留所有200字节，并逐一验证新记录掉电边界。 */
static void
test_current_v4_migration(void)
{
    profile_reset();
    profile_record_t old=original_record();
    memcpy(&old.data,settings_get(),sizeof(old.data));
    old.version=4U;old.sequence=1457U;old.data.algorithm=CONTROL_ENCODER_FOC;old.data.mode=CONTROL_CURRENT;
    old.data.foc.target_iq_ma=-123;old.data.foc.target_rpm=650;old.data.parameter[CONTROL_PARAM_ELEC_ZERO]=526;
    old.crc=profile_crc(&old,offsetof(profile_record_t,crc));
    for(unsigned cut=0;cut<=sizeof(current_record_t);cut+=8U)
    {
        profile_reset();memcpy(flash[0],&old,sizeof(old));settings_init();
        CHECK(memcmp(settings_get(),&old.data,sizeof(old.data))==0);
        CHECK(settings_get()->position.kp_milli==8000 && settings_get()->position.tolerance_mdeg==100);
        cut_after=cut;settings_service(true,2000);
        CHECK(memcmp(flash[0],&old,sizeof(old))==0);
        settings_init();
        CHECK(memcmp(settings_get(),&old.data,sizeof(old.data))==0);
        CHECK(strcmp(settings_status(),cut<sizeof(current_record_t)?"PENDING":"SAVED")==0);
    }
}

/** @brief 旧槽完整性、择新、停机另槽保存和全部双字掉电边界必须保留。 */
static void
test_profile_migration(void)
{
    profile_record_t old=original_record();
    for(unsigned cut=0;cut<=sizeof(current_record_t);cut+=8)
    {
        profile_reset();
        memcpy(flash[0],&old,sizeof(old));
        settings_init();
        check_new_values();
        CHECK(settings_get()->brightness==73 && settings_get()->animation==0 && settings_get()->source==CONTROL_REMOTE);
        CHECK(settings_get()->algorithm==CONTROL_SENSORLESS_6STEP);
        CHECK(strcmp(settings_status(),"PENDING")==0 && writes==0);
        settings_service(false,2000);settings_service(true,999);
        CHECK(writes==0);
        cut_after=cut;
        settings_service(true,1000);
        CHECK(writes==1 && memcmp(flash[0],&old,sizeof(old))==0);
        CHECK(strcmp(settings_status(),cut<sizeof(current_record_t)?"ERROR":"SAVED")==0);
        cut_after=UINT_MAX;
        settings_init();
        check_new_values();
        CHECK(strcmp(settings_status(),cut<sizeof(current_record_t)?"PENDING":"SAVED")==0);
        if(cut==sizeof(current_record_t))
        {
            current_record_t saved;
            memcpy(&saved,flash[1],sizeof(saved));
            CHECK(saved.sequence==157 && saved.version==6 && saved.length==248);
            CHECK(saved.crc==profile_crc(&saved,offsetof(current_record_t,crc)));
            settings_service(true,3000);CHECK(writes==1);
        }
    }
    /* 最新原记录超过新电机目标/电流上限仍应先合法择新，再迁移其序号。 */
    profile_reset();
    memcpy(flash[0],&old,sizeof(old));
    old.sequence=158;old.data.foc.target_rpm=5400;old.data.foc.current_limit_ma=2000;
    old.crc=profile_crc(&old,offsetof(profile_record_t,crc));
    memcpy(flash[1],&old,sizeof(old));
    settings_init();check_new_values();settings_service(true,1000);
    current_record_t saved;memcpy(&saved,flash[0],sizeof(saved));CHECK(saved.sequence==159);
    /* CRC正确但未知模型仍拒绝；不伪造已校验的电机身份。 */
    profile_reset();old=original_record();old.data.foc.resistance_mohm=9999;
    old.crc=profile_crc(&old,offsetof(profile_record_t,crc));memcpy(flash[0],&old,sizeof(old));
    settings_init();CHECK(strcmp(settings_status(),"DEFAULT")==0);check_new_values();
}

/** @brief 真实参数发布、START路由和在线目标都使用7极对新模型及新上限。 */
static void
test_profile_backend(void)
{
    profile_reset();check_new_values();settings_publish();
    const control_parameter_t *p=control_model_parameter(CONTROL_PARAM_POLES);
    CHECK(p->valid && !p->editable && p->value==7 && p->minimum==7 && p->maximum==7);
    CHECK(!settings_set(CONTROL_PARAM_POLES,3,0));
    CHECK(!settings_set(CONTROL_PARAM_CURRENT_LIMIT,801,0));
    CHECK(!settings_set(CONTROL_PARAM_TARGET,1801,0));
    sensorless_six_step_config_t six;
    sensorless_six_step_defaults(&six);
    CHECK(six.min_rpm==429);
    /* 真实六步START也应接受本档默认值，不再因旧机械资格门槛超过目标而拒绝。 */
    CHECK(control_model_select(CONTROL_SENSORLESS_6STEP,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_init();motor_backend_service(now=20);
    request(CONTROL_REQUEST_SOURCE,CONTROL_PARAM_TARGET,CONTROL_REMOTE);
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(hardware.running && strcmp(hardware.fault,"NONE")==0);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    CHECK(control_model_select(CONTROL_SENSORLESS_FOC,CONTROL_SPEED)==CONTROL_OK);
    motor_backend_init();motor_backend_service(now=20);
    request(CONTROL_REQUEST_SOURCE,CONTROL_PARAM_TARGET,CONTROL_REMOTE);
    CHECK(control_model_snapshot()->ready);
    CHECK(!settings_set(CONTROL_PARAM_RESISTANCE,790,now));
    CHECK(settings_set(CONTROL_PARAM_CURRENT_LIMIT,800,now));
    CHECK(!settings_set(CONTROL_PARAM_CURRENT_LIMIT,801,now));
    CHECK(settings_set(CONTROL_PARAM_CURRENT_LIMIT,500,now));
    request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0);
    CHECK(foc_active && received_config.pole_pairs==7 && received_duration==5000);
    CHECK(fabsf(received_config.resistance_ohm-10.1f)<.00001f && fabsf(received_config.inductance_h-.0015f)<.00000001f);
    CHECK(received_config.target_rpm==600 && fabsf(received_config.current_limit_a-.5f)<.00001f);
    commit_confirmed=true;motor_backend_service(now+=20);
    request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1800);
    CHECK(received_config.target_rpm==1800);
    CHECK(control_model_request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_TARGET,1801,now)==CONTROL_RANGE);
    request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0);
    CHECK(!foc_active && !hardware.running && control_model_snapshot()->power_disabled);
}

/** @brief 实际FOC初始化和速度PI验证机械/电角换算及限值，避免沿用原3极对系数。 */
static void
test_profile_drive(void)
{
    foc_drive_config_t c={.resistance_ohm=10.1f,.inductance_h=.0015f,.current_limit_a=.5f,
        .emf_noise_v=.005f,.current_bandwidth_hz=800,.voltage_error_bound_v=.12f,
        .target_rpm=600,.pole_pairs=7,.extended_speed_range=true,.speed_kp=.0004f,.speed_ki=.0005f,.speed_ramp_rpm_s=200};
    foc_drive_t d;
    CHECK(foc_drive_init(&d,&c));
    CHECK(fabsf(d.startup_omega_rad_s-219.9114858f)<.0001f);
    CHECK(fabsf(d.current.kp-7.53982237f)<.00001f);
    d.stage=FOC_CLOSED;d.speed_reference_rpm=700;
    d.observer.omega_rad_s=439.8229715f;
    foc_drive_speed_tick(&d,700);
    CHECK(fabsf(d.speed_q_a-.04005f)<.000001f);
    CHECK(fabsf(d.observer.omega_rad_s*MOTOR_FOC_RPM_PER_RAD_S-600)<.001f);
    c.pole_pairs=3;CHECK(!foc_drive_init(&d,&c));c.pole_pairs=7;
    c.target_rpm=1801;CHECK(!foc_drive_init(&d,&c));c.target_rpm=600;
    c.current_limit_a=.801f;CHECK(!foc_drive_init(&d,&c));
}

/** @brief 两段真实零阶保持RL模型核对新参数下的有限电流阶跃，不能代替实板。 */
static void
test_profile_current(void)
{
    foc_current_t loop;
    CHECK(foc_current_init(&loop,10.1f,.0015f,800,.0001f,8500,425,8075));
    float a=0,b=0,old_a=0,old_b=0,peak=0;
    float decay1=expf(-10.1f*.00005037647f/.0015f),decay2=expf(-10.1f*.00004962353f/.0015f);
    for(unsigned k=0;k<3000;++k)
    {
        float rd=k<1000?.2f:0,rq=k>=1000&&k<2000?.2f:0;
        uint16_t ccr[3];
        CHECK(foc_current_step(&loop,a,b,.8660254f,.5f,.8660254f,.5f,12,rd,rq,ccr));
        foc_voltage_command_t v={.ccr={ccr[0],ccr[1],ccr[2]},.bus_v=12};
        CHECK(foc_voltage_prepare(&v,8500));
        a=decay1*a+(1-decay1)*old_a/10.1f;b=decay1*b+(1-decay1)*old_b/10.1f;
        a=decay2*a+(1-decay2)*v.alpha_v/10.1f;b=decay2*b+(1-decay2)*v.beta_v/10.1f;
        old_a=v.alpha_v;old_b=v.beta_v;peak=fmaxf(peak,hypotf(a,b));
        if(k==999||k==1999||k==2999) { CHECK(fabsf(loop.id_a-rd)<.003f && fabsf(loop.iq_a-rq)<.003f); }
    }
    CHECK(peak<.28f);
}

/** @brief 为资格边界注入独立EMF区间，仅隔离裕量保护，不宣称是机械仿真。
 * @param d 初始化后的资格/融合状态。 @param emf 滑模滤波前幅值V。
 * @return 实际控制入口是否继续输出。 */
static bool
profile_margin_sample(foc_drive_t *d,float emf)
{
    foc_smo_t *o=&d->observer;
    float omega=219.91148575f,angle=omega*.0001f;
    float ea=-emf*sinf(angle),eb=emf*cosf(angle);
    CHECK(foc_smo_compensate(o->lpf_a,-omega,.0001f,&ea,&eb));
    CHECK(foc_smo_compensate(o->observer_pole,-omega,.0001f,&ea,&eb));
    o->sample_initialized=o->pll_initialized=true;
    o->omega_rad_s=o->omega_integral=omega;
    o->pll_theta_rad=0;
    o->filtered_alpha_v=ea;o->filtered_beta_v=eb;
    float factor=(o->boundary_a+o->plant_b*o->gain_v)/(o->plant_b*o->gain_v);
    foc_voltage_interval_t u={.duration_ticks=17000,.alpha_v=ea*factor,.beta_v=eb*factor};
    o->emf_alpha_v=u.alpha_v*cosf(angle)+u.beta_v*sinf(angle);
    o->emf_beta_v=-u.alpha_v*sinf(angle)+u.beta_v*cosf(angle);
    uint16_t ccr[3];
    return foc_drive_sample(d,&u,0,0,12,d->samples+1,.00010037647f,ccr);
}

/** @brief 低EMF裕量、模型界内、噪声抬高或残差异常仍拒绝资格/融合。 */
static void
test_profile_margin_protection(void)
{
    foc_drive_config_t c={.resistance_ohm=10.1f,.inductance_h=.0015f,.current_limit_a=.5f,
        .emf_noise_v=.007f,.current_bandwidth_hz=800,.voltage_error_bound_v=.12f,
        .target_rpm=600,.pole_pairs=7,.startup_rpm=300,.extended_speed_range=true,
        .speed_kp=.0004f,.speed_ki=.0005f,.speed_ramp_rpm_s=200};
    foc_drive_t d;
    for(unsigned scenario=0;scenario<6;++scenario)
    {
        c.emf_noise_v=scenario==3?.02f:.007f;
        CHECK(foc_drive_init(&d,&c));
        d.stage=scenario==0?FOC_QUALIFY:FOC_BLEND;
        d.samples=d.qualify_begun=d.blend_begun=22000;
        d.force_omega_rad_s=d.startup_omega_rad_s;
        if(scenario==4) { d.observer.residual_a=.1f; }
        bool output=profile_margin_sample(&d,scenario==2?.1f:scenario<=1?.25f:.19f);
        if(scenario<=1)
        {
            CHECK(d.emf_margin_v>.13f && d.emf_margin_v<.15f);
            CHECK(output && strcmp(d.fault,"NONE")==0);
            CHECK(scenario==0?d.qualify_cycles==1:d.stage==FOC_BLEND);
        }
        else
        {
            CHECK(!output && d.stage==FOC_STOPPED && !d.closed_seen);
            CHECK(strcmp(d.fault,"FOC BLEND LOST")==0);
        }
        if(scenario==2) { CHECK(d.emf_margin_v<0); }
        if(scenario==3) { CHECK(fabsf(d.emf_floor_v-.12f)<.000001f); }
        if(scenario==5) { CHECK(d.emf_margin_v>.07f && d.emf_margin_v<.09f); }
    }
}

/** @brief 强拖电流低于运行限流且连续定位，不降低六步429RPM资格。 */
static void
test_profile_start_current(void)
{
    const foc_drive_config_t c={.resistance_ohm=10.1f,.inductance_h=.0015f,.current_limit_a=.5f,
        .emf_noise_v=.007f,.current_bandwidth_hz=800,.voltage_error_bound_v=.12f,
        .target_rpm=600,.pole_pairs=7,.extended_speed_range=true,
        .speed_kp=.0004f,.speed_ki=.0005f,.speed_ramp_rpm_s=200};
    foc_drive_t d;
    CHECK(foc_drive_init(&d,&c));foc_drive_start(&d);
    foc_voltage_interval_t u={.duration_ticks=17000};
    uint16_t ccr[3];
    for(unsigned k=0;k<22000;++k)
    {
        CHECK(foc_drive_sample(&d,k?&u:NULL,0,0,12,k+1,.00010037647f,ccr));
        CHECK(d.reference_d_a<=.400001f && d.reference_q_a==0);
    }
    CHECK(fabsf(d.reference_d_a-.4f)<.000001f && d.config.current_limit_a==.5f);
    CHECK(!d.closed_seen && d.speed_ticks==0);
    sensorless_six_step_config_t six;sensorless_six_step_defaults(&six);CHECK(six.min_rpm==429);
}

/** @brief 验证最大调制的前沿建立和末通道保持，不以放宽电流和门限换取通过。
 * @note 使用已配置170MHz、20kHz/4250、95%最高CCR、200ns死区和6.5周期ADC采样。
 * CSA20的0.5V阶跃典型1us仅作时序回归依据，不能替代带载/温度下外部电流测量。 */
static void
test_profile_sampling_window(void)
{
    const uint32_t arr=4250U, maximum_ccr=8075U/2U;
    const uint32_t trigger=MOTOR_FOC_TRIGGER_DEFAULT_TICKS-arr;
    /* 最坏2.5周期同步加6.5周期采样为36tick，下一序列相隔19周期即76tick。 */
    const uint32_t first_hold=trigger+36U, third_hold=first_hold+76U;
    const uint32_t dead_ticks=34U, settle_ticks=170U;
    CHECK(trigger>maximum_ccr && trigger<arr);
    CHECK(first_hold>=maximum_ccr+dead_ticks+settle_ticks);
    CHECK(third_hold+dead_ticks<2U*arr-maximum_ccr);
}

/** @brief 仅执行内存和算法回归。@return 全部检查通过为0。 */
int
main(void)
{
    CHECK(MOTOR_PROFILE_G2208==1);
    test_current_v3_migration();
    test_current_v4_migration();
    test_profile_migration();test_profile_backend();test_profile_drive();test_profile_current();
    test_profile_margin_protection();
    test_profile_start_current();
    test_profile_sampling_window();
    printf("G2208 profile: %u checks passed; no hardware accessed\n",checks);
    return 0;
}
