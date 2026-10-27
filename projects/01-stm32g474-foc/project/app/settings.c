#include "settings.h"
#include "motor_config.h"
#include "board_storage.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* 改变载荷含义或布局时增加版本，禁止误解释旧记录。 */
#define SETTINGS_MAGIC 0x464F4332UL
#define SETTINGS_VERSION 6U
/* V1 与 V2 载荷布局相同，但四个无感控制参数从占位改为实际物理量。 */
#define SETTINGS_LEGACY_VERSION 1U
/* 最后一个双字中的提交标志，掉电前未写完的记录无效。 */
#define SETTINGS_COMMIT 0x434F4D54UL
typedef struct {
    /* 格式、版本、长度及递增序号。 */
    uint32_t magic, version, length, sequence;
    settings_data_t data;
    /* CRC 覆盖前面所有字段，提交标志最后写入。 */
    uint32_t crc, commit;
} settings_record_t;
_Static_assert(sizeof(settings_record_t) % 8U == 0U, "Flash record alignment");
/* 历史CRC/提交位置必须冻结，不能用增加FOC块后的结构解释旧槽。 */
typedef struct {
    int32_t parameter[35];
    uint32_t brightness,animation,algorithm,mode,source;
} settings_legacy_data_t;
typedef struct {
    uint32_t magic,version,length,sequence;
    settings_legacy_data_t data;
    uint32_t crc,commit;
} settings_legacy_record_t;
/* V3/V4的200字节载荷及CRC位置冻结；V5只在尾部追加位置参数。 */
typedef struct {
    settings_legacy_data_t legacy;
    settings_foc_t foc;
} settings_v4_data_t;
typedef struct {
    uint32_t magic,version,length,sequence;
    settings_v4_data_t data;
    uint32_t crc,commit;
} settings_v4_record_t;
/* V5的224字节载荷和CRC位置冻结，V6只在其尾部追加24字节。 */
typedef struct {
    settings_v4_data_t base;
    settings_position_t position;
} settings_v5_data_t;
typedef struct {
    uint32_t magic,version,length,sequence;
    settings_v5_data_t data;
    uint32_t crc,commit;
} settings_v5_record_t;
typedef union { settings_record_t current; settings_legacy_record_t legacy; settings_v4_record_t v4; settings_v5_record_t v5; } settings_slot_t;
_Static_assert(CONTROL_PARAM_COUNT==35U && sizeof(settings_legacy_data_t)==160U, "Historical payload");
_Static_assert(sizeof(settings_legacy_record_t)==184U && offsetof(settings_data_t,foc)==160U, "Historical CRC layout");
_Static_assert(sizeof(settings_foc_t)==40U && sizeof(settings_v4_record_t)==224U && sizeof(settings_v5_record_t)==248U && sizeof(settings_record_t)==272U, "V3/V4/V5 layout");
typedef struct {
    /* 默认值、范围、步长和小数位；名称沿用模型接口。 */
    int32_t initial, min, max;
    uint32_t step;
    uint8_t decimals;
} parameter_spec_t;
/* 参数定义集中在此；零极对数/额定电流表示未知，启动要求填写极对数。 */
static const parameter_spec_t g_settings_spec[CONTROL_PARAM_COUNT] = {
    [CONTROL_PARAM_TARGET] = {MOTOR_TARGET_DEFAULT_RPM, 0, MOTOR_SPEED_MAX_RPM, 10, 0},
#if MOTOR_PROFILE_G2208
    [CONTROL_PARAM_POLES] = {MOTOR_POLE_PAIRS, MOTOR_POLE_PAIRS, MOTOR_POLE_PAIRS, 0, 0},
#else
    [CONTROL_PARAM_POLES] = {MOTOR_POLES_DEFAULT, 0, 32, 1, 0},
#endif
    [CONTROL_PARAM_CURRENT_LIMIT] = {MOTOR_LIMIT_DEFAULT_MA, 100, MOTOR_LIMIT_MAX_MA, 50, 3},
    [CONTROL_PARAM_SPEED_LIMIT] = {MOTOR_SPEED_LIMIT_DEFAULT_RPM, 1, MOTOR_SPEED_MAX_RPM, 10, 0},
    [CONTROL_PARAM_POSITION_MIN] = {-1800, -36000, 36000, 10, 1},
    [CONTROL_PARAM_POSITION_MAX] = {1800, -36000, 36000, 10, 1},
    [CONTROL_PARAM_D_KP] = {0, 0, 100000, 1, 3},
    [CONTROL_PARAM_D_KI] = {0, 0, 100000, 1, 3},
    [CONTROL_PARAM_Q_KP] = {0, 0, 100000, 1, 3},
    [CONTROL_PARAM_Q_KI] = {0, 0, 100000, 1, 3},
    /* D表示占空千分比；实际增益raw/1000，无感后端将0解释为固件默认增益，不改记录。 */
    [CONTROL_PARAM_SPEED_KP] = {0, 0, 100000, 1, 3},
    [CONTROL_PARAM_SPEED_KI] = {0, 0, 100000, 1, 3},
    [CONTROL_PARAM_POSITION_KP] = {0, 0, 100000, 1, 3},
    [CONTROL_PARAM_POSITION_KI] = {0, 0, 100000, 1, 3},
    [CONTROL_PARAM_POSITION_KD] = {0, 0, 100000, 1, 3},
    /* 允许用50 Hz初频隔离低频牵引与加速；普通启动仍检查目标不低于初频。 */
    [CONTROL_PARAM_START_HZ] = {MOTOR_START_DEFAULT_MHZ, 100, 50000, 100, 3},
    [CONTROL_PARAM_RAMP] = {MOTOR_RAMP_DEFAULT_MHZ_S, 100, 100000, 100, 3},
    [CONTROL_PARAM_DUTY] = {MOTOR_DUTY_DEFAULT, 1, MOTOR_ENCODER_DUTY_MAX, 5, 1},
    /* 接管机械转速使用 RPM；换相补偿使用 0.1 电角度/LSB。 */
    [CONTROL_PARAM_HANDOFF] = {0, 0, 5400, 10, 0},
    [CONTROL_PARAM_COMM_TIMING] = {0, -300, 300, 1, 1},
    [CONTROL_PARAM_OFFSET_A] = {0, 0, 4095, 0, 0},
    [CONTROL_PARAM_OFFSET_B] = {0, 0, 4095, 0, 0},
    [CONTROL_PARAM_OFFSET_C] = {0, 0, 4095, 0, 0},
    [CONTROL_PARAM_ENCODER_DIR] = {1, -1, 1, 2, 0},
    [CONTROL_PARAM_ELEC_ZERO] = {0, 0, 3599, 1, 1},
    [CONTROL_PARAM_RESISTANCE] = {MOTOR_PROFILE_G2208?MOTOR_FOC_RESISTANCE_MOHM:0, 0, 100000, 1, 3},
    [CONTROL_PARAM_LD] = {MOTOR_PROFILE_G2208?MOTOR_FOC_INDUCTANCE_UH:0, 0, 100000, 1, 3},
    [CONTROL_PARAM_LQ] = {MOTOR_PROFILE_G2208?MOTOR_FOC_INDUCTANCE_UH:0, 0, 100000, 1, 3},
    [CONTROL_PARAM_FLUX] = {0, 0, 100000, 1, 3},
    [CONTROL_PARAM_SHUNT] = {MOTOR_SHUNT_DEFAULT_MOHM, 1, 50, 49, 0},
    [CONTROL_PARAM_RATED_VOLTAGE] = {12000, 8000, 15000, 100, 3},
    [CONTROL_PARAM_RATED_CURRENT] = {MOTOR_RATED_CURRENT_MA, 0, 10000, 50, 3},
    [CONTROL_PARAM_RATED_SPEED] = {MOTOR_RATED_SPEED_RPM, 1, 20000, 100, 0},
    [CONTROL_PARAM_ALIGN_MS] = {MOTOR_ALIGN_DEFAULT_MS, 10, 1000, 10, 0},
    [CONTROL_PARAM_RUN_MS] = {MOTOR_RUN_DEFAULT_MS, 100, 30000, 100, 0}
};
typedef struct {
    control_parameter_id_t id;
    size_t offset;
    parameter_spec_t spec;
} foc_parameter_spec_t;
#define FOC_SPEC(id,field,value,min,max,step,dec) {id,offsetof(settings_data_t,foc)+offsetof(settings_foc_t,field),{value,min,max,step,dec}}
/* 只暴露已有参数接口；本档图纸模型与带宽只读，物理值仍需实测验收。 */
static const foc_parameter_spec_t g_foc_spec[]={
    FOC_SPEC(CONTROL_PARAM_TARGET,target_rpm,MOTOR_FOC_TARGET_DEFAULT_RPM,1,MOTOR_SPEED_MAX_RPM,10,0),
    FOC_SPEC(CONTROL_PARAM_CURRENT_LIMIT,current_limit_ma,500,500,MOTOR_FOC_LIMIT_MAX_MA,50,3),
    FOC_SPEC(CONTROL_PARAM_RESISTANCE,resistance_mohm,MOTOR_FOC_RESISTANCE_MOHM,MOTOR_FOC_RESISTANCE_MOHM,MOTOR_FOC_RESISTANCE_MOHM,0,3),
    FOC_SPEC(CONTROL_PARAM_LD,inductance_uh,MOTOR_FOC_INDUCTANCE_UH,MOTOR_FOC_INDUCTANCE_UH,MOTOR_FOC_INDUCTANCE_UH,0,3),
    FOC_SPEC(CONTROL_PARAM_LQ,inductance_uh,MOTOR_FOC_INDUCTANCE_UH,MOTOR_FOC_INDUCTANCE_UH,MOTOR_FOC_INDUCTANCE_UH,0,3),
    FOC_SPEC(CONTROL_PARAM_SPEED_KP,speed_kp_micro,MOTOR_FOC_SPEED_KP_DEFAULT,1,2000,10,3),
    FOC_SPEC(CONTROL_PARAM_SPEED_KI,speed_ki_micro,500,1,5000,10,3),
    FOC_SPEC(CONTROL_PARAM_RAMP,speed_ramp_millirpm_s,200000,10000,500000,1000,3),
    /* 正常FOC不限制固定秒数；仅沿用参数int32毫秒的自然表示范围和到时停机。 */
    FOC_SPEC(CONTROL_PARAM_RUN_MS,run_ms,MOTOR_FOC_RUN_DEFAULT_MS,1000,INT32_MAX,100,0)
};
#undef FOC_SPEC
/* CURRENT只开放有符号Iq；固定0.5A及2A/s由模式定义，不修改原速度参数。 */
static const foc_parameter_spec_t g_iq_target_spec={CONTROL_PARAM_TARGET,offsetof(settings_data_t,foc)+offsetof(settings_foc_t,target_iq_ma),{0,-500,500,10,3}};
static const foc_parameter_spec_t g_iq_limit_spec={CONTROL_PARAM_CURRENT_LIMIT,offsetof(settings_data_t,foc)+offsetof(settings_foc_t,current_limit_ma),{500,500,500,0,3}};
/* 位置增益采用本板已验证低速支路的有效尺度，不覆盖SPEED的100/200。 */
#define POS_SPEC(id,field,value,min,max,step,dec) {id,offsetof(settings_data_t,position)+offsetof(settings_position_t,field),{value,min,max,step,dec}}
static const foc_parameter_spec_t g_position_spec[]={
    POS_SPEC(CONTROL_PARAM_POSITION_KP,kp_milli,8000,1,100000,100,3),
    POS_SPEC(CONTROL_PARAM_SPEED_LIMIT,speed_limit_rpm,60,1,MOTOR_SPEED_MAX_RPM,1,0),
    POS_SPEC(CONTROL_PARAM_RAMP,acceleration_millirpm_s,300000,10000,500000,1000,3),
    POS_SPEC(CONTROL_PARAM_SPEED_KP,speed_kp_micro,1333,1,100000,1,3),
    POS_SPEC(CONTROL_PARAM_SPEED_KI,speed_ki_micro,266667,1,1000000,1,3)
};
#undef POS_SPEC
static const foc_parameter_spec_t g_position_target_spec={CONTROL_PARAM_TARGET,0,{0,-INT32_MAX,INT32_MAX,1,3}};
static int32_t g_position_target_mdeg;
/* 手感沿用两个通用参数名，但持久化偏移与其它模式独立。 */
static const foc_parameter_spec_t g_haptic_spec[]={
    {CONTROL_PARAM_CURRENT_LIMIT,offsetof(settings_data_t,haptic)+offsetof(settings_haptic_t,current_limit_ma),{500,500,800,50,3}},
    {CONTROL_PARAM_RUN_MS,offsetof(settings_data_t,haptic)+offsetof(settings_haptic_t,run_ms),{60000,1000,INT32_MAX,100,0}}
};
/* 仅主循环持有的已确认配置和保存状态。 */
static settings_data_t g_settings_data;
static uint32_t g_settings_sequence, g_settings_changed;
static unsigned g_settings_slot;
static const char *g_settings_status;
static bool g_settings_dirty;

/** @brief 计算记录 CRC32，避免依赖芯片 CRC 外设。 @param data 字节数据。 @param size 字节数。 @return CRC32。 */
static uint32_t
record_crc(const void *data, size_t size)
{
    const uint8_t *bytes = data;
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < size; ++i)
    {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
        {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
        }
    }
    return ~crc;
}

/** @brief 区分有感与原六步幅度范围，保存布局不变。 @param algorithm 算法枚举。 @return 千分比上限。 */
static int32_t
duty_max(uint32_t algorithm)
{
    return algorithm == CONTROL_ENCODER_6STEP ? MOTOR_ENCODER_DUTY_MAX : MOTOR_DUTY_MAX;
}

/** @brief 检查枚举和参数域，CRC 正确也不接受越界数据。 @param d 数据。 @return 合法。 */
static bool
data_valid(const settings_data_t *d)
{
    if (d->brightness < 1 || d->brightness > 100 || d->animation > 1 ||
        d->algorithm >= CONTROL_ALGORITHM_COUNT || d->mode >= CONTROL_MODE_COUNT || d->source > CONTROL_REMOTE ||
        (!control_algorithm_foc((control_algorithm_t)d->algorithm) && d->mode != CONTROL_SPEED) ||
        ((d->mode==CONTROL_CURRENT || d->mode==CONTROL_HAPTIC) && d->algorithm!=CONTROL_ENCODER_FOC)) { return false; }
    for (unsigned i = 0; i < CONTROL_PARAM_COUNT; ++i)
    {
        int32_t minimum=g_settings_spec[i].min,maximum=g_settings_spec[i].max;
        if(i==CONTROL_PARAM_DUTY) { maximum=duty_max(d->algorithm); }
#if MOTOR_PROFILE_G2208
        /* 先按已交付范围验证原记录，择出真正最新槽后再迁移，不能因新上限较小丢失序号。 */
        if(i==CONTROL_PARAM_TARGET || i==CONTROL_PARAM_SPEED_LIMIT) { maximum=5400; }
        if(i==CONTROL_PARAM_CURRENT_LIMIT) { maximum=1500; }
        if(i==CONTROL_PARAM_POLES) { minimum=0; maximum=32; }
#endif
        if (d->parameter[i] < minimum || d->parameter[i] > maximum) { return false; }
    }
    return (d->parameter[CONTROL_PARAM_SHUNT] == 1 || d->parameter[CONTROL_PARAM_SHUNT] == 50) &&
        (d->parameter[CONTROL_PARAM_ENCODER_DIR] == 1 || d->parameter[CONTROL_PARAM_ENCODER_DIR] == -1);
}

/** @brief 完整校验 V1/V2 独立记录，先校验原载荷再做迁移。 @param r 记录。 @return 可装载。 */
static bool MOTOR_BACKGROUND_CODE
record_valid(const settings_slot_t *slot)
{
    const settings_record_t *r=&slot->current;
    if(r->magic!=SETTINGS_MAGIC) { return false; }
    if(r->version==1U || r->version==2U)
    {
        const settings_legacy_record_t *old=&slot->legacy;
        if(old->length!=sizeof(old->data) || old->commit!=SETTINGS_COMMIT ||
            old->crc!=record_crc(old,offsetof(settings_legacy_record_t,crc)) || old->data.mode>CONTROL_POSITION) { return false; }
        settings_data_t migrated={0};
        memcpy(&migrated,&old->data,sizeof(old->data));
        return data_valid(&migrated);
    }
    settings_data_t normalized={0};
    const settings_data_t *d=&r->data;
    if(r->version==3U || r->version==4U)
    {
        const settings_v4_record_t *old=&slot->v4;
        if(old->length!=sizeof(old->data) || old->commit!=SETTINGS_COMMIT ||
            old->crc!=record_crc(old,offsetof(settings_v4_record_t,crc)) || old->data.legacy.mode>CONTROL_CURRENT) { return false; }
        memcpy(&normalized,&old->data,sizeof(old->data));
        d=&normalized;
    }
    else if(r->version==5U)
    {
        const settings_v5_record_t *old=&slot->v5;
        if(old->length!=sizeof(old->data) || old->commit!=SETTINGS_COMMIT ||
            old->crc!=record_crc(old,offsetof(settings_v5_record_t,crc)) || old->data.base.legacy.mode>CONTROL_CURRENT) { return false; }
        memcpy(&normalized,&old->data,sizeof(old->data));
        d=&normalized;
    }
    else if(r->version!=SETTINGS_VERSION || r->length!=sizeof(r->data) || r->commit!=SETTINGS_COMMIT ||
        r->crc!=record_crc(r,offsetof(settings_record_t,crc))) { return false; }
    if(!data_valid(d) || (r->version==3U && (d->mode>CONTROL_POSITION || d->foc.target_iq_ma!=0))) { return false; }
    if(r->version==SETTINGS_VERSION && (!encoder_haptic_valid(&d->haptic.config) ||
        d->haptic.current_limit_ma<500 || d->haptic.current_limit_ma>800 || d->haptic.run_ms<1000)) { return false; }
    if(r->version>=5U)
    {
        for(unsigned i=0;i<sizeof(g_position_spec)/sizeof(g_position_spec[0]);++i)
        {
            const foc_parameter_spec_t *p=&g_position_spec[i];
            int32_t value=*(const int32_t *)((const uint8_t *)d+p->offset);
            if(value<p->spec.min || value>p->spec.max) { return false; }
        }
        if(d->position.tolerance_mdeg!=100 && d->position.tolerance_mdeg!=500) { return false; }
    }
#if MOTOR_PROFILE_G2208
    /* 仅接受本档模型或冻结的原电机模型；不把任意R/L当成可迁移的旧电机。 */
    bool original_model=d->foc.resistance_mohm==790 && d->foc.inductance_uh==240;
    bool current_model=d->foc.resistance_mohm==MOTOR_FOC_RESISTANCE_MOHM &&
        d->foc.inductance_uh==MOTOR_FOC_INDUCTANCE_UH;
    if(!original_model && !current_model) { return false; }
    if(current_model)
    {
        /* 已经是新档的记录按新范围检查，不能借历史兼容路径保存超额定指令。 */
        for(unsigned i=0;i<CONTROL_PARAM_COUNT;++i)
        {
            if(d->parameter[i]<g_settings_spec[i].min || d->parameter[i]>g_settings_spec[i].max) { return false; }
        }
    }
#endif
    for(unsigned i=0;i<sizeof(g_foc_spec)/sizeof(g_foc_spec[0]);++i)
    {
        const foc_parameter_spec_t *p=&g_foc_spec[i];
        int32_t value=*(const int32_t *)((const uint8_t *)d+p->offset);
        int32_t maximum=p->spec.max;
#if MOTOR_PROFILE_G2208
        if(p->id==CONTROL_PARAM_RESISTANCE || p->id==CONTROL_PARAM_LD || p->id==CONTROL_PARAM_LQ) { continue; }
        if(original_model && p->id==CONTROL_PARAM_TARGET) { maximum=5400; }
        if(original_model && p->id==CONTROL_PARAM_CURRENT_LIMIT) { maximum=2000; }
#endif
        if(value<p->spec.min || value>maximum) { return false; }
    }
    return d->foc.bandwidth_hz==MOTOR_FOC_BANDWIDTH_HZ &&
        d->foc.target_iq_ma>=-500 && d->foc.target_iq_ma<=500;
}

/** @brief 重置与电机相关的35项及独立FOC块，保留来源、算法和旧显示元数据。 */
static void
motor_defaults(void)
{
    for (unsigned i=0;i<CONTROL_PARAM_COUNT;++i) { g_settings_data.parameter[i]=g_settings_spec[i].initial; }
    g_settings_data.foc=(settings_foc_t){MOTOR_FOC_TARGET_DEFAULT_RPM,500,MOTOR_FOC_RESISTANCE_MOHM,
        MOTOR_FOC_INDUCTANCE_UH,MOTOR_FOC_BANDWIDTH_HZ,MOTOR_FOC_SPEED_KP_DEFAULT,500,200000,MOTOR_FOC_RUN_DEFAULT_MS,0};
    g_settings_data.position=(settings_position_t){8000,60,300000,1333,266667,100};
    g_settings_data.haptic=(settings_haptic_t){{ENCODER_HAPTIC_DETENT,24,100,2000},500,60000};
}

/** @brief 更新脏标记，自动保存延迟从最后一次确认开始。 @param now 当前 ms。 */
static void
changed(uint32_t now)
{
    g_settings_dirty = true;
    g_settings_changed = now;
    g_settings_status = "PENDING";
}

/** @brief 两槽混合版本按序择新；V1 只清除旧占位参数，并排队停机安全保存。 */
void MOTOR_BACKGROUND_CODE
settings_init(void)
{
    settings_slot_t record[2];
    bool valid[2];
    memset(&g_settings_data, 0, sizeof(g_settings_data));
    g_settings_data.brightness = 50;
    g_settings_data.animation = 1;
    g_settings_data.algorithm = CONTROL_OPEN_6STEP;
    motor_defaults();
    g_position_target_mdeg=0;
    for (unsigned i = 0; i < 2; ++i)
    {
        valid[i] = board_storage_read(i, &record[i], sizeof(record[i])) && record_valid(&record[i]);
    }
    g_settings_slot = valid[1] && (!valid[0] || (int32_t)(record[1].current.sequence - record[0].current.sequence) > 0) ? 1U : 0U;
    g_settings_sequence = 0;
    g_settings_changed = 0;
    g_settings_dirty = false;
    g_settings_status = board_storage_layout_valid() ? "DEFAULT" : "ERROR";
    if (valid[g_settings_slot])
    {
        const settings_record_t *selected=&record[g_settings_slot].current;
        memcpy(&g_settings_data,&selected->data,selected->length);
        g_settings_sequence = selected->sequence;
        g_settings_status = "SAVED";
        if (selected->version == SETTINGS_LEGACY_VERSION)
        {
            /* 不能把旧占位值误解释为接管转速、换相补偿或有效 PI 增益。 */
            g_settings_data.parameter[CONTROL_PARAM_HANDOFF] = 0;
            g_settings_data.parameter[CONTROL_PARAM_COMM_TIMING] = 0;
            g_settings_data.parameter[CONTROL_PARAM_SPEED_KP] = 0;
            g_settings_data.parameter[CONTROL_PARAM_SPEED_KI] = 0;
            changed(0);
        }
        if(selected->version!=SETTINGS_VERSION) { changed(0); }
#if MOTOR_PROFILE_G2208
        /* 新电机不能沿用旧额定值、定位、速度或编码器零点；只排队停机另槽保存，不在此擦写。 */
        if(selected->version<3U || g_settings_data.parameter[CONTROL_PARAM_POLES]!=MOTOR_POLE_PAIRS ||
            g_settings_data.foc.resistance_mohm!=MOTOR_FOC_RESISTANCE_MOHM ||
            g_settings_data.foc.inductance_uh!=MOTOR_FOC_INDUCTANCE_UH)
        {
            motor_defaults();
            changed(0);
        }
#endif
    }
}

/** @brief 返回已确认配置，编辑草稿由 UI 独立维护。 */
const settings_data_t *
settings_get(void) { return &g_settings_data; }

/** @brief 读取当前FOC独立参数描述；算法切换后即时使用当前模型选择。
 * @param id 参数枚举，位置电流复用FOC上限，CURRENT电流保持只读。
 * @return 当前模式描述；未属于FOC参数时返回NULL。 */
static const foc_parameter_spec_t *MOTOR_BACKGROUND_CODE
foc_spec(control_parameter_id_t id)
{
    if(control_model_algorithm()!=CONTROL_SENSORLESS_FOC &&
        !(MOTOR_ENCODER_ENABLED && control_model_algorithm()==CONTROL_ENCODER_FOC)) { return NULL; }
    if(control_model_algorithm()==CONTROL_ENCODER_FOC && control_model_mode()==CONTROL_HAPTIC)
    {
        for(unsigned i=0;i<sizeof(g_haptic_spec)/sizeof(g_haptic_spec[0]);++i)
        { if(g_haptic_spec[i].id==id) { return &g_haptic_spec[i]; } }
    }
    if(control_model_algorithm()==CONTROL_ENCODER_FOC && control_model_mode()==CONTROL_POSITION)
    {
        if(id==CONTROL_PARAM_TARGET) { return &g_position_target_spec; }
        for(unsigned i=0;i<sizeof(g_position_spec)/sizeof(g_position_spec[0]);++i)
        { if(g_position_spec[i].id==id) { return &g_position_spec[i]; } }
    }
    if(control_model_algorithm()==CONTROL_ENCODER_FOC && control_model_mode()==CONTROL_CURRENT)
    {
        if(id==CONTROL_PARAM_TARGET) { return &g_iq_target_spec; }
        if(id==CONTROL_PARAM_CURRENT_LIMIT) { return &g_iq_limit_spec; }
    }
    for(unsigned i=0;i<sizeof(g_foc_spec)/sizeof(g_foc_spec[0]);++i)
    { if(g_foc_spec[i].id==id) { return &g_foc_spec[i]; } }
    return NULL;
}

/** @brief 主循环读取独立参数，不修改原六步载荷。 */
int32_t
settings_value(control_parameter_id_t id)
{
    if((unsigned)id>=CONTROL_PARAM_COUNT) { return 0; }
    const foc_parameter_spec_t *spec=foc_spec(id);
    if(spec==&g_position_target_spec) { return g_position_target_mdeg; }
    if(spec==&g_iq_limit_spec) { return 500; }
    return spec?*(int32_t *)((uint8_t *)&g_settings_data+spec->offset):g_settings_data.parameter[id];
}

/** @brief 将统一参数表转换为现有 UI 描述；自动零点不可手动覆盖。 */
void MOTOR_BACKGROUND_CODE
settings_publish(void)
{
    for (unsigned i = 0; i < CONTROL_PARAM_COUNT; ++i)
    {
        control_parameter_t p = *control_model_parameter((control_parameter_id_t)i);
        const foc_parameter_spec_t *foc=foc_spec((control_parameter_id_t)i);
        const parameter_spec_t *spec = foc?&foc->spec:&g_settings_spec[i];
        p.value = settings_value((control_parameter_id_t)i);
        p.minimum = spec->min;
        p.maximum = spec->max;
        if(i==CONTROL_PARAM_DUTY) { p.maximum=duty_max(control_model_algorithm()); }
        p.step = spec->step;
        p.decimals = spec->decimals;
        p.valid = spec->step != 0;
        p.editable = p.valid;
#if MOTOR_PROFILE_G2208
        if(i==CONTROL_PARAM_POLES) { p.valid=true; p.editable=false; }
#endif
        if (i == CONTROL_PARAM_TARGET && control_model_mode() != CONTROL_SPEED)
        {
            /* 未实现模式不把RPM冒充转矩或位置；独立Iq在FOC分支重新发布。 */
            p.valid = p.editable = false;
        }
        if (i == CONTROL_PARAM_RAMP) { snprintf(p.unit, sizeof(p.unit), "HZ/S"); }
        if (control_model_algorithm()==CONTROL_ENCODER_6STEP &&
            (i==CONTROL_PARAM_START_HZ || i==CONTROL_PARAM_HANDOFF || i==CONTROL_PARAM_COMM_TIMING || i==CONTROL_PARAM_ALIGN_MS))
        {
            /* 有感换相直接使用编码器角度，旧无感/强拖参数不提供可编辑入口。 */
            p.valid=p.editable=false;
        }
        if(i==CONTROL_PARAM_SPEED_KP) { snprintf(p.unit,sizeof(p.unit),"D/RPM"); }
        if(i==CONTROL_PARAM_SPEED_KI) { snprintf(p.unit,sizeof(p.unit),"D/RPMS"); }
        if(i>=CONTROL_PARAM_D_KP&&i<=CONTROL_PARAM_Q_KI) { p.unit[0]='\0'; }
        if(control_model_algorithm()==CONTROL_SENSORLESS_FOC ||
            (MOTOR_ENCODER_ENABLED && control_model_algorithm()==CONTROL_ENCODER_FOC))
        {
            if(foc) { p.valid=true; p.editable=spec->step!=0U; }
            if(i==CONTROL_PARAM_RAMP) { snprintf(p.unit,sizeof(p.unit),"RPM/S"); }
            if(i==CONTROL_PARAM_SPEED_KP) { snprintf(p.unit,sizeof(p.unit),"MA/RPM"); }
            if(i==CONTROL_PARAM_SPEED_KI) { snprintf(p.unit,sizeof(p.unit),"MA/R/S"); }
            /* 六步占空/换相/编码器项不参与FOC，不能提供可以编辑却不起作用的参数。 */
            if(i==CONTROL_PARAM_DUTY || i==CONTROL_PARAM_HANDOFF || i==CONTROL_PARAM_COMM_TIMING ||
                i==CONTROL_PARAM_ENCODER_DIR || i==CONTROL_PARAM_ELEC_ZERO || i==CONTROL_PARAM_FLUX ||
                (i>=CONTROL_PARAM_POSITION_MIN&&i<=CONTROL_PARAM_POSITION_MAX) ||
                (i>=CONTROL_PARAM_POSITION_KP&&i<=CONTROL_PARAM_POSITION_KD)) { p.valid=p.editable=false; }
            if(MOTOR_ENCODER_ENABLED && control_model_algorithm()==CONTROL_ENCODER_FOC &&
                (i==CONTROL_PARAM_ENCODER_DIR || i==CONTROL_PARAM_ELEC_ZERO))
            { p.valid=true; p.editable=false; }
            if(i==CONTROL_PARAM_START_HZ || i==CONTROL_PARAM_ALIGN_MS || i==CONTROL_PARAM_SPEED_LIMIT)
            {
                p.value=i==CONTROL_PARAM_START_HZ?50000:i==CONTROL_PARAM_ALIGN_MS?200:MOTOR_SPEED_MAX_RPM;
                p.minimum=p.maximum=p.value; p.valid=true; p.editable=false;
            }
            if(i>=CONTROL_PARAM_D_KP&&i<=CONTROL_PARAM_Q_KI)
            {
                bool proportional=i==CONTROL_PARAM_D_KP||i==CONTROL_PARAM_Q_KP;
                p.value=(int32_t)((proportional?g_settings_data.foc.inductance_uh*.001f:
                    (float)g_settings_data.foc.resistance_mohm)*6.28318530718f*g_settings_data.foc.bandwidth_hz);
                p.minimum=p.maximum=p.value; p.valid=true; p.editable=false;
                snprintf(p.unit,sizeof(p.unit),"%s",proportional?"V/A":"V/AS");
            }
        }
        if(i==CONTROL_PARAM_TARGET)
        {
            snprintf(p.unit,sizeof(p.unit),"RPM");
            if(foc==&g_iq_target_spec) { snprintf(p.unit,sizeof(p.unit),"A"); }
            else if(foc==&g_position_target_spec) { snprintf(p.unit,sizeof(p.unit),"DEG"); p.valid=p.editable=true; }
            else if(control_model_mode()!=CONTROL_SPEED) { p.valid=p.editable=false; }
        }
        if(control_model_mode()==CONTROL_CURRENT &&
            (i==CONTROL_PARAM_SPEED_KP || i==CONTROL_PARAM_SPEED_KI || i==CONTROL_PARAM_RAMP ||
             i==CONTROL_PARAM_START_HZ || i==CONTROL_PARAM_ALIGN_MS)) { p.valid=p.editable=false; }
        if(control_model_algorithm()==CONTROL_ENCODER_FOC && control_model_mode()==CONTROL_POSITION)
        {
            if(i==CONTROL_PARAM_POSITION_KP || i==CONTROL_PARAM_SPEED_LIMIT)
            {
                p.value=settings_value((control_parameter_id_t)i); p.minimum=spec->min; p.maximum=spec->max;
                p.valid=true; p.editable=spec->step!=0U;
            }
            if(i==CONTROL_PARAM_POSITION_KP) { snprintf(p.unit,sizeof(p.unit),"1/S"); }
            if(i==CONTROL_PARAM_POSITION_KI || i==CONTROL_PARAM_POSITION_KD)
            { p.value=p.minimum=p.maximum=0; p.valid=true; p.editable=false; }
            if(i==CONTROL_PARAM_START_HZ || i==CONTROL_PARAM_ALIGN_MS) { p.valid=p.editable=false; }
        }
        if(control_model_mode()==CONTROL_HAPTIC &&
            (i==CONTROL_PARAM_TARGET || i==CONTROL_PARAM_SPEED_KP || i==CONTROL_PARAM_SPEED_KI ||
             i==CONTROL_PARAM_RAMP || i==CONTROL_PARAM_START_HZ || i==CONTROL_PARAM_ALIGN_MS)) { p.valid=p.editable=false; }
        (void)control_model_parameter_publish(&p);
    }
}

/** @brief 验证确认值并修改 RAM，Flash 写入由停机服务统一完成。 */
bool
settings_set(control_parameter_id_t id, int32_t value, uint32_t now)
{
    const foc_parameter_spec_t *foc=foc_spec(id);
    if(foc==&g_position_target_spec) { g_position_target_mdeg=value; return true; }
    if(foc)
    {
        if(foc->spec.step==0U || value<foc->spec.min || value>foc->spec.max) { return false; }
        int32_t *field=(int32_t *)((uint8_t *)&g_settings_data+foc->offset);
        if(*field!=value) { *field=value; changed(now); }
        return true;
    }
    if ((unsigned)id >= CONTROL_PARAM_COUNT || g_settings_spec[id].step == 0 ||
        value < g_settings_spec[id].min || value > g_settings_spec[id].max ||
        (id == CONTROL_PARAM_DUTY && value > duty_max(control_model_algorithm())) ||
        (id == CONTROL_PARAM_SHUNT && value != 1 && value != 50) ||
        (id == CONTROL_PARAM_ENCODER_DIR && value != -1 && value != 1)) { return false; }
    if (g_settings_data.parameter[id] != value)
    {
        g_settings_data.parameter[id] = value;
        changed(now);
    }
    return true;
}

/** @brief 捕获已提交显示和算法选择，取消编辑不会改变绑定值。 */
void
settings_capture(uint32_t brightness, bool animation, uint32_t now)
{
    uint32_t algorithm = (uint32_t)control_model_algorithm();
    uint32_t mode = (uint32_t)control_model_mode();
    if (g_settings_data.brightness != brightness || g_settings_data.animation != (uint32_t)animation ||
        g_settings_data.algorithm != algorithm || g_settings_data.mode != mode)
    {
        g_settings_data.brightness = brightness;
        g_settings_data.animation = animation;
        g_settings_data.algorithm = algorithm;
        g_settings_data.mode = mode;
        /* 共用六步载荷切回旧后端时限幅，不能把95%误带入无感或开环。 */
        if(g_settings_data.parameter[CONTROL_PARAM_DUTY] > duty_max(algorithm))
        {
            g_settings_data.parameter[CONTROL_PARAM_DUTY] = duty_max(algorithm);
        }
        changed(now);
    }
}

/** @brief 控制来源也随配置保存，但恢复来源不会自动启动。 */
void
settings_source(control_source_t source, uint32_t now)
{
    if (g_settings_data.source != (uint32_t)source)
    {
        g_settings_data.source = (uint32_t)source;
        changed(now);
    }
}

/** @brief 写入另一槽并回读后才更新序号；失败保留旧有效槽且不反复擦写。 */
void
settings_service(bool safe, uint32_t now)
{
    if (!safe || !g_settings_dirty || now - g_settings_changed < MOTOR_SAVE_DELAY_MS) { return; }
    settings_record_t r = {0};
    r.magic = SETTINGS_MAGIC;
    r.version = SETTINGS_VERSION;
    r.length = sizeof(r.data);
    r.sequence = g_settings_sequence + 1U;
    r.data = g_settings_data;
    r.crc = record_crc(&r, offsetof(settings_record_t, crc));
    r.commit = SETTINGS_COMMIT;
    unsigned slot = 1U - g_settings_slot;
    g_settings_dirty = false;
    if (board_storage_write(slot, &r, sizeof(r)))
    {
        g_settings_slot = slot;
        g_settings_sequence = r.sequence;
        g_settings_status = "SAVED";
    }
    else { g_settings_status = "ERROR"; }
}

/** @brief 提供界面可见保存状态，ERROR 后下次确认修改可重试。 */
const char *
settings_status(void) { return g_settings_status; }

/** @brief 见settings.h。 */
void
settings_position_reset_target(void) { g_position_target_mdeg=0; }

/** @brief 见settings.h。 */
bool
settings_position_tolerance(int32_t mdeg, uint32_t now)
{
    if(mdeg!=100 && mdeg!=500) { return false; }
    if(g_settings_data.position.tolerance_mdeg!=mdeg)
    { g_settings_data.position.tolerance_mdeg=mdeg; changed(now); }
    return true;
}

/** @brief 见settings.h；配置排队与其它参数共用停机安全保存。 */
bool MOTOR_BACKGROUND_CODE
settings_haptic_configure(const encoder_haptic_config_t *config,uint32_t now)
{
    if(!encoder_haptic_valid(config)) { return false; }
    if(memcmp(config,&g_settings_data.haptic.config,sizeof(*config))!=0)
    { g_settings_data.haptic.config=*config; changed(now); }
    return true;
}
