/* 复用存储替身与冻结V1/V2夹具，单独打开有感参数能力，不运行旧电机测试入口。 */
#define main legacy_migration_unused_main
#include "test_settings_migration.c"
#undef main
#define current_record_t v6_record_t
#define profile_crc bytes_crc
#define flash g_test_flash
#define writes g_test_writes
#define cut_after g_test_cut_after
/** @brief 建立本板默认参数，不连接硬件。 */
static void profile_reset(void) { reset_storage();settings_init(); }
/* V5前224字节冻结为历史输入，禁止随settings_data_t增长。 */
typedef struct {
    uint32_t magic,version,length,sequence;
    struct { int32_t parameter[35]; uint32_t brightness,animation,algorithm,mode,source;
        settings_foc_t foc; settings_position_t position; } data;
    uint32_t crc,commit;
} haptic_v5_record_t;
_Static_assert(sizeof(haptic_v5_record_t)==248 && offsetof(haptic_v5_record_t,crc)==240,"V5 frozen");

/** @brief 保留原位置0.6A/P16/498和速度参数，遍历V6每个双字掉电边界并核对独立手感参数。 */
static void
test_haptic_v6_migration(void)
{
    profile_reset();haptic_v5_record_t old={.magic=0x464f4332,.version=5,.length=224,.sequence=1543,.commit=0x434f4d54};
    memcpy(&old.data,settings_get(),sizeof(old.data));
    old.data.algorithm=CONTROL_ENCODER_FOC;old.data.mode=CONTROL_POSITION;old.data.source=CONTROL_REMOTE;
    old.data.foc.current_limit_ma=600;old.data.foc.target_rpm=600;old.data.foc.run_ms=5000;
    old.data.position.kp_milli=16000;old.data.parameter[CONTROL_PARAM_ELEC_ZERO]=498;
    old.crc=profile_crc(&old,offsetof(haptic_v5_record_t,crc));
    for(unsigned cut=0;cut<=sizeof(current_record_t);cut+=8)
    {
        profile_reset();memcpy(flash[0],&old,sizeof(old));settings_init();
        CHECK(memcmp(settings_get(),&old.data,sizeof(old.data))==0);
        CHECK(settings_get()->haptic.current_limit_ma==500 && settings_get()->haptic.run_ms==60000);
        CHECK(settings_get()->haptic.config.detents==24 && settings_get()->haptic.config.detent_ma==100 && settings_get()->haptic.config.damping_ua_rpm==2000);
        cut_after=cut;settings_service(true,2000);CHECK(memcmp(flash[0],&old,sizeof(old))==0);
        settings_init();CHECK(memcmp(settings_get(),&old.data,sizeof(old.data))==0);
        CHECK(strcmp(settings_status(),cut<sizeof(current_record_t)?"PENDING":"SAVED")==0);
    }
    CHECK(control_model_select(CONTROL_ENCODER_FOC,CONTROL_HAPTIC)==CONTROL_OK);settings_publish();
    CHECK(!control_model_parameter(CONTROL_PARAM_TARGET)->valid);
    CHECK(settings_value(CONTROL_PARAM_CURRENT_LIMIT)==500 && settings_value(CONTROL_PARAM_RUN_MS)==60000);
    CHECK(settings_set(CONTROL_PARAM_CURRENT_LIMIT,800,3000));CHECK(settings_set(CONTROL_PARAM_RUN_MS,12000,3000));
    CHECK(!settings_set(CONTROL_PARAM_CURRENT_LIMIT,801,3000));
    encoder_haptic_config_t c={ENCODER_HAPTIC_DAMPED,256,800,10000};
    CHECK(settings_haptic_configure(&c,3000));unsigned before=writes;
    settings_service(false,9000);CHECK(writes==before && strcmp(settings_status(),"PENDING")==0);
    settings_capture(50,true,9000);settings_service(true,12000);CHECK(writes==before+1);
    CHECK(settings_get()->foc.current_limit_ma==600 && settings_get()->foc.run_ms==5000 && settings_get()->position.kp_milli==16000);
    settings_init();CHECK(settings_get()->haptic.current_limit_ma==800 && settings_get()->haptic.run_ms==12000);
    CHECK(memcmp(&settings_get()->haptic.config,&c,sizeof(c))==0 && settings_get()->mode==CONTROL_HAPTIC);
    CHECK(control_model_select(CONTROL_ENCODER_FOC,CONTROL_POSITION)==CONTROL_OK);
    CHECK(settings_value(CONTROL_PARAM_CURRENT_LIMIT)==600 && settings_value(CONTROL_PARAM_RUN_MS)==5000);
    /* CRC正确也不能接受越界V6字段；旧V5槽作为回退输入独立保留。 */
    current_record_t good={.magic=0x464f4332,.version=6,.length=248,.sequence=1544,.commit=0x434f4d54};
    good.data=*settings_get();
    for(unsigned defect=0;defect<7;++defect)
    {
        current_record_t bad=good;
        if(defect==0) { bad.data.haptic.config.kind=2; }
        if(defect==1) { bad.data.haptic.config.detents=1; }
        if(defect==2) { bad.data.haptic.config.detents=257; }
        if(defect==3) { bad.data.haptic.config.detent_ma=801; }
        if(defect==4) { bad.data.haptic.config.damping_ua_rpm=10001; }
        if(defect==5) { bad.data.haptic.current_limit_ma=801; }
        if(defect==6) { bad.data.haptic.run_ms=0; }
        bad.crc=profile_crc(&bad,offsetof(current_record_t,crc));
        memset(flash,255,sizeof(flash));memcpy(flash[0],&old,sizeof(old));memcpy(flash[1],&bad,sizeof(bad));settings_init();
        CHECK(memcmp(settings_get(),&old.data,sizeof(old.data))==0 && strcmp(settings_status(),"PENDING")==0);
    }
}

/** @brief 验证冻结V5和原始V6记录。 @return 全通过为0。 */
int main(void) { test_haptic_v6_migration();printf("haptic V5/V6: %u checks passed\n",g_test_checks);return 0; }
