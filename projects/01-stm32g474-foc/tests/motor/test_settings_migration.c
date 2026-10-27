#include "settings.h"
#include "motor_config.h"
#include "board_storage.h"
#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* 固定历史布局，避免测试随着当前 settings_data_t 一起变化而掩盖兼容性破坏。 */
typedef struct {
    /* V1 的 35 个参数，顺序对应已交付的持久化参数编号。 */
    int32_t parameter[35];
    /* V1 显示、算法、模式及来源字段。 */
    uint32_t brightness, animation, algorithm, mode, source;
} legacy_data_t;
typedef struct {
    /* V1/V2 公共记录头与冻结的历史载荷。 */
    uint32_t magic, version, length, sequence;
    legacy_data_t data;
    /* 末尾 CRC 与提交标志共同检查半写记录。 */
    uint32_t crc, commit;
} fixture_record_t;
_Static_assert(sizeof(fixture_record_t) == 184U, "historical settings record size");
_Static_assert(sizeof(legacy_data_t) == offsetof(settings_data_t,foc), "V1/V2 payload prefix compatibility");
/* 新载荷长度和CRC位置另行冻结，不能把旧fixture扩大后冒充兼容性测试。 */
typedef struct {
    uint32_t magic,version,length,sequence;
    settings_data_t data;
    uint32_t crc,commit;
} v6_record_t;
_Static_assert(sizeof(v6_record_t)==272U && offsetof(v6_record_t,crc)==264U,"V6 layout");

/* 本测试仅访问内存中的两槽 Flash，不连接板卡。 */
static unsigned char g_test_flash[2][BOARD_STORAGE_SLOT_SIZE];
/* 写入次数、断电字节边界与断言数。 */
static unsigned g_test_writes, g_test_cut_after, g_test_checks;
/* 模拟存储布局无效时必须拒绝读写。 */
static bool g_test_layout;
/* 保存最近一次写入槽，核对迁移不会擦除最新旧槽。 */
static unsigned g_test_last_slot;
/* 断言宏包含次数统计，失败由 assert 直接给出源码位置。 */
#define CHECK(condition) do { ++g_test_checks; assert(condition); } while (0)

/** @brief 查询 Flash 替身布局状态。 @return 本轮设置的布局结果。 */
bool
board_storage_layout_valid(void)
{
    return g_test_layout;
}

/** @brief 从两槽内存读取实际配置记录。 @param slot 槽号。 @param data 目标缓冲。 @param size 字节数。 @return 读取成功。 */
bool
board_storage_read(unsigned slot, void *data, size_t size)
{
    CHECK(slot < 2U && size <= BOARD_STORAGE_SLOT_SIZE);
    if (!g_test_layout) { return false; }
    memcpy(data, g_test_flash[slot], size);
    return true;
}

/** @brief 模拟擦除、双字写入和任意双字处掉电。 @param slot 槽号。 @param data 待写记录。 @param size 字节数。 @return 完整写入。 */
bool
board_storage_write(unsigned slot, const void *data, size_t size)
{
    CHECK(slot < 2U && size % 8U == 0U && size <= BOARD_STORAGE_SLOT_SIZE);
    if (!g_test_layout) { return false; }
    ++g_test_writes;
    g_test_last_slot = slot;
    memset(g_test_flash[slot], 0xFF, sizeof(g_test_flash[slot]));
    size_t written = g_test_cut_after < size ? g_test_cut_after : size;
    memcpy(g_test_flash[slot], data, written);
    return written == size;
}

/** @brief 为冻结的历史测试记录生成标准 CRC32。 @param record 记录。 @return 不含 CRC/提交标志的校验值。 */
static uint32_t
bytes_crc(const void *record,size_t size)
{
    uint32_t crc = UINT32_MAX;
    const unsigned char *bytes = (const unsigned char *)record;
    for (size_t i = 0; i < size; ++i)
    {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8U; ++bit)
        {
            crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? 0xEDB88320UL : 0U);
        }
    }
    return ~crc;
}

/** @brief 冻结旧CRC偏移。 */
static uint32_t
fixture_crc(const fixture_record_t *record)
{ return bytes_crc(record,offsetof(fixture_record_t,crc)); }

/** @brief 建立保留 50 Hz/20% 保存值、且各旧占位项非零的历史记录。 @param version 格式版本。 @param sequence 序号。 @return 完整有效记录。 */
static fixture_record_t
fixture_record(uint32_t version, uint32_t sequence)
{
    fixture_record_t record = {
        .magic = 0x464F4332UL, .version = version, .length = 160U, .sequence = sequence,
        .data = {
            .parameter = {
                1000, 3, 1500, 5400, -2300, 4500, 201, 202, 203, 204,
                205, 206, 207, 208, 209, 50000, 20000, 200, 700, 75,
                2010, 2020, 2030, -1, 1234, 2345, 678, 901, 123, 50,
                12000, 2500, 5400, 200, 20000
            },
            .brightness = 73, .animation = 0, .algorithm = CONTROL_OPEN_6STEP,
            .mode = CONTROL_SPEED, .source = CONTROL_REMOTE
        },
        .commit = 0x434F4D54UL
    };
    record.crc = fixture_crc(&record);
    return record;
}

/** @brief 直接装入独立构造的记录，不借助被测保存函数。 @param slot 槽号。 @param record 已确定的记录字节。 */
static void
fixture_load(unsigned slot, const fixture_record_t *record)
{
    memcpy(g_test_flash[slot], record, sizeof(*record));
}

/** @brief 清空两槽并恢复无断电、布局有效的环境。 */
static void
reset_storage(void)
{
    memset(g_test_flash, 0xFF, sizeof(g_test_flash));
    g_test_writes = 0;
    g_test_cut_after = UINT_MAX;
    g_test_layout = true;
    control_model_init();
}

/** @brief 核对迁移只清四个旧占位值，其他所有字节保持。 @param before 原始旧载荷。 */
static void
check_migrated(const legacy_data_t *before)
{
    legacy_data_t expected = *before;
    expected.parameter[CONTROL_PARAM_HANDOFF] = 0;
    expected.parameter[CONTROL_PARAM_COMM_TIMING] = 0;
    expected.parameter[CONTROL_PARAM_SPEED_KP] = 0;
    expected.parameter[CONTROL_PARAM_SPEED_KI] = 0;
    CHECK(memcmp(settings_get(), &expected, sizeof(expected)) == 0);
    CHECK(settings_get()->parameter[CONTROL_PARAM_START_HZ] == 50000);
    CHECK(settings_get()->parameter[CONTROL_PARAM_DUTY] == 200);
    CHECK(strcmp(settings_status(), "PENDING") == 0);
}

/** @brief 所有历史算法、显示及来源保持，迁移仅在安全停机且到期后保存另一槽。 */
static void
test_v1_migration(void)
{
    for (unsigned algorithm = 0; algorithm < CONTROL_ALGORITHM_COUNT; ++algorithm)
    {
        reset_storage();
        fixture_record_t old = fixture_record(1, 41);
        old.data.algorithm = algorithm;
        old.data.mode = control_algorithm_foc((control_algorithm_t)algorithm) ? CONTROL_POSITION : CONTROL_SPEED;
        old.crc = fixture_crc(&old);
        fixture_load(0, &old);
        settings_init();
        check_migrated(&old.data);
        CHECK(g_test_writes == 0U);
        settings_service(false, MOTOR_SAVE_DELAY_MS + 5000U);
        settings_service(true, MOTOR_SAVE_DELAY_MS - 1U);
        CHECK(g_test_writes == 0U);
        settings_service(true, MOTOR_SAVE_DELAY_MS);
        CHECK(g_test_writes == 1U && g_test_last_slot == 1U);
        CHECK(memcmp(g_test_flash[0], &old, sizeof(old)) == 0);
        v6_record_t saved;
        memcpy(&saved, g_test_flash[1], sizeof(saved));
        CHECK(saved.version == 6U && saved.sequence == 42U && saved.crc == bytes_crc(&saved,offsetof(v6_record_t,crc)));
        settings_data_t migrated = *settings_get();
        settings_init();
        CHECK(memcmp(settings_get(), &migrated, sizeof(migrated)) == 0);
        CHECK(strcmp(settings_status(), "SAVED") == 0);
        settings_service(true, MOTOR_SAVE_DELAY_MS + 10000U);
        CHECK(g_test_writes == 1U);
    }
}

/** @brief 新格式的物理参数保存后恢复原值，单位和定点尺度一致。 */
static void
test_v2_roundtrip_and_units(void)
{
    reset_storage();
    fixture_record_t record = fixture_record(2, 99);
    record.data.algorithm = CONTROL_SENSORLESS_6STEP;
    record.crc = fixture_crc(&record);
    fixture_load(0, &record);
    settings_init();
    CHECK(memcmp(settings_get(), &record.data, sizeof(record.data)) == 0);
    CHECK(strcmp(settings_status(), "PENDING") == 0);
    CHECK(control_model_select(CONTROL_SENSORLESS_6STEP, CONTROL_SPEED) == CONTROL_OK);
    settings_publish();
    CHECK(strcmp(control_model_parameter(CONTROL_PARAM_HANDOFF)->unit, "RPM") == 0);
    CHECK(strcmp(control_model_parameter(CONTROL_PARAM_COMM_TIMING)->unit, "DEG") == 0);
    CHECK(strcmp(control_model_parameter(CONTROL_PARAM_SPEED_KP)->unit, "D/RPM") == 0);
    CHECK(strcmp(control_model_parameter(CONTROL_PARAM_SPEED_KI)->unit, "D/RPMS") == 0);
    CHECK(control_model_parameter(CONTROL_PARAM_COMM_TIMING)->decimals == 1U);
    CHECK(control_model_parameter(CONTROL_PARAM_SPEED_KP)->decimals == 3U);
    CHECK(control_model_parameter(CONTROL_PARAM_SPEED_KI)->decimals == 3U);
    CHECK(settings_set(CONTROL_PARAM_HANDOFF, 1000, 2000));
    CHECK(settings_set(CONTROL_PARAM_COMM_TIMING, -35, 2000));
    CHECK(settings_set(CONTROL_PARAM_SPEED_KP, 1234, 2000));
    CHECK(settings_set(CONTROL_PARAM_SPEED_KI, 5678, 2000));
    settings_data_t expected = *settings_get();
    settings_service(true, 2000U + MOTOR_SAVE_DELAY_MS);
    CHECK(g_test_writes == 1U && g_test_last_slot == 1U);
    v6_record_t saved;
    memcpy(&saved, g_test_flash[1], sizeof(saved));
    CHECK(saved.version == 6U && saved.sequence == 100U);
    settings_init();
    CHECK(memcmp(settings_get(), &expected, sizeof(expected)) == 0);
    CHECK(strcmp(settings_status(), "SAVED") == 0);
}

/** @brief 混合版本只按序号选择最新合法记录，覆盖序号环绕。 */
static void
test_mixed_versions(void)
{
    for (unsigned latest_slot = 0; latest_slot < 2U; ++latest_slot)
    {
        for (unsigned version = 1; version <= 2U; ++version)
        {
            reset_storage();
            fixture_record_t newer = fixture_record(version, 101);
            fixture_record_t older = fixture_record(3U - version, 100);
            older.data.brightness = 27;
            older.crc = fixture_crc(&older);
            fixture_load(latest_slot, &newer);
            fixture_load(1U - latest_slot, &older);
            settings_init();
            if (version == 1U) { check_migrated(&newer.data); }
            else { CHECK(memcmp(settings_get(), &newer.data, sizeof(newer.data)) == 0); }
        }
    }
    reset_storage();
    fixture_record_t old = fixture_record(2, UINT32_MAX);
    fixture_record_t next = fixture_record(1, 0);
    old.data.brightness = 27;
    old.crc = fixture_crc(&old);
    fixture_load(0, &old);
    fixture_load(1, &next);
    settings_init();
    check_migrated(&next.data);
    settings_service(true, MOTOR_SAVE_DELAY_MS);
    memcpy(&next, g_test_flash[0], sizeof(next));
    CHECK(next.version == 6U && next.sequence == 1U);
}

/** @brief 迁移保存的所有半写位置都退回旧记录，故障状态不能连续擦写。 */
static void
test_migration_power_loss(void)
{
    for (unsigned cut = 0; cut < sizeof(v6_record_t); cut += 8U)
    {
        reset_storage();
        fixture_record_t old = fixture_record(1, 55);
        fixture_load(0, &old);
        settings_init();
        g_test_cut_after = cut;
        settings_service(true, MOTOR_SAVE_DELAY_MS);
        CHECK(strcmp(settings_status(), "ERROR") == 0 && g_test_writes == 1U);
        settings_service(true, MOTOR_SAVE_DELAY_MS + 10000U);
        CHECK(g_test_writes == 1U);
        CHECK(memcmp(g_test_flash[0], &old, sizeof(old)) == 0);
        settings_init();
        check_migrated(&old.data);
        g_test_cut_after = UINT_MAX;
        settings_service(true, MOTOR_SAVE_DELAY_MS);
        v6_record_t saved;
        memcpy(&saved, g_test_flash[1], sizeof(saved));
        CHECK(saved.version == 6U && saved.sequence == 56U);
    }
}

/** @brief 新记录 CRC、提交、长度、版本或参数异常时回退旧槽；V1 也必须先完整验证。 */
static void
test_invalid_records(void)
{
    for (unsigned version = 1; version <= 2U; ++version)
    {
        for (unsigned defect = 0; defect < 8U; ++defect)
        {
            reset_storage();
            fixture_record_t old = fixture_record(2, 10);
            fixture_record_t bad = fixture_record(version, 11);
            if (defect == 0U) { bad.crc ^= 1U; }
            else if (defect == 1U) { bad.commit = UINT32_MAX; }
            else
            {
                if (defect == 2U) { bad.length -= 4U; }
                if (defect == 3U) { bad.version = 0; }
                if (defect == 4U) { bad.version = 3; }
                if (defect == 5U) { bad.version = UINT32_MAX; }
                if (defect == 6U) { bad.data.parameter[CONTROL_PARAM_HANDOFF] = INT32_MAX; }
                if (defect == 7U) { bad.data.algorithm = CONTROL_ALGORITHM_COUNT; }
                bad.crc = fixture_crc(&bad);
            }
            fixture_load(0, &old);
            fixture_load(1, &bad);
            settings_init();
            CHECK(memcmp(settings_get(), &old.data, sizeof(old.data)) == 0);
            CHECK(strcmp(settings_status(), "PENDING") == 0);
        }
    }
    reset_storage();
    fixture_record_t bad = fixture_record(3, 999);
    fixture_load(0, &bad);
    settings_init();
    CHECK(strcmp(settings_status(), "DEFAULT") == 0);
    CHECK(settings_get()->parameter[CONTROL_PARAM_SPEED_KP] == 0);
    CHECK(settings_get()->parameter[CONTROL_PARAM_SPEED_KI] == 0);
    g_test_layout = false;
    settings_init();
    CHECK(strcmp(settings_status(), "ERROR") == 0);
}

/** @brief FOC与六步独立保存，V3校验和两槽回退覆盖新增载荷及掉电边界。 */
static void
test_v3_foc_independence(void)
{
    reset_storage();
    fixture_record_t old=fixture_record(2U,300U);
    fixture_load(0,&old);
    settings_init();
    CHECK(settings_get()->foc.target_rpm==1000 && settings_get()->foc.current_limit_ma==500);
    CHECK(settings_get()->foc.speed_kp_micro==200 && settings_get()->foc.speed_ki_micro==500);
    CHECK(control_model_select(CONTROL_SENSORLESS_FOC,CONTROL_SPEED)==CONTROL_OK);
    settings_publish();
    CHECK(settings_value(CONTROL_PARAM_TARGET)==1000 && settings_value(CONTROL_PARAM_CURRENT_LIMIT)==500);
    CHECK(strcmp(control_model_parameter(CONTROL_PARAM_SPEED_KP)->unit,"MA/RPM")==0);
    CHECK(strcmp(control_model_parameter(CONTROL_PARAM_RAMP)->unit,"RPM/S")==0);
    CHECK(control_model_parameter(CONTROL_PARAM_RESISTANCE)->valid && !control_model_parameter(CONTROL_PARAM_RESISTANCE)->editable);
    CHECK(!settings_set(CONTROL_PARAM_RESISTANCE,791,100));
    CHECK(!settings_set(CONTROL_PARAM_TARGET,0,100));
    CHECK(!settings_set(CONTROL_PARAM_TARGET,5401,100));
    CHECK(settings_set(CONTROL_PARAM_TARGET,1,100));
    CHECK(settings_set(CONTROL_PARAM_TARGET,5400,100));
    CHECK(!settings_set(CONTROL_PARAM_CURRENT_LIMIT,2001,100));
    CHECK(settings_set(CONTROL_PARAM_TARGET,2000,100));
    CHECK(settings_set(CONTROL_PARAM_CURRENT_LIMIT,2000,100));
    CHECK(settings_set(CONTROL_PARAM_SPEED_KP,300,100));
    CHECK(memcmp(settings_get(),&old.data,sizeof(old.data))==0);
    settings_service(false,2000);
    CHECK(g_test_writes==0U);
    settings_service(true,2000);
    CHECK(g_test_writes==1U && memcmp(g_test_flash[0],&old,sizeof(old))==0);
    settings_data_t expected=*settings_get();
    settings_init();
    CHECK(memcmp(settings_get(),&expected,sizeof(expected))==0 && strcmp(settings_status(),"SAVED")==0);
    CHECK(control_model_select(CONTROL_SENSORLESS_6STEP,CONTROL_SPEED)==CONTROL_OK);
    settings_publish();
    CHECK(settings_value(CONTROL_PARAM_TARGET)==old.data.parameter[CONTROL_PARAM_TARGET]);
    CHECK(settings_value(CONTROL_PARAM_SPEED_KP)==old.data.parameter[CONTROL_PARAM_SPEED_KP]);
    CHECK(strcmp(control_model_parameter(CONTROL_PARAM_SPEED_KP)->unit,"D/RPM")==0);
    CHECK(strcmp(control_model_parameter(CONTROL_PARAM_RAMP)->unit,"HZ/S")==0);
    v6_record_t stable;
    memcpy(&stable,g_test_flash[1],sizeof(stable));
    for(unsigned defect=0;defect<4U;++defect)
    {
        v6_record_t bad=stable;
        if(defect==0) { bad.data.foc.target_rpm=0; }
        if(defect==1) { bad.data.foc.resistance_mohm=0; }
        if(defect==2) { bad.data.foc.bandwidth_hz=0; }
        if(defect==3) { bad.data.foc.target_iq_ma=501; }
        bad.crc=bytes_crc(&bad,offsetof(v6_record_t,crc));
        memcpy(g_test_flash[1],&bad,sizeof(bad));
        settings_init();
        CHECK(memcmp(settings_get(),&old.data,sizeof(old.data))==0);
        CHECK(settings_get()->foc.current_limit_ma==500 && strcmp(settings_status(),"PENDING")==0);
    }
}

/** @brief 独立配置迁移回归入口。 @return 全部通过返回 0。 */
int
main(void)
{
    test_v1_migration();
    test_v2_roundtrip_and_units();
    test_mixed_versions();
    test_migration_power_loss();
    test_invalid_records();
    test_v3_foc_independence();
    printf("settings migration: %u checks passed\n", g_test_checks);
    return 0;
}
