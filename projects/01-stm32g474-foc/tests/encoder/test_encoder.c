#include "encoder_readout.h"
#include "mt6835.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* 独立模型记录总检查数和总线事务，验证错误传播及禁止的写命令。 */
static unsigned g_checks, g_calls, g_fail_at, g_illegal, g_bus_mode;
/* 模拟易失USER_ID，允许检查探测后是否恢复。 */
static uint8_t g_user_id;
/* 两个配置影子，mock禁止EEPROM编程并只放行迟滞单字节写。 */
static uint8_t g_hyst=0xD7U,g_bandwidth=0xA5U;
/* 采用170MHz计数器验证实际部署时的数值范围和回绕。 */
#define TICK_HZ 170000000U
#define CHECK(x) do { ++g_checks; assert(x); } while (0)

/** @brief 模拟芯片协议，拒绝EEPROM、标定、置零及其他写入。
 * @param tx 请求。 @param rx 输出。 @param count 长度。 @return 传输是否完成。 */
static bool
mock_bus(const uint8_t *tx, uint8_t *rx, uint16_t count)
{
    ++g_calls;
    memset(rx, 0, count);
    if (g_calls == g_fail_at || g_bus_mode == 4U) { return false; }
    if (g_bus_mode == 1U) { memset(rx, 255, count); return true; }
    if (g_bus_mode == 2U) { return true; }
    if (count == 3U && tx[1] == 1U && (tx[0] == 0x30U || tx[0] == 0x60U))
    {
        if (tx[0] == 0x30U) { rx[2] = g_user_id; }
        else if (g_bus_mode != 3U) { g_user_id = tx[2]; }
        return true;
    }
    if(count==3U && tx[0]==0x30U && (tx[1]==0x0DU || tx[1]==0x11U))
    {
        rx[2]=tx[1]==0x0DU?g_hyst:g_bandwidth;
        if(g_bus_mode==6U && g_calls==3U) { rx[2]^=1U; }
        return true;
    }
    if(count==3U && tx[0]==0x60U && tx[1]==0x0DU)
    { g_hyst=tx[2];return true; }
    if (count == 6U && tx[0] == 0xA0U && tx[1] == 3U)
    {
        /* 独立已知CRC向量12 34 56 -> 7C，低三位状态为6。 */
        rx[2] = 0x12U; rx[3] = 0x34U; rx[4] = 0x56U;
        rx[5] = g_bus_mode == 5U ? 0x7DU : 0x7CU;
        return true;
    }
    ++g_illegal;
    return false;
}

/** @brief 验证正确帧、断线、CRC损坏和每个探测事务失败，错误不覆盖输出。 */
static void
test_driver(void)
{
    mt6835_sample_t sample = {123U, 7U};
    CHECK(!mt6835_probe(NULL));
    CHECK(mt6835_read(&sample) == MT6835_IO_ERROR);
    CHECK(sample.angle == 123U && sample.status == 7U);
    mt6835_config_t config={0x12U,0x34U};
    CHECK(!mt6835_read_config(&config) && config.hysteresis==0x12U && config.bandwidth==0x34U);
    g_user_id = 0xC7U;
    CHECK(mt6835_probe(mock_bus) && g_user_id == 0xC7U);
    unsigned count = g_calls;
    const uint8_t vector[] = {0x12U, 0x34U, 0x56U};
    CHECK(mt6835_crc(vector) == 0x7CU);
    CHECK(mt6835_read(&sample) == MT6835_OK);
    CHECK(sample.angle == (0x123456U >> 3U) && sample.status == 6U);
    CHECK(mt6835_read(NULL) == MT6835_IO_ERROR);
    for (unsigned mode = 1U; mode <= 4U; ++mode)
    {
        g_bus_mode = mode; g_calls = 0U; g_user_id = 0xC7U;
        CHECK(!mt6835_probe(mock_bus));
    }
    g_bus_mode = 0U;
    for (unsigned failure = 1U; failure <= count; ++failure)
    {
        g_calls = 0U; g_user_id = 0xC7U; g_fail_at = failure;
        CHECK(!mt6835_probe(mock_bus));
        /* 最初读取失败时不允许写回一个并未获取的旧值。 */
        if (failure == 1U) { CHECK(g_calls == 1U && g_user_id == 0xC7U); }
    }
    g_fail_at = 0U;
    g_bus_mode = 5U;
    CHECK(mt6835_probe(mock_bus));
    sample = (mt6835_sample_t){123U, 7U};
    CHECK(mt6835_read(&sample) == MT6835_CRC_ERROR);
    CHECK(sample.angle == 123U && sample.status == 7U);
    CHECK(g_illegal == 0U);
    g_bus_mode=0U;g_fail_at=0U;
    CHECK(mt6835_read_config(&config) && config.hysteresis==0xD7U && config.bandwidth==0xA5U);
    CHECK(!mt6835_read_config(NULL));
    for(unsigned failure=1U;failure<=4U;++failure)
    {
        config=(mt6835_config_t){0x12U,0x34U};g_calls=0U;g_fail_at=failure;
        CHECK(!mt6835_read_config(&config) && config.hysteresis==0x12U && config.bandwidth==0x34U);
    }
    g_fail_at=0U;
    CHECK(g_illegal==0U && g_user_id==0xC7U);
    g_calls=0U;g_bus_mode=6U;config=(mt6835_config_t){0x12U,0x34U};
    CHECK(!mt6835_read_config(&config) && config.hysteresis==0x12U && config.bandwidth==0x34U);
    g_bus_mode=0U;
    CHECK(!mt6835_set_hysteresis(3U) && !mt6835_set_hysteresis(8U));
    for(unsigned code=4U;code<=7U;++code)
    {
        CHECK(mt6835_set_hysteresis((uint8_t)code));
        CHECK(g_hyst==(uint8_t)(0xD0U|code) && g_bandwidth==0xA5U && g_user_id==0xC7U);
    }
    for(unsigned failure=5U;failure<=9U;++failure)
    {
        g_calls=0U;g_fail_at=failure;g_hyst=0xD7U;
        CHECK(!mt6835_set_hysteresis(5U) && g_hyst==0xD7U && g_bandwidth==0xA5U);
    }
    g_fail_at=0U;CHECK(g_illegal==0U);
}

/** @brief 对单圈机械角生成独立的匀速轨迹，包含正反向与跨零。
 * @param rpm 有符号机械转速。 @param elapsed_us 经过微秒。 @return 原始角度码。 */
static uint32_t
trajectory(int32_t rpm, uint32_t elapsed_us)
{
    int64_t position = (int64_t)ENCODER_COUNTS_PER_TURN * rpm * elapsed_us / 60000000;
    position += ENCODER_COUNTS_PER_TURN - 10U;
    position %= ENCODER_COUNTS_PER_TURN;
    if (position < 0) { position += ENCODER_COUNTS_PER_TURN; }
    return (uint32_t)position;
}

/** @brief 匀速轨迹与DWT/HAL回绕的机械RPM检查。
 * @param rpm 目标机械RPM。 @param wrap true覆盖两个计数器回绕。 */
static void
test_speed(int32_t rpm, bool wrap)
{
    encoder_readout_t readout;
    encoder_readout_init(&readout, TICK_HZ);
    uint32_t start_ms = wrap ? UINT32_MAX - 40U : 0U;
    uint32_t start_tick = wrap ? UINT32_MAX - 10000U : 0U;
    for (unsigned i = 0U; i <= 1000U; ++i)
    {
        /* 0.8～1.2ms抖动采集，速度必须使用实际时间戳而非固定1ms。 */
        uint32_t elapsed_us = i * 1000U + (i % 3U) * 100U;
        uint32_t now_ms = start_ms + elapsed_us / 1000U;
        encoder_readout_sample(&readout, trajectory(rpm, elapsed_us), 0U, now_ms,
            start_tick + elapsed_us * (TICK_HZ / 1000000U));
        encoder_snapshot_t s = encoder_readout_snapshot(&readout, now_ms);
        CHECK(s.angle_valid && s.state == ENCODER_OK && s.age_ms == 0U);
        if (i < 20U) { CHECK(!s.speed_valid); }
        if (i >= 21U) { CHECK(s.speed_valid && s.rpm >= rpm - 1 && s.rpm <= rpm + 1); }
    }
}

/** @brief 验证弱磁场、欠压、陈旧数据、长间隔及通信故障不会留下有效速度。 */
static void
test_validity(void)
{
    encoder_readout_t r;
    encoder_readout_init(&r, TICK_HZ);
    encoder_snapshot_t s = encoder_readout_snapshot(&r, 0U);
    CHECK(s.state == ENCODER_STARTING && s.age_ms == UINT32_MAX && !s.angle_valid && !s.speed_valid);
    for (unsigned status = 0U; status < 8U; ++status)
    {
        encoder_readout_sample(&r, ENCODER_COUNTS_PER_TURN - 1U, (uint8_t)status, status, status * 170000U);
        s = encoder_readout_snapshot(&r, status);
        CHECK(s.status == status && s.angle_mdeg == 359999U && s.angle_valid == (status == 0U));
        if (status != 0U) { CHECK(!s.speed_valid); }
        if (status == 2U) { CHECK(s.state == ENCODER_WEAK_FIELD); }
        if ((status & 5U) != 0U) { CHECK(s.state == ENCODER_SENSOR_FAULT); }
    }
    encoder_readout_init(&r, TICK_HZ);
    for (unsigned i = 0U; i <= 30U; ++i)
    {
        encoder_readout_sample(&r, 777777U, 0U, i, i * 170000U);
    }
    s = encoder_readout_snapshot(&r, 30U);
    CHECK(s.angle_valid && s.speed_valid && s.rpm == 0);
    s = encoder_readout_snapshot(&r, 50U);
    CHECK(s.angle_valid && s.speed_valid && s.age_ms == 20U);
    s = encoder_readout_snapshot(&r, 51U);
    CHECK(s.state == ENCODER_STALE && !s.angle_valid && !s.speed_valid);
    encoder_readout_sample(&r, 888888U, 0U, 60U, 60U * 170000U);
    s = encoder_readout_snapshot(&r, 60U);
    CHECK(s.angle_valid && !s.speed_valid);
    const encoder_state_t failures[] = {ENCODER_IO_ERROR, ENCODER_CRC_ERROR, ENCODER_NO_DEVICE};
    for (unsigned i = 0U; i < 3U; ++i)
    {
        encoder_readout_error(&r, failures[i]);
        s = encoder_readout_snapshot(&r, 60U);
        CHECK(s.state == failures[i] && !s.angle_valid && !s.speed_valid);
        encoder_readout_sample(&r, 0U, 0U, 61U + i, (61U + i) * 170000U);
        CHECK(!encoder_readout_snapshot(&r, 61U + i).speed_valid);
    }
    CHECK(r.value.io_errors == 2U && r.value.crc_errors == 1U);
    encoder_readout_sample(&r, ENCODER_COUNTS_PER_TURN / 2U, 0U, 64U, 64U * 170000U);
    CHECK(!encoder_readout_snapshot(&r, 64U).speed_valid);
    encoder_readout_sample(&r, ENCODER_COUNTS_PER_TURN, 0U, 65U, 65U * 170000U);
    CHECK(encoder_readout_snapshot(&r, 65U).state == ENCODER_IO_ERROR);
    encoder_readout_init(&r, 0U);
    encoder_readout_sample(&r, 0U, 0U, 0U, 0U);
    CHECK(!encoder_readout_snapshot(&r, 0U).angle_valid);
    CHECK(strcmp(encoder_state_name(ENCODER_WEAK_FIELD), "WEAK_FIELD") == 0);
    CHECK(strcmp(encoder_state_name((encoder_state_t)100), "UNKNOWN") == 0);
}

/** @brief 执行芯片协议和独立角度轨迹验证，不接触真实设备。 @return 测试退出码。 */
int
main(void)
{
    test_driver();
    const int32_t speeds[] = {0, 1, -1, 400, -400, 1000, -1000, 2000, 5400, -5400, 6000};
    for (unsigned i = 0U; i < sizeof(speeds) / sizeof(speeds[0]); ++i)
    {
        test_speed(speeds[i], false);
        test_speed(speeds[i], true);
    }
    test_validity();
    printf("Encoder protocol/readout checks=%u failures=0\n", g_checks);
    return 0;
}
