#include "ssd1306.h"
#include <stdio.h>
#include <string.h>

/* 模拟 SSD1306 页缓冲、地址及故障注入。 */
static uint8_t g_frame[8][128], g_address, g_page;
/* 统计有效页和开屏；数据发送失败由 g_fail_data 注入。 */
static unsigned g_pages;
static bool g_on, g_fail_data, g_bad_command;
/* 测试断言失败数。 */
static unsigned g_failures, g_checks;

/** @brief 记录断言。 @param pass 条件。 @param name 用例说明。 */
static void check(bool pass, const char *name)
{
    ++g_checks;
    if (!pass) { ++g_failures; printf("FAIL %s\n", name); }
}

/** @brief 模拟 SSD1306 的关键命令约束与页写入。 @param address 地址。 @param control 控制。 @param data 数据。 @param size 长度。 @return 应答状态。 */
static bool write_mock(uint8_t address, uint8_t control, const uint8_t *data, uint16_t size)
{
    if (address != g_address) { return false; }
    if (control == 0 && size > 3U)
    {
        for (unsigned i = 0; i + 1U < size; ++i)
        {
            if (data[i] == 0xDB && data[i + 1U] != 0x00 && data[i + 1U] != 0x20 && data[i + 1U] != 0x30)
            { g_bad_command = true; return false; }
        }
    }
    if (control == 0 && size == 3U) { g_page = data[0] & 7U; }
    if (control == 0x40)
    {
        if (g_fail_data || size != 128U) { return false; }
        memcpy(g_frame[g_page], data, 128);
        ++g_pages;
    }
    if (control == 0 && size == 1U && data[0] == 0xAF) { g_on = true; }
    return true;
}

/** @brief 验证无屏、备用地址、清屏顺序、错误传播及文字边界。 @return 测试进程结果。 */
int main(void)
{
    check(!ssd1306_init(NULL), "null bus");
    check(!ssd1306_refresh(), "refresh without init");
    g_address = 0;
    check(!ssd1306_init(write_mock) && ssd1306_address() == 0, "absent display");
    g_address = 0x3D;
    check(ssd1306_init(write_mock) && ssd1306_address() == 0x3D, "fallback address");
    check(!g_bad_command, "documented VCOMH value");
    check(g_pages == 8U && g_on, "clear before on");
    ssd1306_text(0, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
    ssd1306_text(1, "?");
    ssd1306_text(2, "~");
    ssd1306_text(8, "OUTSIDE");
    ssd1306_text(0, NULL);
    check(ssd1306_refresh(), "full frame write");
    check(g_frame[0][0] == 0x7E && g_frame[0][126] == 0 && g_frame[0][127] == 0, "text clip and row bounds");
    check(memcmp(g_frame[1], g_frame[2], 128) == 0, "unknown character replacement");
    g_fail_data = true;
    check(!ssd1306_refresh(), "page transfer error");
    check(!ssd1306_init(write_mock) && ssd1306_address() == 0, "failed init clears address");
    g_fail_data = false;
    g_address = 0x3C;
    check(ssd1306_init(write_mock) && ssd1306_address() == 0x3C, "recover primary address");
    ssd1306_clear();
    check(ssd1306_refresh(), "clear refresh");
    unsigned lit = 0;
    for (unsigned p = 0; p < 8U; ++p) { for (unsigned x = 0; x < 128U; ++x) { lit += g_frame[p][x]; } }
    check(lit == 0, "all pixels cleared");
    ssd1306_pattern(1U);
    check(ssd1306_refresh(), "white pattern refresh");
    bool pattern_ok = true;
    for (unsigned p = 0; p < 8U; ++p) { for (unsigned x = 0; x < 128U; ++x) { if (g_frame[p][x] != 0xFFU) { pattern_ok = false; } } }
    check(pattern_ok, "all white pixels");
    ssd1306_pattern(2U);
    check(ssd1306_refresh(), "checker refresh");
    pattern_ok = true;
    for (unsigned p = 0; p < 8U; ++p) { for (unsigned x = 0; x < 128U; ++x) { if (g_frame[p][x] != ((x & 1U) ? 0xAAU : 0x55U)) { pattern_ok = false; } } }
    check(pattern_ok, "checker pixels");
    ssd1306_pattern(0U);
    check(ssd1306_refresh(), "black pattern refresh");
    lit = 0U;
    for (unsigned p = 0; p < 8U; ++p) { for (unsigned x = 0; x < 128U; ++x) { lit += g_frame[p][x]; } }
    check(lit == 0U, "black pixels");
    printf("SSD1306 checks=%u failures=%u\n", g_checks, g_failures);
    return g_failures == 0U ? 0 : 1;
}
