#include "switch_input.h"
#include "console.h"
#include "serial_buffer.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
/* 断言累计和当前编译极性对应的空闲电平。 */
static unsigned g_checks;
static uint8_t g_idle;
#define CHECK(x) do { ++g_checks; assert(x); } while (0)
/** @brief 将逻辑按下转为物理电平。 @param active 逻辑位。 @return 原始电平。 */
static uint8_t
raw(uint8_t active)
{
    return g_idle ^ active;
}
/** @brief 验证消抖、长按、方向消抖、回中、冲突、上电保持和计时回绕。 */
static void
test_switch(void)
{
    switch_input_t s;
    switch_event_t events[2];
    g_idle = (SWITCH_PRESS_ACTIVE_LEVEL ? 0U : 1U) |
             (SWITCH_DOWN_ACTIVE_LEVEL ? 0U : 2U) |
             (SWITCH_UP_ACTIVE_LEVEL ? 0U : 4U);
    switch_input_init(&s,raw(0),0);
    CHECK(switch_input_update(&s,raw(0),20,events) == 0);
    CHECK(switch_input_update(&s,raw(1),30,events) == 0);
    CHECK(switch_input_update(&s,raw(0),35,events) == 0);
    CHECK(switch_input_update(&s,raw(1),40,events) == 0);
    CHECK(switch_input_update(&s,raw(1),59,events) == 0);
    CHECK(switch_input_update(&s,raw(1),60,events) == 0);
    CHECK(switch_input_update(&s,raw(0),100,events) == 0);
    CHECK(switch_input_update(&s,raw(0),120,events) == 1 && events[0] == SWITCH_CLICK);
    CHECK(switch_input_update(&s,raw(1),130,events) == 0);
    CHECK(switch_input_update(&s,raw(1),150,events) == 0);
    CHECK(switch_input_update(&s,raw(1),949,events) == 0);
    CHECK(switch_input_update(&s,raw(1),950,events) == 1 && events[0] == SWITCH_LONG_PRESS);
    CHECK(switch_input_update(&s,raw(1),2000,events) == 0);
    CHECK(switch_input_update(&s,raw(0),2010,events) == 0);
    CHECK(switch_input_update(&s,raw(0),2030,events) == 0);
    CHECK(switch_input_update(&s,raw(2),2040,events)==0);
    CHECK(switch_input_update(&s,raw(2),2059,events)==0);
    CHECK(switch_input_update(&s,raw(2),2060,events)==1 && events[0]==(SWITCH_SWAP_UP_DOWN?SWITCH_UP:SWITCH_DOWN));
    CHECK(switch_input_update(&s,raw(2),4000,events)==0);
    CHECK(switch_input_update(&s,raw(4),4010,events)==0);
    CHECK(switch_input_update(&s,raw(4),4030,events)==0);
    CHECK(switch_input_update(&s,raw(0),4040,events)==0);
    CHECK(switch_input_update(&s,raw(0),4060,events)==0);
    CHECK(switch_input_update(&s,raw(4),4070,events)==0);
    CHECK(switch_input_update(&s,raw(4),4090,events)==1 && events[0]==(SWITCH_SWAP_UP_DOWN?SWITCH_DOWN:SWITCH_UP));
    CHECK(switch_input_update(&s,raw(0),4100,events)==0);
    CHECK(switch_input_update(&s,raw(0),4120,events)==0);
    CHECK(switch_input_update(&s,raw(6),4130,events)==0);
    CHECK(switch_input_update(&s,raw(6),4150,events)==0);
    CHECK(switch_input_update(&s,raw(4),4160,events)==0);
    CHECK(switch_input_update(&s,raw(4),4180,events)==0);
    CHECK(s.invalid_transitions==1);
    switch_input_init(&s,raw(7),0);
    CHECK(switch_input_update(&s,raw(7),5000,events)==0);
    CHECK(switch_input_update(&s,raw(0),5010,events)==0);
    CHECK(switch_input_update(&s,raw(0),5030,events)==0);
    switch_input_init(&s,raw(0),UINT32_MAX - 50U);
    CHECK(switch_input_update(&s,raw(0),UINT32_MAX - 30U,events) == 0);
    CHECK(switch_input_update(&s,raw(1),UINT32_MAX - 10U,events) == 0);
    CHECK(switch_input_update(&s,raw(1),9,events) == 0);
    CHECK(switch_input_update(&s,raw(1),809,events) == 1 && events[0] == SWITCH_LONG_PRESS);
}
/** @brief 模拟执行器。 @param argc 参数数。 @param argv 参数。 @param out 响应。 @param cap 容量。 */
static void
handler(unsigned argc, char **argv, char *out, size_t cap)
{
    snprintf(out,cap,"OK %s ARGC=%u",argv[0],argc);
}
/** @brief 输入文本。 @param c 接收器。 @param text 文本。 @param out 响应。 @return 响应数。 */
static unsigned
feed(console_t *c, const char *text, char *out)
{
    unsigned count = 0;
    for (; *text; ++text) { count += console_feed(c,(uint8_t)*text,handler,out) ? 1U : 0U; }
    return count;
}
/** @brief 验证命令边界与收发缓冲。 */
static void
test_console(void)
{
    console_t c;
    char out[CONSOLE_RESPONSE_MAX] = {0};
    console_init(&c);
    CHECK(feed(&c,"pi",out) == 0);
    CHECK(feed(&c,"ng\r\n",out) == 1 && strcmp(out,"OK PING ARGC=1\r\n") == 0);
    CHECK(feed(&c,"PING\nINPUT\n",out) == 2 && strstr(out,"OK INPUT") != NULL);
    for (unsigned i=0;i<96;++i) { CHECK(!console_feed(&c,'A',handler,out)); }
    CHECK(console_feed(&c,'\n',handler,out) && strstr(out,"OK ") == out);
    for (unsigned i=0;i<97;++i) { (void)console_feed(&c,'A',handler,out); }
    CHECK(console_feed(&c,'\n',handler,out) && strstr(out,"TOO_LONG") != NULL);
    CHECK(feed(&c,"PING\rX\n",out) == 1 && strstr(out,"CORRUPT") != NULL);
    CHECK(feed(&c,"\n",out) == 1 && strstr(out,"ARGUMENT") != NULL);
    CHECK(feed(&c,"A B C D E F G H I\n",out) == 1 && strstr(out,"ARGUMENT") != NULL);
    CHECK(feed(&c,"SET BRIGHT",out) == 0);
    CHECK(console_feed(&c,0x100U|'\n',handler,out) && strstr(out,"CORRUPT") != NULL);
    CHECK(feed(&c,"PING\n",out) == 1 && strstr(out,"OK PING") != NULL);
    serial_buffer_init();
    CHECK(serial_buffer_tx_free() == SERIAL_TX_CAPACITY-1U);
    CHECK(serial_buffer_write("HELLO",5));
    CHECK(!serial_buffer_write(out,SERIAL_TX_CAPACITY));
    for (unsigned i=0;i<5;++i)
    {
        uint8_t ch;
        CHECK(serial_buffer_transmit(&ch) && ch == (uint8_t)"HELLO"[i]);
    }
    uint8_t byte;
    CHECK(!serial_buffer_transmit(&byte));
    for (unsigned round=0;round<2000;++round)
    {
        CHECK(serial_buffer_write("PING",4));
        for (unsigned i=0;i<4;++i) { CHECK(serial_buffer_transmit(&byte) && byte == (uint8_t)"PING"[i]); }
    }
    serial_buffer_receive('S',false);
    serial_buffer_receive('E',true);
    serial_buffer_receive('\n',true);
    serial_buffer_receive('T',false);
    serial_buffer_receive('\n',false);
    uint16_t token;
    CHECK(serial_buffer_read(&token) && token == 'S');
    CHECK(serial_buffer_read(&token) && token == (0x100U|'\n'));
    CHECK(!serial_buffer_read(&token));
    for (unsigned i=0;i<600;++i) { serial_buffer_receive('X',false); }
    CHECK(serial_buffer_overflows() > 0U);
    while (serial_buffer_read(&token)) { CHECK(!console_feed(&c,token,handler,out)); }
    serial_buffer_receive('\n',false);
    CHECK(serial_buffer_read(&token) && console_feed(&c,token,handler,out) && strstr(out,"CORRUPT") != NULL);
    serial_buffer_receive('P',false);
    CHECK(serial_buffer_read(&token) && token == 'P');
}
/** @brief 主机测试入口。 @return 成功为0。 */
int
main(void)
{
    test_switch();
    test_console();
    printf("logic: %u checks passed\n",g_checks);
    return 0;
}
