#include "wave_stream.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t packet[WAVE_PACKET_CAPACITY], decoded[WAVE_PACKET_CAPACITY];
/* 测试保持源编号，检查溢出后不能伪造连续的固定采样流。 */
static unsigned checks;
#define CHECK(value) do { ++checks; assert(value); } while(0)

/** @brief 独立解码COBS供检查包头与有效数值。 @param size 线上长度。 @return 解码长度。 */
static size_t decode(size_t size)
{
    CHECK(size>2U && packet[0]==0U && packet[size-1U]==0U);
    size_t input=1U, output=0U;
    while(input<size-1U)
    {
        unsigned code=packet[input++]; CHECK(code!=0U && input+code-1U<=size-1U);
        for(unsigned n=1;n<code;++n) { decoded[output++]=packet[input++]; }
        if(code<255U && input<size-1U) { decoded[output++]=0U; }
    }
    return output;
}
/** @brief 读取小端32位字段。 @param at 偏移。 @return 数值。 */
static uint32_t get32(size_t at)
{ return (uint32_t)decoded[at]|((uint32_t)decoded[at+1U]<<8U)|((uint32_t)decoded[at+2U]<<16U)|((uint32_t)decoded[at+3U]<<24U); }
/** @brief 产生有独立编号的真实样本替身，不执行电机控制。 @param sequence 序号。 */
static void sample(uint32_t sequence)
{
    wave_sample_t row={.sequence=sequence,.ticks=sequence*17000U,
        .raw={2049U,2047U,2048U},.vdda_mv=3300U,.id_a=.01f,.iq_a=-.01f,
        .reference_d_a=0.f,.reference_q_a=.125f};
    wave_stream_capture(&row);
}
/** @brief 建立明确配置的测试会话。 @param rate 点率。 */
static void begin(uint32_t rate)
{
    const uint16_t zero[3]={2048U,2048U,2048U};
    wave_stream_init(); CHECK(wave_stream_set_rate(rate));
    wave_stream_enable(true); wave_stream_begin(zero,1U,false);
}
/** @brief 覆盖首块、尾块和结束统计，生成跨语言CRC核验原文。 @param fixture 可选输出文件。 */
static void blocks(const char *fixture)
{
    begin(10000U);
    for(unsigned n=1;n<=40U;++n) { sample(n); }
    size_t length=wave_stream_next(packet,sizeof(packet));
    CHECK(length<=484U); CHECK(decode(length)==480U);
    CHECK(memcmp(decoded,"FCV1",4U)==0); CHECK(decoded[5]==1U && decoded[10]==32U);
    CHECK(get32(16U)==1U && get32(20U)==17000U && get32(24U)==17000U);
    CHECK(decoded[28]==7U && decoded[30]==249U && decoded[31]==255U);
    CHECK(decoded[34]==82U && decoded[36]==174U && decoded[37]==255U);
    if(fixture!=NULL)
    { FILE *file=fopen(fixture,"wb"); CHECK(file!=NULL); CHECK(fwrite(packet,1U,length,file)==length); fclose(file); }
    CHECK(wave_stream_next(packet,sizeof(packet))==0U);
    wave_stream_end(); CHECK(decode(wave_stream_next(packet,sizeof(packet)))==144U);
    CHECK(decoded[10]==8U && get32(16)==33U);
    CHECK(decode(wave_stream_next(packet,sizeof(packet)))==64U && decoded[5]==3U);
    CHECK(wave_stream_stats().captured==40U && wave_stream_stats().sent==40U);
    CHECK(wave_stream_next(packet,sizeof(packet))==0U);
}
/** @brief 队列满后保留缺口，禁止补发伪样本。 */
static void overflow(void)
{
    begin(10000U);
    for(unsigned n=1;n<=300U;++n) { sample(n); }
    CHECK(wave_stream_stats().captured==256U && wave_stream_stats().dropped==44U);
    for(unsigned n=0;n<8U;++n) { CHECK(decode(wave_stream_next(packet,sizeof(packet)))==480U); }
    for(unsigned n=301U;n<=332U;++n) { sample(n); }
    CHECK(decode(wave_stream_next(packet,sizeof(packet)))==480U && get32(16)==301U);
    CHECK(wave_stream_stats().backlog_max==256U);
}
/** @brief 验证5kHz只取源奇数样点，关闭后立即停止产包。 */
static void decimation(void)
{
    begin(5000U);
    for(unsigned n=1;n<=64U;++n) { sample(n); }
    CHECK(decode(wave_stream_next(packet,sizeof(packet)))==480U);
    CHECK(get32(16)==1U && get32(24)==34000U);
    CHECK(wave_stream_stats().captured==32U);
    CHECK(!wave_stream_set_rate(10000U));
    wave_stream_enable(false); sample(65U);
    CHECK(wave_stream_next(packet,sizeof(packet))==0U);
}
/** @brief 运行纯协议回归，不访问HAL或硬件。 @param argc 参数数。 @param argv 可选fixture路径。 @return 成功0。 */
int main(int argc,char **argv)
{
    CHECK(sizeof(wave_sample_t)==32U);
    blocks(argc>1?argv[1]:NULL); overflow(); decimation();
    /* 测试源使用独立循环，检查完成和采样计数，不复用控制入口。 */
    wave_stream_init(); CHECK(wave_stream_set_rate(10000U));
    wave_stream_test_start(UINT32_MAX-50000U,100U,10U);
    for(uint32_t ms=1;ms<=11U;++ms)
    {
        wave_stream_test_service(UINT32_MAX-50000U+ms*170000U,100U+ms);
        size_t size;
        while((size=wave_stream_next(packet,sizeof(packet)))!=0U)
        { (void)decode(size); CHECK((decoded[6]&WAVE_FLAG_TEST)!=0U); }
    }
    CHECK(!wave_stream_testing() && wave_stream_stats().captured==100U);
    /* 停机睡眠使DWT不动时仍由系统毫秒产生完整的有界TEST流。 */
    wave_stream_init(); wave_stream_test_start(1234U,UINT32_MAX-4U,10U);
    for(uint32_t ms=1U;ms<=11U;++ms)
    {
        wave_stream_test_service(1234U,UINT32_MAX-4U+ms);
        while(wave_stream_next(packet,sizeof(packet))!=0U) { }
    }
    CHECK(!wave_stream_testing() && wave_stream_stats().captured==50U);
    printf("binary wave: %u checks passed\n",checks);
    return 0;
}
