#include "wave_stream.h"
#include <limits.h>
#include <math.h>
#include <string.h>

#if defined(__ARMCC_VERSION)
#define WAVE_FAST __attribute__((section(".motor_fastcode")))
#define WAVE_SMALL __attribute__((minsize))
#else
#define WAVE_FAST
#define WAVE_SMALL
#endif

/* 两端各自拥有一个索引；主循环从不修改ISR写入的head。 */
static wave_sample_t g_wave_rows[WAVE_RING_CAPACITY];
static volatile uint32_t g_wave_head, g_wave_tail;
static volatile bool g_wave_enabled, g_wave_running, g_wave_end_pending;
static bool g_wave_test;
static uint32_t g_wave_rate, g_wave_session, g_wave_units;
static uint16_t g_wave_zero[3];
static volatile wave_stats_t g_wave_stats;
/* 编码临时区仅主循环持有，不放到中断栈。 */
static uint8_t g_wave_packet[480];
static uint8_t g_wave_status[16];
static bool g_wave_status_pending;
static uint32_t g_wave_status_tick, g_wave_status_due, g_wave_last_tick;
static uint16_t g_wave_status_flags;
/* 测试时间有界，源编号与真实电机控制编号分离。 */
static uint32_t g_wave_test_started, g_wave_test_duration, g_wave_test_sequence;

/** @brief 发布或读取队列前执行同核内存屏障。 */
static inline void wave_barrier(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm volatile ("dmb" ::: "memory");
#else
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
#endif
}

/** @brief 小端编码16位值。 @param out 目标。 @param value 数值。 */
static void put16(uint8_t *out, uint16_t value)
{ out[0]=(uint8_t)value; out[1]=(uint8_t)(value>>8U); }
/** @brief 小端编码32位值。 @param out 目标。 @param value 数值。 */
static void put32(uint8_t *out, uint32_t value)
{ put16(out,(uint16_t)value); put16(out+2,(uint16_t)(value>>16U)); }

/** @brief 计算CRC-32/ISO-HDLC，覆盖解码包头与载荷。
 * @param data 字节。 @param size 长度。 @return CRC。 */
static uint32_t WAVE_SMALL wave_crc(const uint8_t *data, size_t size)
{
    uint32_t crc=UINT32_MAX;
    for(size_t i=0;i<size;++i)
    {
        crc^=data[i];
        for(unsigned bit=0;bit<8U;++bit) { crc=(crc>>1U)^((0U-(crc&1U))&0xEDB88320U); }
    }
    return ~crc;
}

/** @brief 编码有界COBS并添加两端零分隔符。
 * @param out 输出。 @param capacity 容量。 @param length 解码包长度。 @return 线上长度。 */
static size_t WAVE_SMALL wave_encode(uint8_t *out, size_t capacity, size_t length)
{
    if(capacity<length+length/254U+4U) { return 0U; }
    size_t write=2U, code_at=1U;
    uint8_t code=1U;
    out[0]=0U;
    for(size_t i=0;i<length;++i)
    {
        if(g_wave_packet[i]==0U)
        { out[code_at]=code; code_at=write++; code=1U; }
        else
        {
            out[write++]=g_wave_packet[i]; ++code;
            if(code==255U) { out[code_at]=code; code_at=write++; code=1U; }
        }
    }
    out[code_at]=code; out[write++]=0U;
    ++g_wave_stats.packets;
    return write;
}

/** @brief 构造固定28字节头；整个包均参加CRC。
 * @param type 1电流/2状态/3结束。 @param flags 来源与有效性。
 * @param count 样点数。 @param length 载荷长度。 @param sequence 首序号。
 * @param ticks 首时标。 @param step 相邻样点tick间隔。 */
static void wave_header(uint8_t type, uint16_t flags, uint16_t count, uint16_t length,
    uint32_t sequence, uint32_t ticks, uint32_t step)
{
    put32(g_wave_packet,0x31564346U); /* 小端字节FCV1，数据格式版本独立于ASCII协议。 */
    g_wave_packet[4]=1U; g_wave_packet[5]=type;
    put16(g_wave_packet+6,flags); put16(g_wave_packet+8,length); put16(g_wave_packet+10,count);
    put32(g_wave_packet+12,g_wave_session); put32(g_wave_packet+16,sequence);
    put32(g_wave_packet+20,ticks); put32(g_wave_packet+24,step);
}

/** @brief 有界Q13量化；截幅明确标记，绝不影响原控制值。
 * @param value 安培。 @param flags 当前包标记。 @return 编码值。 */
static int16_t wave_q13(float value, uint16_t *flags)
{
    float scaled=value*8192.0f;
    if(!isfinite(scaled) || scaled>32767.0f || scaled<-32768.0f)
    {
        *flags|=WAVE_FLAG_CLIPPED; ++g_wave_stats.clipped;
        return isfinite(scaled)?(scaled<0?INT16_MIN:INT16_MAX):0;
    }
    return (int16_t)(int32_t)(scaled+(scaled<0?-.5f:.5f));
}

/** @brief 见wave_stream.h。 */
void wave_stream_init(void)
{
    g_wave_head=g_wave_tail=0U; g_wave_enabled=false; g_wave_running=false;
    g_wave_end_pending=false; g_wave_test=false; g_wave_rate=5000U; g_wave_session=0U;
    g_wave_status_pending=false; memset((void *)&g_wave_stats,0,sizeof(g_wave_stats));
}
/** @brief 见wave_stream.h。 */
bool wave_stream_set_rate(uint32_t rate)
{
    if(g_wave_enabled || (rate!=5000U && rate!=10000U)) { return false; }
    g_wave_rate=rate; return true;
}
/** @brief 见wave_stream.h。 */
uint32_t wave_stream_rate(void) { return g_wave_rate; }
/** @brief 见wave_stream.h。 */
bool WAVE_FAST wave_stream_enabled(void) { return g_wave_enabled; }
/** @brief 见wave_stream.h。 */
void wave_stream_enable(bool enabled)
{
    g_wave_enabled=false; wave_barrier();
    g_wave_stats.cancelled+=g_wave_head-g_wave_tail;
    g_wave_tail=g_wave_head; g_wave_end_pending=false; g_wave_status_pending=false;
    if(enabled) { ++g_wave_session; }
    g_wave_enabled=enabled;
}
/** @brief 见wave_stream.h。 */
void wave_stream_begin(const uint16_t zero[3], uint32_t units, bool test)
{
    g_wave_running=false; wave_barrier();
    g_wave_tail=g_wave_head; g_wave_test=test; ++g_wave_session;
    memcpy(g_wave_zero,zero,sizeof(g_wave_zero)); g_wave_units=units;
    memset((void *)&g_wave_stats,0,sizeof(g_wave_stats));
    g_wave_end_pending=false; g_wave_status_pending=false;
    g_wave_status_due=0U; g_wave_last_tick=0U;
    wave_barrier(); g_wave_running=true;
}
/** @brief 见wave_stream.h。 */
void WAVE_FAST wave_stream_capture(const wave_sample_t *sample)
{
    if(!g_wave_enabled || !g_wave_running ||
        (g_wave_rate==5000U && ((sample->sequence-1U)&1U)!=0U)) { return; }
    uint32_t head=g_wave_head, backlog=head-g_wave_tail;
    if(backlog>=WAVE_RING_CAPACITY) { ++g_wave_stats.dropped; return; }
    g_wave_rows[head&(WAVE_RING_CAPACITY-1U)]=*sample;
    wave_barrier(); g_wave_head=head+1U; ++g_wave_stats.captured;
    if(backlog+1U>g_wave_stats.backlog_max) { g_wave_stats.backlog_max=backlog+1U; }
}
/** @brief 见wave_stream.h。 */
void WAVE_FAST wave_stream_capture_timing(uint32_t cycles)
{ if(cycles>g_wave_stats.capture_max_cycles) { g_wave_stats.capture_max_cycles=cycles; } }
/** @brief 见wave_stream.h。 */
void WAVE_FAST wave_stream_end(void)
{ g_wave_running=false; wave_barrier(); g_wave_end_pending=g_wave_enabled; }
/** @brief 见wave_stream.h。 */
void wave_stream_status(uint32_t ticks, float rpm, int32_t target_milli,
    uint16_t bus_mv, bool run, uint8_t stage, bool valid)
{
    if(!g_wave_enabled || (!g_wave_running && !g_wave_end_pending)) { return; }
    if((int32_t)(ticks-g_wave_status_due)<0) { return; }
    g_wave_status_due=ticks+WAVE_CLOCK_HZ/50U;
    g_wave_status_tick=ticks;
    g_wave_status_flags=(g_wave_test?WAVE_FLAG_TEST:0U)|(valid?WAVE_FLAG_VALID:0U);
    put32(g_wave_status,(uint32_t)(int32_t)(rpm*1000.0f));
    put32(g_wave_status+4,(uint32_t)target_milli); put16(g_wave_status+8,bus_mv);
    g_wave_status[10]=run?1U:0U; g_wave_status[11]=stage;
    put32(g_wave_status+12,ticks); g_wave_status_pending=true;
}
/** @brief 见wave_stream.h。 */
wave_stats_t wave_stream_stats(void) { return g_wave_stats; }

/** @brief 见wave_stream.h。 */
size_t WAVE_SMALL wave_stream_next(uint8_t *out, size_t size)
{
    if(size<WAVE_PACKET_CAPACITY || !g_wave_enabled) { return 0U; }
    uint32_t tail=g_wave_tail, available=g_wave_head-tail;
    wave_barrier();
    size_t length=0U;
    uint16_t flags=g_wave_test?WAVE_FLAG_TEST:WAVE_FLAG_VALID;
    if(available>=WAVE_BLOCK_SAMPLES || (available>0U && !g_wave_running))
    {
        wave_sample_t first=g_wave_rows[tail&(WAVE_RING_CAPACITY-1U)];
        uint32_t stride=g_wave_rate==5000U?2U:1U;
        uint16_t count=0U;
        while(count<WAVE_BLOCK_SAMPLES && count<available)
        {
            wave_sample_t item=g_wave_rows[(tail+count)&(WAVE_RING_CAPACITY-1U)];
            if(item.sequence!=first.sequence+(uint32_t)count*stride ||
                item.ticks!=first.ticks+(uint32_t)count*stride*17000U) { break; }
            float scale=(float)item.vdda_mv*(.001f/4095.0f)*(float)g_wave_units;
            float values[7]={((int32_t)item.raw[0]-g_wave_zero[0])*scale,
                ((int32_t)item.raw[1]-g_wave_zero[1])*scale,
                ((int32_t)item.raw[2]-g_wave_zero[2])*scale,
                item.id_a,item.iq_a,item.reference_d_a,item.reference_q_a};
            for(unsigned channel=0;channel<7U;++channel)
            { put16(g_wave_packet+28U+(size_t)count*14U+channel*2U,(uint16_t)wave_q13(values[channel],&flags)); }
            g_wave_last_tick=item.ticks; ++count;
        }
        /* 遇到缺口只发布此前连续部分，下一块从真实的新序号开始。 */
        wave_header(1U,flags,count,(uint16_t)(count*14U),first.sequence,first.ticks,stride*17000U);
        wave_barrier(); g_wave_tail=tail+count; g_wave_stats.sent+=count;
        length=28U+(size_t)count*14U;
    }
    else if(g_wave_status_pending)
    {
        wave_header(2U,g_wave_status_flags,1U,16U,0U,g_wave_status_tick,WAVE_CLOCK_HZ/50U);
        memcpy(g_wave_packet+28,g_wave_status,16U); g_wave_status_pending=false; length=44U;
    }
    else if(g_wave_end_pending && available==0U)
    {
        wave_stats_t stats=g_wave_stats;
        wave_header(3U,flags,0U,32U,0U,g_wave_last_tick,0U);
        const uint32_t values[8]={stats.captured,stats.sent,stats.dropped,stats.cancelled,
            stats.clipped,stats.packets,stats.backlog_max,stats.capture_max_cycles};
        for(unsigned i=0;i<8U;++i) { put32(g_wave_packet+28U+i*4U,values[i]); }
        g_wave_end_pending=false; length=60U;
    }
    if(length==0U) { return 0U; }
    put32(g_wave_packet+length,wave_crc(g_wave_packet,length));
    return wave_encode(out,size,length+4U);
}
/** @brief 见wave_stream.h。 */
void wave_stream_test_start(uint32_t now_tick, uint32_t now_ms, uint32_t duration_ms)
{
    const uint16_t zero[3]={2048U,2048U,2048U};
    wave_stream_begin(zero,1U,true); wave_stream_enable(true);
    g_wave_test_started=now_ms; g_wave_test_duration=duration_ms;
    (void)now_tick; g_wave_test_sequence=0U;
}
/** @brief 见wave_stream.h。 */
bool wave_stream_testing(void) { return g_wave_test && g_wave_running; }
/** @brief 见wave_stream.h。 */
void wave_stream_test_service(uint32_t now_tick, uint32_t now_ms)
{
    if(!wave_stream_testing()) { return; }
    /* 停机WFI会暂停DWT，测试源按系统毫秒推进；标记TEST，不冒充硬件采样。 */
    (void)now_tick;
    uint32_t period=17000U;
    uint32_t limit=g_wave_test_duration*10U;
    uint32_t elapsed=now_ms-g_wave_test_started;
    uint32_t due=(elapsed<g_wave_test_duration?elapsed:g_wave_test_duration)*10U;
    /* 每次最多64项，主循环不因通信测试忙等。 */
    for(unsigned n=0;n<64U && g_wave_test_sequence<due;++n)
    {
        ++g_wave_test_sequence;
        wave_sample_t item={.sequence=g_wave_test_sequence,.ticks=g_wave_test_sequence*period,
            .raw={2049U,2047U,2048U},.vdda_mv=3300U,.id_a=.01f,.iq_a=-.01f};
        wave_stream_capture(&item);
    }
    wave_stream_status(g_wave_test_sequence*period,0.0f,0,0U,false,0U,false);
    if(g_wave_test_sequence>=limit || now_ms-g_wave_test_started>g_wave_test_duration+100U)
    { wave_stream_end(); }
}
