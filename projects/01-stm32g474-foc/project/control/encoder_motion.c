#include "encoder_motion.h"
#include "encoder_six_step.h"
#include "motor_config.h"
#include <string.h>
#include <limits.h>

/** @brief 发布原始展开位置和时间；只复制整数，不计算或平滑转速。 @param s 实例。 */
static void MOTOR_FAST_CODE
encoder_motion_publish(encoder_motion_t *s)
{
    encoder_motion_sample_t *p=&s->sample[s->total%ENCODER_MOTION_CAPACITY];
    p->time_lo=(uint32_t)s->elapsed;
    p->time_hi=(uint32_t)(s->elapsed>>32);
    p->position_lo=(uint32_t)s->position;
    p->position_hi=(uint32_t)((uint64_t)s->position>>32);
    p->raw=s->last_raw;
    p->min_delta=s->min_delta;
    p->max_delta=s->max_delta;
    ++s->total;
    s->min_delta=INT32_MAX;
    s->max_delta=INT32_MIN;
}

/** @brief 见encoder_motion.h。 */
void
encoder_motion_init(encoder_motion_t *s, int32_t direction, uint32_t tick_hz)
{
    memset(s,0,sizeof(*s));
    s->direction=direction;
    s->tick_hz=tick_hz;
}

/** @brief 见encoder_motion.h；原运行反馈已逐帧校验，无重复静态参数守卫。 */
void MOTOR_FAST_CODE
encoder_motion_update(encoder_motion_t *s, uint32_t raw, uint32_t tick)
{
    if(!s->initialized)
    {
        s->initialized=true;
        s->last_raw=raw;
        s->last_tick=tick;
        s->position=(int64_t)raw*s->direction;
        encoder_motion_publish(s);
        return;
    }
    uint32_t dt=tick-s->last_tick;
    if(dt==0U) { return; }
    if(dt>s->tick_hz/500U) { ++s->gaps; }
    int32_t delta=(int32_t)raw-(int32_t)s->last_raw;
    if(delta>(int32_t)(ENCODER_SIX_TURN/2U)) { delta-=(int32_t)ENCODER_SIX_TURN; }
    if(delta<-(int32_t)(ENCODER_SIX_TURN/2U)) { delta+=(int32_t)ENCODER_SIX_TURN; }
    delta*=s->direction;
    s->elapsed+=dt;
    s->position+=delta;
    s->last_tick=tick;
    s->last_raw=raw;
    if(delta<s->min_delta) { s->min_delta=delta; }
    if(delta>s->max_delta) { s->max_delta=delta; }
    if(++s->phase==20U)
    {
        s->phase=0U;
        encoder_motion_publish(s);
    }
}

/** @brief 见encoder_motion.h。 */
uint32_t
encoder_motion_read(const encoder_motion_t *s, uint32_t *first, encoder_motion_sample_t *out)
{
    uint32_t oldest=s->total>ENCODER_MOTION_CAPACITY?s->total-ENCODER_MOTION_CAPACITY:0U;
    if(*first<oldest) { *first=oldest; }
    uint32_t count=*first<s->total?s->total-*first:0U;
    if(count>ENCODER_MOTION_BATCH) { count=ENCODER_MOTION_BATCH; }
    for(uint32_t i=0;i<count;++i) { out[i]=s->sample[(*first+i)%ENCODER_MOTION_CAPACITY]; }
    return count;
}
