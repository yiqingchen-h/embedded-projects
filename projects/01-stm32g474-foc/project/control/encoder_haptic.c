#include "encoder_haptic.h"
#include "motor_config.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/* MT6835每圈原码数；整数网格避免长时间多圈角浮点累积漂移。 */
#define HAPTIC_TURN 2097152LL
/* 手感增益采用200ms线性衔接；只在控制环计算，不依赖PC刷新。 */
#define HAPTIC_TRANSITION_S .2f
/* 档位与配置外环保持1kHz，瞬时弹性/阻尼在现有5kHz反馈处校正。 */
#define HAPTIC_OUTER_S .001f
/* 沿用G档已有600k RPM/(A*s)近似计算弹性阻尼；不是独立实板惯量辨识。 */
#define HAPTIC_MODEL_B0 600000.0f
/* 软件测速采用64码可信增量门限，参考HYST=7的约11mdeg迟滞尺度；不是SPI分辨率，原始角/计数完整保留。 */
#define HAPTIC_VELOCITY_COUNTS 64
/* 对可信增量测速作2ms时间加权，平滑慢转过阈事件；不改原始角和保护速度。 */
#define HAPTIC_VELOCITY_FILTER_S .002f
/* 档位响应独立于200ms配置渐变；2A/s会削弱正常手拨的短周期吸附。 */
#define HAPTIC_DETENT_SLEW_A_S 200.0f
#define HAPTIC_DAMPED_SLEW_A_S 10.0f

/** @brief 见encoder_haptic.h。 */
bool MOTOR_BACKGROUND_CODE
encoder_haptic_valid(const encoder_haptic_config_t *c)
{
    return NULL!=c && c->kind>=ENCODER_HAPTIC_DETENT && c->kind<=ENCODER_HAPTIC_DAMPED &&
        c->detents>=2 && c->detents<=256 && c->detent_ma>=0 && c->detent_ma<=800 &&
        c->damping_ua_rpm>=0 && c->damping_ua_rpm<=10000;
}

/** @brief 见encoder_haptic.h；按完整增量时间平滑测速，停转/反向仍撤旧阻尼。 */
float MOTOR_FAST_CODE
encoder_haptic_velocity(encoder_haptic_t *s,int32_t delta,float dt)
{
    if(!s->anchored) { return 0; }
    s->velocity_delta+=delta;
    /* 驻点等待不计入无限长的下一次起步测速窗，最低有效阻尼速度仍为0.2RPM。 */
    s->velocity_elapsed_s+=dt;
    int32_t magnitude=s->velocity_delta<0?-s->velocity_delta:s->velocity_delta;
    if(magnitude>HAPTIC_VELOCITY_COUNTS)
    {
        float window=fminf(.02f,s->velocity_elapsed_s);
        float measured=s->velocity_delta*(60.0f/(float)HAPTIC_TURN)/window;
        /* 按本次完整测速时间加权，而不是按下一次未知事件间隔保持瞬时速度。 */
        s->velocity_rpm+=window/(HAPTIC_VELOCITY_FILTER_S+window)*
            (measured-s->velocity_rpm);
        s->velocity_hold_s=fmaxf(.0004f,2.0f*HAPTIC_VELOCITY_COUNTS*window/magnitude);
        s->velocity_delta=0;
        s->velocity_elapsed_s=0;
    }
    if(delta*s->rpm<0) { s->velocity_rpm=0; return 0; }
    if(s->velocity_elapsed_s>s->velocity_hold_s) { s->velocity_rpm=0; }
    return s->velocity_rpm;
}

/** @brief 见encoder_haptic.h；64位除法仅在1kHz外环跨档/改档时执行。 */
float MOTOR_FAST_CODE
encoder_haptic_tick(encoder_haptic_t *s, const encoder_haptic_config_t *c,
    int32_t delta, uint32_t tick, float dt, float rpm, float limit)
{
    bool first=!s->anchored;
    if(first) { s->anchored=true; }
    else { s->counts+=delta; }
    s->tick=tick;
    s->rpm=rpm;
    s->outer_elapsed_s+=dt;
    s->outer_phase_s+=dt;
    if(first || s->outer_phase_s>=HAPTIC_OUTER_S-.0000001f)
    {
        float outer_dt=s->outer_elapsed_s;
        s->outer_elapsed_s=0;
        /* 保留采样抖动产生的相位余量，不能每次丢掉尾数使外环退化到约833Hz。 */
        s->outer_phase_s=first?0:fmaxf(0,s->outer_phase_s-HAPTIC_OUTER_S);
        if(memcmp(c,&s->applied,sizeof(*c))!=0)
        {
            bool reanchor=c->kind!=s->applied.kind || c->detents!=s->applied.detents;
            if(reanchor)
            {
                s->anchor_counts=s->counts-s->index*HAPTIC_TURN/c->detents;
                s->center_counts=s->counts;
            }
            s->from_strength_a=reanchor?0.0f:s->strength_a;
            s->from_damping_a_rpm=s->damping_a_rpm;
            s->transition_s=0;
            s->applied=*c;
        }
        s->transition_s=fminf(HAPTIC_TRANSITION_S,s->transition_s+outer_dt);
        float blend=s->transition_s/HAPTIC_TRANSITION_S;
        s->strength_a=s->from_strength_a+blend*(c->detent_ma*.001f-s->from_strength_a);
        s->damping_a_rpm=s->from_damping_a_rpm+blend*(c->damping_ua_rpm*.000001f-s->from_damping_a_rpm);
        float error=(float)(s->counts-s->center_counts);
        float pitch=(float)HAPTIC_TURN/c->detents;
        if(c->kind==ENCODER_HAPTIC_DETENT && fabsf(error)>.55f*pitch)
        {
            /* 一拍可跨多档，使用O(1)计算，不以循环追赶高速手拨。 */
            int32_t jump=(int32_t)(error/pitch+(error>0?.45f:-.45f));
            s->index+=jump;
            s->center_counts=s->anchor_counts+s->index*HAPTIC_TURN/c->detents;
        }
        /* 参考SmartKnob按档宽设置D项：细档需要独立弹性阻尼，不依赖用户粘性设置。
         * 按本板已有加速度模型归一为A/RPM；中心死区内不施加这项阻尼。 */
        float dead=fminf((float)HAPTIC_TURN/360.0f,pitch*.2f);
        float stiffness=s->strength_a*(float)HAPTIC_TURN/(6.28318530718f*(.5f*pitch-dead));
        s->spring_damping_a_rpm=.146607657168f*sqrtf(stiffness/(HAPTIC_MODEL_B0*.10471975512f));
    }
    /* 外环配置下一拍才生效；快反馈始终使用同一套已经消费的配置。 */
    c=&s->applied;
    float error=(float)(s->counts-s->center_counts);
    float pitch=(float)HAPTIC_TURN/c->detents;
    float requested=0;
    float spring_fraction=0;
    if(c->kind==ENCODER_HAPTIC_DETENT)
    {
        /* 档内死区参考SmartKnob（Scott Bezek，Apache-2.0）：20%档距且不超过1度。
         * 本实现使用真实Iq闭环与整数机械网格，不移植其电压增益或静止自动漂移中心。
         * 原始参考及许可见docs/参考工程/smartknob_local_20261024。 */
        float dead=fminf((float)HAPTIC_TURN/360.0f,pitch*.2f);
        float spring=fmaxf(0,fabsf(error)-dead)/(.5f*pitch-dead);
        spring_fraction=fminf(1,spring*5.0f);
        requested=-copysignf(fminf(1,spring)*s->strength_a,error);
    }
    /* 阻尼独立按10A/s建立；反向/停止立即撤旧转矩，两种手感均不能沿弹性快通道放大。
     * 上游按真实位移/时间作控制估速，避免复用速度PLL延迟形成高频正反馈。 */
    float gain=s->damping_a_rpm+spring_fraction*s->spring_damping_a_rpm;
    float damping=-copysignf(fmaxf(0,fabsf(rpm)-.2f)*gain,rpm);
    damping=fmaxf(-limit,fminf(limit,damping));
    if(fabsf(rpm)<=.2f || s->damping_iq_a*damping<0)
    { s->damping_iq_a=0; }
    else
    {
        float damping_step=HAPTIC_DAMPED_SLEW_A_S*dt;
        s->damping_iq_a+=fmaxf(-damping_step,fminf(damping_step,damping-s->damping_iq_a));
    }
    requested+=s->damping_iq_a;
    requested=fmaxf(-limit,fminf(limit,requested));
    float previous=s->iq_a;
    /* 连续阻尼反向先用一拍撤旧转矩；档位正反弹性转矩则连续穿零，保留真实跨档力形。 */
    if(c->kind==ENCODER_HAPTIC_DAMPED && previous*requested<0) { s->iq_a=0; return 0; }
    float slew=s->transition_s<HAPTIC_TRANSITION_S?2.0f:
        c->kind==ENCODER_HAPTIC_DETENT?HAPTIC_DETENT_SLEW_A_S:HAPTIC_DAMPED_SLEW_A_S;
    float step=slew*dt;
    float next=previous+fmaxf(-step,fminf(step,requested-previous));
    /* 快速反拖或停止时立即撤掉会助推运动的旧阻尼。 */
    if(c->kind==ENCODER_HAPTIC_DAMPED && (fabsf(rpm)<=.2f || next*rpm>0)) { next=0; }
    s->iq_a=next;
    return next;
}
