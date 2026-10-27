#include "foc_voltage.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief 比较周期平均模型的浮点计算结果。
 * @param actual 实际值。 @param expected 理论值。 */
static void
near(float actual, float expected) { assert(fabsf(actual-expected)<0.00001f); }

/** @brief 制造具有已知电压的最终PWM命令。
 * @param seq 实际序号。 @param ticks 实际提交tick。 @param a A比较值。
 * @return 已准备但尚未提交的命令。 */
static foc_voltage_command_t
command(uint32_t seq,uint32_t ticks,uint16_t a)
{
    foc_voltage_command_t c={.sequence=seq,.applied_ticks=ticks,.ccr={a,2000U,3000U},.bus_v=12.0f};
    assert(foc_voltage_prepare(&c,8500U));
    return c;
}

/** @brief 运行真实历史模块，覆盖未来请求隔离和跨回绕因果关系。
 * @return 所有断言通过返回0。 */
int
main(void)
{
    foc_voltage_history_t h={0};
    foc_voltage_interval_t r;
    foc_voltage_command_t a=command(10U,0U,1000U);
    near(a.alpha_v,-12.0f*3000.0f/(3.0f*8500.0f));
    near(a.beta_v,-12.0f*1000.0f/(sqrtf(3.0f)*8500.0f));
    assert(!foc_voltage_average(&h,100U,200U,&r));
    assert(foc_voltage_commit(&h,&a));
    assert(foc_voltage_average(&h,100U,200U,&r));
    near(r.alpha_v,a.alpha_v);
    assert(r.new_ticks==100U);
    assert(!foc_voltage_average(&h,100U,100U,&r));
    assert(!foc_voltage_commit(&h,&a));
    foc_voltage_command_t b=command(11U,17000U,5000U);
    /* 准备未来请求不能改动当前区间的观察器输入。 */
    assert(foc_voltage_average(&h,8423U,16000U,&r));
    float before=r.alpha_v;
    b.ccr[0]=7000U;
    assert(foc_voltage_prepare(&b,8500U));
    assert(foc_voltage_average(&h,8423U,16000U,&r));
    near(r.alpha_v,before);
    assert(foc_voltage_commit(&h,&b));
    assert(foc_voltage_average(&h,8423U,25423U,&r));
    assert(r.duration_ticks==17000U && r.new_ticks==8423U);
    near(r.alpha_v,(8577.0f*a.alpha_v+8423.0f*b.alpha_v)/17000.0f);
    assert(r.old_sequence==10U && r.new_sequence==11U);
    assert(foc_voltage_average(&h,17000U,17100U,&r));
    near(r.alpha_v,b.alpha_v);
    assert(foc_voltage_average(&h,16900U,17000U,&r));
    near(r.alpha_v,a.alpha_v);
    assert(r.new_ticks==0U);
    assert(!foc_voltage_average(&h,100U,16999U,&r));
    foc_voltage_command_t gap=command(13U,34000U,5000U);
    assert(!foc_voltage_commit(&h,&gap));
    gap=command(12U,16000U,5000U);
    assert(!foc_voltage_commit(&h,&gap));
    memset(&h,0,sizeof(h));
    a=command(0xFFFFFFFFU,0xFFFFE000U,1000U);
    b=command(0U,0x1000U,5000U);
    assert(foc_voltage_commit(&h,&a));
    assert(foc_voltage_commit(&h,&b));
    assert(foc_voltage_average(&h,0xFFFFF000U,0x3000U,&r));
    assert(r.duration_ticks==0x4000U && r.new_ticks==0x2000U);
    near(r.alpha_v,0.5f*(a.alpha_v+b.alpha_v));
    h.count=0U;
    assert(!foc_voltage_average(&h,0U,100U,&r));
    a=command(1U,0U,1000U);a.bus_v=NAN;
    assert(!foc_voltage_prepare(&a,8500U));
    a.bus_v=12.0f;a.ccr[0]=8501U;
    assert(!foc_voltage_prepare(&a,8500U));
    puts("FOC voltage: causal requests, actual commits, interval weights, wrap and invalid history PASS");
    return 0;
}
