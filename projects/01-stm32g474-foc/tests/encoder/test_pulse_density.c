#include "six_step.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* 独立累计脉宽核算，不用算法内部余额替代输出能量检查。 */
static unsigned g_checks;
#define CHECK(condition) do { ++g_checks; if (!(condition)) { fprintf(stderr,"PDM line %d: %s\n",__LINE__,#condition); exit(1); } } while (0)

/** @brief 原模式与独立的累计整除参考逐拍一致，覆盖限流及连续PWM进出。 */
static void
test_original(void)
{
    uint32_t budget=0U,noise=0x6D2B79F5U,reference=0U;
    for (unsigned i=0U;i<100000U;++i)
    {
        uint16_t requested=(uint16_t)((i*73U)%700U);
        bool limited=i%137U==0U,expected=!limited;
        if (requested<391U)
        {
            reference+=requested;
            if (limited) { reference=0U; }
            else
            {
                expected=reference/391U!=0U;
                reference%=391U;
            }
        }
        CHECK(six_step_pulse_density(&budget,&noise,requested,391U,limited,false)==expected);
        CHECK(budget==reference);
    }
    CHECK(noise==0x6D2B79F5U);
}

/** @brief 固定/渐变幅度和极端脉宽下不预支输出，误差小于两个最短脉冲。 */
static void
test_energy(void)
{
    static const uint16_t minima[]={1U,2U,391U,646U,4250U,65535U};
    for (unsigned m=0U;m<sizeof(minima)/sizeof(minima[0]);++m)
    {
        uint16_t minimum=minima[m];
        for (unsigned mode=0U;mode<3U;++mode)
        {
            uint32_t budget=0U,noise=0x6D2B79F5U;
            uint64_t commanded=0U,emitted=0U;
            for (unsigned i=0U;i<23000U;++i)
            {
                uint16_t requested=mode==0U?1U:mode==1U?(uint16_t)(minimum-1U):
                    (uint16_t)((i*31U)%(uint32_t)minimum);
                bool limited=mode==2U && i%151U==0U;
                bool pulse=six_step_pulse_density(&budget,&noise,requested,minimum,limited,true);
                if (limited || requested==0U || requested>=minimum)
                {
                    CHECK(pulse==(!limited && requested!=0U));
                    CHECK(budget==0U);
                    commanded=emitted=0U;
                }
                else
                {
                    commanded+=requested;
                    if (pulse) { emitted+=minimum; }
                    CHECK(emitted<=commanded);
                    CHECK(commanded-emitted<2U*(uint32_t)minimum);
                    CHECK(commanded-emitted==budget);
                }
                CHECK(noise!=0U);
            }
        }
    }
    /* 有余额时限流、零指令或进入连续PWM立即清账，不能在零指令泄出旧脉冲。 */
    for (unsigned mode=0U;mode<3U;++mode)
    {
        uint32_t budget=700U,noise=1U;
        bool pulse=six_step_pulse_density(&budget,&noise,mode==0U?0U:mode==1U?20U:500U,
            391U,mode==1U,true);
        CHECK(pulse==(mode==2U));
        CHECK(budget==0U);
    }
}

/** @brief 真实2%/391tick的固定指令下比较原周期谱线；这不是实物声谱。 */
static void
test_tonal_line(void)
{
    double real[2]={0.0,0.0},imag[2]={0.0,0.0},power[2];
    unsigned pulses[2]={0U,0U};
    for (unsigned mode=0U;mode<2U;++mode)
    {
        uint32_t budget=0U,noise=0x6D2B79F5U;
        for (unsigned i=0U;i<23000U;++i)
        {
            bool pulse=six_step_pulse_density(&budget,&noise,85U,391U,false,mode!=0U);
            if (pulse)
            {
                double angle=6.2831853071795864769*(double)(i%23U)*5.0/23.0;
                real[mode]+=cos(angle);imag[mode]+=sin(angle);++pulses[mode];
            }
        }
        power[mode]=real[mode]*real[mode]+imag[mode]*imag[mode];
    }
    CHECK(pulses[0]==5000U && pulses[1]>=4998U && pulses[1]<=5000U);
    printf("PDM 4347.8Hz command line: %.2f dB, pulses %u/%u (23000 PWM periods)\n",
        10.0*log10(power[1]/power[0]),pulses[0],pulses[1]);
    /* 至少6dB下降才认为这个分散方式对原固定谱线有实际抑制。 */
    CHECK(power[1]<power[0]*0.25);
}

/** @brief 运行纯算法回归，不访问设备。 @return 全部通过返回0。 */
int
main(void)
{
    test_original();test_energy();test_tonal_line();
    printf("pulse density: %u checks passed\n",g_checks);
    return 0;
}
