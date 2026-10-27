#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* runner从正式算法提取实际私有函数，不维护一份优化实现副本。 */
#include "sensorless_math_under_test.inc"
/* 断言数量和固定随机种子，仅用于主机回归。 */
static uint32_t checks, seed = 0x474132U;

/** @brief 生成确定性边界组合，不访问设备。
 * @return 32位伪随机输入。 */
static uint32_t
random_word(void)
{
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;
    return seed;
}

/** @brief 将优化后的实际函数与原数学表达式比较。
 * @param numerator 分子。
 * @param denominator 非零分母。 */
static void
verify(uint64_t numerator, uint64_t denominator)
{
    uint32_t expected = (uint32_t)(numerator / denominator);
    if (divide_wide(numerator, denominator) != expected)
    {
        fprintf(stderr, "sensorless arithmetic mismatch\n");
        exit(1);
    }
    ++checks;
}

/** @brief 覆盖32位分界、宽分母、商截断和实际ADC/时间/角度范围。
 * @return 全部通过为0。 */
int
main(void)
{
    const uint64_t values[] = {0U, 1U, 50U, 4095U, UINT32_MAX - 1ULL, UINT32_MAX,
        UINT32_MAX + 1ULL, UINT32_MAX + 2ULL, UINT64_MAX - 1ULL, UINT64_MAX};
    const uint64_t divisors[] = {1U, 2U, 60U, 600U, 4095U, 1000000U,
        UINT32_MAX, UINT32_MAX + 1ULL, UINT64_MAX};
    for (unsigned i = 0; i < sizeof(values)/sizeof(values[0]); ++i)
    {
        for (unsigned j = 0; j < sizeof(divisors)/sizeof(divisors[0]); ++j)
        {
            verify(values[i], divisors[j]);
        }
    }
    for (unsigned i = 0; i < 10000U; ++i)
    {
        uint32_t a = random_word(), b = random_word();
        uint64_t numerator = (uint64_t)a * b;
        verify(numerator, (uint64_t)random_word() + 1U);
        verify(numerator, ((uint64_t)random_word() << 32) + random_word() + 1ULL);
        verify(a, (uint64_t)b + 1U);
        /* 合法ADC码之间的插值分子及实际最长扇区角度换算。 */
        verify((uint64_t)(a % 1001U) * (b % 4096U), (b % 4095U) + 1U);
        verify((uint64_t)(a % 1666667U) * ((b % 401U) + 100U), 600U);
    }
    printf("sensorless arithmetic checks: %lu\n", (unsigned long)checks);
    return 0;
}
