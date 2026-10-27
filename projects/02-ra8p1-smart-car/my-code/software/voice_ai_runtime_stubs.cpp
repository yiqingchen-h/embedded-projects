#include "hal_data.h"

#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>

/* TFLM profiler 使用的周期计数默认频率。 */
#define VOICE_AI_TFLM_TICKS_PER_SECOND  (1000000000UL)

namespace tflite
{
/**
 * @brief  返回 TFLM profiler 的 tick 频率
 *
 * @return uint32_t 每秒 tick 数
 */
uint32_t ticks_per_second(void)
{
    return VOICE_AI_TFLM_TICKS_PER_SECOND;
}

/**
 * @brief  返回当前 DWT 周期计数
 *
 * @return uint32_t 当前周期计数
 */
uint32_t GetCurrentTimeTicks(void)
{
    static bool s_cycle_counter_ready = false;

    if (!s_cycle_counter_ready)
    {
#if defined(PMU) && defined(PMU_CTRL_ENABLE_Msk) && defined(PMU_CTRL_CYCCNT_RESET_Msk) && defined(PMU_CNTENSET_CCNTR_ENABLE_Msk) && defined(DCB) && defined(DCB_DEMCR_TRCENA_Msk)
        PMU->CTRL |= PMU_CTRL_ENABLE_Msk;
        DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
        PMU->CTRL |= PMU_CTRL_CYCCNT_RESET_Msk;
        PMU->CNTENSET = PMU_CNTENSET_CCNTR_ENABLE_Msk;
        s_cycle_counter_ready = true;
#endif
    }

#if defined(PMU)
    return PMU->CCNTR;
#else
    return 0U;
#endif
}
}

extern "C"
{
/**
 * @brief  计算宽字符串长度
 *
 * @param p_text 宽字符串指针
 * @return size_t 字符数量
 */
size_t wcslen(const wchar_t *p_text)
{
    size_t length = 0U;

    if (NULL == p_text)
    {
        return 0U;
    }

    while (L'\0' != p_text[length])
    {
        length++;
    }

    return length;
}

/**
 * @brief  在宽字符缓冲中查找字符
 *
 * @param p_text 宽字符缓冲
 * @param value  待查找字符
 * @param count  最大检查数量
 * @return wchar_t* 找到的字符地址，NULL 表示未找到
 */
wchar_t *wmemchr(const wchar_t *p_text, wchar_t value, size_t count)
{
    size_t index = 0U;

    if (NULL == p_text)
    {
        return NULL;
    }

    for (index = 0U; index < count; index++)
    {
        if (value == p_text[index])
        {
            return const_cast<wchar_t *>(&p_text[index]);
        }
    }

    return NULL;
}

/**
 * @brief  比较两个宽字符缓冲
 *
 * @param p_left  左侧缓冲
 * @param p_right 右侧缓冲
 * @param count   比较字符数量
 * @return int 0 表示相等，负数或正数表示大小关系
 */
int wmemcmp(const wchar_t *p_left, const wchar_t *p_right, size_t count)
{
    size_t index = 0U;

    for (index = 0U; index < count; index++)
    {
        if (p_left[index] != p_right[index])
        {
            return (p_left[index] < p_right[index]) ? -1 : 1;
        }
    }

    return 0;
}

/**
 * @brief  跳过宽字符串前导空白
 *
 * @param p_text 输入宽字符串
 * @return const wchar_t* 第一个非空白字符
 */
static const wchar_t *voice_ai_wide_skip_spaces(const wchar_t *p_text)
{
    while ((L' ' == *p_text) || (L'\t' == *p_text) || (L'\n' == *p_text) || (L'\r' == *p_text))
    {
        p_text++;
    }

    return p_text;
}

/**
 * @brief  将一个宽字符转换为数字值
 *
 * @param ch 输入宽字符
 * @return int 数字值，-1 表示非法
 */
static int voice_ai_wide_digit_value(wchar_t ch)
{
    if ((ch >= L'0') && (ch <= L'9'))
    {
        return (int)(ch - L'0');
    }

    if ((ch >= L'a') && (ch <= L'z'))
    {
        return (int)(ch - L'a') + 10;
    }

    if ((ch >= L'A') && (ch <= L'Z'))
    {
        return (int)(ch - L'A') + 10;
    }

    return -1;
}

/**
 * @brief  解析无符号宽字符串整数
 *
 * @param p_text 输入宽字符串
 * @param pp_end 结束位置输出
 * @param base   进制，0 表示自动识别
 * @return unsigned long long 解析结果
 */
static unsigned long long voice_ai_wcstoull_impl(const wchar_t *p_text, wchar_t **pp_end, int base)
{
    const wchar_t *p_cursor = p_text;
    unsigned long long value = 0ULL;
    int digit = 0;

    if (NULL == p_text)
    {
        if (NULL != pp_end)
        {
            *pp_end = NULL;
        }

        return 0ULL;
    }

    p_cursor = voice_ai_wide_skip_spaces(p_cursor);
    if (L'+' == *p_cursor)
    {
        p_cursor++;
    }

    if (0 == base)
    {
        base = 10;
        if (L'0' == p_cursor[0])
        {
            base = 8;
            if ((L'x' == p_cursor[1]) || (L'X' == p_cursor[1]))
            {
                base = 16;
                p_cursor += 2;
            }
        }
    }

    while (true)
    {
        digit = voice_ai_wide_digit_value(*p_cursor);
        if ((digit < 0) || (digit >= base))
        {
            break;
        }

        value = (value * (unsigned long long)base) + (unsigned long long)digit;
        p_cursor++;
    }

    if (NULL != pp_end)
    {
        *pp_end = const_cast<wchar_t *>(p_cursor);
    }

    return value;
}

/**
 * @brief  解析宽字符串为 unsigned long
 *
 * @param p_text 输入宽字符串
 * @param pp_end 结束位置输出
 * @param base   进制
 * @return unsigned long 解析结果
 */
unsigned long wcstoul(const wchar_t *p_text, wchar_t **pp_end, int base)
{
    return (unsigned long)voice_ai_wcstoull_impl(p_text, pp_end, base);
}

/**
 * @brief  解析宽字符串为 unsigned long long
 *
 * @param p_text 输入宽字符串
 * @param pp_end 结束位置输出
 * @param base   进制
 * @return unsigned long long 解析结果
 */
unsigned long long wcstoull(const wchar_t *p_text, wchar_t **pp_end, int base)
{
    return voice_ai_wcstoull_impl(p_text, pp_end, base);
}

/**
 * @brief  解析宽字符串为 long
 *
 * @param p_text 输入宽字符串
 * @param pp_end 结束位置输出
 * @param base   进制
 * @return long 解析结果
 */
long wcstol(const wchar_t *p_text, wchar_t **pp_end, int base)
{
    const wchar_t *p_cursor = p_text;
    bool negative = false;
    unsigned long long value = 0ULL;

    if (NULL == p_text)
    {
        if (NULL != pp_end)
        {
            *pp_end = NULL;
        }

        return 0L;
    }

    p_cursor = voice_ai_wide_skip_spaces(p_cursor);
    if (L'-' == *p_cursor)
    {
        negative = true;
        p_cursor++;
    }

    value = voice_ai_wcstoull_impl(p_cursor, pp_end, base);
    return negative ? -(long)value : (long)value;
}

/**
 * @brief  解析宽字符串为 long long
 *
 * @param p_text 输入宽字符串
 * @param pp_end 结束位置输出
 * @param base   进制
 * @return long long 解析结果
 */
long long wcstoll(const wchar_t *p_text, wchar_t **pp_end, int base)
{
    return (long long)wcstol(p_text, pp_end, base);
}

/**
 * @brief  microlib 宽字符格式化兼容入口
 *
 * @param p_buffer 输出缓冲
 * @param length   输出缓冲字符数量
 * @param p_format 格式字符串
 * @return int 写入字符数量
 */
int __2swprintf(wchar_t *p_buffer, size_t length, const wchar_t *p_format, ...)
{
    (void)p_format;

    if ((NULL != p_buffer) && (0U < length))
    {
        p_buffer[0] = L'\0';
    }

    return 0;
}

/**
 * @brief  microlib 宽字符串 double 解析兼容入口
 *
 * @param p_text 输入宽字符串
 * @param pp_end 结束位置输出
 * @param flags  库内部标志，未使用
 * @return double 解析结果
 */
double __wcstod_int(const wchar_t *p_text, wchar_t **pp_end, int flags)
{
    (void)flags;

    if (NULL != pp_end)
    {
        *pp_end = const_cast<wchar_t *>(p_text);
    }

    return 0.0;
}

/**
 * @brief  microlib 宽字符串 float 解析兼容入口
 *
 * @param p_text 输入宽字符串
 * @param pp_end 结束位置输出
 * @param flags  库内部标志，未使用
 * @return float 解析结果
 */
float __wcstof_int(const wchar_t *p_text, wchar_t **pp_end, int flags)
{
    (void)flags;

    if (NULL != pp_end)
    {
        *pp_end = const_cast<wchar_t *>(p_text);
    }

    return 0.0f;
}
}
