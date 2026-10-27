#ifndef VOICE_TEST_VECTORS_H
#define VOICE_TEST_VECTORS_H

#include <stdint.h>
#include <stddef.h>

/* 测试样本采样率，必须与模型输入一致。 */
#define VOICE_TEST_SAMPLE_RATE_HZ 16000U

/* 测试样本采样点数，对应 1 秒音频。 */
#define VOICE_TEST_SAMPLE_COUNT   16000U

typedef struct
{
    /* 样本名称，用于日志打印。 */
    const char *name;
    /* 期望识别标签，必须与 g_voice_ai_labels 一致。 */
    const char *expected_label;
    /* int16 PCM 数据指针，指向 Flash 中的常量数组。 */
    const int16_t *pcm;
    /* PCM 采样点数，当前固定为 16000。 */
    size_t sample_count;
} voice_test_vector_t;

extern const int16_t g_voice_test_backward_0020[VOICE_TEST_SAMPLE_COUNT];
extern const int16_t g_voice_test_turn_left_0006[VOICE_TEST_SAMPLE_COUNT];

extern const voice_test_vector_t g_voice_test_vectors[];
extern const size_t g_voice_test_vector_count;

#endif /* VOICE_TEST_VECTORS_H */
