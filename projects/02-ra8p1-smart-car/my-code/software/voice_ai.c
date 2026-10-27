#include "software/voice_ai.h"

#include "FreeRTOS.h"
#include "task.h"

#include "common_data.h"
#include "hardware/Microphone/microphone.h"
#include "model.h"
#include "software/SDRAM/SDRAM.h"
#include "software/UART.h"
#include "software/takes.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* 禁用 AI 时等待后台任务释放麦克风的超时时间，单位 ms。 */
#define VOICE_AI_DISABLE_TIMEOUT_MS (500U)

/* 禁用 AI 等待轮询周期，单位 ms。 */
#define VOICE_AI_DISABLE_POLL_MS (10U)

/* 是否导出模型前 1 秒音频，1 表示启用临时调试导出。 */
#define VOICE_AI_DEBUG_EXPORT_ENABLED (0U)

/* 模型前音频调试导出最小间隔，单位 ms。 */
#define VOICE_AI_DEBUG_EXPORT_PERIOD_MS (3000U)

/* 模型输入采样率，必须与训练脚本保持一致。 */
#define VOICE_AI_MODEL_SAMPLE_RATE_HZ (16000U)

/* 模型输入采样点数，对应 1 秒音频。 */
#define VOICE_AI_MODEL_SAMPLES (16000U)

/* 语音有效段搜索窗口长度，单位秒。 */
#define VOICE_AI_SEARCH_SECONDS (3U)

/* 麦克风最近搜索窗口采样点数，使用实际 PDM 采样率。 */
#define VOICE_AI_RAW_SAMPLES (MICROPHONE_SAMPLE_RATE_HZ * VOICE_AI_SEARCH_SECONDS)

/* 搜索窗口重采样到 16kHz 后的采样点数。 */
#define VOICE_AI_SEARCH_SAMPLES (VOICE_AI_MODEL_SAMPLE_RATE_HZ * VOICE_AI_SEARCH_SECONDS)

/* STFT 窗长，单位采样点。 */
#define VOICE_AI_FRAME_LENGTH (400U)

/* STFT 帧移，单位采样点。 */
#define VOICE_AI_FRAME_STEP (160U)

/* FFT 长度，必须为 2 的幂。 */
#define VOICE_AI_FFT_LENGTH (512U)

/* FFT 正频率 bin 数量。 */
#define VOICE_AI_FFT_BINS ((VOICE_AI_FFT_LENGTH / 2U) + 1U)

/* log-mel 帧数量。 */
#define VOICE_AI_FRAME_COUNT (98U)

/* mel 滤波器数量，必须等于模型第二维。 */
#define VOICE_AI_MEL_BINS (40U)

/* 模型输入 float 数量。 */
#define VOICE_AI_MODEL_INPUT_FLOATS (VOICE_AI_FRAME_COUNT * VOICE_AI_MEL_BINS)

/* 训练端峰值归一化目标。 */
#define VOICE_AI_TARGET_PEAK (0.75f)

/* 训练端 trim_silence() 使用的有效声音阈值。 */
#define VOICE_AI_TRIM_THRESHOLD (0.01f)

/* 训练端 trim_silence() 两侧保留余量，单位采样点。 */
#define VOICE_AI_TRIM_MARGIN_SAMPLES (1200U)

/* 训练端预加重系数。 */
#define VOICE_AI_PREEMPHASIS (0.97f)

/* log 计算下限，避免 log(0)。 */
#define VOICE_AI_LOG_FLOOR (1.0e-8f)

/* 训练特征全局均值，必须与 tflite_training_report.md 一致。 */
#define VOICE_AI_FEATURE_MEAN (-1.52743685f)

/* 训练特征全局标准差，必须与 tflite_training_report.md 一致。 */
#define VOICE_AI_FEATURE_STD (3.72770362f)

/* 圆周率常量，避免依赖非标准 M_PI。 */
#define VOICE_AI_PI (3.14159265358979323846f)

/* 模型分数转定点打印时的满分值。 */
#define VOICE_AI_SCORE_MILLI (1000.0f)

/* 无效低分值，用于 TopK 初始化。 */
#define VOICE_AI_NEGATIVE_SCORE (-1000000000.0f)

/* 模型前音频调试导出标签，不参与训练标签表。 */
#define VOICE_AI_DEBUG_EXPORT_LABEL "voice_ai_debug"

/* 是否打印模型输入特征统计，1 表示启用 MCU/PC 前处理一致性诊断。 */
#define VOICE_AI_DEBUG_FEATURE_STATS_ENABLED (0U)

/* 是否打印模型输出统计，1 表示启用输出指针、cache 和标签顺序诊断。 */
#define VOICE_AI_DEBUG_OUTPUT_STATS_ENABLED (0U)

/* 是否在初始化后立即跑一次模型自测，1 表示不依赖语音触发也能验证 NPU 输出。 */
#define VOICE_AI_MODEL_SELF_TEST_ON_INIT (0U)

/* 是否在初始化后运行训练集 wav 离线自测，1 表示启用。 */
#define VOICE_AI_WAV_VECTOR_SELF_TEST_ENABLED (0U)

/* VAD 使用 10ms 帧，16kHz 下 160 点。 */
#define VOICE_AI_VAD_FRAME_SAMPLES (160U)

/* VAD 帧数量，3 秒搜索窗口对应 300 帧。 */
#define VOICE_AI_VAD_FRAME_COUNT (VOICE_AI_SEARCH_SAMPLES / VOICE_AI_VAD_FRAME_SAMPLES)

/* 固定最低 VAD 阈值，float 音频幅值。 */
/* 之前 0.020 太高，噪声底 0.001、语音幅值 0.01~0.02 的安静录音会被整段跳过。
   降为 0.005（约 -46dBFS），低于此值视为静音。 */
#define VOICE_AI_VAD_FIXED_THRESHOLD (0.005f)

/* 自适应阈值倍率：threshold=max(fixed, noise_floor*ratio)。 */
#define VOICE_AI_VAD_NOISE_RATIO (4.0f)

/* 连续超过阈值的帧数，达到后认为进入语音段。 */
#define VOICE_AI_VAD_START_FRAMES (3U)

/* 连续低于阈值的帧数，达到后认为语音结束。 */
#define VOICE_AI_VAD_END_SILENCE_FRAMES (20U)

/* 小于该帧数的 active 段认为是毛刺/风噪。 */
#define VOICE_AI_VAD_MIN_ACTIVE_FRAMES (8U)

/* 裁剪时保留语音起点前 200ms。 */
#define VOICE_AI_VAD_PRE_ROLL_FRAMES (20U)

/* 裁剪时保留语音终点后 50ms。 */
#define VOICE_AI_VAD_POST_ROLL_FRAMES (5U)

/* 没有 VAD active 时是否仍然跑最近 1 秒。0 表示静音/风噪不推理。 */
#define VOICE_AI_NO_ACTIVE_FALLBACK (0U)

/* 识别结果重复打印抑制时间，单位 ms。 */
#define VOICE_AI_DETECT_PRINT_COOLDOWN_MS (1500U)

typedef struct st_voice_ai_candidate_plan
{
    bool found_active;
    uint32_t active_start_sample;
    uint32_t active_end_sample;
    uint32_t candidate_count;
    uint32_t candidate_start_samples[VOICE_AI_MAX_CANDIDATES];
    float noise_floor;
    float threshold;
} voice_ai_candidate_plan_t;

/* mel 滤波器每个通道的起始 FFT bin。 */
static const uint16_t g_voice_ai_mel_starts[VOICE_AI_MEL_BINS] =
    {
        1U, 3U, 4U, 6U, 7U, 9U, 11U, 13U, 15U, 18U,
        20U, 23U, 25U, 28U, 31U, 35U, 38U, 42U, 46U, 50U,
        54U, 59U, 64U, 69U, 75U, 81U, 87U, 94U, 101U, 108U,
        116U, 125U, 134U, 143U, 153U, 164U, 176U, 188U, 201U, 214U};

/* mel 滤波器权重在扁平数组中的偏移。 */
static const uint16_t g_voice_ai_mel_offsets[VOICE_AI_MEL_BINS + 1U] =
    {
        0U, 2U, 4U, 6U, 8U, 11U, 14U, 17U, 21U, 25U,
        29U, 33U, 37U, 42U, 48U, 54U, 60U, 67U, 74U, 81U,
        89U, 98U, 107U, 117U, 128U, 139U, 151U, 164U, 177U, 191U,
        207U, 224U, 241U, 259U, 279U, 301U, 324U, 348U, 373U, 400U,
        429U};

/* mel 滤波器非零权重，参数由训练脚本的 make_mel_filterbank() 生成。 */
static const float g_voice_ai_mel_weights[429] =
    {
        5.00000000e-01f, 1.00000000e+00f, 1.00000000e+00f, 5.00000000e-01f, 5.00000000e-01f,
        1.00000000e+00f, 1.00000000e+00f, 5.00000000e-01f, 5.00000000e-01f, 1.00000000e+00f,
        5.00000000e-01f, 5.00000000e-01f, 1.00000000e+00f, 5.00000000e-01f, 5.00000000e-01f,
        1.00000000e+00f, 5.00000000e-01f, 5.00000000e-01f, 1.00000000e+00f, 6.66666687e-01f,
        3.33333343e-01f, 3.33333343e-01f, 6.66666687e-01f, 1.00000000e+00f, 5.00000000e-01f,
        5.00000000e-01f, 1.00000000e+00f, 6.66666687e-01f, 3.33333343e-01f, 3.33333343e-01f,
        6.66666687e-01f, 1.00000000e+00f, 5.00000000e-01f, 5.00000000e-01f, 1.00000000e+00f,
        6.66666687e-01f, 3.33333343e-01f, 3.33333343e-01f, 6.66666687e-01f, 1.00000000e+00f,
        6.66666687e-01f, 3.33333343e-01f, 3.33333343e-01f, 6.66666687e-01f, 1.00000000e+00f,
        7.50000000e-01f, 5.00000000e-01f, 2.50000000e-01f, 2.50000000e-01f, 5.00000000e-01f,
        7.50000000e-01f, 1.00000000e+00f, 6.66666687e-01f, 3.33333343e-01f, 3.33333343e-01f,
        6.66666687e-01f, 1.00000000e+00f, 7.50000000e-01f, 5.00000000e-01f, 2.50000000e-01f,
        2.50000000e-01f, 5.00000000e-01f, 7.50000000e-01f, 1.00000000e+00f, 7.50000000e-01f,
        5.00000000e-01f, 2.50000000e-01f, 2.50000000e-01f, 5.00000000e-01f, 7.50000000e-01f,
        1.00000000e+00f, 7.50000000e-01f, 5.00000000e-01f, 2.50000000e-01f, 2.50000000e-01f,
        5.00000000e-01f, 7.50000000e-01f, 1.00000000e+00f, 7.50000000e-01f, 5.00000000e-01f,
        2.50000000e-01f, 2.50000000e-01f, 5.00000000e-01f, 7.50000000e-01f, 1.00000000e+00f,
        8.00000012e-01f, 6.00000024e-01f, 4.00000006e-01f, 2.00000003e-01f, 2.00000003e-01f,
        4.00000006e-01f, 6.00000024e-01f, 8.00000012e-01f, 1.00000000e+00f, 8.00000012e-01f,
        6.00000024e-01f, 4.00000006e-01f, 2.00000003e-01f, 2.00000003e-01f, 4.00000006e-01f,
        6.00000024e-01f, 8.00000012e-01f, 1.00000000e+00f, 8.00000012e-01f, 6.00000024e-01f,
        4.00000006e-01f, 2.00000003e-01f, 2.00000003e-01f, 4.00000006e-01f, 6.00000024e-01f,
        8.00000012e-01f, 1.00000000e+00f, 8.33333313e-01f, 6.66666687e-01f, 5.00000000e-01f,
        3.33333343e-01f, 1.66666672e-01f, 1.66666672e-01f, 3.33333343e-01f, 5.00000000e-01f,
        6.66666687e-01f, 8.33333313e-01f, 1.00000000e+00f, 8.33333313e-01f, 6.66666687e-01f,
        5.00000000e-01f, 3.33333343e-01f, 1.66666672e-01f, 1.66666672e-01f, 3.33333343e-01f,
        5.00000000e-01f, 6.66666687e-01f, 8.33333313e-01f, 1.00000000e+00f, 8.33333313e-01f,
        6.66666687e-01f, 5.00000000e-01f, 3.33333343e-01f, 1.66666672e-01f, 1.66666672e-01f,
        3.33333343e-01f, 5.00000000e-01f, 6.66666687e-01f, 8.33333313e-01f, 1.00000000e+00f,
        8.57142866e-01f, 7.14285731e-01f, 5.71428597e-01f, 4.28571433e-01f, 2.85714298e-01f,
        1.42857149e-01f, 1.42857149e-01f, 2.85714298e-01f, 4.28571433e-01f, 5.71428597e-01f,
        7.14285731e-01f, 8.57142866e-01f, 1.00000000e+00f, 8.57142866e-01f, 7.14285731e-01f,
        5.71428597e-01f, 4.28571433e-01f, 2.85714298e-01f, 1.42857149e-01f, 1.42857149e-01f,
        2.85714298e-01f, 4.28571433e-01f, 5.71428597e-01f, 7.14285731e-01f, 8.57142866e-01f,
        1.00000000e+00f, 8.57142866e-01f, 7.14285731e-01f, 5.71428597e-01f, 4.28571433e-01f,
        2.85714298e-01f, 1.42857149e-01f, 1.42857149e-01f, 2.85714298e-01f, 4.28571433e-01f,
        5.71428597e-01f, 7.14285731e-01f, 8.57142866e-01f, 1.00000000e+00f, 8.75000000e-01f,
        7.50000000e-01f, 6.25000000e-01f, 5.00000000e-01f, 3.75000000e-01f, 2.50000000e-01f,
        1.25000000e-01f, 1.25000000e-01f, 2.50000000e-01f, 3.75000000e-01f, 5.00000000e-01f,
        6.25000000e-01f, 7.50000000e-01f, 8.75000000e-01f, 1.00000000e+00f, 8.88888896e-01f,
        7.77777791e-01f, 6.66666687e-01f, 5.55555582e-01f, 4.44444448e-01f, 3.33333343e-01f,
        2.22222224e-01f, 1.11111112e-01f, 1.11111112e-01f, 2.22222224e-01f, 3.33333343e-01f,
        4.44444448e-01f, 5.55555582e-01f, 6.66666687e-01f, 7.77777791e-01f, 8.88888896e-01f,
        1.00000000e+00f, 8.88888896e-01f, 7.77777791e-01f, 6.66666687e-01f, 5.55555582e-01f,
        4.44444448e-01f, 3.33333343e-01f, 2.22222224e-01f, 1.11111112e-01f, 1.11111112e-01f,
        2.22222224e-01f, 3.33333343e-01f, 4.44444448e-01f, 5.55555582e-01f, 6.66666687e-01f,
        7.77777791e-01f, 8.88888896e-01f, 1.00000000e+00f, 8.88888896e-01f, 7.77777791e-01f,
        6.66666687e-01f, 5.55555582e-01f, 4.44444448e-01f, 3.33333343e-01f, 2.22222224e-01f,
        1.11111112e-01f, 1.11111112e-01f, 2.22222224e-01f, 3.33333343e-01f, 4.44444448e-01f,
        5.55555582e-01f, 6.66666687e-01f, 7.77777791e-01f, 8.88888896e-01f, 1.00000000e+00f,
        8.99999976e-01f, 8.00000012e-01f, 6.99999988e-01f, 6.00000024e-01f, 5.00000000e-01f,
        4.00000006e-01f, 3.00000012e-01f, 2.00000003e-01f, 1.00000001e-01f, 1.00000001e-01f,
        2.00000003e-01f, 3.00000012e-01f, 4.00000006e-01f, 5.00000000e-01f, 6.00000024e-01f,
        6.99999988e-01f, 8.00000012e-01f, 8.99999976e-01f, 1.00000000e+00f, 9.09090936e-01f,
        8.18181813e-01f, 7.27272749e-01f, 6.36363626e-01f, 5.45454562e-01f, 4.54545468e-01f,
        3.63636374e-01f, 2.72727281e-01f, 1.81818187e-01f, 9.09090936e-02f, 9.09090936e-02f,
        1.81818187e-01f, 2.72727281e-01f, 3.63636374e-01f, 4.54545468e-01f, 5.45454562e-01f,
        6.36363626e-01f, 7.27272749e-01f, 8.18181813e-01f, 9.09090936e-01f, 1.00000000e+00f,
        9.16666687e-01f, 8.33333313e-01f, 7.50000000e-01f, 6.66666687e-01f, 5.83333313e-01f,
        5.00000000e-01f, 4.16666657e-01f, 3.33333343e-01f, 2.50000000e-01f, 1.66666672e-01f,
        8.33333358e-02f, 8.33333358e-02f, 1.66666672e-01f, 2.50000000e-01f, 3.33333343e-01f,
        4.16666657e-01f, 5.00000000e-01f, 5.83333313e-01f, 6.66666687e-01f, 7.50000000e-01f,
        8.33333313e-01f, 9.16666687e-01f, 1.00000000e+00f, 9.16666687e-01f, 8.33333313e-01f,
        7.50000000e-01f, 6.66666687e-01f, 5.83333313e-01f, 5.00000000e-01f, 4.16666657e-01f,
        3.33333343e-01f, 2.50000000e-01f, 1.66666672e-01f, 8.33333358e-02f, 8.33333358e-02f,
        1.66666672e-01f, 2.50000000e-01f, 3.33333343e-01f, 4.16666657e-01f, 5.00000000e-01f,
        5.83333313e-01f, 6.66666687e-01f, 7.50000000e-01f, 8.33333313e-01f, 9.16666687e-01f,
        1.00000000e+00f, 9.23076928e-01f, 8.46153855e-01f, 7.69230783e-01f, 6.92307711e-01f,
        6.15384638e-01f, 5.38461566e-01f, 4.61538464e-01f, 3.84615391e-01f, 3.07692319e-01f,
        2.30769232e-01f, 1.53846160e-01f, 7.69230798e-02f, 7.69230798e-02f, 1.53846160e-01f,
        2.30769232e-01f, 3.07692319e-01f, 3.84615391e-01f, 4.61538464e-01f, 5.38461566e-01f,
        6.15384638e-01f, 6.92307711e-01f, 7.69230783e-01f, 8.46153855e-01f, 9.23076928e-01f,
        1.00000000e+00f, 9.23076928e-01f, 8.46153855e-01f, 7.69230783e-01f, 6.92307711e-01f,
        6.15384638e-01f, 5.38461566e-01f, 4.61538464e-01f, 3.84615391e-01f, 3.07692319e-01f,
        2.30769232e-01f, 1.53846160e-01f, 7.69230798e-02f, 7.69230798e-02f, 1.53846160e-01f,
        2.30769232e-01f, 3.07692319e-01f, 3.84615391e-01f, 4.61538464e-01f, 5.38461566e-01f,
        6.15384638e-01f, 6.92307711e-01f, 7.69230783e-01f, 8.46153855e-01f, 9.23076928e-01f,
        1.00000000e+00f, 9.33333337e-01f, 8.66666675e-01f, 8.00000012e-01f, 7.33333349e-01f,
        6.66666687e-01f, 6.00000024e-01f, 5.33333361e-01f, 4.66666669e-01f, 4.00000006e-01f,
        3.33333343e-01f, 2.66666681e-01f, 2.00000003e-01f, 1.33333340e-01f, 6.66666701e-02f,
        6.66666701e-02f, 1.33333340e-01f, 2.00000003e-01f, 2.66666681e-01f, 3.33333343e-01f,
        4.00000006e-01f, 4.66666669e-01f, 5.33333361e-01f, 6.00000024e-01f, 6.66666687e-01f,
        7.33333349e-01f, 8.00000012e-01f, 8.66666675e-01f, 9.33333337e-01f, 1.00000000e+00f,
        9.33333337e-01f, 8.66666675e-01f, 8.00000012e-01f, 7.33333349e-01f, 6.66666687e-01f,
        6.00000024e-01f, 5.33333361e-01f, 4.66666669e-01f, 4.00000006e-01f, 3.33333343e-01f,
        2.66666681e-01f, 2.00000003e-01f, 1.33333340e-01f, 6.66666701e-02f};

/* 语音标签表，顺序必须与 my-code/ai/labels.txt 保持一致。 */
static const char *const g_voice_ai_labels[VOICE_AI_LABEL_COUNT] =
    {
        "wake",
        "start",
        "stop",
        "select_cube",
        "select_hippo",
        "select_cat",
        "select_stone_pillar",
        "select_capybara",
        "select_corn",
        "select_jiaotailang",
        "select_dog",
        "select_tnt",
        "unknown",
        "silence",
};

/* 模块是否已经初始化。 */
static bool g_voice_ai_initialized = false;

/* NPU 是否已经打开。 */
static bool g_voice_ai_npu_opened = false;

/* 预处理查表数据是否已经初始化。 */
static bool g_voice_ai_tables_ready = false;

/* 用户期望的 AI 启用状态。 */
static bool g_voice_ai_enabled = false;

/* AI 任务当前是否持有麦克风采集。 */
static bool g_voice_ai_microphone_started = false;

/* AI 任务当前是否正在执行前处理或推理。 */
static bool g_voice_ai_busy = false;

/* 当前 AI 模块状态。 */
static voice_ai_state_t g_voice_ai_state = VOICE_AI_STATE_STOPPED;

/* 推理序号。 */
static uint32_t g_voice_ai_sequence = 0U;

/* 推理次数统计。 */
static uint32_t g_voice_ai_inference_count = 0U;

/* 错误次数统计。 */
static uint32_t g_voice_ai_error_count = 0U;

/* 是否存在尚未消费的新结果。 */
static bool g_voice_ai_has_new = false;

/* 上一次识别结果打印时间，单位 ms，用于抑制滑窗重复日志。 */
static uint32_t g_voice_ai_last_detect_print_ms = 0U;

/* PMU 周期计数器是否已经启用。 */
static bool g_voice_ai_pmu_ready = false;

#if VOICE_AI_DEBUG_EXPORT_ENABLED
/* 上一次模型前音频调试导出时间，单位 ms。 */
static uint32_t g_voice_ai_debug_last_export_ms = 0U;
#endif

/* 最近一次推理结果。 */
static voice_ai_result_t g_voice_ai_latest_result =
    {
        .state = VOICE_AI_STATE_STOPPED,
};

/* 最近搜索窗口原始 PCM 数据缓冲，放入 SDRAM 减少片上 SRAM 占用。 */
SDRAM_SECTION_NOINIT static int16_t g_voice_ai_raw_pcm[VOICE_AI_RAW_SAMPLES];

/* 搜索窗口重采样到 16kHz 后的音频缓冲，放入 SDRAM 减少片上 SRAM 占用。 */
SDRAM_SECTION_NOINIT static float g_voice_ai_search_audio[VOICE_AI_SEARCH_SAMPLES];

/* 重采样后的 16000Hz float 音频缓冲，放入 SDRAM 减少片上 SRAM 占用。 */
SDRAM_SECTION_NOINIT static float g_voice_ai_audio[VOICE_AI_MODEL_SAMPLES];

#if VOICE_AI_DEBUG_EXPORT_ENABLED
/* 模型前音频调试导出 PCM 缓冲，放入 SDRAM 减少片上 SRAM 占用。 */
SDRAM_SECTION_NOINIT static int16_t g_voice_ai_debug_pcm[VOICE_AI_MODEL_SAMPLES];
#endif

/* VAD 每 10ms 一帧的平均绝对幅值。 */
static float g_voice_ai_vad_energy[VOICE_AI_VAD_FRAME_COUNT];

/* FFT 实部工作缓冲。 */
static float g_voice_ai_fft_real[VOICE_AI_FFT_LENGTH];

/* FFT 虚部工作缓冲。 */
static float g_voice_ai_fft_imag[VOICE_AI_FFT_LENGTH];

/* FFT 功率谱工作缓冲。 */
static float g_voice_ai_power[VOICE_AI_FFT_BINS];

/* 400 点 Hann 窗。 */
static float g_voice_ai_window[VOICE_AI_FRAME_LENGTH];

/* 512 点 FFT 旋转因子 cos 表。 */
static float g_voice_ai_fft_cos[VOICE_AI_FFT_LENGTH / 2U];

/* 512 点 FFT 旋转因子 sin 表。 */
static float g_voice_ai_fft_sin[VOICE_AI_FFT_LENGTH / 2U];

/**
 * @brief  获取标签名称
 *
 * @param index 标签索引
 * @return const char* 标签名称
 */
static const char *voice_ai_label_name_impl(uint8_t index)
{
    if (index >= VOICE_AI_LABEL_COUNT)
    {
        return "unknown";
    }

    return g_voice_ai_labels[index];
}

/**
 * @brief  设置 AI 模块状态
 *
 * @param state 新状态
 * @return void
 */
static void voice_ai_set_state(voice_ai_state_t state)
{
    taskENTER_CRITICAL();
    g_voice_ai_state = state;
    g_voice_ai_latest_result.state = state;
    taskEXIT_CRITICAL();
}

/**
 * @brief  读取启用标志
 *
 * @return bool true 表示允许后台推理
 */
static bool voice_ai_enabled_get(void)
{
    bool enabled = false;

    taskENTER_CRITICAL();
    enabled = g_voice_ai_enabled;
    taskEXIT_CRITICAL();

    return enabled;
}

/**
 * @brief  设置忙碌标志
 *
 * @param busy true 表示正在推理或前处理
 * @return void
 */
static void voice_ai_busy_set(bool busy)
{
    taskENTER_CRITICAL();
    g_voice_ai_busy = busy;
    taskEXIT_CRITICAL();
}

/**
 * @brief  初始化 PMU 周期计数器
 *
 * @return void
 */
static void voice_ai_cycle_counter_init(void)
{
    if (g_voice_ai_pmu_ready)
    {
        return;
    }

#if defined(PMU) && defined(PMU_CTRL_ENABLE_Msk) && defined(PMU_CTRL_CYCCNT_RESET_Msk) && defined(PMU_CNTENSET_CCNTR_ENABLE_Msk) && defined(DCB) && defined(DCB_DEMCR_TRCENA_Msk)
    PMU->CTRL |= PMU_CTRL_ENABLE_Msk;
    DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
    PMU->CTRL |= PMU_CTRL_CYCCNT_RESET_Msk;
    PMU->CNTENSET = PMU_CNTENSET_CCNTR_ENABLE_Msk;
    g_voice_ai_pmu_ready = true;
#endif
}

/**
 * @brief  读取 PMU 周期计数
 *
 * @return uint32_t 当前周期计数，不支持时返回 0
 */
static uint32_t voice_ai_cycle_get(void)
{
    if (!g_voice_ai_pmu_ready)
    {
        voice_ai_cycle_counter_init();
    }

#if defined(PMU)
    return PMU->CCNTR;
#else
    return 0U;
#endif
}

/**
 * @brief  初始化窗函数和 FFT 旋转因子
 *
 * @return void
 */
static void voice_ai_prepare_tables(void)
{
    float angle = 0.0f;

    if (g_voice_ai_tables_ready)
    {
        return;
    }

    for (uint32_t i = 0U; i < VOICE_AI_FRAME_LENGTH; i++)
    {
        angle = (2.0f * VOICE_AI_PI * (float)i) / (float)(VOICE_AI_FRAME_LENGTH - 1U);
        g_voice_ai_window[i] = 0.5f - (0.5f * cosf(angle));
    }

    for (uint32_t i = 0U; i < (VOICE_AI_FFT_LENGTH / 2U); i++)
    {
        angle = (2.0f * VOICE_AI_PI * (float)i) / (float)VOICE_AI_FFT_LENGTH;
        g_voice_ai_fft_cos[i] = cosf(angle);
        g_voice_ai_fft_sin[i] = sinf(angle);
    }

    g_voice_ai_tables_ready = true;
}

/**
 * @brief  执行 512 点复数 FFT
 *
 * @param p_real 实部缓冲
 * @param p_imag 虚部缓冲
 * @return void
 */
static void voice_ai_fft(float *p_real, float *p_imag)
{
    uint32_t j = 0U;
    uint32_t bit = 0U;
    uint32_t len = 0U;
    uint32_t half_len = 0U;
    uint32_t twiddle_step = 0U;
    float temp = 0.0f;
    float wr = 0.0f;
    float wi = 0.0f;
    float tr = 0.0f;
    float ti = 0.0f;

    for (uint32_t i = 1U; i < VOICE_AI_FFT_LENGTH; i++)
    {
        bit = VOICE_AI_FFT_LENGTH >> 1U;
        while (0U != (j & bit))
        {
            j ^= bit;
            bit >>= 1U;
        }
        j ^= bit;

        if (i < j)
        {
            temp = p_real[i];
            p_real[i] = p_real[j];
            p_real[j] = temp;

            temp = p_imag[i];
            p_imag[i] = p_imag[j];
            p_imag[j] = temp;
        }
    }

    for (len = 2U; len <= VOICE_AI_FFT_LENGTH; len <<= 1U)
    {
        half_len = len >> 1U;
        twiddle_step = VOICE_AI_FFT_LENGTH / len;

        for (uint32_t i = 0U; i < VOICE_AI_FFT_LENGTH; i += len)
        {
            for (uint32_t k = 0U; k < half_len; k++)
            {
                wr = g_voice_ai_fft_cos[k * twiddle_step];
                wi = -g_voice_ai_fft_sin[k * twiddle_step];
                tr = (wr * p_real[i + k + half_len]) - (wi * p_imag[i + k + half_len]);
                ti = (wr * p_imag[i + k + half_len]) + (wi * p_real[i + k + half_len]);

                p_real[i + k + half_len] = p_real[i + k] - tr;
                p_imag[i + k + half_len] = p_imag[i + k] - ti;
                p_real[i + k] += tr;
                p_imag[i + k] += ti;
            }
        }
    }
}

/**
 * @brief  将模型分数转换为千分制定点值
 *
 * @param score 模型原始分数
 * @return int32_t 0~1000 定点分数
 */
static int32_t voice_ai_score_to_milli(float score)
{
    float clamped = score;

    if (clamped < 0.0f)
    {
        clamped = 0.0f;
    }
    else if (clamped > 1.0f)
    {
        clamped = 1.0f;
    }
    else
    {
        /* 分数已在 0~1 范围内，不需要处理。 */
    }

    return (int32_t)((clamped * VOICE_AI_SCORE_MILLI) + 0.5f);
}

#if (VOICE_AI_MODEL_SELF_TEST_ON_INIT || VOICE_AI_DEBUG_FEATURE_STATS_ENABLED || VOICE_AI_DEBUG_OUTPUT_STATS_ENABLED)
/**
 * @brief  将 float 转换为带符号千分制定点值，保留诊断数据的负值和越界值。
 *
 * @param value 输入值
 * @return int32_t 带符号千分制定点值
 */
static int32_t voice_ai_signed_float_to_milli(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * VOICE_AI_SCORE_MILLI) + 0.5f);
    }

    return (int32_t)((value * VOICE_AI_SCORE_MILLI) - 0.5f);
}
#endif

/**
 * @brief  查找 TopK 输出
 *
 * @param p_scores      当前标签表对应的模型输出
 * @param p_top_indices TopK 索引输出
 * @param p_top_scores  TopK 分数输出
 * @return void
 */
static void voice_ai_find_topk(const float *p_scores, uint8_t *p_top_indices, float *p_top_scores)
{
    if ((NULL == p_scores) || (NULL == p_top_indices) || (NULL == p_top_scores))
    {
        return;
    }

    for (uint32_t i = 0U; i < VOICE_AI_TOPK_COUNT; i++)
    {
        p_top_indices[i] = 0U;
        p_top_scores[i] = VOICE_AI_NEGATIVE_SCORE;
    }

    for (uint32_t i = 0U; i < VOICE_AI_LABEL_COUNT; i++)
    {
        for (uint32_t k = 0U; k < VOICE_AI_TOPK_COUNT; k++)
        {
            if (p_scores[i] > p_top_scores[k])
            {
                for (uint32_t move = VOICE_AI_TOPK_COUNT - 1U; move > k; move--)
                {
                    p_top_scores[move] = p_top_scores[move - 1U];
                    p_top_indices[move] = p_top_indices[move - 1U];
                }
                p_top_scores[k] = p_scores[i];
                p_top_indices[k] = (uint8_t)i;
                break;
            }
        }
    }
}

/**
 * @brief  发布最新模型输出结果
 *
 * @param p_scores 模型输出分数
 * @param cycles   本次前处理和推理总周期数
 * @param candidate_count 实际候选窗口数量
 * @param p_candidate_top_indices 候选窗口 TopK 标签索引
 * @param p_candidate_top_scores  候选窗口 TopK 分数
 * @return void
 */
static void voice_ai_publish_result(const float *p_scores,
                                    uint32_t cycles,
                                    uint32_t candidate_count,
                                    const uint8_t p_candidate_top_indices[VOICE_AI_MAX_CANDIDATES][VOICE_AI_TOPK_COUNT],
                                    const float p_candidate_top_scores[VOICE_AI_MAX_CANDIDATES][VOICE_AI_TOPK_COUNT])
{
    voice_ai_result_t result =
        {
            .state = VOICE_AI_STATE_RUNNING,
        };
    uint32_t elapsed_ms = 0U;
    bool should_print = false;

    if (NULL == p_scores)
    {
        return;
    }

    if ((NULL == p_candidate_top_indices) || (NULL == p_candidate_top_scores))
    {
        return;
    }

    result.sequence = ++g_voice_ai_sequence;
    result.tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    result.cycles = cycles;
    result.inference_count = ++g_voice_ai_inference_count;
    result.error_count = g_voice_ai_error_count;
    result.candidate_count = candidate_count;
    if (result.candidate_count > VOICE_AI_MAX_CANDIDATES)
    {
        result.candidate_count = VOICE_AI_MAX_CANDIDATES;
    }

    for (uint32_t i = 0U; i < VOICE_AI_LABEL_COUNT; i++)
    {
        result.scores[i] = p_scores[i];
    }

    for (uint32_t i = 0U; i < result.candidate_count; i++)
    {
        result.candidate_top_indices[i] = p_candidate_top_indices[i][0];
        result.candidate_top_scores[i] = p_candidate_top_scores[i][0];
    }

    voice_ai_find_topk(result.scores, result.top_indices, result.top_scores);
    result.top_index = result.top_indices[0];
    result.top_score = result.top_scores[0];

    taskENTER_CRITICAL();
    g_voice_ai_latest_result = result;
    g_voice_ai_has_new = true;
    taskEXIT_CRITICAL();

    elapsed_ms = result.tick_ms - g_voice_ai_last_detect_print_ms;
    if ((0U == g_voice_ai_last_detect_print_ms) ||
        (elapsed_ms >= VOICE_AI_DETECT_PRINT_COOLDOWN_MS))
    {
        g_voice_ai_last_detect_print_ms = result.tick_ms;
        should_print = true;
    }

    /* 多候选打印已移至 voice_ai_process() 中，此处不再单独打印 */
    (void)should_print;
}

/**
 * @brief  初始化阶段运行一次模型自测
 *
 * @return void
 */
static void voice_ai_run_model_self_test(void)
{
#if VOICE_AI_MODEL_SELF_TEST_ON_INIT
    float *p_input = NULL;
    float *p_output = NULL;
    float scores[VOICE_AI_LABEL_COUNT] = {0.0f};
    uint8_t top_indices[VOICE_AI_TOPK_COUNT] = {0U};
    float top_scores[VOICE_AI_TOPK_COUNT] = {0.0f};
    uint32_t start_cycles = 0U;
    uint32_t cycles = 0U;

    p_input = GetModelInputPtr_serving_default_input_2_0();
    p_output = GetModelOutputPtr_StatefulPartitionedCall_0_70023();
    if ((NULL == p_input) || (NULL == p_output))
    {
        printf("voice_ai:model_self_test:skip null_ptr\r\n");
        return;
    }

    /* 自测只验证模型链路，真实固定 NPU 输入由 model.c 的调试宏覆盖。 */
    memset(p_input, 0, VOICE_AI_MODEL_INPUT_FLOATS * sizeof(float));

    printf("voice_ai:model_self_test:start\r\n");
    start_cycles = voice_ai_cycle_get();
    RunModel(false);
    cycles = voice_ai_cycle_get() - start_cycles;

    for (uint32_t i = 0U; i < VOICE_AI_LABEL_COUNT; i++)
    {
        scores[i] = p_output[i];
    }

    voice_ai_find_topk(scores, top_indices, top_scores);

    printf("voice_ai:model_self_test top1=%s score=%ld/1000 top3=%s=%ld,%s=%ld,%s=%ld cycles=%lu\r\n",
           voice_ai_label_name_impl(top_indices[0]),
           (long)voice_ai_score_to_milli(top_scores[0]),
           voice_ai_label_name_impl(top_indices[0]),
           (long)voice_ai_score_to_milli(top_scores[0]),
           voice_ai_label_name_impl(top_indices[1]),
           (long)voice_ai_score_to_milli(top_scores[1]),
           voice_ai_label_name_impl(top_indices[2]),
           (long)voice_ai_score_to_milli(top_scores[2]),
           (unsigned long)cycles);

    printf("voice_ai:model_self_test_scores");
    for (uint32_t i = 0U; i < VOICE_AI_LABEL_COUNT; i++)
    {
        printf(" %s=%ld",
               voice_ai_label_name_impl((uint8_t)i),
               (long)voice_ai_signed_float_to_milli(scores[i]));
    }
    printf("\r\n");
#endif
}

/**
 * @brief  对麦克风窗口做线性重采样
 *
 * @param p_src            原始 PCM16 数据
 * @param src_samples      原始采样点数
 * @param p_dest           16000Hz float 输出
 * @param dest_samples     输出采样点数
 * @return void
 */
static void voice_ai_resample_to_model_rate(const int16_t *p_src,
                                            uint32_t src_samples,
                                            float *p_dest,
                                            uint32_t dest_samples)
{
    float src_pos = 0.0f;
    float src_step = 0.0f;
    uint32_t index = 0U;
    float frac = 0.0f;
    float s0 = 0.0f;
    float s1 = 0.0f;

    if ((NULL == p_src) || (NULL == p_dest) || (src_samples < 2U) || (dest_samples < 2U))
    {
        return;
    }

    src_step = (float)(src_samples - 1U) / (float)(dest_samples - 1U);
    for (uint32_t i = 0U; i < dest_samples; i++)
    {
        src_pos = (float)i * src_step;
        index = (uint32_t)src_pos;
        if (index >= (src_samples - 1U))
        {
            index = src_samples - 2U;
            frac = 1.0f;
        }
        else
        {
            frac = src_pos - (float)index;
        }

        s0 = (float)p_src[index] / 32767.0f;
        s1 = (float)p_src[index + 1U] / 32767.0f;
        p_dest[i] = s0 + ((s1 - s0) * frac);
    }
}

/**
 * @brief  将窗口起点限制到搜索窗口内
 *
 * @param start_sample  候选起点，允许为负
 * @param src_samples   搜索窗口采样点数
 * @param window_samples 模型窗口采样点数
 * @return uint32_t 限制后的起点
 */
static uint32_t voice_ai_clamp_window_start(int32_t start_sample,
                                            uint32_t src_samples,
                                            uint32_t window_samples)
{
    uint32_t max_start = 0U;

    if (src_samples > window_samples)
    {
        max_start = src_samples - window_samples;
    }

    if (start_sample <= 0)
    {
        return 0U;
    }

    if ((uint32_t)start_sample > max_start)
    {
        return max_start;
    }

    return (uint32_t)start_sample;
}

/**
 * @brief  向候选计划中追加一个 1 秒窗口，自动限幅
 *
 * @param p_plan      候选计划
 * @param start_sample 候选窗口起点
 * @param src_samples 搜索窗口采样点数
 * @return void
 */
static void voice_ai_add_candidate(voice_ai_candidate_plan_t *p_plan,
                                   int32_t start_sample,
                                   uint32_t src_samples)
{
    uint32_t clamped_start = 0U;

    if (NULL == p_plan)
    {
        return;
    }

    if (p_plan->candidate_count >= VOICE_AI_MAX_CANDIDATES)
    {
        return;
    }

    clamped_start = voice_ai_clamp_window_start(start_sample, src_samples, VOICE_AI_MODEL_SAMPLES);

    p_plan->candidate_start_samples[p_plan->candidate_count] = clamped_start;
    p_plan->candidate_count++;
}

/**
 * @brief  计算 10ms 帧平均绝对幅值并估计噪声底
 *
 * @param p_audio 16kHz 搜索窗口音频
 * @param samples 搜索窗口采样点数
 * @param p_noise_floor 噪声底输出
 * @return uint32_t 有效帧数量
 */
static uint32_t voice_ai_compute_vad_energy(const float *p_audio,
                                            uint32_t samples,
                                            float *p_noise_floor)
{
    uint32_t frame_count = 0U;
    float total = 0.0f;
    float mean_all = 0.0f;
    float low_sum = 0.0f;
    uint32_t low_count = 0U;

    if ((NULL == p_audio) || (NULL == p_noise_floor))
    {
        return 0U;
    }

    frame_count = samples / VOICE_AI_VAD_FRAME_SAMPLES;
    if (frame_count > VOICE_AI_VAD_FRAME_COUNT)
    {
        frame_count = VOICE_AI_VAD_FRAME_COUNT;
    }

    for (uint32_t frame = 0U; frame < frame_count; frame++)
    {
        float sum_abs = 0.0f;
        uint32_t offset = frame * VOICE_AI_VAD_FRAME_SAMPLES;

        for (uint32_t i = 0U; i < VOICE_AI_VAD_FRAME_SAMPLES; i++)
        {
            sum_abs += fabsf(p_audio[offset + i]);
        }

        g_voice_ai_vad_energy[frame] = sum_abs / (float)VOICE_AI_VAD_FRAME_SAMPLES;
        total += g_voice_ai_vad_energy[frame];
    }

    if (0U == frame_count)
    {
        *p_noise_floor = 0.0f;
        return 0U;
    }

    mean_all = total / (float)frame_count;

    for (uint32_t frame = 0U; frame < frame_count; frame++)
    {
        if (g_voice_ai_vad_energy[frame] <= mean_all)
        {
            low_sum += g_voice_ai_vad_energy[frame];
            low_count++;
        }
    }

    if (low_count > 0U)
    {
        *p_noise_floor = low_sum / (float)low_count;
    }
    else
    {
        *p_noise_floor = mean_all;
    }

    return frame_count;
}

/**
 * @brief  用帧级 VAD 从 3 秒搜索窗口构造 5 个 1 秒候选窗口
 *
 * @param p_audio 16kHz 搜索窗口音频
 * @param samples 搜索窗口采样点数
 * @param p_plan  候选计划输出
 * @return bool true 表示至少有一个候选窗口
 */
static bool voice_ai_build_candidate_plan(const float *p_audio,
                                          uint32_t samples,
                                          voice_ai_candidate_plan_t *p_plan)
{
    uint32_t frame_count = 0U;
    bool in_active = false;
    bool selected = false;
    uint32_t above_count = 0U;
    uint32_t silence_count = 0U;
    uint32_t segment_start_frame = 0U;
    uint32_t last_active_frame = 0U;
    uint32_t selected_start_frame = 0U;
    uint32_t selected_end_frame = 0U;
    float threshold = 0.0f;

    if ((NULL == p_audio) || (NULL == p_plan) || (samples < VOICE_AI_MODEL_SAMPLES))
    {
        return false;
    }

    memset(p_plan, 0, sizeof(*p_plan));

    frame_count = voice_ai_compute_vad_energy(p_audio, samples, &p_plan->noise_floor);
    if (0U == frame_count)
    {
        return false;
    }

    threshold = p_plan->noise_floor * VOICE_AI_VAD_NOISE_RATIO;
    if (threshold < VOICE_AI_VAD_FIXED_THRESHOLD)
    {
        threshold = VOICE_AI_VAD_FIXED_THRESHOLD;
    }
    p_plan->threshold = threshold;

    for (uint32_t frame = 0U; frame < frame_count; frame++)
    {
        bool active = (g_voice_ai_vad_energy[frame] > threshold);

        if (!in_active)
        {
            if (active)
            {
                above_count++;
                if (above_count >= VOICE_AI_VAD_START_FRAMES)
                {
                    in_active = true;
                    segment_start_frame = frame + 1U - VOICE_AI_VAD_START_FRAMES;
                    last_active_frame = frame;
                    silence_count = 0U;
                }
            }
            else
            {
                above_count = 0U;
            }
        }
        else
        {
            if (active)
            {
                last_active_frame = frame;
                silence_count = 0U;
            }
            else
            {
                silence_count++;
                if (silence_count >= VOICE_AI_VAD_END_SILENCE_FRAMES)
                {
                    uint32_t segment_end_frame = last_active_frame + 1U;
                    if ((segment_end_frame > segment_start_frame) &&
                        ((segment_end_frame - segment_start_frame) >= VOICE_AI_VAD_MIN_ACTIVE_FRAMES))
                    {
                        selected = true;
                        selected_start_frame = segment_start_frame;
                        selected_end_frame = segment_end_frame;
                    }

                    in_active = false;
                    above_count = 0U;
                    silence_count = 0U;
                }
            }
        }
    }

    if (in_active)
    {
        uint32_t segment_end_frame = last_active_frame + 1U;
        if ((segment_end_frame > segment_start_frame) &&
            ((segment_end_frame - segment_start_frame) >= VOICE_AI_VAD_MIN_ACTIVE_FRAMES))
        {
            selected = true;
            selected_start_frame = segment_start_frame;
            selected_end_frame = segment_end_frame;
        }
    }

    if (!selected)
    {
#if VOICE_AI_NO_ACTIVE_FALLBACK
        p_plan->found_active = false;
        voice_ai_add_candidate(p_plan,
                               (int32_t)(samples - VOICE_AI_MODEL_SAMPLES),
                               samples);
        return (p_plan->candidate_count > 0U);
#else
        return false;
#endif
    }

    p_plan->found_active = true;

    if (selected_start_frame > VOICE_AI_VAD_PRE_ROLL_FRAMES)
    {
        selected_start_frame -= VOICE_AI_VAD_PRE_ROLL_FRAMES;
    }
    else
    {
        selected_start_frame = 0U;
    }

    selected_end_frame += VOICE_AI_VAD_POST_ROLL_FRAMES;
    if (selected_end_frame > frame_count)
    {
        selected_end_frame = frame_count;
    }

    p_plan->active_start_sample = selected_start_frame * VOICE_AI_VAD_FRAME_SAMPLES;
    p_plan->active_end_sample = selected_end_frame * VOICE_AI_VAD_FRAME_SAMPLES;
    if (p_plan->active_end_sample > samples)
    {
        p_plan->active_end_sample = samples;
    }

    if (p_plan->active_end_sample <= p_plan->active_start_sample)
    {
        return false;
    }

    /*
     * active >= 1 秒时，5 个窗口在 active 内部按 0/25/50/75/100% 取起点。
     * active < 1 秒时，5 个窗口让短命令分别落在模型窗口的偏前、25%、中心、75%、偏后位置。
     */
    {
        uint32_t active_samples = p_plan->active_end_sample - p_plan->active_start_sample;
        uint32_t align_range = 0U;

        if (active_samples >= VOICE_AI_MODEL_SAMPLES)
        {
            align_range = active_samples - VOICE_AI_MODEL_SAMPLES;

            for (uint32_t candidate = 0U; candidate < VOICE_AI_MAX_CANDIDATES; candidate++)
            {
                uint32_t offset = (align_range * candidate) / (VOICE_AI_MAX_CANDIDATES - 1U);
                int32_t start_sample = (int32_t)(p_plan->active_start_sample + offset);

                voice_ai_add_candidate(p_plan, start_sample, samples);
            }
        }
        else
        {
            align_range = VOICE_AI_MODEL_SAMPLES - active_samples;

            for (uint32_t candidate = 0U; candidate < VOICE_AI_MAX_CANDIDATES; candidate++)
            {
                uint32_t offset = (align_range * candidate) / (VOICE_AI_MAX_CANDIDATES - 1U);
                int32_t start_sample = (int32_t)p_plan->active_start_sample - (int32_t)offset;

                voice_ai_add_candidate(p_plan, start_sample, samples);
            }
        }
    }

    return (p_plan->candidate_count > 0U);
}

/**
 * @brief  从搜索窗口复制 1 秒候选音频到模型音频缓冲
 *
 * @param p_src 搜索窗口音频
 * @param src_samples 搜索窗口采样点数
 * @param start_sample 候选窗口起点
 * @return void
 */
static void voice_ai_copy_candidate_to_audio(const float *p_src,
                                             uint32_t src_samples,
                                             uint32_t start_sample)
{
    uint32_t copy_samples = 0U;

    memset(g_voice_ai_audio, 0, sizeof(g_voice_ai_audio));

    if ((NULL == p_src) || (start_sample >= src_samples))
    {
        return;
    }

    copy_samples = src_samples - start_sample;
    if (copy_samples > VOICE_AI_MODEL_SAMPLES)
    {
        copy_samples = VOICE_AI_MODEL_SAMPLES;
    }

    if (copy_samples > 0U)
    {
        memcpy(g_voice_ai_audio,
               &p_src[start_sample],
               copy_samples * sizeof(float));
    }
}

#if VOICE_AI_DEBUG_EXPORT_ENABLED
/**
 * @brief  将 float 音频样本转换为 PCM16
 *
 * @param sample float 音频样本
 * @return int16_t PCM16 样本
 */
static int16_t voice_ai_float_to_pcm16(float sample)
{
    float clipped = sample;

    if (clipped > 1.0f)
    {
        clipped = 1.0f;
    }
    else if (clipped < -1.0f)
    {
        clipped = -1.0f;
    }
    else
    {
        /* 样本已在合法范围内，不需要处理。 */
    }

    if (clipped >= 0.0f)
    {
        return (int16_t)((clipped * 32767.0f) + 0.5f);
    }

    return (int16_t)((clipped * 32768.0f) - 0.5f);
}
#endif

/**
 * @brief  执行峰值归一化
 *
 * @param p_audio 音频缓冲
 * @param samples 采样点数
 * @return void
 */
static void voice_ai_normalize_peak(float *p_audio, uint32_t samples)
{
    float peak = 0.0f;
    float abs_value = 0.0f;
    float scale = 1.0f;

    if ((NULL == p_audio) || (0U == samples))
    {
        return;
    }

    for (uint32_t i = 0U; i < samples; i++)
    {
        abs_value = fabsf(p_audio[i]);
        if (abs_value > peak)
        {
            peak = abs_value;
        }
    }

    if (peak >= 1.0e-6f)
    {
        scale = VOICE_AI_TARGET_PEAK / peak;
    }

    for (uint32_t i = 0U; i < samples; i++)
    {
        p_audio[i] *= scale;
    }
}

#if VOICE_AI_DEBUG_EXPORT_ENABLED
/**
 * @brief  导出 1 秒调试音频用于离线听音检查
 *
 * 注意：建议在归一化前调用，避免风噪被 normalize_peak() 放大后误判。
 *
 * @param p_audio      1 秒 float 音频
 * @param samples      采样点数
 * @param found_active true 表示本窗口找到有效声音
 * @return void
 */
static void voice_ai_debug_export_model_audio(const float *p_audio, uint32_t samples, bool found_active)
{
    uint32_t now_ms = 0U;

    if ((NULL == p_audio) || (VOICE_AI_MODEL_SAMPLES != samples) || (!found_active))
    {
        return;
    }

    now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if ((now_ms - g_voice_ai_debug_last_export_ms) < VOICE_AI_DEBUG_EXPORT_PERIOD_MS)
    {
        return;
    }

    for (uint32_t i = 0U; i < samples; i++)
    {
        g_voice_ai_debug_pcm[i] = voice_ai_float_to_pcm16(p_audio[i]);
    }

    if (FSP_SUCCESS == uart.write_pcm16_packet(VOICE_AI_DEBUG_EXPORT_LABEL,
                                               g_voice_ai_debug_pcm,
                                               samples,
                                               VOICE_AI_MODEL_SAMPLE_RATE_HZ))
    {
        g_voice_ai_debug_last_export_ms = now_ms;
    }
}
#endif

/**
 * @brief  执行预加重
 *
 * @param p_audio 音频缓冲
 * @param samples 采样点数
 * @return void
 */
static void voice_ai_apply_preemphasis(float *p_audio, uint32_t samples)
{
    float current = 0.0f;
    float previous = 0.0f;

    if ((NULL == p_audio) || (0U == samples))
    {
        return;
    }

    for (uint32_t i = 0U; i < samples; i++)
    {
        current = p_audio[i];
        p_audio[i] = current - (VOICE_AI_PREEMPHASIS * previous);
        previous = current;
    }
}

/**
 * @brief  计算一帧的 log-mel 特征并写入模型输入
 *
 * @param frame_index 帧索引
 * @param p_input     模型输入缓冲
 * @return void
 */
static void voice_ai_extract_frame_features(uint32_t frame_index, float *p_input)
{
    uint32_t frame_offset = frame_index * VOICE_AI_FRAME_STEP;
    uint32_t feature_offset = frame_index * VOICE_AI_MEL_BINS;
    uint16_t mel_start = 0U;
    uint16_t weight_start = 0U;
    uint16_t weight_end = 0U;
    float mel_energy = 0.0f;
    float log_mel = 0.0f;

    if (NULL == p_input)
    {
        return;
    }

    for (uint32_t i = 0U; i < VOICE_AI_FFT_LENGTH; i++)
    {
        g_voice_ai_fft_real[i] = 0.0f;
        g_voice_ai_fft_imag[i] = 0.0f;
    }

    for (uint32_t i = 0U; i < VOICE_AI_FRAME_LENGTH; i++)
    {
        g_voice_ai_fft_real[i] = g_voice_ai_audio[frame_offset + i] * g_voice_ai_window[i];
    }

    voice_ai_fft(g_voice_ai_fft_real, g_voice_ai_fft_imag);

    for (uint32_t i = 0U; i < VOICE_AI_FFT_BINS; i++)
    {
        g_voice_ai_power[i] = (g_voice_ai_fft_real[i] * g_voice_ai_fft_real[i]) +
                              (g_voice_ai_fft_imag[i] * g_voice_ai_fft_imag[i]);
    }

    for (uint32_t mel = 0U; mel < VOICE_AI_MEL_BINS; mel++)
    {
        mel_energy = 0.0f;
        mel_start = g_voice_ai_mel_starts[mel];
        weight_start = g_voice_ai_mel_offsets[mel];
        weight_end = g_voice_ai_mel_offsets[mel + 1U];

        for (uint32_t weight = weight_start; weight < weight_end; weight++)
        {
            uint32_t power_index = (uint32_t)mel_start + weight - weight_start;
            mel_energy += g_voice_ai_power[power_index] * g_voice_ai_mel_weights[weight];
        }

        if (mel_energy < VOICE_AI_LOG_FLOOR)
        {
            mel_energy = VOICE_AI_LOG_FLOOR;
        }

        log_mel = logf(mel_energy);
        p_input[feature_offset + mel] = (log_mel - VOICE_AI_FEATURE_MEAN) / VOICE_AI_FEATURE_STD;
    }
}

/**
 * @brief  使用当前 g_voice_ai_audio 生成模型输入特征
 *
 * @param p_input 模型输入缓冲
 * @return fsp_err_t FSP_SUCCESS 表示输入特征准备完成
 */
static fsp_err_t voice_ai_prepare_features_from_current_audio(float *p_input)
{
    if (NULL == p_input)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    voice_ai_normalize_peak(g_voice_ai_audio, VOICE_AI_MODEL_SAMPLES);
    voice_ai_apply_preemphasis(g_voice_ai_audio, VOICE_AI_MODEL_SAMPLES);

    for (uint32_t frame = 0U; frame < VOICE_AI_FRAME_COUNT; frame++)
    {
        voice_ai_extract_frame_features(frame, p_input);
    }

    return FSP_SUCCESS;
}

/**
 * @brief  打印模型输入特征统计，用于 MCU/PC 前处理一致性对比。
 *
 * @param candidate_index 候选索引
 * @param p_input         模型输入特征缓冲
 * @return void
 */
static void voice_ai_print_feature_stats(uint32_t candidate_index, const float *p_input)
{
#if VOICE_AI_DEBUG_FEATURE_STATS_ENABLED
    float min_value = 0.0f;
    float max_value = 0.0f;
    float sum = 0.0f;
    float mean = 0.0f;

    if (NULL == p_input)
    {
        return;
    }

    min_value = p_input[0];
    max_value = p_input[0];

    for (uint32_t i = 0U; i < VOICE_AI_MODEL_INPUT_FLOATS; i++)
    {
        if (p_input[i] < min_value)
        {
            min_value = p_input[i];
        }

        if (p_input[i] > max_value)
        {
            max_value = p_input[i];
        }

        sum += p_input[i];
    }

    mean = sum / (float)VOICE_AI_MODEL_INPUT_FLOATS;

    printf("voice_ai:input cand=%lu min=%ld/1000 max=%ld/1000 mean=%ld/1000",
           (unsigned long)candidate_index,
           (long)voice_ai_signed_float_to_milli(min_value),
           (long)voice_ai_signed_float_to_milli(max_value),
           (long)voice_ai_signed_float_to_milli(mean));

    for (uint32_t i = 0U; i < 8U; i++)
    {
        printf(" f%lu=%ld/1000",
               (unsigned long)i,
               (long)voice_ai_signed_float_to_milli(p_input[i]));
    }

    printf("\r\n");
#else
    FSP_PARAMETER_NOT_USED(candidate_index);
    FSP_PARAMETER_NOT_USED(p_input);
#endif
}

/**
 * @brief  打印模型输出统计和完整标签分数，用于输出指针和标签顺序诊断。
 *
 * @param candidate_index 候选索引
 * @param p_scores        模型输出分数
 * @return void
 */
static void voice_ai_print_output_stats(uint32_t candidate_index, const float *p_scores)
{
#if VOICE_AI_DEBUG_OUTPUT_STATS_ENABLED
    float min_value = 0.0f;
    float max_value = 0.0f;
    float sum = 0.0f;

    if (NULL == p_scores)
    {
        return;
    }

    min_value = p_scores[0];
    max_value = p_scores[0];

    for (uint32_t i = 0U; i < VOICE_AI_LABEL_COUNT; i++)
    {
        if (p_scores[i] < min_value)
        {
            min_value = p_scores[i];
        }

        if (p_scores[i] > max_value)
        {
            max_value = p_scores[i];
        }

        sum += p_scores[i];
    }

    printf("voice_ai:out cand=%lu sum=%ld/1000 min=%ld/1000 max=%ld/1000\r\n",
           (unsigned long)candidate_index,
           (long)voice_ai_signed_float_to_milli(sum),
           (long)voice_ai_signed_float_to_milli(min_value),
           (long)voice_ai_signed_float_to_milli(max_value));

    printf("voice_ai:scores cand=%lu", (unsigned long)candidate_index);
    for (uint32_t i = 0U; i < VOICE_AI_LABEL_COUNT; i++)
    {
        printf(" %s=%ld",
               voice_ai_label_name_impl((uint8_t)i),
               (long)voice_ai_signed_float_to_milli(p_scores[i]));
    }
    printf("\r\n");
#else
    FSP_PARAMETER_NOT_USED(candidate_index);
    FSP_PARAMETER_NOT_USED(p_scores);
#endif
}

/**
 * @brief  打印候选窗口 VAD 和推理信息
 *
 * @param p_plan 候选窗口计划
 * @param candidate_index 候选索引
 * @param top_index Top1 标签索引
 * @param top_score Top1 分数
 * @return void
 */
static void voice_ai_print_candidate_log(const voice_ai_candidate_plan_t *p_plan,
                                         uint32_t candidate_index,
                                         uint8_t top_index,
                                         float top_score)
{
    FSP_PARAMETER_NOT_USED(p_plan);
    FSP_PARAMETER_NOT_USED(candidate_index);
    FSP_PARAMETER_NOT_USED(top_index);
    FSP_PARAMETER_NOT_USED(top_score);
}

/**
 * @brief  执行一次完整 AI 推理，使用 VAD + 多候选 1 秒窗口滑窗
 *
 * @return fsp_err_t FSP_SUCCESS 表示本次推理完成
 */
static fsp_err_t voice_ai_run_once(void)
{
    fsp_err_t err = FSP_SUCCESS;
    float *p_input = NULL;
    float *p_output = NULL;
    uint32_t actual_samples = 0U;
    uint32_t start_cycles = 0U;
    uint32_t cycles = 0U;
    uint32_t best_candidate = 0U;
    float best_score = VOICE_AI_NEGATIVE_SCORE;
    voice_ai_candidate_plan_t plan;
    float candidate_scores[VOICE_AI_MAX_CANDIDATES][VOICE_AI_LABEL_COUNT];
    uint8_t candidate_top_indices[VOICE_AI_MAX_CANDIDATES][VOICE_AI_TOPK_COUNT];
    float candidate_top_scores[VOICE_AI_MAX_CANDIDATES][VOICE_AI_TOPK_COUNT];

    p_input = GetModelInputPtr_serving_default_input_2_0();
    p_output = GetModelOutputPtr_StatefulPartitionedCall_0_70023();
    if ((NULL == p_input) || (NULL == p_output))
    {
        return FSP_ERR_INVALID_POINTER;
    }

    err = microphone.latest_read(g_voice_ai_raw_pcm,
                                 VOICE_AI_RAW_SAMPLES,
                                 &actual_samples,
                                 NULL);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    if (actual_samples < VOICE_AI_RAW_SAMPLES)
    {
        return FSP_ERR_BUFFER_EMPTY;
    }

    voice_ai_resample_to_model_rate(g_voice_ai_raw_pcm,
                                    actual_samples,
                                    g_voice_ai_search_audio,
                                    VOICE_AI_SEARCH_SAMPLES);

    if (!voice_ai_build_candidate_plan(g_voice_ai_search_audio,
                                       VOICE_AI_SEARCH_SAMPLES,
                                       &plan))
    {
        return FSP_ERR_BUFFER_EMPTY;
    }

    memset(candidate_scores, 0, sizeof(candidate_scores));
    memset(candidate_top_indices, 0, sizeof(candidate_top_indices));
    memset(candidate_top_scores, 0, sizeof(candidate_top_scores));

    for (uint32_t candidate = 0U; candidate < plan.candidate_count; candidate++)
    {
        voice_ai_copy_candidate_to_audio(g_voice_ai_search_audio,
                                         VOICE_AI_SEARCH_SAMPLES,
                                         plan.candidate_start_samples[candidate]);

        err = voice_ai_prepare_features_from_current_audio(p_input);
        if (FSP_SUCCESS != err)
        {
            return err;
        }

        voice_ai_print_feature_stats(candidate, p_input);

        start_cycles = voice_ai_cycle_get();
        RunModel(false);
        cycles += (voice_ai_cycle_get() - start_cycles);

        for (uint32_t i = 0U; i < VOICE_AI_LABEL_COUNT; i++)
        {
            candidate_scores[candidate][i] = p_output[i];
        }

        voice_ai_print_output_stats(candidate, candidate_scores[candidate]);

        voice_ai_find_topk(candidate_scores[candidate],
                           candidate_top_indices[candidate],
                           candidate_top_scores[candidate]);

        voice_ai_print_candidate_log(&plan,
                                     candidate,
                                     candidate_top_indices[candidate][0],
                                     candidate_top_scores[candidate][0]);

        if (candidate_top_scores[candidate][0] > best_score)
        {
            best_score = candidate_top_scores[candidate][0];
            best_candidate = candidate;
        }
    }

    /* 打印全部候选结果：一行显示 5 个候选的 top1，末尾标注最终选中 */
    printf("voice_ai:detect [");
    for (uint32_t c = 0U; c < plan.candidate_count; c++)
    {
        printf("c%lu=%s:%ld",
               (unsigned long)c,
               voice_ai_label_name_impl(candidate_top_indices[c][0]),
               (long)voice_ai_score_to_milli(candidate_top_scores[c][0]));
        if ((c + 1U) < plan.candidate_count)
        {
            printf(" ");
        }
    }
    printf("] best=c%lu\r\n", (unsigned long)best_candidate);

#if VOICE_AI_DEBUG_EXPORT_ENABLED
    printf("voice_ai:debug_export cand=%lu start=%lums label=%s\r\n",
           (unsigned long)best_candidate,
           (unsigned long)((plan.candidate_start_samples[best_candidate] * 1000U) /
                           VOICE_AI_MODEL_SAMPLE_RATE_HZ),
           VOICE_AI_DEBUG_EXPORT_LABEL);

    voice_ai_copy_candidate_to_audio(g_voice_ai_search_audio,
                                     VOICE_AI_SEARCH_SAMPLES,
                                     plan.candidate_start_samples[best_candidate]);
    voice_ai_debug_export_model_audio(g_voice_ai_audio,
                                      VOICE_AI_MODEL_SAMPLES,
                                      plan.found_active);
#endif

    voice_ai_publish_result(candidate_scores[best_candidate],
                            cycles,
                            plan.candidate_count,
                            candidate_top_indices,
                            candidate_top_scores);

    return FSP_SUCCESS;
}

/**
 * @brief  打开 NPU 实例
 *
 * @return fsp_err_t FSP_SUCCESS 表示 NPU 已可用
 */
static fsp_err_t voice_ai_open_npu(void)
{
    fsp_err_t err = FSP_SUCCESS;

    if (g_voice_ai_npu_opened)
    {
        return FSP_SUCCESS;
    }

    err = RM_ETHOSU_Open(&g_rm_ethosu0_ctrl, &g_rm_ethosu0_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        printf("voice_ai:npu:fail:%d\r\n", (int)err);
        return err;
    }

    g_voice_ai_npu_opened = true;

    return FSP_SUCCESS;
}

/**
 * @brief  启动 AI 任务持有的麦克风采集
 *
 * @return fsp_err_t FSP_SUCCESS 表示麦克风采集已启动
 */
static fsp_err_t voice_ai_start_microphone(void)
{
    fsp_err_t err = FSP_SUCCESS;

    if (g_voice_ai_microphone_started)
    {
        return FSP_SUCCESS;
    }

    err = microphone.init();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = microphone.start();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    taskENTER_CRITICAL();
    g_voice_ai_microphone_started = true;
    taskEXIT_CRITICAL();

    return FSP_SUCCESS;
}

/**
 * @brief  停止 AI 任务持有的麦克风采集
 *
 * @return void
 */
static void voice_ai_stop_microphone(void)
{
    bool started = false;

    taskENTER_CRITICAL();
    started = g_voice_ai_microphone_started;
    taskEXIT_CRITICAL();

    if (!started)
    {
        return;
    }

    (void)microphone.stop();

    taskENTER_CRITICAL();
    g_voice_ai_microphone_started = false;
    taskEXIT_CRITICAL();
}

/**
 * @brief  执行一次语音 AI 后台处理
 *
 * @return fsp_err_t FSP_SUCCESS 表示处理完成，FSP_ERR_BUFFER_EMPTY 表示本次没有有效语音
 */
static fsp_err_t voice_ai_process_impl(void)
{
    fsp_err_t err = FSP_SUCCESS;

    if (!voice_ai_enabled_get())
    {
        voice_ai_busy_set(false);
        voice_ai_stop_microphone();
        voice_ai_set_state(VOICE_AI_STATE_PAUSED);
        return FSP_SUCCESS;
    }

    err = voice_ai_start_microphone();
    if (FSP_SUCCESS != err)
    {
        g_voice_ai_error_count++;
        voice_ai_set_state(VOICE_AI_STATE_ERROR);
        printf("voice_ai:mic:fail:%d\r\n", (int)err);
        return err;
    }

    voice_ai_set_state(VOICE_AI_STATE_RUNNING);
    voice_ai_busy_set(true);
    err = voice_ai_run_once();
    voice_ai_busy_set(false);
    if ((FSP_SUCCESS != err) && (FSP_ERR_BUFFER_EMPTY != err))
    {
        g_voice_ai_error_count++;
        voice_ai_set_state(VOICE_AI_STATE_ERROR);
        printf("voice_ai:run:fail:%d\r\n", (int)err);
    }

    return err;
}

#if VOICE_AI_WAV_VECTOR_SELF_TEST_ENABLED
/**
 * @brief  将 int16 PCM 转为 float 并填入内部音频缓冲，走完整前处理+NPU 推理
 *
 * 本函数直接复用麦克风识别流程中的 g_voice_ai_audio 和
 * voice_ai_prepare_features_from_current_audio()，不做重采样/归一化。
 *
 * @param pcm          int16 PCM 数据
 * @param sample_count 采样点数
 * @param out_scores   当前标签表对应的模型分数
 * @return void
 */
static void voice_ai_run_pcm_test_once(const int16_t *pcm,
                                       size_t sample_count,
                                       float *out_scores)
{
    float *p_input = NULL;
    float *p_output = NULL;
    uint32_t copy_samples = 0U;

    if ((NULL == pcm) || (NULL == out_scores) || (0U == sample_count))
    {
        return;
    }

    p_input = GetModelInputPtr_serving_default_input_2_0();
    p_output = GetModelOutputPtr_StatefulPartitionedCall_0_70023();
    if ((NULL == p_input) || (NULL == p_output))
    {
        return;
    }

    /* int16 PCM 转 float，与训练端保持一致：除以 32767.0 */
    copy_samples = (uint32_t)sample_count;
    if (copy_samples > VOICE_AI_MODEL_SAMPLES)
    {
        copy_samples = VOICE_AI_MODEL_SAMPLES;
    }

    for (uint32_t i = 0U; i < copy_samples; i++)
    {
        g_voice_ai_audio[i] = (float)pcm[i] / 32767.0f;
    }

    for (uint32_t i = copy_samples; i < VOICE_AI_MODEL_SAMPLES; i++)
    {
        g_voice_ai_audio[i] = 0.0f;
    }

    /* 走与麦克风识别完全相同的前处理流程 */
    (void)voice_ai_prepare_features_from_current_audio(p_input);

    /* NPU 推理 */
    RunModel(false);

    /* 读取输出分数 */
    for (uint32_t i = 0U; i < VOICE_AI_LABEL_COUNT; i++)
    {
        out_scores[i] = p_output[i];
    }
}

/**
 * @brief  使用训练集 wav 样本运行离线自测，验证 MCU 完整前处理+NPU 推理链路
 *
 * @return void
 */
static void voice_ai_run_wav_vector_self_test(void)
{
    uint32_t pass_count = 0U;
    uint32_t warn_count = 0U;
    uint32_t fail_count = 0U;

    printf("voice_ai:wav_self_test:start count=%u\r\n",
           (unsigned int)g_voice_test_vector_count);

    for (size_t i = 0U; i < g_voice_test_vector_count; i++)
    {
        const voice_test_vector_t *tv = &g_voice_test_vectors[i];
        float scores[VOICE_AI_LABEL_COUNT];
        uint8_t top_indices[VOICE_AI_TOPK_COUNT];
        float top_scores[VOICE_AI_TOPK_COUNT];
        int32_t pcm_min = 0;
        int32_t pcm_max = 0;
        int64_t pcm_sum = 0;
        int32_t pcm_mean = 0;
        uint32_t pcm_crc = 0U;
        const char *result_str = "FAIL";
        int32_t top1_milli = 0;
        int32_t top2_milli = 0;
        int32_t top3_milli = 0;
        int32_t margin_milli = 0;
        int32_t expected_milli = 0;

        printf("voice_ai:wav_test name=%s expected=%s samples=%u\r\n",
               tv->name,
               tv->expected_label,
               (unsigned int)tv->sample_count);

        /* PCM 统计信息 */
        if ((NULL != tv->pcm) && (tv->sample_count > 0U))
        {
            pcm_min = (int32_t)tv->pcm[0];
            pcm_max = (int32_t)tv->pcm[0];

            for (size_t j = 0U; j < tv->sample_count; j++)
            {
                int32_t val = (int32_t)tv->pcm[j];

                if (val < pcm_min) { pcm_min = val; }
                if (val > pcm_max) { pcm_max = val; }
                pcm_sum += (int64_t)val;
                pcm_crc = (pcm_crc * 33U) + (uint32_t)(uint16_t)val;
            }

            pcm_mean = (int32_t)(pcm_sum / (int64_t)tv->sample_count);
        }

        printf("voice_ai:wav_test_pcm min=%ld max=%ld mean=%ld crc=%08lX\r\n",
               (long)pcm_min,
               (long)pcm_max,
               (long)pcm_mean,
               (unsigned long)pcm_crc);

        /* 跑完整前处理+推理 */
        memset(scores, 0, sizeof(scores));
        voice_ai_run_pcm_test_once(tv->pcm, tv->sample_count, scores);

        /* 排序 TopK */
        voice_ai_find_topk(scores, top_indices, top_scores);

        top1_milli = voice_ai_score_to_milli(top_scores[0]);
        top2_milli = voice_ai_score_to_milli(top_scores[1]);
        top3_milli = voice_ai_score_to_milli(top_scores[2]);
        margin_milli = top1_milli - top2_milli;

        /* 查找期望标签对应的分数 */
        for (uint32_t k = 0U; k < VOICE_AI_LABEL_COUNT; k++)
        {
            if (0 == strcmp(tv->expected_label, voice_ai_label_name_impl((uint8_t)k)))
            {
                expected_milli = voice_ai_score_to_milli(scores[k]);
                break;
            }
        }

        /* 打印完整标签分数。 */
        printf("voice_ai:wav_test_scores");
        for (uint32_t k = 0U; k < VOICE_AI_LABEL_COUNT; k++)
        {
            printf(" %s=%ld",
                   voice_ai_label_name_impl((uint8_t)k),
                   (long)voice_ai_score_to_milli(scores[k]));
        }
        printf("\r\n");

        /* 判断 PASS/WARN/FAIL */
        if (0 == strcmp(tv->expected_label, voice_ai_label_name_impl(top_indices[0])))
        {
            result_str = "PASS";
            pass_count++;
        }
        else
        {
            bool in_top3 = false;

            for (uint32_t k = 0U; k < VOICE_AI_TOPK_COUNT; k++)
            {
                if (0 == strcmp(tv->expected_label, voice_ai_label_name_impl(top_indices[k])))
                {
                    in_top3 = true;
                    break;
                }
            }

            if (in_top3)
            {
                result_str = "WARN";
                warn_count++;
            }
            else
            {
                result_str = "FAIL";
                fail_count++;
            }
        }

        printf("voice_ai:wav_test_result name=%s expected=%s top1=%s score=%ld/1000 top2=%s score2=%ld/1000 top3=%s score3=%ld/1000 margin=%ld/1000 expected_score=%ld/1000 %s\r\n",
               tv->name,
               tv->expected_label,
               voice_ai_label_name_impl(top_indices[0]),
               (long)top1_milli,
               voice_ai_label_name_impl(top_indices[1]),
               (long)top2_milli,
               voice_ai_label_name_impl(top_indices[2]),
               (long)top3_milli,
               (long)margin_milli,
               (long)expected_milli,
               result_str);
    }

    printf("voice_ai:wav_self_test:done pass=%u warn=%u fail=%u\r\n",
           (unsigned int)pass_count,
           (unsigned int)warn_count,
           (unsigned int)fail_count);
}
#endif

/**
 * @brief  初始化语音 AI 模块
 *
 * @return fsp_err_t FSP_SUCCESS 表示初始化成功
 */
static fsp_err_t voice_ai_init_impl(void)
{
    fsp_err_t err = FSP_SUCCESS;

    if (g_voice_ai_initialized)
    {
        return FSP_SUCCESS;
    }

    voice_ai_prepare_tables();
    voice_ai_cycle_counter_init();

    err = voice_ai_open_npu();
    if (FSP_SUCCESS != err)
    {
        voice_ai_set_state(VOICE_AI_STATE_ERROR);
        return err;
    }

    voice_ai_run_model_self_test();

#if VOICE_AI_WAV_VECTOR_SELF_TEST_ENABLED
    voice_ai_run_wav_vector_self_test();
#endif

    taskENTER_CRITICAL();
    g_voice_ai_enabled = true;
    taskEXIT_CRITICAL();

    g_voice_ai_initialized = true;
    voice_ai_set_state(VOICE_AI_STATE_STARTING);

    return FSP_SUCCESS;
}

/**
 * @brief  启用或暂停语音 AI 任务
 *
 * @param enabled true 表示启用，false 表示暂停并释放麦克风
 * @return fsp_err_t FSP_SUCCESS 表示设置完成
 */
static fsp_err_t voice_ai_set_enabled_impl(bool enabled)
{
    uint32_t waited_ms = 0U;
    bool stopped = false;

    if (!g_voice_ai_initialized)
    {
        if (enabled)
        {
            return voice_ai_init_impl();
        }

        taskENTER_CRITICAL();
        g_voice_ai_enabled = false;
        taskEXIT_CRITICAL();
        return FSP_SUCCESS;
    }

    taskENTER_CRITICAL();
    g_voice_ai_enabled = enabled;
    taskEXIT_CRITICAL();

    if (enabled)
    {
        voice_ai_set_state(VOICE_AI_STATE_STARTING);
        return FSP_SUCCESS;
    }

    while (waited_ms <= VOICE_AI_DISABLE_TIMEOUT_MS)
    {
        taskENTER_CRITICAL();
        stopped = (!g_voice_ai_busy) && (!g_voice_ai_microphone_started);
        taskEXIT_CRITICAL();

        if (stopped)
        {
            voice_ai_set_state(VOICE_AI_STATE_PAUSED);
            return FSP_SUCCESS;
        }

        vTaskDelay(pdMS_TO_TICKS(VOICE_AI_DISABLE_POLL_MS));
        waited_ms += VOICE_AI_DISABLE_POLL_MS;
    }

    return FSP_ERR_TIMEOUT;
}

/**
 * @brief  读取最近一次推理结果
 *
 * @param p_result 结果输出
 * @return fsp_err_t FSP_SUCCESS 表示读取成功
 */
static fsp_err_t voice_ai_latest_read_impl(voice_ai_result_t *p_result)
{
    if (NULL == p_result)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    taskENTER_CRITICAL();
    *p_result = g_voice_ai_latest_result;
    taskEXIT_CRITICAL();

    return FSP_SUCCESS;
}

/**
 * @brief  查询是否有新结果
 *
 * @return bool true 表示存在未消费结果
 */
static bool voice_ai_has_new_impl(void)
{
    bool has_new = false;

    taskENTER_CRITICAL();
    has_new = g_voice_ai_has_new;
    taskEXIT_CRITICAL();

    return has_new;
}

/**
 * @brief  清除新结果标志
 *
 * @return void
 */
static void voice_ai_clear_new_impl(void)
{
    taskENTER_CRITICAL();
    g_voice_ai_has_new = false;
    taskEXIT_CRITICAL();
}

const voice_ai_api_t voice_ai =
    {
        .init = voice_ai_init_impl,
        .process = voice_ai_process_impl,
        .set_enabled = voice_ai_set_enabled_impl,
        .latest_read = voice_ai_latest_read_impl,
        .has_new = voice_ai_has_new_impl,
        .clear_new = voice_ai_clear_new_impl,
        .label_name = voice_ai_label_name_impl,
};
