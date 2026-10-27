"""
将训练集 wav 文件转换为 MCU C 数组，用于离线自测。

用法: python wav_to_voice_test_vectors.py
输出: voice_test_vectors.h / voice_test_vectors.c
"""

import struct
import zlib
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
OUTPUT_DIR = os.path.join(PROJECT_DIR, "my-code", "software")

SAMPLE_RATE = 16000
SAMPLE_COUNT = 16000

TEST_VECTORS = [
    {
        "wav": os.path.join(
            PROJECT_DIR,
            "my-code", "ai", "dataset_real", "trimmed",
            "backward", "backward_0020.wav",
        ),
        "name": "backward_0020",
        "label": "backward",
        "var": "g_voice_test_backward_0020",
    },
    {
        "wav": os.path.join(
            PROJECT_DIR,
            "my-code", "ai", "dataset_real", "trimmed",
            "turn_left", "turn_left_0006.wav",
        ),
        "name": "turn_left_0006",
        "label": "turn_left",
        "var": "g_voice_test_turn_left_0006",
    },
]


def read_wav_pcm(path):
    """从 WAV 文件中读取 int16 PCM 数据，不做重采样/归一化/声道转换。"""
    with open(path, "rb") as f:
        # RIFF header
        riff = f.read(4)
        if riff != b"RIFF":
            raise ValueError(f"不是有效的 WAV 文件: {path}")
        _file_size = struct.unpack("<I", f.read(4))[0]
        wave = f.read(4)
        if wave != b"WAVE":
            raise ValueError(f"不是有效的 WAV 文件: {path}")

        # 查找 data chunk
        fmt_found = False
        data_found = False
        data_bytes = b""
        sample_rate = 0
        num_channels = 0
        bits_per_sample = 0

        while not data_found:
            chunk_id = f.read(4)
            if len(chunk_id) < 4:
                break
            chunk_size = struct.unpack("<I", f.read(4))[0]

            if chunk_id == b"fmt ":
                fmt_data = f.read(chunk_size)
                audio_format = struct.unpack("<H", fmt_data[0:2])[0]
                num_channels = struct.unpack("<H", fmt_data[2:4])[0]
                sample_rate = struct.unpack("<I", fmt_data[4:8])[0]
                _byte_rate = struct.unpack("<I", fmt_data[8:12])[0]
                _block_align = struct.unpack("<H", fmt_data[12:14])[0]
                bits_per_sample = struct.unpack("<H", fmt_data[14:16])[0]

                if audio_format != 1:
                    raise ValueError(f"只支持 PCM 格式，当前格式: {audio_format}")
                if num_channels != 1:
                    raise ValueError(f"只支持单声道，当前声道数: {num_channels}")
                if bits_per_sample != 16:
                    raise ValueError(f"只支持 16-bit，当前位深: {bits_per_sample}")
                fmt_found = True

            elif chunk_id == b"data":
                data_bytes = f.read(chunk_size)
                data_found = True
            else:
                f.read(chunk_size)

        if not fmt_found:
            raise ValueError("未找到 fmt chunk")
        if not data_found:
            raise ValueError("未找到 data chunk")

    # 解析 int16 PCM
    pcm_count = len(data_bytes) // 2
    pcm = []
    for i in range(pcm_count):
        val = struct.unpack_from("<h", data_bytes, i * 2)[0]
        pcm.append(val)

    print(f"  WAV: {path}")
    print(f"  sample_rate={sample_rate} channels={num_channels} bits={bits_per_sample}")
    print(f"  raw_samples={pcm_count}")

    return pcm, sample_rate


def pad_or_truncate(pcm, target):
    """将 PCM 列表对齐到 target 长度，不足补 0，超出截断。"""
    if len(pcm) > target:
        print(f"  truncate {len(pcm)} -> {target} samples")
        return pcm[:target]
    elif len(pcm) < target:
        pad_count = target - len(pcm)
        print(f"  pad {len(pcm)} -> {target} samples (+{pad_count} zeros)")
        return pcm + [0] * pad_count
    return pcm


def pcm_stats(pcm):
    """计算 PCM 统计信息。"""
    mn = min(pcm)
    mx = max(pcm)
    mean = sum(pcm) // len(pcm)
    return mn, mx, mean


def pcm_crc32(pcm):
    """计算 PCM 数据的 CRC32。"""
    raw = b""
    for v in pcm:
        raw += struct.pack("<h", v)
    return zlib.crc32(raw) & 0xFFFFFFFF


def gen_c_array(var_name, pcm):
    """生成 C const int16_t 数组文本。"""
    lines = []
    lines.append(f"VOICE_TEST_VECTOR_SECTION")
    lines.append(
        f"const int16_t {var_name}[VOICE_TEST_SAMPLE_COUNT] ="
    )
    lines.append("{")

    # 每行 16 个值
    for i in range(0, len(pcm), 16):
        chunk = pcm[i : i + 16]
        vals = ", ".join(str(v) for v in chunk)
        if i + 16 < len(pcm):
            lines.append(f"    {vals},")
        else:
            lines.append(f"    {vals}")

    lines.append("};")
    return "\n".join(lines)


def gen_header():
    """生成 voice_test_vectors.h 内容。"""
    return """#ifndef VOICE_TEST_VECTORS_H
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
"""


def gen_c_file(vectors_info):
    """生成 voice_test_vectors.c 内容。"""
    parts = []
    parts.append('#include "voice_test_vectors.h"')
    parts.append("")
    parts.append("#ifndef VOICE_TEST_VECTOR_SECTION")
    parts.append("#define VOICE_TEST_VECTOR_SECTION")
    parts.append("#endif")
    parts.append("")

    for info in vectors_info:
        parts.append(gen_c_array(info["var"], info["pcm"]))
        parts.append("")

    # 生成 g_voice_test_vectors 数组
    parts.append("const voice_test_vector_t g_voice_test_vectors[] =")
    parts.append("{")
    for info in vectors_info:
        parts.append("    {")
        parts.append(f'        .name = "{info["name"]}",')
        parts.append(f'        .expected_label = "{info["label"]}",')
        parts.append(f"        .pcm = {info['var']},")
        parts.append("        .sample_count = VOICE_TEST_SAMPLE_COUNT,")
        parts.append("    },")
    parts.append("};")
    parts.append("")
    parts.append("const size_t g_voice_test_vector_count =")
    parts.append("    sizeof(g_voice_test_vectors) / sizeof(g_voice_test_vectors[0]);")
    parts.append("")

    return "\n".join(parts)


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    vectors_info = []
    for tv in TEST_VECTORS:
        print(f"处理: {tv['name']}")
        pcm, sr = read_wav_pcm(tv["wav"])
        if sr != SAMPLE_RATE:
            print(f"  WARNING: sample_rate={sr}, expected={SAMPLE_RATE}")
        original_count = len(pcm)
        pcm = pad_or_truncate(pcm, SAMPLE_COUNT)
        mn, mx, mean = pcm_stats(pcm)
        crc = pcm_crc32(pcm)
        print(f"  samples: {original_count} -> {len(pcm)}")
        print(f"  min={mn} max={mx} mean={mean}")
        print(f"  crc32=0x{crc:08X}")
        print()

        vectors_info.append({
            **tv,
            "pcm": pcm,
            "original_count": original_count,
        })

    # 写入 .h
    h_path = os.path.join(OUTPUT_DIR, "voice_test_vectors.h")
    with open(h_path, "w", encoding="utf-8") as f:
        f.write(gen_header())
    print(f"生成: {h_path}")

    # 写入 .c
    c_path = os.path.join(OUTPUT_DIR, "voice_test_vectors.c")
    with open(c_path, "w", encoding="utf-8") as f:
        f.write(gen_c_file(vectors_info))
    print(f"生成: {c_path}")

    # 打印数组大小估计
    for info in vectors_info:
        size_kb = (len(info["pcm"]) * 2) / 1024.0
        print(f"  {info['var']}: {len(info['pcm'])} samples, ~{size_kb:.1f} KB Flash")


if __name__ == "__main__":
    main()
