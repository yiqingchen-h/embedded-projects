import os
import csv
import time
import string
import argparse
import wave
from pathlib import Path


# ============================================================
# 1. 基本配置
# ============================================================

# 关闭无关日志
os.environ["HF_HUB_DISABLE_SYMLINKS_WARNING"] = "1"
os.environ["ONEDNN_VERBOSE"] = "0"
os.environ["DNNL_VERBOSE"] = "0"

# 默认检查 trimmed 目录中的音频


AI_ROOT = Path(__file__).resolve().parents[1]
INPUT_DIR = AI_ROOT / "dataset_real" / "trimmed"


# 输出 CSV 文件


OUTPUT_CSV = None


# OpenVINO 模型目录
# 可切换：
# whisper-base-openvino
# whisper-small-openvino
# whisper-medium-openvino
MODEL_DIR = AI_ROOT / "whisper-medium-openvino"

# 推理设备由 --device 指定
DEVICE = None

# 是否递归扫描子文件夹
RECURSIVE = True

# 支持的音频格式
AUDIO_EXTS = {
    ".wav",
    ".mp3",
    ".m4a",
    ".flac",
    ".aac",
    ".ogg",
}

# OpenVINO Whisper 输入统一重采样到 16kHz
TARGET_SAMPLE_RATE = 16000

# 每条短命令最多生成 token 数
MAX_NEW_TOKENS = 64

# 命令词识别提示
INITIAL_PROMPT = (
    "这是一个智能小车语音命令识别任务。"
    "可能出现的语音包括：你好小车，开始，停止，"
    "抓取方块，抓河马，抓小猫，抓石墩子，"
    "抓水豚，抓玉米，抓蕉太狼，抓小狗，抓炸弹。"
)

# 命令词热词集合
HOTWORDS = (
    "你好小车 开始 停止 "
    "抓取方块 "
    "抓河马 抓小猫 抓石墩子 抓水豚 抓玉米 抓蕉太狼 抓小狗 抓炸弹"
)


# ============================================================
# 2. 标签映射
# ============================================================

LABEL_TEXT_MAP = {
    "wake": ["你好小车", "小车", "小车你好", "小车小车"],

    "start": ["开始","开始任务","开始执行","启动"],
    "stop": [ "停止","停下","停止任务","别动",],

    # 层级匹配：长词优先，短词兜底。列表按优先级从高到低排列。
    "select_cube": ["抓取方块", "取方块", "抓方块", "拿方块","方块"],
    "select_hippo": ["抓取河马", "抓河马", "取河马", "拿河马", "河马"],
    "select_cat": ["抓取小猫", "抓小猫", "取小猫", "拿小猫", "小猫", "猫"],
    "select_stone_pillar": ["抓取石墩子", "抓石墩子", "取石墩子", "拿石墩子", "石墩子", "石墩"],
    "select_capybara": ["抓取水豚", "抓水豚", "取水豚", "拿水豚", "水豚"],
    "select_corn": ["抓取玉米", "抓玉米", "取玉米", "拿玉米", "玉米"],
    "select_jiaotailang": ["抓取蕉太狼", "抓蕉太狼", "取蕉太狼", "拿蕉太狼", "蕉太狼"],
    "select_dog": ["抓取小狗", "抓小狗", "取小狗", "拿小狗", "小狗", "狗"],
    "select_tnt": ["抓取炸弹", "炸弹", "取炸弹", "拿炸弹", "抓 炸弹", "炸弹"],

    # unknown 和 silence 不要求识别出固定文本
    # 判断逻辑是：不应该命中任何有效命令
    "unknown": [],
    "silence": [],
}


COMMAND_KEYWORDS = [
    "你好小车",
    "小车",

    "开始",
    "停止",

    "抓取方块",
    "取方块",
    "抓方块",

    "抓取河马",
    "抓河马",
    "取河马",

    "抓取小猫",
    "抓小猫",
    "取小猫",

    "抓取石墩子",
    "抓石墩子",
    "取石墩子",

    "抓取水豚",
    "抓水豚",
    "取水豚",

    "抓取玉米",
    "抓玉米",
    "取玉米",

    "抓取蕉太狼",
    "抓蕉太狼",
    "取蕉太狼",

    "抓取小狗",
    "抓小狗",
    "取小狗",

    "抓取炸弹",
    "抓炸弹",
    "拿炸弹",

    "方块",
    "河马",
    "小猫",
    "猫",
    "石墩子",
    "石墩",
    "水豚",
    "玉米",
    "蕉太狼",
    "小狗",
    "狗",
    "炸弹",
]


# ============================================================
# 3. 工具函数
# ============================================================

def check_openvino_device(device=None):
    import openvino as ov
    device = device or DEVICE
    if not device:
        raise ValueError("必须显式指定 OpenVINO 设备")
    core = ov.Core()

    print("========== OpenVINO 设备 ==========")
    print("Available devices:", core.available_devices)

    if device not in core.available_devices:
        raise RuntimeError(
            f"当前 OpenVINO 没有发现指定设备：{device}\n"
            f"可用设备：{core.available_devices}"
        )

    try:
        device_name = core.get_property(device, "FULL_DEVICE_NAME")
    except Exception:
        device_name = "unknown"

    print(f"使用设备：{device}")
    print(f"设备名称：{device_name}")
    print("===================================")


def collect_audio_files(input_dir: Path):
    if not input_dir.exists():
        raise FileNotFoundError(f"输入目录不存在：{input_dir}")

    pattern = "**/*" if RECURSIVE else "*"

    files = [
        p for p in input_dir.glob(pattern)
        if p.is_file() and p.suffix.lower() in AUDIO_EXTS
    ]

    return sorted(files)


def infer_expected_label(audio_path: Path, root_dir: Path):
    """优先取输入根目录下第一级子目录名作为标签。

    根目录直接放置音频时，依次检查下划线、连字符和空格，取首个存在的分隔符前缀；均不存在时返回空标签。
    """
    try:
        relative = audio_path.relative_to(root_dir)
        parts = relative.parts

        if len(parts) >= 2:
            return parts[0]

        stem = audio_path.stem
        for sep in ["_", "-", " "]:
            if sep in stem:
                return stem.split(sep)[0]

        return ""
    except Exception:
        return ""


def normalize_text(text: str):
    """
    清理识别文本，去掉空格和常见标点，方便关键词匹配。
    """
    if text is None:
        return ""

    remove_chars = set(string.whitespace)
    remove_chars.update([
        "，", "。", "！", "？", "、", "；", "：",
        ",", ".", "!", "?", ";", ":",
        "“", "”", "\"", "'", "‘", "’",
        "（", "）", "(", ")",
        "[", "]", "【", "】",
    ])

    return "".join(ch for ch in text if ch not in remove_chars)


def match_words(text: str, words):
    """
    层级关键词匹配。

    只匹配 LABEL_TEXT_MAP 中显式列出的关键词。
    列表按优先级从高到低排列（长词 → 短词），
    短词只在长词未命中时生效，避免单字泛化匹配。
    不做反向/前缀匹配——未在表中列出的组合一律不命中。
    """
    text_norm = normalize_text(text)
    if not text_norm:
        return []

    hits = []
    for word in words:
        word_norm = normalize_text(word)
        if word_norm and word_norm in text_norm:
            hits.append(word)

    return hits


def judge_result(expected_label: str, asr_text: str):
    """
    普通命令：
        识别文本中包含该类别的目标词，即通过。

    unknown / silence：
        不应该命中任何有效命令词。
    """
    expected_label = expected_label.strip()
    asr_text = asr_text.strip()

    command_hits = match_words(asr_text, COMMAND_KEYWORDS)

    if expected_label in ["unknown", "silence"]:
        if len(command_hits) == 0:
            return "1", "", "|".join(command_hits)
        else:
            return "0", "unknown/silence 被识别成有效命令", "|".join(command_hits)

    expected_words = LABEL_TEXT_MAP.get(expected_label, [])

    if not expected_words:
        return "", "未知标签，未配置 LABEL_TEXT_MAP", "|".join(command_hits)

    expected_hits = match_words(asr_text, expected_words)

    if len(expected_hits) > 0:
        return "1", "", "|".join(command_hits)

    return "0", "未识别到目标命令词", "|".join(command_hits)


def read_audio_16k(audio_path: Path):
    """
    OpenVINO GenAI WhisperPipeline 输入使用 16kHz 音频。
    """
    if audio_path.suffix.lower() == ".wav":
        with wave.open(str(audio_path), "rb") as wav_file:
            channels = wav_file.getnchannels()
            sample_width = wav_file.getsampwidth()
            sample_rate = wav_file.getframerate()
            frame_count = wav_file.getnframes()
            data = wav_file.readframes(frame_count)

        if sample_width != 2:
            raise RuntimeError(f"只支持 PCM16 WAV：{audio_path}")

        audio = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32767.0
        if channels > 1:
            audio = audio.reshape(-1, channels).mean(axis=1)

        if sample_rate != TARGET_SAMPLE_RATE:
            src_x = np.linspace(0.0, 1.0, num=audio.size, dtype=np.float32)
            dst_len = max(1, int(round(audio.size * TARGET_SAMPLE_RATE / sample_rate)))
            dst_x = np.linspace(0.0, 1.0, num=dst_len, dtype=np.float32)
            audio = np.interp(dst_x, src_x, audio).astype(np.float32)

        duration_sec = len(audio) / TARGET_SAMPLE_RATE
        return audio.astype(np.float32).tolist(), duration_sec

    import librosa

    audio, sample_rate = librosa.load(str(audio_path), sr=TARGET_SAMPLE_RATE, mono=True)
    duration_sec = len(audio) / TARGET_SAMPLE_RATE
    return audio, duration_sec


def result_to_text(result):
    """
    兼容 openvino_genai 返回值。
    将非空返回对象转换为去除首尾空白的字符串。
    """
    if result is None:
        return ""

    text = str(result).strip()

    # 某些版本可能返回类似带字段的对象字符串，这里做简单兜底清理
    if text.startswith("Whisper") and "texts=" in text:
        return text

    return text


def transcribe_one(pipe, audio_path: Path):
    """
    使用 OpenVINO GPU 识别单个音频文件。
    """
    raw_speech, duration_sec = read_audio_16k(audio_path)

    t0 = time.time()

    result = pipe.generate(
        raw_speech,
        language="<|zh|>",
        initial_prompt=INITIAL_PROMPT,
        hotwords=HOTWORDS,
        max_new_tokens=MAX_NEW_TOKENS,
    )

    t1 = time.time()

    text = result_to_text(result)

    return {
        "text": text,
        "duration": round(duration_sec, 3),
        "inference_time": round(t1 - t0, 4),
    }


# ============================================================
# 4. 主程序
# ============================================================

def main():
    global INPUT_DIR
    global OUTPUT_CSV
    global MODEL_DIR
    global DEVICE
    global RECURSIVE
    global np, openvino_genai

    parser = argparse.ArgumentParser(description="OpenVINO GPU 批量语音识别检查")
    parser.add_argument("--input-dir", type=Path, default=INPUT_DIR, help="输入音频根目录")
    parser.add_argument("--output-csv", type=Path, default=OUTPUT_CSV, help="输出 CSV 路径")
    parser.add_argument("--model-dir", type=Path, default=MODEL_DIR, help="OpenVINO Whisper 模型目录")
    parser.add_argument("--device", required=True, help="OpenVINO 推理设备，例如 GPU.1 或 CPU")
    parser.add_argument("--recursive", type=int, choices=(0, 1), default=1 if RECURSIVE else 0, help="是否递归扫描")
    parser.add_argument("--limit", type=int, default=0, help="最多识别多少条，0 表示全部")
    parser.add_argument("--quiet", action="store_true", help="减少逐文件日志输出")
    args = parser.parse_args()

    INPUT_DIR = args.input_dir
    OUTPUT_CSV = args.output_csv or INPUT_DIR / "transcribe_result_all_openvino_gpu.csv"
    MODEL_DIR = args.model_dir
    DEVICE = args.device
    RECURSIVE = bool(args.recursive)

    print("========== OpenVINO GPU 批量语音识别检查 ==========")
    print(f"输入目录：{INPUT_DIR}")
    print(f"输出文件：{OUTPUT_CSV}")
    print(f"模型目录：{MODEL_DIR}")
    print(f"推理设备：{DEVICE}")
    print("===================================================")

    check_openvino_device(DEVICE)
    import numpy as np
    import openvino_genai

    if not INPUT_DIR.is_dir():
        parser.error(f"输入目录不存在: {INPUT_DIR}")
    if not MODEL_DIR.exists():
        raise FileNotFoundError(
            f"模型目录不存在：{MODEL_DIR}\n"
            f"请先导出模型，例如：\n"
            f"optimum-cli export openvino --trust-remote-code --model openai/whisper-base whisper-base-openvino"
        )

    audio_files = collect_audio_files(INPUT_DIR)
    if args.limit > 0:
        audio_files = audio_files[: args.limit]

    print(f"发现音频数量：{len(audio_files)}")

    if not audio_files:
        print("没有找到音频文件。")
        return

    OUTPUT_CSV.parent.mkdir(parents=True, exist_ok=True)

    print("开始加载 OpenVINO WhisperPipeline...")
    t_load0 = time.time()
    pipe = openvino_genai.WhisperPipeline(str(MODEL_DIR), DEVICE)
    t_load1 = time.time()
    print(f"模型加载完成，耗时：{t_load1 - t_load0:.3f} 秒")
    print("开始识别。")

    fieldnames = [
        "index",
        "file",
        "relative_path",
        "expected_label",
        "expected_words",
        "asr_text",
        "command_hits",
        "pass_by_label",
        "fail_reason",
        "duration_sec",
        "inference_time_sec",
        "error",
    ]

    total_count = 0
    pass_count = 0
    fail_count = 0
    error_count = 0

    label_stat = {}

    start_time = time.time()

    with OUTPUT_CSV.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for idx, audio_path in enumerate(audio_files, start=1):
            relative_path = audio_path.relative_to(INPUT_DIR)
            expected_label = infer_expected_label(audio_path, INPUT_DIR)
            expected_words = LABEL_TEXT_MAP.get(expected_label, [])

            if not args.quiet:
                print(f"[{idx}/{len(audio_files)}] {relative_path}")

            row = {
                "index": idx,
                "file": audio_path.name,
                "relative_path": str(relative_path),
                "expected_label": expected_label,
                "expected_words": "|".join(expected_words),
                "asr_text": "",
                "command_hits": "",
                "pass_by_label": "",
                "fail_reason": "",
                "duration_sec": "",
                "inference_time_sec": "",
                "error": "",
            }

            total_count += 1

            if expected_label not in label_stat:
                label_stat[expected_label] = {
                    "total": 0,
                    "pass": 0,
                    "fail": 0,
                    "error": 0,
                }

            label_stat[expected_label]["total"] += 1

            try:
                result = transcribe_one(pipe, audio_path)

                asr_text = result["text"]

                pass_by_label, fail_reason, command_hits = judge_result(
                    expected_label,
                    asr_text,
                )

                row.update({
                    "asr_text": asr_text,
                    "command_hits": command_hits,
                    "pass_by_label": pass_by_label,
                    "fail_reason": fail_reason,
                    "duration_sec": result["duration"],
                    "inference_time_sec": result["inference_time"],
                })

                if pass_by_label == "1":
                    pass_count += 1
                    label_stat[expected_label]["pass"] += 1
                elif pass_by_label == "0":
                    fail_count += 1
                    label_stat[expected_label]["fail"] += 1

                if not args.quiet:
                    print(f"    识别：{asr_text}")
                    print(f"    判断：{pass_by_label} {fail_reason}")
                    print(f"    推理耗时：{result['inference_time']} 秒")
                elif idx % 50 == 0 or pass_by_label == "0":
                    print(f"[{idx}/{len(audio_files)}] {relative_path} 判断={pass_by_label} {fail_reason}")

            except Exception as e:
                error_count += 1
                label_stat[expected_label]["error"] += 1

                row["error"] = repr(e)

                print(f"[{idx}/{len(audio_files)}] {relative_path} 错误：{repr(e)}")

            writer.writerow(row)

    elapsed = time.time() - start_time

    print()
    print("========== 识别完成 ==========")
    print(f"总数量：{total_count}")
    print(f"通过：{pass_count}")
    print(f"失败：{fail_count}")
    print(f"错误：{error_count}")

    if total_count > 0:
        print(f"总体通过率：{pass_count / total_count * 100:.2f}%")

    print()
    print("========== 各类别统计 ==========")

    for label, stat in sorted(label_stat.items()):
        total = stat["total"]
        passed = stat["pass"]
        failed = stat["fail"]
        errors = stat["error"]

        if total > 0:
            rate = passed / total * 100
        else:
            rate = 0.0

        print(
            f"{label:16s} "
            f"total={total:4d} "
            f"pass={passed:4d} "
            f"fail={failed:4d} "
            f"error={errors:4d} "
            f"pass_rate={rate:6.2f}%"
        )

    print()
    print(f"总耗时：{elapsed:.1f} 秒")
    print(f"CSV 结果已保存到：{OUTPUT_CSV}")


if __name__ == "__main__":
    main()
