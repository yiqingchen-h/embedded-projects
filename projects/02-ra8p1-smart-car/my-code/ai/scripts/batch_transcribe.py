import os
import csv
import argparse
import time
import string
from pathlib import Path
from statistics import mean



# ============================================================
# 1. 基本配置
# ============================================================

# 关闭 HuggingFace symlink 警告，不影响功能
os.environ["HF_HUB_DISABLE_SYMLINKS_WARNING"] = "1"

# 数据集根目录，选到 raw 这一层
AI_ROOT = Path(__file__).resolve().parents[1]
INPUT_DIR = AI_ROOT / "dataset_real" / "raw"

# 输出 CSV 文件
OUTPUT_CSV = None

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

# Whisper 模型大小
# 可选：tiny / base / small / medium / large-v3
# CPU 建议 small 或 base
MODEL_SIZE = "small"

# 中文识别
LANGUAGE = "zh"

# 设备：
# "auto" 自动选择
# "cpu" 强制 CPU
# "cuda" 仅 NVIDIA 显卡可用
DEVICE = "auto"

# 对很短的命令词，如果识别为空，可以改成 False 再跑一次
VAD_FILTER = True

# 搜索束宽，越大越准但越慢
BEAM_SIZE = 5

# 命令词识别提示
INITIAL_PROMPT = (
    "这是一个智能小车语音命令识别任务。"
    "可能出现的语音包括：你好小车，开始，停止，前进，后退，"
    "左平移，右平移，左转，右转，抓取方块，抓取圆柱，抓取圆球，"
    "抓河马，抓小猫，抓石墩子，抓水豚，抓玉米，抓蕉太狼，抓小狗，抓TNT。"
)


# ============================================================
# 2. 标签映射
# ============================================================

# 文件夹名 -> 期望识别到的中文内容
LABEL_TEXT_MAP = {
    "wake": ["你好小车"],

    "start": ["开始"],
    "stop": ["停止"],

    "forward": ["前进"],
    "backward": ["后退"],

    "move_left": ["左平移", "左移", "向左平移", "往左平移"],
    "move_right": ["右平移", "右移", "向右平移", "往右平移"],

    "turn_left": ["左转", "向左转"],
    "turn_right": ["右转", "向右转"],

    "select_cube": ["抓取方块", "方块"],
    "select_cylinder": ["抓取圆柱", "圆柱"],
    "select_sphere": ["抓取圆球", "圆球", "球"],
    "select_hippo": ["抓取河马", "抓河马", "河马"],
    "select_cat": ["抓取小猫", "抓小猫", "小猫", "猫"],
    "select_stone_pillar": ["抓取石墩子", "抓石墩子", "石墩子", "石墩"],
    "select_capybara": ["抓取水豚", "抓水豚", "水豚"],
    "select_corn": ["抓取玉米", "抓玉米", "玉米"],
    "select_jiaotailang": ["抓取蕉太狼", "抓蕉太狼", "蕉太狼"],
    "select_dog": ["抓取小狗", "抓小狗", "小狗", "狗"],
    "select_tnt": ["抓取TNT", "抓TNT", "抓 TNT", "TNT", "tnt"],

    # unknown 和 silence 不要求识别出固定文本
    # 它们的判断逻辑是：不应该命中任何有效命令
    "unknown": [],
    "silence": [],
}


# 所有有效命令关键词
# 用于判断 unknown / silence 是否误识别成命令
COMMAND_KEYWORDS = [
    "你好小车",

    "开始",
    "停止",

    "前进",
    "后退",

    "左平移",
    "右平移",
    "左移",
    "右移",

    "左转",
    "右转",

    "抓取方块",
    "抓取圆柱",
    "抓取圆球",
    "抓取河马",
    "抓河马",
    "抓取小猫",
    "抓小猫",
    "抓取石墩子",
    "抓石墩子",
    "抓取水豚",
    "抓水豚",
    "抓取玉米",
    "抓玉米",
    "抓取蕉太狼",
    "抓蕉太狼",
    "抓取小狗",
    "抓小狗",
    "抓取TNT",
    "抓TNT",
    "抓 TNT",

    "方块",
    "圆柱",
    "圆球",
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
    "TNT",
    "tnt",
]


# ============================================================
# 3. 工具函数
# ============================================================

def choose_device():
    """
    自动选择推理设备。
    faster-whisper 的 GPU 加速主要是 NVIDIA CUDA。
    Intel 独显不走 cuda，这里会自动回落 CPU。
    """
    if DEVICE == "cuda":
        return "cuda", "float16"

    if DEVICE == "cpu":
        return "cpu", "int8"

    try:
        import ctranslate2
        if ctranslate2.get_cuda_device_count() > 0:
            return "cuda", "float16"
    except Exception:
        pass

    return "cpu", "int8"


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
    简单清理识别文本，去掉空格和常见标点，方便关键词匹配。
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
    判断文本中命中了哪些关键词。
    """
    text_norm = normalize_text(text)

    hits = []
    for word in words:
        word_norm = normalize_text(word)
        if word_norm and word_norm in text_norm:
            hits.append(word)

    return hits


def judge_result(expected_label: str, asr_text: str):
    """
    判断当前音频是否识别正确。

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


def safe_mean(values):
    values = [v for v in values if v is not None]

    if not values:
        return ""

    return round(mean(values), 4)


def transcribe_one(model, audio_path: Path):
    """
    识别单个音频文件。
    """
    segments, info = model.transcribe(
        str(audio_path),
        language=LANGUAGE,
        vad_filter=VAD_FILTER,
        beam_size=BEAM_SIZE,
        initial_prompt=INITIAL_PROMPT,
        condition_on_previous_text=False,
    )

    text_parts = []
    avg_logprobs = []
    no_speech_probs = []

    for seg in segments:
        text_parts.append(seg.text.strip())

        if hasattr(seg, "avg_logprob"):
            avg_logprobs.append(seg.avg_logprob)

        if hasattr(seg, "no_speech_prob"):
            no_speech_probs.append(seg.no_speech_prob)

    text = "".join(text_parts).strip()

    result = {
        "text": text,
        "duration": round(getattr(info, "duration", 0.0), 3),
        "language": getattr(info, "language", ""),
        "language_probability": round(getattr(info, "language_probability", 0.0), 4),
        "avg_logprob": safe_mean(avg_logprobs),
        "avg_no_speech_prob": safe_mean(no_speech_probs),
    }

    return result


# ============================================================
# 4. 主程序
# ============================================================

def main():
    global INPUT_DIR, OUTPUT_CSV, MODEL_SIZE, DEVICE, WhisperModel
    parser = argparse.ArgumentParser(description="批量转写音频并检查命令词标签")
    parser.add_argument("--input-dir", type=Path, default=INPUT_DIR)
    parser.add_argument("--output-csv", type=Path, default=None)
    parser.add_argument("--model", default=MODEL_SIZE, help="faster-whisper 模型名或本地模型路径")
    parser.add_argument("--device", default=DEVICE, choices=("auto", "cpu", "cuda"))
    args = parser.parse_args()
    INPUT_DIR = args.input_dir
    OUTPUT_CSV = args.output_csv or INPUT_DIR / "transcribe_result_all.csv"
    MODEL_SIZE = args.model
    DEVICE = args.device
    if not INPUT_DIR.is_dir():
        parser.error(f"输入目录不存在: {INPUT_DIR}")
    from faster_whisper import WhisperModel
    device, compute_type = choose_device()

    print("========== 批量语音识别检查 ==========")
    print(f"输入目录：{INPUT_DIR}")
    print(f"输出文件：{OUTPUT_CSV}")
    print(f"模型：{MODEL_SIZE}")
    print(f"设备：{device}")
    print(f"计算类型：{compute_type}")
    print(f"VAD：{VAD_FILTER}")
    print("=====================================")

    audio_files = collect_audio_files(INPUT_DIR)

    print(f"发现音频数量：{len(audio_files)}")

    if not audio_files:
        print("没有找到音频文件。")
        return

    OUTPUT_CSV.parent.mkdir(parents=True, exist_ok=True)

    print("开始加载模型，第一次运行会下载模型，请等待...")
    model = WhisperModel(
        MODEL_SIZE,
        device=device,
        compute_type=compute_type,
    )
    print("模型加载完成，开始识别。")

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
        "language",
        "language_probability",
        "avg_logprob",
        "avg_no_speech_prob",
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
                "language": "",
                "language_probability": "",
                "avg_logprob": "",
                "avg_no_speech_prob": "",
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
                result = transcribe_one(model, audio_path)

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
                    "language": result["language"],
                    "language_probability": result["language_probability"],
                    "avg_logprob": result["avg_logprob"],
                    "avg_no_speech_prob": result["avg_no_speech_prob"],
                })

                if pass_by_label == "1":
                    pass_count += 1
                    label_stat[expected_label]["pass"] += 1
                elif pass_by_label == "0":
                    fail_count += 1
                    label_stat[expected_label]["fail"] += 1

                print(f"    识别：{asr_text}")
                print(f"    判断：{pass_by_label} {fail_reason}")

            except Exception as e:
                error_count += 1
                label_stat[expected_label]["error"] += 1

                row["error"] = repr(e)

                print(f"    错误：{repr(e)}")

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
    print(f"耗时：{elapsed:.1f} 秒")
    print(f"CSV 结果已保存到：{OUTPUT_CSV}")


if __name__ == "__main__":
    main()
