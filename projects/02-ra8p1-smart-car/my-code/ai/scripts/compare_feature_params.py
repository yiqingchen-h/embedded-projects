"""对比两组 FEATURE_MEAN/STD 在全部 raw 数据上的 TFLite 推理准确率。"""

import argparse
import re
import sys
import wave
from collections import defaultdict
from pathlib import Path

import numpy as np

AI_ROOT = Path(__file__).resolve().parents[1]
PROJECT_DIR = AI_ROOT.parents[1]
VOICE_AI_C = PROJECT_DIR / "my-code" / "software" / "voice_ai.c"
RUHMI_SRC_DIR = PROJECT_DIR / "my-code" / "ai" / "ruhmi_generated" / "kws_current" / "output" / "build" / "MCU" / "compilation" / "src"
COMPUTE_SUB_0000 = RUHMI_SRC_DIR / "compute_sub_0000.c"
REPORT_PATH = AI_ROOT / "artifacts_real" / "tflite_training_report.md"

# ── 参数 ──────────────────────────────────────────────
SAMPLE_RATE = 16000
FRAME_LENGTH = 400
FRAME_STEP = 160
N_FFT = 512
MEL_BINS = 40
FRAME_COUNT = 98
TARGET_PEAK = 0.75
PREEMPHASIS = 0.97
LOG_FLOOR = 1.0e-8
LOW_HZ = 20.0
HIGH_HZ = 7600.0

# 对照参数，默认保留上一轮曾上板验证过的参数。
PARAM_A_DEFAULT = {"name": "上一轮参数 (-2.510/3.886)", "mean": -2.50970078, "std": 3.88595586}

TFLITE_PATH = AI_ROOT / "artifacts_real" / "model_int8.tflite"
LABELS_PATH = AI_ROOT / "labels.txt"
DEFAULT_DATASET_DIR = AI_ROOT / "dataset_real" / "combined_1s"


def parse_report_value(report_text: str, key: str) -> str:
    pattern = re.compile(rf"^- {re.escape(key)}:\s*(.+?)\s*$", re.MULTILINE)
    match = pattern.search(report_text)
    if match is None:
        raise RuntimeError(f"训练报告缺少字段: {key}")
    return match.group(1)


def load_report_feature_params():
    report_text = REPORT_PATH.read_text(encoding="utf-8")
    return {
        "name": "训练报告参数",
        "mean": float(parse_report_value(report_text, "feature_mean")),
        "std": float(parse_report_value(report_text, "feature_std")),
    }


def read_c_without_line_comments(path: Path) -> str:
    return re.sub(r"//.*", "", path.read_text(encoding="utf-8"))


def parse_affine_quantize(path: Path) -> tuple[float, int, int]:
    text = read_c_without_line_comments(path)
    pattern = r"AffineQuantizeFloatToInt8\s*\(\s*[^,]+,\s*[^,]+,\s*(\d+)\s*,\s*(-?\d+)\s*,\s*([0-9.eE+-]+)\s*\)"
    match = re.search(pattern, text, flags=re.S)
    if not match:
        raise RuntimeError(f"未能从 {path} 解析 AffineQuantizeFloatToInt8 参数")
    size = int(match.group(1))
    zero_point = int(match.group(2))
    scale = float(match.group(3))
    return scale, zero_point, size


def load_labels():
    labels = []
    for line in LABELS_PATH.read_text(encoding="utf-8").splitlines():
        label = line.strip()
        if label and not label.startswith("#"):
            labels.append(label)
    return labels


def hz_to_mel(hz):
    return 2595.0 * np.log10(1.0 + np.asarray(hz) / 700.0)


def mel_to_hz(mel):
    return 700.0 * (10.0 ** (np.asarray(mel) / 2595.0) - 1.0)


def make_mel_filterbank():
    low_mel = hz_to_mel(LOW_HZ)
    high_mel = hz_to_mel(HIGH_HZ)
    mel_points = np.linspace(low_mel, high_mel, MEL_BINS + 2)
    hz_points = mel_to_hz(mel_points)
    bins = np.floor((N_FFT + 1) * hz_points / SAMPLE_RATE).astype(int)
    bins = np.clip(bins, 0, N_FFT // 2)
    filters = np.zeros((MEL_BINS, N_FFT // 2 + 1), dtype=np.float32)
    for m in range(1, MEL_BINS + 1):
        left, center, right = bins[m - 1], bins[m], bins[m + 1]
        if center <= left: center = left + 1
        if right <= center: right = center + 1
        right = min(right, N_FFT // 2)
        for k in range(left, min(center, filters.shape[1])):
            filters[m - 1, k] = (k - left) / max(center - left, 1)
        for k in range(center, min(right, filters.shape[1])):
            filters[m - 1, k] = (right - k) / max(right - center, 1)
    return filters


def read_wav_int16(path):
    with wave.open(str(path), "rb") as wf:
        channels = wf.getnchannels()
        sw = wf.getsampwidth()
        sr = wf.getframerate()
        data = wf.readframes(wf.getnframes())
    if sw != 2:
        raise RuntimeError(f"非 PCM16: {path}")
    pcm = np.frombuffer(data, dtype=np.int16).copy()
    if channels > 1:
        pcm = pcm.reshape(-1, channels).mean(axis=1).astype(np.int16)
    return pcm.astype(np.float32), int(sr)


def extract_features_mcu(pcm_f32, sr, feature_mean, feature_std):
    """复刻 MCU voice_ai.c 特征提取，使用指定 mean/std。"""
    audio = pcm_f32 / 32767.0
    if audio.size > SAMPLE_RATE:
        audio = audio[:SAMPLE_RATE]
    elif audio.size < SAMPLE_RATE:
        audio = np.pad(audio, (0, SAMPLE_RATE - audio.size))

    peak = float(np.max(np.abs(audio))) if audio.size else 0.0
    if peak >= 1.0e-6:
        audio = audio * (TARGET_PEAK / peak)

    preemph = np.empty_like(audio)
    preemph[0] = audio[0]
    preemph[1:] = audio[1:] - PREEMPHASIS * audio[:-1]

    window = 0.5 - 0.5 * np.cos(2.0 * np.pi * np.arange(FRAME_LENGTH) / (FRAME_LENGTH - 1))
    mel_filters = make_mel_filterbank()
    features = np.zeros((FRAME_COUNT, MEL_BINS), dtype=np.float32)

    for frame in range(FRAME_COUNT):
        offset = frame * FRAME_STEP
        fd = np.zeros(N_FFT, dtype=np.float32)
        fd[:FRAME_LENGTH] = preemph[offset:offset + FRAME_LENGTH] * window
        spectrum = np.fft.rfft(fd, n=N_FFT)
        power = np.real(spectrum * np.conj(spectrum))
        mel_energy = np.maximum(power @ mel_filters.T, LOG_FLOOR)
        log_mel = np.log(mel_energy)
        features[frame, :] = (log_mel - feature_mean) / feature_std

    return features


def parse_args():
    parser = argparse.ArgumentParser(description="对比两组 FEATURE_MEAN/STD 在 TFLite 路径上的推理准确率")
    parser.add_argument("--dataset-dir", type=Path, default=DEFAULT_DATASET_DIR, help="验证数据目录")
    parser.add_argument("--param-a-name", default=PARAM_A_DEFAULT["name"], help="对照参数组名称")
    parser.add_argument("--param-a-mean", type=float, default=PARAM_A_DEFAULT["mean"], help="对照参数组 mean")
    parser.add_argument("--param-a-std", type=float, default=PARAM_A_DEFAULT["std"], help="对照参数组 std")
    return parser.parse_args()


def main():
    args = parse_args()
    param_a = {"name": args.param_a_name, "mean": args.param_a_mean, "std": args.param_a_std}
    param_b = load_report_feature_params()
    raw_dir = args.dataset_dir

    labels = load_labels()
    print(f"标签数: {len(labels)}  -> {labels}")
    print(f"TFLite 模型: {TFLITE_PATH}")
    print(f"数据目录: {raw_dir}")
    print(f"对照参数: {param_a['name']} mean={param_a['mean']:.8f} std={param_a['std']:.8f}")
    print(f"训练报告: {param_b['name']} mean={param_b['mean']:.8f} std={param_b['std']:.8f}")

    # 从 MCU 源码加载量化参数（仅用于参考打印）
    ruhmi_scale, ruhmi_zp, ruhmi_size = parse_affine_quantize(COMPUTE_SUB_0000)
    print(f"RUHMI compute_sub_0000.c 量化参数: scale={ruhmi_scale:.17g} zp={ruhmi_zp} size={ruhmi_size}")

    # 加载 INT8 TFLite 模型，使用模型自带的量化参数
    try:
        import tflite_runtime.interpreter as tflite
    except ImportError:
        import tensorflow.lite as tflite

    interpreter = tflite.Interpreter(model_path=str(TFLITE_PATH))
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    input_index = input_details[0]["index"]
    output_index = output_details[0]["index"]
    input_shape = input_details[0]["shape"]
    # 使用 TFLite 模型自带的量化参数，而不是 RUHMI 的
    quant_scale, quant_zp = input_details[0]["quantization"]
    print(f"模型输入: {input_shape} dtype={input_details[0]['dtype']}")
    print(f"TFLite 量化参数: scale={quant_scale:.17g} zp={quant_zp}")
    if abs(quant_scale - ruhmi_scale) > 1e-6 or quant_zp != ruhmi_zp:
        print(f"[!] TFLite 与 RUHMI 量化参数不一致！MCU 实际使用 RUHMI 参数。")
        print(f"    本脚本使用 TFLite 参数验证模型本身，MCU 部署需以 RUHMI 为准。")

    # 收集全部 raw WAV
    all_files = []
    for label in labels:
        label_dir = raw_dir / label
        if not label_dir.exists():
            print(f"  目录不存在: {label_dir}")
            continue
        for wav in sorted(label_dir.glob("*.wav")):
            all_files.append((label, wav))

    print(f"总文件数: {len(all_files)}")

    # 统计
    stats_a = defaultdict(lambda: {"total": 0, "correct": 0})
    stats_b = defaultdict(lambda: {"total": 0, "correct": 0})
    # 额外：记录两组都错/都对/只有一组对的
    both_correct = 0
    both_wrong = 0
    only_a = 0
    only_b = 0

    for idx, (expected, wav_path) in enumerate(all_files):
        try:
            pcm, sr = read_wav_int16(wav_path)
        except Exception as e:
            print(f"[{idx+1}/{len(all_files)}] ERROR {wav_path}: {e}")
            continue

        expected_idx = labels.index(expected)

        top_a = -1
        top_b = -1
        a_correct = False
        b_correct = False

        # 参数组 A（旧）
        try:
            feats_a = extract_features_mcu(pcm, sr, param_a["mean"], param_a["std"])
            q_a = np.clip(np.round(feats_a.flatten() / quant_scale + quant_zp), -128, 127).astype(np.int8)
            interpreter.set_tensor(input_index, q_a.reshape(input_shape))
            interpreter.invoke()
            out_a = interpreter.get_tensor(output_index)[0]
            top_a = int(np.argmax(out_a))
            a_correct = (top_a == expected_idx)
        except Exception as e:
            a_correct = False

        # 参数组 B（训练报告）
        try:
            feats_b = extract_features_mcu(pcm, sr, param_b["mean"], param_b["std"])
            q_b = np.clip(np.round(feats_b.flatten() / quant_scale + quant_zp), -128, 127).astype(np.int8)
            interpreter.set_tensor(input_index, q_b.reshape(input_shape))
            interpreter.invoke()
            out_b = interpreter.get_tensor(output_index)[0]
            top_b = int(np.argmax(out_b))
            b_correct = (top_b == expected_idx)
        except Exception as e:
            b_correct = False

        stats_a[expected]["total"] += 1
        stats_b[expected]["total"] += 1
        if a_correct: stats_a[expected]["correct"] += 1
        if b_correct: stats_b[expected]["correct"] += 1

        if a_correct and b_correct:
            both_correct += 1
        elif not a_correct and not b_correct:
            both_wrong += 1
        elif a_correct and not b_correct:
            only_a += 1
        else:
            only_b += 1

        if (idx + 1) % 100 == 0 or idx == 0:
            print(f"[{idx+1}/{len(all_files)}] 进度...")

    # ── 打印结果 ──
    print()
    print("=" * 80)
    print(f"{'类别':<18} {'总数':>5} {param_a['name']+' 正确':>25} {param_b['name']+' 正确':>25} {'差值':>8}")
    print("-" * 80)
    total_a_correct = 0
    total_b_correct = 0
    total_all = 0
    for label in labels:
        if stats_a[label]["total"] == 0:
            continue
        ta = stats_a[label]["total"]
        ca = stats_a[label]["correct"]
        cb = stats_b[label]["correct"]
        ra = ca / ta * 100 if ta > 0 else 0
        rb = cb / ta * 100 if ta > 0 else 0
        diff = rb - ra
        print(f"{label:<18} {ta:>5} {ca:>4}/{ta:<4} ({ra:5.1f}%)      {cb:>4}/{ta:<4} ({rb:5.1f}%)      {diff:>+7.1f}%")
        total_a_correct += ca
        total_b_correct += cb
        total_all += ta
    print("-" * 80)
    ra_all = total_a_correct / total_all * 100 if total_all > 0 else 0
    rb_all = total_b_correct / total_all * 100 if total_all > 0 else 0
    print(f"{'总计':<18} {total_all:>5} {total_a_correct:>4}/{total_all:<4} ({ra_all:5.1f}%)      {total_b_correct:>4}/{total_all:<4} ({rb_all:5.1f}%)      {rb_all-ra_all:>+7.1f}%")
    print()
    print(f"两组都对: {both_correct}")
    print(f"两组都错: {both_wrong}")
    print(f"仅 {param_a['name']} 对: {only_a}")
    print(f"仅 {param_b['name']} 对: {only_b}")


if __name__ == "__main__":
    main()
