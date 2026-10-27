"""验证 PC 端复刻的 MCU KWS 特征提取和当前量化参数。"""

from __future__ import annotations

import argparse
import random
import re
import wave
from pathlib import Path

import numpy as np

PROJECT_DIR = Path(__file__).resolve().parents[1]
VOICE_AI_C = PROJECT_DIR / "my-code" / "software" / "voice_ai.c"
RUHMI_SRC_DIR = PROJECT_DIR / "my-code" / "ai" / "ruhmi_generated" / "kws_current" / "output" / "build" / "MCU" / "compilation" / "src"
COMPUTE_SUB_0000 = RUHMI_SRC_DIR / "compute_sub_0000.c"
DEFAULT_DATASET_DIR = PROJECT_DIR / "my-code" / "ai" / "dataset_real" / "combined_1s"
LABELS_PATH = PROJECT_DIR / "my-code" / "ai" / "labels.txt"

SAMPLE_RATE = 16000
FRAME_LENGTH = 400
FRAME_STEP = 160
FFT_LENGTH = 512
MEL_BINS = 40
FRAME_COUNT = 98
TARGET_PEAK = 0.75
PREEMPHASIS = 0.97
LOG_FLOOR = 1.0e-8
LOW_HZ = 20.0
HIGH_HZ = 7600.0


def parse_macro_float(text: str, macro: str) -> float:
    pattern = rf"^\s*#define\s+{re.escape(macro)}\s+\(\s*([-+0-9.eE]+)f?\s*\)\s*$"
    match = re.search(pattern, text, flags=re.M)
    if not match:
        raise RuntimeError(f"未能从 voice_ai.c 解析 {macro}")
    return float(match.group(1))


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


def load_current_feature_params() -> tuple[float, float]:
    text = VOICE_AI_C.read_text(encoding="utf-8")
    return (
        parse_macro_float(text, "VOICE_AI_FEATURE_MEAN"),
        parse_macro_float(text, "VOICE_AI_FEATURE_STD"),
    )


FEATURE_MEAN, FEATURE_STD = load_current_feature_params()
QUANT_SCALE, QUANT_ZERO_POINT, QUANT_SIZE = parse_affine_quantize(COMPUTE_SUB_0000)


def load_labels() -> list[str]:
    labels = []
    for line in LABELS_PATH.read_text(encoding="utf-8").splitlines():
        label = line.strip()
        if label and not label.startswith("#"):
            labels.append(label)
    return labels


def read_wav_int16(path: str | Path) -> tuple[np.ndarray, int]:
    """读取 PCM16 WAV，返回 int16 PCM 和采样率。"""

    with wave.open(str(path), "rb") as wav_file:
        channels = wav_file.getnchannels()
        sample_width = wav_file.getsampwidth()
        sample_rate = wav_file.getframerate()
        frame_count = wav_file.getnframes()
        data = wav_file.readframes(frame_count)

    if sample_width != 2:
        raise RuntimeError(f"只支持 PCM16 WAV: {path}")

    pcm = np.frombuffer(data, dtype=np.int16)
    if channels > 1:
        pcm = pcm.reshape(-1, channels).mean(axis=1).astype(np.int16)

    return pcm.astype(np.float64), int(sample_rate)


def hz_to_mel(hz: float | np.ndarray) -> float | np.ndarray:
    return 2595.0 * np.log10(1.0 + np.asarray(hz) / 700.0)


def mel_to_hz(mel: float | np.ndarray) -> float | np.ndarray:
    return 700.0 * (10.0 ** (np.asarray(mel) / 2595.0) - 1.0)


def make_mel_filterbank() -> np.ndarray:
    low_mel = hz_to_mel(LOW_HZ)
    high_mel = hz_to_mel(HIGH_HZ)
    mel_points = np.linspace(low_mel, high_mel, MEL_BINS + 2)
    hz_points = mel_to_hz(mel_points)
    bins = np.floor((FFT_LENGTH + 1) * hz_points / SAMPLE_RATE).astype(int)
    bins = np.clip(bins, 0, FFT_LENGTH // 2)

    filters = np.zeros((MEL_BINS, FFT_LENGTH // 2 + 1), dtype=np.float64)
    for mel in range(1, MEL_BINS + 1):
        left = bins[mel - 1]
        center = bins[mel]
        right = bins[mel + 1]
        if center <= left:
            center = left + 1
        if right <= center:
            right = center + 1
        right = min(right, FFT_LENGTH // 2)
        for index in range(left, min(center, filters.shape[1])):
            filters[mel - 1, index] = (index - left) / max(center - left, 1)
        for index in range(center, min(right, filters.shape[1])):
            filters[mel - 1, index] = (right - index) / max(right - center, 1)
    return filters


def extract_features_mcu(pcm_int16: np.ndarray, sr: int) -> np.ndarray:
    """复刻 MCU voice_ai.c 的 1 秒 log-mel 特征提取。"""

    if sr != SAMPLE_RATE:
        raise RuntimeError(f"采样率必须为 {SAMPLE_RATE}, got={sr}")

    audio = pcm_int16.astype(np.float64) / 32767.0
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
    features = np.zeros((FRAME_COUNT, MEL_BINS), dtype=np.float64)

    for frame in range(FRAME_COUNT):
        offset = frame * FRAME_STEP
        frame_data = np.zeros(FFT_LENGTH, dtype=np.float64)
        frame_data[:FRAME_LENGTH] = preemph[offset:offset + FRAME_LENGTH] * window
        spectrum = np.fft.rfft(frame_data, n=FFT_LENGTH)
        power = np.real(spectrum * np.conj(spectrum))
        mel_energy = np.maximum(power @ mel_filters.T, LOG_FLOOR)
        log_mel = np.log(mel_energy)
        features[frame, :] = (log_mel - FEATURE_MEAN) / FEATURE_STD

    return features


def quantize_features(features: np.ndarray) -> np.ndarray:
    flat = features.flatten().astype(np.float64)
    quantized = np.round(flat / QUANT_SCALE + QUANT_ZERO_POINT)
    return np.clip(quantized, -128, 127).astype(np.int8)


def build_dataset_tests(dataset_dir: Path, samples_per_class: int, seed: int) -> list[tuple[str, Path]]:
    rng = random.Random(seed)
    tests: list[tuple[str, Path]] = []
    for label in load_labels():
        wav_files = sorted((dataset_dir / label).glob("*.wav"))
        if not wav_files:
            continue
        for path in rng.sample(wav_files, min(samples_per_class, len(wav_files))):
            tests.append((label, path))
    return tests


def verify_one(label: str, wav_path: Path) -> bool:
    pcm, sr = read_wav_int16(wav_path)
    features = extract_features_mcu(pcm, sr)
    quantized = quantize_features(features)

    ok = (
        features.shape == (FRAME_COUNT, MEL_BINS)
        and np.isfinite(features).all()
        and quantized.size == QUANT_SIZE
        and quantized.min() >= -128
        and quantized.max() <= 127
    )

    status = "PASS" if ok else "FAIL"
    print(
        f"{status} {label}/{wav_path.name}: "
        f"feature_min={features.min():.6f} feature_max={features.max():.6f} "
        f"feature_mean={features.mean():.6f} q_sum={int(quantized.astype(np.int32).sum())} "
        f"q_min={int(quantized.min())} q_max={int(quantized.max())}"
    )
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description="验证 MCU 特征提取参数和量化流程")
    parser.add_argument("--dataset-dir", type=Path, default=DEFAULT_DATASET_DIR, help="抽样 WAV 数据集目录")
    parser.add_argument("--wav", type=Path, default=None, help="只验证单个 WAV")
    parser.add_argument("--label", default="sample", help="单个 WAV 的标签显示名")
    parser.add_argument("--samples-per-class", type=int, default=1, help="每类抽样数量")
    parser.add_argument("--seed", type=int, default=42, help="随机种子")
    args = parser.parse_args()

    print("=" * 60)
    print("MCU 特征提取 + 量化一致性验证")
    print("=" * 60)
    print(f"feature_mean={FEATURE_MEAN:.8f} feature_std={FEATURE_STD:.8f}")
    print(f"quant_scale={QUANT_SCALE:.17g} quant_zero_point={QUANT_ZERO_POINT} quant_size={QUANT_SIZE}")

    if args.wav is not None:
        tests = [(args.label, args.wav)]
    else:
        tests = build_dataset_tests(args.dataset_dir, args.samples_per_class, args.seed)

    if not tests:
        print("没有可验证的 WAV。")
        return 2

    all_ok = True
    for label, wav_path in tests:
        all_ok = verify_one(label, wav_path) and all_ok

    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
