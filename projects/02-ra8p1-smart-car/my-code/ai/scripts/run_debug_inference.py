"""对 debug_model_input 下的 WAV 文件跑 TFLite 模型推理，打印 Top-3 结果。"""

import sys
import argparse
from pathlib import Path

AI_ROOT = Path(__file__).resolve().parents[1]

# ── 当前支持的音频特征参数 ──────────────────────
SAMPLE_RATE = 16000
DURATION_SECONDS = 1.0
SAMPLE_COUNT = 16000
FRAME_LENGTH = 400
FRAME_STEP = 160
N_FFT = 512
MEL_BINS = 40
LOW_HZ = 20.0
HIGH_HZ = 7600.0
TARGET_PEAK = 0.75
PRE_EMPHASIS = 0.97

TFLITE_PATH = AI_ROOT / "artifacts_real" / "model_float.tflite"
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
        left = bins[m - 1]
        center = bins[m]
        right = bins[m + 1]
        if center <= left:
            center = left + 1
        if right <= center:
            right = center + 1
        right = min(right, N_FFT // 2)
        for k in range(left, min(center, filters.shape[1])):
            filters[m - 1, k] = (k - left) / max(center - left, 1)
        for k in range(center, min(right, filters.shape[1])):
            filters[m - 1, k] = (right - k) / max(right - center, 1)
    return filters


def read_wav_mono(path):
    """读取 WAV，返回 float32 [-1, 1]。"""
    from scipy.io import wavfile
    sr, data = wavfile.read(str(path))
    if data.ndim > 1:
        data = data.mean(axis=1)
    if np.issubdtype(data.dtype, np.integer):
        max_value = float(np.iinfo(data.dtype).max)
        audio = data.astype(np.float32) / max_value
    else:
        audio = data.astype(np.float32)
    return int(sr), np.clip(audio, -1.0, 1.0)


def preprocess(audio, feature_mean, feature_std):
    """生成当前 log-mel 输入，使用所选模型清单的归一化参数。"""
    # 1. 补齐或截断到 16000 点；采样率转换在调用前完成
    if len(audio) != SAMPLE_COUNT:
        if len(audio) > SAMPLE_COUNT:
            audio = audio[:SAMPLE_COUNT]
        else:
            audio = np.pad(audio, (0, SAMPLE_COUNT - len(audio)))

    audio = audio.astype(np.float32)

    # 2. 峰值归一化到 0.75
    peak = float(np.max(np.abs(audio)))
    if peak > 1e-6:
        audio = audio * (TARGET_PEAK / peak)

    # 3. 预加重
    audio = signal.lfilter([1.0, -PRE_EMPHASIS], [1.0], audio).astype(np.float32)

    # 4. 分帧 + Hann 窗
    frame_count = 1 + (len(audio) - FRAME_LENGTH) // FRAME_STEP
    frames = np.lib.stride_tricks.sliding_window_view(audio, FRAME_LENGTH)[::FRAME_STEP][:frame_count]
    window = np.hanning(FRAME_LENGTH).astype(np.float32)
    framed = frames * window

    # 5. FFT → 功率谱
    spectrum = np.fft.rfft(framed, n=N_FFT)
    power = (np.abs(spectrum) ** 2).astype(np.float32)

    # 6. Mel 滤波 → log
    mel_filters = make_mel_filterbank()
    mel = np.maximum(power @ mel_filters.T, 1e-8)
    log_mel = np.log(mel).astype(np.float32)

    # 7. 标准化（与训练一致）
    log_mel = (log_mel - feature_mean) / feature_std

    # 8. 扩展为 (1, 98, 40, 1)
    return log_mel.reshape(1, frame_count, MEL_BINS, 1)


def main():
    parser = argparse.ArgumentParser(description="使用配套清单对 WAV 进行 KWS 调试推理")
    parser.add_argument("--model-path", type=Path, default=TFLITE_PATH)
    parser.add_argument("--model-manifest", type=Path, default=None, help="默认读取模型同目录的 kws_model_manifest.json")
    parser.add_argument("--wav-dir", type=Path, default=AI_ROOT / "debug_model_input" / "voice_ai_debug")
    args = parser.parse_args()
    global np, signal
    import numpy as np
    from scipy import signal
    from kws_common import load_model_manifest, validate_model_tensors, resample_audio
    manifest = load_model_manifest(args.model_path, args.model_manifest)
    target_files = sorted(args.wav_dir.glob("*.wav"))
    if not target_files:
        raise FileNotFoundError(f"没有找到 WAV 文件: {args.wav_dir}")

    # 加载 TFLite 模型
    import tensorflow as tf
    interpreter = tf.lite.Interpreter(model_content=args.model_path.read_bytes())
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    validate_model_tensors(input_details, output_details, manifest)

    print(f"模型输入: {input_details[0]['shape']} {input_details[0]['dtype']}")
    print(f"模型输出: {output_details[0]['shape']} {output_details[0]['dtype']}")
    print()

    labels = manifest["labels"]
    print(f"标签 ({len(labels)}): {labels}")
    print(f"模型: {args.model_path}  SHA-256={manifest['models'][args.model_path.name]}")
    print(f"标准化参数: mean={manifest['feature_mean']}, std={manifest['feature_std']}")
    print()

    for wav_path in target_files:
        if not wav_path.exists():
            print(f"跳过不存在: {wav_path.name}")
            continue

        sr, audio = read_wav_mono(wav_path)
        print(f"文件: {wav_path.name}  sr={sr}  样本数={len(audio)}  peak={np.max(np.abs(audio)):.3f}")

        audio = resample_audio(audio, sr, SAMPLE_RATE)
        features = preprocess(audio, manifest["feature_mean"], manifest["feature_std"])
        print(f"  特征 shape={features.shape}  range=[{features.min():.3f}, {features.max():.3f}]")

        # 推理
        interpreter.set_tensor(input_details[0]['index'], features)
        interpreter.invoke()
        output = interpreter.get_tensor(output_details[0]['index'])[0]

        # Top-3
        top3_idx = np.argsort(output)[::-1][:3]
        print(f"  Top-1: {labels[top3_idx[0]]:<16s} score={output[top3_idx[0]]:.4f}")
        print(f"  Top-3: {labels[top3_idx[0]]}={output[top3_idx[0]]:.4f}  "
              f"{labels[top3_idx[1]]}={output[top3_idx[1]]:.4f}  "
              f"{labels[top3_idx[2]]}={output[top3_idx[2]]:.4f}")
        print()


if __name__ == "__main__":
    main()
