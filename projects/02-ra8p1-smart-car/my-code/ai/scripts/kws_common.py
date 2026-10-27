"""PC 端 KWS 工具公共函数。"""

from __future__ import annotations

import json
import math
import os
import random
import wave
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.io import wavfile
from scipy import signal


AI_ROOT = Path(__file__).resolve().parents[1]
LABELS_PATH = AI_ROOT / "labels.txt"
DATASET_ROOT = AI_ROOT / "dataset_tts"
RAW_DIR = DATASET_ROOT / "raw"
FEATURES_DIR = DATASET_ROOT / "features"
ARTIFACTS_DIR = AI_ROOT / "artifacts"
CACHE_DIR = AI_ROOT / "_cache"

SAMPLE_RATE = 16000
DURATION_SECONDS = 1.0
SAMPLE_COUNT = int(SAMPLE_RATE * DURATION_SECONDS)
RANDOM_SEED = 20260503

FRAME_LENGTH = 400
FRAME_STEP = 160
N_FFT = 512
MEL_BINS = 40
LOW_HZ = 20.0
HIGH_HZ = 7600.0


@dataclass(frozen=True)
class PreprocessConfig:
    sample_rate: int = SAMPLE_RATE
    duration_seconds: float = DURATION_SECONDS
    sample_count: int = SAMPLE_COUNT
    frame_length: int = FRAME_LENGTH
    frame_step: int = FRAME_STEP
    n_fft: int = N_FFT
    mel_bins: int = MEL_BINS
    low_hz: float = LOW_HZ
    high_hz: float = HIGH_HZ
    feature_type: str = "log_mel"

    def to_dict(self) -> dict:
        return {
            "sample_rate": self.sample_rate,
            "duration_seconds": self.duration_seconds,
            "sample_count": self.sample_count,
            "frame_length": self.frame_length,
            "frame_step": self.frame_step,
            "n_fft": self.n_fft,
            "mel_bins": self.mel_bins,
            "low_hz": self.low_hz,
            "high_hz": self.high_hz,
            "feature_type": self.feature_type,
        }


def configure_local_cache() -> None:
    """把 COM/TTS 缓存放到工程目录，避免写 site-packages 失败。"""

    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("COMTYPES_CACHE", str(CACHE_DIR / "comtypes"))
    os.environ.setdefault("PYTTSX3_CACHE", str(CACHE_DIR / "pyttsx3"))


def ensure_output_dirs() -> None:
    RAW_DIR.mkdir(parents=True, exist_ok=True)
    FEATURES_DIR.mkdir(parents=True, exist_ok=True)
    ARTIFACTS_DIR.mkdir(parents=True, exist_ok=True)
    CACHE_DIR.mkdir(parents=True, exist_ok=True)


def load_labels() -> list[str]:
    labels = []
    for line in LABELS_PATH.read_text(encoding="utf-8").splitlines():
        label = line.strip()
        if label and not label.startswith("#"):
            labels.append(label)
    if not labels:
        raise RuntimeError(f"标签文件为空: {LABELS_PATH}")
    return labels


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_wav_mono(path: Path) -> tuple[int, np.ndarray]:
    sr, data = wavfile.read(str(path))
    if data.ndim > 1:
        data = data.mean(axis=1)
    if np.issubdtype(data.dtype, np.integer):
        max_value = float(np.iinfo(data.dtype).max)
        audio = data.astype(np.float32) / max_value
    else:
        audio = data.astype(np.float32)
    return int(sr), np.clip(audio, -1.0, 1.0)


def write_wav_mono(path: Path, audio: np.ndarray, sample_rate: int = SAMPLE_RATE) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    audio = np.asarray(audio, dtype=np.float32)
    audio = np.clip(audio, -1.0, 1.0)
    pcm = (audio * 32767.0).astype(np.int16)
    wavfile.write(str(path), sample_rate, pcm)


def resample_audio(audio: np.ndarray, from_rate: int, to_rate: int = SAMPLE_RATE) -> np.ndarray:
    if from_rate == to_rate:
        return audio.astype(np.float32, copy=False)
    gcd = math.gcd(int(from_rate), int(to_rate))
    up = to_rate // gcd
    down = from_rate // gcd
    return signal.resample_poly(audio, up, down).astype(np.float32)


def trim_silence(audio: np.ndarray, threshold: float = 0.01, margin: int = 1200) -> np.ndarray:
    active = np.flatnonzero(np.abs(audio) > threshold)
    if active.size == 0:
        return audio
    start = max(int(active[0]) - margin, 0)
    end = min(int(active[-1]) + margin + 1, len(audio))
    return audio[start:end]


def fit_to_one_second(audio: np.ndarray, rng: random.Random | None = None) -> np.ndarray:
    audio = np.asarray(audio, dtype=np.float32)
    if len(audio) > SAMPLE_COUNT:
        start = (len(audio) - SAMPLE_COUNT) // 2
        return audio[start:start + SAMPLE_COUNT]

    result = np.zeros(SAMPLE_COUNT, dtype=np.float32)
    if len(audio) == 0:
        return result

    max_offset = SAMPLE_COUNT - len(audio)
    if rng is None:
        offset = max_offset // 2
    else:
        center = max_offset // 2
        jitter = min(max_offset // 3, int(0.12 * SAMPLE_RATE))
        offset = max(0, min(max_offset, center + rng.randint(-jitter, jitter)))
    result[offset:offset + len(audio)] = audio
    return result


def normalize_peak(audio: np.ndarray, target_peak: float = 0.75) -> np.ndarray:
    peak = float(np.max(np.abs(audio))) if len(audio) else 0.0
    if peak < 1e-6:
        return audio.astype(np.float32, copy=False)
    return (audio * (target_peak / peak)).astype(np.float32)


def add_noise(audio: np.ndarray, rng: np.random.Generator, noise_level: float) -> np.ndarray:
    if noise_level <= 0:
        return audio.astype(np.float32, copy=False)
    noise = rng.normal(0.0, noise_level, size=audio.shape).astype(np.float32)
    return np.clip(audio + noise, -1.0, 1.0)


def load_and_prepare_audio(path: Path) -> np.ndarray:
    sr, audio = read_wav_mono(path)
    audio = resample_audio(audio, sr, SAMPLE_RATE)
    audio = trim_silence(audio)
    audio = fit_to_one_second(audio)
    return normalize_peak(audio, 0.75)


def hz_to_mel(hz: float | np.ndarray) -> float | np.ndarray:
    return 2595.0 * np.log10(1.0 + np.asarray(hz) / 700.0)


def mel_to_hz(mel: float | np.ndarray) -> float | np.ndarray:
    return 700.0 * (10.0 ** (np.asarray(mel) / 2595.0) - 1.0)


def make_mel_filterbank() -> np.ndarray:
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


def log_mel_features(audio: np.ndarray, mel_filters: np.ndarray | None = None) -> np.ndarray:
    audio = fit_to_one_second(audio)
    audio = audio.astype(np.float32, copy=False)
    audio = signal.lfilter([1.0, -0.97], [1.0], audio).astype(np.float32)

    if len(audio) < FRAME_LENGTH:
        audio = np.pad(audio, (0, FRAME_LENGTH - len(audio)))

    frame_count = 1 + (len(audio) - FRAME_LENGTH) // FRAME_STEP
    frames = np.lib.stride_tricks.sliding_window_view(audio, FRAME_LENGTH)[::FRAME_STEP][:frame_count]
    window = np.hanning(FRAME_LENGTH).astype(np.float32)
    framed = frames * window
    spectrum = np.fft.rfft(framed, n=N_FFT)
    power = (np.abs(spectrum) ** 2).astype(np.float32)

    if mel_filters is None:
        mel_filters = make_mel_filterbank()
    mel = np.maximum(power @ mel_filters.T, 1e-8)
    log_mel = np.log(mel).astype(np.float32)
    return log_mel


def iter_wav_files(raw_dir: Path = RAW_DIR) -> Iterable[tuple[str, Path]]:
    labels = load_labels()
    for label in labels:
        label_dir = raw_dir / label
        if not label_dir.exists():
            continue
        for path in sorted(label_dir.glob("*.wav")):
            yield label, path


def wav_duration_seconds(path: Path) -> float:
    with wave.open(str(path), "rb") as wf:
        return wf.getnframes() / float(wf.getframerate())


MODEL_MANIFEST_NAME = "kws_model_manifest.json"


def write_model_manifest(directory: Path, model_paths: list[Path], labels: list[str],
                         feature_mean: float, feature_std: float,
                         preprocess: dict | None = None) -> Path:
    """记录本次导出的模型和前处理配置，不包含数据集或本机绝对路径。"""
    import hashlib

    config = PreprocessConfig().to_dict()
    if preprocess is not None:
        config.update(preprocess)
    config.update(target_peak=0.75, pre_emphasis=0.97,
                  window="hann_symmetric", log_floor=1e-8)
    frame_count = 1 + (config["sample_count"] - config["frame_length"]) // config["frame_step"]
    models = {}
    for path in model_paths:
        if path.parent.resolve() != directory.resolve():
            raise ValueError("清单只记录同目录模型")
        models[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    if not models:
        raise ValueError("没有可记录的模型")
    manifest = {
        "version": 1, "models": models, "labels": list(labels),
        "input_shape": [1, frame_count, config["mel_bins"], 1],
        "feature_mean": float(feature_mean), "feature_std": float(feature_std),
        "preprocess": config,
    }
    path = directory / MODEL_MANIFEST_NAME
    # 写入前复用读取端的数值与标签校验，避免生成不可用清单。
    _validate_manifest_config(manifest)
    write_json(path, manifest)
    return path


def _validate_manifest_config(manifest: dict) -> None:
    if not isinstance(manifest, dict) or manifest.get("version") != 1:
        raise ValueError("模型清单格式或版本无效")
    for name in ("feature_mean", "feature_std"):
        value = manifest.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"模型清单中的 {name} 必须为有限数值")
    if manifest["feature_std"] <= 0:
        raise ValueError("模型清单中的 feature_std 必须大于 0")
    labels = manifest.get("labels")
    if (not isinstance(labels, list) or not labels or
            any(not isinstance(label, str) or not label.strip() for label in labels) or
            len(set(labels)) != len(labels)):
        raise ValueError("模型清单标签必须为非空且不重复的字符串列表")
    expected = PreprocessConfig().to_dict()
    expected.update(target_peak=0.75, pre_emphasis=0.97,
                    window="hann_symmetric", log_floor=1e-8)
    if manifest.get("preprocess") != expected or manifest.get("input_shape") != [1, 98, 40, 1]:
        raise ValueError("当前推理工具仅支持清单中完整的 16kHz、1秒、98×40 log-mel 配置")


def load_model_manifest(model_path: Path, manifest_path: Path | None = None) -> dict:
    """校验模型哈希并读取同版本标签与前处理参数；缺失时不使用固定常量。"""
    import hashlib

    manifest_path = manifest_path or model_path.parent / MODEL_MANIFEST_NAME
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    _validate_manifest_config(manifest)
    models = manifest.get("models")
    if not isinstance(models, dict) or model_path.name not in models:
        raise ValueError(f"模型未登记在清单中: {model_path.name}")
    digest = hashlib.sha256(model_path.read_bytes()).hexdigest()
    if models[model_path.name] != digest:
        raise ValueError(f"模型 SHA-256 与清单不一致: {model_path.name}")
    return manifest


def validate_model_tensors(input_details: list, output_details: list, manifest: dict) -> None:
    """核对推理张量形状和标签数，当前调试路径仅接受 float32 模型。"""
    if len(input_details) != 1 or len(output_details) != 1:
        raise ValueError("当前工具仅支持单输入、单输出模型")
    if list(input_details[0]["shape"]) != manifest["input_shape"]:
        raise ValueError("模型输入形状与清单不一致")
    if list(output_details[0]["shape"]) != [1, len(manifest["labels"])]:
        raise ValueError("模型输出形状与清单标签数不一致")
    if input_details[0]["dtype"] != np.float32 or output_details[0]["dtype"] != np.float32:
        raise ValueError("当前调试工具仅支持 float32 输入和输出")
