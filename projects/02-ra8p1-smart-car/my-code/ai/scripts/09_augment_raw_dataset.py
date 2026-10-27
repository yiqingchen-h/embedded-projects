"""从真实 raw WAV 生成轻量增强样本，输出到 enhance_raw。"""

from __future__ import annotations

import argparse
import csv
import random
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from kws_common import (
    AI_ROOT,
    SAMPLE_RATE,
    add_noise,
    load_labels,
    normalize_peak,
    read_wav_mono,
    resample_audio,
    write_wav_mono,
)

DEFAULT_INPUT_DIR = AI_ROOT / "dataset_real" / "raw"
DEFAULT_OUTPUT_DIR = AI_ROOT / "dataset_real" / "enhance_raw"
DEFAULT_VARIANTS = 4
DEFAULT_SEED = 20260522
DEFAULT_BACKGROUND_SNR_MIN_DB = 8.0
DEFAULT_BACKGROUND_SNR_MAX_DB = 24.0
NOISE_LABELS = {"unknown", "silence"}


@dataclass(frozen=True)
class AugmentParams:
    """记录单个增强样本的参数，便于写入 manifest。"""

    variant: int
    speed: float
    gain_db: float
    leading_ms: int
    trailing_ms: int
    snr_db: float
    echo_delay_ms: int
    echo_gain: float
    peak: float


def db_to_gain(db: float) -> float:
    """将 dB 转换为线性增益。"""

    return 10.0 ** (db / 20.0)


def change_speed(audio: np.ndarray, speed: float) -> np.ndarray:
    """使用线性插值做轻量速度扰动。"""

    if audio.size < 2 or abs(speed - 1.0) < 0.005:
        return audio.astype(np.float32, copy=False)

    new_len = max(2, int(round(audio.size / speed)))
    src_x = np.linspace(0.0, 1.0, num=audio.size, dtype=np.float32)
    dst_x = np.linspace(0.0, 1.0, num=new_len, dtype=np.float32)
    return np.interp(dst_x, src_x, audio).astype(np.float32)


def pad_with_silence(audio: np.ndarray, leading_ms: int, trailing_ms: int) -> np.ndarray:
    """在音频前后插入短静音，模拟触发窗口偏移。"""

    leading = max(0, int(round(SAMPLE_RATE * leading_ms / 1000.0)))
    trailing = max(0, int(round(SAMPLE_RATE * trailing_ms / 1000.0)))
    if 0 == leading and 0 == trailing:
        return audio.astype(np.float32, copy=False)
    return np.pad(audio, (leading, trailing), mode="constant").astype(np.float32)


def add_white_noise_by_snr(audio: np.ndarray, rng: np.random.Generator, snr_db: float) -> np.ndarray:
    """按目标 SNR 添加白噪声。"""

    if snr_db <= 0.0:
        return audio.astype(np.float32, copy=False)

    signal_rms = float(np.sqrt(np.mean(np.square(audio)))) if audio.size else 0.0
    if signal_rms < 1.0e-6:
        noise_level = db_to_gain(-42.0)
    else:
        noise_level = signal_rms / db_to_gain(snr_db)

    return add_noise(audio, rng, noise_level)


def collect_background_noise_paths(background_noise_dir: Path | None) -> list[Path]:
    """收集真实背景噪声 WAV 文件。"""

    if background_noise_dir is None:
        return []
    if not background_noise_dir.exists():
        raise RuntimeError(f"背景噪声目录不存在: {background_noise_dir}")

    paths = sorted(path for path in background_noise_dir.rglob("*.wav") if path.is_file())
    if not paths:
        raise RuntimeError(f"背景噪声目录没有 WAV: {background_noise_dir}")
    return paths


def load_background_noise(path: Path, cache: dict[Path, np.ndarray]) -> np.ndarray:
    """读取并缓存真实背景噪声。"""

    if path in cache:
        return cache[path]

    sample_rate, audio = read_wav_mono(path)
    audio = resample_audio(audio, sample_rate, SAMPLE_RATE)
    cache[path] = audio.astype(np.float32, copy=False)
    return cache[path]


def pick_noise_segment(noise: np.ndarray, target_size: int, rng: np.random.Generator) -> np.ndarray:
    """从背景噪声中取出与命令词等长的片段。"""

    if target_size <= 0:
        return np.zeros(0, dtype=np.float32)
    if noise.size == 0:
        return np.zeros(target_size, dtype=np.float32)
    if noise.size >= target_size:
        max_start = noise.size - target_size
        start = int(rng.integers(0, max_start + 1)) if max_start > 0 else 0
        return noise[start:start + target_size].astype(np.float32, copy=True)

    repeat_count = int(np.ceil(target_size / noise.size))
    tiled = np.tile(noise, repeat_count)
    return tiled[:target_size].astype(np.float32, copy=True)


def mix_background_noise_by_snr(
    audio: np.ndarray,
    background_noise: np.ndarray,
    rng: np.random.Generator,
    snr_db: float,
) -> np.ndarray:
    """按目标 SNR 混合真实背景噪声。"""

    if snr_db <= 0.0:
        return audio.astype(np.float32, copy=False)

    signal_rms = float(np.sqrt(np.mean(np.square(audio)))) if audio.size else 0.0
    noise = pick_noise_segment(background_noise, int(audio.size), rng)
    noise_rms = float(np.sqrt(np.mean(np.square(noise)))) if noise.size else 0.0
    if signal_rms < 1.0e-6 or noise_rms < 1.0e-6:
        return audio.astype(np.float32, copy=False)

    target_noise_rms = signal_rms / db_to_gain(snr_db)
    noise = noise * (target_noise_rms / noise_rms)
    return np.clip(audio + noise, -1.0, 1.0).astype(np.float32)


def add_light_echo(audio: np.ndarray, delay_ms: int, echo_gain: float) -> np.ndarray:
    """添加轻量单次回声，模拟小空间反射。"""

    delay = max(0, int(round(SAMPLE_RATE * delay_ms / 1000.0)))
    if delay <= 0 or delay >= audio.size or echo_gain <= 0.0:
        return audio.astype(np.float32, copy=False)

    output = audio.astype(np.float32).copy()
    output[delay:] += audio[:-delay] * echo_gain
    return np.clip(output, -1.0, 1.0).astype(np.float32)


def make_command_params(variant: int, rng: random.Random) -> AugmentParams:
    """生成命令词类别的中等强度增强参数。"""

    return AugmentParams(
        variant=variant,
        speed=rng.uniform(0.94, 1.07),
        gain_db=rng.uniform(-4.0, 3.0),
        leading_ms=rng.randint(0, 120),
        trailing_ms=rng.randint(0, 100),
        snr_db=rng.uniform(18.0, 30.0),
        echo_delay_ms=rng.choice([0, 35, 55, 75, 90]),
        echo_gain=rng.uniform(0.08, 0.22),
        peak=rng.uniform(0.55, 0.85),
    )


def make_noise_params(variant: int, rng: random.Random) -> AugmentParams:
    """生成 unknown/silence 类别的环境噪声增强参数。"""

    return AugmentParams(
        variant=variant,
        speed=1.0,
        gain_db=rng.uniform(-6.0, 2.0),
        leading_ms=0,
        trailing_ms=0,
        snr_db=rng.uniform(0.0, 12.0),
        echo_delay_ms=0,
        echo_gain=0.0,
        peak=rng.uniform(0.08, 0.45),
    )


def augment_command(
    audio: np.ndarray,
    params: AugmentParams,
    rng: np.random.Generator,
    background_noise: np.ndarray | None = None,
    background_snr_db: float | None = None,
) -> np.ndarray:
    """增强命令词音频。"""

    output = change_speed(audio, params.speed)
    output = output * db_to_gain(params.gain_db)
    output = pad_with_silence(output, params.leading_ms, params.trailing_ms)
    if background_noise is None or background_snr_db is None:
        output = add_white_noise_by_snr(output, rng, params.snr_db)
    else:
        output = mix_background_noise_by_snr(output, background_noise, rng, background_snr_db)
    output = add_light_echo(output, params.echo_delay_ms, params.echo_gain)
    return normalize_peak(output, params.peak)


def augment_noise_label(audio: np.ndarray, params: AugmentParams, rng: np.random.Generator) -> np.ndarray:
    """增强 unknown/silence 音频。"""

    output = audio.astype(np.float32, copy=True) * db_to_gain(params.gain_db)

    if output.size == 0 or float(np.max(np.abs(output))) < 1.0e-5:
        output = rng.normal(0.0, db_to_gain(-42.0), size=max(SAMPLE_RATE, output.size)).astype(np.float32)

    output = add_white_noise_by_snr(output, rng, params.snr_db)
    return normalize_peak(output, params.peak)


def params_suffix(params: AugmentParams) -> str:
    """生成可追踪的增强文件后缀。"""

    return (
        f"aug{params.variant:02d}"
        f"_spd{int(round(params.speed * 100)):03d}"
        f"_g{int(round(params.gain_db * 10)):04d}"
        f"_snr{int(round(params.snr_db)):02d}"
        f"_e{params.echo_delay_ms:03d}"
    )


def iter_label_wavs(input_dir: Path, labels: list[str]):
    """按 labels.txt 顺序遍历 raw WAV。"""

    for label in labels:
        label_dir = input_dir / label
        if not label_dir.exists():
            continue
        for path in sorted(label_dir.glob("*.wav")):
            if path.is_file():
                yield label, path


def write_manifest(manifest_path: Path, rows: list[dict[str, str]]) -> None:
    """写出增强 manifest。"""

    if not rows:
        return

    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with manifest_path.open("w", encoding="utf-8", newline="") as manifest_file:
        writer = csv.DictWriter(manifest_file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="增强真实 raw WAV，输出到 dataset_real/enhance_raw")
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR, help="raw 输入根目录")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="增强 WAV 输出根目录")
    parser.add_argument("--variants", type=int, default=DEFAULT_VARIANTS, help="每条 raw 生成的增强版本数")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, help="随机种子")
    parser.add_argument("--background-noise-dir", type=Path, default=None, help="真实背景噪声 WAV 目录，可递归读取")
    parser.add_argument("--background-snr-min", type=float, default=DEFAULT_BACKGROUND_SNR_MIN_DB, help="真实背景噪声最小 SNR")
    parser.add_argument("--background-snr-max", type=float, default=DEFAULT_BACKGROUND_SNR_MAX_DB, help="真实背景噪声最大 SNR")
    parser.add_argument("--overwrite", action="store_true", help="覆盖已存在的同名增强文件")
    parser.add_argument("--dry-run", action="store_true", help="只统计将要生成的文件，不写 WAV")
    args = parser.parse_args()

    if args.variants <= 0:
        print("--variants 必须大于 0")
        return 1
    if not args.input_dir.exists():
        print(f"输入目录不存在: {args.input_dir}")
        return 1
    if args.background_snr_min > args.background_snr_max:
        print("--background-snr-min 不能大于 --background-snr-max")
        return 1

    try:
        background_paths = collect_background_noise_paths(args.background_noise_dir)
    except RuntimeError as exc:
        print(str(exc))
        return 1

    labels = load_labels()
    rng = random.Random(args.seed)
    np_rng = np.random.default_rng(args.seed)
    background_cache: dict[Path, np.ndarray] = {}
    counts = {label: 0 for label in labels}
    skipped = 0
    rows: list[dict[str, str]] = []

    for label, input_path in iter_label_wavs(args.input_dir, labels):
        sample_rate, audio = read_wav_mono(input_path)
        audio = resample_audio(audio, sample_rate, SAMPLE_RATE)

        for variant in range(args.variants):
            params = make_noise_params(variant, rng) if label in NOISE_LABELS else make_command_params(variant, rng)
            output_rel = Path(label) / f"{input_path.stem}_{params_suffix(params)}.wav"
            output_path = args.output_dir / output_rel
            background_rel = ""
            background_snr_db = ""
            background_noise = None

            if background_paths and label not in NOISE_LABELS:
                background_path = rng.choice(background_paths)
                background_noise = load_background_noise(background_path, background_cache)
                background_rel = str(background_path.relative_to(args.background_noise_dir))
                background_snr_db = f"{rng.uniform(args.background_snr_min, args.background_snr_max):.3f}"

            if output_path.exists() and not args.overwrite:
                skipped += 1
                continue

            if not args.dry_run:
                if label in NOISE_LABELS:
                    augmented = augment_noise_label(audio, params, np_rng)
                else:
                    actual_background_snr_db = float(background_snr_db) if background_snr_db else None
                    augmented = augment_command(audio, params, np_rng, background_noise, actual_background_snr_db)
                write_wav_mono(output_path, augmented, SAMPLE_RATE)

            counts[label] += 1
            rows.append(
                {
                    "label": label,
                    "source": str(input_path.relative_to(args.input_dir)),
                    "output": str(output_rel),
                    "variant": str(params.variant),
                    "speed": f"{params.speed:.4f}",
                    "gain_db": f"{params.gain_db:.3f}",
                    "leading_ms": str(params.leading_ms),
                    "trailing_ms": str(params.trailing_ms),
                    "snr_db": f"{params.snr_db:.3f}",
                    "background_noise": background_rel,
                    "background_snr_db": background_snr_db,
                    "echo_delay_ms": str(params.echo_delay_ms),
                    "echo_gain": f"{params.echo_gain:.4f}",
                    "peak": f"{params.peak:.4f}",
                }
            )

    if not args.dry_run:
        write_manifest(args.output_dir / "augmentation_manifest.csv", rows)

    print(f"增强完成: input={args.input_dir} output={args.output_dir} variants={args.variants}")
    if background_paths:
        print(f"真实背景噪声: dir={args.background_noise_dir} files={len(background_paths)} snr={args.background_snr_min:.1f}~{args.background_snr_max:.1f}dB")
    if args.dry_run:
        print("dry-run: 未写入 WAV")
    for label in labels:
        print(f"  {label}: {counts[label]}")
    if skipped > 0:
        print(f"跳过已存在文件: {skipped}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
