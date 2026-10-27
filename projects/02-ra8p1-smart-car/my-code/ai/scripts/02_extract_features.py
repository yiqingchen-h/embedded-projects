"""从 WAV 数据集中提取 log-mel 特征。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from kws_common import (
    FEATURES_DIR,
    RAW_DIR,
    PreprocessConfig,
    ensure_output_dirs,
    iter_wav_files,
    load_and_prepare_audio,
    load_labels,
    log_mel_features,
    make_mel_filterbank,
    write_json,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="从 WAV 数据集中提取 log-mel 特征")
    parser.add_argument("--raw-dir", type=Path, default=RAW_DIR, help="输入 WAV 根目录，内部按标签分子目录")
    parser.add_argument("--features-dir", type=Path, default=FEATURES_DIR, help="输出特征目录")
    args = parser.parse_args()

    ensure_output_dirs()
    labels = load_labels()
    label_to_id = {label: index for index, label in enumerate(labels)}
    mel_filters = make_mel_filterbank()

    features = []
    targets = []
    files = []

    for label, path in iter_wav_files(args.raw_dir):
        audio = load_and_prepare_audio(path)
        feature = log_mel_features(audio, mel_filters)
        features.append(feature)
        targets.append(label_to_id[label])
        files.append(str(path.relative_to(args.raw_dir)))

    if not features:
        print(f"没有找到 WAV: {args.raw_dir}")
        print("TTS 验证集请先运行: python my-code/ai/scripts/01_generate_tts_dataset.py")
        return 1

    x = np.stack(features).astype(np.float32)
    y = np.asarray(targets, dtype=np.int64)
    file_array = np.asarray(files)

    args.features_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.features_dir / "features.npz",
        x=x,
        y=y,
        files=file_array,
        labels=np.asarray(labels),
    )

    write_json(args.features_dir / "preprocess_config.json", PreprocessConfig().to_dict())

    print(f"特征提取完成: {x.shape}")
    for label in labels:
        print(f"  {label}: {int(np.sum(y == label_to_id[label]))}")
    print(f"输出: {args.features_dir / 'features.npz'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
