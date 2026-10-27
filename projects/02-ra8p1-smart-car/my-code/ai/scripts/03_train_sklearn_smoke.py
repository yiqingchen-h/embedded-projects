"""训练 sklearn baseline，用于快速验证 KWS 流程。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import joblib
import numpy as np
from sklearn.dummy import DummyClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
from sklearn.model_selection import train_test_split
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

from kws_common import ARTIFACTS_DIR, FEATURES_DIR, RANDOM_SEED, ensure_output_dirs


def main() -> int:
    parser = argparse.ArgumentParser(description="训练 sklearn KWS smoke baseline")
    parser.add_argument("--features", type=Path, default=FEATURES_DIR / "features.npz", help="features.npz 路径")
    parser.add_argument("--artifacts-dir", type=Path, default=ARTIFACTS_DIR, help="输出产物目录")
    args = parser.parse_args()

    ensure_output_dirs()
    feature_path = args.features
    if not feature_path.exists():
        print("没有找到 features.npz，请先运行 02_extract_features.py")
        return 1

    data = np.load(feature_path, allow_pickle=True)
    x = data["x"].astype(np.float32)
    y = data["y"].astype(np.int64)
    labels = [str(item) for item in data["labels"]]

    x_flat = x.reshape((x.shape[0], -1))
    class_count = len(labels)
    used_classes = np.unique(y)
    class_samples = np.bincount(y, minlength=class_count)
    split_test_count = int(np.ceil(x.shape[0] * 0.25))
    split_train_count = x.shape[0] - split_test_count
    tiny_dataset = (
        np.any(class_samples[used_classes] < 2)
        or split_test_count < used_classes.size
        or split_train_count < used_classes.size
    )
    if tiny_dataset:
        # 小样本联调阶段无法分层切分，先用全量回测验证流程和产物生成。
        x_train = x_flat
        x_test = x_flat
        y_train = y
        y_test = y
        split_note = "tiny_dataset_full_train_eval"
    else:
        x_train, x_test, y_train, y_test = train_test_split(
            x_flat,
            y,
            test_size=0.25,
            random_state=RANDOM_SEED,
            stratify=y,
        )
        split_note = "stratified_train_test_split"

    if used_classes.size < 2:
        model = DummyClassifier(strategy="most_frequent")
    else:
        model = make_pipeline(
            StandardScaler(),
            LogisticRegression(max_iter=2000, class_weight="balanced", random_state=RANDOM_SEED),
        )
    model.fit(x_train, y_train)
    y_pred = model.predict(x_test)
    accuracy = accuracy_score(y_test, y_pred)
    report = classification_report(
        y_test,
        y_pred,
        labels=np.arange(class_count),
        target_names=labels,
        digits=4,
        zero_division=0,
    )
    matrix = confusion_matrix(y_test, y_pred, labels=np.arange(class_count))

    args.artifacts_dir.mkdir(parents=True, exist_ok=True)
    joblib.dump(
        {
            "model": model,
            "labels": labels,
            "feature_shape": x.shape[1:],
            "split_note": split_note,
            "note": "TTS smoke baseline，仅用于验证流程。",
        },
        args.artifacts_dir / "sklearn_smoke_model.joblib",
    )

    report_text = (
        "# sklearn KWS Smoke Test Report\n\n"
        f"- samples: {x.shape[0]}\n"
        f"- feature_shape: {tuple(x.shape[1:])}\n"
        f"- split_note: {split_note}\n"
        f"- test_accuracy: {accuracy:.4f}\n\n"
        "## Classification Report\n\n"
        "```text\n"
        f"{report}"
        "```\n\n"
        "## Confusion Matrix\n\n"
        "```text\n"
        f"{matrix}\n"
        "```\n\n"
        "说明：该结果来自 TTS 小数据集，只表示电脑端流程跑通，不代表真实板端准确率。\n"
    )
    (args.artifacts_dir / "sklearn_smoke_report.md").write_text(report_text, encoding="utf-8")

    print(f"训练完成，test_accuracy={accuracy:.4f}")
    print(f"模型: {args.artifacts_dir / 'sklearn_smoke_model.joblib'}")
    print(f"报告: {args.artifacts_dir / 'sklearn_smoke_report.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
