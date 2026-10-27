"""可选 ONNX 导出入口。"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

AI_ROOT = Path(__file__).resolve().parents[1]
ARTIFACTS_DIR = AI_ROOT / "artifacts"


def ensure_output_dirs() -> None:
    """创建 ONNX 导出默认产物目录。"""

    ARTIFACTS_DIR.mkdir(parents=True, exist_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="从 Keras 模型可选导出 ONNX")
    parser.add_argument("--artifacts-dir", type=Path, default=ARTIFACTS_DIR, help="包含 model_keras.keras 的产物目录")
    args = parser.parse_args()

    ensure_output_dirs()
    try:
        import tensorflow as tf
        import tf2onnx  # noqa: F401
    except ImportError:
        print("未安装 tensorflow/tf2onnx，跳过 ONNX 导出。")
        print("需要 ONNX 时安装 tensorflow、tf2onnx、onnx 后重新运行本脚本。")
        return 0

    keras_path = args.artifacts_dir / "model_keras.keras"
    if not keras_path.exists():
        print("没有找到 model_keras.keras，请先运行 04_train_tflite_kws.py")
        return 1

    model = tf.keras.models.load_model(keras_path)
    saved_model_dir = args.artifacts_dir / "saved_model_for_onnx"
    if saved_model_dir.exists():
        shutil.rmtree(saved_model_dir)
    model.export(str(saved_model_dir))

    onnx_path = args.artifacts_dir / "model.onnx"
    cmd = [
        sys.executable,
        "-m",
        "tf2onnx.convert",
        "--saved-model",
        str(saved_model_dir),
        "--output",
        str(onnx_path),
        "--opset",
        "13",
    ]
    result = subprocess.run(cmd, text=True, capture_output=True)
    if result.returncode != 0:
        print("ONNX 导出失败，tf2onnx 输出如下：")
        print(result.stdout)
        print(result.stderr)
        return result.returncode

    print(f"ONNX 导出完成: {onnx_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
