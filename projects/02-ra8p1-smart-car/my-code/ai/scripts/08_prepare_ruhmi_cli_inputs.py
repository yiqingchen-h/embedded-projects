"""准备 RUHMI CLI 转换输入和量化校准数据。"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import struct
import sys
from pathlib import Path

import numpy as np

from kws_common import AI_ROOT, LABELS_PATH


def _parse_report_value(report_text: str, key: str) -> str:
    pattern = re.compile(rf"^- {re.escape(key)}:\s*(.+?)\s*$", re.MULTILINE)
    match = pattern.search(report_text)
    if match is None:
        raise RuntimeError(f"训练报告缺少字段: {key}")
    return match.group(1)


def _write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _copy_if_exists(src: Path, dst: Path) -> None:
    if src.exists():
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        print(f"复制: {src} -> {dst}")
    else:
        print(f"跳过，文件不存在: {src}")


def _to_ruhmi_symbol_name(input_name: str) -> str:
    """把 TFLite Tensor 名转换为 RUHMI 生成 C 符号使用的安全名。"""

    symbol_name = re.sub(r"[^0-9A-Za-z_]", "_", input_name)
    if not symbol_name:
        raise RuntimeError(f"RUHMI 输入符号名为空: {input_name}")
    return symbol_name


def _read_tflite_input_name(model_path: Path) -> str:
    """读取 TFLite 第一个输入 Tensor 名称，避免 RUHMI 配置残留旧符号。"""

    try:
        import tflite  # type: ignore
    except ImportError:
        tflite = None

    if tflite is None:
        return _read_tflite_input_name_without_schema(model_path)

    model_content = model_path.read_bytes()
    model = tflite.Model.GetRootAsModel(model_content, 0)
    subgraph = model.Subgraphs(0)
    if subgraph.InputsLength() <= 0:
        raise RuntimeError(f"TFLite 模型没有输入 Tensor: {model_path}")

    input_index = subgraph.Inputs(0)
    tensor = subgraph.Tensors(input_index)
    input_name = tensor.Name().decode("utf-8")
    if not input_name:
        raise RuntimeError(f"TFLite 输入 Tensor 名称为空: {model_path}")
    return input_name


def _read_tflite_input_name_without_schema(model_path: Path) -> str:
    """无 tflite schema 包时，最小化解析 FlatBuffer 输入 Tensor 名。"""

    data = model_path.read_bytes()

    def ensure_range(offset: int, size: int) -> None:
        if offset < 0 or offset + size > len(data):
            raise RuntimeError(f"TFLite FlatBuffer 越界: offset={offset}, size={size}")

    def read_u16(offset: int) -> int:
        ensure_range(offset, 2)
        return struct.unpack_from("<H", data, offset)[0]

    def read_i32(offset: int) -> int:
        ensure_range(offset, 4)
        return struct.unpack_from("<i", data, offset)[0]

    def read_u32(offset: int) -> int:
        ensure_range(offset, 4)
        return struct.unpack_from("<I", data, offset)[0]

    def field_location(table_offset: int, field_index: int) -> int | None:
        ensure_range(table_offset, 4)
        vtable_offset = table_offset - read_i32(table_offset)
        vtable_length = read_u16(vtable_offset)
        entry_offset = vtable_offset + 4 + field_index * 2
        if entry_offset + 2 > vtable_offset + vtable_length:
            return None
        field_offset = read_u16(entry_offset)
        if field_offset == 0:
            return None
        return table_offset + field_offset

    def vector_location(table_offset: int, field_index: int) -> tuple[int, int]:
        location = field_location(table_offset, field_index)
        if location is None:
            raise RuntimeError(f"TFLite FlatBuffer 缺少 vector 字段: {field_index}")
        vector_offset = location + read_u32(location)
        vector_length = read_u32(vector_offset)
        return vector_offset + 4, vector_length

    def table_from_vector(vector_data_offset: int, index: int) -> int:
        element_offset = vector_data_offset + index * 4
        return element_offset + read_u32(element_offset)

    def string_from_field(table_offset: int, field_index: int) -> str:
        location = field_location(table_offset, field_index)
        if location is None:
            raise RuntimeError(f"TFLite FlatBuffer 缺少 string 字段: {field_index}")
        string_offset = location + read_u32(location)
        string_length = read_u32(string_offset)
        ensure_range(string_offset + 4, string_length)
        return data[string_offset + 4:string_offset + 4 + string_length].decode("utf-8")

    root_offset = read_u32(0)
    subgraphs_data_offset, subgraphs_length = vector_location(root_offset, 2)
    if subgraphs_length <= 0:
        raise RuntimeError(f"TFLite 模型没有 SubGraph: {model_path}")
    subgraph_offset = table_from_vector(subgraphs_data_offset, 0)

    inputs_data_offset, inputs_length = vector_location(subgraph_offset, 1)
    if inputs_length <= 0:
        raise RuntimeError(f"TFLite 模型没有输入 Tensor: {model_path}")
    input_tensor_index = read_i32(inputs_data_offset)

    tensors_data_offset, tensors_length = vector_location(subgraph_offset, 0)
    if input_tensor_index < 0 or input_tensor_index >= tensors_length:
        raise RuntimeError(f"TFLite 输入 Tensor 索引越界: {input_tensor_index}")
    tensor_offset = table_from_vector(tensors_data_offset, input_tensor_index)

    input_name = string_from_field(tensor_offset, 3)
    if not input_name:
        raise RuntimeError(f"TFLite 输入 Tensor 名称为空: {model_path}")
    return input_name


def main() -> int:
    parser = argparse.ArgumentParser(description="准备 RUHMI CLI 转换输入")
    parser.add_argument(
        "--features",
        type=Path,
        default=AI_ROOT / "dataset_real" / "features" / "features.npz",
        help="features.npz 路径",
    )
    parser.add_argument(
        "--artifacts-dir",
        type=Path,
        default=AI_ROOT / "artifacts_real",
        help="训练产物目录",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=AI_ROOT / "ruhmi_generated" / "kws_current",
        help="RUHMI 当前转换工作目录",
    )
    parser.add_argument(
        "--preprocess-config",
        type=Path,
        default=AI_ROOT / "dataset_real" / "features" / "preprocess_config.json",
        help="前处理配置文件",
    )
    parser.add_argument(
        "--input-name",
        default=None,
        help="模型输入节点名；默认从当前 TFLite 模型自动读取",
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        default=128,
        help="最多生成多少条校准样本",
    )
    args = parser.parse_args()

    feature_path = args.features
    report_path = args.artifacts_dir / "tflite_training_report.md"
    if not feature_path.exists():
        print(f"没有找到特征文件: {feature_path}")
        return 1
    if not report_path.exists():
        print(f"没有找到训练报告: {report_path}")
        return 1

    report_text = report_path.read_text(encoding="utf-8")
    feature_mean = float(_parse_report_value(report_text, "feature_mean"))
    feature_std = float(_parse_report_value(report_text, "feature_std"))
    if feature_std <= 0.0:
        raise RuntimeError("feature_std 必须大于 0")

    output_root = args.output_root
    inputs_dir = output_root / "_inputs"
    calibration_dir = output_root / "_calibration"
    inputs_dir.mkdir(parents=True, exist_ok=True)
    calibration_dir.mkdir(parents=True, exist_ok=True)

    from kws_common import load_model_manifest, write_model_manifest
    # 先核对本次训练清单，再复制其中登记的 QAT 或普通 float 模型。
    source_manifest_path = args.artifacts_dir / "kws_model_manifest.json"
    source_manifest = json.loads(source_manifest_path.read_text(encoding="utf-8"))
    source_name = "model_qat_float.tflite" if "model_qat_float.tflite" in source_manifest.get("models", {}) else "model_float.tflite"
    source_model = args.artifacts_dir / source_name
    source_manifest = load_model_manifest(source_model, source_manifest_path)
    if (abs(feature_mean - source_manifest["feature_mean"]) > 1e-7 or
            abs(feature_std - source_manifest["feature_std"]) > 1e-7):
        raise ValueError("训练报告与模型清单的归一化参数不一致")
    preprocess = json.loads(args.preprocess_config.read_text(encoding="utf-8"))
    if any(source_manifest["preprocess"].get(key) != value for key, value in preprocess.items()):
        raise ValueError("特征配置与模型清单不一致")
    model_float_path = inputs_dir / "model_float.tflite"
    _copy_if_exists(source_model, model_float_path)
    copied_models = [model_float_path]
    if source_name == "model_qat_float.tflite":
        copied_qat = inputs_dir / source_name
        _copy_if_exists(source_model, copied_qat)
        copied_models.append(copied_qat)
    for name in ("model_int8.tflite",):
        if name in source_manifest["models"]:
            load_model_manifest(args.artifacts_dir / name, source_manifest_path)
            destination = inputs_dir / name
            _copy_if_exists(args.artifacts_dir / name, destination)
            copied_models.append(destination)
    _copy_if_exists(args.artifacts_dir / "model.onnx", inputs_dir / "model.onnx")
    (inputs_dir / "labels.txt").write_text("\n".join(source_manifest["labels"]) + "\n", encoding="utf-8")
    _copy_if_exists(args.preprocess_config, inputs_dir / "preprocess_config.json")
    _copy_if_exists(report_path, inputs_dir / "tflite_training_report.md")
    write_model_manifest(inputs_dir, copied_models, source_manifest["labels"],
                         source_manifest["feature_mean"], source_manifest["feature_std"], preprocess)

    tflite_input_name = args.input_name
    if tflite_input_name is None:
        tflite_input_name = _read_tflite_input_name(model_float_path)
    input_name = tflite_input_name
    ruhmi_symbol_name = _to_ruhmi_symbol_name(tflite_input_name)
    print(f"TFLite 输入名: {tflite_input_name}")
    if ruhmi_symbol_name != tflite_input_name:
        print(f"RUHMI 生成 C 符号输入名: {ruhmi_symbol_name}")

    data = np.load(feature_path, allow_pickle=True)
    x = data["x"].astype(np.float32)
    if x.ndim != 3 or list(x.shape[1:]) != source_manifest["input_shape"][1:-1]:
        raise ValueError("校准特征形状与模型清单不一致")
    sample_count = min(int(args.max_samples), int(x.shape[0]))
    if sample_count <= 0:
        raise RuntimeError("features.npz 中没有可用于校准的样本")

    for old_file in calibration_dir.glob("calib_*.npy"):
        old_file.unlink()
    for old_file in calibration_dir.glob("calib_*.bin"):
        old_file.unlink()

    normalized = ((x[:sample_count] - feature_mean) / feature_std)[..., np.newaxis].astype(np.float32)
    sample_files = []
    for index, sample in enumerate(normalized):
        sample = sample[np.newaxis, ...]
        npy_path = calibration_dir / f"calib_{index:04d}.npy"
        bin_path = calibration_dir / f"calib_{index:04d}.bin"
        np.save(npy_path, sample)
        sample.tofile(bin_path)
        sample_files.append(npy_path.name)

    np.savez_compressed(calibration_dir / "calibration_samples.npz", x=normalized)
    _write_json(
        calibration_dir / "calibration_manifest.json",
        {
            "sample_count": sample_count,
            "input_name": input_name,
            "ruhmi_symbol_input_name": ruhmi_symbol_name,
            "shape": [1, *normalized.shape[1:]],
            "dtype": "float32",
            "feature_mean": feature_mean,
            "feature_std": feature_std,
            "files": sample_files,
        },
    )
    _write_json(inputs_dir / "shape_dict_tflite.json", {input_name: [1, *normalized.shape[1:]]})
    _write_json(inputs_dir / "model_info_tflite.json", {"is_quantized": False})

    _write_json(
        inputs_dir / "quant_config_tflite.json",
        {
            "num_calibrate_samples": sample_count,
            "nodes": [
                {
                    "name": input_name,
                    "calibrate_sample_dir": str(calibration_dir.resolve()).replace("\\", "/"),
                    "mean": [0],
                    "std": [1],
                }
            ],
        },
    )

    print(f"校准样本: {sample_count}")
    print(f"输入目录: {inputs_dir}")
    print(f"校准目录: {calibration_dir}")
    print(f"量化配置: {inputs_dir / 'quant_config_tflite.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
