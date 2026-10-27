"""
用 Python 跑 float 和 int8 TFLite 模型，
对比 MCU 相同的特征提取 + 当前工程生成代码里的量化参数。
"""

import argparse
import numpy as np
import random
import re
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
LABELS_PATH = PROJECT_DIR / "my-code" / "ai" / "labels.txt"

# 复用已验证的特征提取
sys.path.insert(0, str(Path(__file__).resolve().parent))
from verify_mcu_feature_extraction import extract_features_mcu, read_wav_int16

import tflite
import tensorflow as tf


def load_labels():
    labels = []
    for line in LABELS_PATH.read_text(encoding="utf-8").splitlines():
        label = line.strip()
        if label and not label.startswith("#"):
            labels.append(label)

    if not labels:
        raise RuntimeError(f"标签文件为空: {LABELS_PATH}")

    return labels


LABELS = load_labels()

DEFAULT_DATASET_DIR = PROJECT_DIR / "my-code" / "ai" / "dataset_real" / "combined_1s"

FLOAT_MODEL = PROJECT_DIR / "my-code" / "ai" / "artifacts_real" / "model_float.tflite"
INT8_MODEL = PROJECT_DIR / "my-code" / "ai" / "artifacts_real" / "model_int8.tflite"

RUHMI_SRC_DIR = PROJECT_DIR / "my-code" / "ai" / "ruhmi_generated" / "kws_current" / "output" / "build" / "MCU" / "compilation" / "src"
COMPUTE_SUB_0000 = RUHMI_SRC_DIR / "compute_sub_0000.c"
COMPUTE_SUB_0002 = RUHMI_SRC_DIR / "compute_sub_0002.c"

SMOKE_TEST_FILES = [
    ("backward_0020", "backward",
     PROJECT_DIR / "my-code" / "ai" / "dataset_real" / "trimmed" / "backward" / "backward_0020.wav"),
    ("turn_left_0006", "turn_left",
     PROJECT_DIR / "my-code" / "ai" / "dataset_real" / "trimmed" / "turn_left" / "turn_left_0006.wav"),
]


def read_c_without_line_comments(path):
    text = path.read_text(encoding="utf-8")
    return re.sub(r"//.*", "", text)


def parse_affine_quantize(path):
    text = read_c_without_line_comments(path)
    pattern = r"AffineQuantizeFloatToInt8\s*\(\s*[^,]+,\s*[^,]+,\s*(\d+)\s*,\s*(-?\d+)\s*,\s*([0-9.eE+-]+)\s*\)"
    match = re.search(pattern, text, flags=re.S)
    if not match:
        raise RuntimeError(f"未能从 {path} 解析 AffineQuantizeFloatToInt8 参数")
    size = int(match.group(1))
    zero_point = int(match.group(2))
    scale = float(match.group(3))
    return scale, zero_point, size


def parse_affine_dequantize(path):
    text = read_c_without_line_comments(path)
    pattern = r"AffineDequantizeInt8ToFloat\s*\(\s*[^,]+,\s*[^,]+,\s*(\d+)\s*,\s*(-?\d+)\s*,\s*([0-9.eE+-]+)\s*\)"
    match = re.search(pattern, text, flags=re.S)
    if not match:
        raise RuntimeError(f"未能从 {path} 解析 AffineDequantizeInt8ToFloat 参数")
    size = int(match.group(1))
    zero_point = int(match.group(2))
    scale = float(match.group(3))
    return scale, zero_point, size


def quant_match(left_scale, left_zp, right_scale, right_zp):
    return abs(left_scale - right_scale) <= 1e-8 and left_zp == right_zp


def tensor_type_to_numpy(tensor_type):
    mapping = {
        tflite.TensorType.FLOAT32: np.float32,
        tflite.TensorType.INT8: np.int8,
        tflite.TensorType.UINT8: np.uint8,
        tflite.TensorType.INT16: np.int16,
        tflite.TensorType.INT32: np.int32,
        tflite.TensorType.INT64: np.int64,
        tflite.TensorType.BOOL: np.bool_,
    }
    if tensor_type not in mapping:
        raise RuntimeError(f"暂不支持的 TFLite TensorType: {tensor_type}")
    return mapping[tensor_type]


def get_tflite_io_details(model_path):
    model = tflite.Model.GetRootAsModel(model_path.read_bytes(), 0)
    subgraph = model.Subgraphs(0)

    def tensor_to_detail(tensor_index):
        tensor = subgraph.Tensors(tensor_index)
        quantization = tensor.Quantization()
        scale = 0.0
        zero_point = 0
        if quantization is not None and quantization.ScaleLength() > 0:
            scale = float(quantization.Scale(0))
        if quantization is not None and quantization.ZeroPointLength() > 0:
            zero_point = int(quantization.ZeroPoint(0))
        return {
            "index": int(tensor_index),
            "name": tensor.Name().decode("utf-8"),
            "shape": tensor.ShapeAsNumpy(),
            "dtype": tensor_type_to_numpy(tensor.Type()),
            "quantization": (scale, zero_point),
        }

    return (
        tensor_to_detail(subgraph.Inputs(0)),
        tensor_to_detail(subgraph.Outputs(0)),
    )


def build_dataset_tests(dataset_dir, samples_per_class, seed):
    rng = random.Random(seed)
    tests = []
    missing_labels = []

    for label in LABELS:
        label_dir = dataset_dir / label
        wav_files = sorted(label_dir.glob("*.wav"))
        if not wav_files:
            missing_labels.append(label)
            continue

        sample_count = min(samples_per_class, len(wav_files))
        for wav_path in rng.sample(wav_files, sample_count):
            tests.append((wav_path.stem, label, wav_path))

    return tests, missing_labels


def top1_label(scores):
    idx = np.argsort(scores)[::-1]
    return LABELS[int(idx[0])]


def print_topk(scores, label, expected=None):
    idx = np.argsort(scores)[::-1]
    parts = []
    top_count = min(3, len(idx))
    for k in range(top_count):
        i = int(idx[k])
        parts.append(f"{LABELS[i]}={scores[i]*1000:.0f}")
    line = "  ".join(parts)
    top1 = LABELS[int(idx[0])]
    if expected:
        top_labels = [LABELS[int(idx[k])] for k in range(top_count)]
        match = "PASS" if top1 == expected else ("WARN" if expected in top_labels else "FAIL")
        print(f"  {label}: {line}  | top1={top1} expected={expected} -> {match}")
    else:
        print(f"  {label}: {line}  | top1={top1}")

    return top1


def init_accuracy_summary():
    keys = [
        "float",
        "int8_TF_deq",
        "int8_MCU_deq",
        "int8_TFinput_deq",
        "int8_TFinput_MCUdeq",
    ]
    total_correct = {key: 0 for key in keys}
    per_label = {}

    for label in LABELS:
        per_label[label] = {"total": 0}
        for key in keys:
            per_label[label][key] = 0

    return keys, total_correct, per_label


def record_prediction(total_correct, per_label, key, scores, expected):
    predicted = top1_label(scores)
    matched = (predicted == expected)

    if matched:
        total_correct[key] += 1
        per_label[expected][key] += 1

    return predicted, matched


def print_accuracy_summary(keys, total_correct, per_label, failures, total_count):
    print("\n" + "=" * 60)
    print("Accuracy Summary")
    print("=" * 60)

    for key in keys:
        accuracy = 0.0
        if total_count > 0:
            accuracy = total_correct[key] / total_count
        print(f"{key}: {total_correct[key]}/{total_count} = {accuracy:.4%}")

    print("\nPer-class Summary")
    header = "label total " + " ".join(keys)
    print(header)
    for label in LABELS:
        total = per_label[label]["total"]
        values = []
        for key in keys:
            if total > 0:
                values.append(f"{per_label[label][key]}/{total}")
            else:
                values.append("-")
        print(f"{label} {total} " + " ".join(values))

    if failures:
        print("\nMismatches (first 30)")
        for name, expected, key, predicted in failures[:30]:
            print(f"{name}: {key} predicted={predicted} expected={expected}")


def parse_args():
    parser = argparse.ArgumentParser(description="验证当前 TFLite 与 RUHMI 生成代码的量化参数和抽样推理结果")
    parser.add_argument("--dataset-dir", type=Path, default=DEFAULT_DATASET_DIR,
                        help="combined_1s 数据目录，默认使用当前工程 my-code/ai/dataset_real/combined_1s")
    parser.add_argument("--samples-per-class", type=int, default=1,
                        help="每个类别随机抽样的 WAV 数量")
    parser.add_argument("--seed", type=int, default=42,
                        help="随机抽样种子")
    parser.add_argument("--smoke-old", action="store_true",
                        help="仅运行旧 trimmed 两个 WAV 的 smoke test，不作为整体准确率判断")
    parser.add_argument("--require-input-quant-match", action="store_true",
                        help="要求 TFLite int8 输入量化参数与 RUHMI compute_sub_0000.c 完全一致")
    parser.add_argument("--summary-only", action="store_true",
                        help="只输出准确率汇总，不打印每个样本的详细推理日志")
    return parser.parse_args()


def main():
    args = parse_args()

    print("=" * 60)
    print("Python TFLite Float + Int8 Model Verification")
    print("=" * 60)
    print(f"labels: {len(LABELS)} from {LABELS_PATH}")

    # === 加载模型 ===
    mcu_scale, mcu_zp, mcu_input_size = parse_affine_quantize(COMPUTE_SUB_0000)
    mcu_deq_scale, mcu_deq_zp, mcu_output_size = parse_affine_dequantize(COMPUTE_SUB_0002)

    float_interp = tf.lite.Interpreter(model_path=str(FLOAT_MODEL))
    float_interp.allocate_tensors()
    fin, fout = get_tflite_io_details(FLOAT_MODEL)
    print(f"\n[float model] input:  {fin['name']} {fin['shape']} {fin['dtype']}")
    print(f"[float model] output: {fout['name']} {fout['shape']} {fout['dtype']}")

    int8_interp = tf.lite.Interpreter(model_path=str(INT8_MODEL))
    int8_interp.allocate_tensors()
    iin, iout = get_tflite_io_details(INT8_MODEL)
    iin_scale = float(iin['quantization'][0])
    iin_zp = int(iin['quantization'][1])
    iout_scale = float(iout['quantization'][0])
    iout_zp = int(iout['quantization'][1])

    print(f"\n[int8 model] input:  {iin['name']} {iin['shape']} {iin['dtype']}")
    print(f"  TFLite quant:              scale={iin_scale:.17g}  zero_point={iin_zp}")
    print(f"  RUHMI compute_sub_0000.c:  scale={mcu_scale:.17g}  zero_point={mcu_zp}  size={mcu_input_size}")
    input_match = quant_match(iin_scale, iin_zp, mcu_scale, mcu_zp)
    print(f"  INPUT QUANT: {'MATCH' if input_match else '*** MISMATCH ***'}")

    print(f"\n[int8 model] output: {iout['name']} {iout['shape']} {iout['dtype']}")
    print(f"  TFLite quant:              scale={iout_scale:.17g}  zero_point={iout_zp}")
    print(f"  RUHMI compute_sub_0002.c:  scale={mcu_deq_scale:.17g}  zero_point={mcu_deq_zp}  size={mcu_output_size}")
    output_match = quant_match(iout_scale, iout_zp, mcu_deq_scale, mcu_deq_zp)
    print(f"  OUTPUT QUANT: {'MATCH' if output_match else '*** MISMATCH ***'}")

    if args.require_input_quant_match and not input_match:
        print("\n输入量化参数不一致，停止验证。")
        return 1

    if not output_match:
        print("\n量化参数不一致，停止验证。")
        return 1

    if not input_match:
        print(
            "\n提示：TFLite int8 输入量化与 RUHMI 输入量化不一致。"
            "下面的 RUHMI 输入量化对照是把 RUHMI 量化后的 int8 字节喂给 TFLite 参考模型，"
            "不等同于 MCU 上 RUHMI command stream 的真实准确率。"
        )

    if args.smoke_old:
        print("\n使用旧 trimmed WAV 做 smoke test，不作为新模型整体准确率判断。")
        test_files = [(name, expected, wav_path) for name, expected, wav_path in SMOKE_TEST_FILES if wav_path.exists()]
    else:
        test_files, missing_labels = build_dataset_tests(args.dataset_dir, args.samples_per_class, args.seed)
        print(f"\n使用数据集抽样验证: {args.dataset_dir}")
        print(f"每类抽样: {args.samples_per_class}  实际样本数: {len(test_files)}")
        if missing_labels:
            print(f"缺少 WAV 的类别: {', '.join(missing_labels)}")

    if not test_files:
        print("\n没有可验证的 WAV 文件。")
        return 2

    keys, total_correct, per_label = init_accuracy_summary()
    failures = []

    for name, expected, wav_path in test_files:
        if expected not in per_label:
            print(f"\n跳过未知标签样本: {wav_path}")
            continue

        per_label[expected]["total"] += 1

        if not args.summary_only:
            print(f"\n{'=' * 60}")
            print(f"TEST: {name}  expected={expected}")

        pcm, sr = read_wav_int16(str(wav_path))
        if len(pcm) > 16000:
            pcm = pcm[:16000]
        elif len(pcm) < 16000:
            pcm = np.pad(pcm, (0, 16000 - len(pcm)))

        features = extract_features_mcu(pcm, sr)
        flat = features.flatten().astype(np.float32)

        # ── float TFLite ──
        float_interp.set_tensor(fin['index'], flat.reshape(1, 98, 40, 1))
        float_interp.invoke()
        f_out = float_interp.get_tensor(fout['index'])[0]
        predicted, matched = record_prediction(total_correct, per_label, "float", f_out, expected)
        if not matched:
            failures.append((name, expected, "float", predicted))
        if not args.summary_only:
            print_topk(f_out, "float", expected)

        # ── 对照实验：用 RUHMI 输入量化参数生成 int8，再喂给 TFLite 参考模型 ──
        q_mcu = np.round(flat / mcu_scale + mcu_zp)
        q_mcu = np.clip(q_mcu, -128, 127).astype(np.int8)
        if not args.summary_only:
            print(f"\n  [int8-input (RUHMI quant s={mcu_scale:.6f} zp={mcu_zp})]")
            print(f"  sum={int(q_mcu.astype(np.int32).sum())} min={int(q_mcu.min())} max={int(q_mcu.max())}")

        int8_interp.set_tensor(iin['index'], q_mcu.reshape(1, 98, 40, 1))
        int8_interp.invoke()
        io_raw = int8_interp.get_tensor(iout['index'])

        if not args.summary_only:
            print(f"  [int8 output raw] first{len(LABELS)}: {io_raw.flatten()[:len(LABELS)].tolist()}")
            print(f"  [int8 output raw] sum={int(io_raw.astype(np.int32).sum())}")

        # 用 TFLite 模型自身的 output quantization 反量化
        io_tflite_deq = (io_raw.astype(np.float32) - iout_zp) * iout_scale
        predicted, matched = record_prediction(total_correct, per_label, "int8_TF_deq", io_tflite_deq[0], expected)
        if not matched:
            failures.append((name, expected, "int8_TF_deq", predicted))
        if not args.summary_only:
            print(f"\n  [int8 -> float dequant (TFLite params s={iout_scale:.6f} zp={iout_zp})]")
            print_topk(io_tflite_deq[0], "int8_TF_deq", expected)

        # 用 MCU compute_sub_0002 的反量化参数
        io_mcu_deq = (io_raw.astype(np.float32) - mcu_deq_zp) * mcu_deq_scale
        predicted, matched = record_prediction(total_correct, per_label, "int8_MCU_deq", io_mcu_deq[0], expected)
        if not matched:
            failures.append((name, expected, "int8_MCU_deq", predicted))
        if not args.summary_only:
            print(f"\n  [int8 -> float dequant (RUHMI params s={mcu_deq_scale:.6f} zp={mcu_deq_zp})]")
            print_topk(io_mcu_deq[0], "int8_MCU_deq", expected)

        # ── TFLite 参考路径：用 TFLite 模型自身输入量化参数 ──
        q_tf = np.round(flat / iin_scale + iin_zp)
        q_tf = np.clip(q_tf, -128, 127).astype(np.int8)
        if not args.summary_only:
            print(f"\n  [int8-input (TFLite quant s={iin_scale:.6f} zp={iin_zp})]")
            print(f"  sum={int(q_tf.astype(np.int32).sum())} min={int(q_tf.min())} max={int(q_tf.max())}")

        int8_interp.set_tensor(iin['index'], q_tf.reshape(1, 98, 40, 1))
        int8_interp.invoke()
        io2_raw = int8_interp.get_tensor(iout['index'])

        if not args.summary_only:
            print(f"  [int8 output raw] first{len(LABELS)}: {io2_raw.flatten()[:len(LABELS)].tolist()}")
        io2_deq = (io2_raw.astype(np.float32) - iout_zp) * iout_scale
        predicted, matched = record_prediction(total_correct, per_label, "int8_TFinput_deq", io2_deq[0], expected)
        if not matched:
            failures.append((name, expected, "int8_TFinput_deq", predicted))
        if not args.summary_only:
            print(f"\n  [int8 -> float dequant (TFLite params)]")
            print_topk(io2_deq[0], "int8_TFinput_deq", expected)

        io2_mcu_deq = (io2_raw.astype(np.float32) - mcu_deq_zp) * mcu_deq_scale
        predicted, matched = record_prediction(total_correct, per_label, "int8_TFinput_MCUdeq", io2_mcu_deq[0], expected)
        if not matched:
            failures.append((name, expected, "int8_TFinput_MCUdeq", predicted))
        if not args.summary_only:
            print(f"\n  [int8 -> float dequant (RUHMI params)]")
            print_topk(io2_mcu_deq[0], "int8_TFinput_MCUdeq", expected)

        # ── MCU 日志对照 ──
        if not args.summary_only:
            print(f"\n  [MCU log for {name}]")
            print(f"  cpu_to_npu_in: sum={int(q_mcu.astype(np.int32).sum())} min={int(q_mcu.min())} max={int(q_mcu.max())}")
            # Backward MCU log: sum=19658 min=-54 max=69 q0=-42 q1=-25 q2=-10 ...
            # Turn_left MCU log: sum=19327 min=-64 max=69 q0=-31 q1=-16 q2=-16 ...

    checked_count = sum(per_label[label]["total"] for label in LABELS)
    print_accuracy_summary(keys, total_correct, per_label, failures, checked_count)

    return 0


if __name__ == "__main__":
    sys.exit(main())
