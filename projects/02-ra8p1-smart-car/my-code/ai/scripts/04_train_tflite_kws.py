"""TensorFlow 可用时训练小型 KWS 模型并导出 int8 TFLite（支持 QAT）。"""

from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

import numpy as np

AI_ROOT = Path(__file__).resolve().parents[1]
FEATURES_DIR = AI_ROOT / "dataset_tts" / "features"
ARTIFACTS_DIR = AI_ROOT / "artifacts"
RANDOM_SEED = 42
LABEL_SMOOTHING = 0.08
BASE_LEARNING_RATE = 0.0005
QAT_LEARNING_RATE = 0.0001
SPEC_AUGMENT_TIME_MASK_MAX = 12
SPEC_AUGMENT_TIME_MASK_COUNT = 2
SPEC_AUGMENT_FREQ_MASK_MAX = 4
SPEC_AUGMENT_FREQ_MASK_COUNT = 2


def ensure_output_dirs() -> None:
    """创建训练脚本默认输出目录。"""

    FEATURES_DIR.mkdir(parents=True, exist_ok=True)
    ARTIFACTS_DIR.mkdir(parents=True, exist_ok=True)


def stratified_split_indices(y: np.ndarray, test_ratio: float, random_seed: int) -> tuple[np.ndarray, np.ndarray]:
    """按类别切分训练/测试索引，避免额外依赖 sklearn。"""

    rng = np.random.default_rng(random_seed)
    train_indices: list[int] = []
    test_indices: list[int] = []

    for class_id in np.unique(y):
        indices = np.where(y == class_id)[0]
        rng.shuffle(indices)
        test_count = max(1, int(np.ceil(indices.size * test_ratio)))
        if test_count >= indices.size:
            test_count = indices.size - 1
        test_indices.extend(indices[:test_count].tolist())
        train_indices.extend(indices[test_count:].tolist())

    rng.shuffle(train_indices)
    rng.shuffle(test_indices)
    return np.asarray(train_indices, dtype=np.int64), np.asarray(test_indices, dtype=np.int64)


def normalize_group_stem(file_name: str) -> str:
    """从文件名提取分组键，避免增强副本同时进入训练集和测试集。"""

    stem = Path(file_name).stem
    stem = re.sub(r"_aug\d{2}_spd\d+_g[-0-9]+_snr\d+_e\d+$", "", stem)
    stem = re.sub(r"_c\d+$", "", stem)
    return stem


def grouped_stratified_split_indices(
    y: np.ndarray,
    files: list[str],
    test_ratio: float,
    random_seed: int,
) -> tuple[np.ndarray, np.ndarray, int]:
    """按类别分层、按源文件分组切分，防止增强样本数据泄露。"""

    rng = np.random.default_rng(random_seed)
    train_indices: list[int] = []
    test_indices: list[int] = []
    group_count = 0

    for class_id in np.unique(y):
        class_indices = np.where(y == class_id)[0]
        groups: dict[str, list[int]] = {}

        for index in class_indices:
            group_key = f"{int(class_id)}:{normalize_group_stem(files[int(index)])}"
            groups.setdefault(group_key, []).append(int(index))

        group_keys = list(groups.keys())
        rng.shuffle(group_keys)
        group_count += len(group_keys)

        test_group_count = max(1, int(np.ceil(len(group_keys) * test_ratio)))
        if test_group_count >= len(group_keys):
            test_group_count = len(group_keys) - 1

        for group_key in group_keys[:test_group_count]:
            test_indices.extend(groups[group_key])
        for group_key in group_keys[test_group_count:]:
            train_indices.extend(groups[group_key])

    rng.shuffle(train_indices)
    rng.shuffle(test_indices)
    return np.asarray(train_indices, dtype=np.int64), np.asarray(test_indices, dtype=np.int64), group_count


def make_classification_report(y_true: np.ndarray, y_pred: np.ndarray, labels: list[str]) -> str:
    """生成简洁分类报告，格式接近 sklearn 的 classification_report。"""

    lines = ["label               precision    recall  f1-score   support"]
    lines.append("")

    for class_id, label in enumerate(labels):
        true_positive = int(np.sum((y_true == class_id) & (y_pred == class_id)))
        predicted_count = int(np.sum(y_pred == class_id))
        support = int(np.sum(y_true == class_id))
        precision = true_positive / predicted_count if predicted_count > 0 else 0.0
        recall = true_positive / support if support > 0 else 0.0
        f1 = (2.0 * precision * recall / (precision + recall)) if (precision + recall) > 0.0 else 0.0
        lines.append(f"{label:<18} {precision:9.4f} {recall:9.4f} {f1:9.4f} {support:9d}")

    accuracy = float(np.mean(y_true == y_pred)) if y_true.size > 0 else 0.0
    lines.append("")
    lines.append(f"accuracy                               {accuracy:9.4f} {int(y_true.size):9d}")
    return "\n".join(lines) + "\n"


def copy_qat_weights_to_float_model(qat_model, float_model) -> int:
    """将 QAT wrapper 内部的业务层权重拷回普通 float 模型。"""

    copied_layers = 0

    for qat_layer in qat_model.layers:
        inner_layer = getattr(qat_layer, "layer", None)
        if inner_layer is None or not inner_layer.weights:
            continue

        try:
            target_layer = float_model.get_layer(inner_layer.name)
        except ValueError:
            continue

        target_weights = target_layer.get_weights()
        if not target_weights:
            continue

        wrapper_weights = qat_layer.get_weights()[: len(target_weights)]
        if len(target_weights) != len(wrapper_weights):
            continue
        if any(target.shape != wrapper.shape for target, wrapper in zip(target_weights, wrapper_weights)):
            continue

        target_layer.set_weights(wrapper_weights)
        copied_layers += 1

    return copied_layers


def copy_model_weights_by_name(source_model, target_model) -> int:
    """按同名层拷贝权重，用于从训练图同步到干净导出图。"""

    copied_layers = 0

    for target_layer in target_model.layers:
        target_weights = target_layer.get_weights()
        if not target_weights:
            continue

        try:
            source_layer = source_model.get_layer(target_layer.name)
        except ValueError:
            continue

        source_weights = source_layer.get_weights()
        if len(target_weights) != len(source_weights):
            continue
        if any(target.shape != source.shape for target, source in zip(target_weights, source_weights)):
            continue

        target_layer.set_weights(source_weights)
        copied_layers += 1

    return copied_layers


def make_spec_augment_layer(tf):
    """创建训练时频谱遮挡层，导出模型不包含该层。"""

    class SpecAugment(tf.keras.layers.Layer):
        """在 log-mel 特征上随机遮挡时间段和频带。"""

        def __init__(
            self,
            time_mask_max: int = SPEC_AUGMENT_TIME_MASK_MAX,
            time_mask_count: int = SPEC_AUGMENT_TIME_MASK_COUNT,
            freq_mask_max: int = SPEC_AUGMENT_FREQ_MASK_MAX,
            freq_mask_count: int = SPEC_AUGMENT_FREQ_MASK_COUNT,
            **kwargs,
        ):
            super().__init__(**kwargs)
            self.time_mask_max = int(time_mask_max)
            self.time_mask_count = int(time_mask_count)
            self.freq_mask_max = int(freq_mask_max)
            self.freq_mask_count = int(freq_mask_count)

        def _apply_time_mask(self, inputs):
            output = inputs
            time_size = tf.shape(output)[1]
            max_width = tf.minimum(self.time_mask_max, time_size)
            for _ in range(self.time_mask_count):
                width = tf.random.uniform([], minval=0, maxval=max_width + 1, dtype=tf.int32)
                start_limit = tf.maximum(time_size - width + 1, 1)
                start = tf.random.uniform([], minval=0, maxval=start_limit, dtype=tf.int32)
                positions = tf.range(time_size)
                keep = tf.logical_or(positions < start, positions >= (start + width))
                mask = tf.reshape(tf.cast(keep, output.dtype), (1, time_size, 1, 1))
                output = output * mask
            return output

        def _apply_freq_mask(self, inputs):
            output = inputs
            freq_size = tf.shape(output)[2]
            max_width = tf.minimum(self.freq_mask_max, freq_size)
            for _ in range(self.freq_mask_count):
                width = tf.random.uniform([], minval=0, maxval=max_width + 1, dtype=tf.int32)
                start_limit = tf.maximum(freq_size - width + 1, 1)
                start = tf.random.uniform([], minval=0, maxval=start_limit, dtype=tf.int32)
                positions = tf.range(freq_size)
                keep = tf.logical_or(positions < start, positions >= (start + width))
                mask = tf.reshape(tf.cast(keep, output.dtype), (1, 1, freq_size, 1))
                output = output * mask
            return output

        def call(self, inputs, training=None):
            if training is None:
                return inputs
            if isinstance(training, bool):
                if not training:
                    return inputs
                return self._apply_freq_mask(self._apply_time_mask(inputs))
            return tf.cond(
                training,
                lambda: self._apply_freq_mask(self._apply_time_mask(inputs)),
                lambda: inputs,
            )

        def get_config(self):
            config = super().get_config()
            config.update(
                {
                    "time_mask_max": self.time_mask_max,
                    "time_mask_count": self.time_mask_count,
                    "freq_mask_max": self.freq_mask_max,
                    "freq_mask_count": self.freq_mask_count,
                }
            )
            return config

    return SpecAugment


def build_dense_small_model(tf, input_shape: tuple[int, ...], class_count: int):
    """构建原始扁平 Dense 小模型，作为回退对照。"""

    flat_size = int(np.prod(input_shape))
    return tf.keras.Sequential(
        [
            tf.keras.layers.Input(shape=input_shape),
            tf.keras.layers.Reshape((flat_size,)),
            tf.keras.layers.Dense(128, activation="relu"),
            tf.keras.layers.Dropout(0.15),
            tf.keras.layers.Dense(64, activation="relu"),
            tf.keras.layers.Dense(class_count, activation="softmax"),
        ],
        name="kws_dense_small",
    )


def build_cnn_small_model(tf, input_shape: tuple[int, ...], class_count: int, use_spec_augment: bool = False):
    """构建小型 CNN KWS 模型，输入输出接口保持不变。"""

    layers = [tf.keras.layers.Input(shape=input_shape)]
    if use_spec_augment:
        spec_augment_layer = make_spec_augment_layer(tf)
        layers.append(spec_augment_layer(name="spec_augment"))

    layers.extend(
        [
            tf.keras.layers.Conv2D(16, (5, 3), padding="same", use_bias=False, name="conv_1"),
            tf.keras.layers.BatchNormalization(name="bn_1"),
            tf.keras.layers.ReLU(name="relu_1"),
            tf.keras.layers.MaxPooling2D(pool_size=(2, 2), name="pool_1"),
            tf.keras.layers.Dropout(0.10, name="dropout_1"),
            tf.keras.layers.Conv2D(32, (3, 3), padding="same", use_bias=False, name="conv_2"),
            tf.keras.layers.BatchNormalization(name="bn_2"),
            tf.keras.layers.ReLU(name="relu_2"),
            tf.keras.layers.MaxPooling2D(pool_size=(2, 2), name="pool_2"),
            tf.keras.layers.Dropout(0.15, name="dropout_2"),
            tf.keras.layers.Conv2D(48, (3, 3), padding="same", use_bias=False, name="conv_3"),
            tf.keras.layers.BatchNormalization(name="bn_3"),
            tf.keras.layers.ReLU(name="relu_3"),
            tf.keras.layers.GlobalAveragePooling2D(name="global_avg_pool"),
            tf.keras.layers.Dense(64, activation="relu", name="dense_1"),
            tf.keras.layers.Dropout(0.20, name="dropout_3"),
            tf.keras.layers.Dense(class_count, activation="softmax", name="output"),
        ]
    )

    return tf.keras.Sequential(
        layers,
        name="kws_cnn_small",
    )


def build_cnn_medium_model(tf, input_shape: tuple[int, ...], class_count: int):
    """构建中型 CNN KWS 模型，4 层卷积 + 3 次池化。"""

    return tf.keras.Sequential(
        [
            tf.keras.layers.Input(shape=input_shape),
            tf.keras.layers.Conv2D(32, (5, 3), padding="same", use_bias=False, name="conv_1"),
            tf.keras.layers.BatchNormalization(name="bn_1"),
            tf.keras.layers.ReLU(name="relu_1"),
            tf.keras.layers.MaxPooling2D(pool_size=(2, 2), name="pool_1"),
            tf.keras.layers.Dropout(0.15, name="dropout_1"),
            tf.keras.layers.Conv2D(64, (3, 3), padding="same", use_bias=False, name="conv_2"),
            tf.keras.layers.BatchNormalization(name="bn_2"),
            tf.keras.layers.ReLU(name="relu_2"),
            tf.keras.layers.MaxPooling2D(pool_size=(2, 2), name="pool_2"),
            tf.keras.layers.Dropout(0.20, name="dropout_2"),
            tf.keras.layers.Conv2D(96, (3, 3), padding="same", use_bias=False, name="conv_3"),
            tf.keras.layers.BatchNormalization(name="bn_3"),
            tf.keras.layers.ReLU(name="relu_3"),
            tf.keras.layers.MaxPooling2D(pool_size=(2, 2), name="pool_3"),
            tf.keras.layers.Dropout(0.25, name="dropout_3"),
            tf.keras.layers.Conv2D(128, (3, 3), padding="same", use_bias=False, name="conv_4"),
            tf.keras.layers.BatchNormalization(name="bn_4"),
            tf.keras.layers.ReLU(name="relu_4"),
            tf.keras.layers.GlobalAveragePooling2D(name="global_avg_pool"),
            tf.keras.layers.Dense(64, activation="relu", name="dense_1"),
            tf.keras.layers.Dropout(0.25, name="dropout_4"),
            tf.keras.layers.Dense(class_count, activation="softmax", name="output"),
        ],
        name="kws_cnn_medium",
    )


def build_model(
    tf,
    model_family: str,
    input_shape: tuple[int, ...],
    class_count: int,
    use_spec_augment: bool = False,
):
    """按命令行选择构建 KWS 模型。"""

    if "dense-small" == model_family:
        return build_dense_small_model(tf, input_shape, class_count)
    if "cnn-small" == model_family:
        return build_cnn_small_model(tf, input_shape, class_count, use_spec_augment=use_spec_augment)
    if "cnn-medium" == model_family:
        return build_cnn_medium_model(tf, input_shape, class_count)
    raise RuntimeError(f"不支持的模型结构: {model_family}")


def read_tflite_io_quantization(model_content: bytes) -> tuple[float, int, float, int]:
    """直接从 TFLite FlatBuffer 读取输入输出量化参数。"""

    import tflite

    model = tflite.Model.GetRootAsModel(model_content, 0)
    subgraph = model.Subgraphs(0)

    def tensor_quantization(tensor_index: int) -> tuple[float, int]:
        tensor = subgraph.Tensors(tensor_index)
        quantization = tensor.Quantization()
        scale = 0.0
        zero_point = 0
        if quantization is not None and quantization.ScaleLength() > 0:
            scale = float(quantization.Scale(0))
        if quantization is not None and quantization.ZeroPointLength() > 0:
            zero_point = int(quantization.ZeroPoint(0))
        return scale, zero_point

    input_scale, input_zp = tensor_quantization(subgraph.Inputs(0))
    output_scale, output_zp = tensor_quantization(subgraph.Outputs(0))
    return input_scale, input_zp, output_scale, output_zp


def main() -> int:
    parser = argparse.ArgumentParser(description="训练 TensorFlow KWS 模型并导出 int8 TFLite")
    parser.add_argument("--features", type=Path, default=FEATURES_DIR / "features.npz", help="features.npz 路径")
    parser.add_argument("--artifacts-dir", type=Path, default=ARTIFACTS_DIR, help="输出产物目录")
    parser.add_argument("--epochs", type=int, default=80, help="最大训练轮数")
    parser.add_argument("--qat-epochs", type=int, default=10, help="QAT 微调轮数，0 表示跳过 QAT")
    parser.add_argument(
        "--model-family",
        choices=("cnn-small", "cnn-medium", "dense-small"),
        default="cnn-small",
        help="模型结构，默认 cnn-small；cnn-medium 为 4 层卷积中型模型；dense-small 为旧扁平 Dense 结构",
    )
    args = parser.parse_args()

    ensure_output_dirs()
    os.environ.setdefault("TF_USE_LEGACY_KERAS", "1")
    try:
        import tensorflow as tf
    except ImportError:
        print("未安装 TensorFlow，跳过 TFLite 导出。")
        print("需要导出 int8 TFLite 时，在电脑环境安装 tensorflow 后重新运行本脚本。")
        return 0

    feature_path = args.features
    if not feature_path.exists():
        print("没有找到 features.npz，请先运行 02_extract_features.py")
        return 1

    data = np.load(feature_path, allow_pickle=True)
    x = data["x"].astype(np.float32)
    y = data["y"].astype(np.int64)
    labels = [str(item) for item in data["labels"]]
    files = [str(item) for item in data["files"]] if "files" in data else [f"sample_{i}" for i in range(len(y))]
    class_count = len(labels)

    used_classes = np.unique(y)
    class_samples = np.bincount(y, minlength=class_count)
    class_group_counts = np.zeros(class_count, dtype=np.int64)
    for class_id in used_classes:
        stems = {normalize_group_stem(files[int(index)]) for index in np.where(y == class_id)[0]}
        class_group_counts[int(class_id)] = len(stems)
    split_test_count = int(np.ceil(x.shape[0] * 0.25))
    split_train_count = x.shape[0] - split_test_count
    tiny_dataset = (
        np.any(class_samples[used_classes] < 2)
        or np.any(class_group_counts[used_classes] < 2)
        or split_test_count < used_classes.size
        or split_train_count < used_classes.size
    )
    if tiny_dataset:
        # 小样本联调阶段无法分层切分，先用全量回测验证训练和导出链路。
        x_train = x
        x_test = x
        y_train = y
        y_test = y
        split_note = "tiny_dataset_full_train_eval"
        split_group_count = int(np.sum(class_group_counts))
    else:
        train_indices, test_indices, split_group_count = grouped_stratified_split_indices(y, files, 0.25, RANDOM_SEED)
        x_train = x[train_indices]
        x_test = x[test_indices]
        y_train = y[train_indices]
        y_test = y[test_indices]
        split_note = "grouped_stratified_train_test_split"

    # 用训练集全局均值方差标准化，后续板端前处理也要复用同一组参数。
    feature_mean = float(x_train.mean())
    feature_std = float(x_train.std() + 1e-6)
    x_train = ((x_train - feature_mean) / feature_std)[..., np.newaxis]
    x_test = ((x_test - feature_mean) / feature_std)[..., np.newaxis]
    y_train_one_hot = np.eye(class_count, dtype=np.float32)[y_train]
    y_test_one_hot = np.eye(class_count, dtype=np.float32)[y_test]

    tf.keras.utils.set_random_seed(RANDOM_SEED)
    input_shape = x_train.shape[1:]
    use_spec_augment = False
    model = build_model(tf, args.model_family, input_shape, class_count, use_spec_augment=use_spec_augment)
    loss_fn = tf.keras.losses.CategoricalCrossentropy(label_smoothing=LABEL_SMOOTHING)
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=BASE_LEARNING_RATE),
        loss=loss_fn,
        metrics=[tf.keras.metrics.CategoricalAccuracy(name="accuracy")],
    )
    callbacks = [
        tf.keras.callbacks.EarlyStopping(
            monitor="val_accuracy",
            patience=12,
            restore_best_weights=True,
            mode="max",
        )
    ]
    history = model.fit(
        x_train,
        y_train_one_hot,
        validation_data=(x_test, y_test_one_hot),
        epochs=args.epochs,
        batch_size=16,
        callbacks=callbacks,
        verbose=2,
    )
    loss, accuracy = model.evaluate(x_test, y_test_one_hot, verbose=0)
    y_pred = np.argmax(model.predict(x_test, verbose=0), axis=1)
    argmax_accuracy = float(np.mean(y_test == y_pred)) if y_test.size > 0 else 0.0
    report_text = make_classification_report(y_test, y_pred, labels)

    args.artifacts_dir.mkdir(parents=True, exist_ok=True)

    export_model = model
    if use_spec_augment:
        export_model = build_model(tf, args.model_family, input_shape, class_count, use_spec_augment=False)
        copied_layers = copy_model_weights_by_name(model, export_model)
        print(f"训练权重已拷到不含 SpecAugment 的导出模型，层数={copied_layers}")

    converter = tf.lite.TFLiteConverter.from_keras_model(export_model)
    float_model = converter.convert()
    (args.artifacts_dir / "model_float.tflite").write_bytes(float_model)

    def representative_dataset():
        for sample in x_train[: min(80, len(x_train))]:
            yield [sample[np.newaxis, ...].astype(np.float32)]

    qat_accuracy: float | None = None
    if args.qat_epochs > 0:
        import tensorflow_model_optimization as tfmot

        q_aware_model = tfmot.quantization.keras.quantize_model(export_model)
        q_aware_model.compile(
            optimizer=tf.keras.optimizers.Adam(learning_rate=QAT_LEARNING_RATE),
            loss=loss_fn,
            metrics=[tf.keras.metrics.CategoricalAccuracy(name="accuracy")],
        )
        qat_history = q_aware_model.fit(
            x_train,
            y_train_one_hot,
            validation_data=(x_test, y_test_one_hot),
            epochs=args.qat_epochs,
            batch_size=16,
            verbose=2,
        )
        _, qat_accuracy = q_aware_model.evaluate(x_test, y_test_one_hot, verbose=0)
        print(f"QAT 微调完成 val_accuracy={qat_accuracy:.4f}")

        converter_qat = tf.lite.TFLiteConverter.from_keras_model(q_aware_model)
        converter_qat.optimizations = [tf.lite.Optimize.DEFAULT]
        converter_qat.representative_dataset = representative_dataset
        converter_qat.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
        converter_qat.inference_input_type = tf.int8
        converter_qat.inference_output_type = tf.int8
        int8_model = converter_qat.convert()

        copied_layers = copy_qat_weights_to_float_model(q_aware_model, export_model)
        print(f"QAT 权重已拷回普通 float 模型，层数={copied_layers}")

        converter_qat_float = tf.lite.TFLiteConverter.from_keras_model(export_model)
        qat_float_model = converter_qat_float.convert()
        (args.artifacts_dir / "model_qat_float.tflite").write_bytes(qat_float_model)

        keras_path = args.artifacts_dir / "model_keras.keras"
        export_model.save(keras_path)
        final_train_accuracy = qat_history.history["accuracy"][-1]
        final_val_accuracy = qat_history.history["val_accuracy"][-1]
    else:
        keras_path = args.artifacts_dir / "model_keras.keras"
        export_model.save(keras_path)

        converter_ptq = tf.lite.TFLiteConverter.from_keras_model(export_model)
        converter_ptq.optimizations = [tf.lite.Optimize.DEFAULT]
        converter_ptq.representative_dataset = representative_dataset
        converter_ptq.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
        converter_ptq.inference_input_type = tf.int8
        converter_ptq.inference_output_type = tf.int8
        int8_model = converter_ptq.convert()
        final_train_accuracy = history.history["accuracy"][-1]
        final_val_accuracy = history.history["val_accuracy"][-1]

    (args.artifacts_dir / "model_int8.tflite").write_bytes(int8_model)

    int8_input_scale = 0.0
    int8_input_zero_point = 0
    int8_output_scale = 0.0
    int8_output_zero_point = 0
    try:
        (
            int8_input_scale,
            int8_input_zero_point,
            int8_output_scale,
            int8_output_zero_point,
        ) = read_tflite_io_quantization(int8_model)
        print(f"int8 model input  quant: scale={int8_input_scale:.8f}  zero_point={int8_input_zero_point}")
        print(f"int8 model output quant: scale={int8_output_scale:.8f}  zero_point={int8_output_zero_point}")
    except Exception as exc:
        print(f"int8 模型量化参数检查跳过: {exc}")

    qat_accuracy_line = (
        f"- qat_val_accuracy: {qat_accuracy:.4f}\n" if qat_accuracy is not None else "- qat_val_accuracy: skipped\n"
    )
    report = (
        "# TensorFlow KWS Report\n\n"
        f"- test_loss: {loss:.4f}\n"
        f"- test_accuracy: {accuracy:.4f}\n"
        f"- argmax_accuracy: {argmax_accuracy:.4f}\n"
        f"- model_family: {args.model_family}\n"
        f"- spec_augment: {'enabled' if use_spec_augment else 'disabled'}\n"
        f"- label_smoothing: {LABEL_SMOOTHING:.4f}\n"
        f"- base_learning_rate: {BASE_LEARNING_RATE:.6f}\n"
        f"- qat_learning_rate: {QAT_LEARNING_RATE:.6f}\n"
        f"- qat_epochs: {args.qat_epochs}\n"
        f"{qat_accuracy_line}"
    )
    report += (
        f"- split_note: {split_note}\n"
        f"- split_group_count: {split_group_count}\n"
        f"- labels: {labels}\n"
        f"- input_shape: {tuple(x_train.shape[1:])}\n"
        f"- feature_mean: {feature_mean:.8f}\n"
        f"- feature_std: {feature_std:.8f}\n"
        f"- int8_input_scale: {int8_input_scale:.8f}\n"
        f"- int8_input_zero_point: {int8_input_zero_point}\n"
        f"- int8_output_scale: {int8_output_scale:.8f}\n"
        f"- int8_output_zero_point: {int8_output_zero_point}\n"
        f"- final_train_accuracy: {final_train_accuracy:.4f}\n"
        f"- final_val_accuracy: {final_val_accuracy:.4f}\n"
        "\n## Classification Report\n\n"
        "```text\n"
        f"{report_text}"
        "```\n"
    )
    (args.artifacts_dir / "tflite_training_report.md").write_text(report, encoding="utf-8")

    from kws_common import write_model_manifest
    exported_models = [args.artifacts_dir / "model_float.tflite", args.artifacts_dir / "model_int8.tflite"]
    if args.qat_epochs > 0:
        exported_models.append(args.artifacts_dir / "model_qat_float.tflite")
    write_model_manifest(args.artifacts_dir, exported_models, labels, feature_mean, feature_std)

    print(f"TFLite 导出完成，test_accuracy={accuracy:.4f}")
    print(f"int8 模型: {args.artifacts_dir / 'model_int8.tflite'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
