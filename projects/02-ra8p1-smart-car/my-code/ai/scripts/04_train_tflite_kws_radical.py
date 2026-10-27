"""KWS 训练：cnn-medium、余弦学习率、类别权重、SpecAugment 和 QAT。

模型供 RUHMI/vela 转换；部署占用以实际生成结果和链接报告为准。
默认要求 TensorFlow 能看到 GPU，避免误用 CPU 长时间训练。
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys
from pathlib import Path

import numpy as np

AI_ROOT = Path(__file__).resolve().parents[1]
FEATURES_DIR = AI_ROOT / "dataset_real" / "features"
ARTIFACTS_DIR = AI_ROOT / "artifacts_real"
BASE_SCRIPT_PATH = Path(__file__).with_name("04_train_tflite_kws.py")

RANDOM_SEED = 42
LABEL_SMOOTHING = 0.05
BASE_LEARNING_RATE = 0.001
WARMUP_EPOCHS = 5
QAT_LEARNING_RATE = 0.0001
EARLY_STOPPING_PATIENCE = 15
EARLY_STOPPING_MONITOR = "val_loss"
EARLY_STOPPING_MODE = "min"
SPEC_AUGMENT_TIME_MASK_MAX = 10
SPEC_AUGMENT_TIME_MASK_COUNT = 2
SPEC_AUGMENT_FREQ_MASK_MAX = 4
SPEC_AUGMENT_FREQ_MASK_COUNT = 1


def load_base_train_module():
    """加载普通训练脚本，复用切分、报告和 TFLite 量化检查工具。"""

    spec = importlib.util.spec_from_file_location("kws_train_base", BASE_SCRIPT_PATH)
    if (None == spec) or (None == spec.loader):
        raise RuntimeError(f"无法加载基础训练脚本: {BASE_SCRIPT_PATH}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def ensure_gpu_ready(tf, allow_cpu: bool) -> bool:
    """确认 TensorFlow GPU 可用，除非用户明确允许 CPU 训练。"""

    gpus = tf.config.list_physical_devices("GPU")
    if gpus:
        print("TensorFlow GPUs:")
        for gpu in gpus:
            print(f"  {gpu}")
            try:
                tf.config.experimental.set_memory_growth(gpu, True)
            except RuntimeError:
                pass
        return True

    if allow_cpu:
        print("警告：未发现 TensorFlow GPU，已按 --allow-cpu 继续 CPU 训练。")
        return True

    print("未发现 TensorFlow GPU，已停止。")
    print("如确认要用 CPU 长时间训练，请显式增加 --allow-cpu。")
    return False


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


def build_cnn_xlarge_model(tf, input_shape: tuple[int, ...], class_count: int, use_spec_augment: bool = False):
    """构建 cnn-medium KWS 模型（4 层卷积，已验证可被 RUHMI/vela NPU 编码）。"""

    layers = [tf.keras.layers.Input(shape=input_shape)]
    if use_spec_augment:
        spec_augment_layer = make_spec_augment_layer(tf)
        layers.append(spec_augment_layer(name="spec_augment"))

    layers.extend(
        [
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
        ]
    )

    return tf.keras.Sequential(layers, name="kws_cnn_medium")


def create_cosine_schedule(tf, warmup_steps: int, total_steps: int, peak_lr: float):
    """创建 warmup + cosine decay 学习率。"""

    class CosineWarmupSchedule(tf.keras.optimizers.schedules.LearningRateSchedule):
        """Keras 可序列化学习率调度。"""

        def __call__(self, step):
            warmup_steps_f = tf.cast(warmup_steps, tf.float32)
            total_steps_f = tf.cast(total_steps, tf.float32)
            step_f = tf.cast(step, tf.float32)
            warmup_lr = peak_lr * (step_f / tf.maximum(warmup_steps_f, 1.0))
            progress = (step_f - warmup_steps_f) / tf.maximum(total_steps_f - warmup_steps_f, 1.0)
            progress = tf.clip_by_value(progress, 0.0, 1.0)
            cosine_decay = 0.5 * (1.0 + tf.cos(np.pi * progress))
            cosine_lr = 1e-6 + (peak_lr - 1e-6) * cosine_decay
            return tf.where(step_f < warmup_steps_f, warmup_lr, cosine_lr)

        def get_config(self):
            return {
                "warmup_steps": warmup_steps,
                "total_steps": total_steps,
                "peak_lr": peak_lr,
            }

    return CosineWarmupSchedule()


def main() -> int:
    """训练 radical KWS 模型并导出 float/QAT/int8 TFLite。"""

    parser = argparse.ArgumentParser(description="GPU 优先的 radical KWS 训练脚本")
    parser.add_argument("--features", type=Path, default=FEATURES_DIR / "features.npz", help="features.npz 路径")
    parser.add_argument("--artifacts-dir", type=Path, default=ARTIFACTS_DIR, help="输出产物目录")
    parser.add_argument("--epochs", type=int, default=100, help="最大训练轮数")
    parser.add_argument("--qat-epochs", type=int, default=15, help="QAT 微调轮数，0 表示跳过 QAT")
    parser.add_argument("--batch-size", type=int, default=16, help="训练 batch size")
    parser.add_argument("--qat-batch-size", type=int, default=16, help="QAT 微调 batch size，显存紧张时可手动调小")
    parser.add_argument("--allow-cpu", action="store_true", help="未发现 GPU 时仍允许 CPU 训练")
    args = parser.parse_args()

    base = load_base_train_module()
    os.environ.setdefault("TF_USE_LEGACY_KERAS", "1")
    try:
        import tensorflow as tf
    except ImportError:
        print("未安装 TensorFlow，跳过 TFLite 导出。")
        return 0

    if not ensure_gpu_ready(tf, args.allow_cpu):
        return 2

    if not args.features.exists():
        print("没有找到 features.npz，请先运行 02_extract_features.py")
        return 1

    data = np.load(args.features, allow_pickle=True)
    x = data["x"].astype(np.float32)
    y = data["y"].astype(np.int64)
    labels = [str(item) for item in data["labels"]]
    files = [str(item) for item in data["files"]] if "files" in data else [f"sample_{i}" for i in range(len(y))]
    class_count = len(labels)

    used_classes = np.unique(y)
    class_samples = np.bincount(y, minlength=class_count)
    class_group_counts = np.zeros(class_count, dtype=np.int64)
    for class_id in used_classes:
        stems = {base.normalize_group_stem(files[int(index)]) for index in np.where(y == class_id)[0]}
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
        x_train = x
        x_test = x
        y_train = y
        y_test = y
        split_note = "tiny_dataset_full_train_eval"
        split_group_count = int(np.sum(class_group_counts))
    else:
        train_indices, test_indices, split_group_count = base.grouped_stratified_split_indices(
            y,
            files,
            0.25,
            RANDOM_SEED,
        )
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

    class_weights = np.ones(class_count, dtype=np.float32)
    silence_index = labels.index("silence") if "silence" in labels else -1
    unknown_index = labels.index("unknown") if "unknown" in labels else -1
    if silence_index >= 0:
        class_weights[silence_index] = 1.5
    if unknown_index >= 0:
        class_weights[unknown_index] = 1.3
    sample_weights = class_weights[y_train]

    tf.keras.utils.set_random_seed(RANDOM_SEED)
    input_shape = x_train.shape[1:]
    model = build_cnn_xlarge_model(tf, input_shape, class_count, use_spec_augment=True)
    steps_per_epoch = max(1, int(np.ceil(len(x_train) / max(1, args.batch_size))))
    lr_schedule = create_cosine_schedule(
        tf,
        steps_per_epoch * WARMUP_EPOCHS,
        steps_per_epoch * args.epochs,
        BASE_LEARNING_RATE,
    )
    loss_fn = tf.keras.losses.CategoricalCrossentropy(label_smoothing=LABEL_SMOOTHING)
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=lr_schedule),
        loss=loss_fn,
        metrics=[tf.keras.metrics.CategoricalAccuracy(name="accuracy")],
    )
    callbacks = [
        tf.keras.callbacks.EarlyStopping(
            monitor=EARLY_STOPPING_MONITOR,
            patience=EARLY_STOPPING_PATIENCE,
            restore_best_weights=True,
            mode=EARLY_STOPPING_MODE,
        )
    ]

    print(f"训练: epochs={args.epochs} batch_size={args.batch_size} lr_peak={BASE_LEARNING_RATE}")
    print(
        f"early_stopping: monitor={EARLY_STOPPING_MONITOR} "
        f"mode={EARLY_STOPPING_MODE} patience={EARLY_STOPPING_PATIENCE}"
    )
    print(f"class_weight: silence={class_weights[silence_index] if silence_index >= 0 else '?'} "
          f"unknown={class_weights[unknown_index] if unknown_index >= 0 else '?'}")
    history = model.fit(
        x_train,
        y_train_one_hot,
        validation_data=(x_test, y_test_one_hot),
        sample_weight=sample_weights,
        epochs=args.epochs,
        batch_size=args.batch_size,
        callbacks=callbacks,
        verbose=2,
    )
    loss, accuracy = model.evaluate(x_test, y_test_one_hot, verbose=0)
    y_pred = np.argmax(model.predict(x_test, verbose=0), axis=1)
    argmax_accuracy = float(np.mean(y_test == y_pred)) if y_test.size > 0 else 0.0
    report_text = base.make_classification_report(y_test, y_pred, labels)

    args.artifacts_dir.mkdir(parents=True, exist_ok=True)
    export_model = build_cnn_xlarge_model(tf, input_shape, class_count, use_spec_augment=False)
    copied_layers = base.copy_model_weights_by_name(model, export_model)
    print(f"训练权重已拷到不含 SpecAugment 的导出模型，层数={copied_layers}")
    export_model.save(args.artifacts_dir / "model_keras_base.keras")
    base_val_losses = history.history.get("val_loss", [])
    base_best_index = int(np.argmin(base_val_losses)) if base_val_losses else (len(history.epoch) - 1)
    base_best_epoch = base_best_index + 1
    base_best_val_loss = float(history.history["val_loss"][base_best_index])
    base_best_val_accuracy = float(history.history["val_accuracy"][base_best_index])
    base_best_train_accuracy = float(history.history["accuracy"][base_best_index])
    base_last_train_accuracy = float(history.history["accuracy"][-1])
    base_last_val_accuracy = float(history.history["val_accuracy"][-1])
    base_last_val_loss = float(history.history["val_loss"][-1])

    base_report = (
        "# TensorFlow KWS Report (Radical Base)\n\n"
        f"- test_loss: {loss:.4f}\n"
        f"- test_accuracy: {accuracy:.4f}\n"
        f"- argmax_accuracy: {argmax_accuracy:.4f}\n"
        "- model_family: cnn-medium\n"
        "- cosine_lr: enabled\n"
        f"- warmup_epochs: {WARMUP_EPOCHS}\n"
        f"- early_stopping_monitor: {EARLY_STOPPING_MONITOR}\n"
        f"- early_stopping_mode: {EARLY_STOPPING_MODE}\n"
        f"- early_stopping_patience: {EARLY_STOPPING_PATIENCE}\n"
        f"- class_weight_silence: {class_weights[silence_index] if silence_index >= 0 else 1.0:.4f}\n"
        f"- class_weight_unknown: {class_weights[unknown_index] if unknown_index >= 0 else 1.0:.4f}\n"
        "- spec_augment: enabled\n"
        f"- label_smoothing: {LABEL_SMOOTHING:.4f}\n"
        f"- base_learning_rate: {BASE_LEARNING_RATE:.6f}\n"
        "- qat_status: not_started\n"
        f"- split_note: {split_note}\n"
        f"- split_group_count: {split_group_count}\n"
        f"- labels: {labels}\n"
        f"- input_shape: {tuple(x_train.shape[1:])}\n"
        f"- feature_mean: {feature_mean:.8f}\n"
        f"- feature_std: {feature_std:.8f}\n"
        f"- best_epoch: {base_best_epoch}\n"
        f"- best_val_loss: {base_best_val_loss:.4f}\n"
        f"- best_val_accuracy: {base_best_val_accuracy:.4f}\n"
        f"- final_train_accuracy: {base_best_train_accuracy:.4f}\n"
        f"- final_val_accuracy: {base_best_val_accuracy:.4f}\n"
        f"- last_epoch_train_accuracy: {base_last_train_accuracy:.4f}\n"
        f"- last_epoch_val_accuracy: {base_last_val_accuracy:.4f}\n"
        f"- last_epoch_val_loss: {base_last_val_loss:.4f}\n"
        "\n## Classification Report\n\n"
        "```text\n"
        f"{report_text}"
        "```\n"
    )
    (args.artifacts_dir / "tflite_training_report_base.md").write_text(base_report, encoding="utf-8")
    print(f"普通训练基础产物已保存: {args.artifacts_dir / 'model_keras_base.keras'}")
    print(f"普通训练基础报告已保存: {args.artifacts_dir / 'tflite_training_report_base.md'}")

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
            batch_size=args.qat_batch_size,
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

        copied_qat_layers = base.copy_qat_weights_to_float_model(q_aware_model, export_model)
        print(f"QAT 权重已拷回普通 float 模型，层数={copied_qat_layers}")

        converter_qat_float = tf.lite.TFLiteConverter.from_keras_model(export_model)
        qat_float_model = converter_qat_float.convert()
        (args.artifacts_dir / "model_qat_float.tflite").write_bytes(qat_float_model)
        export_model.save(args.artifacts_dir / "model_keras.keras")
        final_train_accuracy = qat_history.history["accuracy"][-1]
        final_val_accuracy = qat_history.history["val_accuracy"][-1]
    else:
        export_model.save(args.artifacts_dir / "model_keras.keras")
        converter_ptq = tf.lite.TFLiteConverter.from_keras_model(export_model)
        converter_ptq.optimizations = [tf.lite.Optimize.DEFAULT]
        converter_ptq.representative_dataset = representative_dataset
        converter_ptq.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
        converter_ptq.inference_input_type = tf.int8
        converter_ptq.inference_output_type = tf.int8
        int8_model = converter_ptq.convert()
        final_train_accuracy = base_best_train_accuracy
        final_val_accuracy = base_best_val_accuracy

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
        ) = base.read_tflite_io_quantization(int8_model)
        print(f"int8 model input  quant: scale={int8_input_scale:.8f}  zero_point={int8_input_zero_point}")
        print(f"int8 model output quant: scale={int8_output_scale:.8f}  zero_point={int8_output_zero_point}")
    except Exception as exc:
        print(f"int8 模型量化参数检查跳过: {exc}")

    qat_accuracy_line = (
        f"- qat_val_accuracy: {qat_accuracy:.4f}\n" if qat_accuracy is not None else "- qat_val_accuracy: skipped\n"
    )
    report = (
        "# TensorFlow KWS Report (Radical)\n\n"
        f"- test_loss: {loss:.4f}\n"
        f"- test_accuracy: {accuracy:.4f}\n"
        f"- argmax_accuracy: {argmax_accuracy:.4f}\n"
        "- model_family: cnn-medium\n"
        "- cosine_lr: enabled\n"
        f"- warmup_epochs: {WARMUP_EPOCHS}\n"
        f"- early_stopping_monitor: {EARLY_STOPPING_MONITOR}\n"
        f"- early_stopping_mode: {EARLY_STOPPING_MODE}\n"
        f"- early_stopping_patience: {EARLY_STOPPING_PATIENCE}\n"
        f"- class_weight_silence: {class_weights[silence_index] if silence_index >= 0 else 1.0:.4f}\n"
        f"- class_weight_unknown: {class_weights[unknown_index] if unknown_index >= 0 else 1.0:.4f}\n"
        "- spec_augment: enabled\n"
        f"- label_smoothing: {LABEL_SMOOTHING:.4f}\n"
        f"- base_learning_rate: {BASE_LEARNING_RATE:.6f}\n"
        f"- qat_learning_rate: {QAT_LEARNING_RATE:.6f}\n"
        f"- qat_epochs: {args.qat_epochs}\n"
        f"- qat_batch_size: {args.qat_batch_size}\n"
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
        f"- base_best_epoch: {base_best_epoch}\n"
        f"- base_best_val_loss: {base_best_val_loss:.4f}\n"
        f"- base_best_val_accuracy: {base_best_val_accuracy:.4f}\n"
        f"- base_last_epoch_val_loss: {base_last_val_loss:.4f}\n"
        f"- base_last_epoch_val_accuracy: {base_last_val_accuracy:.4f}\n"
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
