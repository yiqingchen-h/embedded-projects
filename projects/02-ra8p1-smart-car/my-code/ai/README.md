# KWS 训练与部署工具

本目录保存 PC 端语音数据处理、特征提取、训练与模型转换脚本。板端推理入口在 `my-code/software/voice_ai.c`，训练脚本不加入 Keil 工程。

## 输入与脚本

- [labels.txt](labels.txt)：当前 14 类标签顺序；部署时与模型输出和 MCU 标签表一致。不要沿用旧说明中的另一套标签。
- [scripts/](scripts/)：包含 TTS 流程试验、串口采音、音频裁剪与增强、特征提取、TensorFlow/QAT 训练、RUHMI 输入准备及部署源修补工具。
- [requirements.txt](requirements.txt)：基础依赖；GPU 训练环境见 [GPU_TRAIN_ENV.md](GPU_TRAIN_ENV.md)。按具体脚本的参数与依赖选择环境。

TTS 和轻量分类器用于流程试验，不能代替真实麦克风数据的板端验证。训练需要自行准备有公开或使用权限的语音样本；数据集、环境和校准输入在本地生成，已从提交范围排除。

## 部署

[ruhmi_generated/kws_current/](ruhmi_generated/kws_current/) 保存当前部署说明与最小 C/H 源码。Keil 工程直接引用其中 `output/build/MCU/compilation/src/` 的模型文件；它们与训练缓存分开管理。

重新训练后核对输入形状、标签顺序、归一化均值/标准差、量化参数及模型入口符号，再同步板端参数。当前 MCU 前处理参数为 16 kHz、98×40 log-Mel；配置和脚本不等同于跨环境复现实验已经通过。

## 本地模型配套

两套 TensorFlow 训练入口导出模型后会生成 `kws_model_manifest.json`，记录本次模型的 SHA-256、标签顺序、输入形状、特征均值/标准差和前处理配置。该文件与模型一起留在本地导出目录，不加入公开提交。

调试推理和音频裁剪的模型评分默认读取所选模型同目录的清单，也可通过 `--model-manifest` 指定。清单缺失、模型哈希或张量形状不匹配时停止；不会使用旧固定归一化常量。当前评分路径支持 float32 输入/输出、16 kHz、1 秒、98×40 log-mel 模型。

RUHMI 输入准备工具读取训练清单；其中登记 QAT float 模型时，使用该模型作为输入目录的 `model_float.tflite`，并生成对应清单。不能仅凭文件名把普通 float 与 QAT float 模型当作同一版本。已有模型需先依据对应训练记录建立并核对清单，不能用另一轮报告补配。

在本目录执行，例如：

```powershell
python scripts/run_debug_inference.py --model-path artifacts_real/model_float.tflite --wav-dir debug_model_input/voice_ai_debug
python scripts/07_trim_dataset_audio.py --use-model --model-path artifacts_real/model_float.tflite
```

纯 VAD 剪辑不要求模型或清单；明确启用 `--use-model` 后，配置错误会返回失败，不会继续以其他选样方式生成数据。调试脚本按 WAV 实际采样率重采样后提取特征；结果只用于当前模型的离线检查，不表示已经验证 MCU 的数值等价性或实机准确率。

## 转写、合成与设备选择

以下命令从本目录运行；所有默认目录均由脚本位置推导，工作目录变化不改变默认输入位置。命令中的 `GPU.1` 是显式设备示例，需按本机 OpenVINO 枚举结果选择，也可指定 `CPU`。OpenVINO 工具没有默认设备或自动回退。

```powershell
python scripts/batch_transcribe.py --input-dir dataset_real/raw --model small --device cpu
python scripts/batch_transcribe_GPU.py --input-dir dataset_real/trimmed --model-dir whisper-medium-openvino --device GPU.1
python scripts/test_openvino_whisper_gpu.py --input-dir dataset_real/raw/wake --model-dir whisper-base-openvino --device CPU
python scripts/07_trim_dataset_audio.py --whisper-score --whisper-model whisper-medium-openvino --whisper-device GPU.1
python scripts/generate_synth_voice_dataset.py --output-dir dataset_real/auto
```

两份转写脚本的 `--output-csv` 可覆盖输出位置；省略时 CSV 写入所选输入目录。合成工具保留原有生成数量和人工筛选流程，实际生成需要在线 TTS 服务。各工具的 `--help` 只显示参数，不加载推理依赖、下载模型或调用在线服务。

串口调试采样示例（在工程根目录执行）：

```powershell
python my-code/ai/scripts/06_capture_serial_dataset.py --port COM3 --label voice_ai_debug --count 0 --timeout 0 --out-dir my-code/ai/debug_model_input --allow-extra-labels
```

`COM3` 仅为串口示例；实际采集前需选择本机设备。训练和部署验证仍需独立核对 MCU 参数，配套清单不会自动修改板端源码。

## 安装依赖与输出目录

基础工具清单固定为本轮核对的 Python 3.11 本地环境直接依赖版本；按用途安装可选组。以下命令在 `my-code/ai` 执行：

```powershell
py -3.11 -m venv .venv
.venv/Scripts/python.exe -m pip install -r requirements.txt
# 转写和语音合成工具
.venv/Scripts/python.exe -m pip install -r requirements-asr.txt
# CPU TFLite 调试与可选模型导出工具
.venv/Scripts/python.exe -m pip install -r requirements-model-tools.txt
```

这些文件固定直接依赖，不是完整传递依赖锁；只安装所用工具组。GPU/DirectML 与 QAT 训练继续使用 [独立训练环境](GPU_TRAIN_ENV.md)，不能将 CPU 调试清单视为训练兼容性证明。本轮验证了模型加载与离线前处理，没有重新训练或重跑 ONNX 导出。

下列工具会清理对应输出：

| 工具 | 删除范围及条件 |
| --- | --- |
| `01_generate_tts_dataset.py` | 默认重建 `dataset_tts/raw`；需要保留原数据时传 `--keep-existing`。还会清理 `_cache/tts_tmp` 中间目录 |
| `05_export_onnx_optional.py` | 导出前删除所选产物目录中的 `saved_model_for_onnx`，用于重新生成中间模型 |
| `generate_synth_voice_dataset.py` | 仅 `CLEAR_OLD_OUTPUT=True` 时删除所选输出目录，当前默认 `False` |

覆盖输入或输出目录前核对路径；原始采音与合成输出使用不同目录管理。
