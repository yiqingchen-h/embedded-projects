# KWS GPU 训练环境

`scripts/00_setup_gpu_train_conda_env.ps1` 创建独立 conda prefix 环境；默认位置是本目录的 `.gpu-train/`，可通过 `-EnvDir` 指定本机路径。脚本锁定 TensorFlow CPU 2.10.0、DirectML 插件等训练依赖，具体版本以脚本为准。

在工程根目录执行：

```powershell
pwsh -File my-code/ai/scripts/00_setup_gpu_train_conda_env.ps1
```

创建成功后，用 `.gpu-train/python.exe` 运行 `scripts/04_train_tflite_kws_radical.py`；数据和导出路径通过该脚本参数传入。`--help` 可查看选项。训练类别数由实际标签和数据决定，不将旧实验的 22 类说明用于当前 14 类模型。

采音/转写环境、GPU 训练环境和 RUHMI 转换环境分别安装。GPU 枚举和包兼容性需在本机验证；运行环境、模型缓存与私有语音不提交。

训练环境的 GPU 选择与 OpenVINO 转写设备参数相互独立。转写和单音频检查必须提供 `--device`；音频裁剪启用 Whisper 评分时必须提供 `--whisper-device`，用法见 [工具说明](README.md)。设备编号不能从某台电脑的显卡名称推断。
