# 当前 KWS 部署文件

Keil 工程直接引用 [output/build/MCU/compilation/src/](output/build/MCU/compilation/src/) 中的模型 C/H 文件。该目录包含模型入口、网络计算、NPU 命令流和参数数据；保留生成器附带的版权与许可声明。

## 输入约定

MCU 侧输入为 98×40 log-Mel 特征。当前标签与前处理元数据分别见 [_inputs/labels.txt](_inputs/labels.txt)、[_inputs/preprocess_config.json](_inputs/preprocess_config.json)，数值同步还需核对 `my-code/software/voice_ai.c`。标签顺序和归一化参数应来自同一训练版本。

## 重新生成

训练完成后，使用 `my-code/ai/scripts/08_prepare_ruhmi_cli_inputs.py` 准备输入与本地校准数据，再使用本机安装的 RUHMI quantize/deploy 工具完成转换。部署路径对应 RA8P1 / Ethos-U55；导出后核对模型入口、张量大小、标签与 MCU 前处理参数。

校准特征、训练模型、工具提取目录与转换日志留在本地。本目录只发布固件引用的部署源码和必要元数据，不包含能够重新训练模型的私人数据集。
