# RA8P1 端侧语音智能小车

基于 Renesas RA8P1、FreeRTOS 和 Ethos-U55，接入麦轮底盘、雷达、外部视觉坐标、机械臂与关键词识别。业务代码位于 `my-code/`，Keil 入口为 [ra8p1_template.uvprojx](ra8p1_template.uvprojx)。

## 当前实现

- `my-code/software/takes.c` 创建任务：GPT 中断发送通知，任务执行底盘控制、雷达解析、视觉处理和 Mission 状态推进。
- `my-code/software/voice_ai.c` 完成音频前处理、VAD、log-Mel、量化和结果判定；Ethos-U55 执行 CNN 推理。当前模型为 14 类关键词模型。
- `my-code/software/mission_task.c` 协调唤醒、目标选择、视觉坐标、移动和分步抓取/放置；机械臂动作使用非阻塞状态机。
- `my-code/hardware/Chassis/` 提供麦轮逆运动学、编码器测速和轮速 PI；N10 雷达提供基于规则的局部避障数据。
- `my-code/hardware/vision/` 接收外部视觉端 UART8 坐标并校验。视觉检测算法运行在外部设备，MCU 侧负责协议与坐标使用。

## 构建与模型

使用 Keil MDK、对应 RA8P1 DFP/FSP 和硬件配置打开工程。`ra/`、`ra_cfg/`、`ra_gen/` 保持框架与生成代码结构；调整板卡时核对串口、时钟和存储布局。

Keil 的 `voice_ai_model` 组引用 `my-code/ai/ruhmi_generated/kws_current/output/build/MCU/compilation/src/` 中的部署 C/H 文件。这部分随源码提供；训练环境、原始录音、校准样本、模型备份和工具安装包留在本地。训练与转换脚本位于 [my-code/ai/](my-code/ai/)，复现实验需要自行准备样本及对应工具版本。

当前避障不包含 SLAM 或全局路径规划；离线模型指标不能代表整车噪声环境下的识别率。机械臂反馈读取与任务动作协调，也不能直接称作视觉伺服或机械臂位置闭环。

## PC 工具依赖

使用 Python 3.11，在 `my-code/ai` 建立本地虚拟环境后安装 `requirements.txt`。转写/合成使用 `requirements-asr.txt`，CPU 模型调试使用 `requirements-model-tools.txt`；具体命令、版本与会清理输出目录的工具见 [AI 工具说明](my-code/ai/README.md)。GPU/QAT 训练环境单独管理。

LVGL 副本已裁去上游测试、示例、演示和生成工具，保留当前工程运行源码、必要支持及许可文本；配置仍在 `my-code/lv_conf.h`，部署 C/H 与控制实现未改变。
