# STM32G474 多模式 FOC 电机控制器

基于 STM32G474RBT6 的裸机电机控制工程，集成无感 FOC、反电动势六步换相、MT6835 编码器反馈控制及 PC 调参与波形显示。

本目录展示固件 **1.10.2-g2208-smooth** 源码，配套 FocControl 上位机。固件使用 STM32 HAL，控制算法、板级驱动、参数存储与串口协议分层组织。

## 主要功能

- 无感 FOC：Clarke/Park 变换、dq 电流 PI、SVPWM、SMO 与 PLL 转子状态估计、启动接管及速度闭环。
- 六步控制：开环启动、反电动势过零检测、事件资格判断与定时换相。
- 编码器控制：MT6835 SPI/DMA 采集，支持有感六步、有感 FOC、独立 Iq、多圈位置及虚拟档位/阻尼交互。
- 采样与保护：PWM 同步 ADC 采样、电流零点校准，以及过流、母线电压、反馈异常和控制时序检查。
- 参数与通信：串口命令、Flash 参数持久化、参数备份恢复、二进制波形传输和 CRC 校验。
- FocControl：Qt 桌面界面，提供参数编辑、实时波形、位置与手感页面、模拟板卡及离线测试。

## 硬件与控制节拍

| 项目 | 配置 |
| --- | --- |
| MCU | STM32G474RBT6，Cortex-M4F，170 MHz |
| 功率级 | DRV8353F，三相 MOSFET 桥，50 mΩ 采样电阻 |
| 编码器 | MT6835，SPI/DMA 角度采集 |
| 当前电机配置 | G2208，7 极对；参数见 `project/app/motor_config.h` |
| FOC 节拍 | 20 kHz PWM、10 kHz dq 电流控制、1 kHz 速度环 |
| 编码器反馈 | 5 kHz 角度反馈；位置和手感控制按模式分频执行 |

## 配置与验证范围

当前默认配置为 G2208、7 极对，默认目标 600 RPM、运行 5 s，见 `project/app/motor_config.h`。R/L 为配置参数，磁链来自 PWM 电压模型估算；这些数值不代表所有负载与模式已经完成实机验证。原电机的历史结果不能直接用于这个配置。

主机回归使用硬件替身验证算法和协议；不替代实际板卡的电流采样、参数标定和保护测试。

## 源码导航

| 目录 | 内容 |
| --- | --- |
| [project/](project/) | 正式固件：app、control、board、devices、platform、protocol |
| [project/MDK-ARM/foc.uvprojx](project/MDK-ARM/foc.uvprojx) | Keil 工程，默认目标 `foc_dual` |
| [project-test/](project-test/) | 独立板卡自检与外设验证工程 |
| [FocControl/](FocControl/) | PC 上位机源码及其离线测试 |
| [tests/](tests/) | 控制算法、协议、存储和驱动的主机回归 |
| [tools/](tools/) | 构建、串口及命名参数预设工具 |

## 构建固件

安装 Keil MDK、Arm Compiler 6 和 STM32G4xx DFP；工程声明的 DFP 版本为 1.4.0，依赖使用工程内相对路径。可直接打开工程，也可在本目录执行：

```powershell
$env:KEIL_UVISION = '本机 UV4.exe 的完整路径'
pwsh -File tools/build.ps1
```

`foc_dual` 使用双 Bank 链接布局，Flash 参数区由 scatter 文件独立预留。移植时先核对芯片、DBANK、引脚、电流采样增益、电机参数及编码器方向，再进行标定与运行。

## 运行上位机

在 `FocControl` 目录安装依赖并启动模拟界面：

```powershell
pwsh -File setup.ps1
pwsh -File run.ps1 -Demo
```

连接实际板卡时运行 `run.ps1` 并选择本机串口；只读监视使用 `run.ps1 -ReadOnly`。具体操作见 [上位机说明](FocControl/README.md)。

## 主机回归

安装 GCC 后可在本目录运行 `tests/motor/run.ps1`、`tests/encoder/run.ps1`、`tests/interaction/run.ps1`；上位机测试在其目录使用 `python -m pytest`。测试替身与模拟器用于离线验证，实际电机参数按硬件配置设置。

`tests/display/run.ps1` 和 `tests/selftest/run.ps1` 默认选择 PATH 中第一套 GCC 并打印路径，也可通过 `-CompilerPath` 传入本机编译器完整路径。若 GCC/链接器不能处理当前中文目录，可在独立 ASCII 路径副本运行测试，不修改全局 PATH。

独立 `project-test` 的限流设置为 1.5 A／60 mV；自检应用测试仍保留旧 0.8 A 预期，因此存在 3 项已知连带失败。该差异不代表正式 `project` 保护失效，也不应通过降低实际门限或删除断言消除。

## 第三方组件

STM32 HAL 和 CMSIS 保留各自目录的许可及版权声明；上位机依赖见 [THIRD_PARTY_NOTICES.md](FocControl/THIRD_PARTY_NOTICES.md)。仓库许可证不替代第三方组件的原有许可。
