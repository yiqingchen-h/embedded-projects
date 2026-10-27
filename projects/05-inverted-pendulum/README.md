# STM32 倒立摆控制调试工程

使用 VS Code 的 Embedded IDE 扩展（`cl.eide`）管理工程。ADC 读取摆杆角度，编码器反馈速度与位移，PWM 驱动电机；当前代码包含角度稳定、目标位置调整、启停和越界停机逻辑。

## 工程入口与硬件

- [VS Code 工作区](工程文件/stm32_hel_DaoLiBai.code-workspace) 与 [EIDE 配置](工程文件/.eide/eide.json) 是实际工程入口，不使用 Keil 的 `.uvprojx`。
- [主程序](工程文件/user/main.c) 初始化 OLED、PWM、编码器、ADC、按键和定时器；按键用于记录角度零点、启停与调整目标位置。
- EIDE 的目标宏为 `STM32F103xB`，Cortex-M3，配置 Flash 为 64 KiB、RAM 为 20 KiB；具体板卡型号与晶振需按硬件核对。
- 摆杆电位器接 ADC1 通道 7（PA7），编码器与电机接口见 [speed.c](工程文件/HardWare/speed.c)，控制及定时回调见 [Time.c](工程文件/HardWare/Time.c)。供电与驱动连接按实际电机板确定。

## 构建

1. 安装 VS Code 和 Embedded IDE（`cl.eide`），打开上述工作区，在 EIDE 中加载现有项目。
2. 选择 `Debug`，配置 Arm Compiler 5（AC5）的本机安装路径。这里使用的是编译器工具链，不要求使用 Keil IDE；不要仅因环境缺少 AC5 就直接切换编译器。
3. 确认 `HardWare/`、`CMSIS/`、`user/`、`Library/` 源目录以及配置中的启动文件排除项。若插件提示需要器件包，按配置安装 STM32F1xx DFP 2.2.0；仓库不附带离线安装包。
4. 使用 EIDE 的构建入口，输出写入被忽略的 `工程文件/build/`。核对链接布局、系统时钟、头文件和编译器诊断。

## 下载与调试

工程配置使用 OpenOCD、ST-Link、`stm32f1x` 目标，Flash 起始地址为 `0x08000000`。配置 OpenOCD 路径后，将调试器通过 SWDIO、SWCLK、GND 和目标参考电压连接，再使用 EIDE 的下载入口。

初次下载时先断开电机执行电源，核对 ADC 零点、编码器方向、电机方向和停机逻辑，再进行台架调试。当前源码保留了不同调试阶段的参数，不能据此推断稳定范围或定位精度。本轮只静态核对项目配置和引用，未执行构建或烧录。
