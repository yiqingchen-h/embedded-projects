# STM32 三路红外循迹实验

Keil 入口为 [project.uvprojx](project.uvprojx)，主程序为 [User/main.c](User/main.c)。

程序初始化显示、电机和循迹输入，调用 `Fang_Xiang()` 获取状态，分别执行前进、左转、右转和丢线制动；OLED 显示 PB12、PB13、PB14 输入及方向状态。

当前入口采用状态判断与预设速度，不宣称接入 PID 路径跟踪。电机方向和速度档位需按实际底盘调试。

## 硬件与构建

工程目标为 STM32F103C8，Cortex-M3；工程配置的 Flash 为 64 KiB、RAM 为 20 KiB。三路红外/灰度输入 PB12、PB13、PB14、电机驱动和 OLED。输入模式与有效电平见 `Hard Ware/XunJi.c`，电机方向和 PWM 见 `Hard Ware/Speed.c` 及相关驱动。

1. 使用 Keil MDK 打开本目录 `project.uvprojx`，保留工程声明的目标和源文件分组。
2. 配置工程对应的 Arm 编译器及 STM32F1 器件包；核对 `Start/` 启动文件、系统时钟和头文件路径。主程序为 `User/main.c`。
3. 在 MDK 中构建所选目标，检查编译与链接诊断。构建目录留在本地，不加入公开源码。

## 下载与运行

选择实际使用的 SWD 调试器（例如 ST-Link 或 J-Link），核对 STM32F103C8 的 Flash 算法和起始地址 `0x08000000`。连接 SWDIO、SWCLK、GND 和参考电压，先断开电机执行电源进行下载，再核对驱动方向与输入状态。

本轮只静态核对工程引用，未执行构建或烧录；当前功能范围以上方实际主程序说明为准。
