# STM32 平衡小车实验工程

保留 MPU6050/DMP、编码器、电机驱动和 PID 调试代码。Keil 入口为 [project.uvprojx](project.uvprojx)，主程序为 [User/main.c](User/main.c)。

当前主程序初始化 OLED、测速和 DMP，设置电机输出并读取姿态；部分控制定时器调用处于注释状态。不能将目录中的 PID 与姿态驱动直接表述为当前入口已完成自平衡控制。

公开入口保留外层工程；与其存在差异的内层旧版本已另行归档。MPU/DMP 及厂商库保留原有版权声明。

## 硬件与构建

工程目标为 STM32F103C8，Cortex-M3；工程配置的 Flash 为 64 KiB、RAM 为 20 KiB。MPU6050/DMP、编码器、直流电机驱动和 OLED。接线与方向配置分别见 `Hard Ware/Mpu6050/`、`Hard Ware/Speed.c`、`Hard Ware/PWM.c` 与 `Hard Ware/OLED.c`。

1. 使用 Keil MDK 打开本目录 `project.uvprojx`，保留工程声明的目标和源文件分组。
2. 配置工程对应的 Arm 编译器及 STM32F1 器件包；核对 `Start/` 启动文件、系统时钟和头文件路径。主程序为 `User/main.c`。
3. 在 MDK 中构建所选目标，检查编译与链接诊断。构建目录留在本地，不加入公开源码。

## 下载与运行

选择实际使用的 SWD 调试器（例如 ST-Link 或 J-Link），核对 STM32F103C8 的 Flash 算法和起始地址 `0x08000000`。连接 SWDIO、SWCLK、GND 和参考电压，先断开电机执行电源进行下载，再核对驱动方向与输入状态。

本轮只静态核对工程引用，未执行构建或烧录；当前功能范围以上方实际主程序说明为准。
