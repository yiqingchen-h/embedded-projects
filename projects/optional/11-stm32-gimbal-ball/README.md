# STM32 云台测距联调

使用 STM32、VL53L0X、PWM 和 OLED 进行外设联调。Keil 入口为 [project.uvprojx](project.uvprojx)，主程序为 [User/main.c](User/main.c)。

当前程序初始化串口、PWM、OLED 与 VL53L0X，设置测量模式，循环执行单次测距并显示毫米读数。舵机与 PWM 代码供联调使用，当前入口不实现视觉跟踪或小球位置闭环。

使用前核对传感器连接、I2C 地址和 PWM 引脚；VL53L0X 驱动保留 ST 原有声明。

## 硬件与构建

工程目标为 STM32F103C8，Cortex-M3；工程配置的 Flash 为 64 KiB、RAM 为 20 KiB。VL53L0X 距离传感器、OLED、PWM/舵机接口和串口。传感器 I2C 连接以 `Hard Ware/VL53L0X/` 中的板级接口为准，PWM 与串口引脚从对应初始化文件核对。

1. 使用 Keil MDK 打开本目录 `project.uvprojx`，保留工程声明的目标和源文件分组。
2. 配置工程对应的 Arm 编译器及 STM32F1 器件包；核对 `Start/` 启动文件、系统时钟和头文件路径。主程序为 `User/main.c`。
3. 在 MDK 中构建所选目标，检查编译与链接诊断。构建目录留在本地，不加入公开源码。

## 下载与运行

选择实际使用的 SWD 调试器（例如 ST-Link 或 J-Link），核对 STM32F103C8 的 Flash 算法和起始地址 `0x08000000`。连接 SWDIO、SWCLK、GND 和参考电压，先断开电机执行电源进行下载，再核对驱动方向与输入状态。

本轮只静态核对工程引用，未执行构建或烧录；当前功能范围以上方实际主程序说明为准。
