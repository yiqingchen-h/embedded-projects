# STM32 步进电机驱动实验

工程保留 TMC2209 UART 配置试验与脉冲运动驱动。当前 [main.c](工程文件/user/main.c) 调用 `Stepper_Init(0)` 和 `Stepper_Move(90, GPIO_PIN_SET, subdivision_64)`；UART 初始化、`TMC2209_Init()` 和其运动示例没有在当前入口执行。

## 代码范围

- `工程文件/HardWare/Stepper_motor_drive.*`：步进输出与细分配置接口。
- `工程文件/user/main.c`：初始化、CRC 和 UART 试验代码，以及当前脉冲驱动调用。
- `工程文件/HardWare/OLED*`：显示与字模，保留原始版权。

[工程文件/](工程文件/) 当前保存源码与 HAL/CMSIS 依赖，未提供 Keil 工程文件，构建时需自行建立对应芯片工程。配置中的细分数是指令分辨率，不是实测机械定位精度；本目录不据此宣称 0.007° 精度、失速检测或完整寄存器读写链路已经运行。
