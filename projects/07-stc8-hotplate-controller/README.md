# STC8 双单片机加热台

两颗 STC8G1K08A 通过串口分工：芯片 A 负责温度采样与加热输出，芯片 B 负责按键、OLED 和界面交互。

## 工程组织

- [chip-a/project/chip-a.uvproj](chip-a/project/chip-a.uvproj)：温控端 Keil C51 工程，业务位于 `chip-a/app/`，外设位于 `bsp/`、`drivers/`。
- [chip-b/project/chip-b.uvproj](chip-b/project/chip-b.uvproj)：界面端工程，保存按键、显示、风扇及串口协调代码。

软件包含 NTC 查表换算、量程切换和标定修正，以及整数 PID 与时间比例加热输出。通信使用 `C,<set_temp>,<run>` 控制帧和 `S,<set_temp>,<cur_temp>,<fan>,<heat>,<run>` 状态帧。

## 当前边界

保留双机联调与高温标定阶段的实现。高温段标定及 PID 参数仍需结合实际探头和负载验证，不由源码推断控温精度、连续运行时间或产品级可靠性。烧录前按实际板卡核对引脚、电源和温度标定参数。
