# 硬件模块

外设驱动与硬件执行层位于本目录；任务和业务协调代码位于 [software](../software/)。

| 目录 | 内容 |
| --- | --- |
| [Chassis](Chassis/) | 底盘运动入口、轮速 PI、编码器、运动学、避障及串口电机接口 |
| [mechanical_arm](mechanical_arm/) | 总线舵机、机械臂求解、反馈及相机坐标标定 |
| [vision](vision/) | UART8 视觉协议收发与帧解析 |
| [LCD](LCD/) | MIPI ST7701P、SPI ST7789 及显示回调 |
| [Flash](Flash/) | Flash 读写 |
| [IMU](IMU/) | IMU 接口 |
| [INA226](INA226/) | 电压、电流检测及软件 I2C |
| [key](key/) | 按键输入 |
| [MAX98357A](MAX98357A/) | 音频输出接口 |
| [Microphone](Microphone/) | 麦克风采集接口 |
| [N10](N10/) | N10 雷达数据处理 |

具体启用路径由任务入口、模块配置宏和 FSP 配置共同决定，目录存在不表示所有功能同时运行。
