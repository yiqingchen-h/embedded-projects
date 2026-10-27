# STM32F407 FreeRTOS 智能像素时钟

基于 STM32F407、FreeRTOS 和 32×8 WS2812B 矩阵，实现时间显示、场景切换、温湿度采样、风扇控制与联网信息获取。

## 源码与任务

- [main.c](工程文件/user/main.c)：初始化 HAL、RTC、串口、I2C、传感器和显示，然后调用 `freertos_start()`。
- [My_FreeRTOS.c](工程文件/FreeRTOS/My_FreeRTOS/My_FreeRTOS.c)：创建任务，协调按键、显示、AHT20、风扇与 ESP8266。
- [WS2812B.c](工程文件/HardWare/WS2812B.c)：CPU 准备像素及 PWM 数据，DMA 推送波形；显示任务按 15 ms 周期调度，该周期不是实测帧率。
- [ESP_12F.c](工程文件/HardWare/ESP_12F.c)：ESP8266 AT 命令、Wi-Fi 扫描与连接、NTP 及 HTTP 天气请求。
- `工程文件/System/Src/UART.c`：包含串口及接收处理；`工程文件/HardWare/` 保留 AHT20、AT24C512 等驱动。

启动任务创建 LED、诊断、温湿度、网络初始化、显示、按键、场景及风扇任务后自删除；网络初始化任务在连接成功后创建对时和天气任务，再自删除。因此不将创建调用总数写作“11 个常驻并发任务”。

## 本地网络配置

将 [pixel_clock_config.example.h](工程文件/pixel_clock_config.example.h) 复制为同目录的 `pixel_clock_config.local.h`，在本机填写 Wi-Fi 名称、密码和天气服务 Key。这个本地文件已被忽略。支持 `__has_include` 的编译器会自动载入；其他编译器需定义 `PIXEL_CLOCK_USE_LOCAL_CONFIG`。

未配置时网络任务不连接空 SSID，天气函数不发送空 Key 请求；本地显示和传感器逻辑仍可使用。代码中不包含可用于联网的默认凭据。

## 工程使用与边界

Keil 工程位于 [工程文件/Project/](工程文件/Project/)，按实际 STM32F407 型号、时钟和引脚选择目标。显示参数以 `WS2812B.h` 的 32×8 定义为准。

本版本不宣称零 CPU 占用、自动网络自修复、毫秒级对时精度或未经记录的长期稳定性指标。PCB 层数、加工公差和性能测试需另提供设计文件与测量条件。

## 展示资料

- [照片：4f39a6062636ccc4e17d964a94b3f03b.jpg](media/images/4f39a6062636ccc4e17d964a94b3f03b.jpg)
- [照片：ae4555604757972d01cd3a803e44d93c.jpg](media/images/ae4555604757972d01cd3a803e44d93c.jpg)
- [照片：b50ab41160f8189cd50793db52e99e84.jpg](media/images/b50ab41160f8189cd50793db52e99e84.jpg)
