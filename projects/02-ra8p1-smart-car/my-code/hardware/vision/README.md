# UART8 视觉通信

[Vision_Uart8_Comm.c](Vision_Uart8_Comm.c) 与 [头文件](Vision_Uart8_Comm.h) 实现 UART8 收发缓存、视觉协议解析和帧状态查询。上层通过头文件导出的接口初始化、处理接收字节流并读取最新帧。

- 接收数据先进入缓存，由处理接口解析后更新帧与统计状态。
- 头文件提供发送忙状态、格式错误与 UART 错误计数，以及测试帧注入接口。
- `VISION_UART8_ECHO_BACK_ENABLE` 当前默认为 `1`；回显用途与处理流程见源文件。
- 视觉识别本身由外部模块完成，本目录负责通信。

业务接入从 [software](../../software/) 中的任务和协调层追踪，FSP UART 配置位于项目生成配置中。
