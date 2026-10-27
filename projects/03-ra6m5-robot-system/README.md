# RA6M5 智能机器人控制系统

基于 Renesas RA6M5 的裸机机器人工程，使用主循环、定时节拍和状态机协调底盘、机械臂、雷达及外部视觉坐标。

## 源码入口

- [工程文件/GPT.uvprojx](工程文件/GPT.uvprojx)：Keil 工程。
- [hal_entry.c](工程文件/src/hal_entry.c)：初始化外设与控制模块，主循环处理按键、菜单和 `RobotControl_MainLoopTask()`。
- [Robot_Control.c](工程文件/Hardware/Robot_Control.c)：底盘与机械臂任务协调，包含等待、抓取、放置和错误状态。
- [Vision_Uart9_Comm.c](工程文件/Hardware/Vision_Uart9_Comm.c)：UART9 视觉数据收发、组帧、校验与缓存。
- [Navigation.c](工程文件/Hardware/Navigation.c)、[Calibration.c](工程文件/Hardware/Calibration.c)：导航相关处理与坐标标定实现。

## 使用范围

按实际底板核对 FSP 配置、串口、定时器、电机方向和机械臂坐标，再用 Keil MDK 打开工程。`ra/`、`ra_cfg/`、`ra_gen/` 为框架与生成配置；自有接口和控制逻辑集中在 `Hardware/`、`src/`。

当前入口包含调试启动设置；放置完成后保持预留状态，错误状态不自动恢复。标定工具和实验函数的存在不代表都已接入业务流程。外部设备承担视觉识别，本工程负责接收结果和执行协调。

## 展示资料

- [照片：329deac07c42ad96283deb0d9dd08bca.jpg](media/images/329deac07c42ad96283deb0d9dd08bca.jpg)
