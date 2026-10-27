# 嵌入式开发项目集

面向嵌入式软件开发的项目作品集，涵盖电机控制、端侧 AI、RTOS、多外设通信与视觉联动。

## 技术栈

- 微控制器：STM32G4/F1/F4/H7、Renesas RA8P1/RA6M5、TI MSPM0G3507、STC8。
- 实时系统与外设：裸机 HAL、FreeRTOS、ADC/PWM/DMA、UART、SPI、I2C、HTTP、JSON。
- 控制与算法：FOC、SVPWM、SMO/PLL、PID、位置控制、逆运动学、步进电机控制。
- 边缘计算与视觉：端侧语音推理、Raspberry Pi、OpenCV、MediaPipe、OpenMV。
- 开发工具：VS Code Embedded IDE、Keil MDK、STM32CubeIDE、CCS、嘉立创 EDA、SolidWorks、Python。

## 核心项目

| 编号 | 项目 | 主要内容 |
| --- | --- | --- |
| 01 | [STM32G474 FOC 电机控制器](projects/01-stm32g474-foc/) | 无感/有感 FOC、六步换相、位置与手感控制、PC 调参 |
| 02 | [RA8P1 AI 语音智能小车](projects/02-ra8p1-smart-car/) | 端侧语音识别与机器人控制 |
| 03 | [RA6M5 智能机器人](projects/03-ra6m5-robot-system/) | 多外设、机械臂与底盘联动 |
| 04 | [FreeRTOS 智能像素时钟](projects/04-freertos-pixel-clock/) | 实时任务、像素显示与网络交互 |
| 05 | [STM32 倒立摆](projects/05-inverted-pendulum/) | 运动控制与反馈调节 |
| 06 | [STM32H7 多旋翼飞控](projects/06-multirotor-flight-controller/) | 飞控搭建与调试展示 |
| 07 | [STC8 双单片机加热台](projects/07-stc8-hotplate-controller/) | 温度控制、执行与界面交互 |
| 08 | [MSPM0 竞赛控制小车](projects/08-ccs-m0-smart-car/) | 循迹、搬运与系统迭代 |
| 09 | [树莓派手势识别](projects/09-raspberrypi-gesture-serial/) | 手势识别与串口转发 |

## 附加项目

| 编号 | 项目 |
| --- | --- |
| 10 | [STM32 平衡小车](projects/optional/10-stm32-balance-car/) |
| 11 | [STM32 云台测距联调](projects/optional/11-stm32-gimbal-ball/) |
| 12 | [STM32 巡线小车](projects/optional/12-stm32-line-follower/) |
| 13 | [三路 UART 环形缓冲通信](projects/optional/13-rtos-multi-uart/) |
| 14 | [TMC2209 步进电机驱动](projects/optional/14-stepper-motor-driver/) |
| 15 | [STM32 电机与姿态接口实验](projects/optional/15-stm32-autonomous-car/) |

各项目的源码入口、硬件配置和使用方法见对应 README。第三方代码适用其原有许可。

第三方组件及声明位置见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
