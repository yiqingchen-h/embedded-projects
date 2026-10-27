# 麦轮底盘

[Chassis.h](Chassis.h) 导出 `chassis` 接口，供上层初始化、设置目标速度、切换控制模式和读取状态。

| 文件 | 职责 |
| --- | --- |
| [Chassis.c](Chassis.c) / [Chassis.h](Chassis.h) | 底盘控制入口与模式协调 |
| [motor.c](motor.c) / [motor.h](motor.h) | AT8236 电机 PWM 输出 |
| [encoder.c](encoder.c) / [encoder.h](encoder.h) | GPT 编码器测速 |
| [kinematics.c](kinematics.c) / [kinematics.h](kinematics.h) | 麦轮运动学换算 |
| [Chassis_pid.c](Chassis_pid.c) / [Chassis_pid.h](Chassis_pid.h) | 四路轮速增量 PI |
| [chassis_avoidance.c](chassis_avoidance.c) / [chassis_avoidance.h](chassis_avoidance.h) | 避障状态及速度约束接口 `chassis_avoidance` |
| [chassis_param.c](chassis_param.c) / [chassis_param.h](chassis_param.h) | 底盘参数管理 |
| [uart_motor.c](uart_motor.c) / [uart_motor.h](uart_motor.h) | UART3 四路串口电机板通信 |
| [Chassis_config.h](Chassis_config.h) | 底盘配置、参数和后端选择 |

PWM 电机与串口电机是不同执行后端，按配置选择；实际业务调用可从 [software](../../software/) 的任务代码及 `Chassis.c` 追踪。
