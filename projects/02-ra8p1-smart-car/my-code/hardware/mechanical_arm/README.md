# 机械臂与标定

机械臂动作层接收目标坐标并转换为舵机命令。初始化只打开 UART4 和内部状态，不自动回零，避免上电动作；回零与运动由上层显式调用。

| 文件 | 职责 |
| --- | --- |
| [Mechanical_hand_control.c](Mechanical_hand_control.c) / [头文件](Mechanical_hand_control.h) | 动作接口、逆运动学和舵机姿态换算 |
| [Bus_servo.c](Bus_servo.c) / [头文件](Bus_servo.h) | 总线舵机命令、读写及协议封装 |
| [Mechanical_arm_feedback.c](Mechanical_arm_feedback.c) / [头文件](Mechanical_arm_feedback.h) | 位置反馈及机械臂 UART 总线互斥接口 |
| [Calibration.c](Calibration.c) / [头文件](Calibration.h) | 相机坐标到机械臂基座坐标的转换与标定参数 |
| [calc_calibration_params.py](calc_calibration_params.py) | 根据标定点计算参数的离线工具 |
| [平面标定模板](calibration_plane_template.csv) / [三维标定模板](calibration_3d_template.csv) | 标定工具的输入格式模板，不代表当前设备已完成标定 |

结构尺寸、相机安装或工作平面变化后需重新核对标定参数。参数含义和单位以头文件及求解代码为准。
