# 树莓派手势识别与串口接口

使用 Picamera2/libcamera 采集 CSI 摄像头图像，MediaPipe Hands 提取关键点，OpenCV 显示识别结果。手势规则与串口接口集中在 [src/eRecognition_CSI.py](src/eRecognition_CSI.py)。

## 当前实现

通过关键点角度、距离与手指组合判断 14 类手势，包括数字、OK、Rock、点赞/点踩和单手比心。图像关键点由 MediaPipe 提供，手势规则由本文件组合实现。

主入口尝试以 115200 bps 打开 `/dev/serial0`。成功后发送手势字符串与换行符；同一手势至少间隔 0.3 s，不同手势立即发送。缺少 pyserial 或串口打开失败时继续显示图像。接收端需采用相同字符串约定。

## 运行环境

在支持 CSI 摄像头的树莓派系统安装 Picamera2、libcamera、OpenCV、MediaPipe、NumPy；启用 UART 时还需要 pyserial。运行 `python3 src/eRecognition_CSI.py`。不在这里声明未经测量的帧率或推理耗时。

## 展示资料

- [照片：6f952b99f9af21c0538dd196726020b1.jpg](media/images/6f952b99f9af21c0538dd196726020b1.jpg)
- [视频：7d87af5621d5446ab0825edb90d0d4ba.mp4](media/videos/7d87af5621d5446ab0825edb90d0d4ba.mp4)
