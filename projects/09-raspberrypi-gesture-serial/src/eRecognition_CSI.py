#!/usr/bin/env python3
# encoding: utf-8
"""
树莓派5 单目摄像头手势识别 + UART 串口控制
============================================
硬件：树莓派5 + Camera Module 3 (或兼容 CSI 摄像头)
输出：14 种手势字符串通过 /dev/serial0 发送给 MCU

手势列表：
  数字手势: Zero, One, Two, Three, Four, Five, Six, Seven, Eight
  特殊手势: OK, Rock(摇滚), Thumb_up(点赞), Thumb_down(踩), Heart_single(单手比心)

识别方法：
  - MediaPipe Hands 提取 21 个手部关键点
  - 拇指伸直: 三关节余弦角 > 150°  + 拇指相对于其他四指的位置
  - 其余四指伸直: 指尖 Y 坐标 < PIP 关节 Y 坐标（Y 轴朝下 = y 小 = 更高 = 伸直）
  - 手指组合 + 指间距离 + 关节角度 → 14 种手势判别
"""

import math
import time
import cv2 as cv
import numpy as np
import mediapipe as mp
import libcamera
from picamera2 import Picamera2

# 串口是可选的（如果在没有 GPIO 的设备上调试, serial 导入失败也能跑）
try:
    import serial
except ImportError:
    serial = None


class handDetector:
    """
    手部检测器：封装 MediaPipe Hands + 自写手势判别

    核心数据:
      self.lmList = [[id, x, y], ...]   21 个关键点的像素坐标
      self.tipIds = [4, 8, 12, 16, 20] 五个指尖的 landmark ID

    """

    def __init__(self, mode=False, maxHands=1, detectorCon=0.5, trackCon=0.5):
        # 五个指尖的 MediaPipe landmark ID
        self.tipIds = [4, 8, 12, 16, 20]

        # MediaPipe Hands 初始化
        self.mpHand = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,          # False = 视频流模式（利用帧间连续性）
            max_num_hands=maxHands,          # 1 = 只检测单手（控制场景需要确定性）
            min_detection_confidence=detectorCon,  # 首次检测阈值
            min_tracking_confidence=trackCon       # 持续跟踪阈值（比检测阈值低）
        )

        # self.lmList: 当前帧 21 个关键点的列表 [[id, x像素, y像素], ...]
        self.lmList = []

        # 骨架绘制样式：关键点(红点, 半径6) + 连线(绿线, 宽度2)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(
            color=(0, 0, 255), thickness=-1, circle_radius=6
        )
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(
            color=(0, 255, 0), thickness=2, circle_radius=2
        )

    # ──────────────────────────────────────────────
    #  基础几何计算
    # ──────────────────────────────────────────────

    def get_dist(self, point1, point2):
        """两点间欧氏距离 (像素单位)"""
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt((y1 - y2) ** 2 + (x1 - x2) ** 2)

    def calc_angle(self, pt1, pt2, pt3):
        """
        余弦定理计算三点形成的夹角（以 pt2 为顶点）

        参数: pt1, pt2, pt3 = self.lmList 中的索引（关键点索引）
        返回: 角度（度, 0-180）

        """
        point1 = self.lmList[pt1][1], self.lmList[pt1][2]   # (x, y) 像素坐标
        point2 = self.lmList[pt2][1], self.lmList[pt2][2]
        point3 = self.lmList[pt3][1], self.lmList[pt3][2]

        a = self.get_dist(point1, point2)
        b = self.get_dist(point2, point3)
        c = self.get_dist(point1, point3)

        try:
            radian = math.acos((a * a + b * b - c * c) / (2 * a * b))
            angle = radian / math.pi * 180
        except:
            # 浮点精度导致 a²+b²-c²/(2ab) 略 > 1.0 时 acos 会抛 ValueError
            angle = 0

        return abs(angle)

    # ──────────────────────────────────────────────
    #  MediaPipe 推理 + 关键点提取
    # ──────────────────────────────────────────────

    def findHands(self, frame, draw=True):
        """
        输入: frame = BGR numpy 数组 (640×480×3)
        输出: frame(含骨架绘制), img(仅骨架, 用于右侧调试窗口)
        副作用: 填充 self.lmList
        """
        self.lmList = []
        img = np.zeros(frame.shape, np.uint8)          # 黑底骨架图
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB) # MediaPipe 需要 RGB 输入
        self.results = self.hands.process(img_RGB)

        if self.results.multi_hand_landmarks:
            # 这里只取一只手（maxHands=1 保证了最多 1 只）
            hand_landmarks = self.results.multi_hand_landmarks[0]

            if draw:
                # 在原图上画骨架（给用户看）
                self.mpDraw.draw_landmarks(
                    frame, hand_landmarks,
                    self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec
                )

            # 在纯黑背景上也画骨架（用于 frame_combine 右侧窗口）
            self.mpDraw.draw_landmarks(
                img, hand_landmarks,
                self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec
            )

            # 提取 21 个关键点像素坐标 → self.lmList
            # lm.x, lm.y 是归一化坐标 (0~1), 乘以像素宽高得实际位置
            h, w, c = frame.shape
            for idx, lm in enumerate(hand_landmarks.landmark):
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.lmList.append([idx, cx, cy])    # [landmark_id, x像素, y像素]

        return frame, img

    def frame_combine(self, frame, src):
        """水平拼接两个画面（左:原图+骨架, 右:纯骨架）"""
        if len(frame.shape) == 3:
            frameH, frameW = frame.shape[:2]
            srcH, srcW = src.shape[:2]
            dst = np.zeros((max(frameH, srcH), frameW + srcW, 3), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        else:
            src = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
            frameH, frameW = frame.shape[:2]
            imgH, imgW = src.shape[:2]
            dst = np.zeros((frameH, frameW + imgW), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        return dst

    # ──────────────────────────────────────────────
    #  手指弯曲检测（两个算法，因为拇指和其余四指解剖结构不同）
    # ──────────────────────────────────────────────

    def fingersUp(self):
        """
        返回 5 个手指的伸直状态 [拇指, 食指, 中指, 无名指, 小指]
        1 = 伸直, 0 = 弯曲

        拇指伸直的判断（余弦角度法，因为拇指旋转了 90°）:
          拇指尖(4)→IP(3)→MCP(2) 角度 > 150° 且
          IP(3)→MCP(2)→基部(1) 角度 > 150°
          = 拇指接近 180° 直线 ≈ 伸直

        其余四指伸直的判断（Y 坐标比较法，简单高效）:
          指尖的 Y 坐标 < PIP 关节的 Y 坐标
          Y 轴在像素坐标中朝下，所以 y 更小 = 位置更高 = 手指直立伸直
        """
        fingers = []

        # ── 拇指单独判断 ──
        if (self.calc_angle(self.tipIds[0], self.tipIds[0] - 1, self.tipIds[0] - 2) > 150.0) and \
           (self.calc_angle(self.tipIds[0] - 1, self.tipIds[0] - 2, self.tipIds[0] - 3) > 150.0):
            fingers.append(1)      # 拇指伸
        else:
            fingers.append(0)      # 拇指弯

        # ── 其余四指（食指/中指/无名指/小指）──
        # 指尖 Y 坐标 < PIP 关节 Y 坐标 = 手指伸
        for idx in range(1, 5):
            if self.lmList[self.tipIds[idx]][2] < self.lmList[self.tipIds[idx] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)

        return fingers

    # ──────────────────────────────────────────────
    #  14 种手势判别（手写逻辑，没有用任何手势库）
    # ──────────────────────────────────────────────

    def get_gesture(self):
        """
        从当前帧的 21 个关键点判断手势类型。

        判别优先级（代码执行顺序 = 优先级）：
          1. Thumb_down / Thumb_up（拇指相对其余四指的位置）
          2. 按手指伸的数量分类（0~5 个手指伸）
          3. 特殊手势：Seven(握拳), Eight(八字)
          4. 指间距离微调（OK, Heart_single）
        """
        gesture = ""
        fingers = self.fingersUp()   # [拇指, 食指, 中指, 无名指, 小指] 每个 0 或 1

        # ── Thumb_down（拇指向下）──
        # 条件：拇指尖 Y 坐标 > 其余四指尖 Y 坐标（拇指在四指下方）
        if self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[1]][2] and \
           self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[2]][2] and \
           self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[3]][2] and \
           self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[4]][2]:
            gesture = "Thumb_down"

        # ── Thumb_up（点赞）──
        # 条件：拇指尖 Y < 其余四指尖 Y（拇指在四指上方）
        #       + 食指近端关节角度 < 150°（食指必须弯着，不是伸直的）
        elif self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[1]][2] and \
             self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[2]][2] and \
             self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[3]][2] and \
             self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[4]][2] and \
             self.calc_angle(self.tipIds[1] - 1, self.tipIds[1] - 2, self.tipIds[1] - 3) < 150.0:
            gesture = "Thumb_up"

        # ── 3~4 个手指伸 ──
        if fingers.count(1) == 3 or fingers.count(1) == 4:
            # OK 手势：拇指伸 + 拇指尖靠近食指尖
            if fingers[0] == 1 and (
                self.get_dist(self.lmList[4][1:], self.lmList[8][1:]) <
                self.get_dist(self.lmList[4][1:], self.lmList[5][1:])
                # 拇指尖(4)到食指尖(8)的距离 < 拇指尖(4)到食指MCP(5)的距离
                # = 拇指和食指捏在一起（形成 OK 圆圈）
            ):
                gesture = "OK"

            # Rock（摇滚手势）：中指(index=2)和无名指(index=3)弯曲 = 食指+小指伸
            elif fingers[2] == fingers[3] == 0:
                gesture = "Rock"

            # Three：正好 3 个手指伸（非 OK、非 Rock）
            elif fingers.count(1) == 3:
                gesture = "Three"
            else:
                gesture = "Four"

        # ── 0 个手指伸 → Zero（握拳）──
        elif fingers.count(1) == 0:
            gesture = "Zero"

        # ── 1 个手指伸 → One（具体哪根后续判定）──
        elif fingers.count(1) == 1:
            gesture = "One"

        # ── 2 个手指伸 → 需要细分 ──
        elif fingers.count(1) == 2:
            # Six（打电话）：拇指+小指
            if fingers[0] == 1 and fingers[4] == 1:
                gesture = "Six"

            # Eight（八字）：拇指伸+食指伸+食指尖到拇指的角度 > 90°
            elif fingers[0] == 1 and self.calc_angle(4, 5, 8) > 90:
                gesture = "Eight"

            # Heart_single（单手比心）：拇指+食指伸+两指尖贴近(< 50px)
            elif fingers[0] == 1 and fingers[1] == 1 and \
                    self.get_dist(self.lmList[4][1:], self.lmList[8][1:]) < 50:
                gesture = "Heart_single"

            # 普通的 Two
            else:
                gesture = "Two"

        # ── 5 个手指伸 → Five（全张开）──
        elif fingers.count(1) == 5:
            gesture = "Five"

        # ── Seven（握拳备用条件）──
        # 不论前面判出什么，如果四指尖都贴近拇指尖(< 60px) = 就是握拳
        if self.get_dist(self.lmList[4][1:], self.lmList[8][1:]) < 60 and \
           self.get_dist(self.lmList[4][1:], self.lmList[12][1:]) < 60 and \
           self.get_dist(self.lmList[4][1:], self.lmList[16][1:]) < 60 and \
           self.get_dist(self.lmList[4][1:], self.lmList[20][1:]) < 60:
            gesture = "Seven"

        # ── Eight（八字备用：拇指在上且食指完全伸直 > 150°）──
        if self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[1]][2] and \
           self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[2]][2] and \
           self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[3]][2] and \
           self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[4]][2] and \
           self.calc_angle(self.tipIds[1] - 1, self.tipIds[1] - 2, self.tipIds[1] - 3) > 150.0:
            gesture = "Eight"

        return gesture


# ═══════════════════════════════════════════════════════════════
#  串口通信（通过树莓派 GPIO14/15 硬件 UART 发给 MCU）
# ═══════════════════════════════════════════════════════════════

def open_serial(port='/dev/serial0', baudrate=115200):
    """打开硬件串口 /dev/serial0 = GPIO14(TX) + GPIO15(RX)"""
    if serial is None:
        print("没有 serial 模块，串口部分先跳过")
        return None

    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        # timeout=0.1 为串口读取超时；写入未设置 write_timeout。
        print("串口打开成功:", port, baudrate)
        return ser
    except Exception as e:
        print("串口打开失败:", e)
        return None


def send_gesture(ser, gesture):
    """发送手势字符串 + 换行符给 MCU"""
    if ser is None or not gesture:
        return

    try:
        ser.write((gesture + '\n').encode('utf-8'))
    except Exception as e:
        print("串口发送失败:", e)


'''
所有可识别的手势字符串（MCU 端根据这些字符串做动作映射）:
  Zero One Two Three Four Five Six Seven Eight
  OK: OK手势
  Rock: 摇滚手势（食指+小指伸, 其余弯）
  Thumb_up: 点赞（拇指朝上）
  Thumb_down: 拇指向下
  Heart_single: 单手比心（拇指+食指指尖靠近）
'''


# ═══════════════════════════════════════════════════════════════
#  主程序入口
# ═══════════════════════════════════════════════════════════════

if __name__ == '__main__':
    cv.startWindowThread()

    # ── 摄像头初始化 ──
    # Picamera2 + libcamera：树莓派5 官方相机库
    # 640×480 RGB888：MediaPipe 内部缩放到 256×256, 640 够用且省 ISP 带宽
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": 'RGB888', "size": (640, 480)}
    )
    # vflip=1：摄像头排线方向导致画面上下颠倒, 硬件垂直翻转纠正
    # hflip=0：不需要水平镜像（如果画面左右反了改成 1）
    config["transform"] = libcamera.Transform(hflip=0, vflip=1)
    picam2.configure(config)
    picam2.start()

    # ── 串口初始化 ──
    ser = open_serial('/dev/serial0', 115200)

    # ── FPS 计时变量 ──
    pTime = 0
    cTime = 0

    # ── 手势检测器 ──
    # maxHands=1: 只检测一只手（控制场景, 双手会混淆）
    # detectorCon=0.75: 首次检测需 75% 置信度（避免误检）
    # trackCon=0.5(默认): 持续跟踪可放松到 50%（视频流帧间连续性）
    hand_detector = handDetector(maxHands=1, detectorCon=0.75)

    # ── 手势去抖 ──
    last_gesture = ""       # 上一帧发送的手势
    last_send_time = 0      # 上一次发送的时间戳
    send_interval = 0.3     # 相同手势至少隔 0.3s 才重发（不同手势立即发）

    while True:
        # 1. 从摄像头取一帧（libcamera DMA 零拷贝, CPU 0%）
        frame = picam2.capture_array()
        # cv.flip(frame, 1)  # 水平镜像（注释掉了, 当前不需要）

        # 2. MediaPipe 提取 21 个关键点
        frame, img = hand_detector.findHands(frame, draw=False)

        # 3. 如果有手 → 判手势 → 串口发送
        if len(hand_detector.lmList) != 0:
            totalFingers = hand_detector.get_gesture()

            # 显示手势名（画面底部绿色横条 + 文字）
            cv.rectangle(frame, (0, 430), (260, 480), (0, 255, 0), cv.FILLED)
            cv.putText(frame, str(totalFingers), (10, 470),
                       cv.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

            now = time.time()
            # 发送条件：手势非空 且 (手势变了 或 上次发送超 0.3s)
            if totalFingers != "" and (
                totalFingers != last_gesture or (now - last_send_time) > send_interval
            ):
                send_gesture(ser, totalFingers)
                last_gesture = totalFingers
                last_send_time = now

        # 4. 按 q 退出（用于调试, 实际部署不需要）
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        # 5. 实时 FPS 显示（画面左上角）
        cTime = time.time()
        if cTime != pTime:
            fps = 1 / (cTime - pTime)
        else:
            fps = 0
        pTime = cTime

        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (10, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)

        # 6. 显示：左半=原图, 右半=纯骨架图（方便调试看 landmark 位置）
        dist = hand_detector.frame_combine(frame, img)
        cv.imshow('dist', dist)

    # 清理资源
    if ser is not None:
        ser.close()

    picam2.stop()
    cv.destroyAllWindows()
