"""Qt 与后台 asyncio 的桥接，所有实际串口访问在同一线程。"""

from __future__ import annotations

import asyncio
import queue
import time
from collections import deque
from pathlib import Path
from typing import cast

from PySide6.QtCore import QThread, Signal

from .binary_wave import BinaryRecorder, WaveBuffer
from .controller import Controller
from .models import BoardSnapshot, ControlError, ParameterValue
from .position_feedback import PositionPoint
from .protocol import PlotPoint
from .simulator import DemoTransport
from .storage import Journal, Preset
from .transport import NativePort, Transport

# 展示用队列有界；每个实时行仍先写入原始会话日志。
MAX_UI_LOGS = 500


class SerialWorker(QThread):
    """主线程仅投递意图，后台按30Hz合并数据。"""

    state_changed = Signal(object)
    points_received = Signal(object)
    position_received = Signal(object)
    wave_received = Signal(object)
    logs_received = Signal(object)
    operation_done = Signal(str, object)
    operation_failed = Signal(str, str)

    def __init__(self, port: str, baud: int, data_dir: Path, read_only: bool, demo: bool) -> None:
        """@brief 创建后台线程。@param port 端口。@param baud 波特率。@param data_dir 数据。
        @param read_only 只读模式。@param demo 离线演示。
        """
        super().__init__()
        self.port, self.baud, self.data_dir = port, baud, data_dir
        self.read_only, self.demo = read_only, demo
        self.requests: queue.SimpleQueue[tuple[str, object]] = queue.SimpleQueue()
        self.force_requested = False
        self.latest = BoardSnapshot(read_only=read_only, busy="正在连接")
        self.log_batch: deque[tuple[str, str]] = deque(maxlen=MAX_UI_LOGS)
        self.point_batch: deque[PlotPoint] = deque(maxlen=2048)
        self.position_batch: deque[PositionPoint] = deque(maxlen=2048)
        self.journal: Journal | None = None
        self.wave: WaveBuffer | None = None
        self.wave_revision = -1
        self.recorder: BinaryRecorder | None = None
        self.recorder_error_reported = ""
        self.preferred_wave_baud = 0
        self.fallback_baud = 2000000

    def submit(self, action: str, payload: object = None) -> None:
        """@brief 从界面投递操作。@param action 操作名。@param payload 已验证参数。"""
        self.requests.put((action, payload))

    def force_close(self) -> None:
        """@brief 用户明确强制关闭，绝不报告已停止。"""
        self.force_requested = True

    def run(self) -> None:
        """@brief 在线程内创建 asyncio 和原生串口。"""
        try:
            asyncio.run(self._main())
        except (OSError, ControlError, ValueError) as error:
            self.state_changed.emit(
                BoardSnapshot(read_only=self.read_only, error=str(error), run="UNKNOWN")
            )
            self.operation_failed.emit("connect", str(error))

    def _state(self, state: BoardSnapshot) -> None:
        """@brief 暂存独立快照。@param state 最新状态。"""
        self.latest = state

    def _log(self, kind: str, text: str) -> None:
        """@brief 原文落盘与有界 UI 队列。@param kind 类型。@param text 原文。"""
        if self.journal is not None:
            self.journal.write(kind, text)
        if kind != "DATA":
            self.log_batch.append((kind, text))

    def _flush(self) -> None:
        """@brief 合并发布，防止逐帧 Qt 信号淹没界面。"""
        self.state_changed.emit(self.latest)
        if self.wave is not None and self.wave.revision != self.wave_revision:
            self.wave_revision = self.wave.revision
            self.wave_received.emit(self.wave)
        if self.recorder is not None and self.recorder.error:
            message = self.recorder.error
            if message != self.recorder_error_reported:
                self.recorder_error_reported = message
                self.log_batch.append(("ERROR", message))
        if self.position_batch:
            self.position_received.emit(list(self.position_batch))
            self.position_batch.clear()
        if self.point_batch:
            points = list(self.point_batch)
            self.point_batch.clear()
            self.points_received.emit(points)
        if self.log_batch:
            records = list(self.log_batch)
            self.log_batch.clear()
            self.logs_received.emit(records)

    async def _perform(self, controller: Controller, action: str, payload: object) -> None:
        """@brief 执行一项可取消操作。@param controller 控制器。@param action 名称。@param payload 参数。"""
        labels = {
            "apply": "应用参数",
            "start": "应用并启动",
            "stop": "正在停止",
            "disconnect": "停止并断开",
            "algorithm": "切换算法",
            "mode": "备份并切换模式",
            "position_zero": "建立机械零点",
            "position_initialize": "位置初始化",
            "recalibrate": "重新标定并初始化",
            "position_tolerance": "设置位置容差",
            "remote": "取得上位机控制",
            "clear": "重配置与校准",
            "calibrate": "编码器七点标定",
            "backup": "读取参数备份",
            "restore": "恢复参数预设",
        }
        controller.update(busy=labels.get(action, action), error="")
        result: object = None
        try:
            if action == "wave_baud":
                value = int(cast(int, payload))
                if value not in (0, 921600, 1000000, 2000000, 3000000):
                    raise ControlError("不支持的波形速率")
                controller.preferred_wave_baud = value
                controller.update(preferred_wave_baud=value)
            elif action == "apply":
                await controller.apply(cast(dict[str, ParameterValue], payload))
            elif action in ("start", "haptic_enable"):
                if controller.state.mode == "HAPTIC":
                    await controller.enable_haptic(cast(dict[str, ParameterValue], payload))
                else:
                    await controller.start_motor(cast(dict[str, ParameterValue], payload))
            elif action == "stop":
                await controller.stop_motor()
            elif action == "disconnect":
                await controller.disconnect()
            elif action == "algorithm":
                await controller.select_algorithm(cast(str, payload))
            elif action == "mode":
                await controller.select_mode(cast(str, payload))
            elif action == "position_zero":
                await controller.zero_position(cast(dict[str, ParameterValue] | None, payload))
            elif action == "position_initialize":
                await controller.initialize_position(
                    cast(dict[str, ParameterValue] | None, payload)
                )
            elif action == "recalibrate":
                draft = cast(dict[str, ParameterValue] | None, payload)
                if controller.state.mode == "POSITION":
                    await controller.initialize_position(draft, force_calibration=True)
                else:
                    await controller.calibrate_encoder(draft)
            elif action == "position_tolerance":
                await controller.position_tolerance(int(cast(int, payload)))
            elif action == "remote":
                await controller.take_remote()
            elif action == "clear":
                await controller.clear_fault()
            elif action == "calibrate":
                await controller.calibrate_encoder(cast(dict[str, ParameterValue] | None, payload))
            elif action == "backup":
                preset = await controller.capture(cast(str, payload))
                path = self.data_dir / "presets" / f"{preset.name}.json"
                preset.save(path)
                result = path
            elif action == "restore":
                await controller.restore(cast(Preset, payload))
            # 先发布已验证状态再发完成信号，关闭窗口不能依赖前一帧的缓存。
            self.latest = controller.state
            self._flush()
            self.operation_done.emit(action, result)
        except asyncio.CancelledError:
            self._log("AUDIT", "用户停止请求取消操作：" + action)
            raise
        except (OSError, ControlError, ValueError) as error:
            if not controller.session.closed and controller.state.connected:
                try:
                    await controller.refresh_status()
                    if controller.state.run != "RUNNING":
                        await controller.read_diagnostics()
                except (OSError, ControlError, ValueError):
                    pass
            message = str(error)
            if action in ("disconnect", "stop"):
                message = "停止未确认：" + message
            controller.error(message)
            self.operation_failed.emit(action, message)
        finally:
            controller.update(busy="")

    async def _poll(self, controller: Controller) -> None:
        """@brief 轮询失败发布未知状态，不自动清故障。@param controller 控制器。"""
        try:
            await controller.poll()
        except (OSError, ControlError, ValueError) as error:
            controller.error(str(error))

    async def _poll_position(self, controller: Controller) -> None:
        """位置快照异常时撤销旧显示，STOP仍可抢占普通请求。"""
        try:
            if controller.state.mode == "HAPTIC":
                await controller.read_haptic()
            else:
                await controller.read_position()
        except (OSError, ControlError, ValueError) as error:
            controller.update(position=None, haptic=None)
            controller.error(str(error))

    @staticmethod
    async def _cancel(task: asyncio.Task[None] | None) -> None:
        """@brief 取消且消费任务结束。@param task 任务。"""
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    async def _main(self) -> None:
        """@brief 保持接收、操作、轮询与界面发布独立。"""
        transport: Transport = (
            DemoTransport(
                "demo-position",
                "ENCODER_FOC",
                g2208=True,
                current_support=True,
                position_support=True,
                haptic_support=True,
            )
            if self.demo
            else NativePort(self.port, self.baud)
        )
        controller = Controller(
            transport,
            self.data_dir,
            self.read_only,
            self._state,
            self._log,
            self.point_batch.append,
        )
        controller.preferred_wave_baud = self.preferred_wave_baud
        controller.fallback_baud = self.fallback_baud
        controller.on_position = self.position_batch.append
        self.wave = controller.wave_buffer
        action_task: asyncio.Task[None] | None = None
        poll_task: asyncio.Task[None] | None = None
        init_task: asyncio.Task[None] | None = None
        try:
            self.journal = Journal(self.data_dir / "logs", "DEMO" if self.demo else self.port)
            self.recorder = BinaryRecorder(self.journal.path.with_suffix(".focwave"))
            controller.session.on_wire = self.recorder.append
            if self.demo:
                self._log("INFO", "离线演示：所有数据来自模拟板卡，不访问任何串口")
            init_task = asyncio.create_task(controller.initialize())
            interrupted = False
            while not init_task.done():
                self.latest = controller.state
                self._flush()
                if self.force_requested:
                    await self._cancel(init_task)
                    return
                if not self.requests.empty():
                    action, payload = self.requests.get()
                    if action in ("stop", "disconnect"):
                        await self._cancel(init_task)
                        if controller.state.connected:
                            await self._perform(controller, action, payload)
                            if not controller.session.closed:
                                controller.update(
                                    algorithm=await controller.get("ALGORITHM"),
                                    mode=await controller.get("MODE"),
                                    source=await controller.get("SOURCE"),
                                )
                                await controller.read_parameters()
                        else:
                            await controller.session.close()
                            controller.update(connected=False, busy="", run="UNKNOWN")
                            if self.read_only and action == "disconnect":
                                self.operation_done.emit(action, None)
                            else:
                                message = "连接验证尚未完成，板卡状态未知，未确认停机和输出禁用"
                                controller.error(message)
                                self.operation_failed.emit(action, message)
                        interrupted = True
                        break
                    self.operation_failed.emit(action, "连接验证尚未完成")
                await asyncio.sleep(0.05)
            if not interrupted:
                await init_task
            if not controller.session.closed:
                self.operation_done.emit("connect", None)
            last_publish, last_poll = 0.0, time.monotonic()
            last_position = 0.0
            while not controller.session.closed and not self.force_requested:
                while not self.requests.empty():
                    action, payload = self.requests.get()
                    if action in ("stop", "disconnect"):
                        await self._cancel(action_task)
                        await self._cancel(poll_task)
                        poll_task = None
                        action_task = asyncio.create_task(
                            self._perform(controller, action, payload)
                        )
                    elif action_task is None or action_task.done():
                        await self._cancel(poll_task)
                        poll_task = None
                        action_task = asyncio.create_task(
                            self._perform(controller, action, payload)
                        )
                    else:
                        self.operation_failed.emit(action, "正在处理上一项操作")
                now = time.monotonic()
                if now - last_publish >= 1 / 30:
                    self.latest = controller.state
                    self._flush()
                    last_publish = now
                if (
                    now - last_poll
                    >= (
                        0.2
                        if controller.preferred_wave_baud == 0 and controller.state.run == "RUNNING"
                        else 1
                    )
                    and (action_task is None or action_task.done())
                    and (poll_task is None or poll_task.done())
                ):
                    poll_task = asyncio.create_task(self._poll(controller))
                    last_poll = now
                elif (
                    (
                        controller.state.position_available
                        and controller.state.mode == "POSITION"
                        or controller.state.haptic_available
                        and controller.state.mode == "HAPTIC"
                    )
                    and now - last_position
                    >= (
                        0.2
                        if controller.preferred_wave_baud == 0
                        else 0.1 if controller.state.mode == "HAPTIC" else 0.05
                    )
                    and (action_task is None or action_task.done())
                    and (poll_task is None or poll_task.done())
                ):
                    poll_task = asyncio.create_task(self._poll_position(controller))
                    last_position = now
                await asyncio.sleep(0.005)
            if self.force_requested:
                self._log("AUDIT", "用户强制关闭：未确认板卡停止")
        finally:
            await self._cancel(init_task)
            await self._cancel(action_task)
            await self._cancel(poll_task)
            await controller.session.close()
            if self.force_requested or controller.session.link_error:
                controller.error(controller.session.link_error or "强制关闭，板卡状态未知")
            self.latest = controller.state
            self._flush()
            if self.recorder is not None:
                self.recorder.close()
                if self.recorder.error:
                    self._log("ERROR", self.recorder.error)
            if self.journal is not None:
                self.journal.close()
