"""高速数组绘图与演示视图；显示滤波和降采样均不改写原始数据。"""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Any

import numpy as np
from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QPushButton,
    QWidget,
)

from .binary_wave import CHANNELS, WaveBuffer, export_binary_csv


def smooth_current(x: Any, y: Any, window_ms: float, gaps: Any = None) -> Any:
    """@brief 按真实时标作对称三角加权显示滤波，不跨缺口、不改输入。
    @param x 原始秒时标。@param y 一路电流。@param window_ms 完整窗口毫秒，0为关闭。
    @param gaps 样点前的缺口标记，可省略。@return 同长度显示值，非有限值保持。
    """
    values = np.asarray(y, dtype=np.float64)
    if window_ms <= 0 or len(values) < 3:
        return values
    differences = np.diff(x)
    positive = differences[np.isfinite(differences) & (differences > 0)]
    if not len(positive):
        return values
    interval = float(np.median(positive))
    # 两次等宽平均构成三角权重；总支撑时宽约为所选窗口。
    width = int(window_ms / (2000 * interval) + 0.5) + 1
    if width < 2:
        return values
    breaks = np.zeros(len(values), dtype=bool)
    breaks[1:] = ~np.isfinite(differences) | (differences <= 0) | (differences > interval * 1.5)
    if gaps is not None:
        breaks |= gaps
    valid = np.isfinite(values) & np.isfinite(x)
    starts = np.flatnonzero(valid & np.r_[True, ~valid[:-1] | breaks[1:]])
    stops = np.flatnonzero(valid & np.r_[~valid[1:] | breaks[1:], True]) + 1
    result = values.copy()
    for start, stop in zip(starts, stops, strict=True):
        # 短片段缩小窗口；边缘延拓本段端值，避免引入零值或借用缺口另一侧。
        local_width = min(width, (int(stop) - int(start) + 1) // 2)
        if local_width < 2:
            continue
        segment = np.pad(values[start:stop], local_width - 1, mode="edge")
        for _ in range(2):
            integral = np.r_[0.0, np.cumsum(segment, dtype=np.float64)]
            segment = (integral[local_width:] - integral[:-local_width]) / local_width
        result[start:stop] = segment
    return result


def peak_view(x: Any, y: Any, width: int) -> tuple[Any, Any]:
    """@brief 长窗口仅显示每像素桶的真实极值，保留时序和断点。@param x 时标。@param y 原值。@param width 像素宽。@return 真实点子集。"""
    step = len(y) // max(width, 1)
    if step <= 2:
        return x, y
    groups = len(y) // step
    matrix = y[: groups * step].reshape(groups, step)
    finite = np.isfinite(matrix)
    minimum = np.where(finite, matrix, np.inf).argmin(axis=1)
    maximum = np.where(finite, matrix, -np.inf).argmax(axis=1)
    base = np.arange(groups) * step
    chosen = np.unique(
        np.concatenate(
            (
                base + minimum,
                base + maximum,
                np.flatnonzero(~np.isfinite(y)),
                np.arange(groups * step, len(y)),
                np.array([0, len(y) - 1]),
            )
        )
    )
    return x[chosen], y[chosen]


class ExportJob(QObject):
    """独立后台导出，不阻塞波形接收和停止按钮。"""

    completed = Signal(str)
    failed = Signal(str)

    def __init__(self, path: Path, buffer: WaveBuffer) -> None:
        """@brief 持有导出输入。@param path 输出。@param buffer 接收缓存。"""
        super().__init__()
        self.path, self.buffer = path, buffer

    def start(self) -> None:
        """@brief 启动有界快照导出，调用方持有本对象直到完成。"""
        threading.Thread(target=self._run, daemon=True, name="波形CSV导出").start()

    def _run(self) -> None:
        """@brief 在后台复制有界数据并生成三份文件。"""
        try:
            paths = export_binary_csv(self.path, self.buffer.frozen())
            self.completed.emit("导出完成：" + "、".join(path.name for path in paths))
        except (OSError, ValueError) as error:
            self.failed.emit(str(error))


class BinaryPlot:
    """将高速数组绘制接入已有面板，保留旧文本模式的交互。"""

    def __init__(self, panel: Any, controls: Any) -> None:
        """@brief 添加演示和高级设置。@param panel 现有波形面板。@param controls 底部工具行。"""
        self.panel = panel
        self.buffer: WaveBuffer | None = None
        self.frozen: WaveBuffer | None = None
        self.link_baud = 921600
        self.pole_pairs = 7
        self.saved: dict[str, Any] | None = None
        self.demo = QPushButton("演示视图")
        self.demo.setCheckable(True)
        self.demo.toggled.connect(self.toggle_demo)
        controls.insertWidget(6, self.demo)
        settings = panel.axis_dialog.layout()
        # 显示方式/触发与高速档属于上位机设置，由主窗口放入「上位机设置」页；
        # 坐标设置弹窗只保留坐标轴相关项。
        self.host_settings = QWidget()
        host = QFormLayout(self.host_settings)
        host.setContentsMargins(0, 0, 0, 0)
        # 设置按字段分行，避免窄窗口把触发值和速率裁到横向滚动区外。
        host.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        self.trigger = QComboBox()
        self.trigger.addItem("连续滚动", "scroll")
        self.trigger.addItem("Ia 上升沿触发", "trigger")
        self.trigger.setAccessibleName("波形显示方式")
        self.trigger.currentIndexChanged.connect(self.changed)
        host.addRow("显示方式", self.trigger)
        self.level = QDoubleSpinBox()
        self.level.setRange(-4000, 4000)
        self.level.setDecimals(3)
        self.level.setSuffix(" mA")
        self.level.valueChanged.connect(self.changed)
        host.addRow("触发电平", self.level)
        self.hysteresis = QDoubleSpinBox()
        self.hysteresis.setRange(0.122, 1000)
        self.hysteresis.setDecimals(3)
        self.hysteresis.setValue(1)
        self.hysteresis.setSuffix(" mA")
        self.hysteresis.valueChanged.connect(self.changed)
        host.addRow("迟滞", self.hysteresis)
        self.baud = QComboBox()
        for label, value in (
            ("日常监视 · 关闭高速波形", 0),
            ("921600 · 5 kHz", 921_600),
            ("1 Mbaud · 5 kHz", 1_000_000),
            ("2 Mbaud · 10 kHz", 2_000_000),
            ("3 Mbaud · 10 kHz", 3_000_000),
        ):
            self.baud.addItem(label, value)
        self.baud.setAccessibleName("下次启动的采集方式")
        self.baud.setToolTip(
            "默认只读状态和真实Id/Iq数值；选高速档后下次启动才上传波形，不改变板卡控制周期"
        )
        host.addRow("下次启动的采集", self.baud)
        lock_row = QHBoxLayout()
        lock = QPushButton("锁定当前 Y")
        lock.clicked.connect(self.lock_y)
        lock_row.addWidget(lock)
        lock_row.addStretch()
        settings.addLayout(lock_row)
        panel.x_mode.setItemText(0, "自适应时宽")

    def changed(self, *_args: object) -> None:
        """@brief 仅刷新显示，不发控制命令。"""
        self.panel.dirty = True
        self.panel.redraw()

    def receive(self, buffer: WaveBuffer) -> None:
        """@brief 接收共享数组引用，无逐点Qt事件。@param buffer 后台缓存。"""
        self.buffer = buffer
        if self.panel.paused and self.frozen is None and buffer.count:
            self.frozen = buffer.frozen()
        self.panel.dirty = True

    def pause_changed(self) -> None:
        """@brief 暂停只冻结显示副本，接收持续运行。"""
        self.frozen = (
            self.buffer.frozen() if self.panel.paused and self.buffer is not None else None
        )

    def lock_y(self) -> None:
        """@brief 保持当前两个Y量程，避免演示时数值跳动。"""
        self.panel.y_auto.update(speed=False, current=False)
        self.panel._sync_y_controls()

    def toggle_demo(self, enabled: bool) -> None:
        """@brief 应用或完整恢复视图配置。@param enabled 演示开关。"""
        p = self.panel
        if enabled:
            self.saved = {
                "view": p.view_combo.currentIndex(),
                "x_mode": p.x_mode.currentIndex(),
                "span": p.x_span.value(),
                "y_auto": dict(p.y_auto),
                "current": p.current_plot.viewRange()[1],
                "speed": p.speed_plot.viewRange()[1],
                "split": p.splitter.sizes(),
                "follow": p.follow,
                "channels": {key: check.isChecked() for key, check in p.channel_checks.items()},
            }
            p.view_combo.setCurrentIndex(1)
            p.x_mode.setCurrentIndex(0)
            p.follow_button.setChecked(True)
            p.y_auto.update(speed=True, current=True)
            p.splitter.setSizes([150, 420])
            self.demo.setText("退出演示")
        elif self.saved is not None:
            saved = self.saved
            p.view_combo.setCurrentIndex(saved["view"])
            p.x_mode.setCurrentIndex(saved["x_mode"])
            p.x_span.setValue(saved["span"])
            p.y_auto = saved["y_auto"]
            p.current_plot.setYRange(*saved["current"], padding=0)
            p.speed_plot.setYRange(*saved["speed"], padding=0)
            p.splitter.setSizes(saved["split"])
            p.follow_button.setChecked(saved["follow"])
            for key, checked in saved["channels"].items():
                if key in p.channel_checks:
                    p.channel_checks[key].setChecked(checked)
            self.saved = None
            self.demo.setText("演示视图")
        self.changed()

    def redraw(self) -> bool:
        """@brief 绘制可见样点的选定滤波结果，长窗口保留显示极值。@return 是否使用二进制缓存。"""
        p = self.panel
        data = self.frozen if p.paused and self.frozen is not None else self.buffer
        if data is None or not data.count:
            return False
        # 已按可视时宽切片，无需让pyqtgraph在X变化时重算旧数据一次。
        for curve in p.lines.values():
            if curve.opts["clipToView"]:
                curve.setClipToView(False)
            if curve.opts["autoDownsample"]:
                curve.setDownsampling(ds=1, auto=False)
        slow = data.statuses()
        latest = slow[-1] if slow else None
        last = data.last_seconds
        kind = "speed" if p.compact and p.compact_combo.currentIndex() == 0 else p.view
        if kind == "phase":
            rpm = latest.rpm if latest is not None else None
            span = (
                min(5.0, max(0.020, 300 / (abs(rpm) * self.pole_pairs)))
                if rpm is not None and abs(rpm) > 0
                else 0.250
            )
        else:
            span = {"dq": 0.5, "bus": 5.0, "speed": 2.0}[kind]
        mode = p.x_mode.currentData()
        if mode == "manual":
            span = p.x_span.value() / 1000
        elif mode == "all":
            span = max(0.001, last - data.first_seconds)
        low, high = last - span, last
        trigger_wait = False
        if p.follow and self.trigger.currentData() == "trigger":
            tx, tv, gaps, _ = data.current(last - span * 2, last)
            level, hysteresis = self.level.value() / 1000, self.hysteresis.value() / 1000
            a = tv[:, 0].astype(np.float64) / 8192
            rises = (
                np.flatnonzero((a[:-1] < level + hysteresis) & (a[1:] >= level + hysteresis)) + 1
            )
            eligible: list[int] = []
            previous = 0
            for index in rises:
                if (
                    tx[index] + span * 0.8 <= last
                    and tx[index] - span * 0.2 >= data.first_seconds
                    and np.any(a[previous:index] <= level - hysteresis)
                ):
                    eligible.append(int(index))
                previous = int(index)
            if latest is None or latest.rpm is None or not eligible:
                trigger_wait = True
            else:
                anchor = float(tx[eligible[-1]])
                low, high = anchor - span * 0.2, anchor + span * 0.8
                selected = (tx >= low) & (tx <= high)
                if np.any(gaps[selected]):
                    trigger_wait = True
        if p.follow:
            p.speed_plot.setXRange(low, high, padding=0)
        low, high = p.speed_plot.viewRange()[0]
        window_ms = p.filter_window_ms()
        # 先取窗口两侧的真实上下文再滤波，缩放/平移不改变同一内部样点的显示值。
        context = window_ms / 1000
        x, raw, gaps, _ = data.current(low - context, high + context)
        values = raw.astype(np.float64) / 8192
        channels = {name: values[:, index] for index, name in enumerate(CHANNELS)}
        channels["ic"] = -channels["ia"] - channels["ib"]
        visible = (x >= low) & (x <= high)
        visible_x = x[visible]
        slow_visible = [point for point in slow if low <= point.seconds <= high]
        sx = np.asarray([point.seconds for point in slow_visible])
        slow_channels = {
            "rpm": np.asarray(
                [np.nan if point.rpm is None else point.rpm for point in slow_visible]
            ),
            "target": np.asarray([point.target for point in slow_visible]),
            "bus": np.asarray([point.bus for point in slow_visible]),
        }
        ranges: dict[str, list[float]] = {"speed": [], "current": []}
        gap_indices = np.flatnonzero(gaps[visible])
        line_x = np.insert(visible_x, gap_indices, np.nan)
        for key, curve in p.lines.items():
            if not p.channel_checks[key].isChecked():
                continue
            if key in slow_channels:
                yy = slow_channels[key]
                xx = sx
            else:
                yy = smooth_current(x, channels[key], window_ms, gaps)[visible]
                xx = visible_x
            finite = yy[np.isfinite(yy)]
            if finite.size:
                ranges["speed" if key in ("rpm", "target") else "current"].extend(
                    (float(finite.min()), float(finite.max()))
                )
            if key in channels and gap_indices.size:
                xx, yy = line_x, np.insert(yy, gap_indices, np.nan)
            xx, yy = peak_view(xx, yy, p.current_plot.width())
            curve.setData(xx, yy, connect="finite")
        for group, plot in (("speed", p.speed_plot), ("current", p.current_plot)):
            limits = ranges[group]
            if p.y_auto[group] and limits:
                bottom, top = min(limits), max(limits)
                if top == bottom:
                    margin = max(abs(top) * 0.05, 1 if group == "speed" else 0.001)
                    bottom, top = bottom - margin, top + margin
                plot.setYRange(bottom, top, padding=0.08)
        integrity = (
            f"缺 {data.missing} 点 / 坏包 {data.corrupt}"
            if data.missing or data.corrupt
            else "接收完整"
        )
        if data.clipped:
            integrity += f" · 编码截幅 {data.clipped} 块"
        observed_span = data.last_seconds - data.first_seconds
        effective = (data.count - 1) / observed_span if observed_span > 0 else 0.0
        source = "通信测试数据 · " if data.test else ""
        state = (
            "显示已暂停，仍在接收"
            if p.paused
            else (
                "历史样本 · 上传已结束"
                if data.ended
                else (
                    "触发锁定"
                    if self.trigger.currentData() == "trigger"
                    else "跟随最新" if p.follow else "历史视图"
                )
            )
        )
        if trigger_wait:
            # 等待时清空曲线，不能把旧周期伪装成当前触发成功。
            for curve in p.lines.values():
                curve.setData([], [])
            state = "等待触发：实际转速或有效周期不足"
        p.notice.setText(
            f"{source}{state} · 有效 {effective / 1000:.2f} k点/s · {self.link_baud / 1_000_000:g} Mbaud · {integrity} · {p.filter_description()}"
        )
        p.notice.setToolTip(
            f"源采样率 {170_000_000 / data.step:g} 点/s；可见 {len(visible_x):,} / 缓存 {data.count:,} 点。"
            "滤波仅影响电流曲线，记录与导出均为原始数据。"
        )
        p.count.setText(f"可见 {len(visible_x):,} / 缓存 {data.count:,} 点")
        return True
