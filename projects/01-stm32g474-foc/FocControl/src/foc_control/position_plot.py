"""多圈机械角与误差图，原始点导出和缺口分段。"""

from __future__ import annotations

import csv
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import (
    QCheckBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from .position_feedback import PositionPoint, PositionSample


class PositionPanel(QWidget):
    """机械目标、实际角度和误差共享源时间，支持缩放、暂停和原始导出。"""

    def __init__(self) -> None:
        """构建独立位置页，不复用RPM通道或电流显示滤波。"""
        super().__init__()
        self.points: deque[PositionPoint] = deque(maxlen=6000)
        layout = QVBoxLayout(self)
        row = QHBoxLayout()
        self.status = QLabel("连接位置模式后读取机械坐标")
        self.status.setWordWrap(True)
        row.addWidget(self.status, 1)
        self.follow = QCheckBox("自动跟随")
        self.follow.setChecked(True)
        row.addWidget(self.follow)
        self.pause = QPushButton("暂停绘图")
        self.pause.setCheckable(True)
        self.pause.toggled.connect(self._paused)
        row.addWidget(self.pause)
        self.export = QPushButton("导出原始位置 CSV")
        self.export.setEnabled(False)
        self.export.clicked.connect(self.export_csv)
        row.addWidget(self.export)
        layout.addLayout(row)
        self.angle = pg.PlotWidget(background="w")
        self.error = pg.PlotWidget(background="w")
        self.error.setXLink(self.angle)
        for plot in (self.angle, self.error):
            plot.showGrid(x=True, y=True, alpha=0.15)
            plot.setLabel("bottom", "运行源时间", units="s")
            plot.getAxis("left").setTextPen("#475569")
            plot.getAxis("bottom").setTextPen("#475569")
        self.angle.setLabel("left", "多圈机械角", units="°")
        self.error.setLabel("left", "目标减实际", units="°")
        self.angle.addLegend()
        self.target_curve = self.angle.plot(pen=pg.mkPen("#D97706", width=1.5), name="目标角")
        self.actual_curve = self.angle.plot(pen=pg.mkPen("#2563EB", width=1.5), name="实际角")
        self.error_curve = self.error.plot(pen=pg.mkPen("#DC2626", width=1.3))
        layout.addWidget(self.angle, 2)
        layout.addWidget(self.error, 1)

    def show_sample(self, sample: PositionSample | None) -> None:
        """显示真实快照；坐标无效时不用旧角度代替。"""
        if sample is None or not sample.valid:
            self.status.setText("机械坐标尚未建立 · 停机后点击建立机械零点")
            return
        stage = (
            "到位保持"
            if sample.running and sample.arrived
            else "定位中" if sample.running else "输出关闭"
        )
        self.status.setText(
            f"实际 {sample.angle_deg:.4f}°  ·  目标 {sample.target_deg:.3f}°  ·  "
            f"误差 {sample.target_deg - sample.angle_deg:+.4f}°  ·  {stage}"
        )

    def receive(self, payload: object) -> None:
        """接收后台批次；暂停只停止绘制，接收及记录继续。"""
        if not isinstance(payload, list):
            return
        for item in payload:
            if not isinstance(item, PositionPoint):
                continue
            if self.points and item.seconds <= self.points[-1].seconds:
                self.points.clear()
            self.points.append(item)
        self.export.setEnabled(bool(self.points))
        if not self.pause.isChecked():
            self.redraw()

    def _paused(self, paused: bool) -> None:
        """恢复绘图时显示当前完整缓存。"""
        self.pause.setText("继续绘图" if paused else "暂停绘图")
        if not paused:
            self.redraw()

    def redraw(self) -> None:
        """缺口处插入NaN断线，原始样点值保持不变。"""
        x: list[float] = []
        target: list[float] = []
        actual: list[float] = []
        error: list[float] = []
        for point in self.points:
            if x and not point.continuous:
                x.append(float("nan"))
                target.append(float("nan"))
                actual.append(float("nan"))
                error.append(float("nan"))
            x.append(point.seconds)
            target.append(point.sample.target_deg)
            actual.append(point.sample.angle_deg)
            error.append(point.sample.target_deg - point.sample.angle_deg)
        times = np.asarray(x)
        self.target_curve.setData(times, np.asarray(target), connect="finite")
        self.actual_curve.setData(times, np.asarray(actual), connect="finite")
        self.error_curve.setData(times, np.asarray(error), connect="finite")
        if self.points and self.follow.isChecked():
            end = self.points[-1].seconds
            self.angle.setXRange(max(0.0, end - 10), max(10.0, end), padding=0)

    def export_csv(self) -> None:
        """导出原始原码、源时标及实际误差，不导出绘图断线占位。"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "导出原始位置", "position.csv", "CSV (*.csv)"
        )
        if not filename:
            return
        with open(filename, "w", encoding="utf-8-sig", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(
                (
                    "source_time_s",
                    "source_ticks",
                    "zero_id",
                    "raw",
                    "counts",
                    "target_deg",
                    "actual_deg",
                    "error_deg",
                    "ref_rpm",
                    "arrived",
                    "tolerance_deg",
                    "continuous",
                )
            )
            for point in self.points:
                s = point.sample
                writer.writerow(
                    (
                        point.seconds,
                        s.ticks,
                        s.zero_id,
                        s.raw,
                        s.counts,
                        s.target_deg,
                        s.angle_deg,
                        s.target_deg - s.angle_deg,
                        s.reference_rpm,
                        s.arrived,
                        s.tolerance_deg,
                        point.continuous,
                    )
                )
