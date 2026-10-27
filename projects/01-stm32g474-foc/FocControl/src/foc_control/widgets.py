"""参数输入和实时波形，采样与绘制相互独立。"""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QEvent, QRectF, Qt, Signal
from PySide6.QtGui import (
    QFont,
    QHelpEvent,
    QPainter,
    QPaintEvent,
    QResizeEvent,
    QTextOption,
)
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QToolTip,
    QVBoxLayout,
    QWidget,
)

from .models import ControlError, ParameterSpec, ParameterValue
from .protocol import PlotPoint
from .theme import COLORS
from .wave_plot import BinaryPlot, smooth_current


def refresh_style(widget: QWidget) -> None:
    """@brief 应用状态属性而不改变控件布局。@param widget 控件。"""
    widget.style().unpolish(widget)
    widget.style().polish(widget)


class ElidedLabel(QLabel):
    """单行提示：过长时以省略号截断显示，text() 仍返回完整文字，悬停显示全文。"""

    def __init__(self, text: str = "") -> None:
        """@brief 创建不换行、可被压缩的提示标签。@param text 初始文字。"""
        super().__init__(text)
        self.setWordWrap(False)
        self.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        self.setMinimumWidth(40)

    def paintEvent(self, _event: QPaintEvent) -> None:
        """@brief 只在绘制时省略，不改写文字内容。@param _event 绘制事件。"""
        painter = QPainter(self)
        rect = self.contentsRect()
        shown = self.fontMetrics().elidedText(
            self.text(), Qt.TextElideMode.ElideRight, rect.width()
        )
        painter.setPen(self.palette().color(self.foregroundRole()))
        painter.drawText(QRectF(rect), shown, QTextOption(self.alignment()))
        painter.end()

    def event(self, event: QEvent) -> bool:
        """@brief 悬停时显示完整文字及附加说明。@param event 事件。@return 是否已处理。"""
        if event.type() == QEvent.Type.ToolTip and isinstance(event, QHelpEvent):
            extra = self.toolTip()
            QToolTip.showText(
                event.globalPos(), self.text() + ("\n" + extra if extra else ""), self
            )
            return True
        return super().event(event)


class ParameterEditor(QFrame):
    """保留物理量草稿及当前真实参数，输入不直接写板卡。"""

    changed = Signal()

    def __init__(self, definition: ParameterSpec, value: ParameterValue) -> None:
        """@brief 构建输入。@param definition 能力。@param value 回读值。"""
        super().__init__()
        self.definition, self.current = definition, value
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.label = QLabel(definition.label)
        self.label.setObjectName("muted")
        layout.addWidget(self.label)
        row = QHBoxLayout()
        self.edit = QLineEdit(definition.format(value))
        self.edit.setAccessibleName(definition.label)
        self.label.setBuddy(self.edit)
        self.edit.setReadOnly(not definition.editable)
        self.edit.setMinimumWidth(100)
        row.addWidget(self.edit, 1)
        self.unit = QLabel(definition.suffix)
        self.unit.setObjectName("muted")
        row.addWidget(self.unit)
        layout.addLayout(row)
        self.message = QLabel()
        self.message.setObjectName("error")
        self.message.setWordWrap(True)
        self.message.hide()
        layout.addWidget(self.message)
        low = definition.format(
            ParameterValue(definition.minimum, definition.decimals, definition.unit)
        )
        high = definition.format(
            ParameterValue(definition.maximum, definition.decimals, definition.unit)
        )
        self.edit.setToolTip(f"允许范围：{low}～{high} {definition.suffix}\n原始值：{value.value}")
        self.edit.textChanged.connect(self._changed)
        self.edit.editingFinished.connect(self.show_error)

    def candidate(self) -> ParameterValue:
        """@brief 编码草稿，固定字段返回原值。@return 候选值。"""
        return (
            self.definition.encode(self.edit.text()) if self.definition.editable else self.current
        )

    @property
    def dirty(self) -> bool:
        """@brief 当前输入是否未应用。@return 草稿状态。"""
        try:
            return self.candidate() != self.current
        except ControlError:
            return True

    def _changed(self, _text: str) -> None:
        """@brief 只更新草稿标记。@param _text 输入文本。"""
        self.edit.setProperty("draft", self.dirty)
        refresh_style(self.edit)
        self.changed.emit()

    def show_error(self) -> None:
        """@brief 失焦或提交时显示字段旁错误。"""
        try:
            self.candidate()
            message = ""
        except ControlError as error:
            message = str(error)
        self.message.setText(message)
        self.message.setVisible(bool(message))
        self.edit.setProperty("invalid", bool(message))
        refresh_style(self.edit)

    def set_current(self, value: ParameterValue) -> None:
        """@brief 更新回读而不覆盖尚未提交的草稿。@param value 新值。"""
        was_dirty = self.dirty
        self.current = value
        if not was_dirty:
            self.edit.blockSignals(True)
            self.edit.setText(self.definition.format(value))
            self.edit.blockSignals(False)
        self.edit.setProperty("draft", self.dirty)
        refresh_style(self.edit)

    def discard(self) -> None:
        """@brief 撤销输入，恢复板卡最近回读。"""
        self.edit.setText(self.definition.format(self.current))
        self.show_error()


class Metric(QFrame):
    """固定大小的数字卡片，更新数值不使布局跳动。"""

    def __init__(self, title: str, unit: str) -> None:
        """@brief 构建指标。@param title 标题。@param unit 单位。"""
        super().__init__()
        self.setObjectName("metric")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        label = QLabel(title)
        label.setObjectName("muted")
        layout.addWidget(label)
        row = QHBoxLayout()
        self.value = QLabel("—")
        self.value.setObjectName("value")
        row.addWidget(self.value)
        row.addStretch()
        self.unit = QLabel(unit)
        self.unit.setObjectName("muted")
        row.addWidget(self.unit)
        layout.addLayout(row)

    def display(self, value: str) -> None:
        """@brief 写入新数字。@param value 已格式化值。"""
        if self.value.text() != value:
            self.value.setText(value)


class PlotPanel(QWidget):
    """两块共享时间轴曲线；显示暂停不影响原始接收和导出。"""

    def __init__(self) -> None:
        """@brief 创建波形工具条、通道开关和有界数据缓存。"""
        super().__init__()
        self.points: deque[PlotPoint] = deque(maxlen=20000)
        self.paused = False
        self.follow = True
        self.dirty = False
        self.compact = False
        self.view = "dq"
        self.control_mode = "SPEED"
        # 暂停保留绘制快照，轴调整仍可工作；接收缓存继续更新。
        self.display_points: list[PlotPoint] = []
        self.y_auto = {"speed": True, "current": True}
        self.lines: dict[str, Any] = {}
        self.channel_checks: dict[str, QCheckBox] = {}
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)
        toolbar = QHBoxLayout()
        self.pause = QPushButton("暂停显示")
        self.pause.clicked.connect(self.toggle_pause)
        toolbar.addWidget(self.pause)
        self.follow_button = QPushButton("自动跟随：开")
        self.follow_button.setCheckable(True)
        self.follow_button.setChecked(True)
        self.follow_button.setToolTip(
            "开启后始终滚动到最新数据；关闭后可拖动查看历史，缩放不关闭跟随"
        )
        self.follow_button.toggled.connect(self._follow_changed)
        toolbar.addWidget(self.follow_button)
        self.view_combo = QComboBox()
        self.view_combo.addItem("dq 电流", "dq")
        self.view_combo.addItem("三相电流", "phase")
        self.view_combo.addItem("母线电压", "bus")
        self.view_combo.currentIndexChanged.connect(self._view_changed)
        toolbar.addWidget(self.view_combo)
        self.compact_combo = QComboBox()
        self.compact_combo.addItems(("转速图", "电流 / 电压图"))
        self.compact_combo.currentIndexChanged.connect(self._compact_changed)
        self.compact_combo.hide()
        toolbar.addWidget(self.compact_combo)
        # 滤波只存在于绘制路径，不发送板卡命令，也不写回接收缓存。
        # 下拉框由主窗口放入「上位机设置」页，属性仍挂在本面板上。
        self.filter_combo = QComboBox()
        for label, window in (
            ("滤波：关闭（原始）", 0.0),
            ("滤波：轻度 1 ms", 1.0),
            ("滤波：标准 2 ms", 2.0),
            ("滤波：较强 4 ms", 4.0),
        ):
            self.filter_combo.addItem(label, window)
        self.filter_combo.setCurrentIndex(2)
        self.filter_combo.setAccessibleName("电流显示滤波强度")
        self.filter_combo.setToolTip(
            "按真实时标作对称三角加权平滑；较强档也会减小快速变化的幅值。"
            "仅影响电流曲线，原始记录/CSV、转速、电压和触发判据保持原值。"
        )
        self.filter_combo.currentIndexChanged.connect(self._filter_changed)
        # 状态说明与工具按钮同处一行，单行省略显示，悬停查看全文。
        toolbar.addSpacing(8)
        self.notice = ElidedLabel("等待 FOC 运行数据 · 横轴为板卡时间 · 暂停只影响显示")
        self.notice.setObjectName("hint")
        toolbar.addWidget(self.notice, 1)
        self.export = QPushButton("导出 CSV")
        toolbar.addWidget(self.export)
        layout.addLayout(toolbar)
        self.splitter = QSplitter(Qt.Orientation.Vertical)
        self.speed_plot = pg.PlotWidget(background="white")
        self.current_plot = pg.PlotWidget(background="white")
        self._setup_plot(self.speed_plot, "机械转速", "RPM")
        self._setup_plot(self.current_plot, "dq 电流", "A")
        self.splitter.addWidget(self.speed_plot)
        self.splitter.addWidget(self.current_plot)
        self.current_plot.setXLink(self.speed_plot)
        layout.addWidget(self.splitter, 1)
        self.channels = QGridLayout()
        self.channels.setHorizontalSpacing(12)
        self.channels.setVerticalSpacing(4)
        layout.addLayout(self.channels)
        # 详细范围在独立窗口调整，避免占用波形的可视高度。
        self.axis_dialog = QDialog(self, Qt.WindowType.Tool)
        self.axis_dialog.setWindowTitle("坐标轴设置")
        self.axis_dialog.setLayout(self._axis_controls())
        controls = QHBoxLayout()
        for label, axis, factor in (
            ("X＋", "X", 0.5),
            ("X－", "X", 2.0),
            ("Y＋", "Y", 0.5),
            ("Y－", "Y", 2.0),
        ):
            button = QPushButton(label)
            button.setToolTip(axis + ("轴放大" if factor < 1 else "轴缩小"))
            action = self.zoom if axis == "X" else self.zoom_y
            button.clicked.connect(lambda _checked=False, scale=factor, fn=action: fn(scale))
            controls.addWidget(button)
        settings = QPushButton("坐标设置")
        settings.setToolTip("设置X/Y自适应、X时宽及独立Y轴上下限")
        settings.clicked.connect(self.show_axis_settings)
        controls.addWidget(settings)
        reset = QPushButton("复位视图")
        reset.clicked.connect(self.reset_view)
        controls.addWidget(reset)
        controls.addStretch()
        self.count = QLabel("0 个有效点")
        self.count.setObjectName("hint")
        controls.addWidget(self.count)
        layout.addLayout(controls)
        self._build_lines()
        self.binary = BinaryPlot(self, controls)

    def filter_window_ms(self) -> float:
        """@brief 获取本次显示滤波窗口。@return 毫秒；0表示原始样点。"""
        return float(self.filter_combo.currentData())

    def filter_description(self) -> str:
        """@brief 在图表状态栏明确区分滤波与原始显示。@return 状态文字。"""
        window = self.filter_window_ms()
        if window and self.view != "bus":
            return f"电流显示滤波 {window:g} ms"
        return "原始样点"

    def _filter_changed(self, _index: int) -> None:
        """@brief 切换强度立即重绘，暂停时仍可对照。@param _index 所选档位。"""
        self.dirty = True
        self.redraw()

    def show_axis_settings(self) -> None:
        """@brief 显示非模态坐标设置，保留图表和停止按钮操作。"""
        self._sync_y_controls()
        self.axis_dialog.show()
        self.axis_dialog.raise_()

    def _axis_controls(self) -> QVBoxLayout:
        """@brief 创建适用于全部通道的独立X/Y轴控制。@return 两行紧凑控制布局。"""
        axes = QVBoxLayout()
        x_row = QHBoxLayout()
        x_row.addWidget(QLabel("X 轴"))
        self.x_mode = QComboBox()
        self.x_mode.addItem("自适应（最近500点）", "auto")
        self.x_mode.addItem("全部记录", "all")
        self.x_mode.addItem("手动时宽", "manual")
        self.x_mode.setAccessibleName("X轴范围模式")
        self.x_mode.setToolTip("自适应按最近实际样点确定时间范围，所有通道共用真实时间轴")
        self.x_mode.currentIndexChanged.connect(self._x_changed)
        x_row.addWidget(self.x_mode)
        self.x_span = QDoubleSpinBox()
        self.x_span.setRange(0.001, 1e12)
        self.x_span.setDecimals(3)
        self.x_span.setValue(100)
        self.x_span.setSuffix(" ms")
        self.x_span.setMaximumWidth(130)
        self.x_span.setAccessibleName("X轴手动时间宽度，毫秒")
        self.x_span.setEnabled(False)
        self.x_span.valueChanged.connect(self._x_changed)
        x_row.addWidget(self.x_span)
        for label, factor in (("X 放大", 0.5), ("X 缩小", 2.0)):
            button = QPushButton(label)
            button.clicked.connect(lambda _checked=False, scale=factor: self.zoom(scale))
            x_row.addWidget(button)
        x_row.addStretch()
        axes.addLayout(x_row)
        y_row = QHBoxLayout()
        self.y_target = QComboBox()
        self.y_target.addItem("Y 电流/电压", "current")
        self.y_target.addItem("Y 转速", "speed")
        self.y_target.setAccessibleName("Y轴调整对象")
        self.y_target.currentIndexChanged.connect(self._sync_y_controls)
        y_row.addWidget(self.y_target)
        self.y_auto_check = QCheckBox("自适应")
        self.y_auto_check.setChecked(True)
        self.y_auto_check.toggled.connect(self._y_auto_changed)
        y_row.addWidget(self.y_auto_check)
        for label, factor in (("Y 放大", 0.5), ("Y 缩小", 2.0)):
            button = QPushButton(label)
            button.clicked.connect(lambda _checked=False, scale=factor: self.zoom_y(scale))
            y_row.addWidget(button)
        self.y_low = QDoubleSpinBox()
        self.y_high = QDoubleSpinBox()
        for spin, name, value in ((self.y_low, "Y轴下限", -0.1), (self.y_high, "Y轴上限", 0.1)):
            spin.setRange(-1e12, 1e12)
            spin.setDecimals(6)
            spin.setMaximumWidth(105)
            spin.setAccessibleName(name)
            spin.setToolTip(name + "；使用左侧当前图的单位，点击应用Y生效")
            spin.setValue(value)
            y_row.addWidget(spin)
        apply_y = QPushButton("应用 Y")
        apply_y.clicked.connect(self.apply_y_range)
        y_row.addWidget(apply_y)
        y_row.addStretch()
        axes.addLayout(y_row)
        return axes

    def _setup_plot(self, plot: Any, label: str, unit: str) -> None:
        """@brief 统一网格和字体。@param plot 图。@param label 标签。@param unit 单位。"""
        plot.showGrid(x=True, y=True, alpha=0.12)
        plot.setLabel("left", label, units=unit, color=COLORS["muted"])
        plot.setLabel("bottom", "时间", units="s", color=COLORS["muted"])
        font = QFont("Consolas")
        font.setPixelSize(13)
        for name in ("left", "bottom"):
            axis = plot.getAxis(name)
            axis.setTickFont(font)
            axis.setTextPen(COLORS["muted"])
            axis.setPen(COLORS["input_border"])
        plot.getAxis("left").enableAutoSIPrefix(False)
        plot.getAxis("left").setWidth(72)
        plot.getViewBox().sigRangeChangedManually.connect(
            lambda axes, target=plot: self._manual_view(target, axes)
        )
        plot.enableAutoRange(axis="y", enable=False)
        plot.setMenuEnabled(False)

    def _manual_view(self, plot: Any, axes: Any) -> None:
        """@brief 鼠标缩放更新时宽而不退出跟随。@param plot 当前图。@param axes X/Y变化标记。"""
        if axes[0] and self.follow:
            low, high = plot.getViewBox().viewRange()[0]
            self.x_mode.blockSignals(True)
            self.x_mode.setCurrentIndex(2)
            self.x_mode.blockSignals(False)
            self.x_span.blockSignals(True)
            self.x_span.setValue(max(0.001, (high - low) * 1000))
            self.x_span.blockSignals(False)
            self.x_span.setEnabled(True)
        if axes[1]:
            self.y_auto["speed" if plot is self.speed_plot else "current"] = False
            self._sync_y_controls()
        self.dirty = True

    def _x_changed(self, _value: object = None) -> None:
        """@brief X时基独立更新，不改变Y量程。@param _value 控件新值。"""
        self.x_span.setEnabled(self.x_mode.currentData() == "manual")
        self.resume_follow()
        self.dirty = True
        self.redraw()

    def _selected_y_plot(self) -> Any:
        """@brief 获取当前Y轴操作对象。@return 电流/电压图或转速图。"""
        return self.speed_plot if self.y_target.currentData() == "speed" else self.current_plot

    def _sync_y_controls(self, _index: int = 0) -> None:
        """@brief 切换对象时同步其独立量程，不写回另一张图。@param _index 控件索引。"""
        key = str(self.y_target.currentData())
        self.y_auto_check.blockSignals(True)
        self.y_auto_check.setChecked(self.y_auto[key])
        self.y_auto_check.blockSignals(False)
        low, high = self._selected_y_plot().getViewBox().viewRange()[1]
        self.y_low.setValue(low)
        self.y_high.setValue(high)

    def _y_auto_changed(self, enabled: bool) -> None:
        """@brief 单独启停当前图的Y轴自适应。@param enabled 是否自适应。"""
        self.y_auto[str(self.y_target.currentData())] = enabled
        self.dirty = True
        self.redraw()

    def apply_y_range(self) -> None:
        """@brief 应用明确的Y上下限，错误输入只提示，不改图或板卡。"""
        low, high = self.y_low.value(), self.y_high.value()
        if low >= high:
            self.notice.setText("Y轴下限必须小于上限")
            return
        self.y_auto_check.setChecked(False)
        self._selected_y_plot().setYRange(low, high, padding=0)

    def zoom_y(self, factor: float) -> None:
        """@brief 只缩放所选图的Y轴，X继续跟随。@param factor 缩放倍率。"""
        self.y_auto_check.setChecked(False)
        self._selected_y_plot().getViewBox().scaleBy((1, factor))
        self._sync_y_controls()

    def resume_follow(self) -> None:
        """@brief 恢复X跟随，保留用户选择的时宽及Y量程。"""
        self.follow_button.setChecked(True)
        self.follow = True
        self.dirty = True
        self.redraw()

    def _follow_changed(self, enabled: bool) -> None:
        """@brief 仅通过明确的跟随开关进入历史浏览。@param enabled 是否滚动到最新样点。"""
        self.follow = enabled
        self.follow_button.setText("自动跟随：开" if enabled else "自动跟随：关")
        self.dirty = True
        self.redraw()

    def _build_lines(self) -> None:
        """@brief 切换分组时重建图例，颜色与线型共同表达通道。"""
        self.speed_plot.clear()
        self.current_plot.clear()
        self.lines.clear()
        while self.channels.count():
            item = self.channels.takeAt(0)
            widget = item.widget() if item is not None else None
            if widget is not None:
                widget.hide()
                widget.deleteLater()
        self.channel_checks.clear()
        definitions = [("rpm", "机械转速", COLORS["primary"], False, self.speed_plot)]
        if self.control_mode not in ("CURRENT", "HAPTIC"):
            definitions.insert(
                0, ("target", "目标转速", COLORS["reference"], True, self.speed_plot)
            )
        if self.view == "dq":
            self.current_plot.setLabel("left", "dq 电流", units="A")
            definitions.extend(
                [
                    ("id", "Id 实际", COLORS["warning"], False, self.current_plot),
                    ("iq", "Iq 实际", COLORS["success"], False, self.current_plot),
                    ("rd", "Id 参考", COLORS["warning"], True, self.current_plot),
                    ("rq", "Iq 参考", COLORS["success"], True, self.current_plot),
                ]
            )
        elif self.view == "phase":
            self.current_plot.setLabel("left", "相电流", units="A")
            definitions.extend(
                [
                    ("ia", "Ia", "#2563EB", False, self.current_plot),
                    ("ib", "Ib", "#B45309", False, self.current_plot),
                    ("ic", "Ic 重建", "#15803D", False, self.current_plot),
                    ("isc", "ISC 独立采样", "#7E22CE", True, self.current_plot),
                ]
            )
        else:
            self.current_plot.setLabel("left", "母线电压", units="V")
            definitions.append(("bus", "母线", "#7E22CE", False, self.current_plot))
        for index, (key, label, color, dashed, plot) in enumerate(definitions):
            pen = pg.mkPen(
                color, width=1.7, style=Qt.PenStyle.DashLine if dashed else Qt.PenStyle.SolidLine
            )
            curve = plot.plot([], [], pen=pen, name=label, connect="finite")
            curve.setDownsampling(auto=True, method="peak")
            curve.setClipToView(True)
            self.lines[key] = curve
            unit = "RPM" if key in ("target", "rpm") else "V" if key == "bus" else "A"
            checkbox = QCheckBox(f"{label} ({unit})")
            checkbox.setChecked(key != "isc")
            curve.setVisible(checkbox.isChecked())
            checkbox.toggled.connect(lambda visible, name=key: self._channel_changed(name, visible))
            self.channel_checks[key] = checkbox
            checkbox.setStyleSheet(f"color: {color};")
            self.channels.addWidget(
                checkbox, 0 if index < 2 else 1, index if index < 2 else index - 2
            )
        self.channels.setColumnStretch(4, 1)
        self._compact_changed(0)
        self.dirty = True

    def set_control_mode(self, mode: str) -> None:
        """@brief 电流模式只显示实测机械转速，Iq目标使用dq参考。@param mode 板上模式。"""
        if self.control_mode != mode:
            self.control_mode = mode
            if mode in ("CURRENT", "HAPTIC"):
                self.view_combo.setCurrentIndex(self.view_combo.findData("dq"))
            self.compact_combo.setCurrentIndex(1 if mode in ("CURRENT", "HAPTIC") else 0)
            self._build_lines()
            self.redraw()

    def _channel_changed(self, key: str, visible: bool) -> None:
        """@brief 通道开关参与Y自适应，暂停时也能比较。@param key 通道。@param visible 可见性。"""
        self.lines[key].setVisible(visible)
        self.dirty = True
        self.redraw()

    def _view_changed(self, _index: int) -> None:
        """@brief 更换下图通道，不改变板卡打印。@param _index 选中项。"""
        self.view = str(self.view_combo.currentData())
        self.y_auto["current"] = True
        self._build_lines()
        self._sync_y_controls()
        self.redraw()

    def add_points(self, points: list[PlotPoint]) -> None:
        """@brief 接收批量数据，即使暂停仍保存。@param points 新记录。"""
        for point in points:
            if self.points and point.seconds < self.points[-1].seconds:
                self.points.clear()
            self.points.append(point)
        if not self.paused:
            self.dirty = True

    def _clock_text(self) -> str:
        """@brief 根据最近有效数据标注实际时间来源。@return 时间说明。"""
        return (
            "板卡时间"
            if not self.points or self.points[-1].frame.ticks is not None
            else "主机接收时间（原格式没有 tick）"
        )

    def toggle_pause(self) -> None:
        """@brief 暂停或恢复绘制，接收保持运行。"""
        if not self.paused:
            self.redraw()
        self.paused = not self.paused
        self.binary.pause_changed()
        self.pause.setText("恢复显示" if self.paused else "暂停显示")
        self.notice.setText(
            "显示已暂停 · 仍在接收和记录数据"
            if self.paused
            else "正在显示 · 横轴为" + self._clock_text()
        )
        self.dirty = True
        self.redraw()

    def reset_view(self) -> None:
        """@brief 恢复全通道自适应，去除固定十秒时基。"""
        self.follow_button.setChecked(True)
        self.follow = True
        self.x_mode.setCurrentIndex(0)
        self.y_auto = {"speed": True, "current": True}
        self.dirty = True
        self.redraw()
        self._sync_y_controls()

    def zoom(self, factor: float) -> None:
        """@brief 仅调整X时宽，保留独立Y轴和原跟随状态。@param factor 缩放比例。"""
        following = self.follow
        view = self.speed_plot.getViewBox()
        low, high = view.viewRange()[0]
        self.x_mode.blockSignals(True)
        self.x_mode.setCurrentIndex(2)
        self.x_mode.blockSignals(False)
        self.x_span.blockSignals(True)
        self.x_span.setValue(max(0.001, (high - low) * factor * 1000))
        self.x_span.blockSignals(False)
        self.x_span.setEnabled(True)
        if following:
            self.dirty = True
            self.redraw()
        else:
            view.scaleBy((factor, 1))

    def redraw(self) -> None:
        """@brief 最多30Hz更新现有曲线，不销毁重建控件。"""
        if not self.dirty:
            return
        self.dirty = False
        if self.binary.redraw():
            return
        if not self.paused:
            self.display_points = list(self.points)
        points = self.display_points
        if not points:
            return
        x = np.asarray([point.seconds for point in points], dtype=float)
        if self.follow:
            mode = self.x_mode.currentData()
            start = max(0, len(x) - 500) if mode == "auto" else 0
            span = (
                self.x_span.value() / 1000
                if mode == "manual"
                else max(float(x[-1] - x[start]), 0.001)
            )
            self.speed_plot.setXRange(
                float(x[-1]) - span, float(x[-1]), padding=0 if mode == "manual" else 0.02
            )
        x_low, x_high = self.speed_plot.getViewBox().viewRange()[0]
        visible = (x >= x_low) & (x <= x_high)
        names = {
            "id": "id_ma",
            "iq": "iq_ma",
            "rd": "ref_d_ma",
            "rq": "ref_q_ma",
            "ia": "ia_ma",
            "ib": "ib_ma",
            "ic": "ic_ma",
            "isc": "isc_ma",
            "bus": "bus_mv",
        }
        ranges: dict[str, list[float]] = {"speed": [], "current": []}
        for key, curve in self.lines.items():
            values: list[float] = []
            for point in points:
                item = (
                    point.target_rpm
                    if key == "target"
                    else point.frame.rpm if key == "rpm" else getattr(point.frame, names[key])
                )
                values.append(
                    float("nan") if item is None else float(item) / (1000 if key in names else 1)
                )
            array = np.asarray(values)
            if key not in ("target", "rpm", "bus"):
                array = smooth_current(x, array, self.filter_window_ms())
            curve.setData(x, array)
            if self.channel_checks[key].isChecked():
                selected = array[visible & np.isfinite(array)]
                if selected.size:
                    group = "speed" if key in ("target", "rpm") else "current"
                    ranges[group].extend((float(selected.min()), float(selected.max())))
        for group, plot in (("speed", self.speed_plot), ("current", self.current_plot)):
            limits = ranges[group]
            if self.y_auto[group] and limits:
                low, high = min(limits), max(limits)
                if high == low:
                    margin = max(abs(low) * 0.05, 1.0 if group == "speed" else 0.001)
                    low, high = low - margin, high + margin
                plot.setYRange(low, high, padding=0.08)
        self.count.setText(f"可见 {int(visible.sum()):,} / 缓存 {len(self.points):,} 点")
        self.notice.setToolTip(self.count.text() + "；滤波仅影响电流曲线，记录/导出为原始数据。")
        self.notice.setText(
            ("显示已暂停，仍在接收 · " if self.paused else "正在显示 · ")
            + ("跟随最新 · " if self.follow else "历史视图（关闭跟随） · ")
            + f"{self._clock_text()} · X时宽 {(x_high - x_low) * 1000:.3f} ms · {self.filter_description()}"
        )

    def _compact_changed(self, _index: int) -> None:
        """@brief 低高度窗口选择当前图。@param _index 索引。"""
        # 低高度减少工具栏留白，给真实曲线保留纵向空间。
        self.setStyleSheet(
            "QPushButton, QComboBox { padding: 3px 6px; min-height: 22px; font-size: 14px; }"
            if self.compact
            else ""
        )
        panel_layout = self.layout()
        if panel_layout is not None:
            panel_layout.setSpacing(4 if self.compact else 8)
        self.speed_plot.setVisible(not self.compact or self.compact_combo.currentIndex() == 0)
        self.current_plot.setVisible(not self.compact or self.compact_combo.currentIndex() == 1)
        # 紧凑宽度优先保证各操作按钮完整，详细点数仍在状态提示中可查看。
        self.count.setVisible(not self.compact)
        if self.compact:
            self.y_target.setCurrentIndex(1 if self.compact_combo.currentIndex() == 0 else 0)
        for key, checkbox in self.channel_checks.items():
            checkbox.setVisible(
                not self.compact
                or (key in ("target", "rpm")) == (self.compact_combo.currentIndex() == 0)
            )

    def resizeEvent(self, event: QResizeEvent) -> None:
        """@brief 低高度改为图表页签，保留左右同屏。@param event 窗口变化。"""
        super().resizeEvent(event)
        compact = self.height() < 430
        if compact != self.compact:
            self.compact = compact
            self.compact_combo.setVisible(compact)
            self._compact_changed(0)
