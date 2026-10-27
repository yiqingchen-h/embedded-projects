"""虚拟编码器独立页面：原始机械角盘、板卡确认值和运行中完整配置。"""

from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QPainter, QPaintEvent, QPen, QResizeEvent
from PySide6.QtWidgets import (
    QButtonGroup,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from .haptic_feedback import HAPTIC_KEYS
from .models import BoardSnapshot, ParameterValue
from .theme import COLORS


class AngleDial(QWidget):
    """只使用板卡真实原码绘制，未连接或快照失效时显示占位。"""

    def __init__(self) -> None:
        """@brief 初始化原始角度显示，不访问设备。"""
        super().__init__()
        self.angle: float | None = None
        self.detents = 24
        self.center_angle = 0.0
        self.setMinimumSize(100, 100)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.setAccessibleName("真实机械角度盘")

    def paintEvent(self, event: QPaintEvent) -> None:
        """@brief 绘制实际机械角。@param event Qt绘制区域。"""
        del event
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        side = min(self.width(), self.height()) - 26
        center = QPointF(self.width() / 2, self.height() / 2)
        radius = side / 2
        p.setPen(QPen(QColor(COLORS["border"]), 2))
        p.setBrush(QColor(COLORS["card"]))
        p.drawEllipse(center, radius, radius)
        for i in range(self.detents):
            theta = i * math.tau / self.detents + math.radians(self.center_angle) - math.pi / 2
            a = QPointF(
                center.x() + math.cos(theta) * radius * 0.9,
                center.y() + math.sin(theta) * radius * 0.9,
            )
            b = QPointF(
                center.x() + math.cos(theta) * radius, center.y() + math.sin(theta) * radius
            )
            p.drawLine(a, b)
        if self.angle is not None:
            theta = math.radians(self.angle) - math.pi / 2
            end = QPointF(
                center.x() + math.cos(theta) * radius * 0.72,
                center.y() + math.sin(theta) * radius * 0.72,
            )
            p.setPen(
                QPen(QColor(COLORS["primary"]), 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
            )
            p.drawLine(center, end)
        p.setPen(QColor(COLORS["text"]))
        p.drawText(
            QRectF(0, center.y() + 5, self.width(), 24),
            Qt.AlignmentFlag.AlignCenter,
            "—" if self.angle is None else f"{self.angle:.2f}°",
        )
        p.end()


class HapticPanel(QWidget):
    """完整配置由主窗口交给串口事务；该页面本身不发送指令。"""

    # 完整四项草稿及切回波形的UI事件；开启统一使用主窗口左侧按钮。
    configure = Signal(object)
    waveform = Signal()

    def __init__(self) -> None:
        """@brief 创建无需滚动的紧凑手感页，编辑期间保留焦点和文字。"""
        super().__init__()
        self.setObjectName("hapticPanel")
        self.pending: dict[str, ParameterValue] | None = None
        # 区分已发出的配置与后续草稿，避免旧回读提前清掉用户的新输入。
        self._submitted: dict[str, ParameterValue] | None = None
        self._editing: QSpinBox | QDoubleSpinBox | None = None
        self.state = BoardSnapshot()
        layout = QVBoxLayout(self)
        self._content_layout = layout
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(6)
        feels = QHBoxLayout()
        self.group = QButtonGroup(self)
        self.detent = QPushButton("档位吸附＋阻尼")
        self.damped = QPushButton("连续阻尼")
        for index, button in enumerate((self.detent, self.damped)):
            button.setProperty("role", "feel")
            button.setCheckable(True)
            self.group.addButton(button, index)
            feels.addWidget(button)
            button.clicked.connect(self._changed)
        self.detent.setChecked(True)
        self.wave_button = QPushButton("查看 Id / Iq 波形")
        self.wave_button.clicked.connect(self.waveform)
        feels.addWidget(self.wave_button)
        layout.addLayout(feels)
        display = QHBoxLayout()
        self.dial = AngleDial()
        self.dial.setFixedSize(140, 140)
        display.addWidget(self.dial)
        readouts = QGridLayout()
        self._readouts = readouts
        readouts.setVerticalSpacing(10)
        self.turns = QLabel("累计圈数 —")
        self.index = QLabel("累计档数 —")
        self.center = QLabel("档位中心 —")
        self.current = QLabel("参考 / 实际 Iq —")
        self.speed = QLabel("实际转速 —")
        for row, label in enumerate((self.turns, self.index, self.center, self.current)):
            label.setWordWrap(True)
            readouts.addWidget(label, row // 2, row % 2)
        self.speed.setWordWrap(True)
        readouts.addWidget(self.speed, 0, 1)
        self.speed.hide()
        display.addLayout(readouts, 1)
        layout.addLayout(display)
        settings = QGroupBox("手感设置")
        settings.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        form = QGridLayout(settings)
        self._form = form
        self._field_labels: list[QLabel] = []
        form.setContentsMargins(10, 8, 10, 6)
        form.setHorizontalSpacing(12)
        form.setVerticalSpacing(4)
        self.detents = QSpinBox()
        self.detents.setRange(2, 256)
        self.detents.setValue(24)
        self.detents.setSuffix(" 档/圈")
        self.strength = QSpinBox()
        self.strength.setRange(0, 800)
        self.strength.setValue(100)
        self.strength.setSuffix(" mA")
        self.damping = QDoubleSpinBox()
        self.damping.setRange(0, 10)
        self.damping.setDecimals(3)
        self.damping.setSingleStep(0.1)
        self.damping.setValue(2)
        self.damping.setSuffix(" mA/RPM")
        self._inputs = (self.detents, self.strength, self.damping)
        fields: tuple[tuple[str, QSpinBox | QDoubleSpinBox], ...] = (
            ("每圈档位", self.detents),
            ("档位力度", self.strength),
            ("阻尼系数", self.damping),
        )
        for column, (title, control) in enumerate(fields):
            control.setAccessibleName(title)
            control.setKeyboardTracking(False)
            # 紧凑窗口中内嵌QLineEdit贴近圆角边界，保留字形左侧空间，避免首位数字裁切。
            control.lineEdit().setTextMargins(6, 0, 0, 0)
            control.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            label = QLabel(title)
            label.setBuddy(control)
            self._field_labels.append(label)
            form.addWidget(label, 0, column)
            form.addWidget(control, 1, column)
            form.setColumnStretch(column, 1)
            control.valueChanged.connect(self._changed)
            control.lineEdit().textEdited.connect(
                lambda _text, field=control: self._begin_edit(field)
            )
            control.editingFinished.connect(self._edit_finished)
        layout.addWidget(settings)
        self.feel_hint = QLabel()
        self.feel_hint.setWordWrap(True)
        self.feel_hint.setObjectName("hint")
        layout.addWidget(self.feel_hint)
        self.confirmed = QLabel("板卡回读：—")
        self.confirmed.setWordWrap(True)
        layout.addWidget(self.confirmed)
        self.status = QLabel("连接板卡后显示角度与手感")
        self.status.setWordWrap(True)
        self.status.setObjectName("hint")
        layout.addWidget(self.status)
        layout.addStretch()
        self.setStyleSheet(
            "QWidget#hapticPanel QLabel { font-size: 14px; }"
            "QWidget#hapticPanel QPushButton { padding: 4px 8px; font-size: 14px; }"
            "QWidget#hapticPanel QGroupBox { margin-top: 10px; padding-top: 2px; }"
            "QWidget#hapticPanel QSpinBox, QWidget#hapticPanel QDoubleSpinBox { font-size: 14px; }"
        )
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.setInterval(140)
        self.timer.timeout.connect(self._submit)
        self._show_kind()

    def _show_kind(self) -> None:
        """@brief 按当前类型草稿显示有效参数，隐藏的档位配置仍保留供切回。"""
        detent = self.group.checkedId() == 0
        for label, control in zip(self._field_labels[:2], self._inputs[:2], strict=True):
            label.setVisible(detent)
            control.setVisible(detent)
        self._form.setColumnStretch(0, int(detent))
        self._form.setColumnStretch(1, int(detent))
        self.index.setVisible(detent)
        self.center.setVisible(detent)
        self.speed.setVisible(not detent)
        self._readouts.removeWidget(self.current)
        self._readouts.addWidget(self.current, 1, 1 if detent else 0, 1, 1 if detent else 2)
        self.dial.detents = self.detents.value() if detent else 0
        self.dial.update()
        self.feel_hint.setText(
            "每圈档位决定间距，档位力度决定吸附强度；阻尼用于抑制回弹。"
            if detent
            else "转得越快，阻力越大；静止不回中。阻尼转矩受左侧电流上限约束。"
        )

    def values(self) -> dict[str, ParameterValue]:
        """@brief 生成四项完整配置。@return 与串口尺度一致的草稿。"""
        return dict(
            zip(
                HAPTIC_KEYS,
                (
                    ParameterValue(self.group.checkedId(), 0, ""),
                    ParameterValue(self.detents.value(), 0, ""),
                    ParameterValue(self.strength.value(), 0, "MA"),
                    ParameterValue(round(self.damping.value() * 1000), 3, "MA/RPM"),
                ),
                strict=True,
            )
        )

    def _changed(self) -> None:
        """@brief 合并已确认的数值或档位按钮，只提交完整配置。"""
        self._show_kind()
        if (
            not self.state.writable
            or self.state.mode != "HAPTIC"
            or self.state.busy not in ("", "应用参数")
        ):
            return
        values = self.values()
        if (
            self.pending is None
            and self.state.haptic is not None
            and values == self.state.haptic.parameters()
        ):
            self.show_state(self.state)
            return
        self.pending = values
        self.status.setText("切换中 · 等待板卡确认")
        self.timer.start()

    def _begin_edit(self, control: QSpinBox | QDoubleSpinBox) -> None:
        """@brief 标记正在输入，不提交未完成数字。@param control 当前输入框。"""
        self._editing = control
        self.status.setText("编辑中 · Enter确认或移开焦点应用")

    def _edit_finished(self) -> None:
        """@brief Enter或失去焦点后才提交数字，Qt继续负责范围和小数校验。"""
        self._editing = None
        self._changed()

    def cancel_drafts(self) -> None:
        """@brief 停止、断开或模式变化时撤销待提交草稿。"""
        self.timer.stop()
        self.pending = self._submitted = None
        self._editing = None

    def _submit(self) -> None:
        """@brief 提交最后一次草稿；停机/断线或模式变化时撤销旧提交。"""
        if self.pending is not None and self.state.writable and self.state.mode == "HAPTIC":
            if self.state.busy or self._submitted is not None:
                self.timer.start()
            else:
                self._submitted = dict(self.pending)
                self.configure.emit(self._submitted)

    def show_state(self, state: BoardSnapshot) -> None:
        """@brief 显示真实回读并管理断线及待保存状态。@param state 后台不可变快照。"""
        self.state = state
        active = state.connected and state.haptic_available and state.mode == "HAPTIC"
        sample = state.haptic if active else None
        if not active or state.error:
            self.cancel_drafts()
        if sample is not None and self._submitted == sample.parameters():
            self._submitted = None
        if sample is not None and self._submitted is None and self.pending == sample.parameters():
            self.pending = None
        if self.pending is None and sample is not None:
            controls = (self.detent, self.damped, self.detents, self.strength, self.damping)
            for control in controls:
                control.blockSignals(True)
            self.detent.setChecked(sample.kind == "DETENT")
            self.damped.setChecked(sample.kind == "DAMPED")
            for control, value in (
                (self.detents, sample.detents),
                (self.strength, sample.strength_ma),
                (self.damping, sample.damping_ua_rpm / 1000),
            ):
                # 即使回读值未变，setValue也可能重写未完成文本和光标；焦点内完全保留。
                if not control.hasFocus() and not control.lineEdit().hasFocus():
                    if isinstance(control, QDoubleSpinBox):
                        control.setValue(value)
                    else:
                        control.setValue(int(value))
            for control in controls:
                control.blockSignals(False)
        editable = (
            active and state.writable and state.busy in ("", "应用参数") and state.fault == "NONE"
        )
        for control in (self.detent, self.damped, self.detents, self.strength, self.damping):
            control.setEnabled(editable)
        if sample is None:
            self.status.setText("未连接 / 尚无手感数据")
        elif self._editing is not None:
            self.status.setText("编辑中 · Enter确认或移开焦点应用")
        elif state.busy:
            self.status.setText(state.busy)
        elif self.pending is not None or sample.transitioning:
            self.status.setText("切换中")
        else:
            self.status.setText(
                ("运行中 · " if sample.running else "已停机 · ")
                + "已生效"
                + (" · 待停机保存" if state.save == "PENDING" else " · 已保存")
            )
        self.dial.angle = sample.angle_deg if sample is not None and sample.valid else None
        self._show_kind()
        direction = state.parameters.get("ENCODER_DIR", ParameterValue(1, 0, "")).value
        self.dial.center_angle = (
            (sample.raw + (sample.center - sample.counts) * direction) * 360 / 2097152
            if sample is not None
            else 0
        )
        self.dial.update()
        self.turns.setText(f"累计圈数 {sample.turns:+.4f}" if sample is not None else "累计圈数 —")
        self.index.setText(f"累计档数 {sample.index:+d}" if sample is not None else "累计档数 —")
        self.center.setText(
            f"档位中心 {sample.center * 360 / 2097152:+.2f}°（相对起点）"
            if sample is not None
            else "档位中心 —"
        )
        self.current.setText(
            f"参考 / 实际 Iq {sample.reference_a:+.3f} / {sample.iq_a:+.3f} A"
            if sample is not None
            else "参考 / 实际 Iq —"
        )
        self.speed.setText(
            f"实际转速 {sample.rpm:+.3f} RPM" if sample is not None else "实际转速 —"
        )
        self.confirmed.setText(
            (
                f"回读：档位吸附＋阻尼 · {sample.detents}档 · {sample.strength_ma}mA · {sample.damping_ua_rpm / 1000:g}mA/RPM"
                if sample.kind == "DETENT"
                else f"回读：连续阻尼 · {sample.damping_ua_rpm / 1000:g}mA/RPM"
            )
            if sample is not None
            else "板卡回读：—"
        )

    def resizeEvent(self, event: QResizeEvent) -> None:
        """@brief 按可用高度缩放角盘，保持所有操作完整可见。@param event 尺寸变化。"""
        super().resizeEvent(event)
        if hasattr(self, "dial"):
            side = max(100, min(168, self.height() - 245))
            self.dial.setFixedSize(side, side)
            self._content_layout.setSpacing(4 if self.height() < 330 else 6)
