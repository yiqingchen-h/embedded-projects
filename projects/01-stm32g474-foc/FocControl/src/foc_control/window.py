"""FocControl 中文工程面板，编辑草稿与实际板卡状态明确区分。"""

from __future__ import annotations

import json
from collections import deque
from dataclasses import replace
from pathlib import Path
from typing import cast

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QCloseEvent, QColor, QResizeEvent
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QInputDialog,
    QLabel,
    QLayout,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from . import __version__
from .binary_wave import WaveBuffer
from .haptic_feedback import HAPTIC_KEYS
from .haptic_panel import HapticPanel
from .models import (
    ALGORITHMS,
    ENCODER_ALGORITHMS,
    FOC_ALGORITHMS,
    MODES,
    BoardSnapshot,
    ControlError,
    ParameterSpec,
    ParameterValue,
    protocol_supported,
    specs_for,
)
from .position_plot import PositionPanel
from .protocol import PlotPoint
from .storage import Preset, export_csv, settings_load, timestamp
from .theme import icon
from .transport import available_ports
from .wave_plot import ExportJob
from .widgets import ElidedLabel, Metric, ParameterEditor, PlotPanel, refresh_style
from .worker import SerialWorker


class MainWindow(QMainWindow):
    """控制、波形、命名预设及有界展示日志。"""

    def __init__(self, data_dir: Path, read_only: bool = False, demo: bool = False) -> None:
        """@brief 构建界面但不连接。@param data_dir 数据目录。@param read_only 只读门禁。
        @param demo 离线演示，不访问串口。
        """
        super().__init__()
        self.data_dir, self.read_only, self.demo = data_dir, read_only, demo
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.preferences = settings_load(data_dir / "settings.json")
        self.snapshot = BoardSnapshot(read_only=read_only)
        self.worker: SerialWorker | None = None
        self.editors: dict[str, ParameterEditor] = {}
        self.editor_algorithm = ""
        self.editor_mode = ""
        self.editor_specs: dict[str, ParameterSpec] = {}
        self.log_records: deque[tuple[str, str, str]] = deque(maxlen=500)
        self.selected_preset: Preset | None = None
        self.close_intent = False
        self.disconnect_confirmed = False
        self.force_confirmed = False
        self.awaiting_disconnect = False
        self.setWindowTitle(
            f"FocControl {__version__} · FOC 板卡控制台" + (" · 离线演示" if demo else "")
        )
        self.setMinimumSize(900, 560)
        self.resize(1280, 800)
        screen = self.screen()
        if screen is not None:
            area = screen.availableGeometry()
            self.setMinimumSize(min(900, area.width() - 32), min(560, area.height() - 32))
            self.resize(min(1280, area.width() - 32), min(800, area.height() - 32))
        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        outer.addWidget(self._toolbar())
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(16, 12, 16, 8)
        layout.setSpacing(8)
        self.banner = QLabel()
        self.banner.setObjectName("banner")
        self.banner.setWordWrap(True)
        self.banner.hide()
        layout.addWidget(self.banner)
        self.main_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.main_splitter.setChildrenCollapsible(False)
        self.main_splitter.addWidget(self._controls())
        self.main_splitter.addWidget(self._right_panel())
        self.main_splitter.setStretchFactor(0, 0)
        self.main_splitter.setStretchFactor(1, 1)
        self.main_splitter.setSizes([320, 920])
        layout.addWidget(self.main_splitter, 1)
        # 通信日志已移入右侧「上位机设置」页，主界面底部只保留状态栏。
        outer.addWidget(content, 1)
        self.statusBar().showMessage("未连接 · 固件 — · 参数 —")
        self.timer = QTimer(self)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self.plots.redraw)
        self.timer.start()
        self.refresh_ports()
        self.refresh_presets()
        self.preferences.setdefault("baud", self.baud_combo.currentText())
        if not self.demo and self.port_combo.currentText():
            self.preferences.setdefault("port", self.port_combo.currentText())
        self.port_combo.currentTextChanged.connect(self._port_preference_changed)
        self.baud_combo.currentTextChanged.connect(self._baud_preference_changed)
        self._buttons()

    def _toolbar(self) -> QFrame:
        """@brief 创建始终可见的连接和停止栏。@return 顶栏。"""
        frame = QFrame()
        frame.setObjectName("toolbar")
        row = QHBoxLayout(frame)
        row.setContentsMargins(16, 10, 16, 10)
        row.setSpacing(8)
        brand = QLabel("FocControl")
        brand.setObjectName("brand")
        row.addWidget(brand)
        row.addSpacing(12)
        self.port_combo = QComboBox()
        self.port_combo.setAccessibleName("串口")
        self.port_combo.setMinimumWidth(90)
        self.port_combo.setToolTip("记住上次选择；刷新只读取系统列表，不探测设备")
        row.addWidget(self.port_combo)
        self.refresh_button = QPushButton()
        self.refresh_button.setIcon(icon("refresh"))
        self.refresh_button.setFixedWidth(40)
        self.refresh_button.setAccessibleName("刷新串口")
        self.refresh_button.setToolTip("刷新串口")
        self.refresh_button.clicked.connect(self.refresh_ports)
        row.addWidget(self.refresh_button)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(("921600", "1000000", "2000000", "3000000", "115200"))
        self.baud_combo.setAccessibleName("波特率")
        self.baud_combo.setToolTip(
            "记住连接时的选择；连接后显示实际速率。固件默认921600 / 8N1 / 无流控"
        )
        baud = str(self.preferences.get("baud", "921600"))
        if self.baud_combo.findText(baud) >= 0:
            self.baud_combo.setCurrentText(baud)
        row.addWidget(self.baud_combo)
        self.connect_button = QPushButton("连接")
        self.connect_button.setIcon(icon("link"))
        self.connect_button.clicked.connect(self.toggle_connection)
        row.addWidget(self.connect_button)
        self.connection_badge = QLabel("未连接")
        self.connection_badge.setObjectName("badge")
        row.addWidget(self.connection_badge)
        self.mode_badge = QLabel("演示数据" if self.demo else "只读模式" if self.read_only else "")
        self.mode_badge.setObjectName("badge")
        self.mode_badge.setProperty("tone", "warning")
        self.mode_badge.setVisible(self.demo or self.read_only)
        row.addWidget(self.mode_badge)
        row.addStretch()
        self.stop_button = QPushButton("停止")
        self.stop_button.setProperty("role", "danger")
        self.stop_button.setIcon(icon("stop", "#FFFFFF"))
        self.stop_button.setMinimumWidth(112)
        self.stop_button.setMinimumHeight(28)
        self.stop_button.setToolTip("提交 STOP，并回读运行状态与输出禁用；串口失联不能保证停机")
        self.stop_button.clicked.connect(lambda: self.submit("stop"))
        row.addWidget(self.stop_button)
        return frame

    def _controls(self) -> QFrame:
        """@brief 创建可滚动参数与固定应用/启动按钮。@return 控制区。"""
        frame = QFrame()
        frame.setObjectName("controls")
        frame.setMinimumWidth(284)
        frame.setMaximumWidth(480)
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(16, 16, 16, 12)
        layout.setSpacing(8)
        self.control_heading = QLabel("控制与参数")
        self.control_heading.setObjectName("heading")
        layout.addWidget(self.control_heading)
        algorithm_label = QLabel("控制算法")
        algorithm_label.setObjectName("muted")
        layout.addWidget(algorithm_label)
        self.algorithm_combo = QComboBox()
        for key, name in ALGORITHMS.items():
            self.algorithm_combo.addItem(name, key)
        algorithm_label.setBuddy(self.algorithm_combo)
        self.algorithm_combo.currentIndexChanged.connect(self._algorithm_changed)
        layout.addWidget(self.algorithm_combo)
        mode_label = QLabel("控制模式")
        mode_label.setObjectName("muted")
        mode_row = QHBoxLayout()
        mode_row.addWidget(mode_label)
        self.control_mode_combo = QComboBox()
        self.control_mode_combo.setAccessibleName("控制模式")
        mode_label.setBuddy(self.control_mode_combo)
        self.control_mode_combo.addItem(MODES["SPEED"], "SPEED")
        self.control_mode_combo.currentIndexChanged.connect(self._mode_changed)
        mode_row.addWidget(self.control_mode_combo, 1)
        layout.addLayout(mode_row)
        self.current_hint = QLabel("Iq ±0.500 A · Id=0\nIq=0 仍通电；停机用顶部停止")
        self.current_hint.setObjectName("hint")
        self.current_hint.setWordWrap(True)
        self.current_hint.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)
        self.current_hint.hide()
        layout.addWidget(self.current_hint)
        self.position_hint = QLabel("多圈机械角 · 到位通电保持")
        self.position_hint.setToolTip("到位后仍通电保持，停止使用顶部停止按钮。")
        self.position_hint.setObjectName("hint")
        self.position_hint.setWordWrap(True)
        self.position_hint.hide()
        layout.addWidget(self.position_hint)
        self.source_hint = QLabel("连接后读取板卡当前参数")
        self.source_hint.setObjectName("hint")
        self.source_hint.setWordWrap(True)
        layout.addWidget(self.source_hint)
        self.remote_button = QPushButton("切换为上位机控制")
        self.remote_button.clicked.connect(lambda: self.submit("remote"))
        self.remote_button.hide()
        layout.addWidget(self.remote_button)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        self.form_widget = QWidget()
        self.form_layout = QVBoxLayout(self.form_widget)
        self.form_layout.setContentsMargins(0, 4, 4, 4)
        self.form_layout.setSpacing(12)
        self.empty_form = QLabel(
            "连接成功后显示真实参数。\n\n高级参数默认收起，输入值不会直接写入板卡。"
        )
        self.empty_form.setObjectName("hint")
        self.empty_form.setWordWrap(True)
        self.form_layout.addWidget(self.empty_form)
        self.form_layout.addStretch()
        scroll.setWidget(self.form_widget)
        layout.addWidget(scroll, 1)
        self.draft_label = QLabel("参数尚未读取")
        self.draft_label.setObjectName("hint")
        self.draft_label.setWordWrap(True)
        layout.addWidget(self.draft_label)
        self.discard_button = QPushButton("撤销草稿")
        self.discard_button.clicked.connect(self.discard_drafts)
        layout.addWidget(self.discard_button)
        self.apply_button = QPushButton("应用参数")
        self.apply_button.setIcon(icon("save"))
        self.apply_button.clicked.connect(lambda: self.submit_drafts("apply"))
        layout.addWidget(self.apply_button)
        self.start_button = QPushButton("启动")
        self.start_button.setProperty("role", "primary")
        self.start_button.setIcon(icon("play", "#FFFFFF"))
        self.start_button.setMinimumHeight(28)
        self.start_button.clicked.connect(lambda: self.submit_drafts("start"))
        layout.addWidget(self.start_button)
        self.calibrate_button = QPushButton("编码器七点标定")
        self.calibrate_button.clicked.connect(self.initialize_control)
        self.calibrate_button.hide()
        layout.addWidget(self.calibrate_button)
        self.clear_button = QPushButton("清除故障 / 重配置")
        self.clear_button.clicked.connect(lambda: self.submit("clear"))
        layout.addWidget(self.clear_button)
        self.control_reason = QLabel("请先连接板卡")
        self.control_reason.setObjectName("hint")
        self.control_reason.setWordWrap(True)
        layout.addWidget(self.control_reason)
        return frame

    def _right_panel(self) -> QWidget:
        """@brief 构建指标、阶段和波形/预设页。@return 内容区。"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 0, 0, 0)
        layout.setSpacing(8)
        metrics = QHBoxLayout()
        metrics.setSpacing(8)
        self.speed_metric = Metric("机械转速", "RPM")
        self.iq_metric = Metric("Iq 电流", "A")
        self.id_metric = Metric("Id 电流", "A")
        self.bus_metric = Metric("母线电压", "V")
        self.run_metric = Metric("运行状态", "")
        self.run_metric.value.setStyleSheet('font-family: "Microsoft YaHei"; font-size: 19px;')
        for metric in (
            self.speed_metric,
            self.id_metric,
            self.iq_metric,
            self.bus_metric,
            self.run_metric,
        ):
            metrics.addWidget(metric)
        layout.addLayout(metrics)
        # 阶段徽章与读取时间合并为一行：徽章紧凑靠左，时间说明占剩余宽度靠右。
        stage_row = QHBoxLayout()
        stage_row.setSpacing(6)
        self.stage_labels: dict[str, QLabel] = {}
        for name in ("校准", "定位", "升频", "资格确认", "融合", "闭环"):
            label = QLabel(name)
            label.setObjectName("badge")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setStyleSheet("padding: 2px 8px;")
            label.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
            stage_row.addWidget(label)
            self.stage_labels[name] = label
        stage_row.addSpacing(12)
        self.data_hint = ElidedLabel("尚无有效数据")
        self.data_hint.setObjectName("hint")
        self.data_hint.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        stage_row.addWidget(self.data_hint, 1)
        layout.addLayout(stage_row)
        card = QFrame()
        card.setObjectName("card")
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(8, 4, 8, 8)
        self.tabs = QTabWidget()
        self.plots = PlotPanel()
        self.wave_exports: list[ExportJob] = []
        preferred = self.preferences.get("preferred_wave_baud", 0)
        if preferred not in (0, 921600, 1000000, 2000000, 3000000):
            preferred = 0
        preferred_index = self.plots.binary.baud.findData(preferred)
        self.plots.binary.baud.setCurrentIndex(max(0, preferred_index))
        self.plots.binary.baud.currentIndexChanged.connect(self.wave_baud_changed)
        self.plots.export.clicked.connect(self.export_waveform)
        self.tabs.addTab(self.plots, "实时波形")
        self.position_panel = PositionPanel()
        self.position_tab = self.tabs.addTab(self.position_panel, "位置与误差")
        self.tabs.setTabVisible(self.position_tab, False)
        self.haptic_panel = HapticPanel()
        self.haptic_tab = self.tabs.addTab(self.haptic_panel, "虚拟编码器")
        self.tabs.setTabVisible(self.haptic_tab, False)
        self.haptic_panel.configure.connect(lambda values: self.submit("apply", values))
        self.haptic_panel.waveform.connect(lambda: self.tabs.setCurrentWidget(self.plots))
        self.tabs.addTab(self._preset_panel(), "参数预设")
        self.tabs.addTab(self._host_settings_panel(), "上位机设置")
        self.tabs.currentChanged.connect(self._tab_changed)
        card_layout.addWidget(self.tabs)
        layout.addWidget(card, 1)
        return panel

    def _preset_panel(self) -> QWidget:
        """@brief 创建本地方案列表和差异表。@return 预设页。"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 8)
        row = QHBoxLayout()
        self.backup_button = QPushButton("备份当前参数")
        self.backup_button.clicked.connect(self.backup_preset)
        row.addWidget(self.backup_button)
        self.import_button = QPushButton("导入")
        self.import_button.clicked.connect(self.import_preset)
        row.addWidget(self.import_button)
        self.copy_button = QPushButton("复制修改")
        self.copy_button.clicked.connect(self.copy_preset)
        row.addWidget(self.copy_button)
        self.preset_export_button = QPushButton("导出预设")
        self.preset_export_button.clicked.connect(self.export_preset)
        row.addWidget(self.preset_export_button)
        row.addStretch()
        layout.addLayout(row)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        self.preset_list = QListWidget()
        self.preset_list.setMinimumWidth(120)
        self.preset_list.currentItemChanged.connect(self._preset_selected)
        splitter.addWidget(self.preset_list)
        self.diff_table = QTableWidget(0, 3)
        self.diff_table.setHorizontalHeaderLabels(("参数", "当前板卡", "预设值"))
        self.diff_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.diff_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.diff_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        splitter.addWidget(self.diff_table)
        splitter.setSizes([160, 520])
        layout.addWidget(splitter, 1)
        self.preset_hint = QLabel("暂无预设；保持停机后备份当前配置。")
        self.preset_hint.setObjectName("hint")
        self.preset_hint.setWordWrap(True)
        layout.addWidget(self.preset_hint)
        footer = QHBoxLayout()
        compare = QPushButton("刷新只读比较")
        compare.clicked.connect(self.show_difference)
        footer.addWidget(compare)
        footer.addStretch()
        self.restore_button = QPushButton("恢复所选预设")
        self.restore_button.setProperty("role", "primary")
        self.restore_button.clicked.connect(self.restore_preset)
        footer.addWidget(self.restore_button)
        layout.addLayout(footer)
        return panel

    def _host_settings_panel(self) -> QWidget:
        """@brief 集中放置显示、波形通信、位置设置和通信日志。@return 设置页。"""
        panel = QWidget()
        outer = QVBoxLayout(panel)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        content = QWidget()
        layout = QVBoxLayout(content)
        # 矮窗口改为滚动设置，不能挤压分组导致字段和说明重叠。
        layout.setSizeConstraint(QLayout.SizeConstraint.SetMinimumSize)
        layout.setContentsMargins(12, 12, 12, 8)
        layout.setSpacing(10)
        display = QGroupBox("波形显示")
        display_layout = QVBoxLayout(display)
        filter_row = QHBoxLayout()
        filter_row.addWidget(QLabel("电流显示滤波"))
        filter_row.addWidget(self.plots.filter_combo)
        filter_row.addStretch()
        display_layout.addLayout(filter_row)
        display_note = QLabel("仅影响电流曲线显示，记录与导出 CSV 保持原始数据")
        display_note.setObjectName("hint")
        display_note.setWordWrap(True)
        display_layout.addWidget(display_note)
        layout.addWidget(display)
        position = QGroupBox("位置控制设置")
        position_layout = QVBoxLayout(position)
        tolerance_row = QHBoxLayout()
        tolerance_row.addWidget(QLabel("到位容差"))
        self.position_tolerance_combo = QComboBox()
        self.position_tolerance_combo.addItem("±0.1°", 100)
        self.position_tolerance_combo.addItem("±0.5°", 500)
        self.position_tolerance_combo.setCurrentIndex(-1)
        self.position_tolerance_combo.setAccessibleName("位置到位容差")
        self.position_tolerance_combo.currentIndexChanged.connect(self._position_tolerance_changed)
        tolerance_row.addWidget(self.position_tolerance_combo)
        tolerance_row.addStretch()
        self.recalibrate_button = QPushButton("重新执行七点标定")
        self.recalibrate_button.clicked.connect(lambda: self.submit_drafts("recalibrate"))
        tolerance_row.addWidget(self.recalibrate_button)
        position_layout.addLayout(tolerance_row)
        self.position_settings_hint = QLabel("连接后在位置模式停机设置；容差会写入板卡并保存。")
        self.position_settings_hint.setObjectName("hint")
        self.position_settings_hint.setWordWrap(True)
        position_layout.addWidget(self.position_settings_hint)
        layout.addWidget(position)
        link = QGroupBox("波形通信")
        link_layout = QVBoxLayout(link)
        link_layout.addWidget(self.plots.binary.host_settings)
        layout.addWidget(link)
        self.log_panel = self._log_panel()
        self.log_panel.setMinimumHeight(160)
        # 设置页内日志可占满剩余高度。
        self.log_text.setMaximumHeight(16777215)
        self.log_toggle = QToolButton()
        self.log_toggle.setText("通信日志")
        self.log_toggle.setArrowType(Qt.ArrowType.RightArrow)
        self.log_toggle.setCheckable(True)
        self.log_toggle.toggled.connect(self._toggle_logs)
        self.log_toggle.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        layout.addWidget(self.log_toggle, alignment=Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self.log_panel, 1)
        layout.addStretch(0)
        # 设置页内日志默认展开，仍可点标题折叠。
        self.log_toggle.setChecked(True)
        scroll.setWidget(content)
        outer.addWidget(scroll)
        return panel

    def _log_panel(self) -> QFrame:
        """@brief 创建有界展示日志和停机诊断原文入口。@return 日志区。"""
        panel = QFrame()
        panel.setObjectName("card")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        row = QHBoxLayout()
        self.log_filter = QComboBox()
        for text, key in (
            ("全部日志", ""),
            ("发送", "TX"),
            ("回执", "RX"),
            ("错误", "ERROR"),
            ("恢复审计", "AUDIT"),
        ):
            self.log_filter.addItem(text, key)
        self.log_filter.currentIndexChanged.connect(self._render_logs)
        row.addWidget(self.log_filter)
        export = QPushButton("导出展示日志")
        export.clicked.connect(self.export_logs)
        row.addWidget(export)
        diagnostics = QPushButton("查看停机诊断")
        diagnostics.clicked.connect(self.show_diagnostics)
        row.addWidget(diagnostics)
        row.addStretch()
        layout.addLayout(row)
        self.log_text = QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumBlockCount(500)
        self.log_text.setMinimumHeight(100)
        self.log_text.setMaximumHeight(180)
        layout.addWidget(self.log_text)
        return panel

    def refresh_ports(self) -> None:
        """@brief 枚举但不打开设备，暂时缺席的端口仍保留用户选择。"""
        current = self.port_combo.currentText() or str(self.preferences.get("port", ""))
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        if self.demo:
            self.port_combo.addItem("DEMO")
        else:
            ports = list(available_ports())
            if current and current not in ports:
                ports.append(current)
            self.port_combo.addItems(ports)
            if current:
                self.port_combo.setCurrentText(current)
            self.port_combo.setPlaceholderText("选择串口")
        self.port_combo.blockSignals(False)

    def _save_preferences(self) -> None:
        """@brief 立即保存本机偏好，完整替换文件并保留其它已存字段，不发送板卡命令。"""
        path = self.data_dir / "settings.json"
        temporary = path.with_suffix(".tmp")
        try:
            temporary.write_text(
                json.dumps(self.preferences, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            temporary.replace(path)
        except OSError as error:
            self.show_message("设置保存失败：" + str(error))

    def _port_preference_changed(self, port: str) -> None:
        """@brief 保存用户选中的COM，空列表或演示不覆盖原端口。@param port 端口名。"""
        if port and not self.demo:
            self.preferences["port"] = port
            self._save_preferences()

    def _baud_preference_changed(self, baud: str) -> None:
        """@brief 保存用户连接档，实际链路更新由独立缓存处理。@param baud 波特率文本。"""
        self.preferences["baud"] = baud
        self._save_preferences()

    def toggle_connection(self) -> None:
        """@brief 用户显式连接或停机断开。"""
        if self.worker is not None and self.worker.isRunning():
            self.awaiting_disconnect = True
            self.disconnect_confirmed = False
            self.submit("disconnect")
            return
        port = self.port_combo.currentText()
        if not port:
            self.show_message("未找到串口，请连接 USB 串口后刷新。")
            return
        self.disconnect_confirmed = False
        self.force_confirmed = False
        self.snapshot = BoardSnapshot(read_only=self.read_only, busy="正在验证连接")
        self.worker = SerialWorker(
            port, int(self.baud_combo.currentText()), self.data_dir, self.read_only, self.demo
        )
        self.worker.preferred_wave_baud = int(self.plots.binary.baud.currentData())
        cached = self.preferences.get("last_link_baud", 2000000)
        self.worker.fallback_baud = cached if isinstance(cached, int) else 2000000
        self.worker.wave_received.connect(self.receive_wave)
        self.worker.state_changed.connect(self.receive_state)
        self.worker.points_received.connect(self.receive_points)
        self.worker.position_received.connect(self.position_panel.receive)
        self.worker.logs_received.connect(self.receive_logs)
        self.worker.operation_done.connect(self.operation_done)
        self.worker.operation_failed.connect(self.operation_failed)
        self.worker.finished.connect(self.worker_finished)
        self.worker.start()
        self._buttons()

    def submit(self, action: str, payload: object = None) -> None:
        """@brief 投递用户意图。@param action 操作。@param payload 已校验参数。"""
        if action in ("stop", "disconnect"):
            self.haptic_panel.cancel_drafts()
        if self.worker is None or not self.worker.isRunning():
            self.show_message("串口尚未连接")
            return
        self.worker.submit(action, payload)

    def drafts(self) -> dict[str, ParameterValue]:
        """@brief 读取并验证草稿。@return 仅修改项。"""
        result: dict[str, ParameterValue] = {}
        for key, editor in self.editors.items():
            if editor.dirty:
                result[key] = editor.candidate()
        if self.snapshot.mode == "HAPTIC" and self.haptic_panel.pending is not None:
            result.update(self.haptic_panel.pending)
        return result

    def submit_drafts(self, action: str) -> None:
        """@brief 提交草稿，位置配置修改合并保存和置零。@param action 用户操作。"""
        try:
            draft = self.drafts()
            if action == "apply" and self.snapshot.mode == "POSITION" and set(draft) - {"TARGET"}:
                action = "position_initialize"
            self.submit(action, draft)
        except ControlError as error:
            for editor in self.editors.values():
                editor.show_error()
            self.show_message(str(error))

    def initialize_control(self) -> None:
        """@brief 按当前模式提交一次初始化，位置模式自动串联按需标定及机械置零。"""
        self.submit_drafts(
            "position_initialize" if self.snapshot.mode == "POSITION" else "calibrate"
        )

    def discard_drafts(self) -> None:
        """@brief 恢复最近真实回读值。"""
        for editor in self.editors.values():
            editor.discard()
        self._buttons()

    def _algorithm_changed(self, _index: int) -> None:
        """@brief 算法切换交给停机业务，不本地伪造新参数。@param _index 索引。"""
        if self.snapshot.connected:
            algorithm = str(self.algorithm_combo.currentData())
            if algorithm != self.snapshot.algorithm:
                self.submit("algorithm", algorithm)

    def _mode_changed(self, _index: int) -> None:
        """@brief 由后台确认停机、静止并备份后切换模式。@param _index 选中索引。"""
        mode = str(self.control_mode_combo.currentData())
        if self.snapshot.connected and mode != self.snapshot.mode:
            self.submit("mode", mode)

    def _build_fields(self, state: BoardSnapshot) -> None:
        """@brief 按真实算法分组，仅常用参数默认展开。@param state 快照。"""
        self.editors.clear()
        while self.form_layout.count():
            item = self.form_layout.takeAt(0)
            widget = item.widget() if item is not None else None
            if widget is not None:
                widget.hide()
                widget.deleteLater()
        groups: dict[str, QWidget] = {}
        group_layouts: dict[str, QVBoxLayout] = {}
        profile = state.parameter_specs or specs_for(state.algorithm, state.parameters, state.mode)
        order = ("常用参数", "位置环", "速度环", "启动参数", "电机参数", "固定模型", "预留参数")
        ordered = sorted(
            ((k, v) for k, v in profile.items() if k not in HAPTIC_KEYS),
            key=lambda item: order.index(item[1].group),
        )
        for key, definition in ordered:
            value = state.parameters.get(key)
            if value is None:
                continue
            group = definition.group
            if group not in groups:
                section = QWidget()
                section_layout = QVBoxLayout(section)
                section_layout.setContentsMargins(0, 0, 0, 0)
                section_layout.setSpacing(12)
                if group != "常用参数":
                    toggle = QToolButton()
                    toggle.setText(group)
                    toggle.setArrowType(Qt.ArrowType.RightArrow)
                    toggle.setCheckable(True)
                    toggle.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
                    toggle.toggled.connect(section.setVisible)
                    toggle.toggled.connect(
                        lambda expanded, button=toggle: button.setArrowType(
                            Qt.ArrowType.DownArrow if expanded else Qt.ArrowType.RightArrow
                        )
                    )
                    self.form_layout.addWidget(toggle)
                    section.hide()
                groups[group], group_layouts[group] = section, section_layout
                self.form_layout.addWidget(section)
            editor = ParameterEditor(definition, value)
            editor.changed.connect(self._buttons)
            group_layouts[group].addWidget(editor)
            self.editors[key] = editor
        self.form_layout.addStretch()
        self.editor_algorithm = state.algorithm
        self.editor_mode = state.mode
        self.editor_specs = dict(profile)

    def receive_state(self, payload: object) -> None:
        """@brief 用真实快照更新界面，保留编辑草稿。@param payload 后台快照。"""
        state = cast(BoardSnapshot, payload)
        previous_haptic = (
            self.snapshot.connected
            and self.snapshot.mode == "HAPTIC"
            and self.snapshot.haptic_available
        )
        self.snapshot = state
        self.mode_badge.setText("演示数据" if self.demo else "只读模式" if state.read_only else "")
        self.mode_badge.setVisible(self.demo or state.read_only)
        if state.parameters and (
            self.editor_algorithm != state.algorithm
            or self.editor_mode != state.mode
            or not self.editors
            or (state.parameter_specs and self.editor_specs != state.parameter_specs)
        ):
            self._build_fields(state)
        else:
            for key, value in state.parameters.items():
                if key in self.editors and self.editors[key].current != value:
                    self.editors[key].set_current(value)
        self.algorithm_combo.blockSignals(True)
        choices = list(state.available_algorithms) if state.connected else list(ALGORITHMS)
        if state.algorithm not in choices:
            choices.append(state.algorithm)
        if [
            self.algorithm_combo.itemData(i) for i in range(self.algorithm_combo.count())
        ] != choices:
            self.algorithm_combo.clear()
            for key in choices:
                self.algorithm_combo.addItem(ALGORITHMS.get(key, key), key)
        index = self.algorithm_combo.findData(state.algorithm)
        if index >= 0:
            self.algorithm_combo.setCurrentIndex(index)
        self.algorithm_combo.blockSignals(False)
        self.control_mode_combo.blockSignals(True)
        modes = ["SPEED", "CURRENT"] if state.algorithm == "ENCODER_FOC" else ["SPEED"]
        if state.algorithm == "ENCODER_FOC" and state.position_available:
            modes.append("POSITION")
        if state.algorithm == "ENCODER_FOC" and state.haptic_available:
            modes.append("HAPTIC")
        if state.mode not in modes:
            modes.append(state.mode)
        if [
            self.control_mode_combo.itemData(i) for i in range(self.control_mode_combo.count())
        ] != modes:
            self.control_mode_combo.clear()
            for key in modes:
                self.control_mode_combo.addItem(MODES.get(key, key), key)
        self.control_mode_combo.setCurrentIndex(self.control_mode_combo.findData(state.mode))
        self.control_mode_combo.blockSignals(False)
        self.current_hint.setVisible(state.mode == "CURRENT")
        position_mode = state.mode == "POSITION"
        self.position_hint.setVisible(position_mode)
        self.tabs.setTabVisible(self.position_tab, position_mode)
        self.position_panel.show_sample(state.position if state.connected else None)
        self.tabs.setTabVisible(
            self.haptic_tab, state.haptic_available and state.algorithm == "ENCODER_FOC"
        )
        self.haptic_panel.show_state(state)
        if (
            state.connected
            and state.mode == "HAPTIC"
            and state.haptic_available
            and not previous_haptic
        ):
            self.tabs.setCurrentIndex(self.haptic_tab)
        self._sync_position_tolerance(state)
        self.plots.set_control_mode(state.mode)
        self.connection_badge.setText("已连接" if state.connected else state.busy or "未连接")
        self.connection_badge.setProperty(
            "tone", "success" if state.connected else "warning" if state.busy else ""
        )
        refresh_style(self.connection_badge)
        live = state.connected and state.run == "RUNNING"
        self.speed_metric.display(
            (
                f"{state.estimated_rpm:.3f}"
                if abs(state.estimated_rpm) < 100
                else f"{state.estimated_rpm:.1f}"
            )
            if state.estimated_rpm is not None
            else "—"
        )
        self.iq_metric.display(f"{state.iq_a:.3f}" if state.iq_a is not None else "—")
        self.id_metric.display(f"{state.id_a:.3f}" if state.id_a is not None else "—")
        self.bus_metric.display(f"{state.bus_v:.2f}" if state.bus_v is not None else "—")
        status = (
            "闭环运行"
            if live and state.closed
            else (
                "启动 / 运行"
                if live
                else (
                    "故障"
                    if state.run == "FAULT"
                    else "停机" if state.run == "STOPPED" else "状态未知"
                )
            )
        )
        # 完整进度保留在操作提示和状态栏，指标卡只显示短状态，避免长文裁切。
        self.run_metric.display(
            "标定中" if "标定" in state.busy else "处理中" if state.busy else status
        )
        self.run_metric.value.setToolTip(state.busy or status)
        for name, label in self.stage_labels.items():
            label.setVisible(state.algorithm not in ENCODER_ALGORITHMS or name in ("校准", "闭环"))
            active = live and name == state.stage
            tone = "success" if active and state.closed else "primary" if active else ""
            if label.property("tone") != tone:
                label.setProperty("tone", tone)
                refresh_style(label)
        prefix = "演示数据 · " if self.demo else "只读模式 · " if self.read_only else ""
        if state.algorithm not in FOC_ALGORITHMS and live:
            hint = prefix + "六步运行中实时信息受限；停机后读取统计。"
        elif state.data_time:
            hint = (
                prefix
                + ("当前数据" if live else "历史 / 最近读取")
                + ("" if state.mode == "HAPTIC" else " · " + state.data_time)
                + " · "
                + state.stage
            )
        else:
            hint = prefix + "尚无有效运行数据"
        self.data_hint.setText(hint)
        calibration = (
            "\n编码器标定："
            + ("本次上电有效" if state.encoder_calibration == "VALID" else "需执行七点标定")
            if state.encoder_available
            else ""
        )
        self.source_hint.setText(
            "控制来源："
            + (
                "上位机 REMOTE"
                if state.source == "REMOTE"
                else "LOCAL" if state.source else "尚未读取"
            )
            + calibration
        )
        if position_mode and state.source == "REMOTE":
            self.source_hint.setText(
                "上位机控制 · 标定有效"
                if state.encoder_calibration == "VALID"
                else "上位机控制 · 需七点标定"
            )
        if state.error:
            self.banner.setText(state.error)
            self.banner.show()
        elif state.fault and state.fault != "NONE":
            self.banner.setText("板卡故障：" + state.fault)
            self.banner.show()
        elif state.notices:
            self.banner.setText("兼容提示：" + "；".join(state.notices))
            self.banner.show()
        else:
            self.banner.hide()
        monitoring = (
            "日常监视 · 高速波形关闭"
            if state.wave_rate == 0
            else f"波形 {state.wave_rate / 1000:g} kHz"
        )
        self.statusBar().showMessage(
            f"{'演示连接' if self.demo else self.port_combo.currentText() or '未连接'} · "
            f"固件 {state.firmware or '—'} · 参数 {state.save or '—'} · {state.busy or state.stage} · {monitoring}"
        )
        self._buttons()

    def _buttons(self) -> None:
        """@brief 状态门禁和禁用原因集中更新，停止不受普通繁忙状态阻挡。"""
        state = self.snapshot
        # 低高度时省去重复标题和来源文字，优先留出完整目标输入框。
        compact = self.height() < 680
        self.control_heading.setVisible(not compact)
        self.source_hint.setVisible(not compact)
        active_worker = self.worker is not None and self.worker.isRunning()
        busy = bool(state.busy)
        dirty = any(editor.dirty for editor in self.editors.values())
        position_config_dirty = state.mode == "POSITION" and any(
            editor.dirty for key, editor in self.editors.items() if key != "TARGET"
        )
        invalid = False
        try:
            self.drafts()
        except ControlError:
            invalid = True
        idle = state.run == "STOPPED" and not busy and state.connected
        running_target = state.run == "RUNNING" and (
            state.algorithm not in FOC_ALGORITHMS or state.closed
        )
        writable = state.writable
        for key, editor in self.editors.items():
            editable = (
                writable
                and not busy
                and editor.definition.editable
                and (idle or (running_target and key == "TARGET"))
            )
            editor.edit.setReadOnly(not editable)
        self.algorithm_combo.setEnabled(idle and writable and not dirty)
        self.control_mode_combo.setEnabled(
            idle and writable and not dirty and state.algorithm == "ENCODER_FOC"
        )
        self.control_mode_combo.setToolTip(
            "模式切换前自动备份；要求停机且机械轴静止。旧固件不支持时保留原模式。"
        )
        self.port_combo.setEnabled(not active_worker)
        self.baud_combo.setEnabled(not active_worker)
        self.refresh_button.setEnabled(not active_worker)
        self.connect_button.setText("断开" if active_worker else "连接")
        self.connect_button.setEnabled(not self.awaiting_disconnect)
        self.stop_button.setEnabled(state.can_stop)
        self.apply_button.setEnabled(
            writable and not busy and dirty and not invalid and (idle or running_target)
        )
        self.apply_button.setText("应用参数并置零" if position_config_dirty else "应用参数")
        self.start_button.setText(
            "开启虚拟编码器" if state.mode == "HAPTIC" else "应用并启动" if dirty else "启动"
        )
        self.start_button.setEnabled(
            writable
            and idle
            and not invalid
            and state.ready
            and state.source == "REMOTE"
            and state.fault == "NONE"
            and (
                state.mode == "HAPTIC"
                or state.algorithm not in ENCODER_ALGORITHMS
                or state.encoder_calibration == "VALID"
            )
            and (state.mode != "POSITION" or state.position is not None and state.position.valid)
            and not position_config_dirty
        )
        self.calibrate_button.setVisible(state.encoder_available)
        self.calibrate_button.setText(
            "一键初始化（标定并置零）" if state.mode == "POSITION" else "应用参数并七点标定"
        )
        self.calibrate_button.setToolTip(
            "自动备份、按需七点标定、保存参数并建立机械零点；已有有效标定时跳过重复标定。"
            "以初始化完成时的轴位置为零点，保留目标草稿；可用顶部停止中断，完成后不自动启动。"
            if state.mode == "POSITION"
            else "自动备份、执行七点标定并应用参数草稿；可用顶部停止中断，完成后不自动启动。"
        )
        can_initialize = (
            writable
            and idle
            and not invalid
            and state.source == "REMOTE"
            and state.fault == "NONE"
            and (state.mode != "POSITION" or state.still is True)
        )
        self.calibrate_button.setEnabled(can_initialize)
        self.recalibrate_button.setEnabled(can_initialize and state.encoder_available)
        self.recalibrate_button.setText(
            "重新标定并置零" if state.mode == "POSITION" else "重新执行七点标定"
        )
        self.recalibrate_button.setToolTip(self.calibrate_button.toolTip())
        self.position_tolerance_combo.setEnabled(
            writable
            and idle
            and not dirty
            and state.mode == "POSITION"
            and state.position is not None
            and state.fault == "NONE"
        )
        self.position_settings_hint.setText(
            "容差写入板卡并保存；修改后若零点失效，点击左侧一键初始化。重新标定会短时通电，可用顶部停止中断。"
            if state.mode == "POSITION"
            else "到位容差需在位置模式停机设置，写入板卡并保存；重新标定会短时通电，可用顶部停止中断。"
        )
        self.discard_button.setEnabled(dirty and not busy)
        self.discard_button.setVisible(dirty)
        self.clear_button.setEnabled(
            writable and state.connected and not busy and state.run != "RUNNING"
        )
        self.clear_button.setVisible(state.run == "FAULT" or state.fault not in ("", "NONE"))
        self.remote_button.setVisible(
            state.connected and state.source != "REMOTE" and not self.read_only
        )
        self.remote_button.setEnabled(idle and writable)
        self.backup_button.setEnabled(idle and not dirty and state.control_available)
        self.copy_button.setEnabled(self.selected_preset is not None)
        self.preset_export_button.setEnabled(self.selected_preset is not None)
        self.restore_button.setEnabled(
            idle
            and writable
            and not dirty
            and self.selected_preset is not None
            and protocol_supported(self.selected_preset.protocol)
            and self.selected_preset.algorithm in state.available_algorithms
        )
        self.plots.export.setEnabled(
            bool(self.plots.points)
            or bool(self.plots.binary.buffer is not None and self.plots.binary.buffer.count)
        )
        self.plots.binary.link_baud = state.link_baud
        poles = state.parameters.get("POLE_PAIRS")
        if poles is not None and poles.value > 0:
            self.plots.binary.pole_pairs = poles.value
        if state.connected:
            self.remember_link_baud()
        self.draft_label.setText(
            "输入有误，请检查标记项"
            if invalid
            else (
                "未应用的参数草稿"
                if dirty
                else "当前参数已回读" if self.editors else "参数尚未读取"
            )
        )
        reason = (
            "只读模式禁止所有写命令"
            if state.read_only
            else (
                "请先连接板卡"
                if not state.connected
                else (
                    state.busy
                    if busy
                    else (
                        "；".join(state.notices) or "当前固件能力尚未适配"
                        if not state.control_available
                        else (
                            "编码器启动前需执行本次上电七点标定"
                            if state.algorithm in ENCODER_ALGORITHMS
                            and state.encoder_calibration != "VALID"
                            else (
                                "进入实际闭环后可调整目标"
                                if state.run == "RUNNING" and not running_target
                                else (
                                    (
                                        "运行中仅允许调整目标 Iq"
                                        if state.mode == "CURRENT"
                                        else (
                                            "运行中仅允许调整目标位置"
                                            if state.mode == "POSITION"
                                            else "运行中仅允许调整目标转速"
                                        )
                                    )
                                    if state.run == "RUNNING"
                                    else (
                                        "参数切换前请应用或撤销草稿"
                                        if dirty
                                        else "启动前确认参数保存；停止按钮始终位于顶部"
                                    )
                                )
                            )
                        )
                    )
                )
            )
        )
        if (
            writable
            and not busy
            and state.mode == "POSITION"
            and state.run == "STOPPED"
            and state.source == "REMOTE"
            and state.fault == "NONE"
        ):
            if state.still is not True:
                reason = "请等待机械轴静止后初始化"
            elif state.encoder_calibration != "VALID":
                reason = "设置参数后点击一键初始化，自动完成七点标定和机械置零"
            elif position_config_dirty:
                reason = "配置已修改，点击应用参数并置零；有效标定会自动复用"
            elif state.position is None or not state.position.valid:
                reason = "点击一键初始化建立机械零点；本次上电标定有效，将跳过重复标定"
            else:
                reason = "初始化就绪，可直接启动；下次仅改目标无需重复标定或置零"
        self.control_reason.setText(reason)
        self.apply_button.setToolTip(reason)
        self.start_button.setToolTip(reason)

    def receive_wave(self, payload: object) -> None:
        """@brief 更新共享高速缓存引用，绘制由30Hz定时器执行。@param payload 缓存。"""
        self.plots.binary.receive(cast(WaveBuffer, payload))

    def wave_baud_changed(self, _index: int) -> None:
        """@brief 保存下次启动档位；运行期间不立即切速。@param _index 选择。"""
        baud = int(self.plots.binary.baud.currentData())
        self.preferences["preferred_wave_baud"] = baud
        self._save_preferences()
        if self.worker is not None and self.worker.isRunning():
            self.worker.submit("wave_baud", baud)

    def _position_tolerance_changed(self, _index: int) -> None:
        """@brief 提交用户选择的新容差，实际值仍由板卡回读确认。@param _index 选择索引。"""
        if _index >= 0 and self.snapshot.connected and self.snapshot.mode == "POSITION":
            value = int(self.position_tolerance_combo.currentData())
            if (
                self.snapshot.position is None
                or self.snapshot.position.tolerance_deg != value / 1000
            ):
                self.submit("position_tolerance", value)

    def _sync_position_tolerance(self, state: BoardSnapshot) -> None:
        """@brief 显示最后实际回读容差，未知状态留空且不触发写入。@param state 板卡快照。"""
        self.position_tolerance_combo.blockSignals(True)
        self.position_tolerance_combo.setCurrentIndex(
            self.position_tolerance_combo.findData(round(state.position.tolerance_deg * 1000))
            if state.connected and state.position is not None
            else -1
        )
        self.position_tolerance_combo.blockSignals(False)

    def receive_points(self, payload: object) -> None:
        """@brief 批量接收绘图数据。@param payload 有效记录。"""
        self.plots.add_points(cast(list[PlotPoint], payload))

    def receive_logs(self, payload: object) -> None:
        """@brief 保留有界展示日志，原始会话日志在后台完整落盘。@param payload 日志批次。"""
        for kind, text in cast(list[tuple[str, str]], payload):
            self.log_records.append((timestamp(), kind, text))
        if self.log_panel.isVisible():
            self._render_logs()

    def _render_logs(self, _index: int = 0) -> None:
        """@brief 按类型显示最新记录。@param _index 筛选索引。"""
        selected = str(self.log_filter.currentData() or "")
        self.log_text.setPlainText(
            "\n".join(
                f"{stamp[11:23]} [{kind}] {text}"
                for stamp, kind, text in self.log_records
                if not selected or kind == selected
            )
        )
        bar = self.log_text.verticalScrollBar()
        bar.setValue(bar.maximum())

    def _toggle_logs(self, expanded: bool) -> None:
        """@brief 展开日志不隐藏停止或主操作。@param expanded 展开状态。"""
        self.log_panel.setVisible(expanded)
        self.log_toggle.setArrowType(
            Qt.ArrowType.DownArrow if expanded else Qt.ArrowType.RightArrow
        )
        if expanded:
            self._render_logs()

    def _tab_changed(self, _index: int) -> None:
        """@brief 切到设置页时补刷新日志（隐藏期间只累积不渲染）。@param _index 页签。"""
        if self.log_panel.isVisible():
            self._render_logs()

    def operation_done(self, action: str, result: object) -> None:
        """@brief 收到实际业务完成通知。@param action 操作。@param result 结果。"""
        if action in ("backup", "restore"):
            self.refresh_presets()
        if action == "backup" and isinstance(result, Path):
            self.preset_hint.setText("备份已保存：" + result.name)
        if action in ("apply", "start", "restore", "algorithm", "mode", "clear"):
            QTimer.singleShot(100, self._buttons)
        if action == "disconnect":
            self.awaiting_disconnect = False
            self.disconnect_confirmed = True
            # 线程合并快照可能把已回921600与断开同时发布，必须在完成通知同步界面。
            self.remember_link_baud()

    def remember_link_baud(self) -> None:
        """@brief 显示并缓存实测速率，不覆盖用户下次连接和采集的选择。"""
        baud = self.snapshot.link_baud
        self.baud_combo.blockSignals(True)
        self.baud_combo.setCurrentText(str(baud))
        self.baud_combo.blockSignals(False)
        if not self.demo and self.preferences.get("last_link_baud") != baud:
            self.preferences["last_link_baud"] = baud
            self._save_preferences()

    def operation_failed(self, action: str, message: str) -> None:
        """@brief 错误持续显示，断开失败由用户决定是否强制释放。@param action 操作。@param message 原因。"""
        self.show_message(message)
        if action == "position_tolerance":
            # 失败后撤回尚未确认的下拉选择，保留板卡最后实际回读值。
            self._sync_position_tolerance(self.snapshot)
        if action == "disconnect":
            self.awaiting_disconnect = False
            requested_close = self.close_intent
            self.close_intent = False
            answer = QMessageBox.warning(
                self,
                "停止未确认",
                message + "\n\n继续保留窗口，或明确强制关闭连接？强制关闭不能保证电机停机。",
                QMessageBox.StandardButton.Cancel | QMessageBox.StandardButton.Abort,
                QMessageBox.StandardButton.Cancel,
            )
            if answer == QMessageBox.StandardButton.Abort and self.worker is not None:
                self.force_confirmed = True
                self.close_intent = requested_close
                self.worker.force_close()
                if requested_close and not self.worker.isRunning():
                    QTimer.singleShot(0, self.close)
        self._buttons()

    def worker_finished(self) -> None:
        """@brief 线程结束后才允许关闭窗口，避免销毁正在运行的 QThread。"""
        self.awaiting_disconnect = False
        self.snapshot = replace(self.snapshot, connected=False, busy="")
        self._buttons()
        if self.close_intent:
            if self.disconnect_confirmed or self.force_confirmed:
                QTimer.singleShot(0, self.close)
            else:
                self.operation_failed(
                    "disconnect", self.snapshot.error or "连接中断，无法确认已停止"
                )

    def show_message(self, text: str) -> None:
        """@brief 持续反馈而不制造大量弹窗。@param text 内容。"""
        self.banner.setText(text)
        self.banner.show()

    def refresh_presets(self) -> None:
        """@brief 列出本地 JSON，不访问设备。"""
        selected = self.selected_preset.name if self.selected_preset else ""
        self.preset_list.clear()
        directory = self.data_dir / "presets"
        if directory.exists():
            for path in sorted(directory.glob("*.json")):
                item = QListWidgetItem(path.stem)
                item.setData(Qt.ItemDataRole.UserRole, str(path))
                self.preset_list.addItem(item)
                if path.stem == selected:
                    self.preset_list.setCurrentItem(item)
        self._buttons()

    def _preset_selected(
        self, current: QListWidgetItem | None, _previous: QListWidgetItem | None
    ) -> None:
        """@brief 校验所选本地文件。@param current 新项。@param _previous 旧项。"""
        self.selected_preset = None
        if current is not None:
            try:
                self.selected_preset = Preset.load(
                    Path(str(current.data(Qt.ItemDataRole.UserRole)))
                )
            except (OSError, ControlError) as error:
                self.preset_hint.setText(str(error))
        self.show_difference()
        self._buttons()

    def show_difference(self) -> None:
        """@brief 与最近板卡快照比较，隐藏算法不伪造数据。"""
        preset = self.selected_preset
        self.diff_table.setRowCount(0)
        if preset is None:
            return
        values = {**preset.parameters, **preset.readonly}
        profile = specs_for(preset.algorithm, values, preset.mode)
        self.diff_table.setRowCount(len(values))
        same_algorithm = (preset.algorithm, preset.mode) == (
            self.snapshot.algorithm,
            self.snapshot.mode,
        )
        for row, (key, value) in enumerate(values.items()):
            definition = profile[key]
            actual = self.snapshot.parameters.get(key) if same_algorithm else None
            current_text = (
                definition.format(actual) + " " + definition.suffix
                if actual
                else "当前算法/模式不同" if not same_algorithm else "尚未读取"
            )
            target_text = definition.format(value) + " " + definition.suffix
            for column, text in enumerate((definition.label, current_text, target_text)):
                item = QTableWidgetItem(text)
                if actual is not None and actual != value:
                    item.setBackground(QColor("#FEF3C7"))
                self.diff_table.setItem(row, column, item)
        self.preset_hint.setText(
            f"{ALGORITHMS[preset.algorithm]} · {MODES[preset.mode]} · FW={preset.firmware} · 来源 {preset.source}\n"
            "比较使用最近回读；恢复会重新读取、保留原配置并验证保存，且不会启动电机。"
        )

    def backup_preset(self) -> None:
        """@brief 命名备份当前真实配置。"""
        name, accepted = QInputDialog.getText(
            self, "备份当前参数", "预设名（英文字母、数字、下划线或连字符）："
        )
        if accepted:
            self.submit("backup", name.strip())

    def restore_preset(self) -> None:
        """@brief 恢复所选完整预设，修改前备份由后台完成。"""
        if self.selected_preset is not None:
            self.submit("restore", self.selected_preset)

    def import_preset(self) -> None:
        """@brief 校验后导入，不覆盖已有文件。"""
        filename, _filter = QFileDialog.getOpenFileName(
            self, "导入参数预设", "", "JSON 预设 (*.json)"
        )
        if filename:
            try:
                preset = Preset.load(Path(filename))
                preset.save(self.data_dir / "presets" / f"{preset.name}.json")
                self.refresh_presets()
            except (OSError, ControlError) as error:
                self.show_message(str(error))

    def copy_preset(self) -> None:
        """@brief 用本地表单复制修改，不发送 SET。"""
        preset = self.selected_preset
        if preset is None:
            return
        name, accepted = QInputDialog.getText(
            self, "复制预设", "新预设名：", text=preset.name + "_copy"
        )
        if not accepted:
            return
        dialog = QDialog(self)
        dialog.setWindowTitle("离线修改副本 · 不写入板卡")
        layout = QVBoxLayout(dialog)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        form = QWidget()
        form_layout = QVBoxLayout(form)
        editors: dict[str, ParameterEditor] = {}
        for key, value in preset.parameters.items():
            definition = specs_for(
                preset.algorithm, {**preset.readonly, **preset.parameters}, preset.mode
            )[key]
            if definition.editable:
                editor = ParameterEditor(definition, value)
                form_layout.addWidget(editor)
                editors[key] = editor
        scroll.setWidget(form)
        layout.addWidget(scroll)
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        dialog.resize(440, 620)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            try:
                copied = preset.copy(
                    name.strip(),
                    {key: editor.edit.text() for key, editor in editors.items() if editor.dirty},
                )
                copied.save(self.data_dir / "presets" / f"{copied.name}.json")
                self.refresh_presets()
            except (OSError, ControlError) as error:
                self.show_message(str(error))

    def export_preset(self) -> None:
        """@brief 导出完整兼容预设，已有文件不覆盖。"""
        if self.selected_preset is None:
            return
        filename, _filter = QFileDialog.getSaveFileName(
            self, "导出预设", self.selected_preset.name + ".json", "JSON (*.json)"
        )
        if filename:
            try:
                self.selected_preset.save(Path(filename))
            except (OSError, ControlError) as error:
                self.show_message(str(error))

    def export_waveform(self) -> None:
        """@brief 导出当前有界缓存，不补造均匀采样。"""
        filename, _filter = QFileDialog.getSaveFileName(
            self, "导出有效波形", "foc_waveform.csv", "CSV (*.csv)"
        )
        if filename:
            try:
                buffer = self.plots.binary.frozen or self.plots.binary.buffer
                if buffer is not None and buffer.count:
                    job = ExportJob(Path(filename), buffer)
                    self.wave_exports.append(job)
                    job.completed.connect(self.statusBar().showMessage)
                    job.failed.connect(self.show_message)
                    job.start()
                else:
                    export_csv(Path(filename), list(self.plots.points))
            except OSError as error:
                self.show_message(str(error))

    def export_logs(self) -> None:
        """@brief 导出展示记录；完整原文已存放于 logs。"""
        filename, _filter = QFileDialog.getSaveFileName(
            self, "导出展示日志", "foc_log.json", "JSON (*.json)"
        )
        if filename:
            try:
                Path(filename).write_text(
                    json.dumps(
                        [
                            {"time": stamp, "kind": kind, "text": text}
                            for stamp, kind, text in self.log_records
                        ],
                        ensure_ascii=False,
                        indent=2,
                    ),
                    encoding="utf-8",
                )
            except OSError as error:
                self.show_message(str(error))

    def show_diagnostics(self) -> None:
        """@brief 只显示已有停机快照，不在运行时发送完整诊断。"""
        dialog = QDialog(self)
        dialog.setWindowTitle("最近停机诊断原文")
        layout = QVBoxLayout(dialog)
        text = QPlainTextEdit()
        text.setReadOnly(True)
        text.setPlainText("\n\n".join(self.snapshot.diagnostics.values()) or "尚未读取停机诊断")
        layout.addWidget(text)
        dialog.resize(800, 480)
        dialog.exec()

    def resizeEvent(self, event: QResizeEvent) -> None:
        """@brief 按窗口高度收起重复提示，保留目标输入与固定主操作。@param event 窗口尺寸事件。"""
        super().resizeEvent(event)
        if hasattr(self, "position_settings_hint"):
            self._buttons()

    def closeEvent(self, event: QCloseEvent) -> None:
        """@brief 活跃连接先停机断开，完成后关闭。@param event 关闭事件。"""
        if self.worker is not None and self.worker.isRunning():
            event.ignore()
            self.close_intent = True
            if not self.awaiting_disconnect:
                self.awaiting_disconnect = True
                self.disconnect_confirmed = False
                self.submit("disconnect")
            return
        self._save_preferences()
        event.accept()
