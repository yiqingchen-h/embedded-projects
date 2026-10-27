"""浅色工程面板的统一颜色、字形和交互状态。"""

from __future__ import annotations

import os
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QFontDatabase, QIcon, QPainter, QPixmap
from PySide6.QtSvg import QSvgRenderer
from PySide6.QtWidgets import QApplication

# 操作语义与曲线颜色集中定义，组件不自行发明颜色。
COLORS = {
    "background": "#F5F7FB",
    "card": "#FFFFFF",
    "text": "#0F172A",
    "muted": "#475569",
    "primary": "#2563EB",
    "success": "#15803D",
    "warning": "#B45309",
    "danger": "#B91C1C",
    "border": "#E2E8F0",
    "input_border": "#64748B",
    "reference": "#64748B",
}
STYLE = """
QWidget { font-family: "Microsoft YaHei"; font-size: 16px; color: #0F172A; }
QMainWindow, QWidget#root { background: #F5F7FB; }
QFrame#toolbar, QFrame#card, QFrame#controls, QFrame#metric { background: white; border: 1px solid #E2E8F0; border-radius: 6px; }
QFrame#toolbar { border-radius: 0; border-bottom: 1px solid #E2E8F0; }
QLabel#brand { font-size: 20px; font-weight: 700; }
QLabel#heading { font-size: 18px; font-weight: 600; }
QLabel#muted, QLabel#hint { color: #475569; font-size: 14px; }
QLabel#value { font-family: "Consolas"; font-size: 28px; font-weight: 600; }
QLabel#error { color: #B91C1C; font-size: 14px; }
QLabel#badge { border-radius: 4px; padding: 6px 10px; font-size: 14px; background: #F1F5F9; color: #475569; }
QLabel#badge[tone="success"] { background: #DCFCE7; color: #166534; }
QLabel#badge[tone="warning"] { background: #FEF3C7; color: #92400E; }
QLabel#badge[tone="danger"] { background: #FEE2E2; color: #991B1B; }
QLabel#badge[tone="primary"] { background: #DBEAFE; color: #1E40AF; }
QLabel#banner { border: 1px solid #FCA5A5; background: #FEF2F2; color: #991B1B; border-radius: 6px; padding: 8px; }
QLineEdit, QComboBox { background: white; border: 1px solid #64748B; border-radius: 6px; padding: 5px 8px; min-height: 24px; }
QLineEdit { font-family: "Consolas"; }
QLineEdit:focus, QComboBox:focus, QPushButton:focus, QToolButton:focus { border: 2px solid #2563EB; }
QLineEdit[invalid="true"] { border: 2px solid #B91C1C; }
QLineEdit[draft="true"] { background: #FFFBEB; }
QLineEdit:read-only, QComboBox:disabled { background: #F1F5F9; color: #475569; }
QComboBox::drop-down { border: none; width: 20px; }
QPushButton, QToolButton { background: white; border: 1px solid #64748B; border-radius: 6px; padding: 7px 12px; min-height: 22px; }
QPushButton:hover, QToolButton:hover { background: #EFF6FF; border-color: #2563EB; }
QPushButton:pressed, QToolButton:pressed { background: #DBEAFE; }
QPushButton[role="feel"]:checked { background: #DBEAFE; border: 2px solid #2563EB; color: #1D4ED8; font-weight: 600; }
QWidget#hapticPanel QSpinBox, QWidget#hapticPanel QDoubleSpinBox { background: white; border: 1px solid #64748B; border-radius: 6px; padding: 4px 8px; min-height: 26px; }
QPushButton[role="primary"] { background: #2563EB; border-color: #2563EB; color: white; font-weight: 600; }
QPushButton[role="primary"]:hover { background: #1D4ED8; }
QPushButton[role="danger"] { background: #B91C1C; color: white; border-color: #B91C1C; font-weight: 600; }
QPushButton[role="danger"]:hover { background: #991B1B; }
QPushButton:disabled, QToolButton:disabled { background: #E2E8F0; border-color: #CBD5E1; color: #475569; }
QCheckBox { spacing: 6px; font-size: 14px; }
QCheckBox::indicator { width: 16px; height: 16px; }
QTabWidget::pane { border: none; background: white; }
QTabBar::tab { padding: 10px 18px; color: #475569; border-bottom: 2px solid transparent; }
QTabBar::tab:selected { color: #1D4ED8; border-bottom: 2px solid #2563EB; font-weight: 600; }
QTabBar::tab:focus { border: 2px solid #2563EB; }
QScrollArea { border: none; background: white; }
QScrollArea > QWidget > QWidget { background: white; }
QGroupBox { border: 1px solid #E2E8F0; border-radius: 6px; margin-top: 12px; padding-top: 12px; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
QSplitter::handle { background: #E2E8F0; }
QSplitter::handle:hover { background: #93C5FD; }
QTableWidget, QListWidget, QPlainTextEdit { background: white; border: 1px solid #E2E8F0; border-radius: 6px; selection-background-color: #DBEAFE; selection-color: #0F172A; }
QHeaderView::section { background: #F1F5F9; border: none; padding: 8px; font-size: 14px; }
QTableWidget { gridline-color: #E2E8F0; font-size: 14px; }
QTableWidget::item { padding: 6px; }
QPlainTextEdit { font-family: "Consolas"; font-size: 14px; padding: 6px; }
QStatusBar { color: #475569; background: #F5F7FB; font-size: 13px; }
QToolTip { background: #0F172A; color: white; border: none; padding: 6px; }
"""


def install_theme(app: QApplication) -> None:
    """@brief 配置统一 Qt Fusion 风格。@param app 应用。"""
    app.setStyle("Fusion")
    # Windows 的 offscreen 平台不自动枚举字体；验收时显式注册系统字体，不打包或安装字体。
    if app.platformName() == "offscreen":
        directory = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
        for name in ("msyh.ttc", "segoeui.ttf", "consola.ttf"):
            path = directory / name
            if path.exists():
                QFontDatabase.addApplicationFont(str(path))
    font = QFont("Microsoft YaHei")
    font.setPixelSize(16)
    app.setFont(font)
    app.setStyleSheet(STYLE)


def icon(name: str, color: str = "#475569") -> QIcon:
    """@brief 创建统一线性 SVG 图标。@param name 图标名。@param color 线色。@return 图标。"""
    shapes = {
        "stop": '<rect x="6" y="6" width="12" height="12" rx="1"/>',
        "refresh": '<path d="M20 8a8 8 0 1 0 1 7M20 3v5h-5"/>',
        "play": '<path d="m8 5 11 7-11 7Z"/>',
        "save": '<path d="M5 3h12l4 4v14H3V3h2M7 3v6h10V3M7 21v-7h10v7"/>',
        "link": '<path d="m10 14 4-4M8 16l-1 1a4 4 0 0 1-6-6l4-4a4 4 0 0 1 6 0M16 8l1-1a4 4 0 0 1 6 6l-4 4a4 4 0 0 1-6 0"/>',
    }
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24">'
        f'<g fill="none" stroke="{color}" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">'
        + shapes.get(name, shapes["save"])
        + "</g></svg>"
    )
    renderer = QSvgRenderer(svg.encode())
    pixmap = QPixmap(48, 48)
    pixmap.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pixmap)
    renderer.render(painter)
    painter.end()
    return QIcon(pixmap)
