"""真实数组绘图、触发与暂停交互回归，输入均标记为TEST。"""

from __future__ import annotations

import json
import os
import struct
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from foc_control.binary_wave import TEST, VALID, WaveBlock, WaveBuffer, export_binary_csv
from foc_control.protocol import PlotPoint, Telemetry
from foc_control.theme import install_theme
from foc_control.widgets import PlotPanel
from PySide6.QtWidgets import QApplication


@pytest.fixture(scope="module")
def app() -> QApplication:
    """@brief 创建隐藏的Qt应用。@return 应用。"""
    existing = QApplication.instance()
    if isinstance(existing, QApplication):
        return existing
    application = QApplication([])
    install_theme(application)
    return application


def fill(data: WaveBuffer, start: int = 0, count: int = 6000) -> None:
    """@brief 生成已标记的70Hz三相测试数据。@param data 缓存。@param start 源起点。@param count 数量。"""
    for first in range(start, start + count, 32):
        n = min(32, start + count - first)
        sample = np.arange(first, first + n)
        angle = sample * (2 * np.pi * 70 / 10000)
        values = np.zeros((n, 7), dtype="<i2")
        values[:, 0] = np.rint(0.020 * np.sin(angle) * 8192).astype(np.int16)
        values[:, 1] = np.rint(0.020 * np.sin(angle - 2 * np.pi / 3) * 8192).astype(np.int16)
        values[:, 2] = np.rint(0.020 * np.sin(angle + 2 * np.pi / 3) * 8192).astype(np.int16)
        values[:, 4] = values[:, 6] = 1024
        assert data.append(
            WaveBlock(1, TEST | VALID, 1, first + 1, first * 17000, 17000, n, values.tobytes())
        )
        for index in range(first, first + n):
            if index % 200 == 0:
                tick = index * 17000
                assert data.append(
                    WaveBlock(
                        2,
                        TEST | VALID,
                        1,
                        0,
                        tick,
                        3400000,
                        1,
                        struct.pack("<iiHBBI", 600000, 600000, 12400, 1, 6, tick),
                    )
                )


def panel_with_data(app: QApplication) -> tuple[PlotPanel, WaveBuffer]:
    """@brief 建立测试面板。@param app Qt。@return 面板与缓存。"""
    panel = PlotPanel()
    panel.resize(1000, 700)
    panel.show()
    data = WaveBuffer()
    fill(data)
    panel.binary.receive(data)
    panel.redraw()
    app.processEvents()
    return panel, data


def test_binary_axes_and_native_status_rate(app: QApplication) -> None:
    """@brief 相电流按5周期显示，转速保持50Hz源点。"""
    panel, data = panel_with_data(app)
    panel.view_combo.setCurrentIndex(1)
    panel.dirty = True
    panel.redraw()
    low, high = panel.speed_plot.viewRange()[0]
    assert high - low == pytest.approx(5 / 70)
    assert "有效 10.00 k点/s" in panel.notice.text()
    assert "通信测试数据" in panel.notice.text()
    assert len(panel.lines["rpm"].xData) <= 5
    assert len(panel.lines["ia"].xData) >= 700
    assert data.count == 6000
    assert not panel.channel_checks["isc"].isChecked()
    panel.close()


def test_binary_pause_zoom_and_resume(app: QApplication) -> None:
    """@brief 暂停后缩放只操作冻结副本，恢复跳到最新源数据。"""
    panel, data = panel_with_data(app)
    panel.toggle_pause()
    before = panel.speed_plot.viewRange()[0]
    fill(data, 6000, 2000)
    panel.binary.receive(data)
    panel.redraw()
    assert panel.speed_plot.viewRange()[0] == pytest.approx(before)
    assert panel.binary.frozen is not None and panel.binary.frozen.count == 6000
    panel.zoom(0.5)
    assert panel.speed_plot.viewRange()[0][1] <= 0.6
    panel.toggle_pause()
    panel.dirty = True
    panel.redraw()
    assert panel.speed_plot.viewRange()[0][1] == pytest.approx(0.7999)
    panel.close()


def test_trigger_and_missing_signal_wait(app: QApplication) -> None:
    """@brief 触发使用真实交越；缺少有效幅值时清空显示并等待。"""
    panel, _data = panel_with_data(app)
    panel.view_combo.setCurrentIndex(1)
    panel.binary.trigger.setCurrentIndex(1)
    panel.dirty = True
    panel.redraw()
    assert "触发锁定" in panel.notice.text()
    assert len(panel.lines["ia"].xData) > 600
    panel.binary.level.setValue(100)
    panel.dirty = True
    panel.redraw()
    assert "等待触发" in panel.notice.text()
    assert panel.lines["ia"].xData is None or len(panel.lines["ia"].xData) == 0
    panel.close()


def test_demo_restores_existing_manual_view(app: QApplication) -> None:
    """@brief 演示退出恢复手动范围与跟随状态。"""
    panel, _data = panel_with_data(app)
    panel.x_mode.setCurrentIndex(2)
    panel.x_span.setValue(120)
    panel.follow_button.setChecked(False)
    panel.binary.demo.setChecked(True)
    assert panel.view == "phase" and panel.follow
    panel.binary.demo.setChecked(False)
    assert panel.view == "dq" and not panel.follow
    assert panel.x_mode.currentData() == "manual"
    assert panel.x_span.value() == 120
    panel.close()


def test_filter_toggle_paused_keeps_raw_export_and_axes(app: QApplication, tmp_path: Path) -> None:
    """@brief 暂停切换滤波不改时轴、原始缓存或CSV，Ic重建关系保持。@param app Qt。@param tmp_path 输出。"""
    panel, data = panel_with_data(app)
    panel.view_combo.setCurrentIndex(1)
    before = data.current()
    panel.toggle_pause()
    panel.binary.trigger.setCurrentIndex(0)
    axis_before = panel.speed_plot.viewRange()[0]
    assert panel.filter_window_ms() == 2
    filtered_x = panel.lines["ia"].xData.copy()
    filtered_y = panel.lines["ia"].yData.copy()
    assert "电流显示滤波 2 ms" in panel.notice.text()
    np.testing.assert_allclose(
        panel.lines["ic"].yData,
        -panel.lines["ia"].yData - panel.lines["ib"].yData,
        atol=1e-13,
    )
    panel.filter_combo.setCurrentIndex(0)
    np.testing.assert_array_equal(panel.lines["ia"].xData, filtered_x)
    assert np.max(np.abs(filtered_y - panel.lines["ia"].yData)) > 1e-6
    assert "原始样点" in panel.notice.text()
    assert panel.speed_plot.viewRange()[0] == pytest.approx(axis_before)
    for actual, expected in zip(data.current(), before, strict=True):
        np.testing.assert_array_equal(actual, expected)
    panel.filter_combo.setCurrentIndex(3)
    path, _status, metadata = export_binary_csv(tmp_path / "raw.csv", data)
    exported = np.loadtxt(path, delimiter=",", skiprows=1, usecols=(2,), encoding="utf-8-sig")
    np.testing.assert_array_equal(np.rint(exported * 8192).astype(np.int16), before[1][:, 0])
    assert json.loads(metadata.read_text(encoding="utf-8"))["filtered"] is False
    panel.close()


def test_filter_preserves_gap_breaks_and_zoom_context(app: QApplication) -> None:
    """@brief 滤波不跨缺口连接，平移缩放后重叠内部样点保持一致。@param app Qt。"""
    panel, data = panel_with_data(app)
    data.mark_corrupt()
    fill(data, 6000, 1000)
    panel.binary.receive(data)
    panel.x_span.setValue(120)
    panel.x_mode.setCurrentIndex(2)
    panel.view_combo.setCurrentIndex(1)
    panel.redraw()
    assert np.any(~np.isfinite(panel.lines["ia"].yData))
    panel.toggle_pause()
    panel.follow_button.setChecked(False)
    panel.speed_plot.setXRange(0.620, 0.660, padding=0)
    panel.dirty = True
    panel.redraw()
    first_x = panel.lines["ia"].xData.copy()
    first_y = panel.lines["ia"].yData.copy()
    panel.speed_plot.setXRange(0.630, 0.650, padding=0)
    panel.dirty = True
    panel.redraw()
    indices = np.searchsorted(first_x, panel.lines["ia"].xData)
    np.testing.assert_allclose(panel.lines["ia"].yData, first_y[indices], atol=1e-13)
    panel.close()


def test_legacy_filter_does_not_change_speed_or_bus(app: QApplication) -> None:
    """@brief 旧文本电流也可滤波，低速状态与原始对象保持原值。@param app Qt。"""
    panel = PlotPanel()
    panel.resize(1000, 700)
    panel.show()
    points = [
        PlotPoint(
            index / 10000,
            "TEST",
            Telemetry(
                index,
                index * 17000,
                6,
                (-1) ** index * 20,
                0,
                0,
                0,
                rpm=600 + index % 2,
                bus_mv=12000 + index % 2,
            ),
            600,
        )
        for index in range(1000)
    ]
    panel.add_points(points)
    panel.view_combo.setCurrentIndex(1)
    app.processEvents()
    panel.redraw()
    assert np.std(panel.lines["ia"].yData[40:-40]) < 0.001
    rpm = panel.lines["rpm"].yData.copy()
    panel.filter_combo.setCurrentIndex(0)
    assert np.std(panel.lines["ia"].yData) > 0.019
    np.testing.assert_array_equal(panel.lines["rpm"].yData, rpm)
    panel.view_combo.setCurrentIndex(2)
    bus = panel.lines["bus"].yData.copy()
    panel.filter_combo.setCurrentIndex(3)
    np.testing.assert_array_equal(panel.lines["bus"].yData, bus)
    assert "电流显示滤波" not in panel.notice.text()
    assert list(panel.points) == points
    panel.close()
