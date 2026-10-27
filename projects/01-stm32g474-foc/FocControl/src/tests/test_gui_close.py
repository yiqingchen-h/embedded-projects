"""Qt 关闭门禁回归；只创建未启动的演示线程，不打开 COM。"""

from __future__ import annotations

import asyncio
import json
import math
import os
from dataclasses import replace
from pathlib import Path

import foc_control.window as window_module
import foc_control.worker as worker_module
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from foc_control.models import BoardSnapshot, ParameterValue, specs_for
from foc_control.protocol import PlotPoint, Telemetry
from foc_control.theme import install_theme
from foc_control.widgets import PlotPanel
from foc_control.window import MainWindow
from foc_control.worker import SerialWorker
from PySide6.QtWidgets import QApplication, QMessageBox


def test_connection_choices_persist_immediately(
    app: QApplication, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """@brief 采集档、COM和用户波特率选择后即落盘，重开不自动连接。@param app Qt应用。
    @param tmp_path 隔离数据。@param monkeypatch 替换端口枚举。
    """
    monkeypatch.setattr(window_module, "available_ports", lambda: ["COM3", "COM11"])
    window = MainWindow(tmp_path)
    window.port_combo.setCurrentText("COM11")
    window.baud_combo.setCurrentText("1000000")
    wave = window.plots.binary.baud
    wave.setCurrentIndex(wave.findData(921600))
    saved = json.loads((tmp_path / "settings.json").read_text(encoding="utf-8"))
    assert saved["port"] == "COM11" and saved["baud"] == "1000000"
    assert saved["preferred_wave_baud"] == 921600
    window.close()
    restored = MainWindow(tmp_path)
    assert restored.port_combo.currentText() == "COM11"
    assert restored.baud_combo.currentText() == "1000000"
    assert restored.plots.binary.baud.currentData() == 921600
    assert restored.worker is None and not restored.snapshot.connected
    restored.close()


def test_actual_baud_does_not_erase_connection_choice(
    app: QApplication, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """@brief 实际链路变成2M时保留用户初始921600选择和采集档。@param app Qt应用。
    @param tmp_path 隔离数据。@param monkeypatch 替换端口枚举。
    """
    monkeypatch.setattr(window_module, "available_ports", lambda: ["COM11"])
    window = MainWindow(tmp_path)
    window.baud_combo.setCurrentText("1000000")
    window.baud_combo.setCurrentText("921600")
    wave = window.plots.binary.baud
    wave.setCurrentIndex(wave.findData(921600))
    window.snapshot = replace(window.snapshot, link_baud=2000000)
    window.remember_link_baud()
    assert window.baud_combo.currentText() == "2000000"
    window.close()
    saved = json.loads((tmp_path / "settings.json").read_text(encoding="utf-8"))
    assert saved["baud"] == "921600" and saved["last_link_baud"] == 2000000
    assert saved["preferred_wave_baud"] == 921600


def test_unplugged_port_and_extra_settings_are_kept(
    app: QApplication, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """@brief 端口暂时不在枚举列表也不清空选择，未知偏好原样保留。@param app Qt应用。
    @param tmp_path 隔离数据。@param monkeypatch 模拟USB未连接。
    """
    saved = {"port": "COM11", "baud": "921600", "preferred_wave_baud": 921600, "custom": ["retain"]}
    (tmp_path / "settings.json").write_text(json.dumps(saved), encoding="utf-8")
    monkeypatch.setattr(window_module, "available_ports", lambda: [])
    window = MainWindow(tmp_path)
    assert window.port_combo.currentText() == "COM11"
    window.refresh_ports()
    assert window.port_combo.currentText() == "COM11"
    window.close()
    assert json.loads((tmp_path / "settings.json").read_text(encoding="utf-8")) == saved


@pytest.fixture(scope="module")
def app() -> QApplication:
    """@brief 创建隐藏平台的统一 Qt 应用。@return 应用。"""
    application = QApplication.instance()
    if not isinstance(application, QApplication):
        application = QApplication([])
    install_theme(application)
    return application


def waveform_points(count: int = 4000, offset: float = 0.0) -> list[PlotPoint]:
    """@brief 构造2kHz原始样点，含不应主导量程的隐藏通道。@param count 点数。
    @param offset 起始秒数。@return 保留原值的三相及速度测试记录。
    """
    return [
        PlotPoint(
            offset + index / 2000,
            "offline",
            Telemetry(
                index,
                index * 85000,
                6,
                20 * math.sin(2 * math.pi * 70 * index / 2000),
                20 * math.sin(2 * math.pi * 70 * index / 2000 - 2 * math.pi / 3),
                20 * math.sin(2 * math.pi * 70 * index / 2000 + 2 * math.pi / 3),
                100000,
                id_ma=0,
                iq_ma=125,
                ref_d_ma=0,
                ref_q_ma=125,
                rpm=600,
                bus_mv=12000,
            ),
            600,
        )
        for index in range(count)
    ]


def test_axes_adapt_to_actual_visible_data(app: QApplication) -> None:
    """@brief 全通道共用样点自适应，Y仅按可见通道。@param app Qt应用。"""
    panel = PlotPanel()
    panel.resize(1000, 700)
    panel.show()
    panel.view_combo.setCurrentIndex(1)
    points = waveform_points()
    panel.add_points(points)
    app.processEvents()
    panel.redraw()
    x_low, x_high = panel.speed_plot.getViewBox().viewRange()[0]
    y_low, y_high = panel.current_plot.getViewBox().viewRange()[1]
    assert 0.24 < x_high - x_low < 0.3
    assert -0.03 < y_low < -0.02 and 0.02 < y_high < 0.03
    assert not panel.channel_checks["isc"].isChecked()
    panel.x_mode.setCurrentIndex(1)
    assert panel.speed_plot.getViewBox().viewRange()[0][0] <= 0
    assert panel.speed_plot.getViewBox().viewRange()[0][1] >= points[-1].seconds
    panel.channel_checks["isc"].setChecked(True)
    assert panel.current_plot.getViewBox().viewRange()[1][1] >= 100
    panel.close()


def test_x_y_adjustments_are_independent(app: QApplication) -> None:
    """@brief X缩放不改变手动Y，Y缩放不改X或停止跟随。@param app Qt应用。"""
    panel = PlotPanel()
    panel.show()
    panel.add_points(waveform_points())
    panel.redraw()
    panel.y_low.setValue(-0.4)
    panel.y_high.setValue(0.4)
    panel.apply_y_range()
    assert panel.current_plot.getViewBox().viewRange()[1] == pytest.approx([-0.4, 0.4])
    before_x = panel.speed_plot.getViewBox().viewRange()[0]
    panel.zoom(0.5)
    after_x = panel.speed_plot.getViewBox().viewRange()[0]
    assert after_x[1] - after_x[0] < before_x[1] - before_x[0]
    assert panel.current_plot.getViewBox().viewRange()[1] == pytest.approx([-0.4, 0.4])
    panel.zoom_y(0.5)
    assert panel.current_plot.getViewBox().viewRange()[1] == pytest.approx([-0.2, 0.2])
    assert panel.speed_plot.getViewBox().viewRange()[0] == pytest.approx(after_x)
    assert panel.follow
    panel._manual_view(panel.current_plot, [False, True])
    assert panel.follow
    panel._manual_view(panel.current_plot, [True, False])
    assert panel.follow
    panel.close()


def test_mouse_x_zoom_keeps_live_follow(app: QApplication) -> None:
    """@brief 鼠标改变X量程后继续跟随新样点，显式关闭才查看历史。@param app Qt应用。"""
    panel = PlotPanel()
    panel.resize(1000, 700)
    panel.show()
    panel.add_points(waveform_points())
    app.processEvents()
    panel.redraw()
    panel.speed_plot.setXRange(1.9, 2.0, padding=0)
    panel.speed_plot.getViewBox().sigRangeChangedManually.emit([True, False])
    assert panel.follow
    panel.add_points(waveform_points(2000, 2.0))
    panel.redraw()
    low, high = panel.speed_plot.viewRange()[0]
    assert high == pytest.approx(2.9995)
    assert high - low == pytest.approx(0.1)
    panel.follow_button.click()
    assert not panel.follow and not panel.follow_button.isChecked()
    panel.add_points(waveform_points(2000, 3.0))
    panel.redraw()
    assert panel.speed_plot.viewRange()[0] == pytest.approx([low, high])
    assert "历史" in panel.notice.text()
    panel.follow_button.click()
    assert panel.follow and panel.speed_plot.viewRange()[0][1] == pytest.approx(3.9995)
    panel.close()


def test_paused_waveform_can_be_scaled_without_changing_samples(app: QApplication) -> None:
    """@brief 暂停冻结数据而非锁死交互，恢复后跟随最新缓存。@param app Qt应用。"""
    panel = PlotPanel()
    panel.show()
    original = waveform_points()
    panel.add_points(original)
    panel.redraw()
    panel.toggle_pause()
    panel.add_points(waveform_points(2000, 2.0))
    panel.zoom(0.5)
    assert panel.display_points[-1] is original[-1]
    assert len(panel.points) == 6000
    panel.y_low.setValue(1)
    panel.y_high.setValue(-1)
    prior = panel.current_plot.getViewBox().viewRange()[1]
    panel.apply_y_range()
    assert "下限" in panel.notice.text()
    assert panel.current_plot.getViewBox().viewRange()[1] == pytest.approx(prior)
    panel.toggle_pause()
    assert panel.display_points[-1].seconds > original[-1].seconds
    panel.reset_view()
    assert panel.x_mode.currentData() == "auto" and all(panel.y_auto.values())
    panel.close()


def test_link_loss_cancels_normal_close(
    app: QApplication,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """@brief 线程意外退出不等于停机，取消正常关闭。"""
    window = MainWindow(tmp_path, demo=True)
    window.worker = SerialWorker("DEMO", 921600, tmp_path, False, True)
    window.snapshot = BoardSnapshot(error="USB disconnected", run="UNKNOWN")
    window.close_intent = True
    window.show()
    monkeypatch.setattr(QMessageBox, "warning", lambda *_args: QMessageBox.StandardButton.Cancel)
    window.worker_finished()
    app.processEvents()
    assert window.isVisible() and not window.close_intent and not window.disconnect_confirmed
    assert "USB disconnected" in window.banner.text()
    window.close()


def test_force_close_requires_explicit_choice(
    app: QApplication,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """@brief 用户明确强制关闭后才能退出未知状态窗口。"""
    window = MainWindow(tmp_path, demo=True)
    window.worker = SerialWorker("DEMO", 921600, tmp_path, False, True)
    window.snapshot = BoardSnapshot(run="UNKNOWN")
    window.close_intent = True
    window.show()
    monkeypatch.setattr(QMessageBox, "warning", lambda *_args: QMessageBox.StandardButton.Abort)
    window.worker_finished()
    app.processEvents()
    assert window.force_confirmed and not window.isVisible()


def test_readonly_controls_and_drafts(app: QApplication, tmp_path: Path) -> None:
    """@brief 只读快照不能通过控件写入，草稿只保存在界面。"""
    window = MainWindow(tmp_path, read_only=True, demo=True)
    window.receive_state(
        BoardSnapshot(
            connected=True,
            firmware="1.4.6-serial",
            read_only=True,
            run="STOPPED",
            source="LOCAL",
            ready=True,
        )
    )
    assert not window.stop_button.isEnabled() and not window.start_button.isEnabled()
    assert not window.apply_button.isEnabled()
    window.close()


def test_current_encoder_ui(app: QApplication, tmp_path: Path) -> None:
    """@brief 当前有感后端显示小数速度和资格，版本号不禁用控制。@param app Qt 应用。
    @param tmp_path 截图目录。
    """
    window = MainWindow(tmp_path, demo=True)
    parameters = {
        "TARGET": ParameterValue(1, 0, "RPM"),
        "CURRENT_LIMIT": ParameterValue(500, 3, "A"),
        "RUN_TIME": ParameterValue(5000, 0, "MS"),
        "POLE_PAIRS": ParameterValue(7, 0, ""),
        "SPEED_LIMIT": ParameterValue(1800, 0, "RPM"),
    }
    state = BoardSnapshot(
        connected=True,
        firmware="1.6.21-g2208-encfoc",
        protocol="1.2",
        control_available=True,
        algorithm="ENCODER_FOC",
        available_algorithms=("ENCODER_FOC", "ENCODER_6STEP"),
        encoder_available=True,
        encoder_calibration="REQUIRED",
        source="REMOTE",
        run="STOPPED",
        fault="NONE",
        ready=True,
        estimated_rpm=1.012,
        parameters=parameters,
        parameter_specs={
            key: value
            for key, value in specs_for("ENCODER_FOC", parameters).items()
            if key in parameters
        },
    )
    window.receive_state(state)
    window.show()
    app.processEvents()
    assert window.stop_button.isEnabled() and window.calibrate_button.isEnabled()
    assert not window.empty_form.isVisible()
    assert window.speed_metric.value.text() == "1.012"
    assert not window.start_button.isEnabled()
    assert "七点标定" in window.control_reason.text()
    window.receive_state(replace(state, encoder_calibration="VALID"))
    assert window.start_button.isEnabled()
    assert window.editors["CURRENT_LIMIT"].definition.maximum == 800
    for width, height in ((1280, 800), (1000, 640)):
        window.resize(width, height)
        app.processEvents()
        assert window.calibrate_button.isVisible() and window.start_button.isVisible()
        assert window.grab().save(str(tmp_path / f"encoder_{width}x{height}.png"))
    window.close()


@pytest.mark.parametrize("read_only", (False, True))
def test_disconnect_before_identity_never_confirms_motor_stop(
    app: QApplication,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    read_only: bool,
) -> None:
    """@brief 身份响应尚未到达时，普通关闭必须报未确认；只读关闭仅释放链路。"""

    class WaitingIdentity:
        """仅保存命令且不回复 VERSION，不打开设备。"""

        def __init__(self) -> None:
            """@brief 建立收发记录。"""
            self.commands: list[str] = []
            self.closed = False

        def write(self, data: bytes) -> None:
            """@brief 保存原文。@param data 命令。"""
            self.commands.append(data.decode("ascii").strip())

        def read(self) -> bytes:
            """@brief 持续模拟等待身份回执。@return 空字节。"""
            return b""

        def close(self) -> None:
            """@brief 记录释放。"""
            self.closed = True

    transport = WaitingIdentity()
    monkeypatch.setattr(worker_module, "DemoTransport", lambda *_args, **_kwargs: transport)
    worker = SerialWorker("DEMO", 921600, tmp_path, read_only, True)
    completed: list[str] = []
    errors: list[tuple[str, str]] = []
    worker.operation_done.connect(lambda action, _result: completed.append(action))
    worker.operation_failed.connect(lambda action, message: errors.append((action, message)))

    async def run_disconnect() -> None:
        """@brief 在 VERSION 已发送但尚未收到回执时断开。"""
        task = asyncio.create_task(worker._main())

        async def wait_identity() -> None:
            """@brief 构造失败立即暴露，身份等待有界，避免测试自身无限挂起。"""
            while not transport.commands:
                if task.done():
                    await task
                    raise AssertionError("尚未发送VERSION，后台已退出")
                await asyncio.sleep(0.002)

        await asyncio.wait_for(wait_identity(), 1)
        worker.submit("disconnect")
        await asyncio.wait_for(task, 1)

    asyncio.run(run_disconnect())
    assert transport.closed and transport.commands == ["VERSION"]
    assert worker.latest.run == "UNKNOWN" and worker.latest.output_disabled is None
    assert "connect" not in completed
    if read_only:
        assert completed == ["disconnect"] and not errors
    else:
        assert not completed and len(errors) == 1
        assert errors[0][0] == "disconnect" and "未确认停机" in errors[0][1]
