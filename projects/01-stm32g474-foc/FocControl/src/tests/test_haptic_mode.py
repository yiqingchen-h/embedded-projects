"""双手感协议、原始计数、原子更新、初始化取消和实际Qt页面回归。"""

from __future__ import annotations

import asyncio
import os
from dataclasses import replace
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from foc_control import controller as module
from foc_control.controller import Controller
from foc_control.haptic_feedback import HAPTIC_KEYS, HapticSample, config_command
from foc_control.models import BoardSnapshot, ControlError, ParameterValue, specs_for
from foc_control.protocol import command_bytes, reply_matches
from foc_control.simulator import DemoTransport
from foc_control.storage import Preset
from foc_control.window import MainWindow
from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QLabel, QPushButton, QScrollArea


def response(counts: int = 0) -> str:
    """@brief 生成完整手感夹具。@param counts 原始多圈计数。@return 响应。"""
    return f"OK FOC HAPTIC KIND=DETENT DETENTS=24 DETENT_MA=100 DAMPING_UA_RPM=2000 LIMIT_MA=500 RUN_MS=60000 VALID=1 RUN=0 TRANSITION=0 SESSION=1 RAW=123456 TICKS=170000 POS_LO={counts & 0xFFFFFFFF} POS_HI={counts >> 32} CENTER_LO=0 CENTER_HI=0 INDEX_LO=0 INDEX_HI=0 REF_MA=0 IQ_MA=0 RPM_MILLI=0"


@pytest.mark.parametrize("counts", (-0x100000001, -4194304, 0, 4194304, 0x100000001))
def test_original_counts(counts: int) -> None:
    """@brief 64位双向原码和真实角度原样保留。@param counts 正负边界。"""
    sample = HapticSample.parse(response(counts))
    assert sample.counts == counts and sample.turns == counts / 2097152
    assert sample.angle_deg == 123456 * 360 / 2097152


@pytest.mark.parametrize(
    "original, invalid",
    (
        ("DETENTS=24", "DETENTS=1"),
        ("LIMIT_MA=500", "LIMIT_MA=801"),
        ("POS_HI=0", "POS_HI=2147483648"),
        ("VALID=1", "VALID=2"),
        ("DAMPING_UA_RPM=2000", "DAMPING_UA_RPM=10001"),
    ),
)
def test_invalid_snapshot(original: str, invalid: str) -> None:
    """@brief 快照越界不当成真实状态显示。@param original 字段。@param invalid 无效值。"""
    with pytest.raises(ControlError):
        HapticSample.parse(response().replace(original, invalid))


def test_protocol_and_position_speed() -> None:
    """@brief 只读允许快照禁止配置，位置速度复用原RPM字段。"""
    sample = HapticSample.parse(response())
    command = config_command(sample.parameters())
    assert command == "MOTOR FOC HAPTIC CONFIG DETENT 24 100 2000"
    assert reply_matches(command, response())
    assert command_bytes("MOTOR FOC HAPTIC", True)
    with pytest.raises(ControlError):
        command_bytes(command, True)
    speed = specs_for("ENCODER_FOC", mode="POSITION")["SPEED_LIMIT"]
    assert speed.label == "定位移动速度" and speed.group == "常用参数"
    assert speed.encode("60") == ParameterValue(60, 0, "RPM")


def test_enable_live_presets_and_independent_limits(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """@brief 一次备份按需标定后开启，在线原子配置、停机持久化和位置参数隔离。"""
    monkeypatch.setattr(module, "SAVE_WAIT", 0.002)

    async def scenario() -> None:
        """@brief 执行完整离线串口事务，不将模拟结果当成实测。"""
        port = DemoTransport("demo-haptic", "ENCODER_FOC", True, True, True, True)
        port.save_delay = 0.001
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            assert client.state.haptic_available and "START" not in port.commands
            await client.select_mode("POSITION")
            await client.apply(
                {
                    "CURRENT_LIMIT": ParameterValue(600, 3, "A"),
                    "SPEED_LIMIT": ParameterValue(30, 0, "RPM"),
                    "POSITION_KP": ParameterValue(16000, 3, "1/S"),
                }
            )
            await client.select_mode("HAPTIC")
            assert "TARGET" not in client.state.parameters
            assert client.state.parameters["CURRENT_LIMIT"].value == 500
            assert client.state.parameters["RUN_TIME"].value == 60000
            before = len(list((tmp_path / "restore_backups").glob("*.json")))
            port.calibration = "REQUIRED"
            await client.enable_haptic()
            assert port.run == "RUNNING" and port.commands.count("MOTOR ENCODER CAL") == 1
            assert len(list((tmp_path / "restore_backups").glob("*.json"))) == before + 1
            first_start = port.commands.index("START")
            assert "MOTOR ENCODER CAL" in port.commands[:first_start]
            changes = HapticSample.parse(response()).parameters()
            changes["HAPTIC_KIND"] = ParameterValue(1, 0, "")
            changes["HAPTIC_DETENTS"] = ParameterValue(256, 0, "")
            await client.apply(changes)
            assert port.commands[-2:] == [
                "MOTOR FOC HAPTIC CONFIG DAMPED 256 100 2000",
                "MOTOR FOC HAPTIC",
            ]
            assert client.state.haptic is not None and client.state.haptic.kind == "DAMPED"
            assert client.state.save == "PENDING" and port.run == "RUNNING"
            with pytest.raises(ControlError, match="运行中"):
                await client.apply({"CURRENT_LIMIT": ParameterValue(800, 3, "A")})
            await client.stop_motor()
            await client.saved()
            preset = await client.capture("haptic-config")
            preset.save(tmp_path / "haptic.json")
            restored = Preset.load(tmp_path / "haptic.json")
            assert set(HAPTIC_KEYS) <= restored.parameters.keys()
            assert not {"RAW", "SESSION", "RUN", "POS_LO", "TICKS"} & restored.parameters.keys()
            await client.apply(
                {
                    "HAPTIC_DETENTS": ParameterValue(2, 0, ""),
                    "CURRENT_LIMIT": ParameterValue(800, 3, "A"),
                }
            )
            await client.restore(preset)
            assert (
                port.haptic_detents == 256 and client.state.parameters["CURRENT_LIMIT"].value == 500
            )
            await client.select_mode("POSITION")
            assert client.state.parameters["CURRENT_LIMIT"].value == 600
            assert client.state.parameters["SPEED_LIMIT"].value == 30
            assert client.state.parameters["POSITION_KP"].value == 16000
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_cancel_enable_before_power(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """@brief 七点阶段取消后STOP，不继续保存恢复或追加普通START。"""
    monkeypatch.setattr(module, "SAVE_WAIT", 0.002)

    async def scenario() -> None:
        """@brief 原生异步取消实际串口事务链。"""
        port = DemoTransport("demo-haptic", "ENCODER_FOC", True, True, True, True)
        port.save_delay = 0.001
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            await client.select_mode("HAPTIC")
            port.calibration = "REQUIRED"
            task = asyncio.create_task(client.enable_haptic())
            for _ in range(1000):
                if "MOTOR ENCODER CAL" in port.commands:
                    break
                await asyncio.sleep(0.002)
            else:
                pytest.fail("未进入标定阶段")
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
            await client.stop_motor()
            assert "START" not in port.commands and port.run == "STOPPED"
        finally:
            await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize("width,height", ((1280, 800), (1000, 640), (900, 560)))
def test_qt_haptic_tab_and_controls(tmp_path: Path, width: int, height: int) -> None:
    """@brief 检查真实Qt控件、一次切页和旧固件隐藏。@param width 窗宽。@param height 窗高。"""
    app = QApplication.instance() or QApplication([])
    window = MainWindow(tmp_path)
    assert window.plots.binary.baud.currentData() == 0
    profile = specs_for("ENCODER_FOC", mode="HAPTIC")
    sample = HapticSample.parse(response())
    values = {key: ParameterValue(p.minimum, p.decimals, p.unit) for key, p in profile.items()}
    values.update(sample.parameters())
    values["RUN_TIME"] = ParameterValue(60000, 0, "MS")
    state = BoardSnapshot(
        connected=True,
        protocol="1.5",
        control_available=True,
        algorithm="ENCODER_FOC",
        mode="HAPTIC",
        source="REMOTE",
        run="STOPPED",
        fault="NONE",
        ready=True,
        still=True,
        output_disabled=True,
        save="SAVED",
        haptic_available=True,
        haptic=sample,
        parameters=values,
        parameter_specs=profile,
    )
    window.receive_state(state)
    window.resize(width, height)
    window.show()
    app.processEvents()
    assert window.tabs.currentIndex() == window.haptic_tab
    assert window.start_button.isEnabled()
    window.tabs.setCurrentWidget(window.plots)
    window.receive_state(replace(state, revision=1))
    assert window.tabs.currentWidget() is window.plots
    window.tabs.setCurrentIndex(window.haptic_tab)
    panel = window.haptic_panel
    assert panel.dial.angle == sample.angle_deg
    emitted: list[object] = []
    panel.configure.disconnect()
    panel.configure.connect(emitted.append)
    panel.detents.setValue(2)
    panel.detents.setValue(256)
    QTest.qWait(180)
    assert len(emitted) == 1
    assert panel.pending is not None and panel.pending["HAPTIC_DETENTS"].value == 256
    window.receive_state(
        replace(state, haptic=replace(sample, detents=256, transitioning=True), save="PENDING")
    )
    assert panel.status.text() == "切换中"
    window.receive_state(
        replace(state, haptic=replace(sample, detents=256, transitioning=False), save="PENDING")
    )
    assert "待停机保存" in panel.status.text()
    window.receive_state(replace(state, connected=False, haptic=None))
    assert panel.dial.angle is None and not window.start_button.isEnabled()
    window.receive_state(replace(state, haptic_available=False, mode="SPEED", parameters={}))
    assert not window.tabs.isTabVisible(window.haptic_tab)
    assert window.control_mode_combo.findData("HAPTIC") == -1
    window.close()


def haptic_ui_state(running: bool = False) -> BoardSnapshot:
    """@brief 建立UI快照，不访问板卡。@param running 是否模拟运行。@return 完整手感界面状态。"""
    sample = replace(
        HapticSample.parse(response()),
        strength_ma=120,
        detents=120,
        damping_ua_rpm=1200,
        running=running,
    )
    profile = specs_for("ENCODER_FOC", mode="HAPTIC")
    values = {key: ParameterValue(p.minimum, p.decimals, p.unit) for key, p in profile.items()}
    values.update(sample.parameters())
    values["RUN_TIME"] = ParameterValue(60000, 0, "MS")
    return BoardSnapshot(
        connected=True,
        protocol="1.5",
        control_available=True,
        algorithm="ENCODER_FOC",
        mode="HAPTIC",
        source="REMOTE",
        run="RUNNING" if running else "STOPPED",
        fault="NONE",
        ready=True,
        still=not running,
        output_disabled=not running,
        save="SAVED",
        haptic_available=True,
        haptic=sample,
        parameters=values,
        parameter_specs=profile,
        data_time="2026-10-25T11:00:00.000+08:00",
    )


@pytest.mark.parametrize("running", (False, True))
@pytest.mark.parametrize("field", ("strength", "detents", "damping"))
def test_haptic_typing_survives_refresh_and_apply(
    tmp_path: Path, running: bool, field: str
) -> None:
    """@brief 慢速从120编辑到160时，刷新不抢焦点、不提交半成品。@param tmp_path 隔离数据。
    @param running 停机/运行。@param field 数值控件。
    """
    app = QApplication.instance() or QApplication([])
    window = MainWindow(tmp_path)
    state = haptic_ui_state(running)
    window.receive_state(state)
    window.show()
    app.processEvents()
    panel = window.haptic_panel
    pending: list[dict[str, ParameterValue]] = []
    panel.configure.disconnect()
    panel.configure.connect(pending.append)
    control = getattr(panel, field)
    line = control.lineEdit()
    control.setFocus()
    line.setCursorPosition(2 if field == "damping" else 1)
    QTest.keyClick(line, Qt.Key.Key_Delete)
    assert line.text().startswith("1.00" if field == "damping" else "10")
    window.receive_state(replace(state, revision=1))
    QTest.qWait(250)
    assert not pending, "用户尚未确认，不能把中间值10提交到板卡"
    assert control.hasFocus() or line.hasFocus()
    QTest.keyClicks(line, "6")
    window.receive_state(replace(state, revision=2))
    QTest.qWait(220)
    expected_text = "1.600" if field == "damping" else "160"
    assert line.text().startswith(expected_text) and not pending
    QTest.keyClick(line, Qt.Key.Key_Return)
    QTest.qWait(200)
    key = {"strength": "HAPTIC_STRENGTH", "detents": "HAPTIC_DETENTS", "damping": "HAPTIC_DAMPING"}[
        field
    ]
    assert len(pending) == 1 and pending[0][key].value == (1600 if field == "damping" else 160)
    window.receive_state(replace(state, busy="应用参数", revision=3))
    assert control.isEnabled() and (control.hasFocus() or line.hasFocus())
    sample = replace(
        state.haptic,
        **(
            {"strength_ma": 160}
            if field == "strength"
            else {"damping_ua_rpm": 1600} if field == "damping" else {"detents": 160}
        ),
    )
    window.receive_state(
        replace(
            state, haptic=sample, parameters={**state.parameters, **sample.parameters()}, revision=4
        )
    )
    assert line.text().startswith(expected_text) and (control.hasFocus() or line.hasFocus())
    assert panel.pending is None
    window.receive_state(replace(state, connected=False, haptic=None))
    assert not control.isEnabled() and panel.pending is None and not panel.timer.isActive()
    window.close()


def test_new_haptic_draft_survives_previous_ack(tmp_path: Path) -> None:
    """@brief 前一配置尚在应用时反向修改，旧回读不能清掉最新草稿。@param tmp_path 隔离数据。"""
    app = QApplication.instance() or QApplication([])
    window = MainWindow(tmp_path)
    state = haptic_ui_state(True)
    window.receive_state(state)
    window.show()
    app.processEvents()
    panel = window.haptic_panel
    changes: list[dict[str, ParameterValue]] = []
    panel.configure.disconnect()
    panel.configure.connect(changes.append)
    panel.strength.setValue(160)
    QTest.qWait(200)
    assert len(changes) == 1 and changes[0]["HAPTIC_STRENGTH"].value == 160
    window.receive_state(replace(state, busy="应用参数"))
    panel.strength.setFocus()
    line = panel.strength.lineEdit()
    line.selectAll()
    QTest.keyClicks(line, "120")
    QTest.keyClick(line, Qt.Key.Key_Return)
    window.receive_state(replace(state, busy="应用参数", revision=1))
    assert panel.pending is not None and panel.pending["HAPTIC_STRENGTH"].value == 120
    first_ack = replace(state.haptic, strength_ma=160)
    window.receive_state(replace(state, haptic=first_ack, revision=2))
    QTest.qWait(250)
    assert len(changes) == 2 and changes[1]["HAPTIC_STRENGTH"].value == 120
    window.receive_state(replace(state, revision=3))
    assert panel.pending is None and (panel.strength.hasFocus() or line.hasFocus())
    window.close()


@pytest.mark.parametrize("width,height", ((1280, 800), (1000, 640), (900, 560)))
def test_haptic_compact_page_has_one_start_and_no_scroll(
    tmp_path: Path, width: int, height: int
) -> None:
    """@brief 三尺寸完整显示，唯一开启在左侧，不显示重复标题或时间戳。@param tmp_path 数据。
    @param width 窗宽。@param height 窗高。
    """
    app = QApplication.instance() or QApplication([])
    window = MainWindow(tmp_path)
    window.receive_state(haptic_ui_state())
    window.resize(width, height)
    window.show()
    app.processEvents()
    panel = window.haptic_panel
    starts = [
        button for button in window.findChildren(QPushButton) if button.text() == "开启虚拟编码器"
    ]
    assert starts == [window.start_button]
    assert not panel.findChildren(QScrollArea)
    assert not any(
        label.text() == "虚拟编码器" or "源时间戳" in label.text()
        for label in panel.findChildren(QLabel)
    )
    assert "2026-" not in window.data_hint.text()
    for control in (
        panel.detent,
        panel.damped,
        panel.detents,
        panel.strength,
        panel.damping,
        panel.confirmed,
    ):
        assert control.isVisible()
        assert panel.rect().contains(control.mapTo(panel, control.rect().topLeft()))
        assert panel.rect().contains(control.mapTo(panel, control.rect().bottomRight()))
        assert control.parentWidget().rect().contains(control.geometry())
    window.close()


def test_haptic_capability_arrives_after_mode(tmp_path: Path) -> None:
    """@brief MODE早于HAP1时仍只自动切页一次，普通刷新不抢回波形。"""
    app = QApplication.instance() or QApplication([])
    window = MainWindow(tmp_path)
    state = BoardSnapshot(connected=True, algorithm="ENCODER_FOC", mode="HAPTIC")
    window.receive_state(state)
    window.receive_state(replace(state, haptic_available=True))
    app.processEvents()
    assert window.tabs.currentIndex() == window.haptic_tab
    window.tabs.setCurrentWidget(window.plots)
    window.receive_state(replace(state, haptic_available=True, revision=2))
    assert window.tabs.currentWidget() is window.plots
    window.close()


@pytest.mark.parametrize("width,height", ((1280, 800), (1000, 640), (900, 560)))
def test_damped_page_only_shows_active_parameters(tmp_path: Path, width: int, height: int) -> None:
    """@brief 连续阻尼隐藏档位设置和刻度，保留完整四项协议及旧档位值。
    @param tmp_path 临时目录。@param width 窗宽。@param height 窗高。
    """
    app = QApplication.instance() or QApplication([])
    window = MainWindow(tmp_path)
    state = haptic_ui_state()
    assert state.haptic is not None
    state = replace(state, haptic=replace(state.haptic, kind="DAMPED"))
    window.receive_state(state)
    window.resize(width, height)
    window.show()
    app.processEvents()
    panel = window.haptic_panel
    assert panel.damping.isVisible() and not panel.detents.isVisible()
    assert not panel.strength.isVisible()
    assert not panel.index.isVisible() and not panel.center.isVisible()
    assert panel.dial.detents == 0
    assert "档" not in panel.confirmed.text()
    assert "转得越快" in panel.feel_hint.text()
    assert panel.values()["HAPTIC_DETENTS"].value == 120
    assert panel.values()["HAPTIC_STRENGTH"].value == 120
    for control in (panel.damping, panel.feel_hint, panel.confirmed, panel.status):
        assert control.isVisible()
        assert panel.rect().contains(control.mapTo(panel, control.rect().bottomRight()))
    window.close()


def test_kind_draft_updates_layout_before_old_ack(tmp_path: Path) -> None:
    """@brief 立即响应模式草稿，旧回读不显示错误设置，切回时原档位值保持。"""
    app = QApplication.instance() or QApplication([])
    window = MainWindow(tmp_path)
    state = haptic_ui_state()
    window.receive_state(state)
    window.show()
    app.processEvents()
    panel = window.haptic_panel
    panel.configure.disconnect()
    panel.damped.click()
    assert not panel.detents.isVisible() and panel.dial.detents == 0
    window.receive_state(replace(state, revision=2))
    assert not panel.detents.isVisible()
    assert panel.pending is not None
    panel.detent.click()
    assert panel.detents.isVisible() and panel.strength.isVisible()
    assert panel.detents.value() == 120 and panel.strength.value() == 120
    assert panel.dial.detents == 120
    window.close()
