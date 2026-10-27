"""独立Iq的单位、切换、预设、标定及界面回归，全部为离线协议数据。"""

from __future__ import annotations

import asyncio
import json
import os
import struct
from dataclasses import replace
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from foc_control import controller as module
from foc_control.binary_wave import VALID, WaveBlock
from foc_control.controller import Controller
from foc_control.models import ControlError, ParameterValue, specs_for
from foc_control.simulator import DemoTransport
from foc_control.storage import Preset
from foc_control.window import MainWindow
from PySide6.QtWidgets import QApplication


class IqPort(DemoTransport):
    """仅模拟1.8模式契约，可注入错误回执。"""

    def __init__(self, overrides: dict[str, str] | None = None) -> None:
        """@brief 创建有感模拟板。@param overrides 指定命令回执。"""
        super().__init__("1.8.0-g2208-iq", "ENCODER_FOC", g2208=True, current_support=True)
        self.overrides = overrides or {}

    def write(self, data: bytes) -> None:
        """@brief 注入原始回执，其余使用演示状态机。@param data 命令。"""
        command = data.decode("ascii").strip()
        if command in self.overrides:
            self.commands.append(command)
            self.emit(self.overrides[command])
        else:
            super().write(data)


@pytest.fixture(autouse=True)
def fast_save(monkeypatch: pytest.MonkeyPatch) -> None:
    """@brief 仅缩短模拟Flash静默窗口。@param monkeypatch 测试替身。"""
    monkeypatch.setattr(module, "SAVE_WAIT", 0.03)


def test_mode_switch_backs_up_and_retains_both_targets(tmp_path: Path) -> None:
    """@brief 模式切换保留独立目标，JSON备份发生在SET之前。@param tmp_path 临时目录。"""

    async def scenario() -> None:
        port = IqPort()
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            await client.select_mode("CURRENT")
            assert client.state.mode == "CURRENT"
            assert client.state.parameters["TARGET"] == ParameterValue(0, 3, "A")
            assert not client.state.parameter_specs["CURRENT_LIMIT"].editable
            assert not {"RAMP", "SPEED_KP", "SPEED_KI"} & client.state.parameters.keys()
            backups = list((tmp_path / "restore_backups").glob("*.json"))
            assert len(backups) == 1 and Preset.load(backups[0]).mode == "SPEED"
            await client.apply({"TARGET": ParameterValue(-123, 3, "A")})
            await client.select_mode("SPEED")
            assert client.state.parameters["TARGET"] == ParameterValue(600, 0, "RPM")
            await client.apply({"TARGET": ParameterValue(321, 0, "RPM")})
            await client.select_mode("CURRENT")
            assert client.state.parameters["TARGET"] == ParameterValue(-123, 3, "A")
            await client.select_mode("SPEED")
            assert client.state.parameters["TARGET"].value == 321
            assert "START" not in port.commands and "CLEAR" not in port.commands
        finally:
            await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize("target", (-500, -20, 0, 20, 500))
def test_initial_current_connection_and_live_signed_target(tmp_path: Path, target: int) -> None:
    """@brief 重连CURRENT及在线正负零使用毫安，零目标仍保持RUNNING。@param tmp_path 临时目录。@param target 毫安。"""

    async def scenario() -> None:
        port = IqPort()
        port.mode = "CURRENT"
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            assert client.state.writable
            assert "SET MODE SPEED" not in port.commands and "START" not in port.commands
            await client.start_motor({})
            await client.read_drive()
            assert client.state.closed
            await client.apply({"TARGET": ParameterValue(target, 3, "A")})
            assert f"SET TARGET {target}" in port.commands
            assert port.run == "RUNNING" and client.state.output_disabled is False
            before = list(port.commands)
            with pytest.raises(ControlError, match="停止"):
                await client.select_mode("SPEED")
            assert "SET MODE SPEED" not in port.commands[len(before) :]
            await client.stop_motor()
            assert client.state.output_disabled
            await client.poll()
            assert client.state.parameters["TARGET"] == ParameterValue(target, 3, "A")
        finally:
            await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize("text", ("-0.501", "0.501", "NaN", "Infinity", "0.0001", "600"))
def test_iq_editor_rejects_invalid_amps(text: str) -> None:
    """@brief 越界、非有限和精度不能进入毫安协议。@param text 输入文本。"""
    with pytest.raises(ControlError):
        specs_for("ENCODER_FOC", mode="CURRENT")["TARGET"].encode(text)


@pytest.mark.parametrize(
    "value",
    (ParameterValue(-501, 3, "A"), ParameterValue(501, 3, "A"), ParameterValue(1, 0, "RPM")),
)
def test_controller_blocks_invalid_current_before_set(
    tmp_path: Path, value: ParameterValue
) -> None:
    """@brief 绕过界面仍不能越过单位与0.5A约束。@param tmp_path 临时目录。@param value 非法值。"""

    async def scenario() -> None:
        port = IqPort()
        port.mode = "CURRENT"
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            offset = len(port.commands)
            with pytest.raises(ControlError):
                await client.apply({"TARGET": value})
            with pytest.raises(ControlError, match="不允许编辑"):
                await client.apply({"CURRENT_LIMIT": ParameterValue(600, 3, "A")})
            assert not any(c.startswith("SET ") for c in port.commands[offset:])
        finally:
            await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "override,available",
    (
        ("OK GET TARGET=0 DECIMALS=3 UNIT=A MIN=-2000 MAX=2000 EDITABLE=1", True),
        ("OK GET TARGET=0 DECIMALS=0 UNIT=RPM MIN=0 MAX=1800 EDITABLE=1", False),
        ("OK GET TARGET=0 DECIMALS=3 UNIT=A MIN=-500 MAX=500 EDITABLE=0", False),
    ),
)
def test_current_metadata_never_expands_half_amp(
    tmp_path: Path, override: str, available: bool
) -> None:
    """@brief 元数据不能扩流或掩盖单位变化。@param tmp_path 临时目录。@param override 回执。@param available 是否可控。"""

    async def scenario() -> None:
        port = IqPort({"GET TARGET": override})
        port.mode = "CURRENT"
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            assert client.state.control_available is available
            if available:
                definition = client.state.parameter_specs["TARGET"]
                assert (definition.minimum, definition.maximum) == (-500, 500)
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_wrong_fixed_limit_is_monitor_only(tmp_path: Path) -> None:
    """@brief 电流模式实际限流不是0.5A时不能开放控制。@param tmp_path 临时目录。"""

    async def scenario() -> None:
        port = IqPort({"GET CURRENT_LIMIT": "OK GET CURRENT_LIMIT=800 DECIMALS=3 UNIT=A"})
        port.mode = "CURRENT"
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            assert not client.state.writable and client.state.can_stop
            assert not any(c.startswith("SET ") for c in port.commands)
        finally:
            await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize("still", ("0", "missing"))
def test_switch_needs_actual_still(tmp_path: Path, still: str) -> None:
    """@brief 输出关闭不等于机械静止，缺失实测静止不能切换。@param tmp_path 临时目录。@param still 静止字段。"""

    async def scenario() -> None:
        suffix = "" if still == "missing" else " STILL=0"
        port = IqPort({"MOTOR FOC REST": "OK MOTOR FOC REST OUTPUT_DISABLED=1 FAULT=NONE" + suffix})
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            with pytest.raises(ControlError, match="静止"):
                await client.select_mode("CURRENT")
            assert "SET MODE CURRENT" not in port.commands
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_old_firmware_mode_rejection_keeps_speed(tmp_path: Path) -> None:
    """@brief 老固件拒绝CURRENT仍保持连接和原速度参数，不自动试写。@param tmp_path 临时目录。"""

    async def scenario() -> None:
        port = IqPort({"SET MODE CURRENT": "ERR SET NOT_READY"})
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            assert "SET MODE CURRENT" not in port.commands
            with pytest.raises(ControlError, match="NOT_READY"):
                await client.select_mode("CURRENT")
            assert client.state.connected and client.state.writable
            assert client.state.mode == "SPEED" and client.state.parameters["TARGET"].unit == "RPM"
            assert port.commands.count("SET MODE CURRENT") == 1
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_external_mode_change_blocks_stale_rpm_draft(tmp_path: Path) -> None:
    """@brief 外部切换真实模式后，旧RPM草稿不能写成毫安。@param tmp_path 临时目录。"""

    async def scenario() -> None:
        port = IqPort()
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            port.mode = "CURRENT"
            offset = len(port.commands)
            with pytest.raises(ControlError, match="模式已变化"):
                await client.apply({"TARGET": ParameterValue(100, 0, "RPM")})
            assert not any(c.startswith("SET TARGET") for c in port.commands[offset:])
            assert client.state.mode == "CURRENT" and client.state.parameters["TARGET"].unit == "A"
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_current_presets_and_calibration_restore_mode(tmp_path: Path) -> None:
    """@brief 正负预设跨模式恢复，并在重标后返回CURRENT保留目标。@param tmp_path 临时目录。"""

    async def scenario() -> None:
        port = IqPort()
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            speed = await client.capture("speed")
            await client.select_mode("CURRENT")
            current = (await client.capture("iq")).copy("negative", {"TARGET": "-0.020"})
            path = tmp_path / "negative.json"
            current.save(path)
            assert Preset.load(path) == current
            await client.restore(current)
            assert port.iq_target == -20
            await client.restore(speed)
            assert client.state.mode == "SPEED" and port.parameters["TARGET"].value == 600
            await client.restore(current)
            assert client.state.mode == "CURRENT" and port.parameters["TARGET"].value == -20
            port.calibration = "REQUIRED"
            await client.calibrate_encoder()
            assert port.calibration == "VALID" and client.state.mode == "CURRENT"
            assert client.state.parameters["TARGET"] == ParameterValue(-20, 3, "A")
            assert port.run == "STOPPED" and "START" not in port.commands
            with pytest.raises(ControlError):
                replace(
                    current,
                    parameters={**current.parameters, "TARGET": ParameterValue(501, 3, "A")},
                ).validate()
            with pytest.raises(ControlError):
                replace(current, algorithm="SENSORLESS_FOC").validate()
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_stop_cancels_mode_save_without_stale_control(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """@brief SET后等待保存仍可优先STOP，随后从真实模式恢复界面。@param tmp_path 临时目录。@param monkeypatch 测试替身。"""

    async def scenario() -> None:
        port = IqPort()
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            original = client.saved
            entered = asyncio.Event()

            async def slow_saved(changed: bool = False) -> None:
                """@brief 仅在模式已写入后制造可取消等待。@param changed 是否更改。"""
                if client.state.mode == "CURRENT":
                    entered.set()
                    await asyncio.sleep(10)
                await original(changed)

            monkeypatch.setattr(client, "saved", slow_saved)
            task = asyncio.create_task(client.select_mode("CURRENT"))
            await asyncio.wait_for(entered.wait(), 2)
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
            await asyncio.wait_for(client.stop_motor(), 0.5)
            assert client.state.output_disabled and port.run == "STOPPED"
            await client.poll()
            assert client.state.mode == "CURRENT" and client.state.writable
            assert client.state.parameters["TARGET"].unit == "A"
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_binary_actual_dq_and_current_gui(tmp_path: Path) -> None:
    """@brief 真实采样通道和参考分开，模式切换清旧草稿和RPM目标图例。@param tmp_path 临时目录。"""
    application = QApplication.instance()
    if not isinstance(application, QApplication):
        application = QApplication([])
    port = IqPort()
    port.mode = "CURRENT"
    client = Controller(port, tmp_path)

    async def initialize() -> None:
        """@brief 只读取演示CURRENT。"""
        await client.initialize()
        await client.session.close()

    asyncio.run(initialize())
    raw = np.zeros((32, 7), dtype="<i2")
    raw[:, 3], raw[:, 4], raw[:, 6] = 82, -164, -246
    block = WaveBlock(1, VALID, 9, 1, 17000, 17000, 32, raw.tobytes())
    client._binary(block)
    client._binary(
        WaveBlock(
            2, VALID, 9, 0, 17000, 3400000, 1, struct.pack("<iiHBBI", -20000, 0, 12000, 1, 6, 17000)
        )
    )
    assert client.state.id_a == pytest.approx(82 / 8192)
    assert client.state.iq_a == pytest.approx(-164 / 8192)
    window = MainWindow(tmp_path / "gui", demo=True)
    window.receive_state(client.state)
    window.receive_wave(client.wave_buffer)
    window.show()
    application.processEvents()
    assert window.control_mode_combo.currentData() == "CURRENT"
    assert window.current_hint.isVisible() and "Iq=0" in window.current_hint.text()
    assert window.editors["TARGET"].definition.suffix == "A"
    assert (
        "target" not in window.plots.lines and {"id", "iq", "rd", "rq"} <= window.plots.lines.keys()
    )
    window.plots.filter_combo.setCurrentIndex(0)
    window.plots.dirty = True
    window.plots.redraw()
    assert np.allclose(window.plots.lines["iq"].yData, raw[:, 4] / 8192)
    assert np.allclose(window.plots.lines["rq"].yData, raw[:, 6] / 8192)
    window.resize(1000, 640)
    application.processEvents()
    assert window.plots.compact and window.plots.current_plot.isVisible()
    assert not window.plots.speed_plot.isVisible()
    window.editors["TARGET"].edit.setText("-0.030")
    speed_values = dict(port.blocks["ENCODER_FOC"])
    speed = replace(
        client.state,
        mode="SPEED",
        parameters=speed_values,
        parameter_specs=specs_for("ENCODER_FOC", speed_values),
    )
    window.receive_state(speed)
    assert (
        window.editors["TARGET"].definition.suffix == "RPM" and not window.editors["TARGET"].dirty
    )
    assert "target" in window.plots.lines
    assert np.array_equal(client.wave_buffer.current()[1], raw)
    window.close()


def test_disconnect_baud_syncs_coalesced_snapshot(tmp_path: Path) -> None:
    """@brief 断开和921600合并发布时也同步下次连接，不依赖中间的connected帧。@param tmp_path 数据目录。"""
    application = QApplication.instance()
    if not isinstance(application, QApplication):
        application = QApplication([])
    window = MainWindow(tmp_path)
    live = replace(
        window.snapshot,
        connected=True,
        firmware="1.8.0-g2208-iq",
        protocol="1.3",
        run="STOPPED",
        output_disabled=True,
        link_baud=2000000,
    )
    window.receive_state(live)
    assert window.baud_combo.currentText() == "2000000"
    window.receive_state(replace(live, connected=False, link_baud=921600))
    window.operation_done("disconnect", None)
    assert window.baud_combo.currentText() == "921600"
    assert (
        json.loads((tmp_path / "settings.json").read_text(encoding="utf-8"))["last_link_baud"]
        == 921600
    )
    window.close()
