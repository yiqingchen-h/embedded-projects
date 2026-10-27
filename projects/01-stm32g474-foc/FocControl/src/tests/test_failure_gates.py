"""状态前提、保存错误、取消和坏遥测的独立故障注入。"""

from __future__ import annotations

import asyncio
import time
from dataclasses import replace
from pathlib import Path

import pytest
from foc_control import controller as module
from foc_control.controller import Controller, Session
from foc_control.models import BoardSnapshot, ControlError, ParameterValue
from foc_control.protocol import Telemetry, TelemetryBuffer
from foc_control.simulator import DemoTransport
from foc_control.storage import Preset


class OverridePort(DemoTransport):
    """只覆盖指定查询或控制响应，保留正常请求日志。"""

    def __init__(self, overrides: dict[str, str]) -> None:
        """@brief 注入覆盖表。@param overrides 命令到响应。"""
        super().__init__()
        self.overrides = overrides

    def write(self, data: bytes) -> None:
        """@brief 覆盖响应但不偷偷更新设备状态。@param data 命令字节。"""
        command = data.decode().strip()
        if command in self.overrides:
            self.commands.append(command)
            self.emit(self.overrides[command])
        else:
            super().write(data)


@pytest.fixture(autouse=True)
def fast(monkeypatch: pytest.MonkeyPatch) -> None:
    """@brief 测试保存等待缩短到模拟保存窗口。"""
    monkeypatch.setattr(module, "SAVE_WAIT", 0.03)


@pytest.mark.parametrize(
    "command,response",
    (
        ("VERSION", "OK VERSION PROTOCOL=1.2"),
        ("VERSION", "ERR VERSION UNKNOWN"),
    ),
)
def test_identity_and_metadata_no_write(tmp_path: Path, command: str, response: str) -> None:
    """@brief 身份缺失或查询失败时不能自动接管。@param tmp_path 临时目录。
    @param command 注入命令。@param response 注入回执。
    """

    async def scenario() -> None:
        port = OverridePort({command: response})
        client = Controller(port, tmp_path)
        with pytest.raises(ControlError):
            await client.initialize()
        assert not any(value.startswith("SET ") for value in port.commands)
        await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "overrides",
    (
        {"STATUS": "OK STATUS MOTOR=FAULT READY=1"},
        {"GET RESULT": "OK GET RESULT=PENDING"},
        {"MOTOR FOC REST": "OK MOTOR FOC REST OUTPUT_DISABLED=0 FAULT=NONE"},
        {"MOTOR FOC REST": "OK MOTOR FOC REST OUTPUT_DISABLED=1 FAULT=DRV"},
        {"STORAGE": "OK STORAGE MATCH=0 RESULT=LAYOUT"},
    ),
)
def test_idle_guards(tmp_path: Path, overrides: dict[str, str]) -> None:
    """@brief 所有修改前提失败都在 SET 之前终止。"""

    async def scenario() -> None:
        port = OverridePort(overrides)
        client = Controller(port, tmp_path)
        await client.session.start()
        with pytest.raises(ControlError):
            await client.idle()
        assert not any(command.startswith("SET") for command in port.commands)
        await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "response", ("ERR STATUS UNKNOWN", "OK STATUS READY=1", "OK STATUS MOTOR=BAD")
)
def test_bad_status(tmp_path: Path, response: str) -> None:
    """@brief 缺失或未知状态不得映射成已停止。"""

    async def scenario() -> None:
        port = OverridePort({"STATUS": response})
        client = Controller(port, tmp_path)
        await client.session.start()
        with pytest.raises(ControlError):
            await client.refresh_status()
        assert client.state.run == "UNKNOWN"
        await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "command,response",
    (
        ("GET SAVE", "OK GET SAVE=ERROR"),
        ("GET SAVE", "OK GET SAVE=PENDING"),
        ("STORAGE", "OK STORAGE MATCH=1 RESULT=VERIFY"),
    ),
)
def test_save_refuses_false_success(tmp_path: Path, command: str, response: str) -> None:
    """@brief PENDING、Flash 校验失败都不报告已保存。"""

    async def scenario() -> None:
        port = OverridePort({command: response})
        client = Controller(port, tmp_path)
        await client.session.start()
        with pytest.raises(ControlError):
            await client.saved(changed=True)
        await client.session.close()

    asyncio.run(scenario())


def test_default_pending_and_source_restore(tmp_path: Path) -> None:
    """@brief 冷启动 DEFAULT 可读，显示元数据和 LOCAL 来源恢复后不自动启动。"""

    async def scenario() -> None:
        port = OverridePort({"GET SAVE": "OK GET SAVE=DEFAULT"})
        client = Controller(port, tmp_path)
        await client.session.start()
        await client.saved(changed=False)
        with pytest.raises(ControlError):
            await client.saved(changed=True)
        await client.session.close()
        good = DemoTransport()
        second = Controller(good, tmp_path)
        await second.initialize()
        preset = await second.capture("local")
        await second.restore(replace(preset, source="LOCAL", brightness=60, animation=0))
        assert second.state.source == "LOCAL" and "START" not in good.commands
        with pytest.raises(ControlError, match="上位机"):
            await second.start_motor({})
        await second.take_remote()
        assert second.state.source == "REMOTE"
        await second.session.close()

    asyncio.run(scenario())


def test_set_ack_and_full_readback(tmp_path: Path) -> None:
    """@brief 即时回执和实际数值都必须正确。"""

    async def scenario() -> None:
        for overrides in (
            {"SET TARGET 1000": "OK SET TARGET=999"},
            {"GET TARGET": "OK GET TARGET=999 DECIMALS=0 UNIT=RPM"},
            {"GET TARGET": "OK GET TARGET=1000 DECIMALS=3 UNIT=A"},
        ):
            port = OverridePort(overrides)
            client = Controller(port, tmp_path)
            await client.session.start()
            with pytest.raises(ControlError):
                if "SET TARGET 1000" in overrides or "999" in overrides.get("GET TARGET", ""):
                    await client.set_value("TARGET", "1000")
                else:
                    client.update(
                        connected=True, firmware="1.4.6-serial", source="REMOTE", run="STOPPED"
                    )
                    await client.apply({"TARGET": ParameterValue(1000, 0, "RPM")})
            await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "overrides",
    (
        {"STOP": "ERR STOP BUSY"},
        {"STATUS": "OK STATUS MOTOR=RUNNING"},
        {"MOTOR FOC REST": "OK MOTOR FOC REST OUTPUT_DISABLED=0 FAULT=NONE"},
    ),
)
def test_stop_requires_output_proof(tmp_path: Path, overrides: dict[str, str]) -> None:
    """@brief STOP 被接受不等于已验证输出关闭。"""

    async def scenario() -> None:
        port = OverridePort(overrides)
        client = Controller(port, tmp_path)
        await client.session.start()
        with pytest.raises(ControlError):
            await client.stop_motor()
        assert client.state.output_disabled is not True
        await client.session.close()

    asyncio.run(scenario())


def test_apply_unknown_and_readonly_fields(tmp_path: Path) -> None:
    """@brief 未知和只读字段不能进入写队列。"""

    async def scenario() -> None:
        client = Controller(DemoTransport(), tmp_path)
        await client.initialize()
        for changes in (
            {"OFFSET_A": ParameterValue(2048, 0, "ADC")},
            {"LD": ParameterValue(240, 3, "MH")},
            {"TARGET": ParameterValue(999999, 0, "RPM")},
        ):
            with pytest.raises(ControlError):
                await client.apply(changes)
        await client.session.close()

    asyncio.run(scenario())


def test_telemetry_bad_lines_and_post_stop_stale(tmp_path: Path) -> None:
    """@brief 实时解析错误保留原文，停机后迟到 TRACE 不重新显示闭环。"""
    port = DemoTransport()
    port.run = "RUNNING"
    port.started = time.monotonic() - 4
    points: list[object] = []
    logs: list[tuple[str, str]] = []
    client = Controller(
        port, tmp_path, on_log=lambda kind, text: logs.append((kind, text)), on_point=points.append
    )
    client.update(
        connected=True, run="RUNNING", parameters={"TARGET": ParameterValue(2000, 0, "RPM")}
    )
    client._telemetry(port.trace())
    assert client.state.closed and client.state.bus_v is not None and points
    client._telemetry("OK PING")
    client._telemetry("FOC:bad,1,2,3")
    assert any("原文" in message for _kind, message in logs)
    client.stop_requested = True
    client.update(run="STOPPED", closed=False, stage="停机")
    client._telemetry(port.trace())
    assert not client.state.closed and client.state.stage == "停机"
    client.session.link_error = "USB disconnected"
    client.error("读取失败")
    assert client.state.run == "UNKNOWN" and not client.state.connected
    assert not BoardSnapshot(connected=True, firmware="1.4.6-serial").writable
    assert BoardSnapshot(connected=True, protocol="1.2", control_available=True).writable


def test_readonly_poll_and_automatic_stop(tmp_path: Path) -> None:
    """@brief 只读运行诊断不写打印开关；自动停机后读取保存和统计。"""

    async def scenario() -> None:
        port = DemoTransport()
        port.run, port.started = "RUNNING", time.monotonic() - 4
        client = Controller(port, tmp_path, read_only=True)
        await client.initialize()
        await client.poll()
        assert client.state.closed and not any(
            command.startswith("SET") for command in port.commands
        )
        port.started -= 30
        await client.poll()
        assert client.state.run == "STOPPED" and client.state.diagnostics
        await client.poll()
        await client.disconnect()

    asyncio.run(scenario())


def test_receiver_oserror_and_priority_gate() -> None:
    """@brief 驱动错误导致未知状态，只有 STOP 可绕过普通锁。"""

    async def scenario() -> None:
        port = DemoTransport()
        session = Session(port, False, lambda _kind, _text: None, lambda _line: None)
        await session.start()
        with pytest.raises(ControlError):
            await session.ask("PING", priority=True)
        port.closed = True
        await asyncio.sleep(0.01)
        assert session.closed and session.link_error
        with pytest.raises(ControlError):
            await session.ask("PING")
        await session.close()

    asyncio.run(scenario())


def test_capture_change_and_malformed_file(tmp_path: Path) -> None:
    """@brief 双读期间变化拒绝备份，坏文件不进入恢复。"""

    async def scenario() -> None:
        port = DemoTransport()
        client = Controller(port, tmp_path)
        await client.initialize()
        original = port.write
        count = 0

        def changing(data: bytes) -> None:
            """@brief 两次读取返回不同目标。@param data 命令。"""
            nonlocal count
            if data.decode().strip() == "GET TARGET":
                count += 1
                value = port.parameters["TARGET"]
                port.parameters["TARGET"] = replace(value, value=2000 + count)
            original(data)

        port.write = changing
        with pytest.raises(ControlError, match="变化"):
            await client.capture("unstable")
        await client.session.close()

    asyncio.run(scenario())
    for text in ("[]", "{", '{"format":"x"}'):
        path = tmp_path / "bad.json"
        path.write_text(text, encoding="utf-8")
        with pytest.raises(ControlError):
            Preset.load(path)
    huge = tmp_path / "huge.json"
    huge.write_bytes(b" " * 262145)
    with pytest.raises(ControlError):
        Preset.load(huge)


def test_tick_discontinuity() -> None:
    """@brief 时间计数不合理后退时重新建立时间轴。"""
    buffer = TelemetryBuffer()
    frame = Telemetry(1, 100, 6, 1, 2, 3, 4)
    buffer.append(frame, 0, "a", 2000)
    buffer.append(replace(frame, sample=2, ticks=50), 1, "b", 2000)
    assert len(buffer.points) == 1 and buffer.points[-1].seconds == 0
