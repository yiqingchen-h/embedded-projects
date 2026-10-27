"""使用协议替身验证业务，不访问 COM，不生成实测记录。"""

from __future__ import annotations

import asyncio
import json
import time
from dataclasses import asdict, replace
from pathlib import Path

import pytest
from foc_control import controller as module
from foc_control.controller import Controller, Session
from foc_control.models import ControlError, ParameterValue
from foc_control.protocol import PlotPoint, Telemetry
from foc_control.simulator import DemoTransport
from foc_control.storage import (
    Journal,
    Preset,
    export_csv,
    name_valid,
    parameter_map,
    settings_load,
)


@pytest.fixture(autouse=True)
def fast_save(monkeypatch: pytest.MonkeyPatch) -> None:
    """@brief 测试替身使用短保存窗口，正式 1.5 秒常量不变。"""
    monkeypatch.setattr(module, "SAVE_WAIT", 0.04)


def test_complete_foc_workflow(tmp_path: Path) -> None:
    """@brief 接管、保存、正常启停、在线目标、清故障和安全断开。"""

    async def scenario() -> None:
        port = DemoTransport()
        client = Controller(port, tmp_path)
        await client.initialize()
        assert client.state.source == "REMOTE" and client.state.output_disabled
        await client.apply({"CURRENT_LIMIT": ParameterValue(1500, 3, "A")})
        await client.start_motor({})
        assert client.state.run == "RUNNING" and "SET FOC_PRINT OFF" in port.commands
        assert "SET FOC_PRINT TRACE" not in port.commands
        with pytest.raises(ControlError, match="CLOSED"):
            await client.apply({"TARGET": ParameterValue(1500, 0, "RPM")})
        port.started = time.monotonic() - 4
        await client.read_drive()
        assert client.state.closed
        await client.apply({"TARGET": ParameterValue(1500, 0, "RPM")})
        assert client.state.save == "PENDING"
        with pytest.raises(ControlError, match="只能"):
            await client.apply({"CURRENT_LIMIT": ParameterValue(1000, 3, "A")})
        await client.stop_motor()
        assert client.state.output_disabled and not client.state.closed
        await client.clear_fault()
        await client.disconnect()
        assert port.closed and not client.state.connected

    asyncio.run(scenario())


@pytest.mark.parametrize("algorithm", ("SENSORLESS_6STEP", "OPEN_6STEP"))
def test_sixstep_and_independent_parameters(tmp_path: Path, algorithm: str) -> None:
    """@brief 参数块相互独立，六步查询限制不当作故障。"""

    async def scenario() -> None:
        port = DemoTransport()
        client = Controller(port, tmp_path)
        await client.initialize()
        await client.select_algorithm(algorithm)
        assert client.state.parameters["TARGET"].value == 1000
        await client.start_motor({})
        await client.poll()
        assert client.state.run == "RUNNING"
        await client.apply({"TARGET": ParameterValue(1200, 0, "RPM")})
        await client.stop_motor()
        await client.select_algorithm("SENSORLESS_FOC")
        assert client.state.parameters["TARGET"].value == 2000
        await client.session.close()

    asyncio.run(scenario())


def test_strict_read_only(tmp_path: Path) -> None:
    """@brief 所有写入口均由发送层拦截，关闭不发送 STOP。"""

    async def scenario() -> None:
        port = DemoTransport()
        client = Controller(port, tmp_path, read_only=True)
        await client.initialize()
        for action in (
            client.apply({}),
            client.stop_motor(),
            client.take_remote(),
            client.clear_fault(),
            client.select_algorithm("OPEN_6STEP"),
            client.start_motor({}),
        ):
            with pytest.raises(ControlError):
                await action
        await client.disconnect()
        assert not any(
            command.startswith(("SET", "STOP", "START", "CLEAR")) for command in port.commands
        )

    asyncio.run(scenario())


def test_capture_copy_restore_and_audit(tmp_path: Path) -> None:
    """@brief 预设双读、离线复制、同算法恢复和跨算法原块备份。"""

    async def scenario() -> None:
        port = DemoTransport()
        client = Controller(port, tmp_path)
        await client.initialize()
        preset = await client.capture("reference")
        path = tmp_path / "reference.json"
        preset.save(path)
        assert Preset.load(path) == preset
        with pytest.raises(FileExistsError):
            preset.save(path)
        copy = preset.copy("slow", {"TARGET": "1500", "CURRENT_LIMIT": "1.5"})
        assert copy.parameters["TARGET"].value == 1500
        await client.restore(copy)
        assert port.parameters["TARGET"].value == 1500
        await client.select_algorithm("OPEN_6STEP")
        six = await client.capture("six")
        await client.restore(preset)
        assert port.algorithm == "SENSORLESS_FOC"
        await client.restore(six)
        assert port.algorithm == "OPEN_6STEP"
        assert len(list((tmp_path / "restore_backups").glob("*.json"))) >= 5
        await client.session.close()

    asyncio.run(scenario())


def test_restore_protocol_model_failure(tmp_path: Path) -> None:
    """@brief 协议不兼容不写入，模型不符保留备份后停止。@param tmp_path 临时目录。"""

    async def scenario() -> None:
        port = DemoTransport()
        client = Controller(port, tmp_path)
        await client.initialize()
        preset = await client.capture("ref")
        before = len([command for command in port.commands if command.startswith("SET")])
        with pytest.raises(ControlError, match="版本"):
            await client.restore(replace(preset, protocol="2.0"))
        assert len([command for command in port.commands if command.startswith("SET")]) == before
        changed_model = dict(preset.readonly)
        changed_model["LD"] = ParameterValue(300, 3, "MH")
        with pytest.raises(ControlError, match="只读"):
            await client.restore(replace(preset, readonly=changed_model))
        assert list((tmp_path / "restore_backups").glob("*.json"))
        await client.session.close()

    asyncio.run(scenario())


def test_priority_stop_interrupts_save(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """@brief 保存静默等待可取消，STOP 不等其到期。"""

    async def scenario() -> None:
        port = DemoTransport()
        client = Controller(port, tmp_path)
        await client.initialize()
        monkeypatch.setattr(module, "SAVE_WAIT", 10)
        applying = asyncio.create_task(client.apply({"TARGET": ParameterValue(1800, 0, "RPM")}))
        while "SET TARGET 1800" not in port.commands:
            await asyncio.sleep(0.002)
        applying.cancel()
        try:
            await applying
        except asyncio.CancelledError:
            pass
        before = time.monotonic()
        await client.stop_motor()
        assert "STOP" in port.commands and time.monotonic() - before < 0.2
        await client.session.close()

    asyncio.run(scenario())


class BrokenPort(DemoTransport):
    """对指定命令注入响应异常，不改变其余真实业务路径。"""

    def __init__(self, target: str, reply: str | None) -> None:
        """@brief 配置故障。@param target 命令。@param reply 注入回执或丢失。"""
        super().__init__()
        self.target, self.reply = target, reply

    def write(self, data: bytes) -> None:
        """@brief 注入单个协议错误。@param data 命令字节。"""
        command = data.decode().strip()
        if command == self.target:
            self.commands.append(command)
            if self.reply is not None:
                self.emit(self.reply)
        else:
            super().write(data)


@pytest.mark.parametrize("reply", ("OK GET RESULT=OK", "GARBAGE", "\xff"))
def test_mismatch_unknown_and_bad_bytes(reply: str) -> None:
    """@brief 响应异常不会被误配到待处理请求。"""

    async def scenario() -> None:
        port = BrokenPort("GET TARGET", reply if reply != "\xff" else None)
        session = Session(port, False, lambda _kind, _text: None, lambda _line: None)
        await session.start()
        if reply == "\xff":
            port.incoming.extend(b"\xff\n")
        with pytest.raises(ControlError):
            await session.ask("GET TARGET", timeout=0.05)
        assert session.link_error and port.closed
        await session.close()

    asyncio.run(scenario())


def test_timeout_cancellation_and_unsolicited() -> None:
    """@brief 超时结果未知、取消请求迟到回执与遥测交错。"""

    async def scenario() -> None:
        port = BrokenPort("GET TARGET", None)
        telemetry: list[str] = []
        session = Session(port, False, lambda _kind, _text: None, telemetry.append)
        await session.start()
        task = asyncio.create_task(session.ask("GET TARGET", timeout=1))
        await asyncio.sleep(0.005)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        port.emit("FOC:1,2,-3,-2")
        port.emit("OK GET TARGET=2000 DECIMALS=0 UNIT=RPM")
        assert await session.ask("STOP", priority=True) == "OK STOP PENDING"
        assert telemetry
        port.emit("OK PING")
        await asyncio.sleep(0.005)
        with pytest.raises(ControlError):
            await session.ask("GET TARGET", timeout=0.01)
        await session.close()

    asyncio.run(scenario())


def test_pending_failure_save_error_and_partial_backup(tmp_path: Path) -> None:
    """@brief 后端失败、Flash 失败和部分恢复均停止后续操作。"""

    async def scenario() -> None:
        port = BrokenPort("GET RESULT", "OK GET RESULT=DRV/ADC CAL")
        client = Controller(port, tmp_path)
        await client.session.start()
        with pytest.raises(ControlError, match="DRV/ADC"):
            await client.await_result()
        await client.session.close()
        good = DemoTransport()
        second = Controller(good, tmp_path)
        await second.initialize()
        preset = await second.capture("ref")
        partial = preset.copy("partial", {"TARGET": "1500", "CURRENT_LIMIT": "1.5"})
        original = good.write

        def failing(data: bytes) -> None:
            """@brief 第二个修改失败，第一项允许已生效。@param data 命令。"""
            if data.decode().startswith("SET CURRENT_LIMIT"):
                good.emit("ERR SET RANGE")
            else:
                original(data)

        good.write = failing
        with pytest.raises(ControlError):
            await second.restore(partial)
        assert list((tmp_path / "restore_backups").glob("*.json"))
        assert not any(command in ("START", "CLEAR") for command in good.commands)

        def bad_save(data: bytes) -> None:
            """@brief 保存查询返回真实错误回执。@param data 命令。"""
            if data.decode().strip() == "GET SAVE":
                good.emit("OK GET SAVE=ERROR")
            else:
                original(data)

        good.write = bad_save
        with pytest.raises(ControlError):
            await second.saved(changed=True)
        await second.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize("bad", ("", "../x", "中文", "a" * 65))
def test_names(bad: str) -> None:
    """@brief 文件命名不得穿越目录。"""
    with pytest.raises(ControlError):
        name_valid(bad)


def test_json_csv_and_journal(tmp_path: Path) -> None:
    """@brief JSON 类型校验、UTF8 CSV、原始日志和偏好容错。"""
    for value in (
        [],
        {"TARGET": {}},
        {"TARGET": {"value": True, "decimals": 0, "unit": "RPM"}},
        {"TARGET": {"value": 1, "decimals": 5, "unit": "RPM"}},
    ):
        with pytest.raises(ControlError):
            parameter_map(value)
    path = tmp_path / "settings.json"
    assert settings_load(path) == {}
    path.write_text('{"port":"COM10"}', encoding="utf-8")
    assert settings_load(path)["port"] == "COM10"
    path.write_text("[1]", encoding="utf-8")
    assert settings_load(path) == {}
    point = PlotPoint(1.5, "stamp", Telemetry(1, 2, 6, 1000, 2000, -3000, -2900), 2000)
    export_csv(tmp_path / "wave.csv", [point])
    assert "1.0,2.0,-3.0,-2.9" in (tmp_path / "wave.csv").read_text(encoding="utf-8-sig")
    log = Journal(tmp_path / "logs", "DEMO")
    log.write("TX", "PING")
    log.write("DATA", "FOC:1,2,3,4")
    log.close()
    log.write("INFO", "closed")
    assert len(log.path.read_text(encoding="utf-8").splitlines()) == 3


def test_corrupt_preset_files(tmp_path: Path) -> None:
    """@brief 文件字段、格式与完整性在任何写命令前验证。"""

    async def scenario() -> Preset:
        client = Controller(DemoTransport(), tmp_path)
        await client.initialize()
        preset = await client.capture("ref")
        await client.session.close()
        return preset

    preset = asyncio.run(scenario())
    for mutation in (
        {"schema_version": True},
        {"mode": "TORQUE"},
        {"brightness": 0},
        {"animation": True},
        {"format": "bad"},
        {"source": "OTHER"},
        {"parameters": {}},
        {"readonly": {}},
        {"firmware": ""},
    ):
        payload = asdict(preset)
        payload.update(mutation)
        path = tmp_path / "invalid.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        with pytest.raises(ControlError):
            Preset.load(path)
    with pytest.raises(ControlError):
        preset.copy("x", {"UNKNOWN": "2"})
