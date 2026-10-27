"""固件演进、有感后端、缺失能力及跨版本预设的离线回归。"""

from __future__ import annotations

import asyncio
from dataclasses import replace
from pathlib import Path

import pytest
from foc_control import controller as module
from foc_control.controller import Controller
from foc_control.models import ControlError, ParameterValue
from foc_control.protocol import command_bytes, parse_telemetry
from foc_control.simulator import DemoTransport


class CapabilityPort(DemoTransport):
    """模拟当前七极对板卡，按查询注入缺失字段与未来元数据。"""

    def __init__(
        self, overrides: dict[str, str] | None = None, algorithm: str = "ENCODER_FOC"
    ) -> None:
        """@brief 创建模拟板卡。@param overrides 覆盖回执。@param algorithm 当前算法。"""
        super().__init__("1.6.21-g2208-encfoc", algorithm, g2208=True)
        self.overrides = overrides or {}

    def write(self, data: bytes) -> None:
        """@brief 注入指定响应，其余保持状态机行为。@param data 请求。"""
        command = data.decode("ascii").strip()
        if command in self.overrides:
            self.commands.append(command)
            self.emit(self.overrides[command])
        else:
            super().write(data)


@pytest.mark.parametrize("corrupt_replies", (1, 2, 3))
def test_initial_corrupt_version_has_bounded_retry(tmp_path: Path, corrupt_replies: int) -> None:
    """@brief 首条身份查询损坏可恢复，持续损坏最多三次。@param tmp_path 临时目录。
    @param corrupt_replies 连续损坏回执数。
    """

    class HandshakePort(CapabilityPort):
        """复现固件丢弃损坏行后在LF边界恢复的行为。"""

        def write(self, data: bytes) -> None:
            """@brief 仅损坏开头的身份查询。@param data 请求字节。"""
            if data == b"VERSION\r\n" and self.commands.count("VERSION") < corrupt_replies:
                self.commands.append("VERSION")
                self.emit("ERR LINE CORRUPT")
            else:
                super().write(data)

    async def scenario() -> None:
        """@brief 原始错误保留日志，恢复过程不会执行任何控制写入。"""
        port = HandshakePort()
        records: list[tuple[str, str]] = []
        client = Controller(
            port, tmp_path, read_only=True, on_log=lambda kind, line: records.append((kind, line))
        )
        try:
            if corrupt_replies == 3:
                with pytest.raises(ControlError, match="ERR LINE CORRUPT"):
                    await client.initialize()
                assert not client.state.connected
            else:
                await client.initialize()
                assert client.state.connected and client.state.firmware == "1.6.21-g2208-encfoc"
            assert port.commands.count("VERSION") == min(corrupt_replies + 1, 3)
            assert (
                sum(kind == "RX" and line == "ERR LINE CORRUPT" for kind, line in records)
                == corrupt_replies
            )
            assert not any(
                command.startswith(("SET ", "START", "STOP", "CLEAR")) for command in port.commands
            )
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_corrupt_parameter_write_is_not_replayed(tmp_path: Path) -> None:
    """@brief 身份查询恢复策略不能重放参数写入。@param tmp_path 临时目录。"""

    async def scenario() -> None:
        """@brief 写入损坏后立即报告，保留现场。"""
        port = CapabilityPort({"SET TARGET 100": "ERR LINE CORRUPT"})
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            with pytest.raises(ControlError, match="ERR LINE CORRUPT"):
                await client.apply({"TARGET": ParameterValue(100, 0, "RPM")})
            assert port.commands.count("SET TARGET 100") == 1
        finally:
            await client.session.close()

    asyncio.run(scenario())


@pytest.fixture(autouse=True)
def fast_save(monkeypatch: pytest.MonkeyPatch) -> None:
    """@brief 缩短模拟保存窗口。@param monkeypatch 测试替身。"""
    monkeypatch.setattr(module, "SAVE_WAIT", 0.03)


@pytest.mark.parametrize(
    "firmware", ("1.4.7-g2208", "1.5.5-g2208-enc6", "1.6.21-g2208-encfoc", "dev-custom-build")
)
def test_firmware_label_does_not_lock_control(tmp_path: Path, firmware: str) -> None:
    """@brief 同协议升级不再匹配固件字符串。@param tmp_path 临时目录。@param firmware 身份。"""

    async def scenario() -> None:
        """@brief 核对当前模型、写入与停止均能工作。"""
        port = CapabilityPort()
        port.firmware = firmware
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            assert client.state.writable and client.state.firmware == firmware
            assert client.state.parameter_specs["TARGET"].maximum == 1800
            assert client.state.parameter_specs["CURRENT_LIMIT"].maximum == 800
            assert not client.state.parameter_specs["POLE_PAIRS"].editable
            await client.apply({"TARGET": ParameterValue(1, 0, "RPM")})
            await client.start_motor({})
            await client.read_drive()
            assert client.state.closed and client.state.estimated_rpm == 1.012
            await client.stop_motor()
            assert client.state.output_disabled and client.state.run == "STOPPED"
        finally:
            await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "overrides",
    (
        {"GET D_KP": "ERR GET NOT_READY"},
        {"GET FOC_PRINT": "ERR GET ARGUMENT"},
        {"MOTOR RUN": "ERR MOTOR ARGUMENT"},
        {"GET RAMP": "OK GET RAMP=200000 DECIMALS=2 UNIT=OTHER"},
    ),
)
def test_optional_capability_does_not_disconnect(tmp_path: Path, overrides: dict[str, str]) -> None:
    """@brief 可选能力缺失只隐藏或禁用相应项。@param tmp_path 临时目录。@param overrides 回执。"""

    async def scenario() -> None:
        """@brief 缺失诊断和参数不影响普通启停。"""
        port = CapabilityPort(overrides)
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            assert client.state.connected and client.state.writable
            if "GET RAMP" in overrides:
                assert not client.state.parameter_specs["RAMP"].editable
                preset = await client.capture("changed_units")
                assert "RAMP" in preset.readonly
            await client.start_motor({})
            await client.stop_motor()
        finally:
            await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "overrides",
    (
        {"GET CURRENT_LIMIT": "OK GET CURRENT_LIMIT=500 DECIMALS=0 UNIT=MA"},
        {"GET TARGET": "ERR GET NOT_READY"},
        {"GET ALGORITHM": "OK GET ALGORITHM=FUTURE_FOC"},
        {"GET MODE": "OK GET MODE=TORQUE"},
    ),
)
def test_unknown_control_keeps_monitor_and_stop(tmp_path: Path, overrides: dict[str, str]) -> None:
    """@brief 未知控制语义仍能监视与停止，禁止其它写入。@param tmp_path 临时目录。@param overrides 回执。"""

    async def scenario() -> None:
        """@brief 后台入口也必须拒绝绕过界面的写入。"""
        port = CapabilityPort(overrides)
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            assert client.state.connected and not client.state.writable and client.state.can_stop
            assert not any(command.startswith("SET ") for command in port.commands)
            with pytest.raises(ControlError):
                await client.start_motor({})
            with pytest.raises(ControlError):
                await client.set_value("TARGET", "600")
            await client.stop_motor()
            assert client.state.output_disabled
        finally:
            await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "protocol,controllable", (("1.2", True), ("1.3", True), ("1.1", False), ("2.0", False))
)
def test_protocol_compatibility(tmp_path: Path, protocol: str, controllable: bool) -> None:
    """@brief 协议主版本变化保留只读连接。@param tmp_path 临时目录。@param protocol 协议。
    @param controllable 是否可控。
    """

    async def scenario() -> None:
        """@brief 不兼容协议不能发出任何写命令。"""
        port = CapabilityPort({"VERSION": f"OK VERSION FW=next PROTOCOL={protocol} EXTRA=1"})
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            assert client.state.connected and client.state.writable == controllable
            if not controllable:
                assert client.state.read_only and not client.state.can_stop
                await client.disconnect()
                assert not any(
                    command.startswith(("SET ", "START", "STOP", "CLEAR"))
                    for command in port.commands
                )
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_optional_range_metadata(tmp_path: Path) -> None:
    """@brief 未来追加范围字段无需改版上位机。@param tmp_path 临时目录。"""

    async def scenario() -> None:
        """@brief 读取范围和权限，禁止越界草稿。"""
        port = CapabilityPort(
            {
                "GET RUN_TIME": "OK GET RUN_TIME=5000 DECIMALS=0 UNIT=MS MIN=1000 MAX=90000 EDITABLE=1",
                "GET SPEED_KP": "OK GET SPEED_KP=100 DECIMALS=3 UNIT=MA/RPM MIN=1 MAX=2000 EDITABLE=0",
            }
        )
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            assert client.state.parameter_specs["RUN_TIME"].encode("80").value == 80000
            assert not client.state.parameter_specs["SPEED_KP"].editable
            preset = await client.capture("range_metadata")
            assert "SPEED_KP" in preset.readonly
            with pytest.raises(ControlError):
                await client.apply({"TARGET": ParameterValue(1801, 0, "RPM")})
            with pytest.raises(ControlError):
                client.state.parameter_specs["RUN_TIME"].encode("91")
        finally:
            await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize("algorithm", ("ENCODER_FOC", "ENCODER_6STEP"))
def test_encoder_calibration_and_profiles(tmp_path: Path, algorithm: str) -> None:
    """@brief 有感后端显式标定后可启动，停机可取消。@param tmp_path 临时目录。@param algorithm 算法。"""

    async def scenario() -> None:
        """@brief 七点完成后恢复原算法，不自动 START。"""
        port = CapabilityPort(algorithm=algorithm)
        port.calibration = "REQUIRED"
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            if algorithm == "ENCODER_6STEP":
                assert client.state.parameter_specs["DUTY"].encode("95").value == 950
                assert "HANDOFF" not in client.state.parameters
            with pytest.raises(ControlError, match="标定"):
                await client.start_motor({})
            assert "START" not in port.commands
            await client.calibrate_encoder()
            assert (
                client.state.algorithm == algorithm and client.state.encoder_calibration == "VALID"
            )
            assert "START" not in port.commands and "CLEAR" not in port.commands
            await client.start_motor({})
            await client.poll()
            assert client.state.run == "RUNNING"
            await client.stop_motor()
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_cross_version_restore_checks_model(tmp_path: Path) -> None:
    """@brief 同电机跨版本可恢复，派生增益不锁死预设。@param tmp_path 临时目录。"""

    async def scenario() -> None:
        """@brief 保留原配置备份并逐项回读，不覆盖会话标定。"""
        port = CapabilityPort()
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            preset = await client.capture("compatible")
            values = {**preset.parameters, "TARGET": ParameterValue(100, 0, "RPM")}
            readonly = {
                **preset.readonly,
                "D_KP": ParameterValue(9999, 3, "V/A"),
                "ELEC_ZERO": ParameterValue(521, 1, "DEG"),
            }
            port.firmware = "1.6.22-next"
            await client.restore(replace(preset, parameters=values, readonly=readonly))
            assert port.parameters["TARGET"].value == 100
            assert port.parameters["ELEC_ZERO"].value == 504
            assert list((tmp_path / "restore_backups").glob("*.json"))
            before = len(port.commands)
            bad = replace(preset, readonly={**preset.readonly, "LD": ParameterValue(240, 3, "MH")})
            with pytest.raises(ControlError, match="模型"):
                await client.restore(bad)
            assert not any(command.startswith("SET ") for command in port.commands[before:])
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_trace_extension_and_readonly_encoder() -> None:
    """@brief TRACE 追加数值字段兼容，标定不进入只读白名单。"""
    port = CapabilityPort()
    assert parse_telemetry(port.trace() + ",123,456", 7) is not None
    assert command_bytes("MOTOR ENCODER", True)
    assert command_bytes("ENCODER", True)
    with pytest.raises(ControlError):
        command_bytes("MOTOR ENCODER CAL", True)


@pytest.mark.parametrize(
    "overrides",
    (
        {"GET RESULT": "OK GET RESULT=PENDING"},
        {"STORAGE": "OK STORAGE MATCH=0 RESULT=LAYOUT"},
        {"STORAGE": "ERR STORAGE ARGUMENT"},
        {"MOTOR FOC REST": "ERR FOC ARGUMENT"},
        {"GET SAVE": "ERR GET ARGUMENT"},
    ),
)
def test_initial_precondition_failure_keeps_connection(
    tmp_path: Path, overrides: dict[str, str]
) -> None:
    """@brief 初始化遇到临时前提或缺失诊断不关闭链路。@param tmp_path 临时目录。@param overrides 回执。"""

    async def scenario() -> None:
        """@brief 未确认写入前提时不自动切换 REMOTE。"""
        port = CapabilityPort(overrides)
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            assert client.state.connected and client.state.notices
            assert not any(command.startswith("SET ") for command in port.commands)
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_calibration_can_be_interrupted(tmp_path: Path) -> None:
    """@brief 优先停止可中止标定，不触发重新启动或清故障。@param tmp_path 临时目录。"""

    async def scenario() -> None:
        """@brief 取消标定任务后，STOP 仍须取得输出关闭回读。"""
        port = CapabilityPort()
        port.calibration = "REQUIRED"
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            task = asyncio.create_task(client.calibrate_encoder())
            while "MOTOR ENCODER CAL" not in port.commands:
                await asyncio.sleep(0.002)
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
            await client.stop_motor()
            assert client.state.output_disabled and client.state.encoder_calibration == "REQUIRED"
            assert "CLEAR" not in port.commands and "START" not in port.commands
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_future_preset_range_is_checked_against_board(tmp_path: Path) -> None:
    """@brief 允许保存未来范围，但不能写到仍限制32秒的板卡。@param tmp_path 临时目录。"""

    async def scenario() -> None:
        """@brief 文件范围解锁不绕过在线写入校验。"""
        port = CapabilityPort()
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            preset = await client.capture("future_range")
            preset = replace(
                preset, parameters={**preset.parameters, "RUN_TIME": ParameterValue(80000, 0, "MS")}
            )
            preset.validate()
            preset.save(tmp_path / "future_range.json")
            before = len(port.commands)
            with pytest.raises(ControlError, match="范围"):
                await client.restore(preset)
            assert not any(command.startswith("SET ") for command in port.commands[before:])
        finally:
            await client.session.close()

    asyncio.run(scenario())
