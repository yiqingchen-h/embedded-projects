"""参数尺度、损坏分包和时间轴边界回归。"""

from __future__ import annotations

from dataclasses import replace

import pytest
from foc_control.models import BoardSnapshot, ControlError, ParameterValue, specs_for
from foc_control.protocol import (
    BadLine,
    LineDecoder,
    Telemetry,
    TelemetryBuffer,
    command_bytes,
    expected_prefix,
    fields,
    is_read_command,
    parse_telemetry,
    reply_matches,
)
from foc_control.simulator import DemoTransport


@pytest.mark.parametrize("algorithm", ("SENSORLESS_FOC", "SENSORLESS_6STEP", "OPEN_6STEP"))
def test_parameter_roundtrip(algorithm: str) -> None:
    """@brief 每个实际可写参数的两端精确往返。"""
    for definition in specs_for(algorithm).values():
        for raw in (definition.minimum, definition.maximum):
            value = ParameterValue(raw, definition.decimals, definition.unit)
            if definition.editable and definition.key not in ("SHUNT", "ENCODER_DIR"):
                assert definition.encode(definition.format(value)) == value
            assert definition.suffix is not None


@pytest.mark.parametrize(
    "text", ("nan", "Infinity", "1.00001", "abc", "0", "2.001", "1\nSET TARGET 1000")
)
def test_invalid_current(text: str) -> None:
    """@brief 无效数字及超精度不转成 SET。"""
    with pytest.raises(ControlError):
        specs_for("SENSORLESS_FOC")["CURRENT_LIMIT"].encode(text)


def test_specs_and_wire_parameter() -> None:
    """@brief 范围、只读和单位必须独立校验。"""
    with pytest.raises(ControlError):
        specs_for("OTHER")
    profile = specs_for("SENSORLESS_FOC")
    with pytest.raises(ControlError):
        profile["TARGET"].validate(ParameterValue(2000, 3, "A"))
    with pytest.raises(ControlError):
        profile["SHUNT"].encode("2")
    with pytest.raises(ControlError):
        specs_for("OPEN_6STEP")["ENCODER_DIR"].validate(ParameterValue(0, 0, ""))
    with pytest.raises(ControlError):
        profile["LD"].encode("0.24")
    assert ParameterValue.parse("OK GET TARGET=2000 DECIMALS=0 UNIT=RPM", "TARGET").value == 2000
    with pytest.raises(ControlError):
        ParameterValue.parse("OK GET TARGET=2000", "TARGET")
    assert profile["RUN_TIME"].encode("20.005").value == 20005
    assert not BoardSnapshot().idle
    assert replace(BoardSnapshot(), connected=True, run="STOPPED").idle


@pytest.mark.parametrize(
    "command",
    ("START", "STOP", "CLEAR", "SET FOC_PRINT TRACE", "MOTOR FOC ZERO 20", "MOTOR FOC HARMONIC ON"),
)
def test_read_only_rejects_write(command: str) -> None:
    """@brief 只读限制位于发送层。"""
    with pytest.raises(ControlError):
        command_bytes(command, True)


@pytest.mark.parametrize(
    "command",
    ("PING", "VERSION", "GET SOURCE", "STATUS", "MOTOR FOC REST", "MOTOR FOC DRIVE", "STORAGE"),
)
def test_read_commands(command: str) -> None:
    """@brief 所有验收命令进入只读白名单。"""
    assert is_read_command(command)
    assert command_bytes(command, True).endswith(b"\r\n")


@pytest.mark.parametrize(
    "command", ("", "PING\nSTART", "PING;START", "中文", "A" * 97, "SET TARGET +2")
)
def test_invalid_command(command: str) -> None:
    """@brief 不能将注入和非协议正文发送。"""
    with pytest.raises(ControlError):
        command_bytes(command)


def test_reply_matching() -> None:
    """@brief GET 字段不能按相同顶层名称错配。"""
    assert expected_prefix("GET TARGET") == "OK GET TARGET="
    assert expected_prefix("SET TARGET 2000") == "OK SET "
    assert reply_matches("GET TARGET", "OK GET TARGET=2000 DECIMALS=0 UNIT=RPM")
    assert not reply_matches("GET TARGET", "OK GET RESULT=OK")
    assert not reply_matches("START", "OK STARTUP")
    assert reply_matches("STOP", "ERR STOP BUSY")
    assert reply_matches("MOTOR FOC REST", "ERR FOC ARGUMENT")
    assert reply_matches("STATUS", "ERR RUNNING STOP TO READ")
    assert not reply_matches("GET TARGET", "ERR RUNNING STOP TO READ")
    assert reply_matches("PING", "ERR LINE CORRUPT")
    assert reply_matches("MOTOR UART TEST 60000", "OK MOTOR UART TEST MS=60000 SOURCE=TEST")
    assert reply_matches("MOTOR FOC MOTION 18", "OK FOC MOTION RUN=1 FIRST=18 COUNT=6")
    assert not reply_matches("MOTOR UART TEST 60000", "OK MOTOR UART TESTING MS=60000")
    assert fields("OK TEST A=1 B=2") == {"A": "1", "B": "2"}


@pytest.mark.parametrize("fault", ("ADC READ", "FOC CURRENT SUM", "ENCODER CAL REQUIRED"))
def test_multiword_fault_fields(fault: str) -> None:
    """@brief 多词故障不截断，同时保留后续字段与空单位。@param fault 固件故障原文。"""
    assert fields(f"OK MOTOR FAULT={fault} OUTPUT_DISABLED=1")["FAULT"] == fault
    assert fields(f"OK MOTOR FAULT={fault}")["FAULT"] == fault
    assert fields("OK GET POLE_PAIRS=7 DECIMALS=0 UNIT=")["UNIT"] == ""


def test_fragmentation_and_recovery() -> None:
    """@brief 每字节分包、非法控制字符、超长和后续恢复。"""
    decoder = LineDecoder(12)
    result: list[str | BadLine] = []
    for byte in b"OK PING\r\n":
        result.extend(decoder.feed(bytes([byte])))
    assert result == ["OK PING"]
    for payload in (b"\xff\n", b"A\rB\n", b"\n", b"X" * 13 + b"\n"):
        assert isinstance(decoder.feed(payload)[0], BadLine)
    assert decoder.feed(b"OK PING\n") == ["OK PING"]
    assert len(decoder.pending) == 0


def test_all_telemetry_formats() -> None:
    """@brief TRACE 的电角速度换算机械 RPM，旧格式也可区分。"""
    port = DemoTransport()
    port.run = "RUNNING"
    frame = parse_telemetry(port.trace())
    assert frame is not None and frame.id_ma is not None and frame.bus_mv == 11980
    data = parse_telemetry(
        "DATA FOC SAMPLE=1 TICKS=2 IA_MA=1 IB_MA=2 IC_MA=-3 ISC_MA=-2 EST_RPM=2000 STAGE_ID=6"
    )
    assert data is not None and data.rpm == 2000
    assert parse_telemetry("FOC:1,2,-3,-2") is not None
    assert parse_telemetry("OK START PENDING") is None
    for invalid in (
        "FOC:1,2,3",
        "FOC:nan,2,3,4",
        "FOC:x,2,3,4",
        "DATA FOC SAMPLE=1",
        "FOC_TRACE:1,2",
        "FOC_TRACE:" + ",".join(["x"] * 27),
    ):
        with pytest.raises(ControlError):
            parse_telemetry(invalid)
    parts = port.trace().split(":")[1].split(",")
    parts[3] = "7"
    with pytest.raises(ControlError):
        parse_telemetry("FOC_TRACE:" + ",".join(parts))
    with pytest.raises(ControlError):
        parse_telemetry(port.trace(), 0)


def test_tick_wrap_reset_duplicate_and_bound() -> None:
    """@brief 32位回绕、重新启动、重复样本和内存上限。"""
    buffer = TelemetryBuffer(3)
    frame = Telemetry(1, 0xFFFFFFF0, 6, 1, 2, -3, -2)
    buffer.append(frame, 0, "first", 2000)
    assert buffer.append(frame, 1, "duplicate", 2000) is None
    next_point = buffer.append(replace(frame, sample=2, ticks=0x10), 2, "wrap", 2000)
    assert next_point is not None and next_point.seconds == pytest.approx(32 / 170000000)
    buffer.append(replace(frame, sample=1, ticks=100), 3, "newrun", 2000)
    assert len(buffer.points) == 1 and buffer.points[-1].seconds == 0
    buffer.append(Telemetry(None, None, None, 1, 2, 3, 4), 4, "no_clock", None)
    assert buffer.points[-1].seconds == 1
    for i in range(10):
        buffer.append(replace(frame, sample=i + 4, ticks=200 + i * 100), 5 + i, "bound", None)
    assert len(buffer.points) == 3
