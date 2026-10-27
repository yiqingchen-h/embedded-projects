"""高速协议、丢样、时间回绕、独立状态与固定数组回归，不访问设备。"""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np
import pytest
from foc_control.binary_wave import (
    TEST,
    VALID,
    BadWave,
    BinaryRecorder,
    MixedDecoder,
    WaveBlock,
    WaveBuffer,
    cobs_decode,
    cobs_encode,
    decode_block,
    encode_block,
    export_binary_csv,
)
from foc_control.models import ControlError


def block(
    sequence: int = 1,
    count: int = 32,
    *,
    session: int = 1,
    step: int = 17000,
    ticks: int | None = None,
    flags: int = VALID,
) -> WaveBlock:
    """@brief 构造明确的测试块，保留整数精度。@return 数据块。"""
    values = np.tile(np.array((7, -7, 0, 82, -82, 0, 1024), dtype="<i2"), (count, 1))
    return WaveBlock(
        1,
        flags,
        session,
        sequence,
        (sequence * 17000 if ticks is None else ticks) & 0xFFFFFFFF,
        step,
        count,
        values.tobytes(),
    )


@pytest.mark.parametrize(
    "data",
    (
        b"",
        b"\0",
        b"\0" * 480,
        bytes(range(256)),
        b"x" * 480,
        b"\r\nOK STOP PENDING\r\n\0FOC_TRACE:",
    ),
)
def test_cobs_round_trip(data: bytes) -> None:
    """@brief 所有分隔符都只属于二进制载荷。"""
    encoded = cobs_encode(data)
    assert b"\0" not in encoded
    assert cobs_decode(encoded) == data


def test_every_split_and_mixed_replies() -> None:
    """@brief 任意字节拆包仍能正确分离命令回执。"""
    packet = encode_block(block())
    wire = b"OK PING\r\n" + packet + b"OK STOP PENDING\r\n" + packet
    for split in range(len(wire) + 1):
        decoder = MixedDecoder()
        events = decoder.feed(wire[:split]) + decoder.feed(wire[split:])
        assert [item for item in events if isinstance(item, str)] == ["OK PING", "OK STOP PENDING"]
        assert sum(isinstance(item, WaveBlock) for item in events) == 2
        assert not any(isinstance(item, BadWave) for item in events)


def test_crc_failure_and_next_packet_recovery() -> None:
    """@brief CRC失败后恢复下个独立块，不产生伪回执。"""
    packet = bytearray(encode_block(block()))
    packet[-6] ^= 1
    events = MixedDecoder().feed(bytes(packet) + b"OK PING\r\n" + encode_block(block(33)))
    assert isinstance(events[0], BadWave)
    assert events[1] == "OK PING"
    assert isinstance(events[2], WaveBlock)
    assert events[2].sequence == 33


def test_attach_mid_frame_and_lost_delimiter_recovery() -> None:
    """@brief 从任意帧内位置接入或丢一个零边界，后续完整帧和回执可恢复。"""
    first = encode_block(block())
    following = encode_block(block(33)) + encode_block(block(65)) + b"OK PING\r\n"
    for offset in range(1, len(first)):
        events = MixedDecoder().feed(first[offset:] + following)
        packets = [item for item in events if isinstance(item, WaveBlock)]
        assert packets[-1].sequence == 65
        assert [item for item in events if isinstance(item, str)] == ["OK PING"]
    wire = first + following
    for offset, value in enumerate(first):
        if value == 0:
            events = MixedDecoder().feed(wire[:offset] + wire[offset + 1 :])
            assert any(isinstance(item, WaveBlock) and item.sequence == 65 for item in events)
            assert [item for item in events if isinstance(item, str)] == ["OK PING"]


def test_leading_boot_nul_does_not_hide_ascii_identity() -> None:
    """@brief 已观察的启动单NUL后仍能读身份，完整二进制头不送文本解析。"""
    wire = b"\0OK VERSION FW=1.7.0 PROTOCOL=1.3\r\n" + encode_block(block()) + b"OK PING\r\n"
    for split in range(len(wire) + 1):
        decoder = MixedDecoder()
        events = decoder.feed(wire[:split]) + decoder.feed(wire[split:])
        assert [item for item in events if isinstance(item, str)] == [
            "OK VERSION FW=1.7.0 PROTOCOL=1.3",
            "OK PING",
        ]
        assert len([item for item in events if isinstance(item, WaveBlock)]) == 1
        assert not any(isinstance(item, BadWave) for item in events)


def test_oversized_packet_recovery() -> None:
    """@brief 超长二进制缓存有界，下一帧可恢复。"""
    decoder = MixedDecoder()
    events = decoder.feed(b"\0" + b"x" * 10000 + b"\0" + encode_block(block()))
    assert isinstance(events[0], BadWave)
    assert isinstance(events[1], WaveBlock)
    assert len(decoder.encoded) <= 510


def test_c_fixture_matches_python_crc_and_quantization() -> None:
    """@brief 使用实际C模块生成的原文交叉核验；存在时必须严格一致。"""
    fixture = Path(__file__).resolve().parents[3] / "tem/wave_binary_20261023/c_fixture.bin"
    if not fixture.exists():
        pytest.skip("先运行固件纯C波形回归以生成fixture")
    frame = decode_block(fixture.read_bytes()[1:-1])
    assert frame.count == 32 and frame.step == 17000
    assert frame.sequence == 1
    np.testing.assert_array_equal(frame.currents(), block().currents())
    assert encode_block(frame) == fixture.read_bytes()


def test_array_ring_and_missing_samples() -> None:
    """@brief 缓存覆盖有界，重复不重画，缺样不补点。"""
    data = WaveBuffer(96)
    assert data.append(block())
    assert not data.append(block())
    assert data.duplicates == 1
    assert data.append(block(65))
    assert data.missing == 32
    times, _raw, gaps, sequence = data.current()
    assert len(times) == 64 and gaps[32]
    assert sequence[32] == 65
    assert data.append(block(97))
    assert data.append(block(129))
    assert data.count == 96
    assert data.current()[3][0] == 65


def test_clock_and_sequence_wrap() -> None:
    """@brief 32位序号与170MHz时标自然回绕后时间仍连续。"""
    data = WaveBuffer(96)
    first = block(0xFFFFFFE0, ticks=0xFFFF0000)
    assert data.append(first)
    second = block(0, ticks=(first.ticks + 32 * 17000) & 0xFFFFFFFF)
    assert data.append(second)
    np.testing.assert_allclose(data.current()[0], np.arange(64) / 10000, atol=1e-12)
    assert data.missing == 0


def test_slow_packet_before_first_fast_and_separate_rate(tmp_path: Path) -> None:
    """@brief 状态先到不能造成约25秒跳变，也不能填成高频状态。"""
    data = WaveBuffer()
    status = WaveBlock(
        2,
        VALID,
        1,
        0,
        170000,
        3400000,
        1,
        struct.pack("<iiHBBI", 600123, 600000, 12400, 1, 6, 170000),
    )
    assert data.append(status)
    assert data.statuses() == []
    assert data.append(block())
    assert len(data.statuses()) == 1
    assert data.statuses()[0].seconds == pytest.approx(0.0009)
    assert data.statuses()[0].rpm == 600.123
    paths = export_binary_csv(tmp_path / "actual.csv", data)
    assert len(paths[0].read_text(encoding="utf-8-sig").splitlines()) == 33
    assert len(paths[1].read_text(encoding="utf-8-sig").splitlines()) == 2
    assert '"filtered": false' in paths[2].read_text(encoding="utf-8")


def test_pause_snapshot_and_source_separation() -> None:
    """@brief 暂停副本保持不变，测试与真实会话不能混接。"""
    data = WaveBuffer()
    data.append(block())
    frozen = data.frozen()
    data.append(block(33))
    assert frozen.count == 32 and data.count == 64
    assert not data.append(block(65, flags=TEST))
    assert data.corrupt == 1
    assert data.append(block(session=2, flags=TEST))
    assert data.test and data.count == 32


def test_thirty_seconds_retained_at_five_kilohertz() -> None:
    """@brief 兼容档也按真实30秒截断，而不是保留60秒。"""
    data = WaveBuffer()
    for sequence in range(1, 310001, 64):
        assert data.append(block(sequence, step=34000))
    assert 149900 <= data.count <= 150001
    assert data.last_seconds - data.first_seconds <= 30.000001


def test_recorder_failure_does_not_block_receiver(tmp_path: Path) -> None:
    """@brief 磁盘路径错误可见，不抛到串口接收循环。"""
    recorder = BinaryRecorder(tmp_path / "missing" / "out.focwave")
    recorder.thread.join(timeout=1)
    assert recorder.error
    recorder.append(b"still receiving")
    recorder.close()


def test_bad_payload_is_rejected_before_numpy_decode() -> None:
    """@brief CRC正确但不合法的长度仍被拒绝。"""
    bad = WaveBlock(1, VALID, 1, 1, 17000, 17000, 32, b"short")
    with pytest.raises(ControlError, match="载荷"):
        decode_block(encode_block(bad)[1:-1])
