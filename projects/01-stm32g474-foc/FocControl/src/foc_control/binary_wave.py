"""COBS波形块、混合串口分帧与30秒数组缓存；不插值或平滑。"""

from __future__ import annotations

import csv
import json
import queue
import struct
import threading
import time
import zlib
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from .models import ControlError
from .protocol import BadLine, LineDecoder

HEADER = struct.Struct("<4sBBHHHIIII")
CLOCK = 170_000_000
TEST = 1
CLIPPED = 2
VALID = 4
CHANNELS = ("ia", "ib", "isc", "id", "iq", "rd", "rq")
FloatArray = NDArray[np.float64]
IntArray = NDArray[np.int16]


@dataclass(frozen=True)
class WaveBlock:
    """完整CRC验证后的原始块；payload仍保留线上Q13整数。"""

    kind: int
    flags: int
    session: int
    sequence: int
    ticks: int
    step: int
    count: int
    payload: bytes

    def currents(self) -> IntArray:
        """@brief 零拷贝读取七路Q13数据。@return 样点×通道的只读数组。"""
        return np.frombuffer(self.payload, dtype="<i2").reshape(self.count, 7)


@dataclass(frozen=True)
class BadWave:
    """损坏遥测只影响观测完整性，不冒充命令回执。"""

    reason: str


def cobs_encode(data: bytes) -> bytes:
    """@brief 编码单个COBS载荷。@param data 原始字节。@return 无零字节编码。"""
    out = bytearray(b"\0")
    code_at, code = 0, 1
    for value in data:
        if value == 0:
            out[code_at] = code
            code_at = len(out)
            out.append(0)
            code = 1
        else:
            out.append(value)
            code += 1
            if code == 255:
                out[code_at] = code
                code_at = len(out)
                out.append(0)
                code = 1
    out[code_at] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """@brief 有界解码COBS。@param data 无零编码。@return 校验前原始字节。"""
    if not data or len(data) > 510:
        raise ControlError("COBS长度无效")
    out, index = bytearray(), 0
    while index < len(data):
        code = data[index]
        index += 1
        end = index + code - 1
        if code == 0 or end > len(data):
            raise ControlError("COBS编码损坏")
        out.extend(data[index:end])
        index = end
        if code < 255 and index < len(data):
            out.append(0)
    return bytes(out)


def decode_block(encoded: bytes) -> WaveBlock:
    """@brief 校验CRC、长度、类型和采样间隔。@param encoded COBS编码。@return 完整块。"""
    data = cobs_decode(encoded)
    if not 32 <= len(data) <= 480:
        raise ControlError("波形块长度无效")
    if zlib.crc32(data[:-4]) != struct.unpack_from("<I", data, len(data) - 4)[0]:
        raise ControlError("波形CRC错误")
    magic, version, kind, flags, size, count, session, sequence, ticks, step = HEADER.unpack_from(
        data
    )
    if magic != b"FCV1" or version != 1 or flags & ~7 or size + 32 != len(data):
        raise ControlError("波形包头无效")
    if kind == 1:
        valid = 1 <= count <= 32 and size == count * 14 and step in (17_000, 34_000)
    elif kind == 2:
        valid = count == 1 and size == 16
    elif kind == 3:
        valid = count == 0 and size == 32
    else:
        valid = False
    if not valid:
        raise ControlError("波形类型或载荷不匹配")
    return WaveBlock(kind, flags, session, sequence, ticks, step, count, data[28:-4])


def encode_block(block: WaveBlock) -> bytes:
    """@brief 为模拟板卡和交叉回归生成明确来源的数据。@param block 块。@return 线上帧。"""
    data = (
        HEADER.pack(
            b"FCV1",
            1,
            block.kind,
            block.flags,
            len(block.payload),
            block.count,
            block.session,
            block.sequence,
            block.ticks,
            block.step,
        )
        + block.payload
    )
    return b"\0" + cobs_encode(data + struct.pack("<I", zlib.crc32(data))) + b"\0"


class MixedDecoder:
    """单接收通道区分ASCII行和零分隔二进制，二进制内容不送文本解析。"""

    def __init__(self) -> None:
        """@brief 初始化固定边界与有界缓存。"""
        self.lines = LineDecoder()
        self.binary = False
        self.encoded = bytearray()
        self.oversized = False

    def feed(self, data: bytes) -> list[str | BadLine | WaveBlock | BadWave]:
        """@brief 接受任意串口分包。@param data 新字节。@return 已分离的事件。"""
        result: list[str | BadLine | WaveBlock | BadWave] = []
        start = 0
        while start < len(data):
            boundary = data.find(b"\0", start)
            end = len(data) if boundary < 0 else boundary
            part = data[start:end]
            if self.binary:
                first = self.encoded[0] if self.encoded else part[0] if part else 0
                if first in b"OEFD":
                    candidate = bytes(self.encoded) + part
                    if candidate.startswith(
                        (b"OK ", b"ERR ", b"FOC_TRACE:", b"DATA FOC ", b"FOC:")
                    ):
                        # 烧录后已观察到单个前导NUL；有效FCV1的COBS首字节为08，不与文本前缀混淆。
                        self.encoded.clear()
                        self.oversized = False
                        self.binary = False
                        part = candidate
            if self.binary:
                if len(self.encoded) + len(part) > 510:
                    self.oversized = True
                elif not self.oversized:
                    self.encoded.extend(part)
            else:
                result.extend(self.lines.feed(part))
            if boundary < 0:
                break
            if self.binary:
                if self.encoded or self.oversized:
                    try:
                        if self.oversized:
                            raise ControlError("二进制块超长")
                        result.append(decode_block(bytes(self.encoded)))
                    except ControlError as error:
                        result.append(BadWave(str(error)))
                    self.binary = False
                self.encoded.clear()
                self.oversized = False
            else:
                if self.lines.pending:
                    # 从帧中途连接时，残余载荷不是命令错误；零边界重新同步。
                    result.append(BadWave("二进制边界前有不完整数据"))
                self.lines = LineDecoder()
                self.binary = True
            start = boundary + 1
        return result


@dataclass(frozen=True)
class SlowPoint:
    """原生50Hz状态，保留有效性与来源。"""

    seconds: float
    rpm: float | None
    target: float
    bus: float
    running: bool
    stage: int
    test: bool


class WaveBuffer:
    """后台写入、界面只读的有界数组；不创建高频逐点对象。"""

    def __init__(self, capacity: int = 300_000) -> None:
        """@brief 初始化最多30秒数据。@param capacity 数组样点上限。"""
        self.capacity = capacity
        self.time: FloatArray = np.empty(capacity, dtype=np.float64)
        self.values: IntArray = np.empty((capacity, 7), dtype=np.int16)
        self.sequence: NDArray[np.uint32] = np.empty(capacity, dtype=np.uint32)
        self.gaps: NDArray[np.bool_] = np.empty(capacity, dtype=np.bool_)
        self.lock = threading.RLock()
        self.slow: deque[SlowPoint] = deque(maxlen=1600)
        self.revision = 0
        self.clear()

    def clear(self) -> None:
        """@brief 清理显示会话，原始文件由独立记录器保留。"""
        with self.lock:
            self.head = self.count = 0
            self.session: int | None = None
            self.last_tick: int | None = None
            self.last_sequence: int | None = None
            self.absolute_tick = 0
            self.origin_tick = 0
            self.step = 17_000
            self.received = self.missing = self.duplicates = self.corrupt = self.clipped = 0
            self.pending_gap = False
            self.test = False
            self.ended = False
            self.end_stats: tuple[int, ...] = ()
            self.slow.clear()
            self.pending_status: deque[WaveBlock] = deque(maxlen=8)
            self.revision += 1

    def mark_corrupt(self) -> None:
        """@brief 标记CRC或帧损坏，下一批不跨缺口连线。"""
        with self.lock:
            self.corrupt += 1
            self.pending_gap = True
            self.revision += 1

    @property
    def last_seconds(self) -> float:
        """@brief 最近真实样点时间。@return 秒。"""
        with self.lock:
            return float(self.time[(self.head - 1) % self.capacity]) if self.count else 0.0

    @property
    def first_seconds(self) -> float:
        """@brief 当前缓存起点。@return 秒。"""
        with self.lock:
            return float(self.time[(self.head - self.count) % self.capacity]) if self.count else 0.0

    def append(self, block: WaveBlock) -> bool:
        """@brief 批量加入源样点，拒绝重复与倒序。@param block 完整块。@return 接受。"""
        with self.lock:
            if block.session != self.session:
                self.clear()
                self.session = block.session
                self.test = bool(block.flags & TEST)
                self.origin_tick = block.ticks
            if bool(block.flags & TEST) != self.test:
                self.mark_corrupt()
                return False
            if block.kind == 3:
                self.end_stats = struct.unpack("<8I", block.payload)
                self.ended = True
                self.revision += 1
                return True
            if block.kind == 2:
                if self.last_tick is None:
                    self.pending_status.append(block)
                    return True
                rpm, target, bus, run, stage, source_tick = struct.unpack("<iiHBBI", block.payload)
                reference = self.last_tick if self.last_tick is not None else self.origin_tick
                offset = (source_tick - reference + 0x80000000) % 0x100000000 - 0x80000000
                seconds = (self.absolute_tick + offset) / CLOCK
                self.slow.append(
                    SlowPoint(
                        seconds,
                        rpm / 1000 if block.flags & VALID else None,
                        target / 1000,
                        bus / 1000,
                        bool(run),
                        stage,
                        self.test,
                    )
                )
                self.revision += 1
                return True
            gap = self.pending_gap
            if self.last_sequence is not None and self.last_tick is not None:
                distance = (block.sequence - self.last_sequence) & 0xFFFFFFFF
                if distance == 0 or distance > 0x7FFFFFFF:
                    self.duplicates += 1
                    return False
                stride = block.step // 17_000
                if distance != stride:
                    self.missing += max(1, distance // stride - 1)
                    gap = True
                delta = (block.ticks - self.last_tick) & 0xFFFFFFFF
                if delta > 0x7FFFFFFF or delta != distance * 17_000:
                    self.mark_corrupt()
                    return False
                self.absolute_tick += delta
            else:
                self.origin_tick = block.ticks
                self.absolute_tick = 0
            times = (
                self.absolute_tick + np.arange(block.count, dtype=np.float64) * block.step
            ) / CLOCK
            sequence = (
                block.sequence + np.arange(block.count, dtype=np.uint64) * (block.step // 17_000)
            ).astype(np.uint32)
            index = (self.head + np.arange(block.count)) % self.capacity
            self.time[index] = times
            self.values[index] = block.currents()
            self.sequence[index] = sequence
            self.gaps[index] = False
            self.gaps[index[0]] = gap
            self.head = (self.head + block.count) % self.capacity
            self.count = min(self.capacity, self.count + block.count)
            while (
                self.count and times[-1] - self.time[(self.head - self.count) % self.capacity] > 30
            ):
                self.count -= 1
            self.step = block.step
            self.last_tick = (block.ticks + (block.count - 1) * block.step) & 0xFFFFFFFF
            self.absolute_tick += (block.count - 1) * block.step
            self.last_sequence = int(sequence[-1])
            self.received += block.count
            self.clipped += bool(block.flags & CLIPPED)
            self.pending_gap = False
            self.revision += 1
            while self.pending_status:
                self.append(self.pending_status.popleft())
            return True

    def current(
        self, low: float = -float("inf"), high: float = float("inf")
    ) -> tuple[FloatArray, IntArray, NDArray[np.bool_], NDArray[np.uint32]]:
        """@brief 只复制可见范围，环回最多处理两段。@param low 起点。@param high 终点。@return 原始数组。"""
        with self.lock:
            start = (self.head - self.count) % self.capacity
            ranges = (
                [(start, start + self.count)]
                if start + self.count <= self.capacity
                else [(start, self.capacity), (0, self.head)]
            )
            indices: list[NDArray[np.int64]] = []
            for left, right in ranges:
                times = self.time[left:right]
                a, b = np.searchsorted(times, (low, high), side="left")
                if b < len(times) and times[b] <= high:
                    b += 1
                indices.append(np.arange(left + a, left + b, dtype=np.int64))
            chosen = np.concatenate(indices)
            return (
                self.time[chosen].copy(),
                self.values[chosen].copy(),
                self.gaps[chosen].copy(),
                self.sequence[chosen].copy(),
            )

    def statuses(self) -> list[SlowPoint]:
        """@brief 读取独立慢速序列快照。@return 最多30秒左右状态。"""
        with self.lock:
            return list(self.slow)

    def frozen(self) -> WaveBuffer:
        """@brief 暂停时复制有界快照，后台继续接收。@return 独立缓存。"""
        with self.lock:
            other = WaveBuffer(self.capacity)
            for name in (
                "head",
                "count",
                "session",
                "last_tick",
                "last_sequence",
                "absolute_tick",
                "origin_tick",
                "step",
                "received",
                "missing",
                "duplicates",
                "corrupt",
                "clipped",
                "pending_gap",
                "test",
                "ended",
                "end_stats",
                "revision",
            ):
                setattr(other, name, getattr(self, name))
            other.time[:] = self.time
            other.values[:] = self.values
            other.sequence[:] = self.sequence
            other.gaps[:] = self.gaps
            other.slow.extend(self.slow)
            return other


class BinaryRecorder:
    """独立写盘线程，有界8MiB队列；失败显式报告而不阻塞串口接收。"""

    def __init__(self, path: Path) -> None:
        """@brief 创建原始RX记录。@param path 新文件。"""
        self.path = path
        self.error = ""
        self.pending: queue.Queue[bytes | None] = queue.Queue(maxsize=2048)
        self.thread = threading.Thread(target=self._write, name="波形原始记录", daemon=True)
        self.thread.start()

    def append(self, data: bytes) -> None:
        """@brief 非阻塞提交至多4096字节的读取批次。@param data 原始RX。"""
        if not data or self.error:
            return
        try:
            self.pending.put_nowait(bytes(data))
        except queue.Full:
            self.error = "波形记录队列已满，原始记录不完整；串口仍在接收"

    def _write(self) -> None:
        """@brief 后台顺序保存原文，磁盘错误保留原因。"""
        try:
            with self.path.open("xb") as output:
                while True:
                    data = self.pending.get()
                    if data is None:
                        break
                    output.write(data)
        except OSError as error:
            self.error = f"波形记录失败：{error}"

    def close(self) -> None:
        """@brief 接收停止后有界等待写盘，失败不会伪报完整。"""
        if not self.thread.is_alive():
            return
        try:
            self.pending.put(None, timeout=2)
        except queue.Full:
            self.error = self.error or "关闭时记录队列未能排空"
            return
        self.thread.join(timeout=3)
        if self.thread.is_alive():
            self.error = self.error or "原始波形记录尚未完成写盘"


def export_binary_csv(path: Path, buffer: WaveBuffer) -> tuple[Path, Path, Path]:
    """@brief 分别导出原生高速电流和低速状态。@param path 电流CSV。@param buffer 快照。@return 三文件。"""
    times, raw, gaps, sequence = buffer.current()
    status_path = path.with_name(path.stem + "_status.csv")
    meta_path = path.with_suffix(".json")
    with path.open("w", newline="", encoding="utf-8-sig") as output:
        writer = csv.writer(output)
        writer.writerow(
            (
                "time_s",
                "sample",
                "ia_a",
                "ib_a",
                "ic_reconstructed_a",
                "isc_a",
                "id_a",
                "iq_a",
                "ref_d_a",
                "ref_q_a",
                "gap_before",
                "source",
            )
        )
        # 分批向量化换算，批次间让出GIL，避免导出占满界面线程的执行机会。
        source = "TEST" if buffer.test else "MOTOR"
        fmt = "%.9f,%u," + "%.9f," * 8 + "%u," + source
        for first in range(0, len(times), 4096):
            end = min(first + 4096, len(times))
            values = raw[first:end].astype(np.float64) / 8192
            rows = np.column_stack(
                (
                    times[first:end],
                    sequence[first:end],
                    values[:, 0],
                    values[:, 1],
                    -values[:, 0] - values[:, 1],
                    values[:, 2],
                    values[:, 3],
                    values[:, 4],
                    values[:, 5],
                    values[:, 6],
                    gaps[first:end],
                )
            )
            np.savetxt(output, rows, fmt=fmt)
            time.sleep(0)
    with status_path.open("w", newline="", encoding="utf-8-sig") as output:
        writer = csv.writer(output)
        writer.writerow(("time_s", "rpm", "target_rpm", "bus_v", "running", "stage", "source"))
        for point in buffer.statuses():
            writer.writerow(
                (
                    point.seconds,
                    point.rpm,
                    point.target,
                    point.bus,
                    point.running,
                    point.stage,
                    "TEST" if point.test else "MOTOR",
                )
            )
    metadata: dict[str, Any] = {
        "format": "BIN1",
        "session": buffer.session,
        "source": "TEST" if buffer.test else "MOTOR",
        "rate_hz": CLOCK / buffer.step,
        "current_q": 13,
        "received": buffer.received,
        "missing": buffer.missing,
        "corrupt": buffer.corrupt,
        "clipped_blocks": buffer.clipped,
        "end_stats": buffer.end_stats,
        "filtered": False,
        "time_source": "board_ticks",
        "files": [path.name, status_path.name],
    }
    meta_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    return path, status_path, meta_path
