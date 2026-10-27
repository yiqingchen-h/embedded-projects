"""有界 ASCII 分帧、回执识别与 FOC 遥测时间轴。"""

from __future__ import annotations

import math
import re
from collections import deque
from dataclasses import dataclass
from decimal import Decimal

from .models import ControlError


def command_bytes(command: str, read_only: bool = False) -> bytes:
    """@brief 编码且检查命令。@param command 命令正文。@param read_only 只读模式。@return CRLF 字节。"""
    if not command or len(command) > 96 or re.fullmatch(r"[A-Z0-9_ .-]+", command) is None:
        raise ControlError("命令必须是单行 ASCII，且不超过 96 字节")
    if read_only and not is_read_command(command):
        raise ControlError(f"只读模式禁止发送：{command}")
    return (command + "\r\n").encode("ascii")


def is_read_command(command: str) -> bool:
    """@brief 判断严格只读白名单。@param command 命令。@return 是否只读。"""
    words = command.split()
    if command in (
        "PING",
        "VERSION",
        "HELP",
        "ENCODER",
        "STATUS",
        "INPUT",
        "STORAGE",
        "MOTOR",
        "MOTOR RUN",
        "MOTOR BEMF",
        "MOTOR ENCODER",
    ):
        return True
    if len(words) == 2 and words[0] == "GET":
        return True
    return command in (
        "MOTOR FOC",
        "MOTOR FOC DRIVE",
        "MOTOR FOC REST",
        "MOTOR FOC OBSERVER",
        "MOTOR FOC CONFIDENCE",
        "MOTOR FOC CURRENT",
        "MOTOR FOC POSITION",
        "MOTOR FOC HAPTIC",
        "MOTOR FOC VOLTAGE",
        "MOTOR FOC HARMONIC",
        "MOTOR FOC HARMONIC2",
        "MOTOR FOC TRIGGER",
    )


def fields(response: str) -> dict[str, str]:
    """@brief 按下一个键边界提取字段，保留ADC READ等多词故障。@param response 回执。
    @return 协议大写键及其完整值，空单位仍保留。
    """
    return {
        key: value.strip()
        for key, value in re.findall(
            r"(?:^| )([A-Z][A-Z0-9_]*)=(.*?)(?= [A-Z][A-Z0-9_]*=|$)", response
        )
    }


def expected_prefix(command: str) -> str:
    """@brief 构建回执匹配前缀。@param command 发送命令。@return 成功前缀。"""
    if command.startswith("GET "):
        return "OK GET " + command[4:] + "="
    if command.startswith("SET "):
        return "OK SET "
    # 批读起点和测试时限是请求参数，回执以命令名及键值字段报告结果。
    if command.startswith("MOTOR FOC HAPTIC"):
        return "OK FOC HAPTIC"
    if command.startswith("MOTOR FOC POSITION"):
        words = command.split()
        return "OK FOC POSITION" + (" " + words[3] if len(words) > 3 else "")
    if command.startswith("MOTOR FOC MOTION "):
        return "OK FOC MOTION"
    if command == "MOTOR UART TEST" or command.startswith("MOTOR UART TEST "):
        return "OK MOTOR UART TEST"
    return "OK " + command


def reply_matches(command: str, response: str) -> bool:
    """@brief 防止迟到回执错配。@param command 命令。@param response 完整回执。@return 是否匹配。"""
    if response.startswith("ERR LINE "):
        return True
    if response == "ERR RUNNING STOP TO READ":
        return command.split()[0] in ("STATUS", "MOTOR", "INPUT", "STORAGE", "HELP")
    if response.startswith("ERR "):
        return response.split()[1] in (
            command.split()[0],
            "FOC" if "FOC" in command else "",
            "ENCODER" if command.startswith("MOTOR ENCODER") else "",
        )
    # 固件中编码器标定使用 OK ENCODER，查询使用 OK MOTOR ENCODER。
    if command.startswith("MOTOR ENCODER ") and response.startswith("OK ENCODER "):
        return response.startswith("OK " + command.removeprefix("MOTOR "))
    prefix = expected_prefix(command)
    return response.startswith(prefix) and (
        command.startswith(("GET ", "SET "))
        or len(response) == len(prefix)
        or response[len(prefix)] in (" ", "=")
    )


@dataclass(frozen=True)
class BadLine:
    """保留损坏行的可读原文，不把替换字符解释成有效数据。"""

    raw: str
    reason: str


class LineDecoder:
    """逐字节流分帧，损坏和超长行恢复到下一可信 LF。"""

    def __init__(self, maximum: int = 8192) -> None:
        """@brief 初始化有界缓存。@param maximum 最大行长度。"""
        self.maximum = maximum
        self.pending = bytearray()
        self.discarding = False

    def feed(self, data: bytes) -> list[str | BadLine]:
        """@brief 处理任意分包。@param data 新字节。@return 完整行或错误。"""
        result: list[str | BadLine] = []
        for byte in data:
            if self.discarding:
                if byte == 10:
                    self.discarding = False
                continue
            if byte == 10:
                raw = bytes(self.pending).removesuffix(b"\r")
                self.pending.clear()
                try:
                    line = raw.decode("ascii")
                except UnicodeDecodeError:
                    result.append(BadLine(raw.hex(), "非 ASCII 字节"))
                    continue
                if any(ord(char) < 32 for char in line):
                    result.append(BadLine(line, "非法控制字符"))
                elif line:
                    result.append(line)
                else:
                    result.append(BadLine("", "空响应"))
            else:
                self.pending.append(byte)
                if len(self.pending) > self.maximum:
                    result.append(BadLine(bytes(self.pending[:128]).hex(), "响应行超长"))
                    self.pending.clear()
                    self.discarding = True
        return result


@dataclass(frozen=True)
class Telemetry:
    """统一电流遥测；None 表示格式没有提供该字段。"""

    sample: int | None
    ticks: int | None
    stage: int | None
    ia_ma: float
    ib_ma: float
    ic_ma: float
    isc_ma: float
    id_ma: float | None = None
    iq_ma: float | None = None
    ref_d_ma: float | None = None
    ref_q_ma: float | None = None
    rpm: float | None = None
    bus_mv: float | None = None


def parse_telemetry(line: str, poles: int = 3) -> Telemetry | None:
    """@brief 解析三种现有打印格式。@param line 完整行。@param poles 极对数。@return 遥测或非遥测 None。"""
    if line.startswith("FOC_TRACE:"):
        parts = line.removeprefix("FOC_TRACE:").split(",")
        if len(parts) < 27 or any(re.fullmatch(r"-?\d+", value) is None for value in parts):
            raise ControlError("TRACE 必须至少包含 27 个完整整数")
        # 已定义字段保持原序，新增尾字段记录原文但不改变旧字段含义。
        values = [int(value) for value in parts[:27]]
        if any(abs(value) > 0xFFFFFFFF for value in values) or not 0 <= values[3] <= 6:
            raise ControlError("TRACE 字段超出有效范围")
        if any(values[index] < 0 for index in (0, 1, 2, 4, 5, 6, 21, 22, 23, 24, 26)):
            raise ControlError("TRACE 无符号字段无效")
        if poles <= 0:
            raise ControlError("极对数无效，无法换算机械转速")
        return Telemetry(
            values[0],
            values[1],
            values[3],
            values[7],
            values[8],
            values[9],
            values[10],
            values[11],
            values[12],
            values[13],
            values[14],
            values[16] * 60 / (2 * math.pi * 1000 * poles),
            float(values[22]),
        )
    if line.startswith("DATA FOC "):
        data = fields(line)
        try:
            frame = Telemetry(
                int(data["SAMPLE"]),
                int(data["TICKS"]),
                int(data["STAGE_ID"]) if "STAGE_ID" in data else None,
                float(data["IA_MA"]),
                float(data["IB_MA"]),
                float(data["IC_MA"]),
                float(data["ISC_MA"]),
                rpm=float(data["EST_RPM"]) if "EST_RPM" in data else None,
            )
            if not all(
                math.isfinite(value)
                for value in (frame.ia_ma, frame.ib_ma, frame.ic_ma, frame.isc_ma)
            ):
                raise ControlError("DATA FOC 包含非有限数值")
            if frame.rpm is not None and not math.isfinite(frame.rpm):
                raise ControlError("DATA FOC 转速无效")
            if frame.stage is not None and not 0 <= frame.stage <= 6:
                raise ControlError("DATA FOC 阶段无效")
            return frame
        except (KeyError, ValueError) as error:
            raise ControlError("DATA FOC 字段不完整") from error
    if line.startswith("FOC:"):
        try:
            values_float = [float(Decimal(value)) for value in line[4:].split(",")]
        except (ValueError, ArithmeticError) as error:
            raise ControlError("FireWater 数值无效") from error
        if len(values_float) != 4 or not all(math.isfinite(value) for value in values_float):
            raise ControlError("FireWater 必须包含四个有限数值")
        return Telemetry(
            None, None, None, values_float[0], values_float[1], values_float[2], values_float[3]
        )
    return None


@dataclass(frozen=True)
class PlotPoint:
    """用于图表和 CSV 的一条有效记录，时间单位为秒。"""

    seconds: float
    received_at: str
    frame: Telemetry
    target_rpm: float | None


class TelemetryBuffer:
    """保留最多指定条数的遥测；样本跳号不补造数据。"""

    def __init__(self, maximum: int = 20000) -> None:
        """@brief 创建环形缓存。@param maximum 样本上限。"""
        self.points: deque[PlotPoint] = deque(maxlen=maximum)
        self.last_sample: int | None = None
        self.last_ticks: int | None = None
        self.seconds = 0.0
        self.origin_received: float | None = None

    def clear(self) -> None:
        """@brief 清除当前运行时间轴。"""
        self.points.clear()
        self.last_sample = self.last_ticks = None
        self.seconds = 0.0
        self.origin_received = None

    def append(
        self,
        frame: Telemetry,
        received: float,
        stamp: str,
        target: float | None,
    ) -> PlotPoint | None:
        """@brief 加入新样本并展开 32 位 tick。@param frame 遥测。@param received 主机单调时钟。
        @param stamp 日志时间。@param target 同期目标。@return 新记录，重复样本为 None。
        """
        if frame.sample is not None and self.last_sample is not None:
            delta = (frame.sample - self.last_sample) & 0xFFFFFFFF
            if delta == 0:
                return None
            if delta > 0x7FFFFFFF:
                self.clear()
        if self.origin_received is None:
            self.origin_received = received
        if frame.ticks is not None and self.last_ticks is not None:
            delta_tick = (frame.ticks - self.last_ticks) & 0xFFFFFFFF
            if delta_tick > 0x7FFFFFFF:
                self.clear()
                self.origin_received = received
            else:
                self.seconds += delta_tick / 170_000_000
        elif frame.ticks is None:
            self.seconds = received - self.origin_received
        self.last_sample, self.last_ticks = frame.sample, frame.ticks
        point = PlotPoint(self.seconds, stamp, frame, target)
        self.points.append(point)
        return point
