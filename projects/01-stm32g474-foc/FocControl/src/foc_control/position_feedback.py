"""原始多圈位置快照和源时标缓存，不做显示滤波或插值。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from .models import ControlError
from .protocol import fields


@dataclass(frozen=True)
class PositionSample:
    """同一机械坐标快照，角度单位为度。"""

    valid: bool
    zero_id: int
    raw: int
    counts: int
    ticks: int
    target_deg: float
    reference_rpm: float
    error_deg: float
    arrived: bool
    tolerance_deg: float
    running: bool

    @property
    def angle_deg(self) -> float:
        """按原码换算机械角，保留多圈符号。"""
        return self.counts * (360.0 / 2097152)

    @classmethod
    def parse(cls, response: str) -> PositionSample:
        """校验独立位置回执，不能把旧RPM或单圈角冒充多圈位置。"""
        try:
            if not response.startswith("OK FOC POSITION "):
                raise ValueError("位置响应前缀不符")
            values = {key: int(value) for key, value in fields(response).items()}
            for key in ("VALID", "ARRIVED", "RUN"):
                if values[key] not in (0, 1):
                    raise ValueError("状态无效")
            for key in ("ZERO_ID", "POS_LO", "TICKS"):
                if not 0 <= values[key] <= 0xFFFFFFFF:
                    raise ValueError("无符号字段无效")
            if not -0x80000000 <= values["POS_HI"] <= 0x7FFFFFFF:
                raise ValueError("高位无效")
            if not 0 <= values["RAW"] < 2097152 or values["TOL_MDEG"] not in (100, 500):
                raise ValueError("原码或容差无效")
            if any(
                abs(values[key]) > 0x7FFFFFFF for key in ("TARGET_MDEG", "REF_MRPM", "ERROR_MDEG")
            ):
                raise ValueError("有符号字段无效")
            return cls(
                bool(values["VALID"]),
                values["ZERO_ID"],
                values["RAW"],
                (values["POS_HI"] << 32) | values["POS_LO"],
                values["TICKS"],
                values["TARGET_MDEG"] / 1000,
                values["REF_MRPM"] / 1000,
                values["ERROR_MDEG"] / 1000,
                bool(values["ARRIVED"]),
                values["TOL_MDEG"] / 1000,
                bool(values["RUN"]),
            )
        except (KeyError, ValueError) as error:
            raise ControlError("位置快照字段无效或不完整") from error


@dataclass(frozen=True)
class PositionPoint:
    """运行期间的原始位置样点，时间来自MCU DWT。"""

    seconds: float
    sample: PositionSample
    continuous: bool


class PositionBuffer:
    """运行采样有界缓存；停机的DWT休眠时间不拼入运动曲线。"""

    def __init__(self, maximum: int = 6000) -> None:
        """创建默认五分钟20Hz缓存。"""
        self.points: deque[PositionPoint] = deque(maxlen=maximum)
        self.previous: PositionSample | None = None
        self.seconds = 0.0

    def clear(self) -> None:
        """新START清曲线，不改变设备机械零点。"""
        self.points.clear()
        self.previous = None
        self.seconds = 0.0

    def append(self, sample: PositionSample) -> PositionPoint | None:
        """只记录有效运行样点；缺口和零点变化断开曲线。"""
        if not sample.valid or not sample.running:
            self.previous = None
            return None
        previous = self.previous
        continuous = False
        if previous is not None and previous.zero_id == sample.zero_id:
            delta = (sample.ticks - previous.ticks) & 0xFFFFFFFF
            if delta == 0:
                return None
            if delta <= 0x7FFFFFFF:
                self.seconds += delta / 170000000
                continuous = delta <= 25500000
            else:
                self.clear()
        elif previous is not None:
            self.clear()
        point = PositionPoint(self.seconds, sample, continuous)
        self.points.append(point)
        self.previous = sample
        return point
