"""虚拟编码器的真实机械快照；计数和时间保持原始尺度，不做显示平滑。"""

from __future__ import annotations

from dataclasses import dataclass

from .models import ControlError, ParameterValue
from .protocol import fields

# 四项必须作为同一完整配置提交，不能以四条SET形成中间组合。
HAPTIC_KEYS = ("HAPTIC_KIND", "HAPTIC_DETENTS", "HAPTIC_STRENGTH", "HAPTIC_DAMPING")


@dataclass(frozen=True)
class HapticSample:
    """同一机械拍的真实状态，角度原码每圈2097152，时钟170MHz。"""

    kind: str
    detents: int
    strength_ma: int
    damping_ua_rpm: int
    limit_ma: int
    run_ms: int
    valid: bool
    running: bool
    transitioning: bool
    session: int
    raw: int
    ticks: int
    counts: int
    center: int
    index: int
    reference_a: float
    iq_a: float
    rpm: float

    @property
    def angle_deg(self) -> float:
        """@brief 按真实单圈原码换算角度。@return 机械角度。"""
        return self.raw * (360.0 / 2097152)

    @property
    def turns(self) -> float:
        """@brief 返回本次开启以来的真实净累计圈数。@return 有符号机械圈数。"""
        return self.counts / 2097152

    def parameters(self) -> dict[str, ParameterValue]:
        """@brief 返回已确认的四项配置供表单及预设复用。@return 参数字典。"""
        return dict(
            zip(
                HAPTIC_KEYS,
                (
                    ParameterValue(int(self.kind == "DAMPED"), 0, ""),
                    ParameterValue(self.detents, 0, ""),
                    ParameterValue(self.strength_ma, 0, "MA"),
                    ParameterValue(self.damping_ua_rpm, 3, "MA/RPM"),
                ),
                strict=True,
            )
        )

    @classmethod
    def parse(cls, response: str) -> HapticSample:
        """@brief 严格解析真实板卡快照。@param response 完整回执。@return 已校验样点。"""
        try:
            if not response.startswith("OK FOC HAPTIC "):
                raise ValueError("前缀无效")
            f = fields(response)
            kind = f.pop("KIND")
            if kind not in ("DETENT", "DAMPED"):
                raise ValueError("类型无效")
            v = {key: int(value) for key, value in f.items()}
            for key in ("VALID", "RUN", "TRANSITION"):
                if v[key] not in (0, 1):
                    raise ValueError("状态无效")
            if not 2 <= v["DETENTS"] <= 256 or not 0 <= v["DETENT_MA"] <= 800:
                raise ValueError("档位配置无效")
            if not 0 <= v["DAMPING_UA_RPM"] <= 10000 or not 500 <= v["LIMIT_MA"] <= 800:
                raise ValueError("阻尼或限流无效")
            if not 1000 <= v["RUN_MS"] <= 2147483647 or not 0 <= v["RAW"] < 2097152:
                raise ValueError("时限或原码无效")
            for key in ("SESSION", "TICKS", "POS_LO", "CENTER_LO", "INDEX_LO"):
                if not 0 <= v[key] <= 0xFFFFFFFF:
                    raise ValueError("无符号字段无效")
            for key in ("POS_HI", "CENTER_HI", "INDEX_HI", "REF_MA", "IQ_MA", "RPM_MILLI"):
                if not -0x80000000 <= v[key] <= 0x7FFFFFFF:
                    raise ValueError("有符号字段无效")
            return cls(
                kind,
                v["DETENTS"],
                v["DETENT_MA"],
                v["DAMPING_UA_RPM"],
                v["LIMIT_MA"],
                v["RUN_MS"],
                bool(v["VALID"]),
                bool(v["RUN"]),
                bool(v["TRANSITION"]),
                v["SESSION"],
                v["RAW"],
                v["TICKS"],
                (v["POS_HI"] << 32) | v["POS_LO"],
                (v["CENTER_HI"] << 32) | v["CENTER_LO"],
                (v["INDEX_HI"] << 32) | v["INDEX_LO"],
                v["REF_MA"] / 1000,
                v["IQ_MA"] / 1000,
                v["RPM_MILLI"] / 1000,
            )
        except (KeyError, ValueError) as error:
            raise ControlError("手感快照字段无效或不完整") from error


def config_command(parameters: dict[str, ParameterValue]) -> str:
    """@brief 编码完整原子配置。@param parameters 已校验四项参数。@return 单条命令。"""
    kind, count, strength, damping = (parameters[key].value for key in HAPTIC_KEYS)
    return f"MOTOR FOC HAPTIC CONFIG {'DETENT' if kind == 0 else 'DAMPED'} {count} {strength} {damping}"
