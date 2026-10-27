"""板卡参数与界面快照；所有写入使用整数尺度。"""

from __future__ import annotations

import re
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from decimal import Decimal, InvalidOperation
from typing import TYPE_CHECKING, TypedDict

if TYPE_CHECKING:
    from .haptic_feedback import HapticSample
    from .position_feedback import PositionSample

# 仅作为旧版演示板卡的身份；实际连接按协议与读取到的能力判断。
FIRMWARE = "1.4.6-serial"
PROTOCOL = "1.2"
ALGORITHMS = {
    "SENSORLESS_FOC": "无感 FOC",
    "SENSORLESS_6STEP": "无感六步",
    "OPEN_6STEP": "开环六步",
    "ENCODER_FOC": "有感 FOC（编码器）",
    "ENCODER_6STEP": "有感六步（编码器）",
}
# FOC 共用独立电流/速度参数块，有感模式使用显式七点资格。
FOC_ALGORITHMS = ("SENSORLESS_FOC", "ENCODER_FOC")
ENCODER_ALGORITHMS = ("ENCODER_FOC", "ENCODER_6STEP")
MODES = {
    "SPEED": "速度闭环",
    "CURRENT": "Iq 电流闭环",
    "POSITION": "多圈位置闭环",
    "HAPTIC": "虚拟编码器",
}


def mode_supported(algorithm: str, mode: str) -> bool:
    """@brief 核对已实现的算法/模式组合。@param algorithm 算法。@param mode 模式。@return 是否已适配。"""
    return algorithm in ALGORITHMS and (
        mode == "SPEED"
        or (algorithm == "ENCODER_FOC" and mode in ("CURRENT", "POSITION", "HAPTIC"))
    )


# 这些字段缺失或尺度变化时，仅保留监视与停止，避免错误单位下控制。
REQUIRED_PARAMETERS = frozenset(("TARGET", "CURRENT_LIMIT", "RUN_TIME"))
# 电机身份参与预设恢复校验，派生增益与会话零点不作为版本锁。
MODEL_PARAMETERS = frozenset(
    (
        "POLE_PAIRS",
        "SHUNT",
        "SPEED_LIMIT",
        "RESISTANCE",
        "LD",
        "LQ",
        "RATED_VOLTAGE",
        "RATED_CURRENT",
        "RATED_SPEED",
    )
)
STAGES = ("停机", "校准", "定位", "升频", "资格确认", "融合", "闭环")


def protocol_supported(protocol: str) -> bool:
    """@brief 接受同主版本的向后兼容协议。@param protocol 板卡协议。@return 是否可控。"""
    match = re.fullmatch(r"(\d+)\.(\d+)", protocol)
    return match is not None and int(match[1]) == 1 and int(match[2]) >= 2


class ControlError(ValueError):
    """参数、状态或协议前提不满足，停止后续写入。"""


@dataclass(frozen=True)
class ParameterValue:
    """GET 返回的原始整数、尺度和单位。"""

    # 原始整数，直接用于 SET。
    value: int
    # 固件返回的小数位数。
    decimals: int
    # 固件返回的单位。
    unit: str

    @classmethod
    def parse(cls, response: str, key: str) -> ParameterValue:
        """@brief 严格解析数值 GET。@param response 回执。@param key 参数名。@return 参数值。"""
        match = re.fullmatch(
            rf"OK GET {re.escape(key)}=(-?\d+)((?: [A-Z_]+=[^ ]*)+)",
            response,
        )
        if match is None:
            raise ControlError(f"{key} 回执不完整：{response}")
        tokens = [token.split("=", 1) for token in match[2].strip().split(" ")]
        metadata = dict(tokens)
        if (
            len(metadata) != len(tokens)
            or metadata.get("DECIMALS") not in ("0", "1", "2", "3")
            or "UNIT" not in metadata
        ):
            raise ControlError(f"{key} 回执的单位或尺度不完整：{response}")
        return cls(int(match[1]), int(metadata["DECIMALS"]), metadata["UNIT"])


@dataclass(frozen=True)
class ParameterSpec:
    """当前固件参数能力及界面单位。"""

    # 串口参数名和中文标签。
    key: str
    label: str
    # 允许写入的原始范围。
    minimum: int
    maximum: int
    # 固件尺度和单位。
    decimals: int
    unit: str
    # 所属界面分组。
    group: str = "高级参数"
    # 是否允许编辑，固定模型和预留项只读。
    editable: bool = True
    # 界面单位转换，例如毫秒转秒。
    factor: Decimal = Decimal(1)
    display_unit: str = ""

    def validate(self, value: ParameterValue) -> None:
        """@brief 核对范围及单位。@param value 待写参数。"""
        if (value.decimals, value.unit) != (self.decimals, self.unit):
            raise ControlError(f"{self.label} 的单位或尺度与固件不符")
        if not self.minimum <= value.value <= self.maximum:
            raise ControlError(f"{self.label} 超出允许范围")
        if self.key == "SHUNT" and value.value not in (1, 50):
            raise ControlError("采样电阻只接受 1 或 50 mΩ")
        if self.key == "ENCODER_DIR" and value.value not in (-1, 1):
            raise ControlError("编码器方向只接受 -1 或 1")

    def physical(self, value: ParameterValue) -> Decimal:
        """@brief 换算界面值。@param value 原始参数。@return 物理量。"""
        return Decimal(value.value).scaleb(-value.decimals) * self.factor

    def encode(self, text: str) -> ParameterValue:
        """@brief 精确转换输入。@param text 物理量字符串。@return 原始参数。"""
        try:
            number = Decimal(text.strip())
            raw = number / self.factor * (10**self.decimals)
        except (InvalidOperation, ZeroDivisionError) as error:
            raise ControlError(f"{self.label} 必须是十进制数字") from error
        if not number.is_finite() or raw != raw.to_integral_value():
            raise ControlError(f"{self.label} 的小数精度超出固件能力")
        value = ParameterValue(int(raw), self.decimals, self.unit)
        self.validate(value)
        if not self.editable:
            raise ControlError(f"{self.label} 为只读参数")
        return value

    def format(self, value: ParameterValue) -> str:
        """@brief 格式化输入框值。@param value 参数。@return 不含单位的文本。"""
        return format(self.physical(value).normalize(), "f")

    @property
    def suffix(self) -> str:
        """@brief 获取界面单位。@return 单位文本。"""
        translations = {
            "RPM": "RPM",
            "A": "A",
            "MS": "ms",
            "HZ": "Hz",
            "HZ/S": "Hz/s",
            "RPM/S": "RPM/s",
            "MA/RPM": "mA/RPM",
            "MA/R/S": "mA/(RPM·s)",
            "D/RPM": "‰/RPM",
            "D/RPMS": "‰/(RPM·s)",
            "MOHM": "mΩ",
            "OHM": "Ω",
            "MH": "mH",
            "MWB": "mWb",
            "DEG": "°",
            "ADC": "ADC",
            "V": "V",
        }
        return self.display_unit or translations.get(self.unit, self.unit)


def spec(
    key: str,
    label: str,
    low: int,
    high: int,
    decimals: int,
    unit: str,
    group: str = "高级参数",
    editable: bool = True,
) -> ParameterSpec:
    """@brief 创建统一参数描述。@param key 串口名。@param label 标签。@param low 下限。
    @param high 上限。@param decimals 尺度。@param unit 单位。@param group 分组。
    @param editable 是否可写。@return 参数描述。
    """
    return ParameterSpec(
        key,
        label,
        low,
        high,
        decimals,
        unit,
        group,
        editable,
        Decimal("0.001") if key == "RUN_TIME" else Decimal(1),
        "s" if key == "RUN_TIME" else "",
    )


# 范围取自 settings.c/motor_config.h；未实现参数仍能备份但不在 GUI 中开放编辑。
SIXSTEP = {
    p.key: p
    for p in (
        spec("TARGET", "目标转速", 0, 5400, 0, "RPM", "常用参数"),
        spec("CURRENT_LIMIT", "相电流限幅", 100, 1500, 3, "A", "常用参数"),
        spec("DUTY", "定位 / 牵引占空比", 1, 250, 1, "%", "常用参数"),
        spec("RUN_TIME", "运行时长", 100, 30000, 0, "MS", "常用参数"),
        spec("START_HZ", "起始电频率", 100, 50000, 3, "HZ", "启动参数"),
        spec("RAMP", "电频率斜坡", 100, 100000, 3, "HZ/S", "启动参数"),
        spec("ALIGN_TIME", "定位时长", 10, 1000, 0, "MS", "启动参数"),
        spec("SPEED_KP", "速度 Kp", 0, 100000, 3, "D/RPM", "速度环"),
        spec("SPEED_KI", "速度 Ki", 0, 100000, 3, "D/RPMS", "速度环"),
        spec("HANDOFF", "接管阈值", 0, 5400, 0, "RPM", "启动参数"),
        spec("COMM_TIMING", "换相补偿", -300, 300, 1, "DEG", "启动参数"),
        spec("POLE_PAIRS", "极对数", 0, 32, 0, "", "电机参数"),
        spec("SHUNT", "采样电阻", 1, 50, 0, "MOHM", "电机参数"),
        spec("SPEED_LIMIT", "转速上限", 1, 5400, 0, "RPM", "电机参数"),
        spec("RATED_VOLTAGE", "额定电压", 8000, 15000, 3, "V", "电机参数"),
        spec("RATED_CURRENT", "额定电流", 0, 10000, 3, "A", "电机参数"),
        spec("RATED_SPEED", "额定转速", 1, 20000, 0, "RPM", "电机参数"),
        spec("POSITION_MIN", "位置下限（预留）", -36000, 36000, 1, "DEG", "预留参数", False),
        spec("POSITION_MAX", "位置上限（预留）", -36000, 36000, 1, "DEG", "预留参数", False),
        spec("D_KP", "电流 D Kp（预留）", 0, 100000, 3, "", "预留参数", False),
        spec("D_KI", "电流 D Ki（预留）", 0, 100000, 3, "", "预留参数", False),
        spec("Q_KP", "电流 Q Kp（预留）", 0, 100000, 3, "", "预留参数", False),
        spec("Q_KI", "电流 Q Ki（预留）", 0, 100000, 3, "", "预留参数", False),
        spec("POSITION_KP", "位置 Kp（预留）", 0, 100000, 3, "", "预留参数", False),
        spec("POSITION_KI", "位置 Ki（预留）", 0, 100000, 3, "", "预留参数", False),
        spec("POSITION_KD", "位置 Kd（预留）", 0, 100000, 3, "", "预留参数", False),
        spec("ENCODER_DIR", "编码器方向（预留）", -1, 1, 0, "", "预留参数", False),
        spec("ELEC_ZERO", "电角零点（预留）", 0, 3599, 1, "DEG", "预留参数", False),
        spec("RESISTANCE", "电阻（预留）", 0, 100000, 3, "OHM", "预留参数", False),
        spec("LD", "D 轴电感（预留）", 0, 100000, 3, "MH", "预留参数", False),
        spec("LQ", "Q 轴电感（预留）", 0, 100000, 3, "MH", "预留参数", False),
        spec("FLUX", "磁链（预留）", 0, 100000, 3, "MWB", "预留参数", False),
    )
}
FOC = {
    key: SIXSTEP[key]
    for key in ("POLE_PAIRS", "SHUNT", "RATED_VOLTAGE", "RATED_CURRENT", "RATED_SPEED")
}
FOC.update(
    {
        p.key: p
        for p in (
            spec("TARGET", "目标转速", 1, 5400, 0, "RPM", "常用参数"),
            spec("CURRENT_LIMIT", "相电流限幅", 500, 2000, 3, "A", "常用参数"),
            spec("RUN_TIME", "运行时长", 1000, 32000, 0, "MS", "常用参数"),
            spec("RAMP", "速度斜坡", 10000, 500000, 3, "RPM/S", "速度环"),
            spec("SPEED_KP", "速度 Kp", 1, 2000, 3, "MA/RPM", "速度环"),
            spec("SPEED_KI", "速度 Ki", 1, 5000, 3, "MA/R/S", "速度环"),
            spec("SPEED_LIMIT", "转速上限", 5400, 5400, 0, "RPM", "固定模型", False),
            spec("START_HZ", "起始电频率", 50000, 50000, 3, "HZ", "固定模型", False),
            spec("ALIGN_TIME", "定位时长（兼容字段）", 200, 200, 0, "MS", "固定模型", False),
            spec("RESISTANCE", "相电阻", 790, 790, 3, "OHM", "固定模型", False),
            spec("LD", "D 轴电感", 240, 240, 3, "MH", "固定模型", False),
            spec("LQ", "Q 轴电感", 240, 240, 3, "MH", "固定模型", False),
            spec("D_KP", "D 轴 Kp", 0, 2147483647, 3, "V/A", "固定模型", False),
            spec("D_KI", "D 轴 Ki", 0, 2147483647, 3, "V/AS", "固定模型", False),
            spec("Q_KP", "Q 轴 Kp", 0, 2147483647, 3, "V/A", "固定模型", False),
            spec("Q_KI", "Q 轴 Ki", 0, 2147483647, 3, "V/AS", "固定模型", False),
        )
    }
)


def specs_for(
    algorithm: str, parameters: Mapping[str, ParameterValue] | None = None, mode: str = "SPEED"
) -> dict[str, ParameterSpec]:
    """@brief 合并算法与已读电机模型。@param algorithm 算法。@param parameters 实际参数。
    @param mode 真实控制模式。@return 参数表副本；未知范围不擅自扩展。
    """
    if algorithm not in ALGORITHMS:
        raise ControlError(f"尚不支持算法 {algorithm}")
    if not mode_supported(algorithm, mode):
        raise ControlError(f"尚不支持 {algorithm}/{mode}")
    result = dict(FOC if algorithm in FOC_ALGORITHMS else SIXSTEP)
    if algorithm in ENCODER_ALGORITHMS:
        for key, label in (("ENCODER_DIR", "编码器方向"), ("ELEC_ZERO", "电角零点")):
            result[key] = replace(SIXSTEP[key], label=label, group="电机参数", editable=False)
    if algorithm == "ENCODER_6STEP":
        for key in ("START_HZ", "HANDOFF", "COMM_TIMING", "ALIGN_TIME"):
            result.pop(key)
    actual = parameters or {}
    poles, limit = actual.get("POLE_PAIRS"), actual.get("SPEED_LIMIT")
    # 当前正式 G2208 模型由实际参数识别，固件补丁号和后缀不参与判断。
    g2208_rated = actual.get("RATED_SPEED") == ParameterValue(1800, 0, "RPM") and actual.get(
        "RATED_CURRENT"
    ) == ParameterValue(800, 3, "A")
    if poles == ParameterValue(7, 0, "") and (
        limit == ParameterValue(1800, 0, "RPM") or g2208_rated
    ):
        result["TARGET"] = replace(result["TARGET"], maximum=1800)
        result["CURRENT_LIMIT"] = replace(result["CURRENT_LIMIT"], maximum=800)
        result["POLE_PAIRS"] = replace(result["POLE_PAIRS"], minimum=7, maximum=7, editable=False)
        if algorithm not in FOC_ALGORITHMS:
            result["DUTY"] = replace(
                result["DUTY"], maximum=950 if algorithm == "ENCODER_6STEP" else 600
            )
            if "HANDOFF" in result:
                result["HANDOFF"] = replace(result["HANDOFF"], maximum=1800)
            result["SPEED_LIMIT"] = replace(result["SPEED_LIMIT"], maximum=1800)
    if mode == "CURRENT":
        # 独立 Iq 直接使用安培显示，串口仍是有符号毫安；不能沿用速度范围。
        result["TARGET"] = spec("TARGET", "目标 Iq（正负）", -500, 500, 3, "A", "常用参数")
        result["CURRENT_LIMIT"] = spec(
            "CURRENT_LIMIT", "电流指令限幅", 500, 500, 3, "A", "常用参数", False
        )
        for key in ("RAMP", "SPEED_KP", "SPEED_KI", "START_HZ", "ALIGN_TIME"):
            result.pop(key, None)
    if mode == "POSITION":
        result["TARGET"] = spec(
            "TARGET", "目标机械角（多圈）", -2147483647, 2147483647, 3, "DEG", "常用参数"
        )
        result["CURRENT_LIMIT"] = spec(
            "CURRENT_LIMIT", "位置电流限幅", 500, 800, 3, "A", "常用参数"
        )
        result["POSITION_KP"] = spec("POSITION_KP", "位置 Kp", 1, 100000, 3, "1/S", "位置环")
        result["SPEED_LIMIT"] = spec("SPEED_LIMIT", "定位移动速度", 1, 1800, 0, "RPM", "常用参数")
        result["RAMP"] = spec("RAMP", "位置运动加速度", 10000, 500000, 3, "RPM/S", "位置环")
        result["SPEED_KP"] = spec("SPEED_KP", "位置内环速度 Kp", 1, 100000, 3, "MA/RPM", "位置环")
        result["SPEED_KI"] = spec("SPEED_KI", "位置内环速度 Ki", 1, 1000000, 3, "MA/R/S", "位置环")
        for key in ("START_HZ", "ALIGN_TIME"):
            result.pop(key, None)
    if mode == "HAPTIC":
        for key in ("TARGET", "SPEED_KP", "SPEED_KI", "RAMP", "START_HZ", "ALIGN_TIME"):
            result.pop(key, None)
        result.update(
            {
                p.key: p
                for p in (
                    spec("CURRENT_LIMIT", "手感电流上限", 500, 800, 3, "A", "常用参数"),
                    spec("RUN_TIME", "运行时长", 1000, 2147483647, 0, "MS", "常用参数"),
                    spec("HAPTIC_KIND", "手感类型", 0, 1, 0, "", "手感参数"),
                    spec("HAPTIC_DETENTS", "每圈档数", 2, 256, 0, "", "手感参数"),
                    spec("HAPTIC_STRENGTH", "档位力度", 0, 800, 0, "MA", "手感参数"),
                    spec("HAPTIC_DAMPING", "连续阻尼", 0, 10000, 3, "MA/RPM", "手感参数"),
                )
            }
        )
    return result


@dataclass(frozen=True)
class BoardSnapshot:
    """从后台发送给界面的独立快照，字典在发布前复制。"""

    connected: bool = False
    # 只读开关同时约束底层发送，不依赖按钮禁用。
    read_only: bool = False
    firmware: str = ""
    protocol: str = ""
    # 完成协议、模式与核心参数核对后才开放普通写入；停止使用单独条件。
    control_available: bool = False
    notices: tuple[str, ...] = ()
    available_algorithms: tuple[str, ...] = ()
    encoder_available: bool = False
    encoder_calibration: str = ""
    trace_available: bool = False
    haptic_available: bool = False
    haptic: HapticSample | None = None
    position_available: bool = False
    position: PositionSample | None = None
    wave_available: bool = False
    wave_rate: int = 0
    link_baud: int = 921600
    # 0为日常监视；正值才表示下一次启动选择的高速波形波特率。
    preferred_wave_baud: int = 0
    algorithm: str = "SENSORLESS_FOC"
    source: str = ""
    mode: str = "SPEED"
    run: str = "UNKNOWN"
    ready: bool = False
    # 繁忙操作与错误为界面提供持续反馈。
    busy: str = ""
    error: str = ""
    save: str = ""
    fault: str = ""
    output_disabled: bool | None = None
    still: bool | None = None
    # 只有当前 ACTIVE/CLOSED 成立时允许在线调整。
    closed: bool = False
    stage: str = "停机"
    estimated_rpm: float | None = None
    iq_a: float | None = None
    id_a: float | None = None
    bus_v: float | None = None
    data_time: str = ""
    parameters: Mapping[str, ParameterValue] = field(default_factory=dict)
    # 参数定义随连接发布，界面与发送层使用同一范围和编辑权限。
    parameter_specs: Mapping[str, ParameterSpec] = field(default_factory=dict)
    # 完整诊断原文在停机后供用户查阅。
    diagnostics: Mapping[str, str] = field(default_factory=dict)
    revision: int = 0

    @property
    def idle(self) -> bool:
        """@brief 当前是否满足界面停机编辑条件。@return 停机状态。"""
        return self.connected and self.run == "STOPPED" and not self.busy

    @property
    def writable(self) -> bool:
        """@brief 当前连接是否允许写命令。@return 写权限。"""
        return self.can_stop and self.control_available

    @property
    def can_stop(self) -> bool:
        """@brief 核心参数不可用时仍允许同协议 STOP。@return 停止权限。"""
        return self.connected and not self.read_only and protocol_supported(self.protocol)


class SnapshotChanges(TypedDict, total=False):
    """后台允许发布的字段，防止动态字典绕过类型检查。"""

    connected: bool
    read_only: bool
    firmware: str
    protocol: str
    control_available: bool
    notices: tuple[str, ...]
    available_algorithms: tuple[str, ...]
    encoder_available: bool
    encoder_calibration: str
    trace_available: bool
    haptic_available: bool
    haptic: HapticSample | None
    position_available: bool
    position: PositionSample | None
    wave_available: bool
    wave_rate: int
    link_baud: int
    preferred_wave_baud: int
    algorithm: str
    source: str
    mode: str
    run: str
    ready: bool
    busy: str
    error: str
    save: str
    fault: str
    output_disabled: bool | None
    still: bool | None
    closed: bool
    stage: str
    estimated_rpm: float | None
    iq_a: float | None
    id_a: float | None
    bus_v: float | None
    data_time: str
    parameters: Mapping[str, ParameterValue]
    parameter_specs: Mapping[str, ParameterSpec]
    diagnostics: Mapping[str, str]
