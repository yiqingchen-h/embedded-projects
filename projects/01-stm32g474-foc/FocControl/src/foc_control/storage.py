"""兼容命名预设格式、审计日志和便携数据目录。"""

from __future__ import annotations

import csv
import json
import re
from collections.abc import Iterable
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import TextIO, cast

from .models import (
    ENCODER_ALGORITHMS,
    FOC_ALGORITHMS,
    MODEL_PARAMETERS,
    PROTOCOL,
    REQUIRED_PARAMETERS,
    ControlError,
    ParameterValue,
    mode_supported,
    protocol_supported,
    specs_for,
)
from .protocol import PlotPoint

# 与现有 motor_presets.py 文件格式保持一致，兼容显示元数据而不驱动显示硬件。
FORMAT = "foc_motor_preset"
SHANGHAI = timezone(timedelta(hours=8))


def timestamp() -> str:
    """@brief 取得上海时区时间。@return ISO 时间戳。"""
    return datetime.now(SHANGHAI).isoformat(timespec="milliseconds")


def name_valid(name: str) -> str:
    """@brief 限制文件名，避免路径注入。@param name 预设名。@return 原名。"""
    if re.fullmatch(r"[A-Za-z0-9_-]{1,64}", name) is None:
        raise ControlError("预设名需为 1～64 个英文字母、数字、下划线或连字符")
    return name


def parameter_map(data: object) -> dict[str, ParameterValue]:
    """@brief 校验 JSON 参数对象。@param data 原对象。@return 参数字典。"""
    if not isinstance(data, dict):
        raise ControlError("预设参数必须是对象")
    result: dict[str, ParameterValue] = {}
    for key, value in data.items():
        if (
            not isinstance(key, str)
            or not isinstance(value, dict)
            or set(value) != {"value", "decimals", "unit"}
        ):
            raise ControlError("预设参数字段不完整")
        number, decimals, unit = value["value"], value["decimals"], value["unit"]
        if type(number) is not int or type(decimals) is not int or not isinstance(unit, str):
            raise ControlError("预设参数类型无效")
        if not 0 <= decimals <= 3 or not -2147483647 <= number <= 2147483647:
            raise ControlError("预设参数范围无效")
        result[key] = ParameterValue(number, decimals, unit)
    return result


def writable_preset_keys(
    algorithm: str, parameters: dict[str, ParameterValue] | None = None, mode: str = "SPEED"
) -> set[str]:
    """@brief 保留旧格式并排除有感标定只读字段。@param algorithm 算法。
    @param parameters 已读参数。@param mode 控制模式。@return 可保存字段。
    """
    values = specs_for(algorithm, parameters, mode)
    # 六步预留项仍属于旧格式的保存字段；GUI 不将其开放编辑。
    return (
        set(values)
        if algorithm not in FOC_ALGORITHMS and algorithm not in ENCODER_ALGORITHMS
        else {key for key, value in values.items() if value.editable}
    )


@dataclass(frozen=True)
class Preset:
    """单个算法的完整预设；隐藏算法参数块不伪装成当前快照。"""

    format: str
    schema_version: int
    name: str
    created_at: str
    firmware: str
    protocol: str
    algorithm: str
    mode: str
    source: str
    brightness: int
    animation: int
    parameters: dict[str, ParameterValue]
    readonly: dict[str, ParameterValue]

    def validate(self) -> None:
        """@brief 在任何设备操作前完整校验文件。"""
        name_valid(self.name)
        if (
            self.format != FORMAT
            or self.schema_version != 1
            or not protocol_supported(self.protocol)
            or not self.firmware
        ):
            raise ControlError("预设格式或版本不受支持")
        if not mode_supported(self.algorithm, self.mode) or self.source not in ("LOCAL", "REMOTE"):
            raise ControlError("不支持该算法/模式或控制来源")
        if type(self.brightness) is not int or not 1 <= self.brightness <= 100:
            raise ControlError("兼容亮度字段无效")
        if type(self.animation) is not int or self.animation not in (0, 1):
            raise ControlError("兼容动画字段无效")
        actual = {**self.readonly, **self.parameters}
        profile = specs_for(self.algorithm, actual, self.mode)
        writable = writable_preset_keys(self.algorithm, actual, self.mode)
        if (
            not (REQUIRED_PARAMETERS - {"TARGET"} if self.mode == "HAPTIC" else REQUIRED_PARAMETERS)
            <= set(actual)
            or not MODEL_PARAMETERS & set(profile) <= set(actual)
            or set(self.parameters) - (writable | {"POLE_PAIRS"})
            or set(self.readonly) - set(profile)
            or set(self.parameters) & set(self.readonly)
        ):
            raise ControlError("预设参数缺失或包含未知字段")
        for key, parameter in self.parameters.items():
            # 文件允许保存未来固件扩展的数值范围；真实写入前使用目标板能力完整校验。
            if (parameter.decimals, parameter.unit) != (profile[key].decimals, profile[key].unit):
                raise ControlError(f"预设参数 {key} 单位或尺度不符")
            if not -2147483647 <= parameter.value <= 2147483647:
                raise ControlError(f"预设参数 {key} 范围无效")
        for key, value in self.readonly.items():
            if not 0 <= value.decimals <= 3 or abs(value.value) > 2147483647:
                raise ControlError(f"只读参数 {key} 无效")
        if self.mode == "HAPTIC":
            for key in (
                "HAPTIC_KIND",
                "HAPTIC_DETENTS",
                "HAPTIC_STRENGTH",
                "HAPTIC_DAMPING",
                "CURRENT_LIMIT",
            ):
                profile[key].validate(actual[key])
        if self.mode in ("CURRENT", "POSITION"):
            profile["TARGET"].validate(actual["TARGET"])
            profile["CURRENT_LIMIT"].validate(actual["CURRENT_LIMIT"])

    @classmethod
    def load(cls, path: Path) -> Preset:
        """@brief 读取并校验 JSON。@param path 文件。@return 完整预设。"""
        if path.stat().st_size > 262144:
            raise ControlError("预设文件超过合理大小")
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                raise ControlError("预设顶层必须是对象")
            for key in (
                "format",
                "name",
                "created_at",
                "firmware",
                "protocol",
                "algorithm",
                "mode",
                "source",
            ):
                if not isinstance(data.get(key), str):
                    raise ControlError(f"预设缺少 {key}")
            if type(data.get("schema_version")) is not int:
                raise ControlError("schema_version 类型无效")
            result = cls(
                data["format"],
                data["schema_version"],
                data["name"],
                data["created_at"],
                data["firmware"],
                data["protocol"],
                data["algorithm"],
                data["mode"],
                data["source"],
                data["brightness"],
                data["animation"],
                parameter_map(data["parameters"]),
                parameter_map(data["readonly"]),
            )
        except (KeyError, TypeError, json.JSONDecodeError) as error:
            raise ControlError("预设文件不完整") from error
        result.validate()
        return result

    def save(self, path: Path) -> None:
        """@brief 独占写入，已有文件不覆盖。@param path 目标文件。"""
        self.validate()
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x", encoding="utf-8") as stream:
            json.dump(asdict(self), stream, ensure_ascii=False, indent=2)
            stream.write("\n")

    def copy(self, name: str, changes: dict[str, str]) -> Preset:
        """@brief 离线复制修改物理量。@param name 新名。@param changes 输入文本。@return 新预设。"""
        name_valid(name)
        values = dict(self.parameters)
        profile = specs_for(self.algorithm, {**self.readonly, **self.parameters}, self.mode)
        for key, text in changes.items():
            if key not in values:
                raise ControlError(f"预设没有可写参数 {key}")
            values[key] = profile[key].encode(text)
        result = replace(self, name=name, created_at=timestamp(), parameters=values)
        result.validate()
        return result

    def difference(self, other: Preset) -> list[str]:
        """@brief 比较完整预设。@param other 目标。@return 不同字段。"""
        keys = (
            "firmware",
            "protocol",
            "algorithm",
            "mode",
            "source",
            "brightness",
            "animation",
            "readonly",
        )
        result = [key for key in keys if getattr(self, key) != getattr(other, key)]
        result.extend(
            key
            for key in self.parameters.keys() | other.parameters.keys()
            if self.parameters.get(key) != other.parameters.get(key)
        )
        return result


class Journal:
    """原始收发与审计写入同一时间戳 JSONL，UI 日志可以独立过滤。"""

    def __init__(self, directory: Path, port: str) -> None:
        """@brief 新建会话记录。@param directory 数据目录。@param port 端口说明。"""
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(SHANGHAI).strftime("%Y%m%d_%H%M%S_%f")
        self.path = directory / f"{stamp}.jsonl"
        self.stream: TextIO | None = self.path.open("x", encoding="utf-8")
        self.write("SESSION", f"{port} / FocControl / 协议自动识别（最低 {PROTOCOL}）")

    def write(self, kind: str, text: str) -> None:
        """@brief 记录原文。@param kind 事件类型。@param text 原文。"""
        if self.stream is not None:
            self.stream.write(
                json.dumps({"time": timestamp(), "kind": kind, "text": text}, ensure_ascii=False)
                + "\n"
            )
            if kind != "DATA":
                self.stream.flush()

    def close(self) -> None:
        """@brief 刷新并释放记录文件。"""
        if self.stream is not None:
            self.stream.close()
            self.stream = None


def export_csv(path: Path, points: Iterable[PlotPoint]) -> None:
    """@brief 导出全部有效点，不插值补齐。@param path CSV 文件。@param points 记录。"""
    header = (
        "time_s",
        "received_at",
        "sample",
        "ticks",
        "stage_id",
        "target_rpm",
        "estimated_rpm",
        "ia_a",
        "ib_a",
        "ic_a",
        "isc_a",
        "id_a",
        "iq_a",
        "ref_d_a",
        "ref_q_a",
        "bus_v",
        "time_source",
    )
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for point in points:
            value = point.frame
            currents = (
                value.ia_ma,
                value.ib_ma,
                value.ic_ma,
                value.isc_ma,
                value.id_ma,
                value.iq_ma,
                value.ref_d_ma,
                value.ref_q_ma,
            )
            writer.writerow(
                (
                    point.seconds,
                    point.received_at,
                    value.sample,
                    value.ticks,
                    value.stage,
                    point.target_rpm,
                    value.rpm,
                    *(item / 1000 if item is not None else "" for item in currents),
                    value.bus_mv / 1000 if value.bus_mv is not None else "",
                    "board_ticks" if value.ticks is not None else "pc_received",
                )
            )


def settings_load(path: Path) -> dict[str, object]:
    """@brief 容错读取本机偏好，不执行其中命令。@param path 设置文件。@return 设置对象。"""
    try:
        result = json.loads(path.read_text(encoding="utf-8"))
        return cast(dict[str, object], result) if isinstance(result, dict) else {}
    except (OSError, ValueError):
        return {}
