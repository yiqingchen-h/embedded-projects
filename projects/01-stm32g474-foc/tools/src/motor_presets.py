"""正式固件的命名参数预设；本地编辑，停机恢复，不发送电机启停命令。"""

from __future__ import annotations

import argparse
from collections.abc import Iterator, Callable, Sequence
from contextlib import contextmanager
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime, timedelta, timezone
from decimal import Decimal
import importlib
import json
from pathlib import Path
import re
import sys
import time
from typing import Protocol, cast

# 本工具只支持当前已实现的三种速度算法与协议，不导入任意串口命令。
ALGORITHMS = ("OPEN_6STEP", "SENSORLESS_6STEP", "SENSORLESS_FOC")
FORMAT = "foc_motor_preset"
PROTOCOL_VERSION = "1.2"
DEFAULT_DIRECTORY = Path(__file__).resolve().parents[2] / "host" / "parameter_presets"
SHANGHAI = timezone(timedelta(hours=8), "Asia/Shanghai")


class PresetError(ValueError):
    """预设、响应或运行前提无效；发生后停止后续写入。"""


@dataclass(frozen=True)
class Parameter:
    """整数参数及固件返回的尺度，保存时不经过浮点转换。"""

    # 串口 SET 使用的原始整数。
    value: int
    # 实际值的小数位数，真实值为 value / 10**decimals。
    decimals: int
    # 固件返回的单位；空串表示无单位。
    unit: str

    def display(self) -> str:
        """@brief 显示真实尺度与原始整数。@return 可阅读的参数文本。"""
        physical = Decimal(self.value).scaleb(-self.decimals)
        return f"{physical:f} {self.unit} (raw={self.value})".strip()


@dataclass(frozen=True)
class ParameterSpec:
    """协议 1.2 参数白名单；范围来自当前 settings.c 与 motor_config.h。"""

    # 允许写入的原始整数范围。
    minimum: int
    maximum: int
    # 与该算法 GET 响应一致的尺度和单位。
    decimals: int
    unit: str

    def validate(self, name: str, parameter: Parameter) -> None:
        """@brief 核对数值和单位。@param name 参数名。@param parameter 候选参数。"""
        if not self.minimum <= parameter.value <= self.maximum:
            raise PresetError(f"{name} 超出范围 {self.minimum}..{self.maximum}")
        if (parameter.decimals, parameter.unit) != (self.decimals, self.unit):
            raise PresetError(f"{name} 的 DECIMALS/UNIT 与当前算法不符")
        if name == "SHUNT" and parameter.value not in (1, 50):
            raise PresetError("SHUNT 只允许 1 或 50 mΩ")
        if name == "ENCODER_DIR" and parameter.value not in (-1, 1):
            raise PresetError("ENCODER_DIR 只允许 -1 或 1")


# 六步全部可保存参数；OFFSET_A/B/C 是自动校准结果，不进入预设。
SIXSTEP_SPECS = {
    "TARGET": ParameterSpec(0, 5400, 0, "RPM"),
    "POLE_PAIRS": ParameterSpec(0, 32, 0, ""),
    "CURRENT_LIMIT": ParameterSpec(100, 1500, 3, "A"),
    "SPEED_LIMIT": ParameterSpec(1, 5400, 0, "RPM"),
    "POSITION_MIN": ParameterSpec(-36000, 36000, 1, "DEG"),
    "POSITION_MAX": ParameterSpec(-36000, 36000, 1, "DEG"),
    "D_KP": ParameterSpec(0, 100000, 3, ""),
    "D_KI": ParameterSpec(0, 100000, 3, ""),
    "Q_KP": ParameterSpec(0, 100000, 3, ""),
    "Q_KI": ParameterSpec(0, 100000, 3, ""),
    "SPEED_KP": ParameterSpec(0, 100000, 3, "D/RPM"),
    "SPEED_KI": ParameterSpec(0, 100000, 3, "D/RPMS"),
    "POSITION_KP": ParameterSpec(0, 100000, 3, ""),
    "POSITION_KI": ParameterSpec(0, 100000, 3, ""),
    "POSITION_KD": ParameterSpec(0, 100000, 3, ""),
    "START_HZ": ParameterSpec(100, 50000, 3, "HZ"),
    "RAMP": ParameterSpec(100, 100000, 3, "HZ/S"),
    "DUTY": ParameterSpec(1, 250, 1, "%"),
    "HANDOFF": ParameterSpec(0, 5400, 0, "RPM"),
    "COMM_TIMING": ParameterSpec(-300, 300, 1, "DEG"),
    "ENCODER_DIR": ParameterSpec(-1, 1, 0, ""),
    "ELEC_ZERO": ParameterSpec(0, 3599, 1, "DEG"),
    "RESISTANCE": ParameterSpec(0, 100000, 3, "OHM"),
    "LD": ParameterSpec(0, 100000, 3, "MH"),
    "LQ": ParameterSpec(0, 100000, 3, "MH"),
    "FLUX": ParameterSpec(0, 100000, 3, "MWB"),
    "SHUNT": ParameterSpec(1, 50, 0, "MOHM"),
    "RATED_VOLTAGE": ParameterSpec(8000, 15000, 3, "V"),
    "RATED_CURRENT": ParameterSpec(0, 10000, 3, "A"),
    "RATED_SPEED": ParameterSpec(1, 20000, 0, "RPM"),
    "ALIGN_TIME": ParameterSpec(10, 1000, 0, "MS"),
    "RUN_TIME": ParameterSpec(100, 30000, 0, "MS"),
}
# FOC 独立参数块以及仍可编辑的公共额定参数。
FOC_SPECS = {
    name: SIXSTEP_SPECS[name]
    for name in ("POLE_PAIRS", "SHUNT", "RATED_VOLTAGE", "RATED_CURRENT", "RATED_SPEED")
}
FOC_SPECS.update(
    {
        "TARGET": ParameterSpec(1, 5400, 0, "RPM"),
        "CURRENT_LIMIT": ParameterSpec(500, 2000, 3, "A"),
        "SPEED_KP": ParameterSpec(1, 2000, 3, "MA/RPM"),
        "SPEED_KI": ParameterSpec(1, 5000, 3, "MA/R/S"),
        "RAMP": ParameterSpec(10000, 500000, 3, "RPM/S"),
        "RUN_TIME": ParameterSpec(1000, 32000, 0, "MS"),
    }
)
# FOC 模型、派生增益和固定启动参数只用于核对，不发送 SET。
FOC_READONLY = (
    "SPEED_LIMIT",
    "START_HZ",
    "ALIGN_TIME",
    "D_KP",
    "D_KI",
    "Q_KP",
    "Q_KI",
    "RESISTANCE",
    "LD",
    "LQ",
)


def specs_for(algorithm: str) -> dict[str, ParameterSpec]:
    """@brief 获取支持算法的白名单。@param algorithm 算法。@return 参数描述表。"""
    if algorithm not in ALGORITHMS:
        raise PresetError(f"不支持的算法：{algorithm}")
    return FOC_SPECS if algorithm == "SENSORLESS_FOC" else SIXSTEP_SPECS


def validate_name(name: str) -> str:
    """@brief 拒绝路径穿越和特殊文件名。@param name 预设名。@return 校验后的名称。"""
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", name):
        raise PresetError("预设名须为 1..64 个英文字母、数字、下划线或连字符")
    if name.split("_")[0].upper() in {"CON", "PRN", "AUX", "NUL"} or re.fullmatch(
        r"(?:COM|LPT)[1-9]", name, re.IGNORECASE
    ):
        raise PresetError("预设名不能使用 Windows 保留设备名")
    return name


def object_dict(value: object, label: str) -> dict[str, object]:
    """@brief 校验 JSON 对象。@param value 输入。@param label 字段名。@return 字符串键字典。"""
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise PresetError(f"{label} 必须是 JSON 对象")
    return cast(dict[str, object], value)


def integer(value: object, label: str) -> int:
    """@brief 严格读取整数，拒绝布尔和浮点。@param value 输入。@param label 字段名。@return 整数。"""
    if type(value) is not int:
        raise PresetError(f"{label} 必须是整数")
    return value


def string(value: object, label: str) -> str:
    """@brief 校验单行字符串。@param value 输入。@param label 字段名。@return 字符串。"""
    if not isinstance(value, str) or any(ord(char) < 32 for char in value):
        raise PresetError(f"{label} 必须是不含控制字符的字符串")
    return value


def decode_parameters(value: object) -> dict[str, Parameter]:
    """@brief 解码数值与尺度。@param value JSON 参数表。@return 类型明确的参数表。"""
    decoded = {}
    for name, entry in object_dict(value, "parameters").items():
        data = object_dict(entry, name)
        decoded[name] = Parameter(
            integer(data.get("value"), f"{name}.value"),
            integer(data.get("decimals"), f"{name}.decimals"),
            string(data.get("unit"), f"{name}.unit"),
        )
    return decoded


@dataclass(frozen=True)
class Preset:
    """一个算法当前可见的持久化配置；不会主动切换算法读取隐藏参数块。"""

    # 文件格式标识和版本，用于拒绝不兼容文件。
    format: str
    schema_version: int
    # 英文命名与上海时区的采集时间。
    name: str
    created_at: str
    # 采集时固件与协议，恢复要求完全相同。
    firmware: str
    protocol: str
    # 算法、模式、来源与显示设置。
    algorithm: str
    mode: str
    source: str
    brightness: int
    animation: int
    # 可写参数与仅供兼容核对的只读参数。
    parameters: dict[str, Parameter]
    readonly: dict[str, Parameter]

    def validate(self) -> None:
        """@brief 在任何串口写入前校验完整预设。"""
        validate_name(self.name)
        if self.format != FORMAT or self.schema_version != 1:
            raise PresetError("预设文件格式或 schema_version 不受支持")
        if self.protocol != PROTOCOL_VERSION or not self.firmware:
            raise PresetError("预设的固件/协议版本无效")
        if self.mode != "SPEED" or self.source not in ("LOCAL", "REMOTE"):
            raise PresetError("仅支持 SPEED 模式与 LOCAL/REMOTE 来源")
        if not 1 <= self.brightness <= 100 or self.animation not in (0, 1):
            raise PresetError("亮度或动画设置超出范围")
        specs = specs_for(self.algorithm)
        if self.parameters.keys() != specs.keys():
            raise PresetError("预设的可写参数缺失或包含未知/只读参数")
        for name, parameter in self.parameters.items():
            specs[name].validate(name, parameter)
        expected_readonly = (
            set(FOC_READONLY) if self.algorithm == "SENSORLESS_FOC" else set()
        )
        if self.readonly.keys() != expected_readonly:
            raise PresetError("预设的只读兼容参数不完整")
        for parameter in self.readonly.values():
            if (
                not 0 <= parameter.decimals <= 3
                or not -2147483647 <= parameter.value <= 2147483647
            ):
                raise PresetError("只读参数的尺度或数值无效")

    def settings(self) -> dict[str, str]:
        """@brief 获取非数值参数的串口值。@return GET/SET 使用的设置表。"""
        return {
            "ALGORITHM": self.algorithm,
            "MODE": self.mode,
            "SOURCE": self.source,
            "BRIGHTNESS": str(self.brightness),
            "ANIMATION": str(self.animation),
        }


def load_preset(path: Path) -> Preset:
    """@brief 读取完整预设并校验。@param path JSON 路径。@return 已校验的预设。"""
    data = object_dict(json.loads(path.read_text(encoding="utf-8-sig")), "preset")
    preset = Preset(
        format=string(data.get("format"), "format"),
        schema_version=integer(data.get("schema_version"), "schema_version"),
        name=string(data.get("name"), "name"),
        created_at=string(data.get("created_at"), "created_at"),
        firmware=string(data.get("firmware"), "firmware"),
        protocol=string(data.get("protocol"), "protocol"),
        algorithm=string(data.get("algorithm"), "algorithm"),
        mode=string(data.get("mode"), "mode"),
        source=string(data.get("source"), "source"),
        brightness=integer(data.get("brightness"), "brightness"),
        animation=integer(data.get("animation"), "animation"),
        parameters=decode_parameters(data.get("parameters")),
        readonly=decode_parameters(data.get("readonly")),
    )
    preset.validate()
    return preset


def write_preset(path: Path, preset: Preset) -> None:
    """@brief 独占创建预设，避免覆盖已有方案。@param path 路径。@param preset 预设。"""
    preset.validate()
    payload = json.dumps(asdict(preset), ensure_ascii=False, indent=2) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as output:
        output.write(payload)


def copy_preset(preset: Preset, name: str, changes: Sequence[str]) -> Preset:
    """@brief 本地复制并修改原始整数。@param preset 原方案。@param name 新名称。
    @param changes NAME=VALUE 列表。@return 已校验的新预设。
    """
    settings = preset.settings()
    parameters = dict(preset.parameters)
    for change in changes:
        key, separator, value = change.partition("=")
        if not separator:
            raise PresetError("修改须使用 NAME=VALUE 格式")
        if key in parameters and re.fullmatch(r"-?\d+", value, re.ASCII):
            parameters[key] = replace(parameters[key], value=int(value))
        elif key in ("SOURCE", "BRIGHTNESS", "ANIMATION"):
            settings[key] = value
        else:
            raise PresetError(f"不允许修改 {key}，数值须使用原始十进制整数")
    if not settings["BRIGHTNESS"].isdigit() or not settings["ANIMATION"].isdigit():
        raise PresetError("BRIGHTNESS/ANIMATION 必须是整数")
    result = replace(
        preset,
        name=validate_name(name),
        created_at=datetime.now(SHANGHAI).isoformat(),
        source=settings["SOURCE"],
        brightness=int(settings["BRIGHTNESS"]),
        animation=int(settings["ANIMATION"]),
        parameters=parameters,
    )
    result.validate()
    return result


class SerialTransport(Protocol):
    """现有 Windows 串口工具的最小契约，测试替身实现同一接口。"""

    # 底层记录的通信错误与交错遥测，仅清理本次客户端的数据。
    driver_errors: list[object]
    unsolicited: list[object]

    def command(self, command: str, timeout: float = 3) -> str:
        """@brief 发送单条命令。@param command 命令。@param timeout 秒。@return 响应。"""
        ...

    def read_line(self, timeout: float = 3) -> str | None:
        """@brief 读取遗留行。@param timeout 秒。@return 行或超时。"""
        ...

    def close(self) -> None:
        """@brief 关闭句柄。"""
        ...


@dataclass
class PresetClient:
    """顺序串口事务客户端，不依赖事件循环或网络。"""

    # 独占串口会话；不并行发送命令。
    transport: SerialTransport
    # 可注入等待函数，使离线测试不消耗真实保存等待时间。
    wait: Callable[[float], None] = time.sleep
    # 每条命令和原始响应留作恢复审计。
    records: list[dict[str, str]] = field(default_factory=list)

    def ask(self, command: str, prefix: str) -> str:
        """@brief 发送并严格核对响应。@param command 命令。@param prefix 预期前缀。@return 原文。"""
        try:
            response = self.transport.command(command, 3.0)
        except (OSError, TimeoutError, RuntimeError, UnicodeError) as error:
            self.records.append({"command": command, "error": str(error)})
            raise PresetError(f"{command} 失败：{error}") from error
        self.records.append({"command": command, "response": response})
        self.transport.unsolicited.clear()
        if self.transport.driver_errors:
            raise PresetError("Windows 串口报告通信错误，已停止后续写入")
        if not response.startswith(prefix):
            raise PresetError(f"{command} 返回非预期响应：{response}")
        self.wait(0.03)
        return response

    def get(self, name: str) -> str:
        """@brief 读取标量设置。@param name 设置名。@return 等号后的原文。"""
        prefix = f"OK GET {name}="
        return (self.ask(f"GET {name}", prefix))[len(prefix) :]

    def parameter(self, name: str) -> Parameter:
        """@brief 读取整数参数和真实尺度。@param name 参数名。@return 参数。"""
        response = self.get(name)
        matched = re.fullmatch(
            r"(-?\d+) DECIMALS=([0-3]) UNIT=([^\s]*)", response, re.ASCII
        )
        if matched is None:
            raise PresetError(f"GET {name} 的数值响应不完整：{response}")
        return Parameter(int(matched[1]), int(matched[2]), matched[3])

    def identity(self) -> tuple[str, str]:
        """@brief 确认正式协议。@return 固件版本和协议版本。"""
        response = self.ask("VERSION", "OK VERSION ")
        fields = response_fields(response)
        if fields.get("PROTOCOL") != PROTOCOL_VERSION or not fields.get("FW"):
            raise PresetError("仅支持带完整参数接口的正式协议 1.2")
        return fields["FW"], fields["PROTOCOL"]

    def idle(self, physical: bool = False) -> None:
        """@brief 确认停机与无在途请求。@param physical 是否同时检查输出禁用。"""
        fields = response_fields(self.ask("STATUS", "OK STATUS "))
        if fields.get("MOTOR") != "STOPPED":
            raise PresetError("请先由用户停机并处理故障，再操作参数预设")
        if self.get("RESULT") == "PENDING":
            raise PresetError("有在途请求，已停止预设操作")
        if physical:
            rest = response_fields(self.ask("MOTOR FOC REST", "OK MOTOR FOC REST "))
            if rest.get("OUTPUT_DISABLED") != "1" or rest.get("FAULT") != "NONE":
                raise PresetError("后端尚未确认功率输出禁用且无故障")

    def saved(self, changed: bool = False) -> None:
        """@brief 静默等待后核对持久化。@param changed 本次是否提交修改。"""
        # 停机 Flash 写入会暂停串口；先让固件完成一秒合并保存，避免密集查询。
        self.wait(1.5)
        state = self.get("SAVE")
        for _ in range(3):
            if state != "PENDING":
                break
            self.wait(1.5)
            state = self.get("SAVE")
        if state != "SAVED" and not (state == "DEFAULT" and not changed):
            raise PresetError(f"参数保存未完成：{state}")
        storage = response_fields(self.ask("STORAGE", "OK STORAGE "))
        if storage.get("MATCH") != "1" or storage.get("RESULT") not in ("NONE", "OK"):
            raise PresetError("Flash 布局或写入校验异常：" + str(storage))

    def capture(self, name: str) -> Preset:
        """@brief 只读备份当前算法，双读拒绝采集中变化。@param name 名称。@return 完整预设。"""
        validate_name(name)
        self.idle(physical=True)
        self.saved()
        firmware, protocol = self.identity()
        settings = {
            key: self.get(key)
            for key in ("ALGORITHM", "MODE", "SOURCE", "BRIGHTNESS", "ANIMATION")
        }
        specs = specs_for(settings["ALGORITHM"])
        if settings["MODE"] != "SPEED":
            raise PresetError("只备份当前已实现的 SPEED 模式")
        readonly_names = (
            FOC_READONLY if settings["ALGORITHM"] == "SENSORLESS_FOC" else ()
        )
        parameters = {name: self.parameter(name) for name in specs}
        readonly = {name: self.parameter(name) for name in readonly_names}
        for key, value in settings.items():
            if self.get(key) != value:
                raise PresetError("采集期间设置发生变化，请保持停机后重新备份")
        for key, parameter in (parameters | readonly).items():
            if self.parameter(key) != parameter:
                raise PresetError(f"采集期间 {key} 发生变化，请重新备份")
        self.idle()
        preset = Preset(
            FORMAT,
            1,
            name,
            datetime.now(SHANGHAI).isoformat(),
            firmware,
            protocol,
            settings["ALGORITHM"],
            settings["MODE"],
            settings["SOURCE"],
            int(settings["BRIGHTNESS"]),
            int(settings["ANIMATION"]),
            parameters,
            readonly,
        )
        preset.validate()
        return preset

    def set_value(self, name: str, value: str) -> None:
        """@brief 写一项并确认实际结果。@param name 参数。@param value 串口值。"""
        self.idle()
        response = self.ask(f"SET {name} {value}", "OK SET ")
        if response == "OK SET PENDING":
            for _ in range(20):
                result = self.get("RESULT")
                if result == "OK":
                    break
                if result != "PENDING":
                    raise PresetError(f"SET {name} 执行失败：{result}")
                self.wait(0.05)
            else:
                raise PresetError(f"SET {name} 一直 PENDING")
        elif response != f"OK SET {name}={value}":
            raise PresetError(f"SET {name} 的确认响应无效：{response}")
        actual = (self.get(name)).split(" ", 1)[0]
        if actual != value:
            raise PresetError(f"SET {name} 回读不一致：期望 {value}，实际 {actual}")

    def restore(self, preset: Preset, directory: Path) -> list[Path]:
        """@brief 停机恢复、保留修改前配置并验证落盘。@param preset 目标预设。
        @param directory 自动备份目录。@return 修改前备份路径；失败时保留已写状态，不自动回滚。
        """
        preset.validate()
        firmware, protocol = self.identity()
        if (firmware, protocol) != (preset.firmware, preset.protocol):
            raise PresetError("预设与板上固件/协议不同，请在当前固件重新备份")
        stamp = datetime.now(SHANGHAI).strftime("%Y%m%d_%H%M%S_%f")
        before = self.capture("before_" + stamp)
        before_path = directory / f"{before.name}.json"
        write_preset(before_path, before)
        backups = [before_path]
        changed = False
        if before.algorithm != preset.algorithm:
            self.set_value("ALGORITHM", preset.algorithm)
            changed = True
            # 两个算法参数块相互独立；切换后另存目标块旧值，失败时可人工恢复两个块。
            selected = self.capture("selected_" + stamp)
            selected_path = directory / f"{selected.name}.json"
            write_preset(selected_path, selected)
            backups.append(selected_path)
        else:
            selected = before
        if selected.readonly != preset.readonly:
            raise PresetError("FOC 只读模型/派生参数不一致，已停止参数写入")
        settings = selected.settings()
        for name, value in preset.settings().items():
            if name not in ("ALGORITHM", "SOURCE") and settings[name] != value:
                self.set_value(name, value)
                changed = True
        for name, parameter in preset.parameters.items():
            if selected.parameters[name] != parameter:
                self.set_value(name, str(parameter.value))
                changed = True
        # 来源放在最后恢复，批量修改只在停机时进行。
        if settings["SOURCE"] != preset.source:
            self.set_value("SOURCE", preset.source)
            changed = True
        self.saved(changed)
        after = self.capture("verify_" + stamp)
        if differences(after, preset):
            raise PresetError("最终完整回读不一致，已保留修改前备份与审计记录")
        return backups


def response_fields(response: str) -> dict[str, str]:
    """@brief 提取诊断键值。@param response 原始响应。@return 字段表。"""
    return dict(token.split("=", 1) for token in response.split() if "=" in token)


def differences(current: Preset, target: Preset) -> list[str]:
    """@brief 比较当前与目标配置。@param current 当前。@param target 目标。@return 差异文本。"""
    result = []
    if (current.firmware, current.protocol) != (target.firmware, target.protocol):
        result.append(
            f"版本: {current.firmware}/{current.protocol} -> {target.firmware}/{target.protocol}"
        )
    for name, value in target.settings().items():
        if current.settings()[name] != value:
            result.append(f"{name}: {current.settings()[name]} -> {value}")
    if current.algorithm != target.algorithm:
        result.append("算法不同：目标算法的隐藏参数块只能在停机切换后读取")
        return result
    for name, parameter in (target.parameters | target.readonly).items():
        actual = (current.parameters | current.readonly).get(name)
        if actual != parameter:
            result.append(
                f"{name}: {actual.display() if actual else '--'} -> {parameter.display()}"
            )
    return result


@contextmanager
def connected(port: str, baud: int) -> Iterator[PresetClient]:
    """@brief 打开现有 Windows 串口并保证关闭。@param port 端口。@param baud 波特率。
    @return 逐条请求客户端。
    """
    if sys.platform != "win32":
        raise PresetError("板卡操作需要 Windows；list/show/copy/dry-run 可离线使用")
    module = importlib.import_module("motor_serial")
    transport = cast(SerialTransport, module.SerialPort(port, baud))
    try:
        # 清掉之前会话的遗留响应，不能让旧 OK 冒充本次命令确认。
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            if transport.read_line(0.05) is None:
                break
        yield PresetClient(transport)
    finally:
        transport.close()


def build_parser() -> argparse.ArgumentParser:
    """@brief 创建命令行接口。@return 参数解析器。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--directory", type=Path, default=DEFAULT_DIRECTORY, help="命名预设目录"
    )
    parser.add_argument("--port", default="COM10", help="明确指定的 Windows 串口")
    parser.add_argument("--baud", type=int, default=921600, help="当前固件 921600/8N1")
    commands = parser.add_subparsers(dest="action", required=True)
    commands.add_parser("list", help="离线列出命名预设")
    for action, help_text in (
        ("backup", "只读备份当前算法"),
        ("show", "离线查看方案"),
        ("diff", "只读比较板卡当前配置"),
        ("restore", "停机恢复并回读确认"),
    ):
        subparser = commands.add_parser(action, help=help_text)
        subparser.add_argument("name", help="英文预设名，不含路径或 .json")
        if action == "restore":
            subparser.add_argument(
                "--dry-run", action="store_true", help="离线显示计划，不打开串口"
            )
    copy = commands.add_parser("copy", help="离线复制方案并修改原始整数")
    copy.add_argument("name", help="现有预设名")
    copy.add_argument("new_name", help="新预设名")
    copy.add_argument(
        "--set", action="append", default=[], dest="changes", metavar="NAME=RAW"
    )
    return parser


def run(args: argparse.Namespace) -> None:
    """@brief 执行选定功能。@param args 命令行参数。"""
    directory = cast(Path, args.directory)
    if args.action == "list":
        for path in sorted(directory.glob("*.json")):
            listed = load_preset(path)
            print(
                f"{path.stem}: {listed.algorithm}/{listed.mode}, FW={listed.firmware}, {listed.created_at}"
            )
        if not list(directory.glob("*.json")):
            print("暂无预设；先使用 backup <name> 备份当前配置。")
        return
    name = validate_name(args.name)
    path = directory / f"{name}.json"
    if args.action == "backup" and path.exists():
        raise PresetError(f"预设已存在，请使用新名称：{path}")
    preset = load_preset(path) if args.action != "backup" else None
    if preset is not None and args.action == "show":
        print(json.dumps(asdict(preset), ensure_ascii=False, indent=2))
        return
    if preset is not None and args.action == "copy":
        updated = copy_preset(preset, args.new_name, args.changes)
        new_path = directory / f"{updated.name}.json"
        write_preset(new_path, updated)
        print(f"本地预设已创建：{new_path}")
        return
    if preset is not None and args.action == "restore" and args.dry_run:
        print(f"离线计划：FW={preset.firmware}，{preset.algorithm}/{preset.mode}")
        for key, value in preset.settings().items():
            print(f"SET {key} {value}")
        for key, parameter in preset.parameters.items():
            print(f"SET {key} {parameter.value}")
        print("执行时只写差异项，保留修改前备份；逐项回读，等待 SAVED；不发送 START。")
        return
    with connected(args.port, args.baud) as client:
        if args.action == "backup":
            write_preset(path, client.capture(name))
            print(f"当前算法配置已备份：{path}")
        elif preset is not None and args.action == "diff":
            current = client.capture("comparison")
            print("\n".join(differences(current, preset)) or "当前配置与预设相同。")
        elif preset is not None and args.action == "restore":
            audit = (
                directory
                / "restore_backups"
                / (datetime.now(SHANGHAI).strftime("%Y%m%d_%H%M%S_%f") + "_audit.json")
            )
            try:
                backups = client.restore(preset, audit.parent)
                print("预设恢复成功：完整回读、保存及输出禁用检查通过。")
                for backup in backups:
                    print(f"修改前备份：{backup}")
            finally:
                audit.parent.mkdir(parents=True, exist_ok=True)
                audit.write_text(
                    json.dumps(client.records, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
                print(f"逐条命令审计：{audit}")


def main(argv: Sequence[str] | None = None) -> int:
    """@brief 执行工具并返回退出码。@param argv 可选参数列表。@return 成功 0，失败 1。"""
    args = build_parser().parse_args(argv)
    try:
        run(args)
    except (ValueError, OSError, RuntimeError) as error:
        print(
            f"预设操作失败：{error}\n已停止后续操作；恢复中途失败时请查看 restore_backups，勿认为已完整恢复。",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
