"""参数预设离线回归；串口替身注入真实协议响应，不访问板卡。"""

from __future__ import annotations

from collections.abc import Iterator, Callable
from contextlib import contextmanager
from dataclasses import asdict, dataclass, field, replace
import json
from pathlib import Path
import sys

import pytest

# 使用 tools/src 中的真实参数工具进行离线回归。
TOOLS = Path(__file__).resolve().parents[2] / "tools" / "src"
sys.path.insert(0, str(TOOLS))
import motor_presets as mp  # noqa: E402


def make_preset(
    algorithm: str = "SENSORLESS_FOC", name: str = "reference"
) -> mp.Preset:
    """@brief 构造明确标为测试的有效预设。@param algorithm 算法。@param name 名称。@return 测试配置。"""
    parameters = {
        key: mp.Parameter(spec.minimum, spec.decimals, spec.unit)
        for key, spec in mp.specs_for(algorithm).items()
    }
    for key, value in {
        "POLE_PAIRS": 3,
        "SHUNT": 50,
        "TARGET": 1000,
        "RUN_TIME": 20000,
    }.items():
        parameters[key] = replace(parameters[key], value=value)
    readonly = (
        {
            "SPEED_LIMIT": mp.Parameter(5400, 0, "RPM"),
            "START_HZ": mp.Parameter(50000, 3, "HZ"),
            "ALIGN_TIME": mp.Parameter(200, 0, "MS"),
            "D_KP": mp.Parameter(150, 3, "V/A"),
            "D_KI": mp.Parameter(4963, 3, "V/AS"),
            "Q_KP": mp.Parameter(150, 3, "V/A"),
            "Q_KI": mp.Parameter(4963, 3, "V/AS"),
            "RESISTANCE": mp.Parameter(790, 3, "OHM"),
            "LD": mp.Parameter(240, 3, "MH"),
            "LQ": mp.Parameter(240, 3, "MH"),
        }
        if algorithm == "SENSORLESS_FOC"
        else {}
    )
    return mp.Preset(
        mp.FORMAT,
        1,
        name,
        "TEST_DATA_ONLY",
        "1.4.5-sixstart-fix1",
        "1.2",
        algorithm,
        "SPEED",
        "REMOTE",
        50,
        1,
        parameters,
        readonly,
    )


def no_wait(seconds: float) -> None:
    """@brief 省略测试中的真实延时。@param seconds 请求时长。"""
    assert seconds >= 0


@dataclass
class FakeSerial:
    """支持三个独立算法参数块的串口替身，记录每条命令及所有写入。"""

    # 独立参数块和当前选中算法。
    presets: dict[str, mp.Preset] = field(
        default_factory=lambda: {key: make_preset(key) for key in mp.ALGORITHMS}
    )
    algorithm: str = "SENSORLESS_FOC"
    # 注入队列为空后回到正常响应；异常用于模拟断线和超时。
    overrides: dict[str, list[str | Exception]] = field(default_factory=dict)
    # 状态门禁及持久化状态。
    state: str = "STOPPED"
    output_disabled: bool = True
    save: str = "SAVED"
    storage_result: str = "OK"
    layout_match: bool = True
    result: str = "OK"
    # 审计、驱动错误和关闭状态。
    commands: list[str] = field(default_factory=list)
    driver_errors: list[object] = field(default_factory=list)
    unsolicited: list[object] = field(default_factory=list)
    closed: bool = False

    def command(self, command: str, timeout: float = 3) -> str:
        """@brief 模拟固件 GET/SET 与诊断。@param command 命令。@param timeout 时限。@return 响应。"""
        assert timeout > 0
        assert command.split()[0] not in {"START", "STOP", "CLEAR", "STEP"}
        self.commands.append(command)
        queue = self.overrides.get(command, [])
        if queue:
            answer = queue.pop(0)
            if isinstance(answer, Exception):
                raise answer
            return answer
        preset = self.presets[self.algorithm]
        if command == "VERSION":
            return f"OK VERSION FW={preset.firmware} PROTOCOL={preset.protocol}"
        if command == "STATUS":
            return f"OK STATUS MOTOR={self.state} READY=1 ALGORITHM={self.algorithm} MODE=SPEED"
        if command == "MOTOR FOC REST":
            return f"OK MOTOR FOC REST OUTPUT_DISABLED={int(self.output_disabled)} STILL=1 FAULT=NONE"
        if command == "STORAGE":
            return f"OK STORAGE SAVE={self.save} RESULT={self.storage_result} MATCH={int(self.layout_match)}"
        words = command.split()
        if words[0] == "GET":
            key = words[1]
            if key in ("SAVE", "RESULT"):
                return f"OK GET {key}={self.save if key == 'SAVE' else self.result}"
            if key in preset.settings():
                return f"OK GET {key}={preset.settings()[key]}"
            parameter = (preset.parameters | preset.readonly)[key]
            return f"OK GET {key}={parameter.value} DECIMALS={parameter.decimals} UNIT={parameter.unit}"
        assert words[0] == "SET"
        key, value = words[1:]
        if key == "ALGORITHM":
            self.algorithm = value
            return f"OK SET ALGORITHM={value}"
        if key in ("SOURCE", "MODE", "BRIGHTNESS", "ANIMATION"):
            if key == "SOURCE":
                self.presets[self.algorithm] = replace(preset, source=value)
                return "OK SET PENDING"
            field_name = key.lower()
            self.presets[self.algorithm] = replace(
                preset, **{field_name: value if key == "MODE" else int(value)}
            )
            return f"OK SET {key}={value}"
        assert key not in preset.readonly
        self.presets[self.algorithm] = mp.copy_preset(
            preset, preset.name, [f"{key}={value}"]
        )
        return "OK SET PENDING"

    def read_line(self, timeout: float = 3) -> str | None:
        """@brief 默认没有遗留响应。@param timeout 时限。@return 超时。"""
        return None

    def close(self) -> None:
        """@brief 记录关闭动作。"""
        self.closed = True


@pytest.fixture
def serial() -> FakeSerial:
    """@brief 创建独立串口替身。@return 替身。"""
    return FakeSerial()


def client(serial: FakeSerial) -> mp.PresetClient:
    """@brief 注入快速等待函数。@param serial 替身。@return 真实客户端。"""
    return mp.PresetClient(serial, wait=no_wait)


def test_roundtrip_and_no_overwrite(tmp_path: Path) -> None:
    """@brief 完整 JSON 往返与同名保护不会改变原文件。@param tmp_path 临时目录。"""
    preset = make_preset()
    path = tmp_path / "reference.json"
    mp.write_preset(path, preset)
    assert mp.load_preset(path) == preset
    with pytest.raises(FileExistsError):
        mp.write_preset(path, mp.copy_preset(preset, "reference", ["TARGET=2000"]))
    assert mp.load_preset(path) == preset
    assert mp.Parameter(500, 3, "A").display() == "0.500 A (raw=500)"


@pytest.mark.parametrize("algorithm", mp.ALGORITHMS)
def test_capture_is_readonly_and_complete(serial: FakeSerial, algorithm: str) -> None:
    """@brief 双读备份覆盖全部算法且只发送查询。@param serial 替身。@param algorithm 算法。"""
    serial.algorithm = algorithm
    captured = client(serial).capture("backup")
    assert not mp.differences(captured, serial.presets[algorithm])
    assert all(not command.startswith("SET ") for command in serial.commands)
    assert not set(captured.parameters) & {"OFFSET_A", "OFFSET_B", "OFFSET_C"}


@pytest.mark.parametrize(
    "change",
    [
        "TARGET=2000",
        "CURRENT_LIMIT=1000",
        "RAMP=300000",
        "SOURCE=LOCAL",
        "BRIGHTNESS=75",
        "ANIMATION=0",
    ],
)
def test_copy_uses_raw_values(change: str) -> None:
    """@brief 本地预设修改保留真实尺度。@param change 参数修改。"""
    original = make_preset()
    updated = mp.copy_preset(original, "candidate", [change])
    assert updated.name == "candidate" and mp.differences(original, updated)
    assert original.name == "reference" and original.parameters["TARGET"].value == 1000


@pytest.mark.parametrize(
    "change",
    [
        "TARGET=1.5",
        "TARGET=5401",
        "SHUNT=2",
        "OFFSET_A=2000",
        "START_HZ=1",
        "ALGORITHM=SENSORLESS_6STEP",
        "SOURCE=REMOTE\r\nSTART",
        "BRIGHTNESS=no",
        "BRIGHTNESS=0",
        "ANIMATION=2",
        "TARGET",
        "CURRENT_LIMIT=-500",
    ],
)
def test_invalid_copy_rejected(change: str) -> None:
    """@brief 拒绝注入、只读项、错误尺度和越界修改。@param change 无效修改。"""
    with pytest.raises(mp.PresetError):
        mp.copy_preset(make_preset(), "candidate", [change])


@pytest.mark.parametrize(
    "name",
    [
        "../escape",
        "with space",
        "",
        "NUL",
        "CON",
        "COM1",
        "LPT9",
        "name.json",
        "a" * 65,
    ],
)
def test_invalid_names(name: str) -> None:
    """@brief 拒绝不安全的文件名。@param name 无效名字。"""
    with pytest.raises(mp.PresetError):
        mp.validate_name(name)


@pytest.mark.parametrize(
    "mutation",
    [
        lambda data: data.update(schema_version=2),
        lambda data: data.update(protocol="1.3"),
        lambda data: data.update(mode="TORQUE"),
        lambda data: data.update(source="UNKNOWN"),
        lambda data: data.update(algorithm="ENCODER_FOC"),
        lambda data: data["parameters"].pop("TARGET"),
        lambda data: data["parameters"].update(
            OFFSET_A={"value": 1, "decimals": 0, "unit": "ADC"}
        ),
        lambda data: data["parameters"]["TARGET"].update(value=True),
        lambda data: data["parameters"]["TARGET"].update(value=1.5),
        lambda data: data["parameters"]["RAMP"].update(unit="HZ/S"),
        lambda data: data["readonly"].pop("LD"),
        lambda data: data["readonly"]["LD"].update(decimals=4),
    ],
)
def test_invalid_json_contract(
    tmp_path: Path, mutation: Callable[[dict[str, object]], None]
) -> None:
    """@brief 不完整或错误协议的导入文件必须被拒绝。@param tmp_path 临时目录。@param mutation 修改函数。"""
    data = asdict(make_preset())
    mutation(data)
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(mp.PresetError):
        mp.load_preset(path)


@pytest.mark.parametrize("state", ["RUNNING", "FAULT"])
def test_restore_requires_stopped(
    serial: FakeSerial, tmp_path: Path, state: str
) -> None:
    """@brief 运行或故障时不发送任何写入。@param serial 替身。@param tmp_path 目录。@param state 状态。"""
    serial.state = state
    with pytest.raises(mp.PresetError):
        client(serial).restore(make_preset(), tmp_path)
    assert not any(command.startswith("SET ") for command in serial.commands)


def test_capture_requires_output_disabled_and_no_pending(serial: FakeSerial) -> None:
    """@brief 软件停机不足以代替输出禁用确认。@param serial 替身。"""
    serial.output_disabled = False
    with pytest.raises(mp.PresetError):
        client(serial).capture("backup")
    serial.output_disabled = True
    serial.result = "PENDING"
    with pytest.raises(mp.PresetError):
        client(serial).capture("backup")


def test_restore_success_and_unchanged_skip(serial: FakeSerial, tmp_path: Path) -> None:
    """@brief 差异项逐项确认，来源最后恢复，成功后完整回读。@param serial 替身。@param tmp_path 目录。"""
    original = serial.presets[serial.algorithm]
    target = mp.copy_preset(
        original,
        "new",
        [
            "TARGET=2000",
            "CURRENT_LIMIT=1000",
            "SOURCE=LOCAL",
            "BRIGHTNESS=75",
            "ANIMATION=0",
        ],
    )
    backups = client(serial).restore(target, tmp_path)
    assert mp.load_preset(backups[0]).parameters == original.parameters
    writes = [command for command in serial.commands if command.startswith("SET ")]
    assert len(writes) == 5 and writes[-1] == "SET SOURCE LOCAL"
    assert not any(any(key in command for key in mp.FOC_READONLY) for command in writes)
    assert not mp.differences(serial.presets[serial.algorithm], target)
    serial.commands.clear()
    client(serial).restore(target, tmp_path)
    assert not any(command.startswith("SET ") for command in serial.commands)


def test_restore_algorithm_switch_preserves_both_old_blocks(
    serial: FakeSerial, tmp_path: Path
) -> None:
    """@brief 算法切换备份原选择与目标块，不覆盖隐藏配置。@param serial 替身。@param tmp_path 目录。"""
    original_foc = serial.presets["SENSORLESS_FOC"]
    target = mp.copy_preset(
        serial.presets["SENSORLESS_6STEP"], "sixstep", ["DUTY=200", "ENCODER_DIR=1"]
    )
    backups = client(serial).restore(target, tmp_path)
    assert len(backups) == 2
    assert mp.load_preset(backups[0]).algorithm == "SENSORLESS_FOC"
    assert mp.load_preset(backups[1]).algorithm == "SENSORLESS_6STEP"
    assert serial.presets["SENSORLESS_FOC"] == original_foc
    assert serial.algorithm == target.algorithm


def test_model_and_version_mismatch_stop(serial: FakeSerial, tmp_path: Path) -> None:
    """@brief 固件与只读模型不匹配时禁止参数写入。@param serial 替身。@param tmp_path 目录。"""
    original = make_preset()
    with pytest.raises(mp.PresetError):
        client(serial).restore(replace(original, firmware="old"), tmp_path)
    changed = dict(original.readonly)
    changed["LD"] = replace(changed["LD"], value=241)
    with pytest.raises(mp.PresetError):
        client(serial).restore(replace(original, readonly=changed), tmp_path)
    assert not any(command.startswith("SET ") for command in serial.commands)


@pytest.mark.parametrize(
    "failure",
    ["ERR SET RANGE", "OK SET BAD", TimeoutError("断线"), RuntimeError("坏行")],
)
def test_write_failure_stops_following_parameters(
    serial: FakeSerial, tmp_path: Path, failure: str | Exception
) -> None:
    """@brief 任一写失败不自动重试或继续下一项。@param serial 替身。@param tmp_path 目录。@param failure 故障。"""
    target = mp.copy_preset(make_preset(), "new", ["TARGET=2000", "CURRENT_LIMIT=1000"])
    serial.overrides["SET TARGET 2000"] = [failure]
    with pytest.raises(mp.PresetError):
        client(serial).restore(target, tmp_path)
    assert "SET CURRENT_LIMIT 1000" not in serial.commands
    assert len(list(tmp_path.glob("*.json"))) == 1


@pytest.mark.parametrize(
    "replies",
    [
        ["OK GET RESULT=PENDING", "OK GET RESULT=OK"],
        ["OK GET RESULT=RANGE"],
        ["OK GET RESULT=PENDING"] * 20,
    ],
)
def test_pending_execution_results(serial: FakeSerial, replies: list[str]) -> None:
    """@brief PENDING 必须等待最终结果，错误或超时不能当成功。@param serial 替身。@param replies 响应队列。"""
    serial.overrides["GET RESULT"] = ["OK GET RESULT=OK"] + replies
    if replies[-1] == "OK GET RESULT=OK":
        client(serial).set_value("TARGET", "2000")
    else:
        with pytest.raises(mp.PresetError):
            client(serial).set_value("TARGET", "2000")


def test_readback_and_final_snapshot_mismatch(
    serial: FakeSerial, tmp_path: Path
) -> None:
    """@brief 单项回读错误和备份期间变化均被发现。@param serial 替身。@param tmp_path 目录。"""
    serial.overrides["GET TARGET"] = ["OK GET TARGET=1999 DECIMALS=0 UNIT=RPM"]
    with pytest.raises(mp.PresetError):
        client(serial).set_value("TARGET", "2000")
    serial.overrides["GET TARGET"] = ["OK GET TARGET=1000 DECIMALS=0 UNIT=RPM"]
    with pytest.raises(mp.PresetError):
        client(serial).capture("backup")
    serial.overrides["GET SOURCE"] = ["OK GET SOURCE=LOCAL"]
    with pytest.raises(mp.PresetError):
        client(serial).capture("backup")


@pytest.mark.parametrize("save", ["ERROR", "PENDING"])
def test_storage_failures(serial: FakeSerial, save: str) -> None:
    """@brief 保存失败或持续待保存不能报告成功。@param serial 替身。@param save 状态。"""
    serial.save = save
    with pytest.raises(mp.PresetError):
        client(serial).saved()


def test_storage_none_default_and_layout_errors(serial: FakeSerial) -> None:
    """@brief 区分冷启动 NONE、DEFAULT 和布局失败。@param serial 替身。"""
    serial.storage_result = "NONE"
    serial.save = "DEFAULT"
    client(serial).saved()
    with pytest.raises(mp.PresetError):
        client(serial).saved(changed=True)
    serial.save = "SAVED"
    serial.layout_match = False
    with pytest.raises(mp.PresetError):
        client(serial).saved()
    serial.layout_match = True
    serial.storage_result = "VERIFY"
    with pytest.raises(mp.PresetError):
        client(serial).saved()
    serial.storage_result = "OK"
    serial.overrides["GET SAVE"] = ["OK GET SAVE=PENDING", "OK GET SAVE=SAVED"]
    client(serial).saved(changed=True)


def test_protocol_corruption_and_driver_error(serial: FakeSerial) -> None:
    """@brief 不匹配的协议、截断行与驱动错误禁止继续。@param serial 替身。"""
    serial.overrides["VERSION"] = ["OK VERSION FW=1 PROTOCOL=1.3"]
    with pytest.raises(mp.PresetError):
        client(serial).identity()
    serial.overrides["GET TARGET"] = ["OK GET TARGET=2000"]
    with pytest.raises(mp.PresetError):
        client(serial).parameter("TARGET")
    serial.driver_errors.append({"flags": 2})
    with pytest.raises(mp.PresetError):
        client(serial).identity()


def test_offline_cli(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """@brief 离线列表、查看、复制和计划不打开串口。@param tmp_path 目录。@param capsys 输出。"""
    prefix = ["--directory", str(tmp_path)]
    assert mp.main(prefix + ["list"]) == 0
    mp.write_preset(tmp_path / "reference.json", make_preset())
    assert mp.main(prefix + ["list"]) == 0
    assert mp.main(prefix + ["show", "reference"]) == 0
    assert mp.main(prefix + ["copy", "reference", "fast", "--set", "TARGET=2000"]) == 0
    assert mp.main(prefix + ["restore", "fast", "--dry-run"]) == 0
    assert mp.load_preset(tmp_path / "fast.json").parameters["TARGET"].value == 2000
    assert "SET TARGET 2000" in capsys.readouterr().out
    assert mp.main(prefix + ["backup", "reference"]) == 1
    (tmp_path / "broken.json").write_text("{", encoding="utf-8")
    assert mp.main(prefix + ["show", "broken"]) == 1


def test_online_cli_and_audit(
    serial: FakeSerial, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """@brief 命令行使用真实流程并保留成功与失败审计。@param serial 替身。@param tmp_path 目录。@param monkeypatch 替换工具。"""

    @contextmanager
    def fake_connected(port: str, baud: int) -> Iterator[mp.PresetClient]:
        """@brief 注入串口并保证关闭。@param port 端口。@param baud 波特率。@return 客户端。"""
        assert port == "COM10" and baud == 921600
        try:
            yield client(serial)
        finally:
            serial.close()

    monkeypatch.setattr(mp, "connected", fake_connected)
    prefix = ["--directory", str(tmp_path)]
    assert mp.main(prefix + ["backup", "original"]) == 0
    assert serial.closed
    assert mp.main(prefix + ["diff", "original"]) == 0
    assert mp.main(prefix + ["copy", "original", "new", "--set", "TARGET=2000"]) == 0
    assert mp.main(prefix + ["restore", "new"]) == 0
    serial.overrides["SET TARGET 1000"] = ["ERR SET RANGE"]
    assert mp.main(prefix + ["restore", "original"]) == 1
    audits = list((tmp_path / "restore_backups").glob("*_audit.json"))
    assert len(audits) == 2
    records = [json.loads(path.read_text(encoding="utf-8")) for path in audits]
    assert any(
        any(row.get("response") == "ERR SET RANGE" for row in log) for log in records
    )


def test_differences_and_non_windows(monkeypatch: pytest.MonkeyPatch) -> None:
    """@brief 差异明确标注版本/隐藏参数，非 Windows 不能访问设备。@param monkeypatch 替换工具。"""
    assert any(
        "版本" in line
        for line in mp.differences(
            replace(make_preset(), firmware="old"), make_preset()
        )
    )
    assert any(
        "隐藏" in line
        for line in mp.differences(make_preset("OPEN_6STEP"), make_preset())
    )
    monkeypatch.setattr(mp.sys, "platform", "linux")

    def attempt() -> None:
        """@brief 验证非 Windows 设备操作被拒绝。"""
        with mp.connected("COM10", 921600):
            pytest.fail("不应成功打开")

    with pytest.raises(mp.PresetError):
        attempt()
