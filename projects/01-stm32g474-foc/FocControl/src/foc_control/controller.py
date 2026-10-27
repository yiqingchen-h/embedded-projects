"""异步事务控制；保存等待可取消，STOP 不等待普通请求超时。"""

from __future__ import annotations

import asyncio
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, replace
from decimal import Decimal
from pathlib import Path
from typing import Unpack

from .binary_wave import TEST, VALID, BadWave, MixedDecoder, WaveBlock, WaveBuffer
from .haptic_feedback import HAPTIC_KEYS, HapticSample, config_command
from .models import (
    ALGORITHMS,
    ENCODER_ALGORITHMS,
    FOC_ALGORITHMS,
    MODEL_PARAMETERS,
    REQUIRED_PARAMETERS,
    STAGES,
    BoardSnapshot,
    ControlError,
    ParameterSpec,
    ParameterValue,
    SnapshotChanges,
    mode_supported,
    protocol_supported,
    specs_for,
)
from .position_feedback import PositionBuffer, PositionPoint, PositionSample
from .protocol import (
    BadLine,
    PlotPoint,
    TelemetryBuffer,
    command_bytes,
    fields,
    is_read_command,
    parse_telemetry,
    reply_matches,
)
from .storage import Preset, timestamp, writable_preset_keys
from .transport import Transport

# 保存合并窗口只影响普通操作，接收和优先停止保持运行。
SAVE_WAIT = 1.5


@dataclass
class Pending:
    """已经写入串口的请求，取消后仍需消费其迟到回执。"""

    command: str
    future: asyncio.Future[str]


class Session:
    """单个串口的异步请求协调器。"""

    def __init__(
        self,
        transport: Transport,
        read_only: bool,
        log: Callable[[str, str], None],
        telemetry: Callable[[str], None],
    ) -> None:
        """@brief 创建会话。@param transport 短调用串口。@param read_only 写入门禁。
        @param log 日志回调。@param telemetry 遥测回调。
        """
        self.transport = transport
        self.read_only = read_only
        self.log = log
        self.telemetry = telemetry
        self.decoder = MixedDecoder()
        self.on_binary: Callable[[WaveBlock], None] = lambda _block: None
        self.on_bad_wave: Callable[[], None] = lambda: None
        self.on_wire: Callable[[bytes], None] = lambda _data: None
        self.pending: deque[Pending] = deque()
        self.normal_lock = asyncio.Lock()
        self.wire_idle = asyncio.Event()
        self.wire_idle.set()
        self.reader: asyncio.Task[None] | None = None
        self.link_error = ""
        self.closed = False
        # 控制器身份握手期间撤销此项，避免对未知协议或错误波特率发送STOP。
        self.stop_on_failure = True
        # 原始字节仍完整落盘，事件只按秒合并坏波形，避免错误刷屏拖慢接收。
        self.bad_wave_count = 0
        self._bad_wave_logged = 0.0

    async def start(self) -> None:
        """@brief 启动唯一接收任务。"""
        self.reader = asyncio.create_task(self._receive())
        await asyncio.sleep(0)

    async def _receive(self) -> None:
        """@brief 持续接收、分帧和配对，绝不将遥测当回执。"""
        try:
            while not self.closed:
                data = self.transport.read()
                self.on_wire(data)
                if flags := getattr(self.transport, "rx_error_flags", 0):
                    self.on_bad_wave()
                    self.log("WARN", f"串口接收错误 flags=0x{flags:X}，保留缺口并继续接收")
                for line in self.decoder.feed(data):
                    if isinstance(line, WaveBlock):
                        self.on_binary(line)
                        continue
                    if isinstance(line, BadWave):
                        self.on_bad_wave()
                        self.bad_wave_count += 1
                        now = time.monotonic()
                        if self.bad_wave_count == 1 or now - self._bad_wave_logged >= 1:
                            self.log(
                                "WARN", f"已丢弃{self.bad_wave_count}个损坏波形块：{line.reason}"
                            )
                            self._bad_wave_logged = now
                        continue
                    if isinstance(line, BadLine):
                        self.log("ERROR", f"{line.reason}：{line.raw}")
                        if self.pending and not is_read_command(self.pending[0].command):
                            raise ControlError("命令响应期间收到损坏字节，结果未知")
                    elif line.startswith(("FOC_TRACE:", "DATA FOC ", "FOC:")):
                        self.log("DATA", line)
                        self.telemetry(line)
                    elif line.startswith(("OK ", "ERR ")):
                        self.log("RX", line)
                        self._deliver(line)
                    else:
                        self.log("ERROR", "未知响应：" + line)
                        if self.pending and not is_read_command(self.pending[0].command):
                            raise ControlError("收到无法识别的命令响应")
                await asyncio.sleep(0.002)
        except (OSError, ControlError) as error:
            self._fail(str(error))

    def _deliver(self, line: str) -> None:
        """@brief 匹配发出顺序并消费取消请求。@param line 完整回执。"""
        if not self.pending:
            self.log("INFO", "未配对的迟到响应：" + line)
            return
        while self.pending and not reply_matches(self.pending[0].command, line):
            if self.pending[0].future.cancelled():
                self.log("INFO", "丢弃取消请求：" + self.pending.popleft().command)
            elif is_read_command(self.pending[0].command):
                # 不可信文本或迟到的其它查询回执不能覆盖当前读取，也不使电机停机。
                self.log("WARN", f"忽略与 {self.pending[0].command} 不匹配的查询回执：{line}")
                return
            else:
                self._fail(f"回执与 {self.pending[0].command} 不匹配：{line}")
                return
        if self.pending:
            request = self.pending.popleft()
            if not request.future.done():
                request.future.set_result(line)
        if not self.pending:
            self.wire_idle.set()

    def _fail(self, reason: str) -> None:
        """@brief 链路失效时尽力STOP后释放，仍保持未知状态；不重放其它写命令。
        @param reason 首个原始原因，只读会话不发送STOP。
        """
        if self.closed:
            return
        self.link_error = reason
        self.closed = True
        if not self.read_only and self.stop_on_failure:
            try:
                self.transport.write(command_bytes("STOP", False))
                self.log("TX", "STOP")
                self.log("AUDIT", "链路失效，已尽力发送一次STOP；未收到有效回执，停机未确认")
            except OSError as error:
                self.log("AUDIT", f"链路失效，尽力STOP发送失败：{error}；停机未确认")
        self.transport.close()
        while self.pending:
            future = self.pending.popleft().future
            if not future.done():
                future.set_exception(ControlError(reason))
        self.wire_idle.set()
        self.log("ERROR", reason)

    async def ask(self, command: str, priority: bool = False, timeout: float = 3.0) -> str:
        """@brief 发送并配对，STOP 可绕过普通锁。@param command 单行命令。
        @param priority 是否优先停止。@param timeout 超时秒数。@return 原始回执。
        """
        payload = command_bytes(command, self.read_only)
        if priority and command != "STOP":
            raise ControlError("只有 STOP 可以绕过普通队列")
        if self.closed:
            raise ControlError(self.link_error or "串口已关闭")
        if priority:
            return await self._exchange(command, payload, timeout)
        async with self.normal_lock:
            try:
                await asyncio.wait_for(self.wire_idle.wait(), timeout)
            except TimeoutError as error:
                self._fail("上条命令的响应尚未排空")
                raise ControlError(self.link_error) from error
            return await self._exchange(command, payload, timeout)

    async def _exchange(self, command: str, payload: bytes, timeout: float) -> str:
        """@brief 只读查询最多三次恢复，写命令只发送一次并保持严格确认。
        @param command 命令。@param payload 字节。@param timeout 每次等待秒数。
        @return 匹配回执；持续失联仍关闭并报告未知状态。
        """
        attempts = 3 if is_read_command(command) else 1
        for attempt in range(attempts):
            future: asyncio.Future[str] = asyncio.get_running_loop().create_future()
            request = Pending(command, future)
            self.pending.append(request)
            self.wire_idle.clear()
            try:
                self.transport.write(payload)
                self.log("TX", command)
                response = await asyncio.wait_for(asyncio.shield(future), timeout)
                if response != "ERR LINE CORRUPT" or attempt == attempts - 1:
                    return response
                reason = "板卡拒收损坏行"
            except asyncio.CancelledError:
                future.cancel()
                raise
            except TimeoutError as error:
                future.cancel()
                if attempt == attempts - 1:
                    self._fail(f"{command} 连续{attempts}次未获有效响应；板卡状态未知")
                    raise ControlError(self.link_error) from error
                if request in self.pending:
                    self.pending.remove(request)
                if not self.pending:
                    self.wire_idle.set()
                reason = "本次查询丢失或回复损坏"
            except OSError as error:
                future.cancel()
                self._fail(f"{command} 失败：{error}；板卡状态未知")
                raise ControlError(self.link_error) from error
            self.log("WARN", f"{reason}，仅重读 {command}（{attempt + 2}/{attempts}）")
            await asyncio.sleep(0.05)
        raise ControlError("查询恢复未完成")

    async def close(self) -> None:
        """@brief 取消等待并释放资源；此接口不冒充板卡停止。"""
        self.closed = True
        if self.reader is not None:
            self.reader.cancel()
            try:
                await self.reader
            except asyncio.CancelledError:
                pass
        while self.pending:
            self.pending.popleft().future.cancel()
        self.transport.close()


class Controller:
    """当前固件能力与可中断业务流程，后台独占持有。"""

    def __init__(
        self,
        transport: Transport,
        data_dir: Path,
        read_only: bool = False,
        on_state: Callable[[BoardSnapshot], None] = lambda _state: None,
        on_log: Callable[[str, str], None] = lambda _kind, _text: None,
        on_point: Callable[[PlotPoint], None] = lambda _point: None,
    ) -> None:
        """@brief 创建控制器。@param transport 串口。@param data_dir 数据目录。
        @param read_only 只读门禁。@param on_state 快照回调。@param on_log 日志。
        @param on_point 图表记录。
        """
        self.data_dir = data_dir
        self.on_state, self.on_log, self.on_point = on_state, on_log, on_point
        self.state = BoardSnapshot(read_only=read_only, busy="正在验证连接")
        self.session = Session(transport, read_only, on_log, self._telemetry)
        self.session.stop_on_failure = False
        self.buffer = TelemetryBuffer()
        self.wave_buffer = WaveBuffer()
        self.position_buffer = PositionBuffer()
        self.on_position: Callable[[PositionPoint], None] = lambda _point: None
        self.session.on_binary = self._binary
        self.session.on_bad_wave = self.wave_buffer.mark_corrupt
        self._wave_iq: float | None = None
        self._wave_id: float | None = None
        # 0表示日常监视：只查询状态和真实Id/Iq，用户按需显式选择高速波形。
        self.preferred_wave_baud = 0
        self.fallback_baud = 2000000
        self.update(link_baud=int(getattr(transport, "baud", 921600)))
        self.original_print = "OFF"
        self.stop_requested = False

    def notice(self, message: str) -> None:
        """@brief 保留能力降级原因且避免轮询重复堆积。@param message 提示。"""
        if message not in self.state.notices:
            self.update(notices=(*self.state.notices, message))
            self.on_log("INFO", message)

    def require_control(self) -> None:
        """@brief 在业务入口校验控制能力，不能只依赖界面禁用。"""
        if not self.state.writable:
            raise ControlError(
                "当前连接仅可监视：" + ("；".join(self.state.notices) or "只读模式或能力未确认")
            )

    async def optional(self, command: str) -> str | None:
        """@brief 只读探测可选能力，真实通信失败仍上报。@param command 查询。@return 回执或缺失。"""
        response = await self.session.ask(command)
        if response.startswith("OK "):
            return response
        if response.split()[-1] in (
            "ARGUMENT",
            "UNKNOWN",
            "UNSUPPORTED",
            "UNAVAILABLE",
            "NOT_READY",
        ):
            self.on_log("INFO", f"可选能力不可用：{command}：{response}")
            return None
        raise ControlError(f"{command}：{response}")

    def update(self, **changes: Unpack[SnapshotChanges]) -> None:
        """@brief 发布新快照。@param changes 新状态字段。"""
        self.state = replace(self.state, revision=self.state.revision + 1, **changes)
        self.on_state(self.state)

    def error(self, message: str) -> None:
        """@brief 持续保留错误且反映链路未知。@param message 错误原文。"""
        changes: SnapshotChanges = {"error": message, "busy": ""}
        if self.session.link_error:
            changes.update(
                {
                    "connected": False,
                    "run": "UNKNOWN",
                    "closed": False,
                    "stage": "状态未知",
                    "ready": False,
                }
            )
        self.update(**changes)
        self.on_log("ERROR", message)

    async def initialize(self) -> None:
        """@brief 按协议与实际能力连接，固件升级不依赖补丁号白名单。"""
        await self.session.start()
        version = fields(await self.read_identity())
        self.update(firmware=version.get("FW", ""), protocol=version.get("PROTOCOL", ""))
        if not self.state.firmware or not self.state.protocol:
            raise ControlError("VERSION 缺少固件或协议身份")
        self.update(connected=True)
        if not protocol_supported(self.state.protocol):
            self.session.read_only = True
            self.update(read_only=True, busy="")
            self.notice(f"协议 {self.state.protocol} 尚未适配，已保留连接并切为只读")
            return
        self.session.stop_on_failure = True
        await self.refresh_status()
        algorithm, mode, source = (
            await self.get("ALGORITHM"),
            await self.get("MODE"),
            await self.get("SOURCE"),
        )
        self.update(algorithm=algorithm, mode=mode, source=source)
        encoder = await self.optional("MOTOR ENCODER")
        self.update(
            encoder_available=encoder is not None,
            available_algorithms=tuple(
                key for key in ALGORITHMS if key not in ENCODER_ALGORITHMS or encoder is not None
            ),
        )
        if encoder is not None:
            self.update(encoder_calibration=fields(encoder).get("CALIBRATION", ""))
        if not mode_supported(algorithm, mode):
            self.notice(f"当前 {algorithm}/{mode} 尚无控制适配，保留状态监视与停止")
            self.update(busy="")
            return
        printing = await self.optional("GET FOC_PRINT")
        if printing is not None:
            self.original_print = fields(printing).get("FOC_PRINT", "OFF")
        self.update(trace_available=printing is not None)
        if tuple(int(part) for part in self.state.protocol.split(".")) >= (1, 3):
            capabilities = await self.optional("GET WAVE_CAPS")
            self.update(
                wave_available=capabilities is not None
                and fields(capabilities).get("WAVE_CAPS") == "BIN1"
            )
        haptic_caps = await self.optional("GET HAPTIC_CAPS")
        self.update(
            haptic_available=haptic_caps is not None
            and fields(haptic_caps).get("HAPTIC_CAPS") == "HAP1"
        )
        position_caps = await self.optional("GET POSITION_CAPS")
        self.update(
            position_available=position_caps is not None
            and fields(position_caps).get("POSITION_CAPS") == "POS1"
        )
        await self.read_parameters()
        if self.state.mode == "POSITION" and self.state.position_available:
            await self.read_position()
        if self.state.run != "RUNNING":
            await self.read_diagnostics()
        if self.state.writable and self.state.run == "STOPPED" and self.state.fault == "NONE":
            try:
                await self.idle()
                if source != "REMOTE":
                    await self.set_value("SOURCE", "REMOTE")
                    await self.saved(changed=True)
                    self.update(source="REMOTE")
            except ControlError as error:
                if self.session.closed:
                    raise
                self.notice(f"已连接，暂未取得上位机控制：{error}")
        elif self.state.writable and self.state.run == "RUNNING" and algorithm in FOC_ALGORITHMS:
            await self.enable_trace()
        self.update(busy="", error="")

    async def read_identity(self) -> str:
        """@brief 仅身份未确认时尝试另一个已知速率，所有探测只发送VERSION。@return 身份。"""
        try:
            return await self._read_identity_once()
        except (OSError, ControlError):
            reopen = getattr(self.session.transport, "reopen", None)
            if reopen is None or self.stop_requested:
                raise
            old = self.session
            alternate = 921600 if self.state.link_baud != 921600 else self.fallback_baud
            if alternate == self.state.link_baud:
                raise
            self.on_log("AUDIT", f"身份尚未确认，仅以{alternate}重试一次VERSION；未发送控制写入")
            await old.close()
            reopen(alternate)
            self.session = Session(old.transport, old.read_only, self.on_log, self._telemetry)
            self.session.stop_on_failure = False
            self.session.on_binary = self._binary
            self.session.on_bad_wave = self.wave_buffer.mark_corrupt
            self.session.on_wire = old.on_wire
            self.update(link_baud=alternate)
            await self.session.start()
            return await self._read_identity_once()

    async def _read_identity_once(self) -> str:
        """@brief 身份查询复用只读恢复，每档三次一秒等待，不重复叠加重试。@return VERSION。"""
        response = await self.session.ask("VERSION", timeout=1.0)
        if not response.startswith("OK "):
            raise ControlError("VERSION：" + response)
        return response

    async def ok(self, command: str) -> str:
        """@brief 请求且要求 OK。@param command 命令。@return 原文。"""
        response = await self.session.ask(command)
        if not response.startswith("OK "):
            raise ControlError(f"{command}：{response}")
        return response

    async def get(self, key: str) -> str:
        """@brief 读取标量。@param key 名称。@return 等号后原文。"""
        prefix = f"OK GET {key}="
        response = await self.ok(f"GET {key}")
        if not response.startswith(prefix):
            raise ControlError(f"{key} 响应错配：{response}")
        return response[len(prefix) :]

    async def parameter(self, key: str) -> ParameterValue:
        """@brief 读取完整数值参数。@param key 参数。@return 真实尺度。"""
        return ParameterValue.parse(await self.ok(f"GET {key}"), key)

    async def refresh_status(self) -> None:
        """@brief 六步的查询限制代表仍在运行，停机后恢复详细读取。"""
        response = await self.session.ask("STATUS")
        if response == "ERR RUNNING STOP TO READ":
            self.update(run="RUNNING", closed=False, stage="六步运行 · 实时信息受限")
            return
        if not response.startswith("OK STATUS "):
            raise ControlError("STATUS：" + response)
        data = fields(response)
        if data.get("MOTOR") not in ("STOPPED", "RUNNING", "FAULT"):
            raise ControlError("STATUS 缺少有效电机状态")
        run = data["MOTOR"]
        algorithm, mode = data.get("ALGORITHM", self.state.algorithm), data.get(
            "MODE", self.state.mode
        )
        if (algorithm, mode) != (self.state.algorithm, self.state.mode):
            # STATUS发现真实选择变化，立即撤销旧单位下的编辑能力，随后完整重读。
            self.update(
                algorithm=algorithm,
                mode=mode,
                parameters={},
                parameter_specs={},
                control_available=False,
                closed=False,
            )
        self.update(
            run=run,
            ready=data.get("READY") == "1",
            closed=self.state.closed if run == "RUNNING" else False,
            stage=self.state.stage if run == "RUNNING" else "停机",
        )

    async def read_parameters(self) -> dict[str, ParameterValue]:
        """@brief 读取存在的参数并合并模型及可选范围元数据。@return 实际参数表。"""
        result: dict[str, ParameterValue] = {}
        responses: dict[str, str] = {}
        for key in specs_for(self.state.algorithm, mode=self.state.mode):
            if key in HAPTIC_KEYS:
                continue
            response = await self.optional(f"GET {key}")
            if response is not None:
                result[key] = ParameterValue.parse(response, key)
                responses[key] = response
        if self.state.mode == "HAPTIC" and self.state.haptic_available:
            await self.read_haptic()
            assert self.state.haptic is not None
            result.update(self.state.haptic.parameters())
            for key in HAPTIC_KEYS:
                responses[key] = ""
        profile = specs_for(self.state.algorithm, result, self.state.mode)
        definitions: dict[str, ParameterSpec] = {}
        usable: set[str] = set()
        for key, value in result.items():
            definition = profile[key]
            metadata = fields(responses[key])
            if "MIN" in metadata and "MAX" in metadata:
                try:
                    minimum, maximum = int(metadata["MIN"]), int(metadata["MAX"])
                except ValueError as error:
                    raise ControlError(f"{key} 的范围元数据无效") from error
                if not -2147483647 <= minimum <= maximum <= 2147483647:
                    raise ControlError(f"{key} 的范围元数据无效")
                definition = replace(definition, minimum=minimum, maximum=maximum)
            if (
                self.state.mode in ("CURRENT", "POSITION", "HAPTIC")
                and key == "CURRENT_LIMIT"
                or self.state.mode == "CURRENT"
                and key == "TARGET"
            ):
                # CURRENT仍固定0.5A；POSITION只在本板已知0.8A范围内合并固件能力。
                original = profile[key]
                definition = replace(
                    definition,
                    minimum=max(original.minimum, definition.minimum),
                    maximum=min(original.maximum, definition.maximum),
                )
            if metadata.get("EDITABLE") == "0":
                definition = replace(definition, editable=False)
            if (
                self.state.mode == "POSITION"
                and key == "CURRENT_LIMIT"
                and not {"MIN", "MAX", "EDITABLE"} <= metadata.keys()
            ):
                # 旧位置固件仅提供只读0.5A；缺少显式能力时继续显示真实值。
                definition = replace(definition, editable=False)
            if (value.decimals, value.unit) != (definition.decimals, definition.unit):
                definition = replace(
                    definition,
                    decimals=value.decimals,
                    unit=value.unit,
                    editable=False,
                    factor=Decimal(1),
                    display_unit="",
                )
                self.notice(f"{key} 单位已变化，仅显示实际读数")
            else:
                usable.add(key)
                if definition.editable:
                    try:
                        definition.validate(value)
                    except ControlError:
                        definition = replace(definition, editable=False)
                        self.notice(f"{key} 超出已知编辑范围，仅显示；固件可追加 MIN/MAX 元数据")
            if not definition.editable:
                definition = replace(definition, minimum=value.value, maximum=value.value)
            definitions[key] = definition
        missing = set(profile) - set(result)
        if missing:
            self.notice("固件未提供以下参数，已隐藏：" + ", ".join(sorted(missing)))
        required = (
            REQUIRED_PARAMETERS - {"TARGET"} if self.state.mode == "HAPTIC" else REQUIRED_PARAMETERS
        )
        core = required <= usable
        if self.state.mode == "HAPTIC":
            core = core and self.state.haptic_available and set(HAPTIC_KEYS) <= usable
        if self.state.mode in ("CURRENT", "POSITION"):
            for key in ("TARGET", "CURRENT_LIMIT"):
                if key in result:
                    try:
                        profile[key].validate(result[key])
                    except ControlError:
                        core = False
            core = core and definitions.get("TARGET", profile["TARGET"]).editable
        if self.state.mode == "POSITION":
            core = (
                core
                and self.state.position_available
                and {"POSITION_KP", "SPEED_LIMIT", "RAMP"} <= usable
            )
        if not core:
            self.notice("核心参数缺失或单位不匹配，保留监视与停止")
        self.update(parameters=result, parameter_specs=definitions, control_available=core)
        return result

    async def read_diagnostics(self) -> None:
        """@brief 停机读取完整诊断及实际输出状态。"""
        records = dict(self.state.diagnostics)
        for command in ("MOTOR", "MOTOR RUN", "MOTOR FOC REST", "STORAGE"):
            response = await self.optional(command)
            if response is None:
                self.notice(f"可选诊断 {command} 不可用")
                if command in ("MOTOR FOC REST", "STORAGE"):
                    self.update(control_available=False)
                continue
            records[command] = response
            data = fields(response)
            changes: SnapshotChanges = {}
            if "FAULT" in data:
                changes["fault"] = data["FAULT"]
            if "BUS_MV" in data:
                changes["bus_v"] = int(data["BUS_MV"]) / 1000
                changes["data_time"] = timestamp()
            if "OUTPUT_DISABLED" in data:
                changes["output_disabled"] = data["OUTPUT_DISABLED"] == "1"
                changes["still"] = data.get("STILL") == "1" if "STILL" in data else None
            if command == "STORAGE" and data.get("MATCH") != "1":
                changes["control_available"] = False
                self.notice("存储布局尚未确认匹配，保留监视与停止")
            self.update(**changes)
        if self.state.algorithm == "SENSORLESS_6STEP":
            response = await self.optional("MOTOR BEMF")
            if response is not None:
                records["MOTOR BEMF"] = response
        if self.state.encoder_available:
            await self.read_encoder()
        saving = await self.optional("GET SAVE")
        if saving is None:
            self.notice("固件未提供保存状态，保留监视与停止")
            self.update(control_available=False)
        self.update(
            diagnostics=records, save=fields(saving).get("SAVE", "") if saving else "不可用"
        )

    async def idle(self, allow_fault: bool = False) -> None:
        """@brief 修改前重新读取停机前提。@param allow_fault 清故障是否允许 FAULT。"""
        await self.refresh_status()
        allowed = ("STOPPED", "FAULT") if allow_fault else ("STOPPED",)
        if self.state.run not in allowed:
            raise ControlError("请先停止电机，再执行该操作")
        if await self.get("RESULT") == "PENDING":
            raise ControlError("板卡有在途请求")
        rest = fields(await self.ok("MOTOR FOC REST"))
        if rest.get("OUTPUT_DISABLED") != "1":
            raise ControlError("功率输出尚未确认禁用")
        if not allow_fault and rest.get("FAULT") != "NONE":
            raise ControlError("请先处理板卡故障")
        storage = fields(await self.ok("STORAGE"))
        if storage.get("MATCH") != "1":
            raise ControlError("Flash 布局不匹配，拒绝修改参数")
        self.update(
            output_disabled=True,
            still=rest.get("STILL") == "1" if "STILL" in rest else None,
            fault=rest.get("FAULT", ""),
        )

    async def await_result(self) -> None:
        """@brief 区分请求被接受和后端执行完成。"""
        for _ in range(40):
            result = await self.get("RESULT")
            if result == "OK":
                return
            if result != "PENDING":
                raise ControlError("板卡执行失败：" + result)
            await asyncio.sleep(0.05)
        raise ControlError("板卡请求持续 PENDING")

    async def set_value(self, key: str, value: str) -> None:
        """@brief 写单项并确认执行与回读。@param key 参数。@param value 串口原始值。"""
        self.require_control()
        response = await self.ok(f"SET {key} {value}")
        if response == "OK SET PENDING":
            await self.await_result()
        elif response != f"OK SET {key}={value}":
            raise ControlError("SET 回执不完整：" + response)
        if (await self.get(key)).split(" ", 1)[0] != value:
            raise ControlError(f"{key} 回读不一致，停止后续写入")

    async def saved(self, changed: bool = False) -> None:
        """@brief 写入后保留Flash静默窗口，已保存的只读确认不重复等待。
        @param changed 本次是否提交了持久配置修改。
        """
        status = "PENDING" if changed else await self.get("SAVE")
        self.update(save=status)
        for _ in range(4):
            if status != "PENDING":
                break
            await asyncio.sleep(SAVE_WAIT)
            status = await self.get("SAVE")
            self.update(save=status)
        if status != "SAVED" and not (status == "DEFAULT" and not changed):
            raise ControlError("参数保存未完成：" + status)
        storage = fields(await self.ok("STORAGE"))
        if storage.get("MATCH") != "1" or storage.get("RESULT") not in ("NONE", "OK"):
            raise ControlError("Flash 布局或保存校验异常")

    async def apply(self, changes: dict[str, ParameterValue]) -> None:
        """@brief 应用界面草稿。@param changes 已按尺度编码的修改。"""
        self.require_control()
        selection = (self.state.algorithm, self.state.mode)
        await self.refresh_status()
        if selection != (self.state.algorithm, self.state.mode):
            if mode_supported(self.state.algorithm, self.state.mode):
                await self.read_parameters()
            raise ControlError("板卡算法或模式已变化，请按新单位重新输入")
        self._validate_changes(changes)
        if self.state.mode == "HAPTIC":
            await self._apply_haptic(changes)
            return
        running = self.state.run == "RUNNING"
        if running:
            if set(changes) - {"TARGET"}:
                raise ControlError(
                    "运行中只能调整目标 Iq"
                    if self.state.mode == "CURRENT"
                    else (
                        "运行中只能调整目标位置"
                        if self.state.mode == "POSITION"
                        else "运行中只能调整目标转速"
                    )
                )
            if self.state.algorithm in FOC_ALGORITHMS:
                await self.read_drive()
                if not self.state.closed:
                    raise ControlError("进入实际 CLOSED 后才能调整目标")
        else:
            await self.idle()
        for key, value in changes.items():
            await self.set_value(key, str(value.value))
            actual = await self.parameter(key)
            if actual != value:
                raise ControlError(f"{key} 的完整回读不一致")
            self.update(parameters={**self.state.parameters, key: actual})
        volatile_position = self.state.mode == "POSITION" and set(changes) <= {"TARGET"}
        if not running and changes and not volatile_position:
            await self.saved(changed=True)
        elif running and changes and self.state.mode != "POSITION":
            self.update(save="PENDING")

    def _validate_changes(self, changes: dict[str, ParameterValue]) -> None:
        """@brief 在任何写入及标定动作前检查整份草稿。@param changes 当前模式的草稿。"""
        profile = self.state.parameter_specs
        for key, value in changes.items():
            if key not in profile or not profile[key].editable:
                raise ControlError("参数不允许编辑：" + key)
            profile[key].validate(value)

    async def start_motor(self, changes: dict[str, ParameterValue]) -> None:
        """@brief 应用草稿、确认保存后普通 START。@param changes 草稿。"""
        self.require_control()
        await self.idle()
        if self.state.source != "REMOTE":
            raise ControlError("先切换为上位机控制")
        await self.apply(changes)
        if not changes:
            await self.saved()
        if self.state.algorithm in ENCODER_ALGORITHMS:
            await self.read_encoder()
            if self.state.encoder_calibration != "VALID":
                raise ControlError("编码器尚未完成本次上电七点标定，请先点击编码器标定")
        if self.state.algorithm in FOC_ALGORITHMS:
            await self.enable_trace()
        self.buffer.clear()
        if self.state.mode == "POSITION":
            await self.read_position()
            if self.state.position is None or not self.state.position.valid:
                raise ControlError("机械零点无效；请点击一键初始化，自动保存参数、按需标定并置零")
        self.wave_buffer.clear()
        self.position_buffer.clear()
        self.stop_requested = False
        response = await self.ok("START")
        if response != "OK START PENDING":
            raise ControlError("START 回执不完整：" + response)
        await self.await_result()
        await self.refresh_status()
        if self.state.run != "RUNNING":
            await self.read_diagnostics()
            raise ControlError("START 执行后未进入运行：" + self.state.fault)
        self.update(output_disabled=False, error="")

    async def stop_motor(self) -> None:
        """@brief 优先 STOP，只有状态和输出回读共同证明停止。"""
        if not self.state.can_stop:
            raise ControlError("当前连接无法发送停止命令")
        self.stop_requested = True
        self.update(closed=False, stage="正在停止", error="")
        response = await self.session.ask("STOP", priority=True)
        if response != "OK STOP PENDING":
            raise ControlError("停止未确认：" + response)
        await self.await_result()
        await self.refresh_status()
        if self.state.run == "RUNNING":
            raise ControlError("停止未确认：板卡仍在运行")
        rest = fields(await self.ok("MOTOR FOC REST"))
        if rest.get("OUTPUT_DISABLED") != "1":
            raise ControlError("停止未确认：功率输出未禁用")
        self.update(output_disabled=True, closed=False, stage="停机", fault=rest.get("FAULT", ""))
        await self.read_diagnostics()

    async def disconnect(self) -> None:
        """@brief 普通模式先停机，只读模式仅释放连接。"""
        if not self.state.read_only:
            await self.stop_motor()
            if self.state.wave_available:
                await self.ok("SET FOC_PRINT OFF")
                await self.ok("SET WAVE_RATE 5000")
                if self.state.link_baud != 921600:
                    await self.change_baud(921600)
            if self.state.trace_available and self.state.algorithm in FOC_ALGORITHMS:
                await self.ok(f"SET FOC_PRINT {self.original_print}")
        await self.session.close()
        self.update(connected=False, ready=False, closed=False, stage="停机", busy="")

    async def take_remote(self) -> None:
        """@brief 在停机前提下显式重新取得控制来源。"""
        await self.idle()
        await self.set_value("SOURCE", "REMOTE")
        await self.saved(changed=True)
        self.update(source="REMOTE")

    async def select_algorithm(self, algorithm: str, mode: str = "SPEED") -> None:
        """@brief 一次切换算法及所需模式后回读，避免标定恢复时再次切模式。
        @param algorithm 新算法。@param mode 恢复的模式，普通切换默认为SPEED。
        """
        self.require_control()
        if algorithm not in self.state.available_algorithms:
            raise ControlError(f"板卡未提供算法能力：{algorithm}")
        if (
            not mode_supported(algorithm, mode)
            or mode == "POSITION"
            and not self.state.position_available
            or mode == "HAPTIC"
            and not self.state.haptic_available
        ):
            raise ControlError("该算法不支持所选模式")
        await self.idle()
        if mode != "SPEED" and self.state.still is not True:
            raise ControlError("请等待机械轴静止后恢复模式")
        await self.set_value("ALGORITHM", algorithm)
        if await self.get("MODE") != mode:
            await self.set_value("MODE", mode)
        self.update(
            algorithm=algorithm,
            mode=mode,
            closed=False,
            stage="停机",
            parameters={},
            parameter_specs={},
        )
        await self.saved(changed=True)
        await self.read_parameters()
        await self.read_diagnostics()
        self.buffer.clear()
        self.wave_buffer.clear()

    async def select_mode(self, mode: str) -> None:
        """@brief 备份后停机切模式，重新读取目标单位与独立目标块。@param mode SPEED或CURRENT。"""
        self.require_control()
        if not mode_supported(self.state.algorithm, mode):
            raise ControlError("该模式只支持有感 FOC")
        if mode == "HAPTIC" and not self.state.haptic_available:
            raise ControlError("旧固件未声明虚拟编码器能力")
        if mode == "POSITION" and not self.state.position_available:
            raise ControlError("当前固件没有位置闭环能力")
        await self.idle()
        if self.state.still is not True:
            raise ControlError("请等待机械轴静止后切换模式")
        if self.state.mode == mode:
            return
        backup = await self.capture("before_mode_" + str(time.time_ns()))
        path = self.data_dir / "restore_backups" / f"{backup.name}.json"
        backup.save(path)
        self.on_log("AUDIT", "模式切换前备份：" + str(path))
        try:
            await self.set_value("MODE", mode)
        except ControlError:
            # 被旧固件拒绝或回读不符时，只恢复真实界面，不重发写入或自动回滚。
            await self.reload_selection()
            raise
        except asyncio.CancelledError:
            self.update(parameters={}, parameter_specs={}, control_available=False)
            raise
        self.update(
            mode=mode,
            closed=False,
            stage="停机",
            parameters={},
            parameter_specs={},
            control_available=False,
        )
        await self.saved(changed=True)
        await self.read_parameters()
        await self.read_diagnostics()
        self.buffer.clear()
        self.wave_buffer.clear()
        self._wave_id = self._wave_iq = None
        self.position_buffer.clear()
        if mode == "HAPTIC":
            await self.read_haptic()
        if mode == "POSITION":
            await self.read_position()

    async def reload_selection(self) -> None:
        """@brief 只读恢复真实算法/模式和参数，失败不继续使用旧单位。"""
        self.update(control_available=False, parameters={}, parameter_specs={})
        algorithm, mode = await self.get("ALGORITHM"), await self.get("MODE")
        self.update(algorithm=algorithm, mode=mode)
        if mode_supported(algorithm, mode):
            await self.read_parameters()
        else:
            self.notice(f"当前 {algorithm}/{mode} 尚无控制适配，保留状态监视与停止")

    async def clear_fault(self) -> None:
        """@brief 用户明确触发的重配置与校准，不自动启动。"""
        self.require_control()
        await self.idle(allow_fault=True)
        if await self.ok("CLEAR") != "OK CLEAR PENDING":
            raise ControlError("CLEAR 回执无效")
        await self.await_result()
        await self.refresh_status()
        await self.read_diagnostics()
        await self.read_parameters()
        self.update(error="")

    async def read_drive(self) -> None:
        """@brief 当前 ACTIVE/CLOSED 才代表已接管，历史 CLOSED_SEEN 不作为依据。"""
        data = fields(await self.ok("MOTOR FOC DRIVE"))
        active = data.get("ACTIVE") == "1"
        closed = active and data.get("STATE") == "CLOSED" and data.get("AWAITING_COMMIT") == "0"
        stages = {
            "STOPPED": "停机",
            "CALIBRATE": "校准",
            "ALIGN": "定位",
            "I_F_RAMP": "升频",
            "QUALIFY": "资格确认",
            "BLEND": "融合",
            "CLOSED": "闭环",
            "WAIT_ENCODER": "等待编码器",
        }
        self.update(
            closed=closed,
            stage=stages.get(data.get("STATE", ""), data.get("STATE", "等待状态")),
            estimated_rpm=(
                int(data["RPM_MILLI"]) / 1000
                if active and "RPM_MILLI" in data
                else (
                    float(data["EST_RPM"])
                    if active and "EST_RPM" in data
                    else self.state.estimated_rpm
                )
            ),
            iq_a=int(data["IQ_MA"]) / 1000 if active and "IQ_MA" in data else self.state.iq_a,
            id_a=int(data["ID_MA"]) / 1000 if active and "ID_MA" in data else self.state.id_a,
            fault=data.get("FAULT", self.state.fault),
            data_time=timestamp(),
        )

    async def read_position(self) -> None:
        """读取原始多圈位置并更新独立曲线，禁止从RPM或电角度推导位置。"""
        if not self.state.position_available:
            return
        sample = PositionSample.parse(await self.ok("MOTOR FOC POSITION"))
        self.update(position=sample)
        point = self.position_buffer.append(sample)
        if point is not None:
            self.on_position(point)

    async def zero_position(self, changes: dict[str, ParameterValue] | None = None) -> None:
        """@brief 已标定时备份并保存草稿后置零。@param changes 位置模式草稿。"""
        self.require_control()
        await self.idle()
        if self.state.mode != "POSITION" or not self.state.position_available:
            raise ControlError("请先选择有感FOC位置模式")
        if self.state.still is not True:
            raise ControlError("请等待机械轴静止")
        draft = changes or {}
        self._validate_changes(draft)
        await self.read_encoder()
        if self.state.encoder_calibration != "VALID":
            raise ControlError("编码器需七点标定，请点击一键初始化")
        path = await self._initialization_backup("before_position_zero_")
        await self.apply({key: value for key, value in draft.items() if key != "TARGET"})
        self.on_log("AUDIT", "机械置零前参数备份：" + str(path))
        await self._set_position_origin(draft.get("TARGET"))

    async def initialize_position(
        self, changes: dict[str, ParameterValue] | None = None, *, force_calibration: bool = False
    ) -> None:
        """@brief 一次完成备份、按需标定、配置保存及机械置零，完成后保持停机。
        @param changes 位置模式草稿，目标在置零后恢复。@param force_calibration 是否强制重新七点。
        """
        self.require_control()
        await self.idle()
        if self.state.mode != "POSITION" or not self.state.position_available:
            raise ControlError("请先选择有感FOC位置模式")
        if self.state.source != "REMOTE" or self.state.still is not True:
            raise ControlError("初始化需要上位机控制且机械轴静止")
        draft = changes or {}
        self._validate_changes(draft)
        target = draft.get("TARGET", self.state.parameters["TARGET"])
        previous_busy = self.state.busy
        try:
            self.update(busy="1/4 备份当前配置")
            await self._initialization_backup("before_initialize_")
            await self.read_encoder()
            if force_calibration or self.state.encoder_calibration != "VALID":
                self.update(busy="2/4 编码器七点标定")
                await self._calibrate_encoder()
            else:
                self.on_log("AUDIT", "本次上电标定有效，跳过重复七点标定")
            self.update(busy="3/4 保存初始化参数")
            await self.apply({key: value for key, value in draft.items() if key != "TARGET"})
            self.update(busy="4/4 建立机械零点")
            await self._set_position_origin(target)
            self.on_log("AUDIT", "位置初始化完成，保持停机，可点击启动")
        finally:
            self.update(busy=previous_busy)

    async def _initialization_backup(self, prefix: str) -> Path:
        """@brief 初始化事务只进行一次双读备份。@param prefix 文件名前缀。@return 备份路径。"""
        backup = await self.capture(prefix + str(time.time_ns()))
        path = self.data_dir / "restore_backups" / f"{backup.name}.json"
        backup.save(path)
        self.on_log("AUDIT", "初始化前参数备份：" + str(path))
        return path

    async def _set_position_origin(self, target: ParameterValue | None) -> None:
        """@brief 确认配置已保存、轴静止及资格有效后置零，最后回读易失目标。
        @param target 置零后的目标；为空时使用板卡置零后的零目标。
        """
        await self.saved()
        await self.idle()
        await self.read_encoder()
        if self.state.still is not True or self.state.encoder_calibration != "VALID":
            raise ControlError("机械置零需要轴静止且本次上电标定有效")
        if await self.ok("MOTOR FOC POSITION ZERO") != "OK FOC POSITION ZERO":
            raise ControlError("机械置零回执未确认")
        self.position_buffer.clear()
        # ZERO会清除板卡目标，先同步真实零目标，再恢复用户原有的目标草稿。
        actual = await self.parameter("TARGET")
        self.update(parameters={**self.state.parameters, "TARGET": actual})
        if target is not None:
            await self.apply({"TARGET": target})
        await self.read_position()
        if self.state.position is None or not self.state.position.valid:
            raise ControlError("机械零点未建立，请核对编码器连续采集状态")

    async def position_tolerance(self, mdeg: int) -> None:
        """@brief 停机切换并保存到位判据，回读真实坐标有效性。@param mdeg 100或500毫度。"""
        self.require_control()
        await self.idle()
        if self.state.mode != "POSITION" or mdeg not in (100, 500):
            raise ControlError("位置容差只支持0.1°或0.5°")
        await self.read_position()
        if self.state.position is not None and self.state.position.tolerance_deg == mdeg / 1000:
            return
        await self.ok(f"MOTOR FOC POSITION TOLERANCE {mdeg}")
        await self.saved(changed=True)
        await self.read_position()
        if self.state.position is None or self.state.position.tolerance_deg != mdeg / 1000:
            raise ControlError("位置容差回读不一致")

    async def poll(self) -> None:
        """@brief 低频轮询，停机后才展开六步诊断。"""
        if not protocol_supported(self.state.protocol):
            return
        before = self.state.run
        await self.refresh_status()
        if not self.state.parameters and mode_supported(self.state.algorithm, self.state.mode):
            await self.read_parameters()
        if self.state.run == "RUNNING":
            if self.state.algorithm in FOC_ALGORITHMS:
                await self.read_drive()
                if self.state.read_only:
                    motor = fields(await self.ok("MOTOR"))
                    if "BUS_MV" in motor:
                        self.update(bus_v=int(motor["BUS_MV"]) / 1000)
            elif self.state.algorithm == "ENCODER_6STEP":
                await self.read_encoder()
            return
        if before == "RUNNING":
            self.update(closed=False, stage="停机")
            await asyncio.sleep(SAVE_WAIT)
            await self.read_diagnostics()
            if mode_supported(self.state.algorithm, self.state.mode):
                await self.read_parameters()
        else:
            saving = await self.optional("GET SAVE")
            self.update(
                save=fields(saving).get("SAVE", "") if saving else "不可用",
                source=await self.get("SOURCE"),
            )

    def _binary(self, block: WaveBlock) -> None:
        """@brief 按块存储；状态保持原生频率。@param block CRC已验证的块。"""
        if not self.wave_buffer.append(block):
            return
        if block.kind == 1:
            currents = block.currents()[-1]
            self._wave_iq, self._wave_id = float(currents[4]) / 8192, float(currents[3]) / 8192
        elif block.kind == 2 and not block.flags & TEST:
            points = self.wave_buffer.statuses()
            point = points[-1] if points else None
            if point is not None:
                self.update(
                    estimated_rpm=point.rpm if block.flags & VALID else self.state.estimated_rpm,
                    bus_v=point.bus,
                    iq_a=self._wave_iq,
                    id_a=self._wave_id,
                    data_time=timestamp(),
                )

    async def change_baud(self, baud: int) -> None:
        """@brief 仅停机与OFF时协商切速，失败不继续START。@param baud 档位。"""
        if self.state.read_only or self.state.run != "STOPPED":
            raise ControlError("切速要求可写连接且电机停机")
        setter = getattr(self.session.transport, "set_baud", None)
        if setter is None:
            raise ControlError("当前串口传输不支持运行时切速")
        response = await self.ok(f"SET UART_BAUD {baud}")
        if response != f"OK SET UART_BAUD={baud}":
            raise ControlError("切速回执不完整")
        try:
            setter(baud)
            await asyncio.sleep(0.15)
            response = await self.session.ask("PING", timeout=1.5)
            if response != "OK PING":
                raise ControlError("新速率PING未确认")
            self.update(link_baud=baud)
        except (OSError, ControlError) as error:
            self.session._fail("切速未确认，设备将在2秒内回退921600；请重新连接")
            raise ControlError(self.session.link_error) from error

    async def configure_wave(self) -> None:
        """@brief START前配置监视或波形档，日常监视保持OFF/921600。"""
        if not self.state.wave_available or self.state.run != "STOPPED":
            return
        await self.ok("SET FOC_PRINT OFF")
        desired = self.preferred_wave_baud
        if desired == 0:
            if self.state.link_baud != 921600:
                await self.change_baud(921600)
            self.update(wave_rate=0)
            return
        await self.ok("SET WAVE_RATE 5000")
        if self.state.link_baud != desired:
            await self.change_baud(desired)
        rate = 10000 if desired >= 2000000 else 5000
        await self.ok(f"SET WAVE_RATE {rate}")
        self.update(wave_rate=rate)

    async def enable_trace(self) -> None:
        """@brief 默认日常监视不上传波形；显式高速档沿用BIN1/旧TRACE。"""
        if self.preferred_wave_baud == 0:
            if self.state.wave_available and self.state.run == "STOPPED":
                await self.configure_wave()
            elif self.state.trace_available:
                await self.ok("SET FOC_PRINT OFF")
            self.update(wave_rate=0)
            return
        if self.state.wave_available:
            await self.configure_wave()
            await self.ok("SET FOC_PRINT BINARY")
            return
        if self.state.trace_available:
            response = await self.session.ask("SET FOC_PRINT TRACE")
            if response != "OK SET FOC_PRINT=TRACE":
                self.update(trace_available=False)
                self.notice("TRACE 实时打印不可用，保留低频状态监视")

    async def read_encoder(self) -> None:
        """@brief 查询本次上电资格及有感六步实时速度。"""
        response = await self.optional("MOTOR ENCODER")
        if response is None:
            self.update(encoder_available=False, encoder_calibration="")
            return
        data = fields(response)
        changes: SnapshotChanges = {"encoder_calibration": data.get("CALIBRATION", "")}
        if self.state.algorithm == "ENCODER_6STEP":
            active = self.state.run == "RUNNING" and data.get("STATE") == "RUNNING"
            changes.update({"closed": active, "stage": "闭环" if active else "停机"})
            if active and data.get("RPM_VALID") == "1":
                changes.update({"estimated_rpm": float(data["RPM"]), "data_time": timestamp()})
        changes["diagnostics"] = {**self.state.diagnostics, "MOTOR ENCODER": response}
        self.update(**changes)

    async def calibrate_encoder(self, changes: dict[str, ParameterValue] | None = None) -> None:
        """@brief 备份并应用草稿后标定，恢复原算法和模式，不自动启动。
        @param changes 原模式下已验证的参数草稿。
        """
        self.require_control()
        if not self.state.encoder_available:
            raise ControlError("板卡没有编码器标定能力")
        draft = changes or {}
        self._validate_changes(draft)
        previous_busy = self.state.busy
        try:
            self.update(busy="备份标定前配置")
            await self._initialization_backup("before_cal_")
            self.update(busy="保存标定前参数")
            await self.apply(draft)
            await self._calibrate_encoder()
        finally:
            self.update(busy=previous_busy)

    async def _calibrate_encoder(self) -> None:
        """@brief 执行已备份事务内的有界七点标定，原模式在一次保存中恢复。"""
        await self.idle()
        original = self.state.algorithm
        original_mode = self.state.mode
        if original != "ENCODER_6STEP":
            await self.select_algorithm("ENCODER_6STEP")
        self.update(busy="编码器七点标定中，可点击顶部停止")
        response = await self.ok("MOTOR ENCODER CAL")
        if response != "OK ENCODER CAL RUNNING":
            raise ControlError("编码器标定未确认：" + response)
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            await asyncio.sleep(0.1)
            await self.refresh_status()
            await self.read_encoder()
            point = fields(self.state.diagnostics.get("MOTOR ENCODER", "")).get("CAL_POINT", "")
            if point:
                self.update(busy="编码器七点标定 " + point + "，可点击顶部停止")
            if self.state.run != "RUNNING":
                break
        else:
            await self.stop_motor()
            raise ControlError("编码器标定超时，已执行停止")
        await self.read_diagnostics()
        if self.state.encoder_calibration != "VALID" or self.state.fault != "NONE":
            raise ControlError(
                "编码器标定失败：" + (self.state.fault or self.state.encoder_calibration)
            )
        await self.saved(changed=True)
        if original != "ENCODER_6STEP":
            self.update(busy="恢复原控制模式")
            await self.select_algorithm(original, original_mode)
        else:
            await self.read_parameters()

    def _telemetry(self, line: str) -> None:
        """@brief 消费有效遥测，错误只留原文不补造点。@param line 实时行。"""
        try:
            poles = self.state.parameters.get("POLE_PAIRS", ParameterValue(3, 0, "")).value
            frame = parse_telemetry(line, poles)
            if frame is None:
                return
            target = self.state.parameters.get("TARGET")
            point = self.buffer.append(
                frame,
                time.monotonic(),
                timestamp(),
                float(target.value) if target and self.state.mode == "SPEED" else None,
            )
            if point is None:
                return
            self.on_point(point)
            if not self.stop_requested and self.state.run == "RUNNING":
                changes: SnapshotChanges = {"data_time": point.received_at}
                if frame.rpm is not None:
                    changes["estimated_rpm"] = frame.rpm
                if frame.iq_ma is not None:
                    changes["iq_a"] = frame.iq_ma / 1000
                if frame.id_ma is not None:
                    changes["id_a"] = frame.id_ma / 1000
                if frame.bus_mv is not None:
                    changes["bus_v"] = frame.bus_mv / 1000
                if frame.stage is not None:
                    changes.update({"stage": STAGES[frame.stage], "closed": frame.stage == 6})
                # 高频样本只更新后台快照，Qt 桥接层按 20Hz 合并发布。
                self.state = replace(self.state, **changes)
        except (ControlError, ValueError, OverflowError) as error:
            self.on_log("ERROR", f"遥测解析失败：{error}；原文：{line}")

    async def capture(self, name: str) -> Preset:
        """@brief 双读备份当前参数，不切换算法。@param name 名称。@return 完整预设。"""
        await self.idle()
        await self.saved()
        first = await self._capture_once(name)
        second = await self._capture_once(name)
        await self.idle()
        if first.difference(second):
            raise ControlError("采集期间配置发生变化，请保持停机后重新备份")
        return first

    async def _capture_once(self, name: str) -> Preset:
        """@brief 一次读取可见配置。@param name 名称。@return 候选预设。"""
        identity = fields(await self.ok("VERSION"))
        values = {
            key: await self.get(key)
            for key in ("ALGORITHM", "MODE", "SOURCE", "BRIGHTNESS", "ANIMATION")
        }
        algorithm = values["ALGORITHM"]
        if (algorithm, values["MODE"]) != (self.state.algorithm, self.state.mode):
            self.update(
                algorithm=algorithm,
                mode=values["MODE"],
                parameters={},
                parameter_specs={},
                control_available=False,
            )
        # 只备份已经探测到的字段，旧版缺少可选项不会使整个备份失败。
        await self.read_parameters()
        parameters = dict(self.state.parameters)
        writable = writable_preset_keys(algorithm, parameters, values["MODE"])
        known = specs_for(algorithm, parameters, values["MODE"])
        writable -= {
            key
            for key, definition in self.state.parameter_specs.items()
            if known[key].editable and not definition.editable
        }
        preset = Preset(
            "foc_motor_preset",
            1,
            name,
            timestamp(),
            identity.get("FW", ""),
            identity.get("PROTOCOL", ""),
            algorithm,
            values["MODE"],
            values["SOURCE"],
            int(values["BRIGHTNESS"]),
            int(values["ANIMATION"]),
            {key: value for key, value in parameters.items() if key in writable},
            {key: value for key, value in parameters.items() if key not in writable},
        )
        preset.validate()
        return preset

    async def restore(self, target: Preset) -> None:
        """@brief 备份原选择与目标块后逐项恢复；失败保持审计和已写状态。@param target 预设。"""
        target.validate()
        self.require_control()
        version = fields(await self.ok("VERSION"))
        if not protocol_supported(version.get("PROTOCOL", "")):
            raise ControlError("预设与板上协议不兼容")
        if target.algorithm not in self.state.available_algorithms:
            raise ControlError("板卡没有预设所需算法")
        directory = self.data_dir / "restore_backups"
        stamp = timestamp().replace(":", "").replace("-", "").replace(".", "").replace("+", "_")
        before = await self.capture("before_" + stamp)
        before.save(directory / f"{before.name}.json")
        self.on_log("AUDIT", "修改前备份：" + str(directory / f"{before.name}.json"))
        target_values = {**target.readonly, **target.parameters}
        before_values = {**before.readonly, **before.parameters}
        for key in ("POLE_PAIRS", "SHUNT", "RATED_VOLTAGE", "RATED_CURRENT", "RATED_SPEED"):
            if key not in target_values or target_values[key] != before_values.get(key):
                raise ControlError(f"电机只读模型 {key} 不一致，不能跨电机恢复")
        selected = before
        if before.algorithm != target.algorithm:
            await self.select_algorithm(target.algorithm)
        if self.state.mode != target.mode:
            await self.select_mode(target.mode)
        if (before.algorithm, before.mode) != (target.algorithm, target.mode):
            selected = await self.capture("selected_" + stamp)
            selected.save(directory / f"{selected.name}.json")
            self.on_log("AUDIT", "目标参数块备份：" + str(directory / f"{selected.name}.json"))
        selected_values = {**selected.readonly, **selected.parameters}
        for key in MODEL_PARAMETERS & set(specs_for(target.algorithm, mode=target.mode)):
            if key == "SPEED_LIMIT" and target.mode == "POSITION":
                # 位置模式的最大运动速度是可编辑轨迹参数，不是电机身份。
                continue
            if key not in target_values or target_values[key] != selected_values.get(key):
                raise ControlError(f"只读电机模型 {key} 不一致")
        # 所有写入先按目标板能力验证，不能等写到中途才发现范围或字段变化。
        for key, value in target.parameters.items():
            definition = self.state.parameter_specs.get(key)
            if definition is None:
                raise ControlError("板卡缺少预设参数：" + key)
            if value != selected_values.get(key):
                if not definition.editable:
                    raise ControlError("预设试图修改只读参数：" + key)
                definition.validate(value)
        if target.firmware != version.get("FW"):
            self.on_log(
                "AUDIT",
                f"跨固件恢复：{target.firmware} → {version.get('FW')}；已核对协议与电机模型",
            )
        changed = (before.algorithm, before.mode) != (target.algorithm, target.mode)
        for key, expected, actual in (
            ("MODE", target.mode, selected.mode),
            ("BRIGHTNESS", str(target.brightness), str(selected.brightness)),
            ("ANIMATION", str(target.animation), str(selected.animation)),
        ):
            if expected != actual:
                await self.set_value(key, expected)
                changed = True
        if target.mode == "HAPTIC":
            haptic_changes = {
                key: value
                for key, value in target.parameters.items()
                if key in HAPTIC_KEYS and value != selected_values.get(key)
            }
            if haptic_changes:
                await self.apply(haptic_changes)
                changed = True
        for key, value in target.parameters.items():
            if target.mode == "HAPTIC" and key in HAPTIC_KEYS:
                continue
            if value != selected_values.get(key):
                await self.set_value(key, str(value.value))
                if await self.parameter(key) != value:
                    raise ControlError(f"{key} 的完整回读不一致")
                changed = True
        if selected.source != target.source:
            await self.set_value("SOURCE", target.source)
            changed = True
        await self.saved(changed)
        verified = await self.capture("verify_" + stamp)
        if (
            verified.algorithm,
            verified.mode,
            verified.source,
            verified.brightness,
            verified.animation,
        ) != (
            target.algorithm,
            target.mode,
            target.source,
            target.brightness,
            target.animation,
        ) or any(
            value != {**verified.readonly, **verified.parameters}.get(key)
            for key, value in target.parameters.items()
        ):
            raise ControlError("恢复后的完整配置不一致")
        await self.read_parameters()
        self.update(source=target.source, error="")

    async def read_haptic(self) -> None:
        """@brief 读取板卡原始机械状态及当前生效请求，原文已由session完整记录。"""
        if not self.state.haptic_available:
            return
        sample = HapticSample.parse(await self.ok("MOTOR FOC HAPTIC"))
        self.update(haptic=sample)

    async def _apply_haptic(self, changes: dict[str, ParameterValue]) -> None:
        """@brief 原子提交手感，限流时限停机修改；运行中不等待Flash。
        @param changes 当前已校验草稿。
        """
        running = self.state.run == "RUNNING"
        if running and set(changes) - set(HAPTIC_KEYS):
            raise ControlError("运行中只能调整手感类型、档数、力度和阻尼")
        if not running:
            await self.idle()
        await self.read_haptic()
        assert self.state.haptic is not None
        expected = {
            **self.state.haptic.parameters(),
            **{k: v for k, v in changes.items() if k in HAPTIC_KEYS},
        }
        configured = bool(set(changes) & set(HAPTIC_KEYS))
        if configured:
            sample = HapticSample.parse(await self.ok(config_command(expected)))
            if sample.parameters() != expected:
                raise ControlError("手感完整配置回读不一致")
            self.update(haptic=sample, parameters={**self.state.parameters, **sample.parameters()})
        for key, value in changes.items():
            if key in HAPTIC_KEYS:
                continue
            await self.set_value(key, str(value.value))
            actual = await self.parameter(key)
            if actual != value:
                raise ControlError(f"{key} 的完整回读不一致")
            self.update(parameters={**self.state.parameters, key: actual})
        if changes:
            if running:
                self.update(save="PENDING")
            else:
                await self.saved(changed=True)
        await self.read_haptic()

    async def enable_haptic(self, changes: dict[str, ParameterValue] | None = None) -> None:
        """@brief 一次备份、按需七点、保存和开启手感；STOP可取消整项事务。
        @param changes 已编码草稿，不保存起点、运行态或累计计数。
        """
        self.require_control()
        await self.idle()
        if self.state.mode != "HAPTIC" or not self.state.haptic_available:
            raise ControlError("请先选择虚拟编码器模式")
        if self.state.source != "REMOTE" or self.state.still is not True:
            raise ControlError("开启手感需要上位机控制且轴静止")
        draft = changes or {}
        self._validate_changes(draft)
        previous_busy = self.state.busy
        try:
            self.update(busy="1/4 备份当前配置")
            await self._initialization_backup("before_haptic_enable_")
            await self.read_encoder()
            if self.state.encoder_calibration != "VALID":
                self.update(busy="2/4 编码器七点标定")
                await self._calibrate_encoder()
            self.update(busy="3/4 保存手感配置")
            await self.apply(draft)
            await self.saved()
            self.update(busy="4/4 开启虚拟编码器，可点击顶部停止")
            await self.start_motor({})
            await self.read_haptic()
        finally:
            self.update(busy=previous_busy)
