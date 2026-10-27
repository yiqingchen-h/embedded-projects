"""协议1.3控制器的协商、旧版回退、只读与异常重连，不访问COM。"""

from __future__ import annotations

import asyncio
from pathlib import Path

import pytest
from foc_control.controller import Controller
from foc_control.models import ControlError
from foc_control.simulator import DemoTransport


class WavePort(DemoTransport):
    """只模拟新文本控制接口，高速数据本身由协议和GUI回归分别验证。"""

    def __init__(self) -> None:
        """@brief 采用当前七极对模型建立协议1.3替身。"""
        super().__init__(firmware="1.7.0-g2208-wave", algorithm="ENCODER_FOC", g2208=True)
        self.baud = 921600
        self.wave_rate = 5000
        self.pending_baud = 921600
        self.fail_switch = False

    def answer(self, text: str) -> None:
        """@brief 排入一条真实格式回执。@param text 回执。"""
        self.incoming.extend((text + "\r\n").encode("ascii"))

    def write(self, data: bytes) -> None:
        """@brief 只拦截新增命令，旧控制行为沿用原替身。@param data 命令字节。"""
        command = data.decode("ascii").strip()
        if command == "VERSION":
            self.commands.append(command)
            self.answer("OK VERSION FW=1.7.0-g2208-wave PROTOCOL=1.3 ENCODER=MT6835")
        elif command == "GET WAVE_CAPS":
            self.commands.append(command)
            self.answer(
                "OK GET WAVE_CAPS=BIN1 RATES=5000,10000 BAUDS=921600,1000000,2000000,3000000"
            )
        elif command.startswith("SET UART_BAUD "):
            self.commands.append(command)
            self.pending_baud = int(command.split()[-1])
            self.answer(f"OK SET UART_BAUD={self.pending_baud}")
        elif command.startswith("SET WAVE_RATE "):
            self.commands.append(command)
            self.wave_rate = int(command.split()[-1])
            self.answer(f"OK SET WAVE_RATE={self.wave_rate}")
        elif command == "SET FOC_PRINT BINARY":
            self.commands.append(command)
            self.print_mode = "BINARY"
            self.answer("OK SET FOC_PRINT=BINARY")
        else:
            super().write(data)

    def set_baud(self, baud: int) -> None:
        """@brief 模拟驱动接受或拒绝速率。@param baud 请求值。"""
        if self.fail_switch:
            raise OSError("模拟驱动拒绝高速档")
        assert baud == self.pending_baud
        self.baud = baud


def test_negotiation_and_compatibility_rate(tmp_path: Path) -> None:
    """@brief 高速档先OFF和5kHz，再切速确认，最后启用10kHz。"""

    async def scenario() -> None:
        port = WavePort()
        client = Controller(port, tmp_path)
        client.preferred_wave_baud = 2000000
        try:
            await client.initialize()
            assert client.state.wave_available
            await client.enable_trace()
            assert port.baud == 2000000 and port.wave_rate == 10000
            assert port.print_mode == "BINARY"
            sequence = port.commands[-6:]
            assert sequence == [
                "SET FOC_PRINT OFF",
                "SET WAVE_RATE 5000",
                "SET UART_BAUD 2000000",
                "PING",
                "SET WAVE_RATE 10000",
                "SET FOC_PRINT BINARY",
            ]
            client.preferred_wave_baud = 921600
            await client.enable_trace()
            assert port.baud == 921600 and port.wave_rate == 5000
            assert "START" not in port.commands
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_readonly_does_not_negotiate_or_enable(tmp_path: Path) -> None:
    """@brief 只读识别新能力，但不得切速或启用波形。"""

    async def scenario() -> None:
        port = WavePort()
        client = Controller(port, tmp_path, read_only=True)
        try:
            await client.initialize()
            assert client.state.wave_available
            with pytest.raises(ControlError):
                await client.enable_trace()
            assert not any(command.startswith("SET ") for command in port.commands)
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_driver_rejects_baud_prevents_start(tmp_path: Path) -> None:
    """@brief 驱动切速失败即链路未知，不发START或重放写入。"""

    async def scenario() -> None:
        port = WavePort()
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            port.fail_switch = True
            client.preferred_wave_baud = 2000000
            with pytest.raises(ControlError, match="切速未确认"):
                await client.enable_trace()
            assert client.session.closed
            assert "START" not in port.commands
            assert "SET FOC_PRINT BINARY" not in port.commands
            assert port.commands.count("SET UART_BAUD 2000000") == 1
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_legacy_firmware_still_uses_trace(tmp_path: Path) -> None:
    """@brief 老固件不接收新增写命令。"""

    async def scenario() -> None:
        port = DemoTransport()
        client = Controller(port, tmp_path)
        client.preferred_wave_baud = 921600
        try:
            await client.initialize()
            await client.enable_trace()
            assert "SET FOC_PRINT TRACE" in port.commands
            assert not any(
                "UART_BAUD" in command or "WAVE_RATE" in command for command in port.commands
            )
        finally:
            await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize("read_only", (False, True))
def test_identity_reopens_only_known_alternate(tmp_path: Path, read_only: bool) -> None:
    """@brief 身份阶段只重试VERSION，普通会话也不向错误波特率发STOP。@param read_only 只读。"""

    class ReconnectPort(WavePort):
        """首档收到损坏字节，重开后恢复的已知速率替身。"""

        def __init__(self) -> None:
            """@brief 从失配高速档开始。"""
            super().__init__()
            self.baud = 2000000
            self.opened: list[int] = []

        def write(self, data: bytes) -> None:
            """@brief 旧档返回不可解析原文。@param data 命令。"""
            if self.baud == 2000000:
                self.commands.append(data.decode("ascii").strip())
                self.incoming.extend(b"\xff\r\n")
            else:
                super().write(data)

        def reopen(self, baud: int) -> None:
            """@brief 只重开已知备用档。@param baud 备用速率。"""
            self.baud = baud
            self.closed = False
            self.opened.append(baud)
            self.incoming.clear()

    async def scenario() -> None:
        port = ReconnectPort()
        client = Controller(port, tmp_path, read_only=read_only)
        try:
            await client.initialize()
            assert port.opened == [921600]
            assert port.commands[:4] == ["VERSION"] * 4
            assert client.state.connected and client.state.link_baud == 921600
            assert "STOP" not in port.commands
            if read_only:
                assert not any(command.startswith("SET ") for command in port.commands)
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_default_monitor_keeps_wave_off_and_original_control(tmp_path: Path) -> None:
    """@brief 默认只查日志和真实Id/Iq，关闭上传不更改控制/限流/目标。"""

    async def scenario() -> None:
        port = WavePort()
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            original = dict(port.parameters)
            count = len(port.commands)
            await client.enable_trace()
            assert port.commands[count:] == ["SET FOC_PRINT OFF"]
            assert port.print_mode == "OFF" and port.baud == 921600
            assert client.state.wave_rate == 0 and port.parameters == original
            assert "START" not in port.commands
        finally:
            await client.session.close()

    asyncio.run(scenario())
