"""偶发只读丢包可恢复；持续失联尽力STOP，写命令绝不自动重放。"""

from __future__ import annotations

import asyncio

import pytest
from foc_control.controller import Session
from foc_control.models import ControlError


class BrokenPort:
    """模拟一条损坏回执或完全无回执的连接，保留线上所有写入。"""

    def __init__(self, silent: bool = False) -> None:
        """@brief 设置响应行为。@param silent 是否模拟超时。"""
        self.silent = silent
        self.received = b""
        self.writes: list[bytes] = []
        self.closed = False

    def read(self) -> bytes:
        """@brief 消费待接收字节。@return 当前字节。"""
        data, self.received = self.received, b""
        return data

    def write(self, data: bytes) -> None:
        """@brief 记录线上指令。@param data 原始字节。"""
        self.writes.append(data)
        if not self.silent:
            self.received = b"OK Fj\r\n"

    def close(self) -> None:
        """@brief 仅标记句柄释放，不模拟电机停止。"""
        self.closed = True


@pytest.mark.parametrize("silent", (False, True))
def test_link_failure_sends_one_unconfirmed_stop(silent: bool) -> None:
    """@brief 坏回执/超时关闭前只发一次STOP，不重放原请求。@param silent 超时路径。"""

    async def scenario() -> None:
        port = BrokenPort(silent)
        log: list[tuple[str, str]] = []
        client = Session(port, False, lambda k, t: log.append((k, t)), lambda _: None)
        await client.start()
        with pytest.raises(ControlError):
            await client.ask("SET TARGET 10", timeout=0.02)
        assert port.writes == [b"SET TARGET 10\r\n", b"STOP\r\n"]
        assert client.closed and port.closed
        original = client.link_error
        client._fail("第二个错误不能覆盖首因")
        assert client.link_error == original and len(port.writes) == 2
        assert any("停机未确认" in text for kind, text in log if kind == "AUDIT")
        await client.close()

    asyncio.run(scenario())


def test_readonly_link_failure_does_not_send_stop() -> None:
    """@brief 只读连接仍只发送读命令，即使回执损坏。"""

    async def scenario() -> None:
        port = BrokenPort()
        client = Session(port, True, lambda *_: None, lambda _: None)
        await client.start()
        with pytest.raises(ControlError):
            await client.ask("STATUS", timeout=0.02)
        assert port.writes == [b"STATUS\r\n"] * 3 and port.closed
        await client.close()

    asyncio.run(scenario())


class LossyPort(BrokenPort):
    """按次返回真实坏回执、丢包或正常响应，不生成板卡计算结果。"""

    def __init__(self, replies: list[bytes]) -> None:
        """@brief 保存每次查询的原始返回。@param replies 顺序字节，可空表示丢失。"""
        super().__init__()
        self.replies = list(replies)

    def write(self, data: bytes) -> None:
        """@brief 记录查询并提供下一批数据。@param data 线上命令。"""
        self.writes.append(data)
        self.received += self.replies.pop(0) if self.replies else b""


@pytest.mark.parametrize(
    "damaged",
    (b"OK Fj\r\n", b"\xff\r\n", b"}RR LINE C\x7fRRUPT\r\n"),
)
def test_damaged_read_then_good_reply_keeps_connection(damaged: bytes) -> None:
    """@brief 今天实际出现的坏行不应立刻停机，后续正确回执可恢复。@param damaged 坏行。"""

    async def scenario() -> None:
        port = LossyPort([damaged + b"OK FOC HAPTIC KIND=DETENT\r\n"])
        log: list[tuple[str, str]] = []
        client = Session(port, False, lambda k, t: log.append((k, t)), lambda _: None)
        await client.start()
        try:
            assert await client.ask("MOTOR FOC HAPTIC", timeout=0.1) == "OK FOC HAPTIC KIND=DETENT"
            assert port.writes == [b"MOTOR FOC HAPTIC\r\n"]
            assert not port.closed and not client.link_error and log
        finally:
            await client.close()

    asyncio.run(scenario())


@pytest.mark.parametrize("lost", (b"", b"ERR LINE CORRUPT\r\n", b"OK Fj\r\n"))
def test_only_read_request_retries_after_loss(lost: bytes) -> None:
    """@brief 丢失或损坏一次只重读原查询，不发送STOP/START/CLEAR。@param lost 原始丢包。"""

    async def scenario() -> None:
        port = LossyPort([lost, b"OK GET MODE=HAPTIC\r\n"])
        client = Session(port, False, lambda *_: None, lambda _: None)
        await client.start()
        try:
            assert await client.ask("GET MODE", timeout=0.1) == "OK GET MODE=HAPTIC"
            assert port.writes == [b"GET MODE\r\n"] * 2
            assert not client.closed and not client.pending and client.wire_idle.is_set()
        finally:
            await client.close()

    asyncio.run(scenario())


def test_continuous_loss_still_reports_unknown_and_stop() -> None:
    """@brief 连续三次查询失败才释放，不把持续失联当作正常运行。"""

    async def scenario() -> None:
        port = LossyPort([])
        client = Session(port, False, lambda *_: None, lambda _: None)
        await client.start()
        with pytest.raises(ControlError):
            await client.ask("STATUS", timeout=0.01)
        assert port.writes == [b"STATUS\r\n"] * 3 + [b"STOP\r\n"]
        assert client.closed and port.closed and "状态未知" in client.link_error
        await client.close()

    asyncio.run(scenario())


def test_write_corruption_never_replays_control_command() -> None:
    """@brief START回执损坏仍失败且只写一次，容错不扩展到控制写入。"""

    async def scenario() -> None:
        port = LossyPort([b"\xff\r\n"])
        client = Session(port, False, lambda *_: None, lambda _: None)
        await client.start()
        with pytest.raises(ControlError):
            await client.ask("START", timeout=0.01)
        assert port.writes == [b"START\r\n", b"STOP\r\n"] and client.closed
        await client.close()

    asyncio.run(scenario())


def test_stop_preempts_cancelled_lossy_query() -> None:
    """@brief 等待重读时可取消并立即STOP，不等待三轮超时，也不重新启动。"""

    async def scenario() -> None:
        port = LossyPort([b"", b"OK STOP PENDING\r\n"])
        client = Session(port, False, lambda *_: None, lambda _: None)
        await client.start()
        task = asyncio.create_task(client.ask("MOTOR FOC HAPTIC"))
        await asyncio.sleep(0.005)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        try:
            assert await client.ask("STOP", priority=True, timeout=0.2) == "OK STOP PENDING"
            assert port.writes == [b"MOTOR FOC HAPTIC\r\n", b"STOP\r\n"]
            assert not client.closed
        finally:
            await client.close()

    asyncio.run(scenario())


def test_corrupt_wave_during_write_does_not_cancel_valid_ack() -> None:
    """@brief 二进制坏包只记缺口；同批写命令的正确回执仍须确认。"""

    async def scenario() -> None:
        port = LossyPort([b"\x00\x02X\x00OK SET TARGET=10\r\n"])
        client = Session(port, False, lambda *_: None, lambda _: None)
        corrupt: list[bool] = []
        client.on_bad_wave = lambda: corrupt.append(True)
        await client.start()
        try:
            assert await client.ask("SET TARGET 10", timeout=0.1) == "OK SET TARGET=10"
            assert port.writes == [b"SET TARGET 10\r\n"]
            assert corrupt == [True] and client.bad_wave_count == 1 and not client.closed
        finally:
            await client.close()

    asyncio.run(scenario())
