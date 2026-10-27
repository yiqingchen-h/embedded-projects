"""Windows API 替身验证配置、错误检测和资源释放，不打开真实 COM。"""

from __future__ import annotations

import ctypes as ct
import sys
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from ctypes import wintypes as wt
from typing import Any

import pytest
from foc_control.transport import DCB, CommStat, NativePort, Timeouts, available_ports

pytestmark = pytest.mark.skipif(sys.platform != "win32", reason="Windows 原生 API 布局")


class ApiCall:
    """允许配置 ctypes 签名的函数替身。"""

    def __init__(self, function: Callable[..., int]) -> None:
        """@brief 保存调用实现。@param function 模拟函数。"""
        self.function = function
        self.argtypes: object = None
        self.restype: object = None

    def __call__(self, *args: Any) -> int:
        """@brief 执行替身。@param args API 参数。@return API 返回。"""
        return self.function(*args)


class Kernel:
    """仅操作传入的内存结构，不触碰内核设备。"""

    def __init__(self) -> None:
        """@brief 建立配置和错误注入接口。"""
        self.calls: list[tuple[str, tuple[Any, ...]]] = []
        self.fail = ""
        self.error_flags = 0
        self.partial_write = False
        self.already_configured = False
        for name in (
            "CreateFileW",
            "CloseHandle",
            "GetCommState",
            "SetCommState",
            "SetCommTimeouts",
            "SetupComm",
            "ClearCommError",
            "ReadFile",
            "WriteFile",
        ):
            setattr(self, name, ApiCall(lambda *args, key=name: self.invoke(key, args)))

    def invoke(self, name: str, args: tuple[Any, ...]) -> int:
        """@brief 模拟签名相同的调用。@param name API 名。@param args 参数。@return 结果。"""
        self.calls.append((name, args))
        if name == "CreateFileW":
            return int(wt.HANDLE(-1).value or 0) if self.fail == name else 99
        if self.fail == name:
            return 0
        if name == "GetCommState":
            state = ct.cast(args[1], ct.POINTER(DCB)).contents
            state.baud = 921600 if self.already_configured else 115200
            state.byte_size, state.parity, state.stop_bits, state.flags = 8, 0, 0, 1
        elif name == "SetCommState":
            state = ct.cast(args[1], ct.POINTER(DCB)).contents
            assert (state.baud, state.byte_size, state.parity, state.stop_bits, state.flags) == (
                921600,
                8,
                0,
                0,
                1,
            )
        elif name == "SetupComm":
            assert args[1:] == (65536, 4096)
        elif name == "SetCommTimeouts":
            values = ct.cast(args[1], ct.POINTER(Timeouts)).contents
            assert values.read_constant == 5 and values.write_constant == 1000
        elif name == "ClearCommError":
            ct.cast(args[1], ct.POINTER(wt.DWORD)).contents.value = self.error_flags
            ct.cast(args[2], ct.POINTER(CommStat)).contents.in_queue = 9
        elif name == "ReadFile":
            ct.memmove(args[1], b"OK PING\r\n", 9)
            ct.cast(args[3], ct.POINTER(wt.DWORD)).contents.value = 9
        elif name == "WriteFile":
            ct.cast(args[3], ct.POINTER(wt.DWORD)).contents.value = args[2] - int(
                self.partial_write
            )
        return 1


def test_native_configuration_and_short_io(monkeypatch: pytest.MonkeyPatch) -> None:
    """@brief 字段布局、8N1、禁用 DTR/RTS 和64KiB接收请求。"""
    api = Kernel()
    monkeypatch.setattr(ct, "WinDLL", lambda *_args, **_kwargs: api)
    assert ct.sizeof(DCB) == 28 and ct.sizeof(Timeouts) == 20 and ct.sizeof(CommStat) == 12
    port = NativePort("COM10")
    assert port.read() == b"OK PING\r\n"
    assert port.rx_queue_peak == 9
    port.write(b"PING\r\n")
    port.close()
    port.close()
    with pytest.raises(OSError):
        port.read()
    with pytest.raises(OSError):
        port.write(b"PING\r\n")
    assert len([name for name, _args in api.calls if name == "CloseHandle"]) == 1


def test_native_configuration_preserved(monkeypatch: pytest.MonkeyPatch) -> None:
    """@brief 当前已经正确配置时不重复切换串口驱动状态。"""
    api = Kernel()
    api.already_configured = True
    monkeypatch.setattr(ct, "WinDLL", lambda *_args, **_kwargs: api)
    port = NativePort("COM10")
    assert not any(name == "SetCommState" for name, _args in api.calls)
    port.close()


@pytest.mark.parametrize("failure", ("CreateFileW", "SetupComm", "SetCommState", "SetCommTimeouts"))
def test_native_setup_failure_closes_handle(monkeypatch: pytest.MonkeyPatch, failure: str) -> None:
    """@brief 初始化失败释放已打开句柄。"""
    api = Kernel()
    api.fail = failure
    monkeypatch.setattr(ct, "WinDLL", lambda *_args, **_kwargs: api)
    with pytest.raises(OSError):
        NativePort("COM10")
    assert any(name == "CloseHandle" for name, _args in api.calls) == (failure != "CreateFileW")


def test_native_driver_error_and_partial_write(monkeypatch: pytest.MonkeyPatch) -> None:
    """@brief 驱动错误与部分写入不得当作有效响应。"""
    api = Kernel()
    monkeypatch.setattr(ct, "WinDLL", lambda *_args, **_kwargs: api)
    port = NativePort("COM10")
    api.error_flags = 0x400
    with pytest.raises(OSError, match="flags"):
        port.read()
    api.partial_write = True
    with pytest.raises(OSError, match="不完整"):
        port.write(b"PING\r\n")
    port.close()
    with pytest.raises(ValueError):
        NativePort("COMabc")


@pytest.mark.parametrize("flags", (1, 2, 4, 8, 15))
def test_transient_receive_errors_keep_reading(monkeypatch: pytest.MonkeyPatch, flags: int) -> None:
    """@brief 接收溢出/校验/帧错误保留计数但不关闭句柄。@param flags 瞬态接收错误位。"""
    api = Kernel()
    api.error_flags = flags
    monkeypatch.setattr(ct, "WinDLL", lambda *_args, **_kwargs: api)
    port = NativePort("COM10")
    assert port.read() == b"OK PING\r\n"
    assert port.rx_error_flags == flags and port.rx_error_count == 1
    api.error_flags = 0
    assert port.read() == b"OK PING\r\n"
    assert port.rx_error_flags == 0 and port.rx_error_count == 1
    port.close()


def test_port_enumeration_is_read_only_and_sorted(monkeypatch: pytest.MonkeyPatch) -> None:
    """@brief 仅枚举注册表映射，过滤无效值并自然排序，不打开设备。"""
    import winreg

    values: list[object] = ["COM20", "COM3", "COM10", "COM2", "COM3", 42, "LPT1", "COMx"]
    closed: list[bool] = []

    @contextmanager
    def registry_key(*_args: object) -> Iterator[object]:
        """@brief 记录注册表上下文是否释放。"""
        try:
            yield object()
        finally:
            closed.append(True)

    monkeypatch.setattr(winreg, "OpenKey", registry_key)
    monkeypatch.setattr(winreg, "QueryInfoKey", lambda _key: (0, len(values), 0))
    monkeypatch.setattr(winreg, "EnumValue", lambda _key, index: (str(index), values[index], 1))
    assert available_ports() == ["COM2", "COM3", "COM10", "COM20"]
    assert closed == [True]


def test_missing_registry_and_unsupported_system(monkeypatch: pytest.MonkeyPatch) -> None:
    """@brief 缺少端口映射或运行于其它系统时安全返回，不触碰真实串口。"""
    import winreg

    def missing_key(*_args: object) -> None:
        """@brief 模拟系统尚未创建串口映射。"""
        raise FileNotFoundError

    monkeypatch.setattr(winreg, "OpenKey", missing_key)
    assert available_ports() == []
    monkeypatch.setattr(sys, "platform", "other")
    assert available_ports() == []
    with pytest.raises(OSError, match="Windows"):
        NativePort("COM10")
