"""Windows 原生串口；后台独占句柄，不使用 DTR/RTS 复位板卡。"""

from __future__ import annotations

import ctypes as ct
import sys
from ctypes import wintypes as wt
from typing import Protocol, cast


class Transport(Protocol):
    """通信和测试替身的短调用接口，所有调用位于串口线程。"""

    def read(self) -> bytes:
        """@brief 读取当前字节，空闲最多等待五毫秒。@return 新字节。"""
        ...

    def write(self, data: bytes) -> None:
        """@brief 发送完整字节。@param data 命令字节。"""
        ...

    def close(self) -> None:
        """@brief 释放通信资源。"""
        ...


class DCB(ct.Structure):
    """Windows DCB，保留 DWORD 位字段布局。"""

    _fields_ = [
        ("length", wt.DWORD),
        ("baud", wt.DWORD),
        ("flags", wt.DWORD),
        ("reserved", wt.WORD),
        ("xon_limit", wt.WORD),
        ("xoff_limit", wt.WORD),
        ("byte_size", wt.BYTE),
        ("parity", wt.BYTE),
        ("stop_bits", wt.BYTE),
        ("xon", ct.c_char),
        ("xoff", ct.c_char),
        ("error", ct.c_char),
        ("eof", ct.c_char),
        ("event", ct.c_char),
        ("reserved1", wt.WORD),
    ]


class Timeouts(ct.Structure):
    """五毫秒空闲读取、最多一秒完整写入。"""

    _fields_ = [
        (name, wt.DWORD)
        for name in (
            "read_interval",
            "read_multiplier",
            "read_constant",
            "write_multiplier",
            "write_constant",
        )
    ]


class CommStat(ct.Structure):
    """只保留 PC 串口队列状态，不增加板卡缓存。"""

    _fields_ = [("flags", wt.DWORD), ("in_queue", wt.DWORD), ("out_queue", wt.DWORD)]


class NativePort:
    """沿用已有 motor_serial.py 的配置和错误检测，分离收发等待。"""

    def __init__(self, port: str, baud: int = 921600) -> None:
        """@brief 打开指定端口。@param port COM 名称。@param baud 波特率。"""
        if sys.platform != "win32":
            raise OSError("此发布版本使用 Windows 原生串口")
        if not port.upper().startswith("COM") or not port[3:].isdigit():
            raise ValueError("请选择明确的 COM 端口")
        self.port = port
        self.baud = baud
        self.api = ct.WinDLL("kernel32", use_last_error=True)
        self.handle: int | None = None
        self.rx_queue_peak = 0
        # 仅本次读取的接收错误位及累计次数；上层记录缺口，不因此关闭控制连接。
        self.rx_error_flags = 0
        self.rx_error_count = 0
        signatures = {
            "CreateFileW": (
                [wt.LPCWSTR, wt.DWORD, wt.DWORD, ct.c_void_p, wt.DWORD, wt.DWORD, wt.HANDLE],
                wt.HANDLE,
            ),
            "CloseHandle": ([wt.HANDLE], wt.BOOL),
            "GetCommState": ([wt.HANDLE, ct.POINTER(DCB)], wt.BOOL),
            "SetCommState": ([wt.HANDLE, ct.POINTER(DCB)], wt.BOOL),
            "SetCommTimeouts": ([wt.HANDLE, ct.POINTER(Timeouts)], wt.BOOL),
            "SetupComm": ([wt.HANDLE, wt.DWORD, wt.DWORD], wt.BOOL),
            "ClearCommError": ([wt.HANDLE, ct.POINTER(wt.DWORD), ct.POINTER(CommStat)], wt.BOOL),
            "ReadFile": (
                [wt.HANDLE, ct.c_void_p, wt.DWORD, ct.POINTER(wt.DWORD), ct.c_void_p],
                wt.BOOL,
            ),
            "WriteFile": (
                [wt.HANDLE, ct.c_void_p, wt.DWORD, ct.POINTER(wt.DWORD), ct.c_void_p],
                wt.BOOL,
            ),
        }
        for name, (arguments, result) in signatures.items():
            function = getattr(self.api, name)
            function.argtypes = arguments
            function.restype = result
        handle = cast(
            int, self.api.CreateFileW("\\\\.\\" + port.upper(), 0xC0000000, 0, None, 3, 0, None)
        )
        if handle == wt.HANDLE(-1).value:
            raise ct.WinError(ct.get_last_error())
        self.handle = handle
        try:
            self.check(self.api.SetupComm(handle, 65536, 4096))
            state = DCB()
            state.length = ct.sizeof(DCB)
            self.check(self.api.GetCommState(handle, ct.byref(state)))
            if (state.baud, state.byte_size, state.parity, state.stop_bits, state.flags) != (
                baud,
                8,
                0,
                0,
                1,
            ):
                state.baud, state.byte_size, state.parity, state.stop_bits = baud, 8, 0, 0
                state.flags = 1
                self.check(self.api.SetCommState(handle, ct.byref(state)))
            self.check(
                self.api.SetCommTimeouts(handle, ct.byref(Timeouts(0xFFFFFFFF, 0, 5, 0, 1000)))
            )
        except OSError:
            self.close()
            raise

    @staticmethod
    def check(ok: object) -> None:
        """@brief 将 API 失败转换为异常。@param ok API 返回值。"""
        if not ok:
            raise ct.WinError(ct.get_last_error())

    def reopen(self, baud: int) -> None:
        """@brief 身份握手尚未成功时重开另一已知速率，不操作DTR/RTS。@param baud 候选档。"""
        port = self.port
        self.close()
        NativePort.__init__(self, port, baud)

    def set_baud(self, baud: int) -> None:
        """@brief 会话内切速，不操作DTR/RTS。@param baud 已协商的档位。"""
        if self.handle is None:
            raise OSError("串口已经关闭")
        state = DCB()
        state.length = ct.sizeof(DCB)
        self.check(self.api.GetCommState(self.handle, ct.byref(state)))
        state.baud = baud
        self.check(self.api.SetCommState(self.handle, ct.byref(state)))
        verify = DCB()
        verify.length = ct.sizeof(DCB)
        self.check(self.api.GetCommState(self.handle, ct.byref(verify)))
        if verify.baud != baud:
            raise OSError("串口驱动未接受协商速率")
        self.baud = baud

    def read(self) -> bytes:
        """@brief 短时间读取字节并核对驱动错误。@return 新字节。"""
        if self.handle is None:
            raise OSError("串口已经关闭")
        errors, status = wt.DWORD(), CommStat()
        self.check(self.api.ClearCommError(self.handle, ct.byref(errors), ct.byref(status)))
        self.rx_queue_peak = max(self.rx_queue_peak, int(status.in_queue))
        self.rx_error_flags = int(errors.value) & 0xF
        if self.rx_error_flags:
            self.rx_error_count += 1
        # RXOVER/OVERRUN/RXPARITY/FRAME允许继续分帧；其它驱动错误仍按失联处理。
        if errors.value & ~0xF:
            raise OSError(f"Windows 串口错误 flags=0x{errors.value:X}, RX_QUEUE={status.in_queue}")
        buffer = ct.create_string_buffer(4096)
        received = wt.DWORD()
        self.check(self.api.ReadFile(self.handle, buffer, len(buffer), ct.byref(received), None))
        return buffer.raw[: received.value]

    def write(self, data: bytes) -> None:
        """@brief 完整发送一条命令。@param data CRLF 字节。"""
        if self.handle is None:
            raise OSError("串口已经关闭")
        written = wt.DWORD()
        self.check(self.api.WriteFile(self.handle, data, len(data), ct.byref(written), None))
        if written.value != len(data):
            raise OSError("串口写入不完整，操作结果未知")

    def close(self) -> None:
        """@brief 释放独占句柄。"""
        if self.handle is not None:
            self.api.CloseHandle(self.handle)
            self.handle = None


def available_ports() -> list[str]:
    """@brief 仅读取 Windows 端口映射，不探测设备。@return 自然排序端口名。"""
    if sys.platform != "win32":
        return []
    import winreg

    ports: list[str] = []
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM") as key:
            for index in range(winreg.QueryInfoKey(key)[1]):
                value = winreg.EnumValue(key, index)[1]
                if isinstance(value, str) and value.startswith("COM") and value[3:].isdigit():
                    ports.append(value)
    except FileNotFoundError:
        return []
    return sorted(set(ports), key=lambda name: int(name[3:]))
