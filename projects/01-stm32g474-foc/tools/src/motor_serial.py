"""通过 Windows 串口读取或控制正式固件，逐条记录命令和响应。"""

import argparse
import ctypes as ct
from ctypes import wintypes as wt
import json
from pathlib import Path
import time
from datetime import datetime


class DCB(ct.Structure):
    """Windows 串口配置；位字段使用原始 DWORD 保存。"""

    _fields_ = [
        ("length", wt.DWORD), ("baud", wt.DWORD), ("flags", wt.DWORD),
        ("reserved", wt.WORD), ("xon_limit", wt.WORD), ("xoff_limit", wt.WORD),
        ("byte_size", wt.BYTE), ("parity", wt.BYTE), ("stop_bits", wt.BYTE),
        ("xon", ct.c_char), ("xoff", ct.c_char), ("error", ct.c_char),
        ("eof", ct.c_char), ("event", ct.c_char), ("reserved1", wt.WORD),
    ]


class Timeouts(ct.Structure):
    """单次空闲读取最多等待5毫秒，单次写入最多等待1秒。"""

    _fields_ = [(name, wt.DWORD) for name in [
        "read_interval", "read_multiplier", "read_constant", "write_multiplier", "write_constant"
    ]]


class CommStat(ct.Structure):
    """Windows接收队列长度与状态，仅用于PC链路诊断。"""

    _fields_ = [("flags", wt.DWORD), ("in_queue", wt.DWORD), ("out_queue", wt.DWORD)]


class SerialPort:
    """独占打开明确指定的串口，不使用 DTR/RTS 复位控制板。"""

    def __init__(self, port, baud=921600):
        """@brief 配置指定速率/8N1。@param port 端口名，例如COM10。@param baud 波特率，旧镜像传115200。"""
        self.api = ct.WinDLL("kernel32", use_last_error=True)
        # 只在PC端保留与命令响应交错的实时行，不增加控制板ADC记录。
        self.unsolicited = []
        # 高速连续遥测可能遇到Windows调度暂停；仅扩大PC驱动缓冲，不缓存MCU ADC。
        self.driver_errors = []
        self.rx_queue_peak = 0
        signatures = {
            "CreateFileW": ([wt.LPCWSTR, wt.DWORD, wt.DWORD, ct.c_void_p, wt.DWORD, wt.DWORD, wt.HANDLE], wt.HANDLE),
            "CloseHandle": ([wt.HANDLE], wt.BOOL),
            "GetCommState": ([wt.HANDLE, ct.POINTER(DCB)], wt.BOOL),
            "SetCommState": ([wt.HANDLE, ct.POINTER(DCB)], wt.BOOL),
            "SetCommTimeouts": ([wt.HANDLE, ct.POINTER(Timeouts)], wt.BOOL),
            "SetupComm": ([wt.HANDLE, wt.DWORD, wt.DWORD], wt.BOOL),
            "ClearCommError": ([wt.HANDLE, ct.POINTER(wt.DWORD), ct.POINTER(CommStat)], wt.BOOL),
            "ReadFile": ([wt.HANDLE, ct.c_void_p, wt.DWORD, ct.POINTER(wt.DWORD), ct.c_void_p], wt.BOOL),
            "WriteFile": ([wt.HANDLE, ct.c_void_p, wt.DWORD, ct.POINTER(wt.DWORD), ct.c_void_p], wt.BOOL),
        }
        for name, (arguments, result) in signatures.items():
            function = getattr(self.api, name)
            function.argtypes = arguments
            function.restype = result
        self.handle = self.api.CreateFileW("\\\\.\\" + port, 0xC0000000, 0, None, 3, 0, None)
        if self.handle == wt.HANDLE(-1).value:
            raise ct.WinError(ct.get_last_error())
        self.pending = b""
        try:
            self.check(self.api.SetupComm(self.handle, 65536, 4096))
            state = DCB()
            state.length = ct.sizeof(DCB)
            self.check(self.api.GetCommState(self.handle, ct.byref(state)))
            # 当前配置已满足要求时不重复重配，减少USB串口驱动的额外状态切换。
            current = (state.baud, state.byte_size, state.parity, state.stop_bits, state.flags)
            if current != (baud, 8, 0, 0, 1):
                state.baud, state.byte_size, state.parity, state.stop_bits = baud, 8, 0, 0
                # 禁止软硬件流控、DTR和RTS，保留二进制通信。
                state.flags = 1
                self.check(self.api.SetCommState(self.handle, ct.byref(state)))
            # read_line的短截止不能被150ms底层空闲等待覆盖，尤其是快速滑行检查。
            timeouts = Timeouts(0xFFFFFFFF, 0, 5, 0, 1000)
            self.check(self.api.SetCommTimeouts(self.handle, ct.byref(timeouts)))
        except Exception:
            self.close()
            raise

    @staticmethod
    def check(ok):
        """@brief 将Windows通信错误转换为异常。@param ok API布尔结果。"""
        if not ok:
            raise ct.WinError(ct.get_last_error())

    def read_line(self, timeout=3):
        """@brief 读取完整一行。@param timeout 秒数上限。@return ASCII行，无响应返回None。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if b"\n" in self.pending:
                line, self.pending = self.pending.split(b"\n", 1)
                return line.decode("ascii", errors="strict").rstrip("\r")
            buffer = ct.create_string_buffer(4096)
            received = wt.DWORD()
            errors, status = wt.DWORD(), CommStat()
            self.check(self.api.ClearCommError(self.handle, ct.byref(errors), ct.byref(status)))
            self.rx_queue_peak = max(self.rx_queue_peak, status.in_queue)
            if errors.value:
                self.driver_errors.append({"time":datetime.now().isoformat(),"flags":errors.value,"rx_queue":status.in_queue})
            self.check(self.api.ReadFile(self.handle, buffer, len(buffer), ct.byref(received), None))
            self.pending += buffer.raw[:received.value]
            if len(self.pending) > 8192:
                raise RuntimeError("串口响应未正常结束")
        return None

    def command(self, command, timeout=3):
        """@brief 发送一条命令并等待响应。@param command 无换行ASCII命令。@param timeout 秒数上限。@return 响应。"""
        if any(char in command for char in "\r\n") or len(command) > 96:
            raise ValueError("命令格式无效")
        payload = (command + "\r\n").encode("ascii")
        written = wt.DWORD()
        self.check(self.api.WriteFile(self.handle, payload, len(payload), ct.byref(written), None))
        if written.value != len(payload):
            raise RuntimeError("串口写入不完整")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            response = self.read_line(max(0.001,deadline-time.monotonic()))
            if response is None:break
            if response.startswith(("DATA FOC ", "FOC:", "FOC_TRACE:")):
                self.unsolicited.append({"time":datetime.now().isoformat(),"unsolicited":response})
                continue
            return response
        raise TimeoutError("命令无响应：" + command)

    def close(self):
        """@brief 释放串口供用户继续使用。"""
        if self.handle is not None:
            self.api.CloseHandle(self.handle)
            self.handle = None


def main():
    """@brief 执行显式命令列表并保存时间戳；默认只查询状态。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM10")
    parser.add_argument("--baud", type=int, default=921600, help="当前开发版921600；历史镜像用115200")
    parser.add_argument("--commands", nargs="+", default=["VERSION", "STATUS", "MOTOR", "MOTOR RUN", "GET SOURCE", "GET TARGET", "GET DUTY", "GET CURRENT_LIMIT", "GET RUN_TIME", "GET START_HZ", "GET RAMP", "GET ALIGN_TIME"])
    parser.add_argument("--interval", type=float, default=0.05)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[2] / "logs" / "motor_serial",
                        help="日志输出目录，默认保存到六步问题资料/motor_serial")
    args = parser.parse_args()
    directory = args.output
    directory.mkdir(parents=True, exist_ok=True)
    output = directory / (datetime.now().strftime("%Y%m%d_%H%M%S") + ".json")
    records = []
    port = SerialPort(args.port, args.baud)
    try:
        # 记录前次遗留响应，避免误配到本次命令。
        drain_until=time.monotonic()+0.25
        while time.monotonic()<drain_until and (line := port.read_line(0.05)) is not None:
            records.append({"time": datetime.now().isoformat(), "unsolicited": line})
        for command in args.commands:
            response = port.command(command)
            records.extend(port.unsolicited)
            port.unsolicited.clear()
            record = {"time": datetime.now().isoformat(), "baud": args.baud, "command": command, "response": response}
            records.append(record)
            print(json.dumps(record, ensure_ascii=False), flush=True)
            time.sleep(args.interval)
    finally:
        records.append({"pc_serial_diagnostics":{"driver_errors":port.driver_errors,"rx_queue_peak_bytes":port.rx_queue_peak,"requested_rx_buffer_bytes":65536}})
        port.close()
        output.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
        print("串口记录：", output, flush=True)


if __name__ == "__main__":
    main()
