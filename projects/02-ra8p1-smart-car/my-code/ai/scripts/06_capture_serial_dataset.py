"""通过串口接收板端麦克风 PCM 数据并保存为 WAV。

运行示例：
    固定采集 10 条：
        python my-code/ai/scripts/06_capture_serial_dataset.py --port COM8 --count 10

    只采 wake，并校验 MCU 发来的 label 必须是 wake：
        python my-code/ai/scripts/06_capture_serial_dataset.py --port COM8 --label wake --count 10

    多标签连续采集，直到 Ctrl+C 手动结束：
        python my-code/ai/scripts/06_capture_serial_dataset.py --port COM8 --count 0 --timeout 0

依赖：
    pyserial。安装方式：pip install pyserial

配置位置： 
    DEFAULT_BAUD：默认串口波特率。
    DEFAULT_OUT_DIR：默认原始 WAV 保存目录。

重要：
    --label 只是校验 MCU 发来的 label，不会强制修改保存标签。
    真正保存目录由 MIC_DATA header 里的 label 决定：
    如果 MCU UI 会切换 wake / forward / backward / stop 等标签，
    PC 端建议不要传 --label：
    python 06_capture_serial_dataset.py --port COM3 --count 0 --timeout 0
"""

import argparse
import binascii 
import re
import sys
import time
import wave
from pathlib import Path


# AI 根目录固定为 my-code/ai，脚本从任意工作目录运行时都能定位到 labels.txt。
AI_ROOT = Path(__file__).resolve().parents[1]

# 标签文件位置。新增标签时改这里对应的 labels.txt，不要只改命令行参数。
LABELS_PATH = AI_ROOT / "labels.txt"

# 默认保存原始采集 WAV。剪辑后的 WAV 会由 07 脚本另存到 dataset_real/trimmed。
DEFAULT_OUT_DIR = AI_ROOT / "dataset_real" / "raw"

# 默认串口配置。若板端 UART 波特率变更，优先通过 --baud 覆盖；稳定后再改这里。
DEFAULT_BAUD = 230400

# 板端协议固定前缀和结束标记。后续板端实现必须和这里保持一致。
HEADER_PREFIX = "MIC_DATA "
END_MARKER = "MIC_END"

# 非 MIC_DATA 日志最长显示长度，防止 PCM 残留刷屏。
MAX_LOG_LINE_DISPLAY_LEN = 160

EXPECTED_RECORD_SECONDS = 3
EXPECTED_RECORD_TOLERANCE_SECONDS = 0.20

class CaptureError(RuntimeError):
    """采集流程中的可恢复错误。"""


def load_labels() -> list[str]:
    """读取 labels.txt，限制采集标签必须来自训练标签表。"""

    labels = []
    for line in LABELS_PATH.read_text(encoding="utf-8").splitlines():
        label = line.strip()
        if label and not label.startswith("#"):
            labels.append(label)

    if not labels:
        raise CaptureError(f"标签文件为空: {LABELS_PATH}")

    return labels


def next_wav_path(out_dir: Path, label: str) -> Path:
    """为指定标签生成下一个递增 WAV 文件名，避免覆盖已有样本。"""

    label_dir = out_dir / label
    label_dir.mkdir(parents=True, exist_ok=True)

    pattern = re.compile(rf"^{re.escape(label)}_(\d+)\.wav$", re.IGNORECASE)
    max_index = 0

    for path in label_dir.glob(f"{label}_*.wav"):
        match = pattern.match(path.name)
        if match:
            max_index = max(max_index, int(match.group(1)))

    return label_dir / f"{label}_{max_index + 1:04d}.wav"


def parse_header(line: str) -> dict[str, str]:
    """解析板端返回的 MIC_DATA 头。

    头格式示例：
        MIC_DATA v=1 label=wake rate=16129 samples=32258 bytes=64516 crc=89ABCDEF
    """

    if not line.startswith(HEADER_PREFIX):
        raise CaptureError(f"收到非 MIC_DATA 头: {line!r}")

    result: dict[str, str] = {}
    for item in line[len(HEADER_PREFIX):].strip().split():
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        result[key] = value

    required = ("v", "label", "rate", "samples", "bytes", "crc")
    missing = [key for key in required if key not in result]
    if missing:
        raise CaptureError(f"MIC_DATA 头缺少字段: {', '.join(missing)}")

    return result


def make_deadline(timeout_s: float) -> float | None:
    """timeout_s <= 0 时返回 None 表示无限等待，否则返回绝对时间。"""

    if timeout_s <= 0:
        return None

    return time.monotonic() + timeout_s


def deadline_expired(deadline: float | None) -> bool:
    """deadline 为 None 表示永不超时。"""

    return (deadline is not None) and (time.monotonic() >= deadline)


def deadline_remaining(deadline: float | None) -> float:
    """返回距离 deadline 的剩余时间，单位秒；None 返回 0.0 表示无限等待。"""

    if deadline is None:
        return 0.0

    return max(0.1, deadline - time.monotonic())


def is_printable_log_line(line: str) -> bool:
    """判断一行是否适合按普通串口日志打印。"""

    if len(line) > MAX_LOG_LINE_DISPLAY_LEN:
        return False

    for ch in line:
        code = ord(ch)
        if ch in "\r\n\t":
            continue
        if code < 32 or code > 126:
            return False

    return True


def print_ignored_line(line: str) -> None:
    """打印被忽略的非 MIC_DATA 行，避免二进制 PCM 残留刷屏。"""

    return


def read_ascii_line(serial_port, timeout_s: float) -> str:
    """从串口读取一行 ASCII 文本。

    payload 是二进制数据，不能用 readline 混读。
    这里只在头部和结束标记阶段调用。
    timeout_s <= 0 表示无限等待。
    """

    deadline = make_deadline(timeout_s)
    buffer = bytearray()

    while not deadline_expired(deadline):
        chunk = serial_port.read(1)
        if not chunk:
            continue

        buffer += chunk

        if chunk == b"\n":
            try:
                return buffer.decode("ascii", errors="strict").strip()
            except UnicodeDecodeError as exc:
                raise CaptureError(f"串口行包含非 ASCII 数据: len={len(buffer)}") from exc

    raise CaptureError("等待串口行超时")


def wait_for_header(serial_port, timeout_s: float) -> dict[str, str]:
    """等待 MIC_DATA 头，期间允许忽略板端启动日志或调试日志。"""

    deadline = make_deadline(timeout_s)

    while not deadline_expired(deadline):
        line = read_ascii_line(serial_port, deadline_remaining(deadline))
        if not line:
            continue

        if line.startswith(HEADER_PREFIX):
            return parse_header(line)

        print_ignored_line(line)

    raise CaptureError("等待 MIC_DATA 头超时")


def read_exact(serial_port, length: int, timeout_s: float) -> bytes:
    """严格读取指定字节数的二进制 payload。

    这里不能按行读取，因为 PCM16LE 中可能包含任意 0x0A/0x0D 字节。
    timeout_s <= 0 表示无限等待。
    """

    deadline = make_deadline(timeout_s)
    data = bytearray()

    while len(data) < length and not deadline_expired(deadline):
        need = length - len(data)
        chunk = serial_port.read(need)
        if chunk:
            data += chunk

    if len(data) != length:
        raise CaptureError(f"payload 长度不足: got={len(data)} expected={length}")

    return bytes(data)


def read_end_marker(serial_port, timeout_s: float) -> None:
    """payload 后等待 MIC_END，确认本条样本完整结束。"""

    deadline = make_deadline(timeout_s)

    while not deadline_expired(deadline):
        line = read_ascii_line(serial_port, deadline_remaining(deadline))
        if not line:
            continue

        if line == END_MARKER:
            return

        print_ignored_line(line)

    raise CaptureError("等待 MIC_END 超时")


def discard_packet_payload(serial_port, payload_bytes: int, timeout_s: float) -> None:
    """丢弃当前 MIC_DATA 包的 payload 和 MIC_END，用于错误后的重同步。

    典型场景：
    - MCU 发来的 label 和 --label 不一致；
    - MCU 发来的 label 不在 labels.txt 中；
    - 当前包不打算保存，但必须把 payload 和 MIC_END 消费掉。
    """

    try:
        if payload_bytes > 0:
            _ = read_exact(serial_port, payload_bytes, timeout_s)
    except CaptureError:
        return

    try:
        read_end_marker(serial_port, timeout_s)
    except CaptureError:
        return


def write_pcm16_wav(path: Path, payload: bytes, sample_rate: int) -> None:
    """把板端 PCM16LE payload 原样封装成单声道 WAV。"""

    if len(payload) % 2 != 0:
        raise CaptureError(f"PCM16 payload 字节数不是偶数: {len(payload)}")

    path.parent.mkdir(parents=True, exist_ok=True)

    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(payload)


def capture_one(
    serial_port,
    expected_label: str | None,
    valid_labels: set[str],
    allow_extra_labels: bool,
    out_dir: Path,
    timeout_s: float,
) -> tuple[Path, str]:
    """被动等待 MCU 发送一条协议包，并保存为 WAV。

    注意：
    标签不匹配或标签非法时，也要先丢弃当前包 payload 和 MIC_END，
    否则下一轮会把 PCM payload 当成串口日志，导致解析失步。
    """

    header = wait_for_header(serial_port, timeout_s)

    label = header["label"]

    try:
        sample_rate = int(header["rate"], 10)
        sample_count = int(header["samples"], 10)
        payload_bytes = int(header["bytes"], 10)
        expected_crc = int(header["crc"].lower().removeprefix("0x"), 16)
    except ValueError as exc:
        raise CaptureError(f"MIC_DATA 头字段格式错误: {header}") from exc

    if sample_rate <= 0:
        raise CaptureError(f"采样率非法: {sample_rate}")

    if sample_count <= 0:
        raise CaptureError(f"样本数非法: {sample_count}")

    if payload_bytes <= 0:
        raise CaptureError(f"payload 字节数非法: {payload_bytes}")

    if payload_bytes != sample_count * 2:
        raise CaptureError(f"样本数和字节数不匹配: samples={sample_count} bytes={payload_bytes}")

    actual_seconds = sample_count / float(sample_rate)
    min_seconds = EXPECTED_RECORD_SECONDS - EXPECTED_RECORD_TOLERANCE_SECONDS
    max_seconds = EXPECTED_RECORD_SECONDS + EXPECTED_RECORD_TOLERANCE_SECONDS
    if (actual_seconds < min_seconds) or (actual_seconds > max_seconds):
        discard_packet_payload(serial_port, payload_bytes, timeout_s)
        raise CaptureError(
            f"录音长度不是接近 {EXPECTED_RECORD_SECONDS}s: "
            f"actual={actual_seconds:.3f}s rate={sample_rate} samples={sample_count} "
            f"allow={min_seconds:.3f}s~{max_seconds:.3f}s"
        )

    if (expected_label is not None) and (label != expected_label):
        discard_packet_payload(serial_port, payload_bytes, timeout_s)
        raise CaptureError(f"返回标签不匹配: got={label} expected={expected_label}")

    if (label not in valid_labels) and (not allow_extra_labels):
        discard_packet_payload(serial_port, payload_bytes, timeout_s)
        raise CaptureError(f"板端标签不在 labels.txt 中: {label}")

    payload = read_exact(serial_port, payload_bytes, timeout_s)
    actual_crc = binascii.crc32(payload) & 0xFFFFFFFF

    if actual_crc != expected_crc:
        try:
            read_end_marker(serial_port, timeout_s)
        except CaptureError:
            pass

        raise CaptureError(f"CRC 不匹配: got=0x{actual_crc:08X} expected=0x{expected_crc:08X}")

    read_end_marker(serial_port, timeout_s)

    wav_path = next_wav_path(out_dir, label)
    write_pcm16_wav(wav_path, payload, sample_rate)

    return wav_path, label


def main() -> int:
    parser = argparse.ArgumentParser(description="被动接收 RA8P1 串口音频协议包并保存 WAV")
    parser.add_argument("--port", required=True, help="串口号，例如 COM8")
    parser.add_argument("--label", help="可选标签校验；不填写时使用 MCU 头部 label")
    parser.add_argument("--count", type=int, default=1, help="连续采集条数；0 表示一直采集直到 Ctrl+C")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="串口波特率")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="原始 WAV 输出根目录")
    parser.add_argument("--timeout", type=float, default=15.0, help="单条采集超时时间，单位秒；0 表示无限等待")
    parser.add_argument("--settle-ms", type=int, default=300, help="两条样本之间的等待时间，单位 ms")
    parser.add_argument("--allow-extra-labels", action="store_true", help="允许保存不在 labels.txt 中的调试标签")
    args = parser.parse_args()

    try:
        labels = load_labels()
    except CaptureError as exc:
        print(f"标签读取失败: {exc}")
        return 1

    if (args.label is not None) and (args.label not in labels) and (not args.allow_extra_labels):
        print(f"标签不在 labels.txt 中: {args.label}")
        print(f"可用标签: {', '.join(labels)}")
        print("如需接收调试标签，请增加 --allow-extra-labels")
        return 1

    if args.count < 0:
        print("--count 不能小于 0")
        return 1

    if args.baud <= 0:
        print("--baud 必须大于 0")
        return 1

    if args.settle_ms < 0:
        print("--settle-ms 不能小于 0")
        return 1

    try:
        import serial
    except ImportError:
        print("缺少 pyserial，请先安装: pip install pyserial")
        return 1

    saved = 0
    failed = 0
    index = 0
    valid_labels = set(labels)

    if args.label is None:
        print("标签模式: 自动按 MCU header 中的 label 保存")
    else:
        print(f"标签模式: 仅校验并接收 label={args.label}")

    try:
        with serial.Serial(
            args.port,
            args.baud,
            timeout=0.1,
            write_timeout=args.timeout if args.timeout > 0 else None,
        ) as serial_port:
            print(f"串口已打开: {args.port} baud={args.baud}")
            print(f"输出目录: {args.out_dir}")

            if hasattr(serial_port, "reset_input_buffer"):
                serial_port.reset_input_buffer()

            while (args.count == 0) or (index < args.count):
                index += 1

                if args.count == 0:
                    print(f"[{index}] 等待 MCU 主动发送 MIC_DATA，Ctrl+C 结束")
                else:
                    print(f"[{index}/{args.count}] 等待 MCU 主动发送 MIC_DATA")

                try:
                    path, label = capture_one(
                        serial_port=serial_port,
                        expected_label=args.label,
                        valid_labels=valid_labels,
                        allow_extra_labels=args.allow_extra_labels,
                        out_dir=args.out_dir,
                        timeout_s=args.timeout,
                    )
                except CaptureError as exc:
                    failed += 1
                    print(f"采集失败: {exc}")

                    # 失败后清空残留，防止下一轮从半包 payload 或 MIC_END 开始解析。
                    if hasattr(serial_port, "reset_input_buffer"):
                        serial_port.reset_input_buffer()

                    continue

                saved += 1
                print(f"标签: {label}")
                print(f"保存位置: {path}")

                if args.settle_ms > 0:
                    if (args.count == 0) or (index < args.count):
                        time.sleep(args.settle_ms / 1000.0)

    except KeyboardInterrupt:
        print("\n用户手动结束采集")

    except Exception as exc:
        print(f"串口采集异常: {exc}")
        print(f"完成: success={saved} failed={failed}")
        return 1

    print(f"完成: success={saved} failed={failed}")

    if args.count == 0:
        return 0 if saved > 0 else 1

    return 0 if saved == args.count else 1


if __name__ == "__main__":
    sys.exit(main())
