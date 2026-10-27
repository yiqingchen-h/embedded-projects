"""按串口有效吞吐模拟连续数据，检查分帧、计数回绕与缓存上限。"""

from __future__ import annotations

from foc_control.protocol import LineDecoder, TelemetryBuffer, parse_telemetry
from foc_control.simulator import DemoTransport


def test_sixty_seconds_wire_stream() -> None:
    """@brief 模拟约60秒、921600/8N1级别的数据量，不补发或伪造实测。"""
    port = DemoTransport()
    parts = port.trace().split(":")[1].split(",")
    decoder = LineDecoder()
    buffer = TelemetryBuffer(20000)
    pending = bytearray()
    count = 0
    # 每秒约460条200字节TRACE，覆盖60秒并穿过25.26秒的tick回绕。
    for index in range(27600):
        parts[0] = str(index * 22 + 1)
        parts[1] = str((index * 370000) & 0xFFFFFFFF)
        parts[3] = "6"
        pending.extend(("FOC_TRACE:" + ",".join(parts) + "\r\n").encode())
        if len(pending) >= 4096:
            for line in decoder.feed(bytes(pending)):
                assert isinstance(line, str)
                frame = parse_telemetry(line)
                assert frame is not None
                buffer.append(frame, index / 460, "DEMO_LOAD", 2000)
                count += 1
            pending.clear()
    for line in decoder.feed(bytes(pending)):
        assert isinstance(line, str)
        frame = parse_telemetry(line)
        assert frame is not None
        buffer.append(frame, 60, "DEMO_LOAD", 2000)
        count += 1
    assert count == 27600
    assert len(buffer.points) == 20000
    assert buffer.points[-1].seconds > 59
