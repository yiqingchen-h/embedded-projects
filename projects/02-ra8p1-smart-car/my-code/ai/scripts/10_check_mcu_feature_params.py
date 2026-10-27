"""检查训练报告中的 feature_mean/std 是否已同步到 MCU voice_ai.c。"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

AI_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = AI_ROOT.parents[1]
DEFAULT_REPORT = AI_ROOT / "artifacts_real" / "tflite_training_report.md"
DEFAULT_VOICE_AI = PROJECT_ROOT / "my-code" / "software" / "voice_ai.c"
DEFAULT_TOLERANCE = 1.0e-6


def parse_report_value(text: str, name: str) -> float:
    """从训练报告中解析 feature_mean 或 feature_std。"""

    match = re.search(rf"^-\s+{re.escape(name)}:\s*([-+0-9.eE]+)\s*$", text, flags=re.M)
    if not match:
        raise RuntimeError(f"训练报告缺少 {name}")
    return float(match.group(1))


def parse_voice_ai_macro(text: str, macro: str) -> float:
    """从 voice_ai.c 中解析特征标准化宏。"""

    pattern = rf"^\s*#define\s+{re.escape(macro)}\s+\(\s*([-+0-9.eE]+)f?\s*\)\s*$"
    match = re.search(pattern, text, flags=re.M)
    if not match:
        raise RuntimeError(f"voice_ai.c 缺少 {macro}")
    return float(match.group(1))


def compare_value(name: str, report_value: float, mcu_value: float, tolerance: float) -> bool:
    """打印并比较单个参数。"""

    diff = abs(report_value - mcu_value)
    matched = diff <= tolerance
    status = "MATCH" if matched else "MISMATCH"
    print(
        f"{name}: report={report_value:.8f} voice_ai.c={mcu_value:.8f} "
        f"diff={diff:.8g} -> {status}"
    )
    return matched


def main() -> int:
    parser = argparse.ArgumentParser(description="检查训练报告与 voice_ai.c 的 feature_mean/std 是否一致")
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT, help="tflite_training_report.md 路径")
    parser.add_argument("--voice-ai", type=Path, default=DEFAULT_VOICE_AI, help="voice_ai.c 路径")
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE, help="允许误差")
    args = parser.parse_args()

    if not args.report.exists():
        print(f"训练报告不存在: {args.report}")
        return 1
    if not args.voice_ai.exists():
        print(f"voice_ai.c 不存在: {args.voice_ai}")
        return 1

    report_text = args.report.read_text(encoding="utf-8")
    voice_ai_text = args.voice_ai.read_text(encoding="utf-8")

    report_mean = parse_report_value(report_text, "feature_mean")
    report_std = parse_report_value(report_text, "feature_std")
    mcu_mean = parse_voice_ai_macro(voice_ai_text, "VOICE_AI_FEATURE_MEAN")
    mcu_std = parse_voice_ai_macro(voice_ai_text, "VOICE_AI_FEATURE_STD")

    mean_ok = compare_value("feature_mean", report_mean, mcu_mean, args.tolerance)
    std_ok = compare_value("feature_std", report_std, mcu_std, args.tolerance)

    if not mean_ok or not std_ok:
        print("feature_mean/std 未同步到 voice_ai.c，停止部署。")
        return 1

    print("feature_mean/std 已同步。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
