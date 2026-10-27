"""冻结发布包的离线自检，强制模拟板卡，不能访问 COM。"""

from __future__ import annotations

import json
from pathlib import Path

from PySide6.QtCore import QTimer

from . import __version__
from .models import ParameterValue
from .window import MainWindow


def run_selftest(window: MainWindow, output: Path) -> None:
    """@brief 在现有 Qt 主循环验证发布包。@param window 演示窗口。@param output 证据目录。"""
    if not window.demo:
        raise ValueError("发布包自检必须使用离线演示")
    # 自检显式验证波形通道，普通用户的新会话仍默认日常监视。
    window.plots.binary.baud.setCurrentIndex(window.plots.binary.baud.findData(921600))
    output.mkdir(parents=True, exist_ok=True)
    report: dict[str, object] = {
        "app_version": __version__,
        "demo_only": True,
        "read_only": window.read_only,
        "ok": False,
    }
    completed = False
    stage = "speed"

    def finish() -> None:
        """@brief 等待演示线程释放再退出。"""
        if window.worker is not None and window.worker.isRunning():
            window.worker.force_close()
            QTimer.singleShot(50, finish)
        else:
            window.close()

    def save() -> None:
        """@brief 将结果写入可供外部验收的 JSON。"""
        nonlocal completed
        if completed:
            return
        completed = True
        report["connected"] = window.snapshot.connected
        report["firmware"] = window.snapshot.firmware
        report["protocol"] = window.snapshot.protocol
        report["run"] = window.snapshot.run
        report["source"] = window.snapshot.source
        report["samples"] = len(window.plots.points)
        report["parameter_count"] = len(window.snapshot.parameters)
        report["stop_visible"] = window.stop_button.isVisible()
        report["output_disabled"] = window.snapshot.output_disabled
        report["ok"] = (
            bool(window.snapshot.parameters)
            and window.snapshot.connected
            and not report.get("error")
            and not window.snapshot.error
            and (
                window.read_only
                or window.snapshot.run == "STOPPED"
                and window.snapshot.output_disabled
                and report.get("current_mode_exercised") is True
                and report.get("position_mode_exercised") is True
                and report.get("haptic_mode_exercised") is True
            )
        )
        window.grab().save(str(output / "release_ui.png"))
        (output / "selftest.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        if window.worker is not None:
            window.worker.force_close()
        finish()

    def failed(action: str, message: str) -> None:
        """@brief 保存失败信息。@param action 操作。@param message 原因。"""
        report["error"] = f"{action}: {message}"
        save()

    def done(action: str, _result: object) -> None:
        """@brief 根据自检阶段推进操作。@param action 操作。@param _result 结果。"""
        nonlocal stage
        if action == "connect":
            if window.read_only:
                QTimer.singleShot(250, save)
            else:
                window.submit("start", {})
        elif action == "start":
            if stage == "speed":
                QTimer.singleShot(3200, lambda: window.submit("stop"))
            elif stage == "current":
                QTimer.singleShot(
                    800, lambda: window.submit("apply", {"TARGET": ParameterValue(-20, 3, "A")})
                )
            elif stage == "position":
                QTimer.singleShot(
                    800,
                    lambda: window.submit("apply", {"TARGET": ParameterValue(720000, 3, "DEG")}),
                )
        elif action == "apply" and stage == "current":
            target = window.snapshot.parameters.get("TARGET")
            if target == ParameterValue(-20, 3, "A"):
                QTimer.singleShot(
                    300, lambda: window.submit("apply", {"TARGET": ParameterValue(0, 3, "A")})
                )
            elif target == ParameterValue(0, 3, "A"):
                report["current_mode_exercised"] = (
                    window.snapshot.mode == "CURRENT" and window.snapshot.run == "RUNNING"
                )
                QTimer.singleShot(150, lambda: window.submit("stop"))
        elif action == "haptic_enable" and stage == "haptic":
            QTimer.singleShot(
                350,
                lambda: window.submit(
                    "apply",
                    {
                        "HAPTIC_KIND": ParameterValue(1, 0, ""),
                        "HAPTIC_DETENTS": ParameterValue(256, 0, ""),
                    },
                ),
            )
        elif action == "apply" and stage == "haptic":
            QTimer.singleShot(400, lambda: window.submit("stop"))
        elif action == "apply" and stage == "position":
            QTimer.singleShot(600, lambda: window.submit("stop"))
        elif action == "recalibrate" and stage == "position":
            report["position_initialized"] = (
                window.snapshot.encoder_calibration == "VALID"
                and window.snapshot.position is not None
                and window.snapshot.position.valid
            )
            window.submit("start", {})
        elif action == "stop":
            if stage == "speed":
                stage = "current"
                window.submit("mode", "CURRENT")
            elif stage == "current":
                stage = "position"
                window.submit("mode", "POSITION")
            elif stage == "position":
                report["position_mode_exercised"] = (
                    window.snapshot.mode == "POSITION"
                    and window.snapshot.position is not None
                    and window.snapshot.position.valid
                    and window.snapshot.position.target_deg == 720
                    and "POSITION_KP" in window.editors
                    and len(window.position_panel.points) > 3
                )
                report["position_samples"] = len(window.position_panel.points)
                window.grab().save(str(output / "release_position_ui.png"))
                stage = "haptic"
                window.submit("mode", "HAPTIC")
            elif stage == "haptic":
                sample = window.snapshot.haptic
                report["haptic_mode_exercised"] = (
                    sample is not None
                    and sample.kind == "DAMPED"
                    and sample.detents == 256
                    and window.snapshot.run == "STOPPED"
                )
                window.grab().save(str(output / "release_haptic_ui.png"))
                stage = "finish"
                window.submit("mode", "SPEED")
        elif action == "mode":
            if stage == "current":
                window.submit("start", {"TARGET": ParameterValue(20, 3, "A")})
            elif stage == "position":
                window.submit("recalibrate", {"TARGET": ParameterValue(-90000, 3, "DEG")})
            elif stage == "haptic":
                window.submit("haptic_enable", {})
            elif stage == "finish":
                QTimer.singleShot(150, save)

    def begin() -> None:
        """@brief 显式连接模拟设备。"""
        window.toggle_connection()
        if window.worker is not None:
            window.worker.operation_done.connect(done)
            window.worker.operation_failed.connect(failed)
        else:
            failed("connect", "演示线程未创建")

    QTimer.singleShot(100, begin)
    QTimer.singleShot(40000, lambda: failed("timeout", "自检超时"))
