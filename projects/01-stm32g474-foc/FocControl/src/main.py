"""FocControl 桌面入口，默认启动界面但不自动打开 COM。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from foc_control import __version__
from foc_control.theme import install_theme
from foc_control.window import MainWindow
from PySide6.QtWidgets import QApplication, QMessageBox


def main() -> int:
    """@brief 解析启动选项并运行 Qt。@return 进程退出码。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", action="version", version=f"FocControl {__version__}")
    parser.add_argument("--read-only", action="store_true", help="发送层禁止所有写命令")
    parser.add_argument("--demo", action="store_true", help="仅使用演示板卡，绝不访问 COM")
    parser.add_argument("--data-dir", type=Path, help="明确指定可写数据目录")
    parser.add_argument("--self-test", type=Path, help="发布包离线自检证据目录，强制 DEMO")
    args = parser.parse_args()
    base = (
        Path(sys.executable).resolve().parent
        if getattr(sys, "frozen", False)
        else Path(__file__).resolve().parents[1]
    )
    app = QApplication(sys.argv[:1])
    app.setApplicationName("FocControl")
    install_theme(app)
    try:
        window = MainWindow(
            args.data_dir or base / "data", args.read_only, args.demo or bool(args.self_test)
        )
    except OSError as error:
        QMessageBox.critical(None, "数据目录不可写", f"{error}\n可使用 --data-dir 指定可写目录。")
        return 1
    window.show()
    if args.self_test:
        from foc_control.selftest import run_selftest

        run_selftest(window, args.self_test)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
