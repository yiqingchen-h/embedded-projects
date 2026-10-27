# 第三方组件

本软件使用以下组件，发布包在 `_internal/licenses/` 保留安装包附带的许可与归属文本，并保留独立 Qt 动态库。

| 组件 | 用途 | 项目 |
| --- | --- | --- |
| Python | 运行时 | https://www.python.org/ |
| PySide6 / Qt | 界面、SVG 与事件系统 | https://doc.qt.io/qtforpython/ |
| pyqtgraph | 工程波形 | https://www.pyqtgraph.org/ |
| NumPy | 绘图数值数组 | https://numpy.org/ |
| PyInstaller | 免安装发布包 | https://pyinstaller.org/ |

具体版本由 `requirements-lock.txt` 锁定；组件原有许可适用。Qt DLL 位于 `_internal/PySide6`，未进行单文件封装，可替换动态组件。微软雅黑、Segoe UI 和 Consolas 使用系统已安装字体，不随本项目分发。ICU/UCRT 使用 Windows 自带组件；本包不包含本机 Poppler 或其它工具的 ICU DLL。

Windows 串口配置与命名预设格式源自同一工程既有工具，所需实现已纳入 `src/foc_control`，程序不依赖工程外脚本。
