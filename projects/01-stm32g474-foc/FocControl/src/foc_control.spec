# FocControl 文件夹发布包；保留独立 Qt DLL，避免压缩或单文件解包行为。
from pathlib import Path
from importlib.util import find_spec
from importlib.metadata import distribution
import sys

project_root = Path(SPECPATH).parent
pyside_spec = find_spec("PySide6")
if pyside_spec is None or pyside_spec.origin is None:
    raise RuntimeError("缺少 PySide6，请先运行 setup.ps1。")
pyside_root = Path(pyside_spec.origin).parent
# 使用 Qt 附带的新运行库，避免 Python 的旧副本优先加载。
runtime_binaries = [
    (str(pyside_root / name), ".")
    for name in ("vcruntime140.dll", "vcruntime140_1.dll")
    if (pyside_root / name).is_file()
]
# 保留已安装运行组件随包提供的全部许可和归属文本。
license_datas = []
for package_name in (
    "PySide6",
    "PySide6_Essentials",
    "shiboken6",
    "pyqtgraph",
    "numpy",
    "pyinstaller",
):
    package = distribution(package_name)
    for relative in package.files or []:
        if "licenses" in relative.parts:
            index = relative.parts.index("licenses")
            destination = (
                Path("licenses") / package_name / Path(*relative.parts[index + 1 :]).parent
            )
            license_datas.append((str(package.locate_file(relative)), str(destination)))
python_license = Path(sys.base_prefix) / "LICENSE.txt"
if python_license.is_file():
    license_datas.append((str(python_license), "licenses/Python"))
analysis = Analysis(
    [str(project_root / "src" / "main.py")],
    pathex=[str(project_root / "src")],
    binaries=runtime_binaries,
    datas=[
        (str(project_root / "README.md"), "."),
        (str(project_root / "THIRD_PARTY_NOTICES.md"), "."),
    ]
    + license_datas,
    hiddenimports=["PySide6.QtSvg"],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        "PySide6.QtWebEngineCore",
        "PySide6.QtWebEngineWidgets",
        "PySide6.QtWebEngineQuick",
        "PySide6.QtPdf",
        "PySide6.QtPdfWidgets",
        "PySide6.QtQuick",
        "PySide6.QtQml",
        "PySide6.QtMultimedia",
        "PySide6.QtMultimediaWidgets",
        "PySide6.Qt3DCore",
        "matplotlib",
        "scipy",
        "IPython",
        "pytest",
        "mypy",
        "black",
        "ruff",
    ],
    noarchive=False,
)
# Qt 使用 Windows 自带的 ICU；同名 Poppler DLL 的符号版本不兼容。
# 不分发系统 ICU/UCRT，也不把 PATH 中另一套工具的 ICU 带入程序。
analysis.binaries = [
    entry
    for entry in analysis.binaries
    if Path(entry[0]).name.casefold() not in {"icuuc.dll", "icuin.dll", "icu.dll", "ucrtbase.dll"}
    and not (
        Path(entry[0]).name.casefold().startswith("icudt") and entry[0].casefold().endswith(".dll")
    )
]
archive = PYZ(analysis.pure)
executable = EXE(
    archive,
    analysis.scripts,
    [],
    exclude_binaries=True,
    name="FocControl",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
)
bundle = COLLECT(
    executable,
    analysis.binaries,
    analysis.datas,
    strip=False,
    upx=False,
    name="FocControl",
)
