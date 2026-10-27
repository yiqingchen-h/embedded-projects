"""只读检查发布包的 PE 导入依赖，定位同名 DLL 的版本冲突。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pefile


def main() -> int:
    """@brief 比对本地 DLL 导出与引用符号，不加载或修改系统库。@return 退出码。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    args = parser.parse_args()
    files = list(args.bundle.rglob("*.dll")) + list(args.bundle.rglob("*.pyd"))
    exports: dict[str, list[tuple[Path, set[bytes]]]] = {}
    for path in files:
        library = pefile.PE(str(path), fast_load=True, max_symbol_exports=65536)
        library.parse_data_directories(
            directories=[pefile.DIRECTORY_ENTRY["IMAGE_DIRECTORY_ENTRY_EXPORT"]]
        )
        if hasattr(library, "DIRECTORY_ENTRY_EXPORT"):
            names = {item.name for item in library.DIRECTORY_ENTRY_EXPORT.symbols if item.name}
            exports.setdefault(path.name.casefold(), []).append((path, names))
        library.close()
    missing: list[dict[str, object]] = []
    for path in files:
        library = pefile.PE(str(path), fast_load=True, max_symbol_exports=65536)
        library.parse_data_directories(
            directories=[pefile.DIRECTORY_ENTRY["IMAGE_DIRECTORY_ENTRY_IMPORT"]]
        )
        for dependency in getattr(library, "DIRECTORY_ENTRY_IMPORT", []):
            name = dependency.dll.decode("ascii").casefold()
            for candidate, names in exports.get(name, []):
                absent = [
                    item.name.decode("ascii")
                    for item in dependency.imports
                    if item.name and item.name not in names
                ]
                if absent:
                    missing.append(
                        {"file": str(path), "dependency": str(candidate), "missing": absent}
                    )
        library.close()
    print(json.dumps(missing, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
