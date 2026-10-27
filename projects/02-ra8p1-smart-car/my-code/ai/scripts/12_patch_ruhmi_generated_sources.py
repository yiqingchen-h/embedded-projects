#!/usr/bin/env python3
"""修补 RUHMI 生成源码，使其适配当前 Keil 工程。"""

from __future__ import annotations

import argparse
from pathlib import Path


DEFAULT_SRC_DIR = (
    Path(__file__).resolve().parents[1]
    / "ruhmi_generated"
    / "kws_current"
    / "output"
    / "build"
    / "MCU"
    / "compilation"
    / "src"
)

ARM_NN_TYPES_H = """#ifndef ARM_NN_TYPES_H_
#define ARM_NN_TYPES_H_

/*
 * RUHMI 生成的 CPU 包装代码会包含 CMSIS-NN 头文件。
 * 当前 KWS 模型未直接使用 CMSIS-NN 类型，这里保留最小兼容头，避免每次 deploy 后缺头文件。
 */
#include <stdint.h>

#endif /* ARM_NN_TYPES_H_ */
"""

ARM_NNFUNCTIONS_H = """#ifndef ARM_NNFUNCTIONS_H_
#define ARM_NNFUNCTIONS_H_

/*
 * 当前 RUHMI 生成路径只需要该头文件存在，实际量化/反量化函数来自 kernel_library_int.c。
 */
#include "arm_nn_types.h"

#endif /* ARM_NNFUNCTIONS_H_ */
"""


def write_if_changed(path: Path, content: str) -> bool:
    """仅在内容变化时写入文件。"""

    if path.exists() and path.read_text(encoding="utf-8") == content:
        return False

    path.write_text(content, encoding="utf-8", newline="\n")
    return True


def patch_text_file(path: Path, replacements: list[tuple[str, str]]) -> bool:
    """按固定文本替换修补生成文件。"""

    text = path.read_text(encoding="utf-8")
    patched = text
    for old, new in replacements:
        patched = patched.replace(old, new)

    if patched == text:
        return False

    path.write_text(patched, encoding="utf-8", newline="\n")
    return True


def patch_compute_file(path: Path) -> bool:
    """消除 RUHMI CPU wrapper 的 main_storage 未使用告警。"""

    return patch_text_file(
        path,
        [
            (
                ") {\n  // Buffers allocated on the main storage",
                ") {\n  (void)main_storage;\n\n  // Buffers allocated on the main storage",
            ),
        ],
    )


def patch_model_file(path: Path) -> bool:
    """避免 model.c 局部指针名遮蔽 model.h 中的全局输出 buffer。"""

    return patch_text_file(
        path,
        [
            (
                "int8_t* buf_StatefulPartitionedCall_0_70026_10088 = "
                "(int8_t*) (sub_0001_arena + sub_0001_address_StatefulPartitionedCall_0_70026_10088);",
                "int8_t* p_buf_StatefulPartitionedCall_0_70026_10088 = "
                "(int8_t*) (sub_0001_arena + sub_0001_address_StatefulPartitionedCall_0_70026_10088);",
            ),
            (
                "compute_sub_0002(compute_arena_sub_0002, "
                "buf_StatefulPartitionedCall_0_70026_10088, buf_StatefulPartitionedCall_0_70026  );",
                "compute_sub_0002(compute_arena_sub_0002, "
                "p_buf_StatefulPartitionedCall_0_70026_10088, buf_StatefulPartitionedCall_0_70026  );",
            ),
        ],
    )


def report_model_data_location(src_dir: Path) -> None:
    """报告 NPU model data 是否由 RUHMI 放到 OSPI 加载区。"""

    model_data = src_dir / "sub_0001_model_data.c"
    text = model_data.read_text(encoding="utf-8")
    if 'section(".sdram_from_ospi0_cs1")' not in text:
        print("sub_0001_model_data.c 使用默认 Flash 段，是否超限以 Keil 链接结果为准。")
        return

    print("sub_0001_model_data.c 使用 .sdram_from_ospi0_cs1。")


def main() -> int:
    """命令行入口。"""

    parser = argparse.ArgumentParser(description="Patch RUHMI generated sources for the smart_car Keil project.")
    parser.add_argument("--src-dir", type=Path, default=DEFAULT_SRC_DIR, help="RUHMI MCU compilation/src directory.")
    args = parser.parse_args()

    src_dir = args.src_dir.resolve()
    if not src_dir.is_dir():
        raise FileNotFoundError(f"RUHMI source directory not found: {src_dir}")

    changed: list[Path] = []
    for filename, content in (
        ("arm_nn_types.h", ARM_NN_TYPES_H),
        ("arm_nnfunctions.h", ARM_NNFUNCTIONS_H),
    ):
        path = src_dir / filename
        if write_if_changed(path, content):
            changed.append(path)

    for filename in ("compute_sub_0000.c", "compute_sub_0002.c"):
        path = src_dir / filename
        if patch_compute_file(path):
            changed.append(path)

    model_path = src_dir / "model.c"
    if patch_model_file(model_path):
        changed.append(model_path)

    report_model_data_location(src_dir)

    if changed:
        print("已修补 RUHMI 生成文件:")
        for path in changed:
            print(f"  {path}")
    else:
        print("RUHMI 生成文件已是修补后状态。")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
