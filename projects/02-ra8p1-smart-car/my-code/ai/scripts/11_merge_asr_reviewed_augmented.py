"""按 ASR 粗检结果把增强 1 秒样本合并到 combined_1s。"""

from __future__ import annotations

import argparse
import csv
import shutil
from pathlib import Path

from kws_common import AI_ROOT, load_labels

DEFAULT_INPUT_DIR = AI_ROOT / "dataset_real" / "enhance_raw_trimmed"
DEFAULT_OUTPUT_DIR = AI_ROOT / "dataset_real" / "combined_1s"
DEFAULT_ASR_CSV = DEFAULT_INPUT_DIR / "transcribe_result_all_openvino_gpu.csv"
NOISE_LABELS = {"unknown", "silence"}


def should_keep(row: dict[str, str], strict: bool) -> tuple[bool, str]:
    """根据 ASR 结果做粗筛，保留口音导致的未识别样本。"""

    expected_label = row.get("expected_label", "").strip()
    pass_by_label = row.get("pass_by_label", "").strip()
    command_hits = row.get("command_hits", "").strip()
    error = row.get("error", "").strip()

    if error:
        return False, "asr_error"

    if expected_label in NOISE_LABELS:
        if "1" == pass_by_label:
            return True, "noise_no_command_hit"
        return False, "noise_has_command_hit"

    if "1" == pass_by_label:
        return True, "command_label_match"

    if not strict and "" == command_hits:
        return True, "command_asr_no_hit_keep_lenient"

    return False, "command_mismatch"


def main() -> int:
    parser = argparse.ArgumentParser(description="按 ASR 粗检结果合并增强样本到 combined_1s")
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR, help="enhance_raw_trimmed 根目录")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="combined_1s 根目录")
    parser.add_argument("--asr-csv", type=Path, default=DEFAULT_ASR_CSV, help="batch_transcribe_GPU.py 输出 CSV")
    parser.add_argument("--strict", action="store_true", help="严格模式：命令词 ASR 不命中目标就丢弃")
    parser.add_argument("--overwrite", action="store_true", help="覆盖 combined_1s 中已存在同名文件")
    parser.add_argument("--dry-run", action="store_true", help="只统计，不复制")
    args = parser.parse_args()

    if not args.input_dir.exists():
        print(f"输入目录不存在: {args.input_dir}")
        return 1
    if not args.asr_csv.exists():
        print(f"ASR CSV 不存在: {args.asr_csv}")
        return 1

    labels = load_labels()
    label_set = set(labels)
    copied = 0
    skipped_existing = 0
    missing_file = 0
    kept_by_label = {label: 0 for label in labels}
    rejected_by_label = {label: 0 for label in labels}
    reject_reasons: dict[str, int] = {}
    manifest_rows: list[dict[str, str]] = []

    with args.asr_csv.open("r", encoding="utf-8-sig", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            expected_label = row.get("expected_label", "").strip()
            relative_path_text = row.get("relative_path", "").strip()

            if expected_label not in label_set or not relative_path_text:
                continue

            source_path = args.input_dir / relative_path_text
            target_path = args.output_dir / relative_path_text
            keep, reason = should_keep(row, args.strict)

            if not keep:
                rejected_by_label[expected_label] += 1
                reject_reasons[reason] = reject_reasons.get(reason, 0) + 1
                continue

            if not source_path.exists():
                missing_file += 1
                continue

            if target_path.exists() and not args.overwrite:
                skipped_existing += 1
            else:
                if not args.dry_run:
                    target_path.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source_path, target_path)
                copied += 1

            kept_by_label[expected_label] += 1
            manifest_rows.append(
                {
                    "label": expected_label,
                    "source": str(source_path.relative_to(args.input_dir)),
                    "target": str(target_path.relative_to(args.output_dir)),
                    "asr_text": row.get("asr_text", ""),
                    "command_hits": row.get("command_hits", ""),
                    "pass_by_label": row.get("pass_by_label", ""),
                    "keep_reason": reason,
                }
            )

    if not args.dry_run and manifest_rows:
        manifest_path = args.output_dir / "enhanced_merge_manifest.csv"
        with manifest_path.open("w", encoding="utf-8-sig", newline="") as manifest_file:
            writer = csv.DictWriter(manifest_file, fieldnames=list(manifest_rows[0].keys()))
            writer.writeheader()
            writer.writerows(manifest_rows)

    print(f"合并完成: input={args.input_dir} output={args.output_dir}")
    if args.dry_run:
        print("dry-run: 未复制 WAV")
    print(f"copied={copied} skipped_existing={skipped_existing} missing_file={missing_file}")
    print("保留数量:")
    for label in labels:
        print(f"  {label}: kept={kept_by_label[label]} rejected={rejected_by_label[label]}")
    if reject_reasons:
        print("拒绝原因:")
        for reason, count in sorted(reject_reasons.items()):
            print(f"  {reason}: {count}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
