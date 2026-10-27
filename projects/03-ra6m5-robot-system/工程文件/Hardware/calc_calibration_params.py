#!/usr/bin/env python3


from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np


def load_csv(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            parsed: dict[str, float] = {}
            for key, value in row.items():
                if key is None:
                    continue
                value = (value or "").strip()
                if key == "id":
                    continue
                parsed[key] = float(value)
            rows.append(parsed)
    return rows


def rms(errors: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.sum(errors * errors, axis=1))))


def solve_plane(rows: list[dict[str, float]]) -> None:
    if len(rows) < 3:
        raise ValueError("平面标定至少需要 3 个点。")

    uv = np.array([[r["u"], r["v"], 1.0] for r in rows], dtype=np.float64)
    xb = np.array([r["x_base"] for r in rows], dtype=np.float64)
    yb = np.array([r["y_base"] for r in rows], dtype=np.float64)

    ax, _, _, _ = np.linalg.lstsq(uv, xb, rcond=None)
    ay, _, _, _ = np.linalg.lstsq(uv, yb, rcond=None)

    pred_x = uv @ ax
    pred_y = uv @ ay
    err = np.column_stack((pred_x - xb, pred_y - yb))

    print("/* ---- Plane Calibration Result ---- */")
    print(f"#define CALIB_A11                             ({ax[0]:.8f}f)")
    print(f"#define CALIB_A12                             ({ax[1]:.8f}f)")
    print(f"#define CALIB_A13                             ({ax[2]:.8f}f)")
    print(f"#define CALIB_A21                             ({ay[0]:.8f}f)")
    print(f"#define CALIB_A22                             ({ay[1]:.8f}f)")
    print(f"#define CALIB_A23                             ({ay[2]:.8f}f)")
    print("")
    print(f"Plane RMS error: {rms(err):.4f} mm")
    print(f"Plane Max error: {float(np.max(np.linalg.norm(err, axis=1))):.4f} mm")


def image_to_camera(rows: list[dict[str, float]], fx: float, fy: float, cx: float, cy: float, depth_scale: float) -> np.ndarray:
    pcs = []
    for r in rows:
        zc = r["depth"] * depth_scale
        xc = ((r["u"] - cx) * zc) / fx
        yc = ((r["v"] - cy) * zc) / fy
        pcs.append([xc, yc, zc])
    return np.array(pcs, dtype=np.float64)


def solve_rigid_transform(pc: np.ndarray, pb: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if pc.shape[0] < 3:
        raise ValueError("3D 标定至少需要 3 个点。")

    pc_mean = np.mean(pc, axis=0)
    pb_mean = np.mean(pb, axis=0)

    pc_centered = pc - pc_mean
    pb_centered = pb - pb_mean

    h = pc_centered.T @ pb_centered
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T

    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T

    t = pb_mean - (r @ pc_mean)
    return r, t


def solve_3d(rows: list[dict[str, float]], fx: float, fy: float, cx: float, cy: float, depth_scale: float) -> None:
    if len(rows) < 3:
        raise ValueError("3D 标定至少需要 3 个点。")

    pc = image_to_camera(rows, fx, fy, cx, cy, depth_scale)
    pb = np.array([[r["x_base"], r["y_base"], r["z_base"]] for r in rows], dtype=np.float64)

    r, t = solve_rigid_transform(pc, pb)
    pred = (r @ pc.T).T + t
    err = pred - pb

    print("/* ---- 3D Calibration Result ---- */")
    print(f"#define CALIB_R11                             ({r[0, 0]:.8f}f)")
    print(f"#define CALIB_R12                             ({r[0, 1]:.8f}f)")
    print(f"#define CALIB_R13                             ({r[0, 2]:.8f}f)")
    print(f"#define CALIB_R21                             ({r[1, 0]:.8f}f)")
    print(f"#define CALIB_R22                             ({r[1, 1]:.8f}f)")
    print(f"#define CALIB_R23                             ({r[1, 2]:.8f}f)")
    print(f"#define CALIB_R31                             ({r[2, 0]:.8f}f)")
    print(f"#define CALIB_R32                             ({r[2, 1]:.8f}f)")
    print(f"#define CALIB_R33                             ({r[2, 2]:.8f}f)")
    print("")
    print(f"#define CALIB_TX_MM                           ({t[0]:.8f}f)")
    print(f"#define CALIB_TY_MM                           ({t[1]:.8f}f)")
    print(f"#define CALIB_TZ_MM                           ({t[2]:.8f}f)")
    print("")
    print(f"3D RMS error: {rms(err):.4f} mm")
    print(f"3D Max error: {float(np.max(np.linalg.norm(err, axis=1))):.4f} mm")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="根据采点 CSV 自动求平面或 3D 标定参数。")
    sub = parser.add_subparsers(dest="mode", required=True)

    plane = sub.add_parser("plane", help="平面标定参数拟合")
    plane.add_argument("--csv", required=True, help="平面采点 CSV 文件路径")

    calib3d = sub.add_parser("3d", help="3D 标定参数拟合")
    calib3d.add_argument("--csv", required=True, help="3D 采点 CSV 文件路径")
    calib3d.add_argument("--fx", required=True, type=float, help="相机 fx")
    calib3d.add_argument("--fy", required=True, type=float, help="相机 fy")
    calib3d.add_argument("--cx", required=True, type=float, help="相机 cx")
    calib3d.add_argument("--cy", required=True, type=float, help="相机 cy")
    calib3d.add_argument("--depth-scale", required=True, type=float, help="depth 到 mm 的比例")

    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.mode == "plane":
        rows = load_csv(Path(args.csv))
        solve_plane(rows)
        return

    if args.mode == "3d":
        rows = load_csv(Path(args.csv))
        solve_3d(rows, args.fx, args.fy, args.cx, args.cy, args.depth_scale)
        return

    raise ValueError(f"不支持的模式: {args.mode}")


if __name__ == "__main__":
    main()
