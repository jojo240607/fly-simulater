#!/usr/bin/env python3
"""读 SIL 仿真 CSV 日志并重渲染 3D 轨迹，用于回归可视化与论文配图。

用法:
    python tools/replay.py sim_out.csv --save trajectory.png
    python tools/replay.py sim_out.csv --no-show   # 仅保存，不弹窗

CSV 由 fly-simulater `--log <path>` 生成，NED 坐标系 (n/e 水平, d 向下为正)。
本脚本把 NED 转回直观的"上为正" Z 轴 (z_plot = -d) 作图。

绘制:
    1) 真值 3D 轨迹 (n,e,-d) + 设定点 (-5m) 参考平面/点
    2) 估计轨迹叠加 (可选, 默认开, 用 --no-est 关)
    3) 末态机体姿态四元数箭头 (可选)

依赖: matplotlib, numpy
"""
import argparse
import csv
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")  # 无 GUI 也可保存
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D  # noqa: E402,F401
import numpy as np  # noqa: E402

COL = {
    "step": 0, "t": 1,
    "true_n": 2, "true_e": 3, "true_d": 4,
    "true_qw": 8, "true_qx": 9, "true_qy": 10, "true_qz": 11,
    "est_n": 17, "est_e": 18, "est_d": 19,
}


def _f(row, k):
    return float(row[COL[k]])


def load(path):
    tn, te, td, en, ee, ed, q = [], [], [], [], [], [], []
    with open(path, newline="") as f:
        r = csv.reader(f)
        if next(r, None) is None:
            raise ValueError(f"{path}: empty")
        for line in r:
            if not line:
                continue
            tn.append(_f(line, "true_n"))
            te.append(_f(line, "true_e"))
            td.append(_f(line, "true_d"))
            en.append(_f(line, "est_n"))
            ee.append(_f(line, "est_e"))
            ed.append(_f(line, "est_d"))
            q.append((_f(line, "true_qw"), _f(line, "true_qx"),
                      _f(line, "true_qy"), _f(line, "true_qz")))
    return {
        "true": np.array([tn, te, td], dtype=float),
        "est": np.array([en, ee, ed], dtype=float),
        "q": np.array(q, dtype=float),
    }


def rotmat_from_quat(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    ap = argparse.ArgumentParser(description="Replay SIL trajectory from CSV.")
    ap.add_argument("log", help="CSV log file")
    ap.add_argument("--save", default=None, help="output image path (png/pdf)")
    ap.add_argument("--no-show", action="store_true", help="do not display (headless)")
    ap.add_argument("--no-est", action="store_true", help="hide estimator trajectory")
    ap.add_argument("--d-set", type=float, default=-5.0, help="NED d setpoint")
    args = ap.parse_args()

    if not os.path.exists(args.log):
        print(f"error: {args.log} not found", file=sys.stderr)
        sys.exit(1)

    data = load(args.log)
    tr = data["true"]
    est = data["est"]
    # NED -> plot (z up = -d)
    tr_p = np.array([tr[0], tr[1], -tr[2]])
    est_p = np.array([est[0], est[1], -est[2]])

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(tr_p[0], tr_p[1], tr_p[2], "-", color="C0", lw=1.5, label="true")
    if not args.no_est:
        ax.plot(est_p[0], est_p[1], est_p[2], "--", color="C1", lw=1.0, alpha=0.7, label="est")
    # setpoint marker (NED d = d_set -> z = -d_set)
    ax.scatter([0], [0], [-args.d_set], c="red", marker="*", s=120, label="setpoint")
    ax.set_xlabel("N (m)")
    ax.set_ylabel("E (m)")
    ax.set_zlabel("Up (m)")
    ax.set_title(f"SIL trajectory: {os.path.basename(args.log)}")
    ax.legend()

    # 末态姿态箭头（机体 -Z 轴在 NED 下指向地面，作图取反）
    if len(data["q"]) > 0:
        q = data["q"][-1]
        R = rotmat_from_quat(q)
        # 机体 z 轴(NED) -> 机体向下; 作图向上轴 = -body_z
        body_down = R[:, 2]
        body_up_plot = -body_down
        tip = tr_p[:, -1]
        ax.quiver(tip[0], tip[1], tip[2],
                  body_up_plot[0], body_up_plot[1], body_up_plot[2],
                  length=0.5, color="green", arrow_length_ratio=0.3, label="body-up")

    ax.set_box_aspect([1, 1, 1])
    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=130)
        print(f"saved: {args.save}")
    if not args.no_show and not args.save:
        plt.show()
    elif not args.no_show and args.save:
        # 也尝试弹窗（有 GUI 时）
        try:
            plt.show()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
