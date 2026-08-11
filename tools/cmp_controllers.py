#!/usr/bin/env python3
"""对比多个 SIL 仿真 CSV 日志，量化控制律表现差异。

用法:
    python tools/cmp_controllers.py sim_out.csv
    python tools/cmp_controllers.py logs/pid.csv logs/indi.csv logs/lqr.csv --labels pid indi lqr

输入 CSV 由 fly-simulater `--log <path>` 生成（28 列，NED: n/e/d，d 向下为正）。
设定点高度在 NED 下为 d_setpoint（默认 -5m，可用 --d-set 覆盖）。

输出指标（逐文件）:
    - RMS_dz     : sqrt(mean((true_d - d_set)^2))   高度跟踪误差 (m)
    - RMS_horiz  : sqrt(mean((n^2 + e^2)))          水平偏离原点 (m)
    - RMS_cmd    : sqrt(mean(mean(motor^2)))         平均指令幅度 (归一化 0..1)
    - final_dz   : 末步 true_d - d_set              末态高度残差 (m)
    - drift_e    : 末步水平位移范数 (m)
    - steps      : 总行数
打印对齐表格，方便横向对比 PID / INDI / LQR。

注意：NED 下 d 向下为正，故"高于设定点"= true_d 比 d_set 更负（如 -4.9 比 -5.0 高 0.1m）。
"""
import argparse
import csv
import math
import os
import sys

COL = {
    "step": 0, "t": 1,
    # true NED pos/vel
    "true_n": 2, "true_e": 3, "true_d": 4,
    "true_vn": 5, "true_ve": 6, "true_vd": 7,
    # est NED pos/vel
    "est_n": 17, "est_e": 18, "est_d": 19,
    "est_vn": 20, "est_ve": 21, "est_vd": 22,
    # motors
    "m0": 27, "m1": 28, "m2": 29, "m3": 30,
}


def _f(row, key):
    return float(row[COL[key]])


def analyze(path, d_set):
    rows = []
    with open(path, newline="") as f:
        r = csv.reader(f)
        header = next(r, None)
        if header is None:
            raise ValueError(f"{path}: empty file")
        for line in r:
            if not line:
                continue
            rows.append(line)
    if not rows:
        raise ValueError(f"{path}: no data rows")

    n = len(rows)
    s_dz = 0.0
    s_horiz = 0.0
    s_cmd = 0.0
    for row in rows:
        td = _f(row, "true_d")
        tn = _f(row, "true_n")
        te = _f(row, "true_e")
        dz = td - d_set
        s_dz += dz * dz
        s_horiz += tn * tn + te * te
        mavg = (_f(row, "m0") + _f(row, "m1") + _f(row, "m2") + _f(row, "m3")) / 4.0
        s_cmd += mavg * mavg

    last = rows[-1]
    final_dz = _f(last, "true_d") - d_set
    drift_e = math.hypot(_f(last, "true_n"), _f(last, "true_e"))
    return {
        "steps": n,
        "RMS_dz": math.sqrt(s_dz / n),
        "RMS_horiz": math.sqrt(s_horiz / n),
        "RMS_cmd": math.sqrt(s_cmd / n),
        "final_dz": final_dz,
        "drift_e": drift_e,
    }


def main():
    ap = argparse.ArgumentParser(description="Compare SIL controller CSV logs.")
    ap.add_argument("logs", nargs="+", help="CSV log files")
    ap.add_argument("--labels", nargs="*", help="per-log labels (default: basename)")
    ap.add_argument("--d-set", type=float, default=-5.0, help="NED d setpoint (default -5.0)")
    args = ap.parse_args()

    labels = args.labels if args.labels else [os.path.splitext(os.path.basename(p))[0] for p in args.logs]
    if len(labels) != len(args.logs):
        print(f"error: --labels count ({len(labels)}) != logs count ({len(args.logs)})", file=sys.stderr)
        sys.exit(2)

    results = []
    for p, lab in zip(args.logs, labels):
        try:
            res = analyze(p, args.d_set)
        except Exception as ex:  # noqa: BLE001
            print(f"error: {p}: {ex}", file=sys.stderr)
            sys.exit(1)
        results.append((lab, res))

    # 打印表格
    hdr = ("label", "steps", "RMS_dz(m)", "RMS_horiz(m)", "RMS_cmd", "final_dz(m)", "drift_e(m)")
    rows_txt = [hdr]
    for lab, res in results:
        rows_txt.append((
            lab,
            str(res["steps"]),
            f"{res['RMS_dz']:.4f}",
            f"{res['RMS_horiz']:.4f}",
            f"{res['RMS_cmd']:.4f}",
            f"{res['final_dz']:.4f}",
            f"{res['drift_e']:.4f}",
        ))
    widths = [max(len(r[i]) for r in rows_txt) for i in range(len(hdr))]
    fmt = "  ".join(f"{{:<{w}}}" for w in widths)
    print(fmt.format(*hdr))
    for r in rows_txt[1:]:
        print(fmt.format(*r))

    # 提示解读
    print()
    print("解读 (NED: d 向下为正):")
    print("  RMS_dz 越小 = 高度跟踪越好; final_dz<0 表示停在设定点上方。")
    print("  RMS_horiz 越小 = 越居中; drift_e 末态水平偏移。")
    print("  RMS_cmd 反映平均油门/操纵幅度 (0..1)。")


if __name__ == "__main__":
    main()
