#!/usr/bin/env python3
"""
plot_pd_tune.py — Plot PD tuning logs produced by arctos_pd_tune.py
--------------------------------------------------------------------
Usage:
  python3 plot_pd_tune.py ~/arctosgui/pd_tune_J1_step_20260409_120000.csv

Shows three panels:
  1. Position error over time for the tuned joint
  2. Velocity command (qdot_cmd) over time for the tuned joint
  3. All-joint errors (faint) so you can see cross-coupling

Requires: matplotlib, numpy  (pip install matplotlib numpy)
"""

import sys
import csv
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def load_log(path):
    times, errors, qdots = [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time_s"]))
            errors.append([float(row[f"error_j{i+1}"]) for i in range(6)])
            qdots.append([float(row[f"qdot_j{i+1}"]) for i in range(6)])
    return (
        np.array(times),
        np.array(errors),   # shape (N, 6)
        np.array(qdots),    # shape (N, 6)
    )


def infer_joint(path):
    """Try to extract joint number from filename, default to 0."""
    basename = os.path.basename(path)
    for part in basename.split("_"):
        if part.startswith("J") and part[1:].isdigit():
            return int(part[1:]) - 1   # 0-based
    return 0


def plot(path):
    times, errors, qdots = load_log(path)
    j = infer_joint(path)
    title_base = os.path.basename(path)

    fig = plt.figure(figsize=(13, 9))
    fig.suptitle(f"PD Tuning Log — J{j+1}\n{title_base}", fontsize=11)
    gs = gridspec.GridSpec(3, 1, hspace=0.45)

    # ── Panel 1: position error of tuned joint ──
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(times, np.degrees(errors[:, j]), color="tab:red", lw=1.2, label=f"J{j+1} error")
    ax1.axhline(0, color="black", lw=0.6, ls="--")
    ax1.set_ylabel("Position error (deg)")
    ax1.set_title(f"J{j+1} Position Error")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)

    # ── Panel 2: velocity command of tuned joint ──
    ax2 = fig.add_subplot(gs[1])
    ax2.plot(times, np.degrees(qdots[:, j]), color="tab:blue", lw=1.2, label=f"J{j+1} qdot_cmd")
    ax2.axhline(0, color="black", lw=0.6, ls="--")
    ax2.set_ylabel("Vel command (deg/s)")
    ax2.set_title(f"J{j+1} Velocity Command")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    # ── Panel 3: all-joint errors (cross-coupling check) ──
    ax3 = fig.add_subplot(gs[2])
    colors = plt.cm.tab10.colors
    for i in range(6):
        alpha = 1.0 if i == j else 0.35
        lw    = 1.4 if i == j else 0.8
        ax3.plot(
            times, np.degrees(errors[:, i]),
            color=colors[i], lw=lw, alpha=alpha, label=f"J{i+1}"
        )
    ax3.axhline(0, color="black", lw=0.6, ls="--")
    ax3.set_ylabel("Error (deg)")
    ax3.set_xlabel("Time (s)")
    ax3.set_title("All-Joint Errors (cross-coupling check)")
    ax3.legend(loc="upper right", fontsize=7, ncol=3)
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()

    # Print a quick summary
    e = np.degrees(errors[:, j])
    print(f"\n── J{j+1} Summary ──────────────────────────")
    print(f"  Peak error   : {e.max():.3f}°  /  {e.min():.3f}°")
    print(f"  RMS error    : {np.sqrt(np.mean(e**2)):.3f}°")
    print(f"  Final error  : {e[-1]:.4f}°")
    print(f"  Samples      : {len(times)}")
    if len(times) > 1:
        fs = 1.0 / np.median(np.diff(times))
        print(f"  Approx rate  : {fs:.1f} Hz")
    print()

    out_path = path.replace(".csv", ".png")
    print(f"Plot saved: {out_path}")
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 plot_pd_tune.py <log.csv>")
        sys.exit(1)
    plot(sys.argv[1])
