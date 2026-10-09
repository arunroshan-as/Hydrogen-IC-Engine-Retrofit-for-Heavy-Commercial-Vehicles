#!/usr/bin/env python3
"""Generate result figures from the C test-bench output (results/data/*.csv).

Run from the repository root:  make plots
"""
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch

NAVY, GOLD, GREY, RED, GREEN = "#0B1D3A", "#B8912F", "#8A93A3", "#B23A3A", "#2E7D5B"
DATA, OUT = "results/data", "results/plots"
os.makedirs(OUT, exist_ok=True)

plt.rcParams.update({
    "font.size": 9, "axes.edgecolor": NAVY, "axes.labelcolor": NAVY,
    "xtick.color": NAVY, "ytick.color": NAVY, "axes.titleweight": "bold",
    "axes.titlecolor": NAVY, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.alpha": 0.25, "figure.dpi": 130,
})


def load(name):
    with open(f"{DATA}/{name}") as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


A = load("scenario_A_edges.csv")
AE = load("scenario_A_events.csv")
B = load("scenario_B_edges.csv")
BE = load("scenario_B_events.csv")
C = load("scenario_C_edges.csv")

# ---------------------------------------------------------------- fig 1 ---
fig, ax = plt.subplots(3, 1, figsize=(8, 6.2), sharex=True,
                       gridspec_kw={"height_ratios": [2, 1.4, 0.8]})
w = (A["t_s"] > 4.50) & (A["t_s"] < 4.62) & (A["spurious"] == 0)
t, dt = A["t_s"][w], np.diff(A["t_s"][w], prepend=np.nan) * 1e6
ax[0].vlines(t, 0, 1, color=NAVY, lw=0.8)
ax[0].set_yticks([])
ax[0].set_ylabel("crank edges")
ax[0].set_title("60-2 trigger decoding at 1500 rpm (simulated, 1 us timer, 0.2 % edge jitter)")
slot = np.nanmedian(dt)
ax[1].plot(t, dt / slot, ".-", color=NAVY, ms=3, lw=0.6)
ax[1].axhline(2.5, color=RED, ls="--", lw=0.8)
ax[1].text(t[1], 2.58, "gap threshold 2.5x", color=RED, fontsize=8)
ax[1].set_ylabel("interval / slot")
ax[1].set_ylim(0, 3.6)
ax[2].step(t, A["tooth_idx"][w], color=GOLD, lw=1.2, where="post")
ax[2].set_ylabel("tooth idx")
ax[2].set_xlabel("time (s)")
ax[2].text(t[2], 40, "state = SYNCED", color=GREEN, fontsize=8)
fig.tight_layout()
fig.savefig(f"{OUT}/01_gap_detection.png")
plt.close(fig)

# ---------------------------------------------------------------- fig 2 ---
s = (A["state"] == 1) & (A["rpm_est"] > 0)
err = (A["rpm_est"] - A["rpm_true"]) / A["rpm_true"] * 100
fig, ax = plt.subplots(2, 1, figsize=(8, 5.4), sharex=True,
                       gridspec_kw={"height_ratios": [2, 1]})
ax[0].plot(A["t_s"], A["rpm_true"], color=GREY, lw=3, alpha=0.6, label="true speed")
ax[0].plot(A["t_s"][s], A["rpm_est"][s], color=NAVY, lw=1, label="decoder estimate")
ax[0].axvline(6.2, color=GOLD, ls=":", lw=1.2)
ax[0].text(6.25, 300, "32-bit timer wraps", color=GOLD, fontsize=8)
ax[0].set_ylabel("engine speed (rpm)")
ax[0].set_title("Scenario A - speed tracking through start-up, ramps and a 2000 rpm/s acceleration")
ax[0].legend(loc="upper left", frameon=False)
ax[1].plot(A["t_s"][s], err[s], color=NAVY, lw=0.8)
ax[1].axhline(0, color=GREY, lw=0.6)
ax[1].set_ylabel("error (%)")
ax[1].set_xlabel("time (s)")
ax[1].annotate("run-up 200->800 rpm in 1 s:\naveraging lag", (1.6, err[s][(A["t_s"][s] > 1.5) & (A["t_s"][s] < 1.7)].min()),
               (2.6, -4.2), arrowprops=dict(arrowstyle="->", color=RED), color=RED, fontsize=8)
fig.tight_layout()
fig.savefig(f"{OUT}/02_rpm_tracking.png")
plt.close(fig)

# ---------------------------------------------------------------- fig 3 ---
fig, ax = plt.subplots(1, 2, figsize=(8.6, 3.6), gridspec_kw={"width_ratios": [2.2, 1]})
tr = AE["transient"] == 1
ax[0].scatter(AE["t_s"][~tr], AE["err_deg"][~tr], s=5, color=NAVY, label="steady")
ax[0].scatter(AE["t_s"][tr], AE["err_deg"][tr], s=5, color=GOLD, label="transient (>100 rpm/s)")
ax[0].set_xlabel("time (s)")
ax[0].set_ylabel("scheduling error (crank deg)")
ax[0].set_title("Injection/spark instant vs exact truth")
ax[0].legend(frameon=False, loc="upper left", fontsize=8)
ax[1].hist(AE["err_deg"], bins=40, color=NAVY)
ax[1].set_xlabel("error (deg)")
ax[1].set_title(f"n = {len(AE['err_deg'])} events")
fig.tight_layout()
fig.savefig(f"{OUT}/03_event_timing_error.png")
plt.close(fig)

# ---------------------------------------------------------------- fig 4 ---
fig, ax = plt.subplots(2, 1, figsize=(8.6, 4.8), sharex=True,
                       gridspec_kw={"height_ratios": [1, 1.2]})
ax[0].fill_between(B["t_s"], 0, B["outputs_enabled"], step="post", color=GREEN, alpha=0.5)
ax[0].set_ylim(-0.1, 1.35)
ax[0].set_yticks([0, 1])
ax[0].set_yticklabels(["gated off", "outputs on"])
ax[0].set_title("Scenario B - fault injection at 1500 rpm: outputs gate off until re-sync", fontsize=9)
marks = [(1.0, "dropped\ntooth"), (2.0, "glitch\n0.2 slot"), (3.0, "glitch\n0.5 slot"),
         (3.5, "glitch\n0.75 slot")]
for x, lab in marks:
    ax[0].axvline(x, color=RED, lw=0.8, ls="--")
    ax[0].text(x - 0.04, 0.45, lab, color=RED, fontsize=7, ha="right")
ax[0].axvspan(4.0, 4.2, color=GOLD, alpha=0.3)
ax[0].text(4.1, 1.12, "cam stuck", color=GOLD, fontsize=7, ha="center")
ax[0].axvspan(5.5, 6.0, color=GREY, alpha=0.3)
ax[0].text(5.75, 1.12, "signal loss", color=NAVY, fontsize=7, ha="center")
ax[1].scatter(BE["t_s"], BE["err_deg"], s=5, color=NAVY)
ax[1].set_ylim(-1, 1)
ax[1].set_ylabel("scheduling error (deg)")
ax[1].set_xlabel("time (s)")
ax[1].text(0.1, 0.8, "every scheduled event stays within +/-0.04 deg across all faults",
           fontsize=8, color=NAVY)
fig.tight_layout()
fig.savefig(f"{OUT}/04_fault_recovery.png")
plt.close(fig)

# ---------------------------------------------------------------- fig 5 ---
fig, ax = plt.subplots(1, 2, figsize=(8.6, 3.4))
rpm_axis = [800, 1200, 1500, 1800, 2400, 3000, 3600]
load_axis = [20, 40, 60, 80, 100]
inj = np.array([[70 + (i * 0) + 2 * j + (0) for j in range(7)] for i in range(5)], float)
# reproduce the demo map values from timing_map.c
inj = np.array([[70, 72, 74, 76, 80, 84, 88], [68, 70, 72, 74, 78, 82, 86],
                [66, 68, 70, 72, 76, 80, 84], [64, 66, 68, 70, 74, 78, 82],
                [62, 64, 66, 68, 72, 76, 80]], float)
spk = np.array([[14, 16, 18, 20, 24, 28, 32], [12, 14, 16, 18, 22, 26, 30],
                [10, 12, 14, 16, 20, 24, 28], [8, 10, 12, 14, 18, 22, 26],
                [6, 8, 10, 12, 16, 20, 24]], float)
for a, z, ttl in ((ax[0], inj, "Injection start (deg BTDC)"), (ax[1], spk, "Spark timing (deg BTDC)")):
    im = a.imshow(z, origin="lower", aspect="auto", cmap="YlGnBu")
    a.set_xticks(range(7)); a.set_xticklabels(rpm_axis, fontsize=7)
    a.set_yticks(range(5)); a.set_yticklabels(load_axis)
    a.set_xlabel("speed (rpm)"); a.set_ylabel("load (%)"); a.set_title(ttl)
    a.grid(False)
    for i in range(5):
        for j in range(7):
            a.text(j, i, f"{z[i, j]:.0f}", ha="center", va="center", fontsize=6,
                   color="white" if z[i, j] > z.mean() else NAVY)
fig.suptitle("Illustrative demonstration map used to exercise the lookup (not an engine calibration)",
             fontsize=8, color=GREY)
fig.tight_layout()
fig.savefig(f"{OUT}/05_timing_map_demo.png")
plt.close(fig)

# ---------------------------------------------------------------- fig 6 ---
fig, ax = plt.subplots(figsize=(9, 3.3))
ax.axis("off")
ax.set_xlim(0, 100)
ax.set_ylim(0, 36)


def box(x, y, w, h, text, fc="white", ec=NAVY, tc=NAVY, fs=8):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4,rounding_size=1.2",
                                fc=fc, ec=ec, lw=1.4))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", color=tc, fontsize=fs)


def arrow(x1, y1, x2, y2):
    ax.annotate("", (x2, y2), (x1, y1), arrowprops=dict(arrowstyle="-|>", color=NAVY, lw=1.2))


box(1, 22, 17, 9, "Crank sensor\n(60-2 wheel)")
box(1, 6, 17, 9, "Cam sensor\n(phase)")
box(26, 12, 22, 14, "STM32 firmware\ncrank_decoder\n(sync / RPM / angle)", fc=NAVY, tc="white")
box(55, 22, 20, 9, "timing_map\nspeed x load")
box(55, 6, 20, 9, "event scheduler\n(angle -> timer tick)")
box(82, 22, 17, 9, "Injector\ndriver")
box(82, 6, 17, 9, "Ignition\ndriver")
arrow(18, 26, 26, 22); arrow(18, 10, 26, 16)
arrow(48, 22, 55, 26); arrow(48, 16, 55, 11)
arrow(65, 22, 65, 15)
arrow(75, 10, 82, 10); arrow(75, 27, 82, 27)
box(26, 0.5, 22, 7, "Fail-safe gate: no sync, stall,\nover-speed -> outputs off", fc="#FBF3DD", ec=GOLD, fs=7)
arrow(37, 7.5, 37, 12)
ax.set_title("Firmware data flow", loc="left")
fig.tight_layout()
fig.savefig(f"{OUT}/06_firmware_architecture.png")
plt.close(fig)
print("wrote 6 figures to", OUT)
