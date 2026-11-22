"""Figures for Parts B, C, D (PLAN §10): F6 fig_fault_map, F3 fig_detection_regret, F8 fig_scaling, F9 fig_burstiness.
Predictions are the straight-path closed forms of PLAN §6 (alpha = 0.3, v = 0.5 m/s, exposure T = 3.5 s, h = 0.6 deg, T_w = 2 s)."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common import load, FIGURES

D2R = np.pi / 180
COL = {"N": "#c0392b", "SB": "#7f8c8d", "SA": "#2c6fb0", "SO": "#27ae60", "dr": "#d08c2c", "ekf": "#2c6fb0", "hold": "#9a9a9a"}


def save(fig, name):
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(FIGURES / f"{name}.{ext}", dpi=200)
    plt.close(fig)


def fault_map():
    df = load("b_fault_map")
    df = df[df.Tb == 4].copy()
    df["dev_mm"] = df.devPeak * 1e3
    faults = ["none", "slip", "diff2", "gyro0.5", "gyro1", "scale3"]
    onsets = ["pre", "entry", "mid", "exit"]
    arms = ["dr", "N", "SB", "SA", "SO"]
    fig, axes = plt.subplots(1, len(arms), figsize=(11, 3.3), sharey=True)
    for ax, arm in zip(axes, arms):
        m = np.array([[df[(df.fault == f) & (df.phase == o) & (df.arm == arm)].dev_mm.median() for o in onsets] for f in faults])
        br = np.array([[df[(df.fault == f) & (df.phase == o) & (df.arm == arm)].breach.mean() for o in onsets] for f in faults])
        im = ax.imshow(m, vmin=0, vmax=60, cmap="magma_r")
        for i in range(len(faults)):
            for j in range(len(onsets)):
                ax.text(j, i, f"{m[i, j]:.0f}" + ("*" if br[i, j] >= 0.5 else ""), ha="center", va="center", fontsize=7,
                        color="white" if m[i, j] > 35 else "black")
        ax.set_xticks(range(len(onsets)))
        ax.set_xticklabels(onsets, fontsize=7)
        ax.set_title({"dr": "DR", "N": "EKF-N", "SB": "EKF-SB", "SA": "EKF-SA", "SO": "EKF-SO"}[arm], fontsize=9)
    axes[0].set_yticks(range(len(faults)))
    axes[0].set_yticklabels(["healthy", "slip 10%", "wheels ±1%", "gyro 0.5°/s", "gyro 1°/s", "wheels +3%"], fontsize=7)
    fig.colorbar(im, ax=axes, shrink=0.8, label="median path-aligned deviation (mm)")
    fig.suptitle("Part B: 4 s blackout, fault × onset (10 seeds). * = corridor breached in ≥ 50% of runs", fontsize=9)
    for ext in ("png", "pdf"):
        fig.savefig(FIGURES / f"fig_fault_map.{ext}", dpi=200, bbox_inches="tight")
    plt.close(fig)


def detection_regret():
    df = load("c_scaling")
    df = df[df.exp == "C2"].copy()
    df["b"] = df.fault.str.replace("gyro", "").astype(float)
    df["dev_mm"] = df.devPeak * 1e3
    bs = np.sort(df.b.unique())
    bb = np.linspace(0.15, 2.05, 200)
    v, a, T, h, Tw = 0.5, 0.3, 3.5, 0.6, 2.0
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8.4, 3.4))
    for arm in ["N", "SO", "SA"]:
        q = np.array([df[(df.b == b) & (df.arm == arm)].dev_mm.quantile([0.25, 0.5, 0.75]).values for b in bs])
        ax1.errorbar(bs, q[:, 1], yerr=[q[:, 1] - q[:, 0], q[:, 2] - q[:, 1]], fmt="o-", ms=4, color=COL[arm], label=f"{arm} (sim)", capsize=2)
    eN = 0.5 * v * bb * D2R * T ** 2 * 1e3
    td = np.minimum(h / bb, T)
    eSA = np.where(bb > h / Tw, 0.5 * v * bb * D2R * (a * T ** 2 + (1 - a) * td ** 2) * 1e3, eN)
    ax1.plot(bb, eN, "--", color=COL["N"], lw=1, label="N (§6.7)")
    ax1.plot(bb, a * eN, "--", color=COL["SO"], lw=1, label="SO (§6.7)")
    ax1.plot(bb, eSA, ":", color=COL["SA"], lw=1.2, label="SA (§6.7)")
    ax1.axhline(30, color="k", lw=0.6, ls="--")
    ax1.set_xlabel("gyro fault b (°/s)")
    ax1.set_ylabel("path-aligned deviation (mm)")
    ax1.legend(fontsize=7)
    ax1.grid(lw=0.3)
    sa = df[df.arm == "SA"]
    q = np.array([sa[sa.b == b].alarmDelay.quantile([0.25, 0.5, 0.75]).values for b in bs])
    ax2.errorbar(bs, q[:, 1], yerr=[q[:, 1] - q[:, 0], q[:, 2] - q[:, 1]], fmt="o", color=COL["SA"], capsize=2, label="SA (sim)")
    ax2.plot(bb, h / bb, "--", color="k", lw=1, label="$t_d = h/b$ (§6.6)")
    ax2.axvline(h / Tw, color="grey", ls=":", lw=1)
    ax2.text(h / Tw + 0.02, 3.2, "$b_{min}=h/T_w$", fontsize=7)
    ax2.set_ylim(0, 3.6)
    ax2.set_xlabel("gyro fault b (°/s)")
    ax2.set_ylabel("detection delay (s)")
    ax2.legend(fontsize=7)
    ax2.grid(lw=0.3)
    fig.suptitle("C2: 4 s blackout from arc entry, R = 1 m, v = 0.5 m/s (10 seeds)", fontsize=9)
    save(fig, "fig_detection_regret")


def scaling():
    df = load("c_scaling")
    df = df[df.exp == "C1"].copy()
    df["dev_mm"] = df.devPeak * 1e3
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8.4, 3.3))
    for R, mk in zip([0.55, 1.0, 1.5], ["o", "s", "^"]):
        for ax, f in [(ax1, "gyro1"), (ax2, "scale3")]:
            s = df[(df.R == R) & (df.fault == f) & (df.arm == "N")]
            med = s.groupby("v").dev_mm.median()
            ax.plot(med.index, med.values, marker=mk, label=f"R = {R} m", color="#444")
    vv = np.linspace(0.3, 0.7, 10)
    ax1.plot(vv, 0.5 * vv * 1.0 * D2R * 3.5 ** 2 * 1e3, "--", color=COL["N"], label="½vbT² (§6.3)")
    ax1.set_title("EKF-N, gyro +1°/s: ∝ v, independent of R", fontsize=8)
    ax2.set_title("EKF-N, wheels +3%: depends on R and v", fontsize=8)
    for ax in (ax1, ax2):
        ax.set_xlabel("speed v (m/s)")
        ax.set_ylabel("path-aligned deviation (mm)")
        ax.grid(lw=0.3)
        ax.legend(fontsize=7)
    fig.suptitle("C1: 4 s blackout from arc entry (10 seeds, L_d = 0.30 m)", fontsize=9)
    save(fig, "fig_scaling")


def burstiness():
    df = load("d_burstiness")
    df["peak_mm"] = df.peak * 1e3
    fig, ax = plt.subplots(figsize=(4.8, 3.3))
    bs = sorted(df.meanBurst.unique())
    for arm, name in [("hold", "HOLD"), ("dr", "DR"), ("ekf", "EKF")]:
        q = np.array([df[(df.meanBurst == b) & (df.arm == arm)].peak_mm.quantile([0.25, 0.5, 0.75]).values for b in bs])
        ax.errorbar(bs, q[:, 1], yerr=[q[:, 1] - q[:, 0], q[:, 2] - q[:, 1]], fmt="o-", color=COL[arm], label=name, capsize=2)
    real = df[df.arm == "ekf"].groupby("meanBurst").agg(rate=("lostSlots", "sum"), tot=("totalSlots", "sum"), mb=("meanBurstRealized", "median"))
    for b in bs:
        ax.text(b, 2, f"real: {100 * real.loc[b, 'rate'] / real.loc[b, 'tot']:.0f}%, L̄ {real.loc[b, 'mb']:.1f}", fontsize=6, ha="center", rotation=90)
    ax.axvspan(12, 20, color="red", alpha=0.08)
    ax.text(16, 60, "under-sampled\n(see §20)", fontsize=7, ha="center")
    ax.axhline(30, color="k", lw=0.6, ls="--")
    ax.set_xscale("log", base=2)
    ax.set_xticks(bs)
    ax.set_xticklabels([str(b) for b in bs])
    ax.set_xlabel("configured mean burst length (reports), 10% loss")
    ax.set_ylabel("peak cross-track over route (mm)")
    ax.legend(fontsize=7)
    ax.grid(lw=0.3)
    ax.set_title("D: Gilbert channel, R = 1 m, v = 0.5 m/s, 30 seeds", fontsize=9)
    save(fig, "fig_burstiness")


if __name__ == "__main__":
    fault_map()
    detection_regret()
    scaling()
    burstiness()
    print("figures written")
