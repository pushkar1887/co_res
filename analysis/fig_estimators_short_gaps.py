"""F5: short gaps (Part A). HOLD vs DR vs EKF: tracking excess, estimate error, steering jump.
Median over 20 seeds, error bars = interquartile range. No-fault runs (slip runs are in the CSV)."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common import load, FIGURES

ARMS = [("hold", "HOLD", "#9a9a9a"), ("dr", "DR", "#d08c2c"), ("ekf", "EKF", "#2c6fb0")]
CELLS = [(p, ph) for p in ["iso8", "2x4", "burst8"] for ph in ["entry", "mid", "exit"]]


def main():
    df = load("a_estimators_short_gaps")
    df = df[df.fault == "none"].copy()
    df["excess_mm"], df["estPeak_mm"] = df.excess * 1e3, df.estPeak * 1e3
    panels = [("excess_mm", "peak excess cross-track (mm)", ARMS), ("estPeak_mm", "peak estimate error (mm)", ARMS[1:]),
              ("steerJump", "steering jump at camera return (rad/s)", ARMS)]
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 7.0), sharex=True)
    x = np.arange(len(CELLS))
    for ax, (metric, label, arms) in zip(axes, panels):
        w = 0.8 / len(arms)
        for k, (arm, name, col) in enumerate(arms):
            q = [df[(df.pattern == p) & (df.phase == ph) & (df.arm == arm)][metric].quantile([0.25, 0.5, 0.75]).values for p, ph in CELLS]
            q = np.array(q)
            ax.bar(x + (k - (len(arms) - 1) / 2) * w, q[:, 1], w, color=col, label=name,
                   yerr=[q[:, 1] - q[:, 0], q[:, 2] - q[:, 1]], capsize=2, error_kw={"lw": 0.7})
        ax.set_ylabel(label, fontsize=8)
        ax.grid(axis="y", lw=0.3)
        ax.axhline(0, color="k", lw=0.5)
    axes[0].legend(fontsize=8, ncol=3)
    axes[-1].set_xticks(x)
    axes[-1].set_xticklabels([f"{p}\n{ph}" for p, ph in CELLS], fontsize=7)
    fig.suptitle("Part A: short camera gaps, R = 0.55 m, v = 0.2 m/s (synthetic, 20 seeds)", fontsize=9)
    fig.tight_layout()
    FIGURES.mkdir(exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(FIGURES / f"fig_estimators_short_gaps.{ext}", dpi=200)


if __name__ == "__main__":
    main()
