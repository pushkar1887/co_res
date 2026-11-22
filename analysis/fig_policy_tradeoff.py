"""F2 (main): peak excess error vs time lost for N / SB / SA / SO (B0, 4 s blackouts).
Each dot = one seed. Large marker = median. Crosses = PLAN §6.9 closed-form predictions (drift)."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common import load, FIGURES

ARMS = [("N", "N (never slow)", "#c0392b"), ("SB", "SB (slow every blackout)", "#7f8c8d"),
        ("SA", "SA (slow on alarm)", "#2c6fb0"), ("SO", "SO (oracle)", "#27ae60")]
PRED = {("gyro1", "N"): (0.0, 53.5), ("gyro1", "SB"): (2.59, 16.0), ("gyro1", "SA"): (2.03, 17.1), ("gyro1", "SO"): (2.45, 16.0),
        ("none", "SB"): (2.59, None), ("none", "SA"): (0.0, None), ("none", "SO"): (0.0, None), ("none", "N"): (0.0, None)}


def main():
    df = load("b0_policy_primary")
    df = df[df.Tb == 4].copy()
    df["excess_mm"] = df.excess * 1e3
    fig, axes = plt.subplots(2, 2, figsize=(7.4, 6.0), sharex=True)
    for r, phase in enumerate(["entry", "mid"]):
        for c, fault in enumerate(["none", "gyro1"]):
            ax = axes[r, c]
            for arm, name, col in ARMS:
                s = df[(df.phase == phase) & (df.fault == fault) & (df.arm == arm)]
                jit = (np.random.default_rng(1).random(len(s)) - 0.5) * 0.04
                ax.scatter(s.timeLost + jit, s.excess_mm, s=8, color=col, alpha=0.35, lw=0)
                ax.scatter(s.timeLost.median(), s.excess_mm.median(), s=70, color=col, edgecolor="k", lw=0.6, label=name, zorder=3)
                px, py = PRED[(fault, arm)]
                if py is not None:
                    ax.scatter(px, py, marker="x", s=40, color=col, lw=1.5, zorder=4)
            ax.axhline(30, color="k", ls="--", lw=0.7)
            ax.text(0.02, 30.5, "corridor 30 mm", fontsize=7)
            ax.set_title(f"onset {phase}, {'healthy' if fault == 'none' else '+1°/s gyro fault'}", fontsize=9)
            ax.grid(lw=0.3)
            if c == 0:
                ax.set_ylabel("peak excess cross-track (mm)", fontsize=8)
            if r == 1:
                ax.set_xlabel("time lost vs no-loss run (s)", fontsize=8)
    axes[0, 0].legend(fontsize=7, loc="upper right")
    fig.suptitle("B0: 4 s camera blackout, R = 1 m, v = 0.5 m/s, 30 seeds (synthetic). × = §6.9 prediction", fontsize=9)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(FIGURES / f"fig_policy_tradeoff.{ext}", dpi=200)


if __name__ == "__main__":
    main()
