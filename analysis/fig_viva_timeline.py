"""F10 (viva): the same 4 s blackout replayed healthy (left) and with a +1 deg/s gyro fault (right).
Rows: camera reports + SA alarm, true cross-track error (N / SB / SA), monitor residual D, SA wheel speeds.
Reads results/examples/viva_example.json (written by web/demo.html -> "Save viva example")."""
import json

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common import RESULTS, FIGURES

COL = {"N": "#c0392b", "SB": "#7f8c8d", "SA": "#2c6fb0"}


def main():
    d = json.loads((RESULTS / "examples" / "viva_example.json").read_text())
    W, tF = d["W"], d["tF"]
    fig, axes = plt.subplots(4, 2, figsize=(10, 8.2), sharex=True, gridspec_kw={"height_ratios": [0.7, 2, 1.4, 1.4]})
    for c, fault in enumerate(["none", "gyro1"]):
        sa = d["runs"][f"{fault}_SA"]
        t = np.array(sa["t"])
        ax = axes[0, c]                                     # camera slots and SA alarm
        for ts, ok in sa["cam"]:
            ax.add_patch(plt.Rectangle((ts, 0.55), 0.09, 0.35, color="#cfd6e0" if ok else "#e74c3c", lw=0))
        al = np.array(sa["alarm"]) > 0
        ax.fill_between(t, 0.05, 0.4, where=al, color=COL["SA"], step="post", lw=0)
        ax.set_ylim(0, 1); ax.set_yticks([0.22, 0.72]); ax.set_yticklabels(["SA alarm", "camera"], fontsize=8)
        ax.set_title("healthy sensors" if fault == "none" else "gyro fault +1°/s (0.5 s into the blackout)", fontsize=10)
        ax = axes[1, c]                                     # cross-track error
        for arm in ["N", "SB", "SA"]:
            r = d["runs"][f"{fault}_{arm}"]
            ax.plot(r["t"], r["e_mm"], color=COL[arm], lw=1.6, label={"N": "N never slow", "SB": "SB slow every blackout", "SA": "SA slow on alarm"}[arm])
        ax.axhline(30, color="k", ls="--", lw=0.7); ax.axhline(-30, color="k", ls="--", lw=0.7)
        ax.set_ylabel("cross-track error (mm)", fontsize=8)
        if c == 0:
            ax.legend(fontsize=7, loc="lower left")
        ax = axes[2, c]                                     # monitor residual
        ax.plot(sa["t"], sa["D_deg"], color=COL["SA"], lw=1.2)
        ax.axhline(0.6, color="#c0392b", ls="--", lw=0.8); ax.axhline(-0.6, color="#c0392b", ls="--", lw=0.8)
        ax.set_ylabel("residual D (°)", fontsize=8)
        ax = axes[3, c]                                     # wheel speeds
        ax.plot(sa["t"], sa["uLc"], color="#16a085", ls="--", lw=1, label="left commanded")
        ax.plot(sa["t"], sa["uL"], color="#16a085", lw=1.4, label="left actual")
        ax.plot(sa["t"], sa["uRc"], color="#d35400", ls="--", lw=1, label="right commanded")
        ax.plot(sa["t"], sa["uR"], color="#d35400", lw=1.4, label="right actual")
        ax.set_ylabel("SA wheel speed (m/s)", fontsize=8); ax.set_xlabel("time (s)", fontsize=8)
        if c == 0:
            ax.legend(fontsize=6, ncol=2, loc="lower left")
        for r in range(4):
            a = axes[r, c]
            a.axvspan(W["tOn"], W["tRet"], color="#e74c3c", alpha=0.06, lw=0)
            if fault == "gyro1" and tF is not None:
                a.axvline(tF, color="#8e44ad", ls=":", lw=1)
            a.grid(lw=0.3)
    fig.suptitle("Viva replay: same seed, same 4 s camera blackout from arc entry, R = 1 m, v = 0.5 m/s (synthetic)", fontsize=10)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(FIGURES / f"fig_viva_timeline.{ext}", dpi=200)


if __name__ == "__main__":
    main()
