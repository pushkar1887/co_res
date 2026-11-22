"""F4: healthy max |D| on calibration seeds vs the analytic threshold window (PLAN §6.8)."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common import load, FIGURES


def main():
    df = load("cal_monitor")
    d = np.degrees(df.maxDgated)
    fig, ax = plt.subplots(figsize=(5.0, 3.2))
    ax.hist(d, bins=30, color="#2c6fb0", alpha=0.8, label=f"monitor v2, healthy (n = {len(d)})")
    for x, lab, ls in [(0.33, "quantization bound 0.33°", ":"), (0.42, "$h_{min}$ 0.42°", "--"), (0.6, "h = 0.6° (frozen)", "-"),
                       (0.76, "v1 max 0.76° (K2)", "-."), (0.86, "$h_{max}$ 0.86°", "--")]:
        ax.axvline(x, color="k", ls=ls, lw=1)
        ax.text(x + 0.005, ax.get_ylim()[1] * 0.9, lab, rotation=90, fontsize=7, va="top")
    ax.set_xlabel("max |D| during healthy blackouts (°)")
    ax.set_ylabel("runs")
    ax.set_xlim(0, 0.95)
    ax.legend(fontsize=7, loc="center right")
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(FIGURES / f"fig_threshold_window.{ext}", dpi=200)


if __name__ == "__main__":
    main()
