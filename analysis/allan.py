"""V6: Allan deviation of the synthetic stationary gyro log (PLAN §9, figure F11).

Checks that the generator reproduces the injected noise model:
  angle random walk N within 10% (fit on tau = 0.1-1 s, where white noise dominates)
  bias-instability floor (min ADEV over tau = 10-270 s) within 30% of the analytic white + Gauss-Markov curve.
Reads results/imu_static/; writes allan.csv, allan_check.json and figures/fig_allan.{png,pdf}.
"""
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
IMU = ROOT / "results" / "imu_static"
FIG = ROOT / "figures"
SKIP_S = 900.0          # drop the warm-up interval (tau_w = 120 s)


def overlapping_adev(w, fs, taus):
    theta = np.concatenate([[0.0], np.cumsum(w) / fs])
    out = []
    for tau in taus:
        m = int(round(tau * fs))
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        out.append(np.sqrt(np.mean(d ** 2) / (2 * tau ** 2)))
    return np.array(out)


def model_adev(taus, N, sig, T):
    u = taus / T
    gm = (sig ** 2 * T ** 2 / taus ** 2) * (2 * u - 3 + 4 * np.exp(-u) - np.exp(-2 * u))
    return np.sqrt(N ** 2 / taus + gm)


def main():
    meta = json.loads((IMU / "meta.json").read_text())
    fs, inj = meta["fs"], meta["injected"]
    w = np.loadtxt(IMU / "gyro_static_1h.csv", skiprows=1)
    w = w[int(SKIP_S * fs):]
    taus = np.unique(np.round(np.logspace(np.log10(2 / fs), np.log10(len(w) / fs / 10), 60) * fs) / fs)
    adev = overlapping_adev(w, fs, taus)
    model = model_adev(taus, inj["ARW"], inj["GMsig"], inj["GMtau"])

    white = (taus >= 0.1) & (taus <= 1.0)
    N_hat = float(np.median(adev[white] * np.sqrt(taus[white])))
    floor = (taus >= 10) & (taus <= 270)
    fl_meas, fl_model = float(adev[floor].min()), float(model[floor].min())
    rel_N, rel_fl = abs(N_hat - inj["ARW"]) / inj["ARW"], abs(fl_meas - fl_model) / fl_model
    d2r = np.pi / 180
    check = {
        "ARW_hat_deg_per_sqrt_s": N_hat / d2r, "ARW_true_deg_per_sqrt_s": inj["ARW"] / d2r, "ARW_rel_err": rel_N,
        "floor_meas_deg_s": fl_meas / d2r, "floor_model_deg_s": fl_model / d2r, "floor_rel_err": rel_fl,
        "pass": bool(rel_N < 0.10 and rel_fl < 0.30), "samples_used": int(len(w)), "skipped_warmup_s": SKIP_S,
        "note": "synthetic data; verifies the generator and the Allan pipeline, not a real IMU",
    }
    (IMU / "allan_check.json").write_text(json.dumps(check, indent=2))
    np.savetxt(IMU / "allan.csv", np.column_stack([taus, adev, model]), delimiter=",",
               header="tau_s,adev_meas_rad_s,adev_model_rad_s", comments="")

    FIG.mkdir(exist_ok=True)
    fig, ax = plt.subplots(figsize=(4.6, 3.4))
    ax.loglog(taus, adev / d2r, "o", ms=3, label="synthetic log")
    ax.loglog(taus, model / d2r, "-", lw=1.2, label="injected model")
    ax.set_xlabel("averaging time τ (s)")
    ax.set_ylabel("Allan deviation (°/s)")
    ax.set_title(f"ARW {N_hat / d2r:.4f}°/√s (inj. {inj['ARW'] / d2r:.3f})", fontsize=9)
    ax.grid(True, which="both", lw=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(FIG / f"fig_allan.{ext}", dpi=200)
    print(json.dumps(check, indent=2))


if __name__ == "__main__":
    main()
