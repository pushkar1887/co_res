import math, statistics
from proto2_ekf_faults_monitor import run, peak
R = 1.0
CASES = {"no sensor fault": dict(), "gyro drift +1deg/s in blackout": dict(bias_late=math.radians(1.0)),
         "wheels both +3%": dict(epsR=0.03, epsL=0.03)}
print(f"{'case':>32} {'phase':>6} | excess peak cm, median of 3 seeds:  enc    ekf    mon | mon time lost")
for cname, ck in CASES.items():
    for ph in ("entry", "mid", "exit"):
        res = {mo: [] for mo in ("enc", "ekf", "mon")}; tl = []
        for seed in (1, 2, 3):
            base, tb, (L1, La) = run(R, "ekf", seed=seed, **{k: v for k, v in ck.items() if k != "bias_late"})
            tc = lambda sv: next(t for t, s, _, _ in base if s >= sv)
            tcen = {"entry": tc(L1), "mid": tc(L1 + La / 2), "exit": tc(L1 + La)}[ph]
            t0, L = tcen - 2.0, 4.0
            for mo in res:
                ref, _, _ = run(R, mo, seed=seed, **{k: v for k, v in ck.items() if k != "bias_late"})
                lg, td, _ = run(R, mo, out=(t0, L), seed=seed, **ck)
                res[mo].append(100 * (peak(lg, t0, t0 + L + 3) - peak(ref, t0, t0 + L + 3)))
                if mo == "mon": tl.append((td or 40) - tb)
        print(f"{cname:>32} {ph:>6} |" + " " * 37 + " ".join(f"{statistics.median(v):6.2f}" for v in res.values()) + f" | {statistics.median(tl):+.2f}s")
