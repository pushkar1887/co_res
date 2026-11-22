"""Pre-registered tests of PLAN §2 / §8.

python analysis/hypothesis_tests.py A    -> H1a, H1b, H1c  (results/a_estimators_short_gaps/tests.csv)
python analysis/hypothesis_tests.py CAL  -> threshold freeze rule (results/cal_monitor/cal_decision.json)
python analysis/hypothesis_tests.py B0   -> P1-P4          (results/b0_policy_primary/tests.csv)
Units in the outputs: mm and s.
"""
import json
import sys

import numpy as np
import pandas as pd

from common import RESULTS, load, paired, bootstrap_ci, signflip_p, tost, holm, clopper_pearson_upper

DELTA_A_MM, DELTA_S_MM = 3.0, 5.0
H_DEG, H_MAX_DEG = 0.6, 0.86


def part_a():
    df = load("a_estimators_short_gaps")
    for c in ["excess", "estPeak", "peak"]:
        df[c + "_mm"] = df[c] * 1e3
    cells = ["pattern", "phase", "fault"]
    rows = []
    # H1a: HOLD - DR > 0 and HOLD - EKF > 0, burst of 8, entry and exit, no fault (Holm within H1a)
    h1a = []
    for phase in ["entry", "exit"]:
        sub = df[(df.pattern == "burst8") & (df.phase == phase) & (df.fault == "none")]
        for other in ["dr", "ekf"]:
            d = paired(sub, cells, "hold", other, "excess_mm")["diff"]
            m, lo, hi = bootstrap_ci(d)
            h1a.append({"test": "H1a", "cell": f"burst8/{phase}/none", "contrast": f"hold-{other} excess", "median": m, "ci95_lo": lo, "ci95_hi": hi, "p": signflip_p(d), "n": len(d)})
    for r, pa in zip(h1a, holm([r["p"] for r in h1a])):
        r["p_holm"] = pa
        r["supported"] = bool(r["ci95_lo"] > 0 and pa < 0.05)
    rows += h1a
    # H1b: DR - EKF tracking equivalence within +-3 mm (TOST, 90% CI), every cell
    for (pat, ph, f), sub in df.groupby(cells):
        d = paired(sub, cells, "dr", "ekf", "excess_mm")["diff"]
        t = tost(d, DELTA_A_MM)
        rows.append({"test": "H1b", "cell": f"{pat}/{ph}/{f}", "contrast": "dr-ekf excess", "median": t["median"], "ci90_lo": t["ci90_lo"], "ci90_hi": t["ci90_hi"], "n": len(d), "supported": t["equivalent"]})
    # H1c: EKF better than DR on estimate error and steering jump, every cell
    for (pat, ph, f), sub in df.groupby(cells):
        for metric in ["estPeak_mm", "steerJump"]:
            d = paired(sub, cells, "dr", "ekf", metric)["diff"]
            m, lo, hi = bootstrap_ci(d)
            ratio = float(np.median(sub[sub.arm == "dr"][metric]) / max(np.median(sub[sub.arm == "ekf"][metric]), 1e-12))
            rows.append({"test": "H1c", "cell": f"{pat}/{ph}/{f}", "contrast": f"dr-ekf {metric}", "median": m, "ci95_lo": lo, "ci95_hi": hi, "p": signflip_p(d), "n": len(d), "ratio_dr_over_ekf": ratio, "supported": bool(lo > 0)})
    out = pd.DataFrame(rows)
    out.to_csv(RESULTS / "a_estimators_short_gaps" / "tests.csv", index=False)
    summ = df.groupby(cells + ["arm"])[["excess_mm", "estPeak_mm", "steerJump", "peak_mm"]].median().round(2).unstack("arm")
    summ.to_csv(RESULTS / "a_estimators_short_gaps" / "summary_medians.csv")
    return out, summ


def part_cal():
    df = load("cal_monitor")
    df["maxD_deg"] = np.degrees(df["maxDgated"])
    mx = float(df["maxD_deg"].max())
    if mx < H_DEG:
        h, decision = H_DEG, "keep h = 0.6 deg"
    elif 1.25 * mx <= H_MAX_DEG:
        h, decision = 1.25 * mx, "raise h to 1.25 x observed max"
    else:
        h, decision = float("nan"), "K2: monitor infeasible (needs h > h_max = 0.86 deg)"
    res = {"runs": int(len(df)), "max_abs_D_deg": mx, "p99_abs_D_deg": float(df["maxD_deg"].quantile(0.99)), "median_abs_D_deg": float(df["maxD_deg"].median()),
           "false_alarms": int(df["alarms"].sum()), "h_deg": h, "decision": decision, "h_min_deg": 0.42, "h_max_deg": H_MAX_DEG,
           "by_cell_max_deg": df.groupby(["Tb", "phase"])["maxD_deg"].max().round(3).reset_index().to_dict("records")}
    (RESULTS / "cal_monitor" / "cal_decision.json").write_text(json.dumps(res, indent=2))
    return res


def part_b0():
    df = load("b0_policy_primary")
    df["excess_mm"] = df["excess"] * 1e3
    cells = ["Tb", "phase", "fault"]
    prim = (df.Tb == 4) & (df.phase == "entry")
    fault, healthy = df.fault == "gyro1", df.fault == "none"
    rows = []
    # P1 non-inferiority: SA - SB excess, fault runs, primary cell: upper 95% bound < 5 mm
    d1 = paired(df[prim & fault], cells, "SA", "SB", "excess_mm")["diff"]
    m1, lo1, hi1 = bootstrap_ci(d1)
    # P2 efficiency: SB - SA completion time, healthy runs, primary cell: lower 95% bound > 0
    d2 = paired(df[prim & healthy], cells, "SB", "SA", "finish")["diff"]
    m2, lo2, hi2 = bootstrap_ci(d2)
    # P3 value: N - SA excess, fault runs, primary cell: lower 95% bound > 0
    d3 = paired(df[prim & fault], cells, "N", "SA", "excess_mm")["diff"]
    m3, lo3, hi3 = bootstrap_ci(d3)
    # P1 one-sided: H0 diff >= 5 mm -> sign-flip on (diff - 5); P2/P3: H0 diff <= 0
    p = [signflip_p(d1 - DELTA_S_MM) if np.median(d1) < DELTA_S_MM else 1.0, signflip_p(d2) if m2 > 0 else 1.0, signflip_p(d3) if m3 > 0 else 1.0]
    ph = holm(p)
    rows.append({"test": "P1", "contrast": "SA-SB excess (fault)", "median": m1, "ci95_lo": lo1, "ci95_hi": hi1, "margin": DELTA_S_MM, "p": p[0], "p_holm": ph[0], "n": len(d1), "supported": bool(hi1 < DELTA_S_MM and ph[0] < 0.05)})
    rows.append({"test": "P2", "contrast": "SB-SA finish (healthy) [s]", "median": m2, "ci95_lo": lo2, "ci95_hi": hi2, "margin": 0, "p": p[1], "p_holm": ph[1], "n": len(d2), "supported": bool(lo2 > 0 and ph[1] < 0.05)})
    rows.append({"test": "P3", "contrast": "N-SA excess (fault)", "median": m3, "ci95_lo": lo3, "ci95_hi": hi3, "margin": 0, "p": p[2], "p_holm": ph[2], "n": len(d3), "supported": bool(lo3 > 0 and ph[2] < 0.05)})
    # P4 false alarms: independent healthy scenarios = unique (Tb, phase, seed) in the monitor-bearing arm SA
    hs = df[healthy & (df.arm == "SA")]
    k, n = int((hs["falseAlarm"] > 0).sum()), int(len(hs))
    rows.append({"test": "P4", "contrast": "false-alarm rate (healthy, B0 only)", "median": k / n, "ci95_lo": 0.0, "ci95_hi": clopper_pearson_upper(k, n), "n": n, "supported": bool(k == 0)})
    # secondary: every cell, every arm vs SB and vs N, plus detection delay
    for key, sub in df.groupby(cells):
        for a, b, metric in [("SA", "SB", "excess_mm"), ("N", "SA", "excess_mm"), ("SA", "SO", "excess_mm"), ("SB", "SA", "finish")]:
            d = paired(sub, cells, a, b, metric)["diff"]
            m, lo, hi = bootstrap_ci(d)
            rows.append({"test": "secondary", "contrast": f"{a}-{b} {metric} @ Tb{key[0]}/{key[1]}/{key[2]}", "median": m, "ci95_lo": lo, "ci95_hi": hi, "n": len(d)})
    out = pd.DataFrame(rows)
    out.to_csv(RESULTS / "b0_policy_primary" / "tests.csv", index=False)
    summ = df.groupby(cells + ["arm"]).agg(excess_mm=("excess_mm", "median"), timeLost_s=("timeLost", "median"), breach=("breach", "mean"),
                                           alarmDelay_s=("alarmDelay", "median"), alarmBeforeReturn=("alarmBeforeReturn", "mean")).round(3)
    summ.to_csv(RESULTS / "b0_policy_primary" / "summary_medians.csv")
    return out, summ


if __name__ == "__main__":
    pd.set_option("display.width", 200, "display.max_columns", 30, "display.max_rows", 200)
    which = sys.argv[1]
    if which == "A":
        out, summ = part_a()
        print(summ.to_string())
        print(out.round(3).to_string())
    elif which == "CAL":
        print(json.dumps(part_cal(), indent=2))
    elif which == "B0":
        out, summ = part_b0()
        print(summ.to_string())
        print(out[out.test != "secondary"].round(3).to_string())
        print(out[out.test == "secondary"].round(2).to_string())
