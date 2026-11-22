"""Summaries for Parts B, C, D, S (PLAN §7). Writes results/<exp>/summary_*.csv and prints the key tables.
Medians over seeds; mm and s. dev = path-aligned deviation (secondary metric, PLAN §8)."""
import numpy as np
import pandas as pd

from common import RESULTS, load, paired, bootstrap_ci


def prep(df):
    df = df.copy()
    df["excess_mm"], df["dev_mm"], df["peak_mm"] = df.excess * 1e3, df.devPeak * 1e3, df.peak * 1e3
    return df


def part_b():
    df = prep(load("b_fault_map"))
    agg = df.groupby(["Tb", "fault", "phase", "arm"]).agg(
        excess_mm=("excess_mm", "median"), dev_mm=("dev_mm", "median"), breach=("breach", "mean"),
        timeLost_s=("timeLost", "median"), alarmBeforeReturn=("alarmBeforeReturn", "mean"), alarmDelay_s=("alarmDelay", "median"),
        falseAlarm=("falseAlarm", "sum"), signedPeak_mm=("signedPeak", lambda x: 1e3 * np.median(x))).round(3)
    agg.to_csv(RESULTS / "b_fault_map" / "summary_medians.csv")
    # SA - SB paired, every fault x onset at Tb = 4 (dev and time)
    rows = []
    for (f, ph), sub in df[df.Tb == 4].groupby(["fault", "phase"]):
        for a, b, m in [("SA", "SB", "dev_mm"), ("SA", "SB", "excess_mm"), ("SB", "SA", "timeLost"), ("dr", "N", "dev_mm")]:
            d = paired(sub, ["Tb", "fault", "phase"], a, b, m)["diff"]
            med, lo, hi = bootstrap_ci(d)
            rows.append({"fault": f, "onset": ph, "contrast": f"{a}-{b} {m}", "median": med, "ci95_lo": lo, "ci95_hi": hi, "n": len(d)})
    pd.DataFrame(rows).to_csv(RESULTS / "b_fault_map" / "paired_tb4.csv", index=False)
    return agg, pd.DataFrame(rows)


def part_c():
    df = prep(load("c_scaling"))
    df.groupby(["exp", "fault", "R", "v", "Ld", "arm"]).agg(dev_mm=("dev_mm", "median"), excess_mm=("excess_mm", "median"),
        alarmDelay_s=("alarmDelay", "median"), alarmBeforeReturn=("alarmBeforeReturn", "mean"), timeLost_s=("timeLost", "median")
        ).round(3).to_csv(RESULTS / "c_scaling" / "summary_medians.csv")
    c2 = df[df.exp == "C2"]
    rows = []
    for f, sub in c2.groupby("fault"):
        b = float(f.replace("gyro", ""))
        for a, o in [("SA", "SO"), ("N", "SO"), ("N", "SA")]:
            d = paired(sub, ["fault"], a, o, "dev_mm")["diff"]
            med, lo, hi = bootstrap_ci(d)
            rows.append({"b_deg_s": b, "contrast": f"{a}-{o} dev_mm", "median": med, "ci95_lo": lo, "ci95_hi": hi, "n": len(d)})
    reg = pd.DataFrame(rows)
    reg.to_csv(RESULTS / "c_scaling" / "regret_c2.csv", index=False)
    return reg


def part_d():
    df = prep(load("d_burstiness"))
    df["lossRate"] = df.lostSlots / df.totalSlots
    s = df.groupby(["meanBurst", "arm"]).agg(peak_mm=("peak_mm", "median"), excess_mm=("excess_mm", "median"), fracOut=("fracOut", "mean"),
        breach=("breach", "mean"), rms_mm=("rms", lambda x: 1e3 * np.median(x)), lossRate=("lossRate", "median"),
        meanBurstRealized=("meanBurstRealized", "median"), bursts=("bursts", "median"), zeroBurstRuns=("bursts", lambda x: int((x == 0).sum()))).round(3)
    s.to_csv(RESULTS / "d_burstiness" / "summary_medians.csv")
    return s


def part_s():
    df = prep(load("s_sensitivity"))
    s = df.groupby(["variant", "Tb", "phase", "fault", "arm"]).agg(excess_mm=("excess_mm", "median"), dev_mm=("dev_mm", "median"),
        timeLost_s=("timeLost", "median"), falseAlarm=("falseAlarm", "sum"), alarmDelay_s=("alarmDelay", "median")).round(3)
    s.to_csv(RESULTS / "s_sensitivity" / "summary_medians.csv")
    rows = []
    for (var, tb, ph, f), sub in df.groupby(["variant", "Tb", "phase", "fault"]):
        d = paired(sub, ["variant", "Tb", "phase", "fault"], "SA", "SB", "excess_mm")["diff"]
        med, lo, hi = bootstrap_ci(d)
        rows.append({"variant": var, "Tb": tb, "onset": ph, "fault": f, "SA-SB excess median": med, "ci95_lo": lo, "ci95_hi": hi, "n": len(d)})
    p = pd.DataFrame(rows)
    p.to_csv(RESULTS / "s_sensitivity" / "paired_sa_sb.csv", index=False)
    return s, p


if __name__ == "__main__":
    pd.set_option("display.width", 200, "display.max_rows", 200)
    _, pb = part_b()
    print(pb[pb.contrast.str.startswith("SA-SB dev")].round(2).to_string())
    print(part_c().round(2).to_string())
    print(part_d().to_string())
    _, ps = part_s()
    print(ps.round(2).to_string())
