"""Quick prototype of the brief's experiment to check whether the effects are big enough to see.
Straight 1.5 m -> left 90 deg arc radius R -> straight 1.5 m. Pure Pursuit, hold last omega on dropped reports."""
import math, statistics, sys

V, DT, REPORT_EVERY = 0.20, 0.01, 10  # 10 ms physics, report every 100 ms
PATTERNS = {
    "isolated": [1, 3, 5, 7, 12, 14, 16, 18],
    "2x4": [2, 3, 4, 5, 14, 15, 16, 17],
    "1x8": list(range(6, 14)),
}

def make_path(R):
    L1, La = 1.5, math.pi / 2 * R
    cx, cy = 1.5, R
    def point(s):
        if s <= L1: return (s, 0.0)
        if s <= L1 + La:
            a = -math.pi / 2 + (s - L1) / R
            return (cx + R * math.cos(a), cy + R * math.sin(a))
        return (1.5 + R, R + (s - L1 - La))
    def project(x, y):
        """returns (s, signed lateral error, left positive)"""
        c = []
        s1 = min(max(x, 0.0), L1); c.append((math.hypot(x - s1, y), s1, y))
        a = math.atan2(y - cy, x - cx); a = min(max(a, -math.pi / 2), 0.0)
        px, py = cx + R * math.cos(a), cy + R * math.sin(a)
        r = math.hypot(x - cx, y - cy)
        c.append((math.hypot(x - px, y - py), L1 + (a + math.pi / 2) * R, R - r))
        s3 = min(max(y - R, 0.0), 1.5); c.append((math.hypot(x - 1.5 - R, y - R - s3), L1 + La + s3, (1.5 + R) - x))
        d, s, e = min(c)
        return s, e
    return point, project, L1, La

def run(R, look, drops=frozenset(), T=None):
    point, project, L1, La = make_path(R)
    T = T or (3.0 + La) / V - 1.0
    x = y = th = 0.0; w = 0.0; log = []
    for k in range(int(T / DT)):
        if k % REPORT_EVERY == 0 and (k // REPORT_EVERY) not in drops:
            s, _ = project(x, y)
            gx, gy = point(s + look)
            dx, dy = gx - x, gy - y
            yr = -math.sin(th) * dx + math.cos(th) * dy
            w = 2 * V * yr / (dx * dx + dy * dy)
        s, e = project(x, y)
        log.append((k * DT, s, e))
        x += V * math.cos(th) * DT; y += V * math.sin(th) * DT; th += w * DT
    return log, L1, La

def experiment(look):
    print(f"\n=== lookahead {look} m ===")
    print(f"{'R':>5} {'phase':>6} {'pattern':>9} | peak excess error cm (median [min-max] over 5 offsets) | signed peak cm | analytic cm")
    for R in (0.55, 0.75, 1.00):
        base, L1, La = run(R, look)
        bpeak = max(abs(e) for _, _, e in base)
        tcross = lambda sv: next(t for t, s, _ in base if s >= sv)
        phases = {"entry": tcross(L1), "mid": tcross(L1 + La / 2), "exit": tcross(L1 + La)}
        print(f"  baseline R={R}: whole-run peak |e| = {100*bpeak:.2f} cm")
        for ph, tc in phases.items():
            for name, pat in PATTERNS.items():
                ex, sg = [], []
                for off in (-0.2, -0.1, 0.0, 0.1, 0.2):
                    k0 = round((tc + off - 1.0) / 0.1)
                    drops = frozenset(k0 + j for j in pat)
                    lr, _, _ = run(R, look, drops)
                    i0, i1 = k0 * REPORT_EVERY, (k0 + 40) * REPORT_EVERY
                    diff = [lr[i][2] - base[i][2] for i in range(i0, min(i1, len(lr)))]
                    j = max(range(len(diff)), key=lambda i: abs(diff[i]))
                    ex.append(100 * abs(diff[j])); sg.append(100 * diff[j])
                gap = {"isolated": 0.2, "2x4": 0.5, "1x8": 0.9}[name]
                pred = 100 * (V * gap) ** 2 / (2 * R) if ph != "mid" else 0.0
                print(f"{R:>5} {ph:>6} {name:>9} | {statistics.median(ex):6.2f} [{min(ex):.2f}-{max(ex):.2f}] | {statistics.median(sg):+6.2f} | {pred:5.2f}")

experiment(0.15)
if "--both" in sys.argv: experiment(0.12)
