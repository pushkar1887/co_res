"""Check Codex's v2 suggestion with its own settings: 0.2 m/s, R = 0.55 m, lookahead 0.15 m, 10 Hz steering clock,
quantized encoders (r = 33 mm, 360 CPR) + gyro at 50 Hz, camera 10 Hz, motor lag 0.12 s, optional 10 % one-wheel slip
for 0.5 s. Methods: hold last command (ZOH), dead reckoning reset by camera (DR), EKF.
Usage: python codex_v2_check.py [speed] [burst_slots]"""
import math, sys, statistics
import numpy as np

DT, CAM_EVERY, SENS_EVERY = 0.01, 10, 2                 # 100 Hz physics, 10 Hz camera + steering, 50 Hz sensors
WB, TAU, R_WHEEL, CPR = 0.20, 0.12, 0.033, 360
RES = 2 * math.pi * R_WHEEL / CPR                       # 0.576 mm per count
SIG_GYR, SIG_CAM = 0.003, np.array([0.01, 0.01, math.radians(1.0)])

def make_path(R, L1=3.0, L3=2.0):
    La, cx, cy = math.pi / 2 * R, L1, R
    def point(s):
        if s <= L1: return (s, 0.0)
        if s <= L1 + La:
            a = -math.pi / 2 + (s - L1) / R; return (cx + R * math.cos(a), cy + R * math.sin(a))
        return (L1 + R, R + (s - L1 - La))
    def project(x, y):
        s1 = min(max(x, 0.0), L1)
        a = min(max(math.atan2(y - cy, x - cx), -math.pi / 2), 0.0)
        px, py = cx + R * math.cos(a), cy + R * math.sin(a)
        s3 = min(max(y - R, 0.0), L3)
        c = [(math.hypot(x - s1, y), s1, y),
             (math.hypot(x - px, y - py), L1 + (a + math.pi / 2) * R, R - math.hypot(x - cx, y - cy)),
             (math.hypot(x - L1 - R, y - R - s3), L1 + La + s3, (L1 + R) - x)]
        _, s, e = min(c); return s, e
    return point, project, L1, La, L1 + La + L3

def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi

def run(R, mode, drops=frozenset(), slip=None, v=0.2, look=0.15, seed=1, bias_res=0.0):
    rng = np.random.default_rng(seed)
    point, project, L1, La, Ltot = make_path(R)
    x = y = th = 0.0; uL = uR = v; distL = distR = 0.0; nL = nR = 0
    X = np.zeros(3); P = np.diag([1e-4, 1e-4, 1e-4]); w_cmd = 0.0
    qd, qw = (RES ** 2) / 6, (SIG_GYR * SENS_EVERY * DT) ** 2
    log, jumps, prev_fresh = [], [], True
    for k in range(int((Ltot / v) / DT)):
        t = k * DT
        s_true, e_true = project(x, y)
        log.append((t, s_true, e_true, math.hypot(X[0] - x, X[1] - y)))
        if s_true >= Ltot - 0.05: break
        # ground speeds (slip: right wheel's ground speed 10 % below its rotation)
        gR = uR * (0.9 if slip and slip[0] <= t < slip[0] + slip[1] else 1.0); gL = uL
        true_v, true_w = (gR + gL) / 2, (gR - gL) / WB
        distL += uL * DT; distR += uR * DT                # encoders count wheel ROTATION, not ground travel
        if k % SENS_EVERY == 0 and mode != "zoh":
            nL2, nR2 = math.floor(distL / RES), math.floor(distR / RES)
            ds = (nL2 - nL + nR2 - nR) * RES / 2; nL, nR = nL2, nR2
            wg = true_w + bias_res + rng.normal(0, SIG_GYR); dtt = SENS_EVERY * DT
            c, s = math.cos(X[2]), math.sin(X[2])
            X[0] += ds * c; X[1] += ds * s; X[2] = wrap(X[2] + wg * dtt)
            F = np.eye(3); F[0, 2] = -ds * s; F[1, 2] = ds * c
            G = np.array([[c, 0], [s, 0], [0, dtt]])
            P = F @ P @ F.T + G @ np.diag([qd + (0.02 * ds) ** 2, (SIG_GYR) ** 2]) @ G.T
        if k % CAM_EVERY == 0:                            # 10 Hz camera slot + 10 Hz steering clock
            fresh = (k // CAM_EVERY) not in drops
            if fresh:
                z = np.array([x, y, th]) + rng.normal(0, SIG_CAM)
                if mode in ("zoh", "dr"): X = z.copy()      # direct reset
                else:
                    inn = z - X; inn[2] = wrap(inn[2]); S = P + np.diag(SIG_CAM ** 2)
                    K = P @ np.linalg.inv(S); X = X + K @ inn; X[2] = wrap(X[2]); P = (np.eye(3) - K) @ P
            if mode != "zoh" or fresh:
                sp, _ = project(X[0], X[1]); gx, gy = point(sp + look)
                dx, dy = gx - X[0], gy - X[1]; yr = -math.sin(X[2]) * dx + math.cos(X[2]) * dy
                new_w = 2 * v * yr / (dx * dx + dy * dy)
                if fresh and not prev_fresh: jumps.append(abs(new_w - w_cmd))
                w_cmd = new_w
            prev_fresh = fresh
        uRc, uLc = v + WB * w_cmd / 2, v - WB * w_cmd / 2
        uR += (uRc - uR) * DT / TAU; uL += (uLc - uL) * DT / TAU
        x += true_v * math.cos(th) * DT; y += true_v * math.sin(th) * DT; th = wrap(th + true_w * DT)
    return log, jumps, (L1, La)

def window(log, t0, t1, col): return max(abs(r[col]) for r in log if t0 <= r[0] <= t1)

if __name__ == "__main__":
    v = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2
    look = 0.15 * v / 0.2
    PATS = {"8 isolated": [1, 3, 5, 7, 12, 14, 16, 18], "burst of 8": list(range(6, 14))}
    R, seeds = 0.55, range(1, 11)
    print(f"v={v} m/s, R={R} m, lookahead={look:.2f} m, 10 seeds. Values = median over seeds.")
    print(f"{'case':>22} {'phase':>6} | excess peak cross-track mm: {'ZOH':>6} {'DR':>6} {'EKF':>6} | est. err mm DR/EKF | steer jump rad/s DR/EKF")
    for slipcase in (False, True):
        for pname, pat in PATS.items():
            for ph in ("entry", "mid", "exit"):
                ex = {m: [] for m in ("zoh", "dr", "ekf")}; est = {"dr": [], "ekf": []}; jmp = {"dr": [], "ekf": []}
                for sd in seeds:
                    base, _, (L1, La) = run(R, "ekf", v=v, look=look, seed=sd)
                    tc = next(t for t, s, _, _ in base if s >= {"entry": L1, "mid": L1 + La / 2, "exit": L1 + La}[ph])
                    k0 = round((tc - 1.0) / 0.1); drops = frozenset(k0 + j for j in pat)
                    t0, t1 = k0 * 0.1, k0 * 0.1 + 4.0
                    sl = (k0 * 0.1 + 0.8, 0.5) if slipcase else None
                    for m in ex:
                        ref, _, _ = run(R, m, v=v, look=look, seed=sd)
                        lg, jp, _ = run(R, m, drops, sl, v=v, look=look, seed=sd)
                        ex[m].append(1000 * (window(lg, t0, t1, 2) - window(ref, t0, t1, 2)))
                        if m in est: est[m].append(1000 * window(lg, t0, t1, 3)); jmp[m].append(max(jp) if jp else 0)
                md = lambda a: statistics.median(a)
                name = pname + (" + slip" if slipcase else "")
                print(f"{name:>22} {ph:>6} | {'':>27} {md(ex['zoh']):6.1f} {md(ex['dr']):6.1f} {md(ex['ekf']):6.1f} |"
                      f" {md(est['dr']):5.1f} / {md(est['ekf']):5.1f}     | {md(jmp['dr']):.3f} / {md(jmp['ekf']):.3f}")
