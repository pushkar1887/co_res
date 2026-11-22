"""Upgraded prototype: diff-drive robot with motor lag, wheel encoders (scale error), gyro (bias), absolute pose
fixes at 10 Hz that can drop out. Estimators: ZOH (no fusion), EKF enc-only, EKF enc+gyro(+bias state),
EKF + curvature-aware covariance speed governor. Measures true cross-track error."""
import math, sys
import numpy as np

W, TAU, VWMAX, DT, RE = 0.30, 0.10, 0.9, 0.01, 10
SIG_ENC, SIG_GYR = 0.005, 0.003           # per-sample noise: m/s, rad/s
SIG_FIX = np.array([0.01, 0.01, math.radians(1.0)])

def make_path(R, L1=3.0, L3=2.0):
    La, cx, cy = math.pi / 2 * R, L1, R
    def point(s):
        if s <= L1: return (s, 0.0)
        if s <= L1 + La:
            a = -math.pi / 2 + (s - L1) / R
            return (cx + R * math.cos(a), cy + R * math.sin(a))
        return (L1 + R, R + (s - L1 - La))
    def project(x, y):
        c = []
        s1 = min(max(x, 0.0), L1); c.append((math.hypot(x - s1, y), s1, y, (0.0, 1.0)))
        a = min(max(math.atan2(y - cy, x - cx), -math.pi / 2), 0.0)
        px, py = cx + R * math.cos(a), cy + R * math.sin(a); r = math.hypot(x - cx, y - cy)
        c.append((math.hypot(x - px, y - py), L1 + (a + math.pi / 2) * R, R - r, (-math.cos(a), -math.sin(a))))
        s3 = min(max(y - R, 0.0), L3); c.append((math.hypot(x - L1 - R, y - R - s3), L1 + La + s3, (L1 + R) - x, (-1.0, 0.0)))
        d, s, e, n = min(c, key=lambda q: q[0])
        return s, e, n
    kinks = (L1, L1 + La)                  # arc-length positions where curvature jumps
    return point, project, L1, La, L1 + La + L3, kinks

def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi

def run(R, mode, out=(0, 0), epsR=0.0, epsL=0.0, bias=0.0, v_nom=0.5, look=0.30, seed=1,
        budget=0.03, preview=0.6, vmin=0.15, bias_late=0.0):
    rng = np.random.default_rng(seed)
    point, project, L1, La, Ltot, kinks = make_path(R)
    x = y = th = 0.0; vr = vl = v_nom
    X = np.zeros(4); P = np.diag([1e-4, 1e-4, 1e-4, (0.01) ** 2])
    w_cmd, v_cmd = 0.0, v_nom
    use_gyro = mode in ("ekf", "gov", "mon")
    r_lp, alarm = 0.0, False
    qv = SIG_ENC ** 2 / 2 + (0.01 * v_nom) ** 2          # inflate for ~1% unmodelled scale error
    qw = SIG_GYR ** 2 if use_gyro else (2 * SIG_ENC / W) ** 2 + (0.01 * v_nom / W) ** 2
    log, k, t_done = [], 0, None
    while k < 4000:
        t = k * DT
        s_true, e_true, _ = project(x, y)
        log.append((t, s_true, e_true, v_cmd))
        if s_true >= Ltot - 0.05: t_done = t; break
        true_v, true_w = (vr + vl) / 2, (vr - vl) / W
        # --- sensors
        er = vr * (1 + epsR) + rng.normal(0, SIG_ENC); el = vl * (1 + epsL) + rng.normal(0, SIG_ENC)
        gy = true_w + bias + (bias_late if out[1] > 0 and t >= out[0] else 0.0) + rng.normal(0, SIG_GYR)
        r_lp += ((gy - X[3]) - (er - el) / W - r_lp) * DT / 0.5
        in_out = out[1] > 0 and out[0] <= t < out[0] + out[1]
        if not in_out: alarm = False
        elif abs(r_lp) > math.radians(0.6): alarm = True
        # --- EKF predict
        vm = (er + el) / 2
        wm = (gy - X[3]) if use_gyro else (er - el) / W
        c, s = math.cos(X[2]), math.sin(X[2])
        X[0] += vm * c * DT; X[1] += vm * s * DT; X[2] = wrap(X[2] + wm * DT)
        F = np.eye(4); F[0, 2] = -vm * s * DT; F[1, 2] = vm * c * DT
        if use_gyro: F[2, 3] = -DT
        G = np.array([[c * DT, 0], [s * DT, 0], [0, DT], [0, 0]])
        P = F @ P @ F.T + G @ np.diag([qv, qw]) @ G.T + np.diag([0, 0, 0, (1e-5) ** 2])
        # --- absolute fix (10 Hz) unless inside outage
        if k % RE == 0 and not (out[0] <= t < out[0] + out[1]):
            z = np.array([x, y, th]) + rng.normal(0, SIG_FIX)
            if mode == "zoh":
                X[:3] = z                                   # no fusion: use the raw fix
            else:
                H = np.zeros((3, 4)); H[:, :3] = np.eye(3)
                inn = z - X[:3]; inn[2] = wrap(inn[2])
                S = H @ P @ H.T + np.diag(SIG_FIX ** 2); K = P @ H.T @ np.linalg.inv(S)
                X = X + K @ inn; X[2] = wrap(X[2]); P = (np.eye(4) - K @ H) @ P
            fresh = True
        else:
            fresh = False
        # --- controller: ZOH only recomputes on a fresh fix; EKF modes every step
        if mode != "zoh" or fresh:
            sp, _, n = project(X[0], X[1])
            gx, gy_ = point(sp + look); dx, dy = gx - X[0], gy_ - X[1]
            yr = -math.sin(X[2]) * dx + math.cos(X[2]) * dy
            v_cmd = v_nom
            if mode == "gov":                                # slow down only if a curvature jump is near
                sig_lat = math.sqrt(max(np.array(n) @ P[:2, :2] @ np.array(n), 0))
                near = any(0 <= kk - sp <= preview for kk in kinks) or any(0 <= sp - kk <= 0.15 for kk in kinks)
                if near and 3 * sig_lat > 0.4 * budget:
                    v_cmd = max(vmin, v_nom * min(1.0, 0.4 * budget / (3 * sig_lat)))
            if mode == "mon" and alarm: v_cmd = vmin
            w_cmd = 2 * v_cmd * yr / (dx * dx + dy * dy)
        # --- actuators: wheel commands, saturation, first-order motor lag
        vrc = max(-VWMAX, min(VWMAX, v_cmd + w_cmd * W / 2)); vlc = max(-VWMAX, min(VWMAX, v_cmd - w_cmd * W / 2))
        vr += (vrc - vr) * DT / TAU; vl += (vlc - vl) * DT / TAU
        x += true_v * math.cos(th) * DT; y += true_v * math.sin(th) * DT; th = wrap(th + true_w * DT)
        k += 1
    return log, t_done, (L1, La)

def peak(log, t0, t1): return max(abs(e) for t, _, e, _ in log if t0 <= t <= t1)

