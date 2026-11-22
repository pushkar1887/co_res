// monitor.js: gyro-encoder consistency residual over a sliding window (PLAN §5.3, §6.6, §6.8).
//
// v2 (rev 3.4): the residual is referenced to the monitor's OWN offset estimate o, not to the EKF bias.
//   r_k = omega_g - omega_enc,  omega_enc = (dR - dL) / (b * dt_s)
//   o   <- o + (r_k - o) * dt_s / tau_m        only while the camera is fresh; frozen during blackouts
//   D(t) = sum over the last T_w of (r_k - o) * dt_s
// v1 used the EKF bias estimate, which wanders ~0.09 deg/s RMS once q_b is large enough for the
// camera-available reference to recover from bias steps. That gave 7/480 healthy false alarms and
// triggered K2 in CAL (PLAN §20).

export function makeMonitor(p, oInit) {
  const n = Math.round(p.monWin * p.sensHz), dtS = 1 / p.sensHz;
  const ring = new Float64Array(n);
  let o = oInit, cum = 0, i = 0, full = false;
  // wg: raw gyro rate [rad/s]; dL, dR: encoder wheel travel this sample [m]; fresh: camera fresh (no missed report)
  return function update(wg, dL, dR, fresh) {
    const r = wg - (dR - dL) / (p.track * dtS);
    if (fresh) o += (r - o) * dtS / p.monRefTau;
    cum += (r - o) * dtS;
    const old = ring[i];
    ring[i] = cum; i = (i + 1) % n; if (i === 0) full = true;
    return { D: full ? cum - old : 0, o };
  };
}
