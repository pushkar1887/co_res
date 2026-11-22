// monitor.js: gyro-encoder consistency residual over a sliding window (PLAN §5.3, §6.6, §6.8).
// D(t) = [theta_g(t) - theta_g(t - T_w)] - [theta_e(t) - theta_e(t - T_w)]

export function makeMonitor(p) {
  const n = Math.round(p.monWin * p.sensHz);
  const ringG = new Float64Array(n), ringE = new Float64Array(n);
  let thG = 0, thE = 0, i = 0, full = false;
  // wCorr: bias-corrected gyro rate [rad/s]; dL, dR: encoder wheel travel this sample [m]
  return function update(wCorr, dL, dR) {
    thG += wCorr / p.sensHz; thE += (dR - dL) / p.track;
    const oG = ringG[i], oE = ringE[i];
    ringG[i] = thG; ringE[i] = thE; i = (i + 1) % n; if (i === 0) full = true;
    return full ? (thG - oG) - (thE - oE) : 0;
  };
}
