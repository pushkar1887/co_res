// metrics.js: per-run scores (PLAN §8). Distances in metres, times in seconds.

export function score(run, ref, W, tF) {
  const L = run.log, p = run.p, dtS = 1 / p.sensHz, isEkf = p.arm !== 'hold' && p.arm !== 'dr';
  const inW = (t) => t >= W.t0 - 1e-9 && t <= W.t1 + 1e-9;
  const inBlack = (t) => W.tOn !== undefined && t >= W.tOn - 1e-9 && t < W.tRet - 1e-9;
  let peak = 0, sgn = 0, estPeak = 0, breachT = NaN, nOut = 0, n = 0, se = 0, neesS = 0, neesN = 0, maxDg = 0, lastT = -Infinity;
  const overlap = [0, 0, 0];
  for (let i = 0; i < L.t.length; i++) {
    const t = L.t[i]; if (!inW(t)) continue;
    lastT = t;
    const e = L.e[i], ae = Math.abs(e); n++; se += e * e;
    if (ae > peak) { peak = ae; sgn = e; }
    if (ae > p.corridor) { nOut++; if (isNaN(breachT)) breachT = t; }
    const ex = L.xe[i] - L.x[i], ey = L.ye[i] - L.y[i], ee = Math.hypot(ex, ey);
    if (ee > estPeak) estPeak = ee;
    if (inBlack(t)) {
      overlap[L.seg[i]] += dtS;
      if (isEkf) { const a = L.P00[i], b = L.P01[i], d = L.P11[i], det = a * d - b * b; if (det > 0) { neesS += (d * ex * ex - 2 * b * ex * ey + a * ey * ey) / det; neesN++; } }
      if (L.missAge[i] >= p.monGate - 1e-9) maxDg = Math.max(maxDg, Math.abs(L.D[i]));
    }
  }
  let refPeak = NaN;
  if (ref) { refPeak = 0; for (let i = 0; i < ref.log.t.length; i++) if (inW(ref.log.t[i])) refPeak = Math.max(refPeak, Math.abs(ref.log.e[i])); }
  // Secondary (post hoc, PLAN §20): path-aligned deviation from the reference, max |e_run(s) - e_ref(s)| over W.
  // Compares the two runs at the same route position, so it is comparable with the §6 drift formulas.
  let devPeak = NaN, devSigned = NaN;
  if (ref) {
    const rs = ref.log.s, re = ref.log.e, rt = ref.log.t;
    let j = 0; devPeak = 0;
    for (let i = 0; i < L.t.length; i++) {
      if (!inW(L.t[i])) continue;
      const s = L.s[i];
      while (j < rs.length - 2 && (rs[j + 1] < s || rt[j + 1] <= p.tStill)) j++;
      const s0 = rs[j], s1 = rs[j + 1], w = s1 > s0 ? Math.min(Math.max((s - s0) / (s1 - s0), 0), 1) : 0;
      const d = L.e[i] - (re[j] + w * (re[j + 1] - re[j]));
      if (Math.abs(d) > devPeak) { devPeak = Math.abs(d); devSigned = d; }
    }
  }
  let jump = 0; for (const [t, j] of run.steerJumps) if (inW(t)) jump = Math.max(jump, j);
  const alarmsW = run.alarmOn.filter(inW);
  const firstAfterFault = isFinite(tF) ? run.alarmOn.find((t) => t >= tF - 1e-9 && (W.tRet === undefined || t < W.tRet + 1e-9)) : undefined;
  const truncated = isFinite(W.t1) && !(lastT >= W.t1 - 1e-6) ? 1 : 0;
  const lostSlots = run.cam.ok.reduce((a, ok) => a + (ok ? 0 : 1), 0);
  // realised burst statistics of the camera channel (Part D)
  let bursts = 0, maxBurst = 0, cur = 0;
  for (const ok of run.cam.ok) { if (!ok) { cur++; if (cur === 1) bursts++; maxBurst = Math.max(maxBurst, cur); } else cur = 0; }
  return {
    peak, refPeak, excess: peak - refPeak, devPeak, devSigned, signedPeak: sgn, breach: peak > p.corridor ? 1 : 0,
    survival: isFinite(tF) && !isNaN(breachT) ? breachT - tF : NaN, fracOut: n ? nOut / n : NaN, rms: n ? Math.sqrt(se / n) : NaN,
    estPeak, steerJump: jump, finish: run.finish, timeLost: ref ? run.finish - ref.finish : NaN,
    alarms: alarmsW.length, alarmDelay: firstAfterFault !== undefined ? firstAfterFault - tF : NaN,
    alarmBeforeReturn: firstAfterFault !== undefined ? 1 : 0, falseAlarm: !isFinite(tF) && alarmsW.length > 0 ? 1 : 0,
    maxDgated: maxDg, nees: neesN ? neesS / neesN : NaN,
    onStraight1: overlap[0], onArc: overlap[1], onStraight2: overlap[2], lostSlots, truncated, bCal: run.bCal,
    bursts, meanBurstRealized: bursts ? lostSlots / bursts : 0, maxBurst,
  };
}
