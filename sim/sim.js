/* sim.js — the ONE simulation model for this project.
 * Robot plant, motors, sensors (quantized encoders, gyro, overhead camera), camera-loss schedules,
 * estimators (HOLD, DR, EKF, EKF+M), Pure Pursuit, consistency monitor, and per-run metrics.
 * Used by: web/index.html (demo), sim/batch_worker.js (experiments), ESP32 trace export.
 * Plain JS, no dependencies. Exposes a global `Sim` (window or worker).
 * All data is simulated; every random stream is seeded so runs are reproducible.
 */
(function (root) {
  'use strict';
  const D2R = Math.PI / 180;

  const DEFAULTS = {
    // route: straight L1 -> 90 deg left arc of radius R -> straight L3
    R: 1.0, L1: 3.0, L3: 3.0,
    // robot + actuators
    v: 0.5,              // nominal forward speed (m/s)
    lookTime: 0.75,      // Pure Pursuit lookahead = lookTime * v  (0.15 m at 0.2 m/s)
    track: 0.20,         // wheel spacing b (m)
    wheelR: 0.033,       // nominal wheel radius (m)
    cpr: 360,            // encoder counts per wheel revolution -> 0.576 mm per count
    motorTau: 0.12,      // first-order motor response (s)
    wheelVmax: 1.0,      // wheel speed limit (m/s)
    tStill: 1.0,         // robot stands still first; gyro bias is calibrated here
    // clocks
    dt: 0.01, sensHz: 50, camHz: 10, steerHz: 10,
    // sensor models (assumed, MEMS-class; see docs/BRIEF.md)
    camSigXY: 0.01, camSigTh: 1.0 * D2R,
    gyroARW: 0.015 * D2R,                       // angle random walk, rad/sqrt(s)
    gyroGMsig: 0.004 * D2R, gyroGMtau: 100,     // bias instability as Gauss-Markov
    gyroTurnOn: 1.0 * D2R,                      // turn-on bias sigma (removed by calibration)
    gyroWarmAmp: 0.2 * D2R, gyroWarmTau: 120,   // warm-up drift after power-on
    // EKF tuning
    ekfScaleSig: 0.01,   // unmodelled wheel-scale error allowance (fraction of ds)
    ekfBiasRW: 3e-6,     // bias random walk per sensor step (rad/s)
    // consistency monitor (gyro vs encoder heading over a sliding window)
    monWin: 1.0, monThr: 0.6 * D2R, monMinGap: 0.3, monSlow: 0.3,
    // scoring
    corridor: 0.03, tMax: 150,
  };
  const METHODS = ['hold', 'dr', 'ekf', 'ekfm'];
  const METHOD_LABEL = { hold: 'Hold last command', dr: 'Dead reckoning', ekf: 'EKF', ekfm: 'EKF + monitor' };
  const PATTERNS = {               // 20-slot (2 s) windows, all drop 8 reports, mean slot 9.5
    iso8: [1, 3, 5, 7, 12, 14, 16, 18],
    '2x4': [2, 3, 4, 5, 14, 15, 16, 17],
    burst8: [6, 7, 8, 9, 10, 11, 12, 13],
  };

  // ---------- helpers ----------
  const clamp = (a, lo, hi) => (a < lo ? lo : a > hi ? hi : a);
  const wrap = (a) => { a = (a + Math.PI) % (2 * Math.PI); if (a < 0) a += 2 * Math.PI; return a - Math.PI; };

  function makeRng(seed) {                    // mulberry32 + Box-Muller
    let a = seed >>> 0, spare = null;
    const u = () => {
      a = (a + 0x6D2B79F5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
    const n = () => {
      if (spare !== null) { const s = spare; spare = null; return s; }
      const u1 = Math.max(u(), 1e-12), u2 = u(), r = Math.sqrt(-2 * Math.log(u1));
      spare = r * Math.sin(2 * Math.PI * u2);
      return r * Math.cos(2 * Math.PI * u2);
    };
    return { u, n };
  }

  // ---------- route ----------
  function makePath(R, L1, L3) {
    const La = Math.PI / 2 * R, cx = L1, cy = R, total = L1 + La + L3;
    function point(s) {
      if (s <= L1) return [s, 0];
      if (s <= L1 + La) { const a = -Math.PI / 2 + (s - L1) / R; return [cx + R * Math.cos(a), cy + R * Math.sin(a)]; }
      return [L1 + R, R + (s - L1 - La)];
    }
    // closest point on the continuous route; e = signed cross-track error (left of travel = +)
    function project(x, y) {
      const s1 = clamp(x, 0, L1);
      let best = { d: Math.hypot(x - s1, y), s: s1, e: y };
      const a = clamp(Math.atan2(y - cy, x - cx), -Math.PI / 2, 0);
      const d2 = Math.hypot(x - (cx + R * Math.cos(a)), y - (cy + R * Math.sin(a)));
      if (d2 < best.d) best = { d: d2, s: L1 + (a + Math.PI / 2) * R, e: R - Math.hypot(x - cx, y - cy) };
      const s3 = clamp(y - R, 0, L3), d3 = Math.hypot(x - (L1 + R), y - (R + s3));
      if (d3 < best.d) best = { d: d3, s: L1 + La + s3, e: (L1 + R) - x };
      return best;
    }
    return { R, L1, L3, La, total, point, project, kinks: [L1, L1 + La] };
  }

  // ---------- gyro: true rate + turn-on bias + warm-up drift + bias instability + white noise ----------
  function makeGyro(p, seed) {
    const g = makeRng(seed), dtS = 1 / p.sensHz;
    const turnOn = p.gyroTurnOn * g.n(), warmSign = g.u() < 0.5 ? -1 : 1;
    const phi = Math.exp(-dtS / p.gyroGMtau), drive = p.gyroGMsig * Math.sqrt(1 - phi * phi);
    const sig = p.gyroARW / Math.sqrt(dtS);
    let gm = p.gyroGMsig * g.n();
    return {
      sample(trueW, t, extra) {
        gm = phi * gm + drive * g.n();
        const bias = turnOn + warmSign * p.gyroWarmAmp * (1 - Math.exp(-t / p.gyroWarmTau)) + gm + extra;
        return { w: trueW + bias + sig * g.n(), bias };
      },
    };
  }

  // ---------- camera loss schedule ----------
  // loss = {kind:'none'} | {kind:'slots', slots:[...]} | {kind:'ge', lossRate, meanBurst, from}
  function makeLoss(loss, seed, p) {
    if (!loss || loss.kind === 'none') return () => false;
    if (loss.kind === 'slots') { const set = new Set(loss.slots); return (j) => set.has(j); }
    if (loss.kind === 'ge') {        // Gilbert two-state Markov channel, all reports lost in the bad state
      const g = makeRng(seed * 7919 + 4), r = 1 / loss.meanBurst, q = loss.lossRate * r / (1 - loss.lossRate);
      const from = Math.round((loss.from || 2.0) * p.camHz), cache = [];
      let bad = false;
      return (j) => {
        while (cache.length <= j) { const i = cache.length; if (i >= from) bad = bad ? g.u() >= r : g.u() < q; cache.push(i >= from && bad); }
        return cache[j];
      };
    }
    throw new Error('unknown loss kind ' + loss.kind);
  }
  function slotsForBlackout(start, len, camHz) {
    const out = [];
    for (let j = Math.ceil(start * camHz - 1e-9); j / camHz < start + len - 1e-9; j++) out.push(j);
    return out;
  }

  // ---------- small matrix helpers (4x4 EKF) ----------
  function ekfPredict(X, P, ds, w, dtS, p) {
    const c = Math.cos(X[2]), s = Math.sin(X[2]);
    X[0] += ds * c; X[1] += ds * s; X[2] = wrap(X[2] + w * dtS);
    // F = I + [0 0 -ds s 0; 0 0 ds c 0; 0 0 0 -dtS; 0 0 0 0]
    const f02 = -ds * s, f12 = ds * c, f23 = -dtS;
    const FP = [0, 1, 2, 3].map((i) => P[i].slice());
    for (let j = 0; j < 4; j++) { FP[0][j] += f02 * P[2][j]; FP[1][j] += f12 * P[2][j]; FP[2][j] += f23 * P[3][j]; }
    for (let i = 0; i < 4; i++) {
      const r = FP[i];
      P[i][0] = r[0] + r[2] * f02; P[i][1] = r[1] + r[2] * f12; P[i][2] = r[2] + r[3] * f23; P[i][3] = r[3];
    }
    const resM = 2 * Math.PI * p.wheelR / p.cpr;
    const qds = resM * resM / 12 + (p.ekfScaleSig * ds) ** 2, qth = p.gyroARW * p.gyroARW * dtS;
    P[0][0] += c * c * qds; P[0][1] += c * s * qds; P[1][0] += c * s * qds; P[1][1] += s * s * qds;
    P[2][2] += qth; P[3][3] += p.ekfBiasRW * p.ekfBiasRW;
  }
  function inv3(m) {
    const [a, b, c] = m[0], [d, e, f] = m[1], [g, h, i] = m[2];
    const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g, det = a * A + b * B + c * C;
    return [[A / det, -(b * i - c * h) / det, (b * f - c * e) / det],
            [B / det, (a * i - c * g) / det, -(a * f - c * d) / det],
            [C / det, -(a * h - b * g) / det, (a * e - b * d) / det]];
  }
  function ekfUpdate(X, P, z, Rd) {            // H = [I3 0]
    const S = [0, 1, 2].map((i) => [0, 1, 2].map((j) => P[i][j] + (i === j ? Rd[i] : 0)));
    const Si = inv3(S), inn = [z[0] - X[0], z[1] - X[1], wrap(z[2] - X[2])];
    const K = [0, 1, 2, 3].map((i) => [0, 1, 2].map((j) => P[i][0] * Si[0][j] + P[i][1] * Si[1][j] + P[i][2] * Si[2][j]));
    for (let i = 0; i < 4; i++) X[i] += K[i][0] * inn[0] + K[i][1] * inn[1] + K[i][2] * inn[2];
    X[2] = wrap(X[2]);
    const top = [P[0].slice(), P[1].slice(), P[2].slice()];
    for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) P[i][j] -= K[i][0] * top[0][j] + K[i][1] * top[1][j] + K[i][2] * top[2][j];
    for (let i = 0; i < 4; i++) for (let j = 0; j < i; j++) { const m = 0.5 * (P[i][j] + P[j][i]); P[i][j] = m; P[j][i] = m; }
  }

  // ---------- one simulation run ----------
  // cfg: {method, seed, loss, faults, ...any DEFAULTS override}
  // faults: {gyroStep (rad/s), gyroStepAt (s), scaleL, scaleR, slip:{wheel:'L'|'R', factor, start, dur}}
  function simulate(cfg) {
    const p = Object.assign({}, DEFAULTS, cfg);
    const method = p.method || 'ekf', seed = p.seed || 1, f = p.faults || {};
    const path = makePath(p.R, p.L1, p.L3), look = p.lookTime * p.v;
    const sensEvery = Math.round(1 / (p.sensHz * p.dt)), camEvery = Math.round(1 / (p.camHz * p.dt)), steerEvery = Math.round(1 / (p.steerHz * p.dt));
    const dtS = sensEvery * p.dt, radPerCount = 2 * Math.PI / p.cpr;
    const rL = p.wheelR * (1 + (f.scaleL || 0)), rR = p.wheelR * (1 + (f.scaleR || 0));
    const gyro = makeGyro(p, seed * 7919 + 1), cam = makeRng(seed * 7919 + 2), dropped = makeLoss(p.loss, seed, p);
    const Rd = [p.camSigXY ** 2, p.camSigXY ** 2, p.camSigTh ** 2];
    const sigG = p.gyroARW / Math.sqrt(dtS);
    const monN = Math.round(p.monWin * p.sensHz), useSensors = method !== 'hold';

    let x = 0, y = 0, th = 0, uL = 0, uR = 0, phiL = 0, phiR = 0, cntL = 0, cntR = 0, trueW = 0;
    let vc = 0, wc = 0, calSum = 0, calN = 0, bCal = 0, calibrated = false;
    const X = [0, 0, 0, 0], P = [[1e-4, 0, 0, 0], [0, 1e-4, 0, 0], [0, 0, (1 * D2R) ** 2, 0], [0, 0, 0, (0.1 * D2R) ** 2]];
    let lastFix = 0, freshSinceSteer = false, gapBeforeFresh = false, alarm = false;
    let thG = 0, thE = 0; const ringG = new Float64Array(monN), ringE = new Float64Array(monN); let ringI = 0, ringFull = false, Dm = 0;
    const L = { t: [], x: [], y: [], th: [], xe: [], ye: [], the: [], e: [], s: [], vc: [], wc: [], uL: [], uR: [], uLc: [], uRc: [],
      alarm: [], D: [], P00: [], P01: [], P11: [], dnL: [], dnR: [], wg: [], bias: [] };
    const camLog = { j: [], t: [], ok: [], z: [] }, steerJumps = [], alarmOn = [];
    let finish = NaN, gyroBiasTrue = 0;

    const nPhys = Math.round(p.tMax / p.dt);
    for (let k = 0; k <= nPhys; k++) {
      const t = k * p.dt, moving = t >= p.tStill - 1e-9;

      // --- onboard sensors at 50 Hz: quantized encoder counts + gyro
      if (k % sensEvery === 0) {
        const nL = Math.floor(phiL / radPerCount), nR = Math.floor(phiR / radPerCount);
        const dnL = nL - cntL, dnR = nR - cntR; cntL = nL; cntR = nR;
        const dL = dnL * radPerCount * p.wheelR, dR = dnR * radPerCount * p.wheelR;     // nominal radius
        const extra = (f.gyroStep && t >= f.gyroStepAt) ? f.gyroStep : 0;
        const gs = gyro.sample(trueW, t, extra), wg = gs.w; gyroBiasTrue = gs.bias;
        if (!calibrated) {
          if (t < p.tStill - 1e-9) { if (k > 0) { calSum += wg; calN++; } }
          else { bCal = calSum / Math.max(calN, 1); calibrated = true; X[3] = bCal; P[3][3] = (sigG / Math.sqrt(Math.max(calN, 1))) ** 2; }
        }
        if (calibrated && useSensors) {
          const ds = 0.5 * (dL + dR);
          if (method === 'dr') { const c = Math.cos(X[2]), s = Math.sin(X[2]); X[0] += ds * c; X[1] += ds * s; X[2] = wrap(X[2] + (wg - bCal) * dtS); }
          else ekfPredict(X, P, ds, wg - X[3], dtS, p);
        }
        // consistency monitor: heading change from gyro vs from encoders over the last monWin seconds
        if (calibrated) {
          thG += (wg - (method === 'dr' || method === 'hold' ? bCal : X[3])) * dtS; thE += (dR - dL) / p.track;
          const oG = ringG[ringI], oE = ringE[ringI]; ringG[ringI] = thG; ringE[ringI] = thE; ringI = (ringI + 1) % monN;
          if (ringI === 0) ringFull = true;
          Dm = ringFull ? (thG - oG) - (thE - oE) : 0;
          if (useSensors && moving && !alarm && t - lastFix > p.monMinGap && Math.abs(Dm) > p.monThr) { alarm = true; alarmOn.push(t); }
        }
        // log (state at time t)
        const pr = path.project(x, y);
        L.t.push(t); L.x.push(x); L.y.push(y); L.th.push(th); L.xe.push(X[0]); L.ye.push(X[1]); L.the.push(X[2]);
        L.e.push(pr.e); L.s.push(pr.s); L.vc.push(vc); L.wc.push(wc); L.uL.push(uL); L.uR.push(uR);
        L.uLc.push(clamp(vc - wc * p.track / 2, -p.wheelVmax, p.wheelVmax)); L.uRc.push(clamp(vc + wc * p.track / 2, -p.wheelVmax, p.wheelVmax));
        L.alarm.push(alarm ? 1 : 0); L.D.push(Dm); L.P00.push(P[0][0]); L.P01.push(P[0][1]); L.P11.push(P[1][1]);
        L.dnL.push(dnL); L.dnR.push(dnR); L.wg.push(wg); L.bias.push(gyroBiasTrue);
        if (moving && pr.s >= path.total - 0.05) { finish = t; break; }
      }

      // --- overhead camera at 10 Hz (noise drawn every slot so random streams stay paired across methods)
      if (k % camEvery === 0) {
        const j = k / camEvery, n0 = cam.n(), n1 = cam.n(), n2 = cam.n(), ok = !dropped(j);
        const z = [x + p.camSigXY * n0, y + p.camSigXY * n1, wrap(th + p.camSigTh * n2)];
        camLog.j.push(j); camLog.t.push(t); camLog.ok.push(ok ? 1 : 0); camLog.z.push(z);
        if (ok) {
          if (!freshSinceSteer && t - lastFix > 1.5 / p.camHz) gapBeforeFresh = true;
          lastFix = t; alarm = false; freshSinceSteer = true;
          if (method === 'hold' || method === 'dr') { X[0] = z[0]; X[1] = z[1]; X[2] = z[2]; }
          else ekfUpdate(X, P, z, Rd);
        }
      }

      // --- steering at 10 Hz, same clock for every method
      if (k % steerEvery === 0 && moving && (useSensors || freshSinceSteer)) {
        const pr = path.project(X[0], X[1]), g = path.point(pr.s + look);
        const dx = g[0] - X[0], dy = g[1] - X[1], yr = -Math.sin(X[2]) * dx + Math.cos(X[2]) * dy;
        const vcmd = (method === 'ekfm' && alarm) ? p.v * p.monSlow : p.v;
        const wnew = 2 * vcmd * yr / (dx * dx + dy * dy);
        if (gapBeforeFresh && freshSinceSteer) { steerJumps.push([t, Math.abs(wnew - wc)]); gapBeforeFresh = false; }
        vc = vcmd; wc = wnew; freshSinceSteer = false;
      } else if (k % steerEvery === 0) freshSinceSteer = freshSinceSteer && !moving ? false : freshSinceSteer;

      // --- motors: wheel-speed command -> saturation -> first-order lag
      const uLc = clamp(vc - wc * p.track / 2, -p.wheelVmax, p.wheelVmax), uRc = clamp(vc + wc * p.track / 2, -p.wheelVmax, p.wheelVmax);
      uL += (uLc - uL) * p.dt / p.motorTau; uR += (uRc - uR) * p.dt / p.motorTau;
      const omL = uL / p.wheelR, omR = uR / p.wheelR;          // motor controller assumes the nominal radius
      phiL += omL * p.dt; phiR += omR * p.dt;                   // encoders see wheel ROTATION
      let gL = omL * rL, gR = omR * rR;                         // ground speed uses the TRUE radius
      if (f.slip && t >= f.slip.start && t < f.slip.start + f.slip.dur) { if (f.slip.wheel === 'L') gL *= f.slip.factor; else gR *= f.slip.factor; }
      const vt = 0.5 * (gL + gR), wt = (gR - gL) / p.track, thm = th + 0.5 * wt * p.dt;
      x += vt * Math.cos(thm) * p.dt; y += vt * Math.sin(thm) * p.dt; th = wrap(th + wt * p.dt); trueW = wt;
    }
    return { p, path, method, seed, log: L, cam: camLog, finish, steerJumps, alarmOn, bCal };
  }

  // ---------- metrics on a time window, relative to a reference (same config without camera loss) ----------
  function score(run, ref, w) {
    const L = run.log, out = {};
    const inW = (t) => t >= w.t0 - 1e-9 && t <= w.t1 + 1e-9;
    let peak = 0, estPeak = 0, survival = NaN, overshoot = 0, lastBad = NaN, neesSum = 0, neesN = 0, nOut = 0, n = 0, se = 0;
    for (let i = 0; i < L.t.length; i++) {
      const t = L.t[i]; if (!inW(t)) continue;
      const ae = Math.abs(L.e[i]); n++; se += ae * ae;
      if (ae > peak) peak = ae;
      if (ae > run.p.corridor) { nOut++; if (isNaN(survival)) survival = t - w.t0; }
      const ex = L.xe[i] - L.x[i], ey = L.ye[i] - L.y[i], ee = Math.hypot(ex, ey);
      if (ee > estPeak) estPeak = ee;
      if (w.lossEnd !== undefined && t >= w.lossEnd) { if (ae > overshoot) overshoot = ae; if (ae > 0.01) lastBad = t; }
      if (w.lossStart !== undefined && t >= w.lossStart && t <= w.lossEnd && (run.method === 'ekf' || run.method === 'ekfm')) {
        const a = L.P00[i], b = L.P01[i], d = L.P11[i], det = a * d - b * b;
        if (det > 0) { neesSum += (d * ex * ex - 2 * b * ex * ey + a * ey * ey) / det; neesN++; }
      }
    }
    let refPeak = NaN;
    if (ref) { refPeak = 0; for (let i = 0; i < ref.log.t.length; i++) if (inW(ref.log.t[i])) refPeak = Math.max(refPeak, Math.abs(ref.log.e[i])); }
    let jump = 0; for (const [t, j] of run.steerJumps) if (inW(t)) jump = Math.max(jump, j);
    const onset = w.faultOnset;
    const alarmsInW = run.alarmOn.filter(inW);
    out.peak = peak; out.refPeak = refPeak; out.excess = peak - refPeak; out.estPeak = estPeak;
    out.corridor = peak > run.p.corridor ? 1 : 0; out.survival = survival; out.fracOut = n ? nOut / n : NaN; out.rms = n ? Math.sqrt(se / n) : NaN;
    out.overshoot = overshoot; out.recovery = isNaN(lastBad) ? 0 : lastBad - w.lossEnd; out.steerJump = jump;
    out.finish = run.finish; out.timeLost = ref ? run.finish - ref.finish : NaN;
    out.nees = neesN ? neesSum / neesN : NaN; out.alarms = alarmsInW.length;
    const first = onset !== undefined ? run.alarmOn.find((t) => t >= onset - 1e-9 && t <= w.t1) : undefined;
    out.alarmDelay = first !== undefined ? first - onset : NaN;
    return out;
  }

  // time when the no-loss robot reaches entry / mid-arc / exit (used to place losses)
  const _phaseCache = {};
  function phaseTimes(over) {
    const key = JSON.stringify(over);
    if (_phaseCache[key]) return _phaseCache[key];
    const run = simulate(Object.assign({}, over, { method: 'ekf', seed: 1, loss: { kind: 'none' }, faults: {} }));
    const P = run.path, at = (s) => { const i = run.log.s.findIndex((v, i) => v >= s && run.log.t[i] > run.p.tStill); return run.log.t[i]; };
    const res = { entry: at(P.L1), mid: at(P.L1 + P.La / 2), exit: at(P.L1 + P.La), finish: run.finish };
    _phaseCache[key] = res;
    return res;
  }

  // ---------- turn an experiment spec into concrete loss slots, fault times and a scoring window ----------
  // spec: {lossSpec:{kind:'pattern',name,phase}|{kind:'blackout',len,phase}|{kind:'ge',lossRate,meanBurst}|{kind:'none'}, fault:'none'|...}
  function resolve(spec) {
    const base = {}; for (const k of Object.keys(spec)) if (k in DEFAULTS) base[k] = spec[k];
    const p = Object.assign({}, DEFAULTS, base), ls = spec.lossSpec || { kind: 'none' };
    let loss = { kind: 'none' }, w;
    if (ls.kind === 'pattern' || ls.kind === 'blackout') {
      const ph = phaseTimes(base);
      let slots, winStart, winEnd;
      if (ls.kind === 'pattern') {
        const start = Math.round((ph[ls.phase] - 1.0) * p.camHz);
        slots = PATTERNS[ls.name].map((i) => start + i); winStart = start / p.camHz; winEnd = winStart + 20 / p.camHz;
      } else {
        const st = ls.phase === 'straight' ? ph.entry - ls.len - 1.0 : ph[ls.phase] - ls.len / 2;
        slots = slotsForBlackout(st, ls.len, p.camHz); winStart = slots[0] / p.camHz; winEnd = (slots[slots.length - 1] + 1) / p.camHz;
      }
      loss = { kind: 'slots', slots };
      const lossStart = slots[0] / p.camHz, lossEnd = (slots[slots.length - 1] + 1) / p.camHz;
      w = { t0: winStart, t1: winEnd + 3.0, lossStart, lossEnd };
    } else if (ls.kind === 'ge') {
      loss = { kind: 'ge', lossRate: ls.lossRate, meanBurst: ls.meanBurst, from: 2.0 };
      w = { t0: 2.0, t1: 1e9 };
    } else w = { t0: 0, t1: 1e9 };
    const onsetT = w.lossStart !== undefined ? w.lossStart : 2.0;
    const faults = faultFor(spec.fault || 'none', onsetT);
    if (faults.gyroStep || faults.slip) w.faultOnset = faults.gyroStep ? faults.gyroStepAt : faults.slip.start;
    return { cfg: Object.assign({}, base, { method: spec.method, seed: spec.seed, loss, faults }), refCfg: Object.assign({}, base, { method: spec.method, seed: spec.seed, loss: { kind: 'none' }, faults }), window: w };
  }
  const FAULTS = {
    none: 'No fault', slip: 'Right wheel slips 10% for 0.5 s', gyro05: 'Gyro drift +0.5 deg/s', gyro10: 'Gyro drift +1 deg/s',
    scale3: 'Both wheels 3% larger', diff1: 'Wheels -1% / +1%',
  };
  function faultFor(name, t) {
    switch (name) {
      case 'none': return {};
      case 'slip': return { slip: { wheel: 'R', factor: 0.9, start: t + 0.3, dur: 0.5 } };
      case 'gyro05': return { gyroStep: 0.5 * D2R, gyroStepAt: t };
      case 'gyro10': return { gyroStep: 1.0 * D2R, gyroStepAt: t };
      case 'scale3': return { scaleL: 0.03, scaleR: 0.03 };
      case 'diff1': return { scaleL: -0.01, scaleR: 0.01 };
      default: throw new Error('unknown fault ' + name);
    }
  }

  // ---------- ESP32 trace export: one run -> C header the firmware replays and checks itself against ----------
  function exportTrace(run, note) {
    const p = run.p, L = run.log, i0 = L.t.findIndex((t) => t >= p.tStill - 1e-9);
    const n = L.t.length - i0, f6 = (v) => (+v).toFixed(6), camOf = {};
    run.cam.j.forEach((j, k) => { camOf[Math.round(run.cam.t[k] * p.sensHz)] = k; });
    const rows = [], cams = [];
    for (let i = i0; i < L.t.length; i++) {
      const si = Math.round(L.t[i] * p.sensHz), ck = camOf[si];
      // camera row applies AFTER the sensor step with the same timestamp (same order as sim.js)
      let camFlag = 0, cz = [0, 0, 0];
      if (ck !== undefined) { camFlag = run.cam.ok[ck] ? 1 : 2; cz = run.cam.z[ck]; }
      rows.push(`  {${L.dnL[i]}, ${L.dnR[i]}, ${f6(L.wg[i])}f, ${camFlag}, ${f6(cz[0])}f, ${f6(cz[1])}f, ${f6(cz[2])}f}`);
    }
    // expected EKF pose after each step = logged estimate at the NEXT sensor sample (it includes the camera update)
    for (let i = i0; i < L.t.length; i++) { const k = Math.min(i + 1, L.t.length - 1); cams.push(`  {${f6(L.xe[k])}f, ${f6(L.ye[k])}f, ${f6(L.the[k])}f}`); }
    return `// AUTO-GENERATED by sim/sim.js exportTrace() - do not edit by hand.\n// ${note || ''}\n` +
      `#pragma once\n#include <stdint.h>\n` +
      `#define TRACE_N ${n}\n#define TRACE_DT ${f6(1 / p.sensHz)}f\n#define TRACE_CAM_EVERY ${Math.round(p.sensHz / p.camHz)}\n` +
      `#define P_R ${f6(p.R)}f\n#define P_L1 ${f6(p.L1)}f\n#define P_L3 ${f6(p.L3)}f\n#define P_V ${f6(p.v)}f\n#define P_LOOK ${f6(p.lookTime * p.v)}f\n` +
      `#define P_TRACK ${f6(p.track)}f\n#define P_WHEEL_R ${f6(p.wheelR)}f\n#define P_CPR ${p.cpr}\n#define P_WHEEL_VMAX ${f6(p.wheelVmax)}f\n` +
      `#define P_CAM_SIG_XY ${f6(p.camSigXY)}f\n#define P_CAM_SIG_TH ${f6(p.camSigTh)}f\n#define P_GYRO_ARW ${p.gyroARW.toExponential(6)}f\n` +
      `#define P_EKF_SCALE_SIG ${f6(p.ekfScaleSig)}f\n#define P_EKF_BIAS_RW ${p.ekfBiasRW.toExponential(6)}f\n` +
      `#define P_MON_WIN_N ${Math.round(p.monWin * p.sensHz)}\n#define P_MON_THR ${f6(p.monThr)}f\n#define P_MON_MIN_GAP ${f6(p.monMinGap)}f\n#define P_MON_SLOW ${f6(p.monSlow)}f\n` +
      `#define INIT_BIAS ${run.bCal.toExponential(8)}f\n#define INIT_P_BIAS ${(Math.pow(p.gyroARW / Math.sqrt(1 / p.sensHz), 2) / Math.max(1, Math.round(p.tStill * p.sensHz) - 1)).toExponential(6)}f\n` +
      `#define INIT_X ${f6(L.xe[i0])}f\n#define INIT_Y ${f6(L.ye[i0])}f\n#define INIT_TH ${f6(L.the[i0])}f\n` +
      `// per 20 ms step: encoder count deltas L/R, gyro rad/s, camera flag (0 none, 1 report, 2 report LOST), camera x y th\n` +
      `struct TraceRow { int16_t dnL, dnR; float gyro; uint8_t cam; float cx, cy, cth; };\n` +
      `static const TraceRow TRACE[TRACE_N] = {\n${rows.join(',\n')}\n};\n` +
      `// browser EKF pose after each step, for the on-device self-check\n` +
      `struct PoseRow { float x, y, th; };\nstatic const PoseRow EXPECT[TRACE_N] = {\n${cams.join(',\n')}\n};\n`;
  }

  // static gyro log for the Allan-variance experiment (robot not moving)
  function staticGyroLog(seconds, seed, over) {
    const p = Object.assign({}, DEFAULTS, over || {}), g = makeGyro(p, seed * 7919 + 1), n = Math.round(seconds * p.sensHz), out = new Float64Array(n), bias = new Float64Array(n);
    for (let i = 0; i < n; i++) { const s = g.sample(0, i / p.sensHz, 0); out[i] = s.w; bias[i] = s.bias; }
    return { fs: p.sensHz, w: out, bias, p };
  }

  root.Sim = { DEFAULTS, METHODS, METHOD_LABEL, PATTERNS, FAULTS, D2R, makePath, makeRng, simulate, score, phaseTimes, resolve, faultFor, exportTrace, staticGyroLog, wrap };
})(typeof self !== 'undefined' ? self : this);
