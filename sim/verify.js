// verify.js: verification checks V1-V5, V7, V10 (PLAN §9) and the synthetic gyro log for V6.
import { DEFAULTS, D2R, PATTERNS } from './params.js';
import { simulate } from './simulate.js';
import { resolve, phaseTimes } from './scenarios.js';
import { makeLoss, staticGyroLog } from './sensors.js';
import { EXPERIMENTS, runSpec, toCSV } from './experiments.js';
import { hashObj } from './util.js';

const P_A = { R: 0.55, v: 0.2, Ld: 0.15 }, P_B = { R: 1.0, v: 0.5, Ld: 0.30 };
const bo = (params, arm, seed, Tb, onset, fault, extra = {}) => ({ exp: 'V', params, arm, seed, loss: { kind: 'blackout', Tb, onset, ...extra }, fault });

function v1() {
  const s = bo(P_B, 'SA', 3, 4, 'entry', { kind: 'gyro', b: 1.0 });
  const a = JSON.stringify(runSpec(s)), b = JSON.stringify(runSpec(s));
  return { id: 'V1', name: 'Determinism', pass: a === b, value: a === b ? 'identical' : 'DIFFERENT', criterion: 'identical rows' };
}

function v2() {
  const cases = [];
  for (const name of Object.keys(PATTERNS)) cases.push({ exp: 'V', params: P_A, arm: 'ekf', seed: 1, loss: { kind: 'pattern', name, phase: 'entry' }, fault: { kind: 'none' } });
  for (const Tb of [0.5, 1, 2, 4]) cases.push(bo(P_B, 'N', 1, Tb, 'exit', { kind: 'none' }));
  let ok = true; const detail = [];
  for (const c of cases) {
    const r = resolve(c), run = simulate(r.cfg);
    const logged = run.cam.j.filter((j, k) => !run.cam.ok[k]);
    const same = logged.length === r.slots.length && logged.every((j, k) => j === r.slots[k]);
    ok = ok && same; detail.push(`${c.loss.name || 'blackout' + c.loss.Tb}:${logged.length}/${r.slots.length}${same ? '' : ' MISMATCH'}`);
  }
  // Gilbert channel: realised loss rate and mean burst over 200,000 slots
  for (const mb of [1, 4, 8, 16]) {
    const lost = makeLoss({ kind: 'ge', lambda: 0.1, meanBurst: mb, from: 0 }, 1, DEFAULTS);
    let n = 0, bursts = 0, inB = false;
    for (let j = 0; j < 200000; j++) { const l = lost(j); if (l) { n++; if (!inB) bursts++; } inB = l; }
    const rate = n / 200000, meanB = n / bursts;
    const good = Math.abs(rate - 0.1) < 0.005 && Math.abs(meanB - mb) / mb < 0.05;
    ok = ok && good; detail.push(`GE${mb}: rate ${rate.toFixed(4)} burst ${meanB.toFixed(2)}${good ? '' : ' OFF'}`);
  }
  return { id: 'V2', name: 'Loss schedule', pass: ok, value: detail.join('; '), criterion: 'logged drops = scheduled; GE rate within 0.005, burst within 5%' };
}

function v3(rows) {
  const cfgs = [{ ...P_A, arms: ['hold', 'dr', 'ekf'] }, { ...P_B, arms: ['hold', 'dr', 'ekf'] }];
  for (const R of [0.55, 1.0, 1.5]) for (const v of [0.3, 0.5, 0.7]) cfgs.push({ R, v, Ld: 0.30, arms: ['ekf'] });
  for (const v of [0.3, 0.5, 0.7]) cfgs.push({ R: 1.0, v, Ld: 0.6 * v, arms: ['ekf'] });
  // PLAN §9: configs at >= 15 mm are excluded from breach metrics and reported. Blocking only for the
  // configs that carry the A / CAL / B0 / B / S hypotheses.
  const blocking = (c) => (c.R === 0.55 && c.v === 0.2 && c.Ld === 0.15) || (c.R === 1.0 && c.v === 0.5 && c.Ld === 0.30);
  let worst = 0; const fails = [], excluded = [];
  for (const c of cfgs) for (const arm of c.arms) for (const seed of [1, 2, 3]) {
    const run = simulate({ R: c.R, v: c.v, Ld: c.Ld, arm, seed, loss: { kind: 'none' }, faults: {} });
    let pk = 0; run.log.t.forEach((t, i) => { if (t >= run.p.tStill + 1) pk = Math.max(pk, Math.abs(run.log.e[i])); });
    rows.push({ R: c.R, v: c.v, Ld: c.Ld, arm, seed, peak_mm: pk * 1e3, finish: run.finish, blocking: blocking(c) ? 1 : 0 });
    if (arm !== 'hold') worst = Math.max(worst, pk);
    const tag = `R${c.R} v${c.v} Ld${c.Ld.toFixed(2)} ${arm} s${seed}: ${(pk * 1e3).toFixed(1)}mm`;
    if (pk >= 0.015) (blocking(c) ? fails : excluded).push(tag);
  }
  return { id: 'V3', name: 'No-loss baselines', pass: fails.length === 0,
    value: `worst DR/EKF ${(worst * 1e3).toFixed(1)} mm; blocking fails ${fails.length}; excluded from breach metrics: ${excluded.length ? excluded.join(', ') : 'none'}`,
    criterion: 'peak < 15 mm on hypothesis configs; others >= 15 mm are excluded from breach metrics', detail: fails };
}

function v4(rows) {
  const specs = [];
  for (const arm of ['N', 'SA', 'SB', 'dr']) specs.push(bo(P_B, arm, 7, 4, 'exit', { kind: 'gyro', b: 1.0 }, { shift: -2 }));   // Codex's centred-exit case
  for (const arm of ['N', 'SA', 'SB', 'SO']) specs.push(bo(P_B, arm, 1, 4, 'entry', { kind: 'gyro', b: 1.0 }));
  for (const arm of ['hold', 'dr', 'ekf']) specs.push({ exp: 'V', params: P_A, arm, seed: 1, loss: { kind: 'pattern', name: 'burst8', phase: 'entry' }, fault: { kind: 'slip' } });
  specs.push({ exp: 'V', params: P_A, arm: 'ekf', seed: 1, loss: { kind: 'pattern', name: 'iso8', phase: 'mid' }, fault: { kind: 'none' } });
  let worst = 0;
  for (const s of specs) {
    const a = runSpec(s), b = runSpec({ ...s, params: { ...s.params, dt: 0.005 } });
    const d = Math.abs(a.excess - b.excess); worst = Math.max(worst, d);
    rows.push({ case: `${s.loss.name || 'Tb' + s.loss.Tb + s.loss.onset + (s.loss.shift ? 'C' : '')}`, arm: s.arm, seed: s.seed, excess10_mm: a.excess * 1e3, excess5_mm: b.excess * 1e3, diff_mm: d * 1e3 });
  }
  return { id: 'V4', name: 'Timestep convergence', pass: worst < 0.001, value: `max |diff| ${(worst * 1e3).toFixed(3)} mm over ${specs.length} cases`, criterion: '< 1 mm (dt 10 ms vs 5 ms)' };
}

function v5() {
  let s = 0, n = 0; const per = [];
  for (let seed = 1; seed <= 10; seed++) {
    const run = simulate({ ...P_B, arm: 'ekf', seed, loss: { kind: 'none' }, faults: {} }), L = run.log;
    let ss = 0, nn = 0;
    for (let i = 0; i < L.t.length; i++) {
      if (L.t[i] < 2) continue;
      const ex = L.xe[i] - L.x[i], ey = L.ye[i] - L.y[i], a = L.P00[i], b = L.P01[i], d = L.P11[i], det = a * d - b * b;
      ss += (d * ex * ex - 2 * b * ex * ey + a * ey * ey) / det; nn++;
    }
    per.push(ss / nn); s += ss; n += nn;
  }
  const m = s / n;
  return { id: 'V5', name: 'EKF consistency (2-D position NEES, no loss)', pass: m >= 1.0 && m <= 3.5, value: `mean ${m.toFixed(2)} (per seed ${per.map((x) => x.toFixed(2)).join(' ')})`, criterion: 'mean in [1.0, 3.5]; consistent = 2' };
}

function v7(rows) {
  // Straight-path blackout (ends 1 s before entry). Fault-induced estimate drift = (faulted - healthy) estimate error, same seed.
  let worst = 0; const notes = [];
  for (const fault of [{ kind: 'gyro', b: 1.0 }, { kind: 'gyro', b: 0.5 }, { kind: 'scale', eps: 0.03 }]) {
    for (let seed = 1; seed <= 5; seed++) {
      const rf = resolve(bo(P_B, 'N', seed, 4, 'pre', fault)), rh = resolve(bo(P_B, 'N', seed, 4, 'pre', { kind: 'none' }));
      const runF = simulate(rf.cfg), runH = simulate(rh.cfg), Lf = runF.log, Lh = runH.log;
      // drift = increment of the (faulted - healthy) estimate error since the drift starts (fault onset, or
      // blackout onset for persistent faults), sampled strictly before the camera returns
      const tOn = rf.W.tOn, tF = isFinite(rf.tF) && rf.tF > 0 ? rf.tF : tOn;
      const idx = (L, t) => L.t.findIndex((x) => x >= t - 1e-9);
      const errAt = (t) => { const i = idx(Lf, t), ih = idx(Lh, t); return [(Lf.ye[i] - Lf.y[i]) - (Lh.ye[ih] - Lh.y[ih]), (Lf.xe[i] - Lf.x[i]) - (Lh.xe[ih] - Lh.x[ih])]; };
      const e0 = errAt(tF);
      for (const tau of [1, 2, 3, 3.4]) {
        const t = tF + tau;
        if (t >= rf.W.tRet - 1e-9) continue;
        const et = errAt(t), lat = et[0] - e0[0], along = et[1] - e0[1];
        let pred, meas, comp;
        if (fault.kind === 'gyro') { pred = 0.5 * 0.5 * fault.b * D2R * tau * tau; meas = lat; comp = 'lateral'; }
        else { pred = -(fault.eps / (1 + fault.eps)) * 0.5 * tau; meas = along; comp = 'along-track'; }
        const rel = Math.abs(meas - pred) / Math.abs(pred); worst = Math.max(worst, rel);
        rows.push({ fault: fault.kind + (fault.b ?? fault.eps), seed, tau, component: comp, pred_mm: pred * 1e3, meas_mm: meas * 1e3, rel_err: rel,
          lateral_mm: lat * 1e3, offset_at_start_lat_mm: e0[0] * 1e3, offset_at_start_along_mm: e0[1] * 1e3 });
      }
    }
  }
  return { id: 'V7', name: 'Drift envelopes on a straight (estimate drift vs PLAN §6.3)', pass: worst < 0.10, value: `max relative error ${(worst * 100).toFixed(1)}%`, criterion: 'within 10% for tau = 1-3.4 s, before camera return' };
}

function v10() {
  const out = []; let bad = 0;
  for (const id of ['A', 'CAL', 'B0', 'B', 'C', 'S']) {
    let n = 0, t = 0;
    for (const s of EXPERIMENTS[id].specs()) {
      const r = resolve(s), fin = phaseTimes({ ...DEFAULTS, ...s.params }).finish; t++;
      if (isFinite(r.W.t1) && r.W.t1 > fin - 0.1) n++;
    }
    bad += n; out.push(`${id}: ${n}/${t}`);
  }
  return { id: 'V10', name: 'Score windows inside the route', pass: bad === 0, value: out.join('; '), criterion: 'no window ends after the no-loss finish' };
}

export function runVerification(onProgress) {
  const t0 = Date.now(), r3 = [], r4 = [], r7 = [], checks = [];
  const steps = [v1, v2, () => v3(r3), () => v4(r4), v5, () => v7(r7), v10];
  steps.forEach((f, i) => { checks.push(f()); if (onProgress) onProgress(i + 1, steps.length); });
  checks.push({ id: 'V6', name: 'Gyro generator (Allan deviation)', pass: null, value: 'computed in analysis/allan.py', criterion: 'ARW within 10%, bias-instability floor within 30%' });
  checks.push({ id: 'V8', name: 'Firmware parity', pass: null, value: 'M4', criterion: 'pose < 1 mm, same alarms' });
  checks.push({ id: 'V9', name: 'Independent reference (Codex)', pass: null, value: 'pending Codex', criterion: 'pose agrees < 1e-6 m' });
  const w = staticGyroLog(DEFAULTS, 3600, 1);
  const gyroCsv = 'w_rad_s\n' + Array.from(w, (x) => x.toPrecision(10)).join('\n') + '\n';
  const meta = { created: new Date().toISOString(), seconds: (Date.now() - t0) / 1000, paramsHash: hashObj(DEFAULTS), defaults: DEFAULTS };
  return {
    v: { folder: 'v_verification', files: { 'checks.json': JSON.stringify({ meta, checks }, null, 2), 'v3_baselines.csv': toCSV(r3), 'v4_dt.csv': toCSV(r4), 'v7_envelopes.csv': toCSV(r7) } },
    imu: { folder: 'imu_static', files: { 'gyro_static_1h.csv': gyroCsv, 'meta.json': JSON.stringify({ ...meta, fs: DEFAULTS.sensHz, seed: 1, injected: { ARW: DEFAULTS.gyroARW, GMsig: DEFAULTS.gyroGMsig, GMtau: DEFAULTS.gyroGMtau, warmAmp: DEFAULTS.gyroWarmAmp, warmTau: DEFAULTS.gyroWarmTau } }, null, 2) } },
  };
}
