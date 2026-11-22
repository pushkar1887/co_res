// experiments.js: the grids of PLAN §7 and a runner that turns a grid into CSV rows.
import { DEFAULTS } from './params.js';
import { simulate } from './simulate.js';
import { resolve, faultName } from './scenarios.js';
import { score } from './metrics.js';
import { hashObj } from './util.js';

const range = (a, b) => Array.from({ length: b - a + 1 }, (_, i) => a + i);
const P_A = { R: 0.55, v: 0.2, Ld: 0.15 };
const P_B = { R: 1.0, v: 0.5, Ld: 0.30 };
const ONSETS = ['pre', 'entry', 'mid', 'exit'];
const B_FAULTS = [{ kind: 'none' }, { kind: 'slip' }, { kind: 'gyro', b: 0.5 }, { kind: 'gyro', b: 1.0 }, { kind: 'scale', eps: 0.03 }, { kind: 'diff', delta: 0.02 }];

function* product(spec) {            // spec: {key: [values]} -> objects
  const keys = Object.keys(spec);
  const rec = function* (i, acc) {
    if (i === keys.length) { yield { ...acc }; return; }
    for (const v of spec[keys[i]]) yield* rec(i + 1, { ...acc, [keys[i]]: v });
  };
  yield* rec(0, {});
}
const grid = (exp, params, axes, mk) => [...product(axes)].map((a) => ({ exp, ...mk(a, params) }));

const b0Axes = { Tb: [2, 4], onset: ['entry', 'mid'], fault: [{ kind: 'none' }, { kind: 'gyro', b: 1.0 }], arm: ['N', 'SB', 'SA', 'SO'] };
const blackoutSpec = (a, params) => ({ params: { ...params, ...(a.over || {}) }, arm: a.arm, seed: a.seed, loss: { kind: 'blackout', Tb: a.Tb, onset: a.onset }, fault: a.fault, variant: a.variant || '' });

export const EXPERIMENTS = {
  A: { folder: 'a_estimators_short_gaps', specs: () => grid('A', P_A,
    { name: ['iso8', '2x4', 'burst8'], phase: ['entry', 'mid', 'exit'], fault: [{ kind: 'none' }, { kind: 'slip' }], arm: ['hold', 'dr', 'ekf'], seed: range(1, 20) },
    (a, params) => ({ params, arm: a.arm, seed: a.seed, loss: { kind: 'pattern', name: a.name, phase: a.phase }, fault: a.fault })) },
  CAL: { folder: 'cal_monitor', specs: () => grid('CAL', P_B,
    { Tb: [0.5, 1, 2, 4], onset: ONSETS, fault: [{ kind: 'none' }], arm: ['N'], seed: range(101, 130) }, blackoutSpec) },
  B0: { folder: 'b0_policy_primary', specs: () => grid('B0', P_B, { ...b0Axes, seed: range(1, 30) }, blackoutSpec) },
  B: { folder: 'b_fault_map', specs: () => grid('B', P_B,
    { Tb: [0.5, 1, 2, 4], onset: ONSETS, fault: B_FAULTS, arm: ['dr', 'N', 'SB', 'SA', 'SO'], seed: range(1, 10) }, blackoutSpec) },
  C: { folder: 'c_scaling', specs: () => [
    ...grid('C1', {}, { R: [0.55, 1.0, 1.5], v: [0.3, 0.5, 0.7], fault: [{ kind: 'gyro', b: 1.0 }, { kind: 'scale', eps: 0.03 }], arm: ['N', 'SA', 'SO'], seed: range(1, 10) },
      (a) => blackoutSpec({ ...a, Tb: 4, onset: 'entry', over: { R: a.R, v: a.v, Ld: 0.30 } }, {})),
    ...grid('C2', P_B, { b: [0.2, 0.3, 0.4, 0.5, 0.75, 1.0, 1.5, 2.0], arm: ['N', 'SA', 'SO'], seed: range(1, 10) },
      (a, params) => blackoutSpec({ ...a, Tb: 4, onset: 'entry', fault: { kind: 'gyro', b: a.b } }, params)),
    ...grid('C3', {}, { v: [0.3, 0.5, 0.7], fault: [{ kind: 'gyro', b: 1.0 }, { kind: 'scale', eps: 0.03 }], arm: ['N', 'SA'], seed: range(1, 10) },
      (a) => blackoutSpec({ ...a, Tb: 4, onset: 'entry', over: { R: 1.0, v: a.v, Ld: 0.6 * a.v } }, {})),
  ] },
  D: { folder: 'd_burstiness', specs: () => grid('D', P_B, { meanBurst: [1, 4, 8, 16], arm: ['hold', 'dr', 'ekf'], seed: range(1, 100) },
    (a, params) => ({ params, arm: a.arm, seed: a.seed, loss: { kind: 'ge', lambda: 0.10, meanBurst: a.meanBurst }, fault: { kind: 'none' } })) },
  S: { folder: 's_sensitivity', specs: () => [
    ...grid('S1', P_B, { ...b0Axes, seed: range(1, 10) }, (a, p) => blackoutSpec({ ...a, variant: 'gyroNoise2', over: { gyroNoiseScale: 2 } }, p)),
    ...grid('S2', P_B, { ...b0Axes, seed: range(1, 10) }, (a, p) => blackoutSpec({ ...a, variant: 'camNoise2', over: { camNoiseScale: 2 } }, p)),
    ...grid('S3', P_B, { ...b0Axes, arm: ['SB', 'SA'], alpha: [0.2, 0.5], seed: range(1, 10) }, (a, p) => blackoutSpec({ ...a, variant: `alpha${a.alpha}`, over: { alpha: a.alpha } }, p)),
    ...grid('S4', P_B, { ...b0Axes, seed: range(1, 10) }, (a, p) => blackoutSpec({ ...a, variant: 'steer50', over: { steerHz: 50 } }, p)),
  ] },
};

export function specRow(spec, r) {
  const p = { ...DEFAULTS, ...(spec.params || {}) }, ls = spec.loss || {};
  return {
    exp: spec.exp, variant: spec.variant || '', arm: spec.arm, seed: spec.seed, R: p.R, v: p.v, Ld: p.Ld,
    loss: ls.kind, pattern: ls.name || '', phase: ls.phase || ls.onset || '', Tb: ls.Tb ?? '', meanBurst: ls.meanBurst ?? '',
    fault: faultName(spec.fault), tOn: r.W.tOn ?? '', tRet: r.W.tRet ?? '', tF: isFinite(r.tF) ? r.tF : '', t1: isFinite(r.W.t1) ? r.W.t1 : '',
  };
}

// Run one spec against its no-loss reference (same seed, arm, faults and fault timing)
export function runSpec(spec) {
  const r = resolve(spec), run = simulate(r.cfg), ref = simulate(r.refCfg);
  return { ...specRow(spec, r), ...score(run, ref, r.W, r.tF), totalSlots: run.cam.ok.length };
}

export function toCSV(rows) {
  const keys = Object.keys(rows[0]);
  const fmt = (v) => (typeof v === 'number' ? (Number.isFinite(v) ? (Number.isInteger(v) ? String(v) : v.toPrecision(8)) : '') : String(v));
  return keys.join(',') + '\n' + rows.map((r) => keys.map((k) => fmt(r[k])).join(',')).join('\n') + '\n';
}

export function runExperiment(id, onProgress) {
  const ex = EXPERIMENTS[id]; if (!ex) throw new Error('unknown experiment ' + id);
  const specs = ex.specs(), rows = [], t0 = Date.now();
  specs.forEach((s, i) => { rows.push(runSpec(s)); if (onProgress && (i % 50 === 0 || i === specs.length - 1)) onProgress(i + 1, specs.length); });
  const meta = { experiment: id, folder: ex.folder, runs: rows.length, seconds: (Date.now() - t0) / 1000, created: new Date().toISOString(),
    paramsHash: hashObj(DEFAULTS), defaults: DEFAULTS, note: 'All data simulated (synthetic). See PLAN.md §7.' };
  return { folder: ex.folder, files: { 'runs.csv': toCSV(rows), 'meta.json': JSON.stringify(meta, null, 2) } };
}
