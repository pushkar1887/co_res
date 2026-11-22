// scenarios.js: experiment spec -> concrete loss slots, fault times and score window (PLAN §7 rules).
// spec = { params:{R,v,Ld,...overrides}, arm, seed,
//          loss:{kind:'pattern',name,phase} | {kind:'blackout',Tb,onset,shift?} | {kind:'ge',lambda,meanBurst} | {kind:'none'},
//          fault:{kind:'none'} | {kind:'slip'} | {kind:'gyro',b} | {kind:'scale',eps} | {kind:'diff',delta} }
import { DEFAULTS, PATTERNS, D2R } from './params.js';
import { simulate } from './simulate.js';

const phaseCache = new Map();
// When the no-loss robot reaches entry, mid-arc and exit. Uses only geometry and speed, so loss
// slots stay fixed when dt or noise settings change (needed for V4).
export function phaseTimes(p) {
  const g = { R: p.R, v: p.v, Ld: p.Ld, L1: p.L1, L3: p.L3 }, key = JSON.stringify(g);
  if (phaseCache.has(key)) return phaseCache.get(key);
  const run = simulate({ ...g, arm: 'ekf', seed: 1, loss: { kind: 'none' }, faults: {} });
  const at = (s) => { const L = run.log; for (let i = 0; i < L.t.length; i++) if (L.s[i] >= s && L.t[i] > run.p.tStill) return L.t[i]; return NaN; };
  const res = { entry: at(g.L1), mid: at(g.L1 + run.path.La / 2), exit: at(g.L1 + run.path.La), finish: run.finish };
  phaseCache.set(key, res);
  return res;
}

export function faultFor(fault, tOn, Tb) {
  const k = fault ? fault.kind : 'none';
  if (k === 'none') return {};
  if (k === 'slip') return { slip: { wheel: 'R', factor: 0.9, start: tOn + 0.3, dur: 0.5 }, onset: tOn + 0.3 };
  if (k === 'gyro') { const tF = tOn + Math.min(0.5, (isFinite(Tb) ? Tb : 2.0) / 4); return { gyroStep: fault.b * D2R, gyroAt: tF, onset: tF }; }
  if (k === 'scale') return { scaleL: fault.eps, scaleR: fault.eps, onset: 0 };
  if (k === 'diff') return { scaleL: -fault.delta / 2, scaleR: fault.delta / 2, onset: 0 };
  throw new Error('unknown fault ' + k);
}

export const faultName = (f) => (!f || f.kind === 'none' ? 'none' : f.kind === 'slip' ? 'slip'
  : f.kind === 'gyro' ? `gyro${f.b}` : f.kind === 'scale' ? `scale${Math.round(f.eps * 100)}` : `diff${Math.round(f.delta * 100)}`);

export function resolve(spec) {
  const over = { ...(spec.params || {}) }, p = { ...DEFAULTS, ...over }, ls = spec.loss || { kind: 'none' };
  let loss = { kind: 'none' }, tOn = NaN, tRet = NaN, Tb = NaN, slots = null;
  if (ls.kind === 'pattern' || ls.kind === 'blackout') {
    const ph = phaseTimes(p);
    if (ls.kind === 'pattern') {
      const j0 = Math.round((ph[ls.phase] - 1.0) * p.camHz);        // 2 s window centred on the landmark
      slots = PATTERNS[ls.name].map((i) => j0 + i);
    } else {
      const raw = (ls.onset === 'pre' ? ph.entry - ls.Tb - 1.0 : ph[ls.onset]) + (ls.shift || 0);
      const j0 = Math.round(raw * p.camHz), n = Math.round(ls.Tb * p.camHz);
      slots = Array.from({ length: n }, (_, i) => j0 + i); Tb = ls.Tb;
    }
    loss = { kind: 'slots', slots };
    tOn = slots[0] / p.camHz; tRet = (slots[slots.length - 1] + 1) / p.camHz;
  } else if (ls.kind === 'ge') loss = { kind: 'ge', lambda: ls.lambda, meanBurst: ls.meanBurst, from: 2.0 };
  const W = isFinite(tOn) ? { t0: tOn, t1: tRet + p.scoreTail, tOn, tRet } : { t0: 2.0, t1: Infinity };
  const faults = faultFor(spec.fault, isFinite(tOn) ? tOn : 2.0, Tb);
  const cfg = { ...over, arm: spec.arm, seed: spec.seed, loss, faults };
  return { cfg, refCfg: { ...cfg, loss: { kind: 'none' } }, W, tF: faults.onset ?? Infinity, slots };
}
