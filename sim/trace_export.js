// trace_export.js: turns one simulated run into C headers that the ESP32 replays (PLAN §8, check V8).
//
// Per 20 ms sensor step i the trace holds the INPUTS the firmware sees (encoder count deltas, gyro sample,
// camera flag + pose) and the browser's REFERENCE at the SAME timestamp, taken after prediction, camera
// correction and steering (Codex review 1, F8): estimated pose, monitor residual D, alarm, speed and turn
// commands, wheel commands. The trace starts at t = 0, so the firmware calibrates the gyro bias itself.
import { DEFAULTS } from './params.js';
import { resolve } from './scenarios.js';
import { simulate } from './simulate.js';
import { makePath } from './route.js';
import { ekfInitP, ekfPredict, ekfUpdate } from './estimators.js';
import { makeMonitor } from './monitor.js';
import { purePursuit } from './control.js';
import { clamp, wrap, hashObj } from './util.js';

const P_B = { R: 1.0, v: 0.5, Ld: 0.30 };
// The B0 primary cell (PLAN §2): 4 s blackout from arc entry, policy SA, seed 1.
export const TRACE_SPECS = {
  healthy:    { params: P_B, arm: 'SA', seed: 1, loss: { kind: 'blackout', Tb: 4, onset: 'entry' }, fault: { kind: 'none' } },
  gyro_fault: { params: P_B, arm: 'SA', seed: 1, loss: { kind: 'blackout', Tb: 4, onset: 'entry' }, fault: { kind: 'gyro', b: 1.0 } },
};

export function buildTrace(name) {
  const spec = TRACE_SPECS[name], r = resolve(spec), run = simulate(r.cfg), p = run.p, L = run.log;
  const camEvery = Math.round(p.sensHz / p.camHz), N = L.t.filter((t) => t <= r.W.t1 + 1e-9).length;
  const camRow = new Map(run.cam.j.map((j, k) => [j * camEvery, k]));
  const rows = [], ref = [];
  for (let i = 0; i < N; i++) {
    const k = camRow.get(i), cam = k === undefined ? 0 : run.cam.ok[k] ? 1 : 2, z = k === undefined ? [0, 0, 0] : run.cam.z[k];
    rows.push({ dnL: L.dnL[i], dnR: L.dnR[i], gyro: L.wg[i], cam, cx: cam ? z[0] : 0, cy: cam ? z[1] : 0, cth: cam ? z[2] : 0 });
    ref.push({ x: L.xe[i], y: L.ye[i], th: L.the[i], D: L.D[i], uL: L.uLc[i], uR: L.uRc[i], vc: L.vc[i], wc: L.wc[i], alarm: L.alarm[i] });
  }
  const tick = (t) => Math.round(t * p.sensHz);
  const meta = { name, N, onTick: tick(r.W.tOn), retTick: tick(r.W.tRet), faultTick: isFinite(r.tF) ? tick(r.tF) : -1, seed: spec.seed, arm: spec.arm };
  return { spec, p, rows, ref, meta };
}

// Replays a trace with the firmware's control flow (integer ticks) using the simulator's own functions.
// It must reproduce the reference to rounding error. That proves the trace carries everything the
// firmware needs and that the firmware's step order is right.
export function replayCheck(trace) {
  const p = trace.p, path = makePath(p.R, p.L1, p.L3), dtS = 1 / p.sensHz, radPerCount = 2 * Math.PI / p.cpr;
  const camEvery = Math.round(p.sensHz / p.camHz), steerEvery = Math.round(p.sensHz / p.steerHz);
  const tStillTicks = Math.round(p.tStill * p.sensHz), gateTicks = Math.round(p.monGate * p.sensHz);
  const kc = p.camNoiseScale, Rd = [(kc * p.camSigXY) ** 2, (kc * p.camSigXY) ** 2, (kc * p.camSigTh) ** 2];
  const X = [0, 0, 0, 0], P = ekfInitP();
  let calibrated = false, calSum = 0, calN = 0, monitor = null, missed = 0, tFixTick = 0, alarm = false, D = 0, vc = 0, wc = 0;
  let maxPos = 0, maxTh = 0, maxD = 0, maxCmd = 0, alarmDiff = 0;
  trace.rows.forEach((row, i) => {
    const moving = i >= tStillTicks, dL = row.dnL * radPerCount * p.wheelR, dR = row.dnR * radPerCount * p.wheelR, wg = row.gyro;
    if (!calibrated) {
      if (!moving) { calSum += wg; calN++; }
      else { const b = calSum / Math.max(calN, 1); calibrated = true; X[3] = b; monitor = makeMonitor(p, b); P[3][3] = (p.gyroNoiseScale * p.gyroARW) ** 2 / (dtS * Math.max(calN, 1)); }
    }
    if (calibrated) {
      ekfPredict(X, P, 0.5 * (dL + dR), wg, dtS, p);
      D = monitor(wg, dL, dR, missed === 0).D;
      const missAge = missed > 0 ? i - (tFixTick + camEvery) : -1;
      if (moving && !alarm && missAge >= gateTicks && Math.abs(D) > p.monThr) alarm = true;
    }
    if (i % camEvery === 0) {
      if (row.cam === 1) { tFixTick = i; missed = 0; alarm = false; ekfUpdate(X, P, [row.cx, row.cy, row.cth], Rd); }
      else if (row.cam === 2) missed++;
    }
    if (i % steerEvery === 0 && moving) { vc = alarm ? p.alpha * p.v : p.v; wc = purePursuit(path, X, p.Ld, vc); }
    const R = trace.ref[i];
    const uL = clamp(vc - wc * p.track / 2, -p.wheelVmax, p.wheelVmax), uR = clamp(vc + wc * p.track / 2, -p.wheelVmax, p.wheelVmax);
    maxPos = Math.max(maxPos, Math.hypot(X[0] - R.x, X[1] - R.y)); maxTh = Math.max(maxTh, Math.abs(wrap(X[2] - R.th)));
    maxD = Math.max(maxD, Math.abs(D - R.D)); maxCmd = Math.max(maxCmd, Math.abs(uL - R.uL), Math.abs(uR - R.uR));
    if ((alarm ? 1 : 0) !== R.alarm) alarmDiff++;
  });
  return { rows: trace.rows.length, maxPos_m: maxPos, maxTh_rad: maxTh, maxD_rad: maxD, maxCmd_mps: maxCmd, alarmRowsDiffering: alarmDiff };
}

const f9 = (x) => `${(+x).toPrecision(9)}f`, f8 = (x) => `${(+x).toPrecision(8)}f`;

export function traceHeader(trace) {
  const { name, N, onTick, retTick, faultTick, seed, arm } = trace.meta, U = name.toUpperCase();
  const rows = trace.rows.map((r) => `  {${r.dnL},${r.dnR},${f9(r.gyro)},${r.cam},${r.cam ? `${f9(r.cx)},${f9(r.cy)},${f9(r.cth)}` : '0,0,0'}}`);
  const refs = trace.ref.map((r) => `  {${f8(r.x)},${f8(r.y)},${f8(r.th)},${f8(r.D)},${f8(r.uL)},${f8(r.uR)},${f8(r.vc)},${f8(r.wc)},${r.alarm}}`);
  return `// AUTO-GENERATED by sim/trace_export.js (web/demo.html -> "Export ESP32 traces"). Do not edit.
// Scenario "${name}": R = ${trace.p.R} m, v = ${trace.p.v} m/s, 4 s blackout from arc entry, arm ${arm}, seed ${seed}.
// All data is simulated. Rows are 20 ms sensor steps from t = 0 (stationary calibration included).
#pragma once
#include "estimator.h"
static const int TRACE_${U}_N = ${N};
static const int TRACE_${U}_ON_TICK = ${onTick};       // first missed camera report
static const int TRACE_${U}_RET_TICK = ${retTick};     // first delivered report after the blackout
static const int TRACE_${U}_FAULT_TICK = ${faultTick}; // recorded gyro fault onset (-1 = healthy)
static const TraceRow TRACE_${U}[${N}] = {
${rows.join(',\n')}
};
static const RefRow REF_${U}[${N}] = {
${refs.join(',\n')}
};
`;
}

export function paramsHeader(p) {
  const D = (k, v) => `#define ${k} ${v}`;
  return [`// AUTO-GENERATED by sim/trace_export.js from sim/params.js (params hash ${hashObj(DEFAULTS)}). Do not edit.`,
    '// The firmware takes every constant from here, so it can never drift from the simulator.',
    '#pragma once',
    D('P_R', f9(p.R)), D('P_L1', f9(p.L1)), D('P_L3', f9(p.L3)), D('P_V', f9(p.v)), D('P_LD', f9(p.Ld)),
    D('P_TRACK', f9(p.track)), D('P_WHEEL_R', f9(p.wheelR)), D('P_CPR', p.cpr), D('P_WHEEL_VMAX', f9(p.wheelVmax)),
    D('P_SENS_HZ', p.sensHz), D('P_CAM_EVERY', Math.round(p.sensHz / p.camHz)), D('P_STEER_EVERY', Math.round(p.sensHz / p.steerHz)),
    D('P_TSTILL_TICKS', Math.round(p.tStill * p.sensHz)),
    D('P_CAM_SIG_XY', f9(p.camNoiseScale * p.camSigXY)), D('P_CAM_SIG_TH', f9(p.camNoiseScale * p.camSigTh)),
    D('P_GYRO_ARW', f9(p.gyroNoiseScale * p.gyroARW)), D('P_EKF_SCALE_SIG', f9(p.ekfScaleSig)), D('P_EKF_BIAS_RW', f9(p.ekfBiasRW)),
    D('P_MON_WIN_N', Math.round(p.monWin * p.sensHz)), D('P_MON_THR', f9(p.monThr)), D('P_MON_GATE_TICKS', Math.round(p.monGate * p.sensHz)),
    D('P_MON_REF_TAU', f9(p.monRefTau)), D('P_ALPHA', f9(p.alpha)),
    D('P_FAULT_STEP', f9(Math.PI / 180)) + '   // fault button: +1 deg/s on the replayed gyro', ''].join('\n');
}

export function exportAll() {
  const traces = Object.keys(TRACE_SPECS).map(buildTrace);
  return {
    files: { 'sim_params.h': paramsHeader(traces[0].p), ...Object.fromEntries(traces.map((t) => [`trace_${t.meta.name}.h`, traceHeader(t)])) },
    checks: Object.fromEntries(traces.map((t) => [t.meta.name, { ...replayCheck(t), meta: t.meta }])),
  };
}
