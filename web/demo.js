// demo.js: viva demo v2. Every policy is simulated by sim/ with the SAME seed, camera blackout and sensor
// fault; only the speed policy differs. Two follow-cameras compare a pair of policies; the ghost robot is the
// EKF estimate with its 2-sigma position ellipse; the story list and presenter mode walk through the events.
import { DEFAULTS } from '../sim/params.js';
import { resolve } from '../sim/scenarios.js';
import { simulate } from '../sim/simulate.js';
import { score } from '../sim/metrics.js';
import { makePath } from '../sim/route.js';
import { wrap } from '../sim/util.js';
import { exportAll } from '../sim/trace_export.js';

const D2R = Math.PI / 180, CORR = DEFAULTS.corridor, H_DEG = DEFAULTS.monThr / D2R;
const P_B = { R: 1.0, v: 0.5, Ld: 0.30 };
const ARMS = {
  N:  { name: 'N',  title: 'Never slows down', short: 'never slows', color: '#ef4444' },
  SB: { name: 'SB', title: 'Slows on every blackout', short: 'slows every blackout', color: '#94a3b8' },
  SA: { name: 'SA', title: 'Slows only when sensors disagree (ours)', short: 'slows on disagreement (ours)', color: '#3b82f6' },
  SO: { name: 'SO', title: 'Oracle: knows exactly when the fault starts', short: 'oracle', color: '#22c55e' },
  dr: { name: 'DR', title: 'Dead reckoning, no Kalman filter', short: 'no Kalman filter', color: '#f59e0b' },
};
const ORDER = ['N', 'SB', 'SA', 'SO', 'dr'];
const PRESETS = [
  { id: 'none', label: 'Healthy sensors', fault: { kind: 'none' }, chip: null },
  { id: 'gyro1', label: 'Gyro drift +1°/s', fault: { kind: 'gyro', b: 1.0 }, chip: 'GYRO DRIFT +1°/s' },
  { id: 'gyro05', label: 'Gyro drift +0.5°/s', fault: { kind: 'gyro', b: 0.5 }, chip: 'GYRO DRIFT +0.5°/s' },
  { id: 'slip', label: 'Wheel slip', fault: { kind: 'slip' }, chip: 'RIGHT WHEEL SLIPPING' },
  { id: 'scale3', label: 'Wheels 3% too big', fault: { kind: 'scale', eps: 0.03 }, chip: 'WHEELS 3% TOO BIG' },
  { id: 'diff2', label: 'Wheels −1% / +1%', fault: { kind: 'diff', delta: 0.02 }, chip: 'WHEELS −1% / +1%' },
];
const ONSET_TXT = { pre: 'on the straight', entry: 'at the turn entry', mid: 'mid-turn', exit: 'at the turn exit' };
const route = makePath(P_B.R, DEFAULTS.L1, DEFAULTS.L3);
// warehouse racks [x0, y0, x1, y1, label] in metres, all >= 0.45 m from the lane centre
const RACKS = [
  [0.0, 0.45, 1.25, 0.78], [1.45, 0.45, 2.42, 0.78], [0.0, -0.78, 1.25, -0.45], [1.45, -0.78, 2.7, -0.45], [2.9, -0.78, 4.78, -0.45],
  [2.55, 0.55, 3.45, 1.45, 'RACK A'], [4.45, -0.25, 4.78, 2.6], [4.45, 2.8, 4.78, 6.3], [3.22, 1.6, 3.55, 3.4], [3.22, 3.6, 3.55, 6.3],
];
const CAMERA_POS = [1.6, 2.4];

const $ = (id) => document.getElementById(id);
const st = { preset: 'gyro1', Tb: 4, onset: 'entry', seed: 1, T: 0, playing: false, speed: 1, presenter: false,
  last: 0, sims: {}, W: null, tF: Infinity, t0: 0, t1: 1, events: [], cap: null, capUntil: 0, tlCache: null, dragging: false };

// ---------------------------------------------------------------- simulation
function runAll() {
  const pre = PRESETS.find((p) => p.id === st.preset);
  for (const arm of ORDER) {
    const r = resolve({ params: P_B, arm, seed: st.seed, loss: { kind: 'blackout', Tb: st.Tb, onset: st.onset }, fault: pre.fault });
    const run = simulate(r.cfg), ref = simulate(r.refCfg);
    st.sims[arm] = { arm, r, run, sc: score(run, ref, r.W, r.tF) };
  }
  const R = st.sims.SA.r;
  st.W = R.W; st.tF = R.tF; st.t0 = Math.max(1.2, R.W.tOn - 2.5); st.t1 = R.W.t1 + 0.6;
  st.T = st.t0; st.playing = false; setPlayIcon();
  buildEvents(pre); renderScores(); renderEvents(); st.tlCache = null;
  showCaption({ t: st.t0, h: `${pre.label} · ${st.Tb} s camera blackout ${ONSET_TXT[st.onset]}`,
    b: 'Five robots, five speed policies: N never slows, SB slows on every blackout, SA slows only when its sensors disagree (ours), SO is an oracle, DR has no Kalman filter. Same seed, same sensor noise, same blackout.',
    k: 'Press Space to play. Presenter mode pauses at every event.' }, performance.now() + 9000);
  draw();
}

function sample(run, T) {
  const L = run.log, n = L.t.length, f = Math.max(0, Math.min(n - 1.0001, T * run.p.sensHz)), i = Math.floor(f), a = f - i, j = Math.min(i + 1, n - 1);
  const li = (k) => L[k][i] + (L[k][j] - L[k][i]) * a, an = (k) => L[k][i] + wrap(L[k][j] - L[k][i]) * a;
  return { i, x: li('x'), y: li('y'), th: an('th'), xe: li('xe'), ye: li('ye'), the: an('the'), e: li('e'), s: li('s'), vc: L.vc[i],
    u: 0.5 * (li('uL') + li('uR')), D: L.D[i], alarm: L.alarm[i], P00: L.P00[i], P01: L.P01[i], P11: L.P11[i], missAge: L.missAge[i] };
}
const inBlackout = (T) => T >= st.W.tOn - 1e-9 && T < st.W.tRet - 1e-9;
const faultActive = (T) => isFinite(st.tF) && T >= st.tF - 1e-9;
const firstBreach = (arm) => { const L = st.sims[arm].run.log; for (let i = 0; i < L.t.length; i++) if (L.t[i] >= st.W.tOn && L.t[i] <= st.W.t1 && Math.abs(L.e[i]) > CORR) return L.t[i]; return null; };

// ---------------------------------------------------------------- story
function buildEvents(pre) {
  const W = st.W, ev = [], SA = st.sims.SA, f = pre.fault;
  ev.push({ t: W.tOn, h: 'Camera blackout', b: 'Rack A blocks the overhead camera. The robot now drives on wheel encoders + gyro alone: dead reckoning inside the EKF. Nothing can correct its estimate until the camera returns.' });
  if (pre.id === 'none') ev.push({ t: W.tOn + DEFAULTS.monGate, h: 'Blanket rule vs. sensor check', b: 'SB slows down after 0.3 s of blackout, as today\'s robots do. SA keeps full speed: the gyro and the encoders still agree, so there is nothing to fear.' });
  if (f.kind === 'gyro') ev.push({ t: st.tF, h: `Sensor fault: gyro drifts +${f.b}°/s`, b: 'The gyro starts reporting a turn that is not happening. The heading estimate (ghost robot) rotates away from the truth, and the estimated position drifts with it.' });
  if (f.kind === 'slip') ev.push({ t: st.tF, h: 'Sensor fault: right wheel slips', b: 'A slippery patch: the right wheel spins 10% faster than the ground moves for 0.5 s. The encoders count motion that never happened; the gyro sees the real yaw.' });
  if (f.kind === 'scale') ev.push({ t: W.tOn + 0.05, h: 'Hidden fault: wheels 3% bigger', b: 'Worn or replaced tyres: the robot under-reads distance. On a straight this is harmless; in a turn the robot swings 3% wider. The fault was there all along. The camera hid it.' });
  if (f.kind === 'diff') ev.push({ t: W.tOn + 0.05, h: 'Harmless calibration error', b: 'Left wheel 1% small, right 1% large. Heading comes from the gyro, so tracking is fine, but the encoders now disagree with the gyro. Watch whether SA slows down for nothing.' });
  const a = SA.run.alarmOn.find((t) => t >= W.tOn && t < W.tRet);
  if (a !== undefined) ev.push({ t: a, h: 'Monitor alarm → SA slows to 0.15 m/s', b: `Gyro heading and encoder heading disagree by more than ${H_DEG.toFixed(1)}° over the last 2 s.${isFinite(st.tF) && st.tF > 0 ? ` Detected ${(a - st.tF).toFixed(2)} s after the fault.` : ''} SA caps its speed: drift grows with speed × time, so slowing buys safety.` });
  else if (pre.id !== 'none') ev.push({ t: W.tOn + 0.4, h: 'No alarm from the monitor', b: 'The gyro and encoders do not disagree enough over the 2 s window, so SA keeps full speed.' });
  for (const arm of ORDER) {
    const tb = firstBreach(arm);
    if (tb !== null) ev.push({ t: tb, h: `${ARMS[arm].name} leaves the ±30 mm lane`, b: `${ARMS[arm].title}: the true robot is now outside the safe corridor (peak ${(st.sims[arm].sc.peak * 1e3).toFixed(1)} mm in this run).`, bad: true });
  }
  ev.push({ t: W.tRet, h: 'Camera back', b: 'The first camera fix arrives. The EKF pulls the estimate back to the truth, the alarm clears and the slowed robots return to full speed.' });
  ev.push({ t: W.t1, h: 'Result', b: verdictText(), result: true });
  ev.sort((p, q) => p.t - q.t);
  st.events = ev;
}

function verdictText() {
  const s = (a) => st.sims[a].sc, mm = (a) => (s(a).peak * 1e3).toFixed(1), lost = (a) => s(a).timeLost.toFixed(2);
  const out = ORDER.filter((a) => a !== 'dr' && s(a).breach).map((a) => ARMS[a].name);
  return `${out.length ? `${out.join(', ')} left the lane. ` : 'Every EKF policy stayed in the lane. '}` +
    `SA peak ${mm('SA')} mm for ${lost('SA')} s lost; SB peak ${mm('SB')} mm for ${lost('SB')} s lost; N peak ${mm('N')} mm.`;
}

function showCaption(e, until) {
  const c = $('caption');
  c.querySelector('.t').textContent = `t = ${e.t.toFixed(2)} s`;
  c.querySelector('.h').textContent = e.h; c.querySelector('.b').textContent = e.b;
  c.querySelector('.k').textContent = e.k || (st.presenter && st.playing === false ? 'Presenter mode: press Space to continue' : '');
  c.style.borderLeftColor = e.bad ? '#ef4444' : e.result ? '#22c55e' : '#3b82f6';
  c.classList.add('show'); st.capUntil = until;
}

// ---------------------------------------------------------------- canvas helpers
function prep(cv) {
  const dpr = window.devicePixelRatio || 1, w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
  const ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, w, h);
  return { ctx, w, h, dpr };
}
function normalLeft(s) {
  if (s <= route.L1) return [0, 1];
  if (s <= route.L1 + route.La) { const a = -Math.PI / 2 + (s - route.L1) / route.R; return [-Math.cos(a), -Math.sin(a)]; }
  return [-1, 0];
}
function bandPath(ctx, s0, s1, half) {
  s0 = Math.max(0, s0); s1 = Math.min(route.total, s1);
  ctx.beginPath();
  for (let s = s0; s <= s1 + 1e-9; s += 0.01) { const b = route.point(s), n = normalLeft(s); ctx.lineTo(b[0] + half * n[0], b[1] + half * n[1]); }
  for (let s = s1; s >= s0 - 1e-9; s -= 0.01) { const b = route.point(s), n = normalLeft(s); ctx.lineTo(b[0] - half * n[0], b[1] - half * n[1]); }
  ctx.closePath();
}
function offsetLine(ctx, s0, s1, d) {
  s0 = Math.max(0, s0); s1 = Math.min(route.total, s1); ctx.beginPath();
  for (let s = s0; s <= s1 + 1e-9; s += 0.01) { const b = route.point(s), n = normalLeft(s); ctx.lineTo(b[0] + d * n[0], b[1] + d * n[1]); }
}
function rr(ctx, x, y, w, h, r) { ctx.beginPath(); ctx.roundRect(x, y, w, h, r); }
function drawRacks(ctx, px, labels) {
  for (const [x0, y0, x1, y1, lab] of RACKS) {
    ctx.fillStyle = lab ? '#1b2440' : '#151d33'; ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
    ctx.strokeStyle = lab ? '#3b4b78' : '#263355'; ctx.lineWidth = 1.2 * px; ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
    ctx.fillStyle = 'rgba(245,158,11,0.16)';            // pallets
    const horiz = x1 - x0 > y1 - y0, L = horiz ? x1 - x0 : y1 - y0;
    for (let u = 0.06; u + 0.16 < L; u += 0.24) horiz ? ctx.fillRect(x0 + u, y0 + 0.05, 0.16, y1 - y0 - 0.1) : ctx.fillRect(x0 + 0.05, y0 + u, x1 - x0 - 0.1, 0.16);
    if (lab && labels) labels.push([(x0 + x1) / 2, (y0 + y1) / 2, lab]);
  }
}
function robotShape(ctx, x, y, th, color, ghost, px) {
  ctx.save(); ctx.translate(x, y); ctx.rotate(th);
  if (ghost) {
    ctx.setLineDash([6 * px, 4 * px]); ctx.strokeStyle = '#22d3ee'; ctx.lineWidth = 1.6 * px; rr(ctx, -0.13, -0.11, 0.26, 0.22, 0.045); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = 'rgba(34,211,238,0.10)'; ctx.fill();
    ctx.beginPath(); ctx.moveTo(0.16, 0); ctx.lineTo(0.1, 0.035); ctx.lineTo(0.1, -0.035); ctx.closePath(); ctx.fillStyle = '#22d3ee'; ctx.fill();
  } else {
    ctx.shadowColor = color; ctx.shadowBlur = 22;
    ctx.fillStyle = '#0a0f1c'; for (const sy of [-1, 1]) { rr(ctx, -0.04, sy * 0.1 - 0.016, 0.08, 0.032, 0.01); ctx.fill(); }
    const g = ctx.createLinearGradient(-0.13, -0.11, 0.13, 0.11); g.addColorStop(0, color); g.addColorStop(1, '#0f172a');
    ctx.fillStyle = g; rr(ctx, -0.13, -0.095, 0.26, 0.19, 0.045); ctx.fill();
    ctx.shadowBlur = 0; ctx.strokeStyle = 'rgba(255,255,255,0.55)'; ctx.lineWidth = 1.2 * px; ctx.stroke();
    ctx.fillStyle = '#e2e8f0'; ctx.beginPath(); ctx.arc(0.055, 0, 0.026, 0, 2 * Math.PI); ctx.fill();        // lidar / camera marker
    ctx.fillStyle = '#0f172a'; ctx.beginPath(); ctx.arc(0.055, 0, 0.012, 0, 2 * Math.PI); ctx.fill();
    ctx.fillStyle = '#fde68a'; ctx.fillRect(0.118, -0.06, 0.012, 0.03); ctx.fillRect(0.118, 0.03, 0.012, 0.03);   // headlights
  }
  ctx.restore();
}
function ellipse(ctx, cx, cy, P00, P01, P11, ns, px) {
  const m = (P00 + P11) / 2, d = Math.sqrt(((P00 - P11) / 2) ** 2 + P01 * P01), l1 = Math.max(m + d, 0), l2 = Math.max(m - d, 0), ang = 0.5 * Math.atan2(2 * P01, P00 - P11);
  ctx.beginPath(); ctx.ellipse(cx, cy, Math.max(ns * Math.sqrt(l1), 2 * px), Math.max(ns * Math.sqrt(l2), 2 * px), ang, 0, 2 * Math.PI);
  ctx.fillStyle = 'rgba(34,211,238,0.10)'; ctx.fill(); ctx.strokeStyle = 'rgba(34,211,238,0.85)'; ctx.lineWidth = 1.4 * px; ctx.stroke();
}
function pill(ctx, x, y, text, bg, fg = '#fff', align = 'left', size = 11.5) {
  ctx.font = `700 ${size}px Inter, system-ui, sans-serif`; const pad = size * 0.95, ph = size * 2.1, w = ctx.measureText(text).width + 2 * pad, X = align === 'right' ? x - w : align === 'center' ? x - w / 2 : x;
  ctx.fillStyle = bg; rr(ctx, X, y, w, ph, ph / 2); ctx.fill(); ctx.fillStyle = fg; ctx.fillText(text, X + pad, y + ph * 0.67); return w;
}
function gauge(ctx, x, y, w, label, valTxt, frac, color, marks = [], zone = null, centered = false) {
  const small = w < 150;
  ctx.font = `600 ${small ? 9 : 10.5}px Inter, system-ui, sans-serif`; ctx.fillStyle = '#8a9bb8'; ctx.fillText(label.toUpperCase(), x, y);
  ctx.font = `700 ${small ? 10.5 : 12}px "JetBrains Mono", monospace`; ctx.fillStyle = '#e2e8f0'; const tw = ctx.measureText(valTxt).width; ctx.fillText(valTxt, x + w - tw, y);
  const by = y + 6, bh = 8; ctx.fillStyle = '#0b1426'; rr(ctx, x, by, w, bh, 4); ctx.fill();
  if (zone) { ctx.fillStyle = 'rgba(239,68,68,0.25)'; for (const [a, b] of zone) ctx.fillRect(x + a * w, by, (b - a) * w, bh); }
  ctx.fillStyle = color;
  if (centered) { const c = x + w / 2, e = Math.max(-0.5, Math.min(0.5, frac)) * w; rr(ctx, Math.min(c, c + e), by, Math.abs(e), bh, 3); ctx.fill(); }
  else { rr(ctx, x, by, Math.max(0, Math.min(1, frac)) * w, bh, 4); ctx.fill(); }
  for (const [f, col] of marks) { ctx.fillStyle = col; ctx.fillRect(x + f * w - 1, by - 3, 2, bh + 6); }
}

// ---------------------------------------------------------------- follow-camera view
function drawView(cv, arm) {
  const { ctx, w, h, dpr } = prep(cv), S = st.sims[arm], s = sample(S.run, st.T), k = w / 1.45, px = 1 / k, T = st.T;
  const bg = ctx.createRadialGradient(w / 2, h / 2, 50, w / 2, h / 2, Math.max(w, h) * 0.75); bg.addColorStop(0, '#0e1730'); bg.addColorStop(1, '#070b14');
  ctx.fillStyle = bg; ctx.fillRect(0, 0, w, h);
  const cx = s.x, cy = s.y + 0.06;
  ctx.setTransform(dpr * k, 0, 0, -dpr * k, dpr * (w / 2 - cx * k), dpr * (h / 2 + cy * k));
  const x0 = cx - w / 2 / k, x1 = cx + w / 2 / k, y0 = cy - h / 2 / k, y1 = cy + h / 2 / k;
  ctx.lineWidth = px;                                                   // floor grid
  for (let g = Math.floor(x0 / 0.25) * 0.25; g <= x1; g += 0.25) { ctx.strokeStyle = Math.abs(g - Math.round(g)) < 1e-6 ? 'rgba(148,163,184,0.13)' : 'rgba(148,163,184,0.05)'; ctx.beginPath(); ctx.moveTo(g, y0); ctx.lineTo(g, y1); ctx.stroke(); }
  for (let g = Math.floor(y0 / 0.25) * 0.25; g <= y1; g += 0.25) { ctx.strokeStyle = Math.abs(g - Math.round(g)) < 1e-6 ? 'rgba(148,163,184,0.13)' : 'rgba(148,163,184,0.05)'; ctx.beginPath(); ctx.moveTo(x0, g); ctx.lineTo(x1, g); ctx.stroke(); }
  const labels = []; drawRacks(ctx, px, labels);
  const sa = s.s - 1.3, sb = s.s + 1.3;
  bandPath(ctx, sa, sb, 0.34); ctx.fillStyle = 'rgba(30,41,59,0.55)'; ctx.fill();                       // aisle
  const L = S.run.log, iOn = Math.round(st.W.tOn * 50), iRet = Math.min(L.s.length - 1, Math.round(st.W.tRet * 50));
  bandPath(ctx, L.s[iOn], L.s[iRet], 0.34);                                                             // where this robot is blind
  ctx.fillStyle = inBlackout(T) ? 'rgba(239,68,68,0.13)' : 'rgba(239,68,68,0.07)'; ctx.fill();
  bandPath(ctx, sa, sb, CORR); ctx.fillStyle = 'rgba(34,197,94,0.20)'; ctx.fill();                      // corridor
  ctx.strokeStyle = 'rgba(34,197,94,0.75)'; ctx.lineWidth = 1.3 * px; for (const d of [CORR, -CORR]) { offsetLine(ctx, sa, sb, d); ctx.stroke(); }
  ctx.strokeStyle = 'rgba(250,204,21,0.75)'; ctx.lineWidth = 2 * px; ctx.setLineDash([0.04, 0.035]); offsetLine(ctx, sa, sb, 0); ctx.stroke(); ctx.setLineDash([]);
  for (const d of [0.34, -0.34]) { ctx.strokeStyle = 'rgba(250,204,21,0.25)'; ctx.lineWidth = 3 * px; offsetLine(ctx, sa, sb, d); ctx.stroke(); }
  // trail coloured by |cross-track error|
  const i0 = Math.round(st.t0 * 50), trail = { g: new Path2D(), a: new Path2D(), r: new Path2D() };   // 3 batched paths (5 views per frame)
  for (let i = i0; i < s.i; i++) { const ae = Math.abs(L.e[i]), P = ae > CORR ? trail.r : ae > 0.015 ? trail.a : trail.g; P.moveTo(L.x[i], L.y[i]); P.lineTo(L.x[i + 1], L.y[i + 1]); }
  ctx.lineWidth = 3.2 * px; ctx.lineCap = 'round';
  for (const [key, col] of [['g', '#34d399'], ['a', '#f59e0b'], ['r', '#ef4444']]) { ctx.strokeStyle = col; ctx.stroke(trail[key]); }
  // estimate: link, covariance ellipse (2 sigma), ghost
  if (arm !== 'hold') {
    ctx.strokeStyle = 'rgba(34,211,238,0.55)'; ctx.lineWidth = 1.2 * px; ctx.setLineDash([3 * px, 3 * px]); ctx.beginPath(); ctx.moveTo(s.x, s.y); ctx.lineTo(s.xe, s.ye); ctx.stroke(); ctx.setLineDash([]);
    if (arm !== 'dr') ellipse(ctx, s.xe, s.ye, s.P00, s.P01, s.P11, 2, px);
    robotShape(ctx, s.xe, s.ye, s.the, '#22d3ee', true, px);
  }
  if (s.alarm) { const r = 0.2 + 0.025 * Math.sin(performance.now() / 120); ctx.strokeStyle = arm === 'SA' ? 'rgba(59,130,246,0.9)' : 'rgba(239,68,68,0.55)'; ctx.lineWidth = 2.5 * px; ctx.beginPath(); ctx.arc(s.x, s.y, r, 0, 2 * Math.PI); ctx.stroke(); }
  robotShape(ctx, s.x, s.y, s.th, ARMS[arm].color, false, px);
  // ---- screen-space overlays
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const C = w < 560;                                   // compact overlay when five views share the screen
  ctx.font = `700 ${C ? 9 : 10}px Inter, system-ui, sans-serif`; ctx.fillStyle = 'rgba(148,163,184,0.55)';
  for (const [lx, ly, lab] of labels) { const X = w / 2 + (lx - cx) * k, Y = h / 2 - (ly - cy) * k; if (X > -50 && X < w + 50 && Y > -20 && Y < h + 20) ctx.fillText(lab + (C ? '' : ' · blocks the camera'), X - (C ? 18 : 52), Y); }
  const out = Math.abs(s.e) > CORR;
  if (out) { ctx.strokeStyle = `rgba(239,68,68,${0.55 + 0.35 * Math.sin(performance.now() / 140)})`; ctx.lineWidth = C ? 4 : 6; ctx.strokeRect(2, 2, w - 4, h - 4); }
  // header
  const ttl = C ? ARMS[arm].short : ARMS[arm].title;
  ctx.font = `500 ${C ? 10.5 : 11.5}px Inter, system-ui, sans-serif`; const hw = ctx.measureText(ttl).width + (C ? 44 : 52);
  ctx.fillStyle = 'rgba(7,11,20,0.75)'; rr(ctx, 10, 10, Math.max(hw, 70), C ? 38 : 44, 11); ctx.fill();
  ctx.fillStyle = ARMS[arm].color; ctx.beginPath(); ctx.arc(25, C ? 29 : 32, C ? 6 : 7, 0, 2 * Math.PI); ctx.fill();
  ctx.fillStyle = '#fff'; ctx.font = `800 ${C ? 14 : 16}px Inter, system-ui, sans-serif`; ctx.fillText(ARMS[arm].name, 37, C ? 26 : 29);
  ctx.fillStyle = '#9fb0cc'; ctx.font = `500 ${C ? 10.5 : 11.5}px Inter, system-ui, sans-serif`; ctx.fillText(ttl, 37, C ? 41 : 45);
  const bo = inBlackout(T), fs = C ? 10 : 11.5, ph = fs * 2.1 + 5;
  let yy = 12; pill(ctx, w - 10, yy, bo ? (C ? `BLACKOUT ${(T - st.W.tOn).toFixed(1)} s` : `CAMERA BLACKOUT · ${(T - st.W.tOn).toFixed(1)} s`) : (C ? 'CAMERA OK' : 'CAMERA OK · 10 Hz'), bo ? '#b91c1c' : '#166534', '#fff', 'right', fs); yy += ph;
  const pre = PRESETS.find((p) => p.id === st.preset);
  if (pre.chip && (faultActive(T) || pre.fault.kind === 'scale' || pre.fault.kind === 'diff')) { pill(ctx, w - 10, yy, pre.chip, '#92400e', '#fff', 'right', fs); yy += ph; }
  if (out) pill(ctx, w - 10, yy, C ? `OUT · ${(Math.abs(s.e) * 1e3).toFixed(0)} mm` : `OUT OF LANE · ${(Math.abs(s.e) * 1e3).toFixed(0)} mm`, '#ef4444', '#fff', 'right', fs);
  const slowed = s.vc > 0 && s.vc < 0.9 * P_B.v, by = h - (C ? 98 : 132);
  if (s.alarm && arm === 'SA') pill(ctx, w / 2, by, C ? 'DISAGREE → SLOW TO 0.15 m/s' : 'SENSORS DISAGREE → SLOWING TO 0.15 m/s', '#1d4ed8', '#fff', 'center', fs);
  else if (slowed) pill(ctx, w / 2, by, arm === 'SB' ? (C ? 'BLANKET RULE → SLOW' : 'BLANKET RULE: BLACKOUT → SLOW TO 0.15 m/s') : arm === 'SO' ? (C ? 'ORACLE → SLOW' : 'ORACLE: FAULT KNOWN → SLOW') : 'SLOWED', '#334155', '#fff', 'center', fs);
  else if (s.alarm) pill(ctx, w / 2, by, C ? `DISAGREE · ${ARMS[arm].name} IGNORES IT` : `SENSORS DISAGREE · ${ARMS[arm].name} IGNORES IT, FULL SPEED`, '#7f1d1d', '#fff', 'center', fs);
  // gauges (+ legend line when there is room)
  const gh = C ? 58 : 86, pad = C ? 10 : 16;
  ctx.fillStyle = 'rgba(7,11,20,0.78)'; rr(ctx, 10, h - gh - 10, w - 20, gh, 10); ctx.fill();
  const gw = (w - 20 - 4 * pad) / 3, gy = h - gh + 12, gx = (j) => 10 + pad + j * (gw + pad);
  gauge(ctx, gx(0), gy, gw, 'speed', `${s.u.toFixed(2)} m/s`, s.u / 0.6, ARMS[arm].color, [[s.vc / 0.6, '#e2e8f0']]);
  gauge(ctx, gx(1), gy, gw, C ? 'lane' : 'lane error', `${(s.e * 1e3).toFixed(1)} mm`, s.e / 0.12, out ? '#ef4444' : '#34d399', [[0.25, '#94a3b8'], [0.75, '#94a3b8']], [[0, 0.25], [0.75, 1]], true);
  gauge(ctx, gx(2), gy, gw, C ? 'gyro ≠ enc' : 'gyro vs encoder', `${(Math.abs(s.D) / D2R).toFixed(2)}°`, Math.abs(s.D) / D2R / 2.4, s.alarm ? '#3b82f6' : '#a78bfa', [[H_DEG / 2.4, '#ef4444']]);
  if (!C) { ctx.font = '500 11px Inter, system-ui, sans-serif'; ctx.fillStyle = '#7d8eab'; ctx.fillText('trail = true path (green · amber > 15 mm · red out)   ghost = EKF estimate + 2σ   band = ±30 mm lane', 28, h - 26); }
}

// ---------------------------------------------------------------- overview map
function drawMini() {
  const cv = $('mini'), { ctx, w, h, dpr } = prep(cv);
  const xmin = -0.3, xmax = 5.0, ymin = -1.0, ymax = 6.45, k = Math.min(w / (xmax - xmin), h / (ymax - ymin)), ox = (w - k * (xmax - xmin)) / 2;
  ctx.setTransform(dpr * k, 0, 0, -dpr * k, dpr * (ox - xmin * k), dpr * (h + ymin * k)); const px = 1 / k;
  ctx.fillStyle = '#0a1122'; ctx.fillRect(xmin, ymin, xmax - xmin, ymax - ymin);
  drawRacks(ctx, px, null);
  bandPath(ctx, 0, route.total, 0.34); ctx.fillStyle = 'rgba(30,41,59,0.8)'; ctx.fill();
  const L = st.sims.SA.run.log; bandPath(ctx, L.s[Math.round(st.W.tOn * 50)], L.s[Math.min(L.s.length - 1, Math.round(st.W.tRet * 50))], 0.34); ctx.fillStyle = 'rgba(239,68,68,0.35)'; ctx.fill();
  ctx.strokeStyle = 'rgba(250,204,21,0.6)'; ctx.lineWidth = 1.5 * px; ctx.setLineDash([4 * px, 4 * px]); offsetLine(ctx, 0, route.total, 0); ctx.stroke(); ctx.setLineDash([]);
  // overhead camera and its line of sight to SA
  const sSA = sample(st.sims.SA.run, st.T), bo = inBlackout(st.T);
  ctx.strokeStyle = bo ? 'rgba(239,68,68,0.6)' : 'rgba(34,197,94,0.5)'; ctx.lineWidth = 1.2 * px; ctx.setLineDash(bo ? [3 * px, 3 * px] : []);
  ctx.beginPath(); ctx.moveTo(CAMERA_POS[0], CAMERA_POS[1]); ctx.lineTo(sSA.x, sSA.y); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = '#e2e8f0'; ctx.fillRect(CAMERA_POS[0] - 0.12, CAMERA_POS[1] - 0.08, 0.24, 0.16); ctx.fillStyle = bo ? '#ef4444' : '#22c55e'; ctx.beginPath(); ctx.arc(CAMERA_POS[0], CAMERA_POS[1], 0.05, 0, 2 * Math.PI); ctx.fill();
  const pos = ORDER.map((a) => [a, sample(st.sims[a].run, st.T)]);
  for (const [a, s] of pos) {
    ctx.fillStyle = ARMS[a].color; ctx.beginPath(); ctx.arc(s.x, s.y, (a === 'SA' ? 0.13 : 0.1), 0, 2 * Math.PI); ctx.fill();
    if (a === 'SA') { ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5 * px; ctx.stroke(); }
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.font = '700 10px Inter, system-ui, sans-serif';
  for (const [a, s] of pos) { const X = ox + (s.x - xmin) * k, Y = h - (s.y - ymin) * k; ctx.fillStyle = ARMS[a].color; ctx.fillText(ARMS[a].name, X + 9, Y - 6); }
  const CX = ox + (CAMERA_POS[0] - xmin) * k, CY = h - (CAMERA_POS[1] - ymin) * k; ctx.fillStyle = '#9fb0cc'; ctx.fillText('overhead camera', CX - 40, CY - 12);
}

function drawRace() {
  const rows = ORDER.map((a) => {
    const s = sample(st.sims[a].run, st.T), L = st.sims[a].run.log, out = L.e.slice(Math.round(st.t0 * 50), s.i + 1).some((e) => Math.abs(e) > CORR);
    return `<div class="row"><span style="color:${ARMS[a].color};font-weight:800">${ARMS[a].name}</span><div class="bar"><i style="width:${(100 * s.s / route.total).toFixed(1)}%;background:${ARMS[a].color}"></i></div>` +
      `<span class="val ${out ? 'out' : ''}">${out ? 'OUT' : s.s.toFixed(2) + ' m'}</span></div>`;
  });
  $('race').innerHTML = rows.join('');
}

function renderScores() {
  const f = (v, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : '—');
  $('verdict').innerHTML = verdictText().replace(/(SA peak [^;]+)/, '<b>$1</b>');
  $('scores').innerHTML = '<tr><th>policy</th><th>peak mm</th><th>lane</th><th>time lost</th><th>alarm</th></tr>' + ORDER.map((a) => {
    const sc = st.sims[a].sc;
    return `<tr class="${a === 'SA' ? 'hl' : ''}"><td><span class="dot" style="background:${ARMS[a].color}"></span>${ARMS[a].name}</td><td>${f(sc.peak * 1e3)}</td>` +
      `<td class="${sc.breach ? 'bad' : 'good'}">${sc.breach ? 'OUT' : 'in'}</td><td>${f(sc.timeLost, 2)} s</td><td>${a === 'SA' && Number.isFinite(sc.alarmDelay) ? f(sc.alarmDelay, 2) + ' s' : '—'}</td></tr>`;
  }).join('');
}
function renderEvents() {
  $('events').innerHTML = st.events.map((e, i) => `<li data-i="${i}"><span class="tt">${e.t.toFixed(2)}s</span><span><span class="hh">${e.h}</span></span></li>`).join('');
}

// ---------------------------------------------------------------- timeline (static layer cached)
const TL = { L: 58, R: 14 };
function tlX(t, w) { return TL.L + (t - st.t0) / (st.t1 - st.t0) * (w - TL.L - TL.R); }
function drawTimeline() {
  const cv = $('timeline'), { ctx, w, h, dpr } = prep(cv);
  if (!st.tlCache || st.tlCache.w !== cv.width || st.tlCache.h !== cv.height) {
    const off = document.createElement('canvas'); off.width = cv.width; off.height = cv.height;
    const c = off.getContext('2d'); c.setTransform(dpr, 0, 0, dpr, 0, 0); paintTimeline(c, w, h); st.tlCache = { canvas: off, w: cv.width, h: cv.height };
  }
  ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.drawImage(st.tlCache.canvas, 0, 0); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const X = tlX(st.T, w);
  ctx.fillStyle = 'rgba(7,11,20,0.45)'; ctx.fillRect(X, 22, w - TL.R - X, h - 40);
  ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(X, 16); ctx.lineTo(X, h - 18); ctx.stroke();
  ctx.fillStyle = '#fff'; ctx.beginPath(); ctx.moveTo(X - 6, 10); ctx.lineTo(X + 6, 10); ctx.lineTo(X, 18); ctx.fill();
}
function paintTimeline(c, w, h) {
  const X = (t) => tlX(t, w), W = st.W, SA = st.sims.SA.run;
  c.font = '600 10.5px Inter, system-ui, sans-serif';
  // lanes: events | camera | fault + alarm | error chart | residual chart
  const lane = (y, label) => { c.fillStyle = '#7d8eab'; c.fillText(label, 6, y + 9); };
  lane(28, 'camera'); lane(46, 'alarm');
  for (let k = 0; k < SA.cam.j.length; k++) { const t = SA.cam.j[k] / 10; if (t < st.t0 || t > st.t1) continue; c.fillStyle = SA.cam.ok[k] ? '#253455' : '#ef4444'; c.fillRect(X(t), 28, Math.max(1, X(t + 0.1) - X(t) - 1), 11); }
  let a0 = null; SA.log.t.forEach((t, i) => { const on = SA.log.alarm[i]; if (on && a0 === null) a0 = t; if ((!on || i === SA.log.t.length - 1) && a0 !== null) { c.fillStyle = '#3b82f6'; c.fillRect(X(Math.max(a0, st.t0)), 46, X(Math.min(t, st.t1)) - X(Math.max(a0, st.t0)), 10); a0 = null; } });
  if (isFinite(st.tF) && st.tF > 0) { c.fillStyle = '#f59e0b'; c.fillRect(X(st.tF), 44, Math.max(2, X(st.t1) - X(st.tF)), 2); }
  const box = (y0, hh, lo, hi, label, ticks, fmt) => {
    c.fillStyle = '#0a1122'; c.fillRect(TL.L, y0, w - TL.L - TL.R, hh); c.strokeStyle = '#1e2a44'; c.strokeRect(TL.L + 0.5, y0 + 0.5, w - TL.L - TL.R - 1, hh - 1);
    c.fillStyle = '#7d8eab'; c.fillText(label, 6, y0 + 10);
    for (const v of ticks) { const Y = y0 + (1 - (v - lo) / (hi - lo)) * hh; c.strokeStyle = 'rgba(148,163,184,0.10)'; c.beginPath(); c.moveTo(TL.L, Y); c.lineTo(w - TL.R, Y); c.stroke(); c.fillStyle = '#5b6b88'; c.fillText(fmt(v), 18, Y + 3); }
    return (v) => y0 + (1 - (Math.max(lo, Math.min(hi, v)) - lo) / (hi - lo)) * hh;
  };
  let m = 45; for (const a of ORDER) st.sims[a].run.log.t.forEach((t, i) => { if (t >= st.t0 && t <= st.t1) m = Math.max(m, Math.abs(st.sims[a].run.log.e[i]) * 1e3 * 1.08); });
  m = Math.ceil(m / 15) * 15;
  const Ye = box(66, 132, -m, m, 'lane err', [-m, -30, 0, 30, m], (v) => `${v}`);
  c.fillStyle = 'rgba(239,68,68,0.08)'; c.fillRect(TL.L, Ye(m), w - TL.L - TL.R, Ye(30) - Ye(m)); c.fillRect(TL.L, Ye(-30), w - TL.L - TL.R, Ye(-m) - Ye(-30));
  for (const a of [...ORDER.filter((q) => q !== 'SA'), 'SA']) {
    const L = st.sims[a].run.log, bold = a === 'SA' || a === 'N'; c.strokeStyle = ARMS[a].color; c.globalAlpha = bold ? 1 : 0.7; c.lineWidth = bold ? 2.2 : 1.4;
    c.beginPath(); let go = false; for (let i = 0; i < L.t.length; i++) { const t = L.t[i]; if (t < st.t0 || t > st.t1) continue; const p = [X(t), Ye(L.e[i] * 1e3)]; go ? c.lineTo(...p) : c.moveTo(...p); go = true; } c.stroke();
  }
  c.globalAlpha = 1;
  const rArm = 'SA', LR = st.sims[rArm].run.log;
  let md = 1.2; LR.t.forEach((t, i) => { if (t >= st.t0 && t <= st.t1) md = Math.max(md, Math.abs(LR.D[i]) / D2R * 1.1); });
  const Yd = box(210, 70, -md, md, `${ARMS[rArm].name} gyro−enc`, [-H_DEG, 0, H_DEG], (v) => `${v.toFixed(1)}°`);
  c.strokeStyle = 'rgba(239,68,68,0.8)'; c.setLineDash([5, 4]); for (const v of [H_DEG, -H_DEG]) { c.beginPath(); c.moveTo(TL.L, Yd(v)); c.lineTo(w - TL.R, Yd(v)); c.stroke(); } c.setLineDash([]);
  c.strokeStyle = '#a78bfa'; c.lineWidth = 1.4; c.beginPath(); let go = false;
  for (let i = 0; i < LR.t.length; i++) { const t = LR.t[i]; if (t < st.t0 || t > st.t1) continue; const p = [X(t), Yd(LR.D[i] / D2R)]; go ? c.lineTo(...p) : c.moveTo(...p); go = true; } c.stroke();
  // blackout shading, events, time axis
  c.fillStyle = 'rgba(239,68,68,0.07)'; c.fillRect(X(W.tOn), 24, X(W.tRet) - X(W.tOn), h - 42);
  for (const e of st.events) { const x = X(e.t); c.strokeStyle = e.bad ? 'rgba(239,68,68,0.6)' : 'rgba(148,163,184,0.35)'; c.setLineDash([2, 3]); c.beginPath(); c.moveTo(x, 20); c.lineTo(x, h - 18); c.stroke(); c.setLineDash([]);
    c.fillStyle = e.bad ? '#ef4444' : e.result ? '#22c55e' : '#93c5fd'; c.beginPath(); c.moveTo(x, 13); c.lineTo(x + 5, 18); c.lineTo(x, 23); c.lineTo(x - 5, 18); c.fill(); }
  c.fillStyle = '#5b6b88'; for (let t = Math.ceil(st.t0); t <= st.t1; t++) c.fillText(`${t}s`, X(t) - 7, h - 4);
}

// ---------------------------------------------------------------- main loop
function draw() {
  for (const a of ORDER) drawView($(`view_${a}`), a);
  drawMini(); drawRace(); drawTimeline();
  const bo = inBlackout(st.T);
  $('clock').innerHTML = `${st.T.toFixed(2)} s<small>${bo ? `blackout ${(st.T - st.W.tOn).toFixed(1)} / ${st.Tb} s` : st.T < st.W.tOn ? `blackout in ${(st.W.tOn - st.T).toFixed(1)} s` : 'camera back'}</small>`;
  const act = st.events.reduce((k, e, i) => (e.t <= st.T + 1e-6 ? i : k), -1);
  document.querySelectorAll('#events li').forEach((li, i) => li.classList.toggle('on', i === act));
  if (performance.now() > st.capUntil) $('caption').classList.remove('show');
}
function setT(T) { st.T = Math.max(st.t0, Math.min(st.t1, T)); }
function frame(ts) {
  const dt = Math.min(0.05, (ts - st.last) / 1000); st.last = ts;
  advance(dt);
  draw(); requestAnimationFrame(frame);
}
// one playback step of dt wall-clock seconds (exposed as window.__advance for testing in hidden tabs)
function advance(dt) {
  if (st.playing) {
    const T0 = st.T, T1 = Math.min(st.t1, st.T + dt * st.speed);
    const hit = st.events.find((e) => e.t > T0 + 1e-9 && e.t <= T1 + 1e-9);
    if (hit) {
      setT(hit.t);
      if (st.presenter) { st.playing = false; setPlayIcon(); showCaption({ ...hit, k: 'Presenter mode: press Space to continue' }, 1e12); }
      else showCaption(hit, performance.now() + (hit.result ? 1e9 : 5500));
    } else setT(T1);
    if (st.T >= st.t1) { st.playing = false; setPlayIcon(); }
  }
}
window.__advance = (sec, step = 0.02) => { for (let t = 0; t < sec && st.playing; t += step) advance(step); draw(); return { T: st.T, playing: st.playing }; };
function setPlayIcon() { $('playIcon').innerHTML = st.playing ? '<path d="M6 4h4v16H6zm8 0h4v16h-4z"/>' : '<path d="M7 4v16l13-8z"/>'; }
function togglePlay() {
  if (st.T >= st.t1 - 1e-6) setT(st.t0);
  st.playing = !st.playing; setPlayIcon();
  if (st.playing && st.capUntil > 1e11) st.capUntil = performance.now() + 600;
}

// ---------------------------------------------------------------- controls
function seg(id, key, cb) {
  const el = $(id), sync = () => el.querySelectorAll('button').forEach((b) => b.classList.toggle('on', String(st[key]) === b.dataset.v));
  el.addEventListener('click', (e) => { const b = e.target.closest('button'); if (!b) return; st[key] = isNaN(+b.dataset.v) ? b.dataset.v : +b.dataset.v; sync(); cb && cb(); });
  sync();
}
$('presets').innerHTML = PRESETS.map((p, i) => `<button class="chip" data-id="${p.id}" title="key ${i + 1}">${p.label}</button>`).join('');
const syncPresets = () => document.querySelectorAll('#presets .chip').forEach((b) => b.classList.toggle('on', b.dataset.id === st.preset));
$('presets').addEventListener('click', (e) => { const b = e.target.closest('.chip'); if (!b) return; st.preset = b.dataset.id; syncPresets(); runAll(); });
syncPresets();
seg('tb', 'Tb', runAll); seg('onset', 'onset', runAll); seg('speed', 'speed');
$('seed').addEventListener('change', (e) => { st.seed = Math.max(1, +e.target.value || 1); runAll(); });
$('dice').onclick = () => { st.seed = 1 + Math.floor(Math.random() * 999); $('seed').value = st.seed; runAll(); };
$('presenter').addEventListener('change', (e) => { st.presenter = e.target.checked; });
$('play').onclick = togglePlay;
$('restart').onclick = () => { setT(st.t0); st.playing = false; setPlayIcon(); };
$('back').onclick = () => setT(st.T - 1); $('fwd').onclick = () => setT(st.T + 1);
$('events').addEventListener('click', (e) => { const li = e.target.closest('li'); if (!li) return; const ev = st.events[+li.dataset.i]; setT(ev.t); showCaption(ev, performance.now() + 7000); });
const seekTL = (e) => { const r = $('timeline').getBoundingClientRect(); setT(st.t0 + (e.clientX - r.left - TL.L) / (r.width - TL.L - TL.R) * (st.t1 - st.t0)); };
$('timeline').addEventListener('mousedown', (e) => { st.dragging = true; seekTL(e); });
window.addEventListener('mousemove', (e) => { if (st.dragging) seekTL(e); });
window.addEventListener('mouseup', () => { st.dragging = false; });
window.addEventListener('resize', () => { st.tlCache = null; });
window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  if (e.code === 'Space') { e.preventDefault(); togglePlay(); }
  else if (e.key === 'ArrowRight') setT(st.T + (e.shiftKey ? 1 : 0.1));
  else if (e.key === 'ArrowLeft') setT(st.T - (e.shiftKey ? 1 : 0.1));
  else if (e.key === 'r' || e.key === 'R') { setT(st.t0); st.playing = false; setPlayIcon(); }
  else if (e.key === 'p' || e.key === 'P') { st.presenter = !st.presenter; $('presenter').checked = st.presenter; }
  else if (/^[1-6]$/.test(e.key)) { st.preset = PRESETS[+e.key - 1].id; syncPresets(); runAll(); }
});

// ---------------------------------------------------------------- tools (unchanged behaviour)
const say = (s) => { $('toolLog').textContent = s; };
async function save(path, text) {
  try { const r = await fetch(`/save?path=${path}`, { method: 'POST', body: text }); if (!r.ok) throw new Error(r.status); return `saved ${path}`; }
  catch { const a = Object.assign(document.createElement('a'), { href: URL.createObjectURL(new Blob([text])), download: path.split('/').pop() }); a.click(); return `downloaded ${path.split('/').pop()}`; }
}
window.exportTraces = async () => {
  const { files, checks } = exportAll(), out = [];
  for (const [name, text] of Object.entries(files)) out.push(await save(`firmware/esp32_wokwi/${name}`, text));
  const c = Object.values(checks);
  say(`replay check: max pose diff ${Math.max(...c.map((x) => x.maxPos_m)).toExponential(1)} m, alarm diffs ${c.reduce((a, x) => a + x.alarmRowsDiffering, 0)} · ${out.length} files`);
  return { checks, out };
};
window.saveVivaExample = async () => {
  const r5 = (x) => +(+x).toPrecision(5), runs = {}, F = { none: { kind: 'none' }, gyro1: { kind: 'gyro', b: 1.0 } };
  let W = null, tF = null;
  for (const fault of ['none', 'gyro1']) for (const arm of ['N', 'SB', 'SA']) {
    const r = resolve({ params: P_B, arm, seed: 1, loss: { kind: 'blackout', Tb: 4, onset: 'entry' }, fault: F[fault] }), run = simulate(r.cfg), L = run.log, t0 = r.W.tOn - 2, t1 = r.W.t1;
    const idx = L.t.map((t, i) => i).filter((i) => L.t[i] >= t0 && L.t[i] <= t1), pick = (k) => idx.map((i) => r5(L[k][i]));
    runs[`${fault}_${arm}`] = { t: pick('t'), e_mm: idx.map((i) => r5(L.e[i] * 1e3)), D_deg: idx.map((i) => r5(L.D[i] / D2R)), alarm: pick('alarm'),
      uL: pick('uL'), uR: pick('uR'), uLc: pick('uLc'), uRc: pick('uRc'), vc: pick('vc'), cam: run.cam.j.map((j, k) => [j / run.p.camHz, run.cam.ok[k]]).filter(([t]) => t >= t0 && t <= t1) };
    W = r.W; if (fault === 'gyro1') tF = r.tF;
  }
  const m = await save('results/examples/viva_example.json', JSON.stringify({ note: 'Synthetic. B0 primary cell (R = 1 m, v = 0.5 m/s, 4 s blackout from arc entry), seed 1. Written by web/demo.js.', W, tF, runs }));
  say(m); return m;
};
$('export').onclick = () => window.exportTraces().catch((err) => say('export failed: ' + err));
$('example').onclick = () => window.saveVivaExample().catch((err) => say('save failed: ' + err));

runAll();
requestAnimationFrame((ts) => { st.last = ts; requestAnimationFrame(frame); });
