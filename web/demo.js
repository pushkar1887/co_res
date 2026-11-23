// demo.js: viva demo (PLAN §8, §10 F10, §13 M4). Plain <canvas>, no external libraries, so it works offline.
// Every policy is simulated with the SAME seed, blackout and fault by the same model the experiments use (sim/).
import { DEFAULTS } from '../sim/params.js';
import { resolve } from '../sim/scenarios.js';
import { simulate } from '../sim/simulate.js';
import { score } from '../sim/metrics.js';
import { makePath } from '../sim/route.js';
import { exportAll } from '../sim/trace_export.js';

const D2R = Math.PI / 180;
const P_B = { R: 1.0, v: 0.5, Ld: 0.30 };
const ARMS = [
  { arm: 'N', label: 'N never slow', color: '#c0392b', on: true },
  { arm: 'SB', label: 'SB slow every blackout', color: '#7f8c8d', on: true },
  { arm: 'SA', label: 'SA slow on alarm (ours)', color: '#2c6fb0', on: true },
  { arm: 'SO', label: 'SO oracle', color: '#27ae60', on: false },
  { arm: 'dr', label: 'DR dead reckoning', color: '#d08c2c', on: false },
];
const FAULTS = { none: { kind: 'none' }, gyro1: { kind: 'gyro', b: 1.0 }, gyro05: { kind: 'gyro', b: 0.5 }, slip: { kind: 'slip' },
  scale3: { kind: 'scale', eps: 0.03 }, diff2: { kind: 'diff', delta: 0.02 } };
const $ = (id) => document.getElementById(id);
const st = { T: 0, playing: false, last: 0, sims: [], W: null, tF: Infinity, t0: 0, t1: 1 };
const route = makePath(P_B.R, DEFAULTS.L1, DEFAULTS.L3);
const sa = () => st.sims.find((s) => s.arm === 'SA');

// ---------- simulation ----------
const spec = (arm, fault, seed, Tb, onset) => ({ params: P_B, arm, seed, loss: { kind: 'blackout', Tb, onset }, fault: FAULTS[fault] });
function runAll() {
  const fault = $('fault').value, seed = Math.max(1, +$('seed').value || 1), Tb = +$('Tb').value, onset = $('onset').value;
  st.sims = ARMS.map((a) => {
    const r = resolve(spec(a.arm, fault, seed, Tb, onset)), run = simulate(r.cfg), ref = simulate(r.refCfg);
    return { ...a, r, run, sc: score(run, ref, r.W, r.tF) };
  });
  st.W = st.sims[0].r.W; st.tF = st.sims[0].r.tF;
  st.t0 = Math.max(0, st.W.tOn - 2); st.t1 = st.W.t1 + 1; st.T = st.t0; st.playing = false; $('play').textContent = 'Play';
  renderStats(); draw();
}

// ---------- canvas helpers ----------
function prep(cv) {
  if (!cv.dataset.h) { cv.dataset.h = cv.getAttribute('height'); cv.style.height = cv.dataset.h + 'px'; }
  const dpr = window.devicePixelRatio || 1, w = cv.clientWidth, h = +cv.dataset.h;
  cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
  const ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, w, h);
  ctx.font = '11px system-ui, sans-serif';
  return { ctx, w, h };
}
const rowAt = (run, t) => Math.max(0, Math.min(run.log.t.length - 1, Math.round(t * run.p.sensHz)));

function plot(cv, { y0, y1, series = [], hlines = [], bands = [], yfmt = (v) => v.toFixed(0) }) {
  const { ctx, w, h } = prep(cv), L = 46, Rm = 8, Tm = 6, B = 18, pw = w - L - Rm, ph = h - Tm - B;
  const X = (t) => L + (t - st.t0) / (st.t1 - st.t0) * pw, Y = (v) => Tm + (1 - (v - y0) / (y1 - y0)) * ph;
  for (const b of bands) { ctx.fillStyle = b.color; ctx.fillRect(X(Math.max(b.t0, st.t0)), Tm, X(Math.min(b.t1, st.t1)) - X(Math.max(b.t0, st.t0)), ph); }
  ctx.strokeStyle = '#e3e7ee'; ctx.fillStyle = '#5d6675'; ctx.lineWidth = 1;
  for (let k = 0; k <= 4; k++) { const v = y0 + (y1 - y0) * k / 4, y = Y(v); ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(L + pw, y); ctx.stroke(); ctx.fillText(yfmt(v), 4, y + 4); }
  for (let t = Math.ceil(st.t0); t <= st.t1; t++) { const x = X(t); ctx.beginPath(); ctx.moveTo(x, Tm); ctx.lineTo(x, Tm + ph); ctx.stroke(); ctx.fillText(`${t}s`, x - 8, h - 4); }
  for (const l of hlines) { ctx.strokeStyle = l.color || '#333'; ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.moveTo(L, Y(l.v)); ctx.lineTo(L + pw, Y(l.v)); ctx.stroke(); ctx.setLineDash([]); }
  for (const s of series) {
    for (const [alpha, upTo] of [[0.22, st.t1], [1, st.T]]) {
      ctx.globalAlpha = alpha; ctx.strokeStyle = s.color; ctx.lineWidth = s.width || 1.6; ctx.setLineDash(s.dash || []);
      ctx.beginPath(); let started = false;
      for (let i = 0; i < s.t.length; i++) {
        const t = s.t[i]; if (t < st.t0 || t > upTo) continue;
        const v = Math.max(y0, Math.min(y1, s.v[i]));
        if (!started) { ctx.moveTo(X(t), Y(v)); started = true; } else ctx.lineTo(X(t), Y(v));
      }
      ctx.stroke(); ctx.setLineDash([]);
    }
  }
  ctx.globalAlpha = 1;
  if (isFinite(st.tF) && st.tF > 0) { ctx.strokeStyle = '#8e44ad'; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(X(st.tF), Tm); ctx.lineTo(X(st.tF), Tm + ph); ctx.stroke(); ctx.setLineDash([]); }
  ctx.strokeStyle = '#111'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(X(st.T), Tm); ctx.lineTo(X(st.T), Tm + ph); ctx.stroke();
}

const alarmBands = (run, color) => {
  const out = []; let start = null;
  run.log.t.forEach((t, i) => { const a = run.log.alarm[i]; if (a && start === null) start = t; if (!a && start !== null) { out.push({ t0: start, t1: t, color }); start = null; } });
  if (start !== null) out.push({ t0: start, t1: run.log.t[run.log.t.length - 1], color });
  return out;
};

// ---------- panels ----------
function drawCam() {
  const { ctx, w, h } = prep($('cam')), L = 46, pw = w - L - 8, X = (t) => L + (t - st.t0) / (st.t1 - st.t0) * pw, run = sa().run;
  ctx.fillStyle = '#5d6675'; ctx.fillText('camera', 2, 20); ctx.fillText('SA alarm', 2, 46);
  run.cam.j.forEach((j, k) => {
    const t = j / run.p.camHz; if (t < st.t0 || t > st.t1) return;
    ctx.fillStyle = run.cam.ok[k] ? '#cfd6e0' : '#e74c3c'; ctx.fillRect(X(t), 10, Math.max(1, X(t + 0.1) - X(t) - 1), 16);
  });
  for (const b of alarmBands(run, '#2c6fb0')) { ctx.fillStyle = b.color; ctx.fillRect(X(Math.max(b.t0, st.t0)), 36, X(Math.min(b.t1, st.t1)) - X(Math.max(b.t0, st.t0)), 14); }
  if (isFinite(st.tF) && st.tF > 0) { ctx.fillStyle = '#8e44ad'; ctx.fillText(`fault ${$('fault').selectedOptions[0].text}`, X(st.tF) + 3, 64); ctx.fillRect(X(st.tF) - 1, 6, 2, 50); }
  ctx.fillStyle = '#111'; ctx.fillRect(X(st.T), 4, 1.5, 56);
}

function drawErr() {
  const shown = st.sims.filter((s) => s.on);
  let m = 40;
  for (const s of shown) s.run.log.t.forEach((t, i) => { if (t >= st.t0 && t <= st.t1) m = Math.max(m, Math.abs(s.run.log.e[i]) * 1e3 * 1.1); });
  plot($('err'), { y0: -m, y1: m, hlines: [{ v: 30, color: '#333' }, { v: -30, color: '#333' }, { v: 0, color: '#bbb' }],
    series: shown.map((s) => ({ t: s.run.log.t, v: s.run.log.e.map((e) => e * 1e3), color: s.color })) });
}

function drawRes() {
  const run = sa().run, h = run.p.monThr / D2R;
  let m = 1.0; run.log.t.forEach((t, i) => { if (t >= st.t0 && t <= st.t1) m = Math.max(m, Math.abs(run.log.D[i]) / D2R * 1.1); });
  plot($('res'), { y0: -m, y1: m, yfmt: (v) => v.toFixed(1), bands: alarmBands(run, 'rgba(44,111,176,0.12)'),
    hlines: [{ v: h, color: '#c0392b' }, { v: -h, color: '#c0392b' }],
    series: [{ t: run.log.t, v: run.log.D.map((d) => d / D2R), color: '#2c6fb0' }] });
}

function drawMotor() {
  const L = sa().run.log;
  plot($('motor'), { y0: 0, y1: 0.75, yfmt: (v) => v.toFixed(2), series: [
    { t: L.t, v: L.uLc, color: '#16a085', dash: [5, 3], width: 1.2 }, { t: L.t, v: L.uL, color: '#16a085' },
    { t: L.t, v: L.uRc, color: '#d35400', dash: [5, 3], width: 1.2 }, { t: L.t, v: L.uR, color: '#d35400' }] });
}

function normalLeft(s) {
  if (s <= route.L1) return [0, 1];
  if (s <= route.L1 + route.La) { const a = -Math.PI / 2 + (s - route.L1) / route.R; return [-Math.cos(a), -Math.sin(a)]; }
  return [-1, 0];
}
function drawMap() {
  const { ctx, w, h } = prep($('map')), K = +$('exag').value;
  // frame the stretch of route the robot covers in the plotted time window (the blackout and its recovery)
  const Ls = sa().run.log, sLo = Ls.s[rowAt(sa().run, st.t0)], sHi = Ls.s[rowAt(sa().run, st.t1)], pad = 0.35 + 0.03 * K;
  let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
  for (let s = sLo; s <= sHi; s += 0.05) { const b = route.point(s); xmin = Math.min(xmin, b[0]); xmax = Math.max(xmax, b[0]); ymin = Math.min(ymin, b[1]); ymax = Math.max(ymax, b[1]); }
  xmin -= pad; xmax += pad; ymin -= pad; ymax += pad;
  const k = Math.min(w / (xmax - xmin), h / (ymax - ymin)), ox = (w - k * (xmax - xmin)) / 2, oy = (h - k * (ymax - ymin)) / 2;
  const P = (x, y) => [ox + (x - xmin) * k, h - oy - (y - ymin) * k];
  const off = (s, d) => { const b = route.point(s), n = normalLeft(s); return P(b[0] + d * n[0], b[1] + d * n[1]); };
  const line = (pts, color, width, dash = []) => { ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.beginPath(); pts.forEach((p, i) => (i ? ctx.lineTo(...p) : ctx.moveTo(...p))); ctx.stroke(); ctx.setLineDash([]); };
  const S = []; for (let s = 0; s <= route.total; s += 0.02) S.push(s);
  // corridor band and centre line
  ctx.fillStyle = 'rgba(39,174,96,0.10)'; ctx.beginPath();
  S.forEach((s, i) => { const p = off(s, 0.03 * K); i ? ctx.lineTo(...p) : ctx.moveTo(...p); });
  [...S].reverse().forEach((s) => ctx.lineTo(...off(s, -0.03 * K))); ctx.closePath(); ctx.fill();
  line(S.map((s) => off(s, 0)), '#9aa3b2', 1, [4, 4]);
  // where the camera is blind (SA run) and where the fault starts
  const run = sa().run, L = run.log, blind = [];
  L.t.forEach((t, i) => { if (t >= st.W.tOn && t < st.W.tRet) blind.push(off(L.s[i], 0)); });
  line(blind, 'rgba(231,76,60,0.35)', 10);
  if (isFinite(st.tF) && st.tF > 0) { const i = rowAt(run, st.tF), p = off(L.s[i], 0); ctx.fillStyle = '#8e44ad'; ctx.fillRect(p[0] - 4, p[1] - 4, 8, 8); }
  // trails and robots
  for (const s of st.sims.filter((x) => x.on)) {
    const Lg = s.run.log, pts = [];
    for (let i = 0; i < Lg.t.length && Lg.t[i] <= st.T; i++) if (Lg.t[i] >= st.p0) pts.push(off(Lg.s[i], Lg.e[i] * K));
    line(pts, s.color, 2);
    const i = rowAt(s.run, st.T), p = off(Lg.s[i], Lg.e[i] * K), th = Lg.th[i];
    ctx.fillStyle = s.color; ctx.beginPath(); ctx.arc(p[0], p[1], 5, 0, 2 * Math.PI); ctx.fill();
    ctx.strokeStyle = s.color; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(...p); ctx.lineTo(p[0] + 14 * Math.cos(th), p[1] - 14 * Math.sin(th)); ctx.stroke();
  }
  const j = Math.floor(st.T * run.p.camHz + 1e-9), kk = run.cam.j.indexOf(j), ok = kk < 0 || run.cam.ok[kk];
  ctx.font = 'bold 13px system-ui, sans-serif'; ctx.fillStyle = ok ? '#27ae60' : '#c0392b';
  ctx.fillText(ok ? 'camera OK' : 'CAMERA LOST', 10, 20);
  if (L.alarm[rowAt(run, st.T)]) { ctx.fillStyle = '#2c6fb0'; ctx.fillText('SA ALARM: slowing to 0.15 m/s', 10, 38); }
  $('mapNote').textContent = `Green band = ±30 mm corridor; red band = where the camera is blind; purple square = fault onset. Lateral error drawn ×${K}.`;
}

function renderStats() {
  const f = (v, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : '—');
  $('stats').innerHTML = '<tr><th>policy</th><th>peak |e| mm</th><th>excess mm</th><th>breach</th><th>time lost s</th><th>alarm delay s</th></tr>' +
    st.sims.map((s) => `<tr><td><span class="swatch" style="background:${s.color}"></span>${s.label}</td><td>${f(s.sc.peak * 1e3)}</td><td>${f(s.sc.excess * 1e3)}</td>` +
      `<td>${s.sc.breach ? 'yes' : 'no'}</td><td>${f(s.sc.timeLost, 2)}</td><td>${s.arm === 'SA' ? f(s.sc.alarmDelay, 2) : '—'}</td></tr>`).join('');
}

function draw() {
  st.p0 = st.t0;
  drawMap(); drawCam(); drawErr(); drawRes(); drawMotor();
  $('clock').textContent = `t = ${st.T.toFixed(2)} s   (blackout ${st.W.tOn.toFixed(1)}–${st.W.tRet.toFixed(1)} s${isFinite(st.tF) && st.tF > 0 ? `, fault at ${st.tF.toFixed(1)} s` : ''})`;
  $('scrub').value = Math.round((st.T - st.t0) / (st.t1 - st.t0) * 1000);
}

// ---------- export ----------
const say = (s) => { $('log').textContent += s + '\n'; };
async function save(path, text) {
  try { const r = await fetch(`/save?path=${path}`, { method: 'POST', body: text }); if (!r.ok) throw new Error(r.status); return `saved ${path}`; }
  catch { const a = Object.assign(document.createElement('a'), { href: URL.createObjectURL(new Blob([text])), download: path.split('/').pop() }); a.click(); return `downloaded ${path.split('/').pop()}`; }
}
window.exportTraces = async () => {
  const { files, checks } = exportAll(), out = [];
  for (const [name, c] of Object.entries(checks))
    say(`replay check ${name}: ${c.rows} rows, max pose diff ${(c.maxPos_m * 1e3).toExponential(2)} mm, alarm rows differing ${c.alarmRowsDiffering}, max wheel-cmd diff ${c.maxCmd_mps.toExponential(2)} m/s`);
  for (const [name, text] of Object.entries(files)) { const m = await save(`firmware/esp32_wokwi/${name}`, text); out.push(m); say(`${m} (${(text.length / 1024).toFixed(0)} KB)`); }
  return { checks, out };
};
window.saveVivaExample = async () => {
  const r5 = (x) => +(+x).toPrecision(5), runs = {};
  let W = null, tF = null;
  for (const fault of ['none', 'gyro1']) for (const arm of ['N', 'SB', 'SA']) {
    const r = resolve(spec(arm, fault, 1, 4, 'entry')), run = simulate(r.cfg), L = run.log, t0 = r.W.tOn - 2, t1 = r.W.t1;
    const idx = L.t.map((t, i) => i).filter((i) => L.t[i] >= t0 && L.t[i] <= t1), pick = (k) => idx.map((i) => r5(L[k][i]));
    runs[`${fault}_${arm}`] = { t: pick('t'), e_mm: idx.map((i) => r5(L.e[i] * 1e3)), D_deg: idx.map((i) => r5(L.D[i] / D2R)), alarm: pick('alarm'),
      uL: pick('uL'), uR: pick('uR'), uLc: pick('uLc'), uRc: pick('uRc'), vc: pick('vc'),
      cam: run.cam.j.map((j, k) => [j / run.p.camHz, run.cam.ok[k]]).filter(([t]) => t >= t0 && t <= t1) };
    W = r.W; if (fault === 'gyro1') tF = r.tF;
  }
  const json = JSON.stringify({ note: 'Synthetic. B0 primary cell (R = 1 m, v = 0.5 m/s, 4 s blackout from arc entry), seed 1. Written by web/demo.js.', W, tF, runs });
  const m = await save('results/examples/viva_example.json', json); say(m); return m;
};

// ---------- wiring ----------
$('armBoxes').innerHTML = ARMS.map((a, i) => `<label><input type="checkbox" data-i="${i}" ${a.on ? 'checked' : ''}><span class="swatch" style="background:${a.color}"></span>${a.arm}</label>`).join(' ');
$('armBoxes').addEventListener('change', (e) => { const i = +e.target.dataset.i; ARMS[i].on = e.target.checked; st.sims[i].on = e.target.checked; draw(); });
for (const id of ['fault', 'Tb', 'onset', 'seed']) $(id).addEventListener('change', runAll);
$('exag').addEventListener('change', draw);
$('play').onclick = () => { if (st.T >= st.t1) st.T = st.t0; st.playing = !st.playing; $('play').textContent = st.playing ? 'Pause' : 'Play'; };
$('restart').onclick = () => { st.T = st.t0; draw(); };
$('scrub').oninput = (e) => { st.T = st.t0 + (+e.target.value / 1000) * (st.t1 - st.t0); draw(); };
$('export').onclick = () => window.exportTraces().catch((err) => say('export failed: ' + err));
$('example').onclick = () => window.saveVivaExample().catch((err) => say('save failed: ' + err));
window.addEventListener('resize', draw);
function frame(ts) {
  if (st.playing) { st.T = Math.min(st.t1, st.T + (ts - st.last) / 1000 * +$('speed').value); if (st.T >= st.t1) { st.playing = false; $('play').textContent = 'Play'; } draw(); }
  st.last = ts; requestAnimationFrame(frame);
}
runAll();
requestAnimationFrame(frame);
