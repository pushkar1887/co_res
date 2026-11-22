// route.js: straight L1 -> 90 deg left arc of radius R -> straight L3 (PLAN §4.1).
import { clamp } from './util.js';

export function makePath(R, L1, L3) {
  const La = Math.PI / 2 * R, cx = L1, cy = R, total = L1 + La + L3;
  function point(s) {
    if (s <= L1) return [s, 0];
    if (s <= L1 + La) { const a = -Math.PI / 2 + (s - L1) / R; return [cx + R * Math.cos(a), cy + R * Math.sin(a)]; }
    return [L1 + R, R + (s - L1 - La)];
  }
  // Closest point on the continuous route. e = signed cross-track error, left of travel = +.
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
  // 0 = first straight, 1 = arc, 2 = exit straight (for blackout-overlap metrics)
  const segment = (s) => (s < L1 ? 0 : s < L1 + La ? 1 : 2);
  return { R, L1, L3, La, total, point, project, segment };
}
