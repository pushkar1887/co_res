// control.js: Pure Pursuit and the speed policies N / SB / SA / SO (PLAN §5.2, §5.4).

export function purePursuit(path, X, Ld, vc) {
  const pr = path.project(X[0], X[1]), g = path.point(pr.s + Ld);
  const dx = g[0] - X[0], dy = g[1] - X[1], yg = -Math.sin(X[2]) * dx + Math.cos(X[2]) * dy;
  return 2 * vc * yg / (dx * dx + dy * dy);      // omega_c = 2 v_c y_g / d^2
}

// ctx: {missAge (s since the first missed report, or -1 if camera fresh), alarm, faultActive}
export function policySpeed(pol, p, ctx) {
  const blackout = ctx.missAge >= p.monGate - 1e-9;
  switch (pol) {
    case 'N': return p.v;
    case 'SB': return blackout ? p.alpha * p.v : p.v;
    case 'SA': return ctx.alarm ? p.alpha * p.v : p.v;
    case 'SO': return blackout && ctx.faultActive ? p.alpha * p.v : p.v;   // oracle reference, not deployable
    default: throw new Error('unknown policy ' + pol);
  }
}
