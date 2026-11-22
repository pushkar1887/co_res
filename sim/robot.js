// robot.js: differential-drive plant (truth), two first-order motors, wheel-radius and slip faults.
// Motor speed AND wheel travel are integrated exactly for a command held over dt (PLAN §4.2, V4).
import { clamp, wrap } from './util.js';

export function makeRobot(p, faults) {
  const epsL = faults.scaleL || 0, epsR = faults.scaleR || 0, slip = faults.slip || null;
  const kExp = Math.exp(-p.dt / p.motorTau), kInt = p.motorTau * (1 - kExp);
  const st = { x: 0, y: 0, th: 0, uL: 0, uR: 0, phiL: 0, phiR: 0, thCum: 0 };   // thCum: unwrapped true heading

  // ground-speed factor of each wheel at time t: true radius and slip patch
  const slipOn = (t) => slip && t >= slip.start - 1e-9 && t < slip.start + slip.dur - 1e-9;
  const factor = (side, t) => (side === 'L' ? 1 + epsL : 1 + epsR) * (slipOn(t) && slip.wheel === side ? slip.factor : 1);

  // commanded wheel speeds for (v_c, w_c), after saturation
  const wheelCmd = (vc, wc) => [clamp(vc - wc * p.track / 2, -p.wheelVmax, p.wheelVmax), clamp(vc + wc * p.track / 2, -p.wheelVmax, p.wheelVmax)];

  function step(t, vc, wc) {
    const [uLc, uRc] = wheelCmd(vc, wc);
    // exact displacement of a first-order response with held command: d = u* dt + (u0 - u*) tau (1 - e^{-dt/tau})
    const dL = uLc * p.dt + (st.uL - uLc) * kInt, dR = uRc * p.dt + (st.uR - uRc) * kInt;
    st.uL = uLc + (st.uL - uLc) * kExp; st.uR = uRc + (st.uR - uRc) * kExp;
    st.phiL += dL / p.wheelR; st.phiR += dR / p.wheelR;            // encoders see wheel ROTATION
    const gL = dL * factor('L', t), gR = dR * factor('R', t);      // ground travel uses true radius and slip
    const ds = 0.5 * (gL + gR), dth = (gR - gL) / p.track, thm = st.th + 0.5 * dth;
    st.x += ds * Math.cos(thm); st.y += ds * Math.sin(thm); st.th = wrap(st.th + dth); st.thCum += dth;
  }
  return { st, step, wheelCmd };
}
