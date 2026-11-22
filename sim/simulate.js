// simulate.js: one closed-loop run. Order inside a physics tick at time t (PLAN §4, §5):
//   1. sensors (50 Hz): encoders + gyro -> calibration or prediction -> monitor
//   2. camera (10 Hz): delivered report -> EKF update / DR + HOLD reset
//   3. steering (10 Hz): speed policy -> Pure Pursuit
//   4. log a same-timestamp snapshot (truth at t, estimate after correction, commands)
//   5. plant + motors advance from t to t + dt
import { DEFAULTS, ARMS } from './params.js';
import { makePath } from './route.js';
import { makeRobot } from './robot.js';
import { makeEncoders, makeGyro, makeCamera, makeLoss } from './sensors.js';
import { drStep, ekfInitP, ekfPredict, ekfUpdate } from './estimators.js';
import { makeMonitor } from './monitor.js';
import { purePursuit, policySpeed } from './control.js';

const LOG_KEYS = ['t', 'x', 'y', 'th', 'xe', 'ye', 'the', 'e', 's', 'seg', 'vc', 'wc', 'uL', 'uR', 'uLc', 'uRc',
  'alarm', 'D', 'missAge', 'P00', 'P01', 'P11', 'dnL', 'dnR', 'wg', 'bias'];

export function simulate(cfg) {
  const p = { ...DEFAULTS, ...cfg };
  const arm = ARMS[p.arm];
  if (!arm) throw new Error('unknown arm ' + p.arm);
  const seed = p.seed ?? 1, f = p.faults || {}, tF = f.onset ?? Infinity;
  const path = makePath(p.R, p.L1, p.L3), robot = makeRobot(p, f);
  const readEnc = makeEncoders(p, robot), gyro = makeGyro(p, seed, f), camera = makeCamera(p, seed);
  const lost = makeLoss(p.loss, seed, p), monitor = makeMonitor(p);
  const sensEvery = Math.round(1 / (p.sensHz * p.dt)), camEvery = Math.round(1 / (p.camHz * p.dt));
  const steerEvery = Math.round(1 / (p.steerHz * p.dt)), dtS = 1 / p.sensHz;
  const kc = p.camNoiseScale, Rd = [(kc * p.camSigXY) ** 2, (kc * p.camSigXY) ** 2, (kc * p.camSigTh) ** 2];

  const X = [0, 0, 0, 0], P = ekfInitP();
  let calSum = 0, calN = 0, bCal = 0, calibrated = false;
  let tFix = 0, missed = 0, alarm = false, D = 0, vc = 0, wc = 0, jumpPending = false, freshSinceSteer = false;
  const L = Object.fromEntries(LOG_KEYS.map((k) => [k, []]));
  const cam = { j: [], ok: [], z: [] }, steerJumps = [], alarmOn = [];
  let finish = NaN, last = { dnL: 0, dnR: 0, wg: 0, bias: 0 }, thCumPrev = 0;
  const missAgeAt = (t) => (missed > 0 ? t - (tFix + 1 / p.camHz) : -1);

  const nMax = Math.round(p.tMax / p.dt);
  for (let k = 0; k <= nMax; k++) {
    const t = k * p.dt, moving = t >= p.tStill - 1e-9, sensTick = k % sensEvery === 0;

    // 1. sensors
    if (sensTick) {
      // gyro reports the MEAN true yaw rate over the last sample interval (integrating / DLPF MEMS output),
      // so rectangle integration of the gyro matches the plant exactly; point sampling would add a
      // 0.5*dt*d(omega) heading error at every curvature change (found in V smoke test, PLAN §20).
      const trueRate = (robot.st.thCum - thCumPrev) / dtS; thCumPrev = robot.st.thCum;
      const enc = readEnc(), gs = gyro(trueRate, t), wg = gs.w;
      last = { dnL: enc.dnL, dnR: enc.dnR, wg, bias: gs.bias };
      if (!calibrated) {
        if (!moving) { calSum += wg; calN++; }
        else {
          bCal = calSum / Math.max(calN, 1); calibrated = true; X[3] = bCal;
          P[3][3] = (p.gyroNoiseScale * p.gyroARW) ** 2 / (dtS * Math.max(calN, 1));
        }
      }
      if (calibrated) {
        const ds = 0.5 * (enc.dL + enc.dR);
        if (arm.est === 'dr') drStep(X, ds, wg - bCal, dtS);
        else if (arm.est === 'ekf') ekfPredict(X, P, ds, wg, dtS, p);
        D = monitor(wg - (arm.est === 'ekf' ? X[3] : bCal), enc.dL, enc.dR);
        if (arm.est !== 'hold' && moving && !alarm && missAgeAt(t) >= p.monGate - 1e-9 && Math.abs(D) > p.monThr) {
          alarm = true; alarmOn.push(t);
        }
      }
    }

    // 2. camera
    if (k % camEvery === 0) {
      const j = Math.round(t * p.camHz), z = camera(robot.st.x, robot.st.y, robot.st.th), ok = !lost(j);
      cam.j.push(j); cam.ok.push(ok ? 1 : 0); cam.z.push(z);
      if (ok) {
        if (missed > 0) jumpPending = true;
        tFix = t; missed = 0; alarm = false; freshSinceSteer = true;
        if (arm.est === 'ekf') ekfUpdate(X, P, z, Rd);
        else { X[0] = z[0]; X[1] = z[1]; X[2] = z[2]; }
      } else missed++;
    }

    // 3. steering (HOLD only re-steers on a fresh report)
    if (k % steerEvery === 0 && moving && (arm.est !== 'hold' || freshSinceSteer)) {
      const vcmd = policySpeed(arm.pol, p, { missAge: missAgeAt(t), alarm, faultActive: t >= tF - 1e-9 });
      const wnew = purePursuit(path, X, p.Ld, vcmd);
      if (jumpPending) { steerJumps.push([t, Math.abs(wnew - wc)]); jumpPending = false; }
      vc = vcmd; wc = wnew; freshSinceSteer = false;
    }

    // 4. same-timestamp snapshot
    if (sensTick) {
      const st = robot.st, pr = path.project(st.x, st.y), [uLc, uRc] = robot.wheelCmd(vc, wc);
      const row = { t, x: st.x, y: st.y, th: st.th, xe: X[0], ye: X[1], the: X[2], e: pr.e, s: pr.s, seg: path.segment(pr.s),
        vc, wc, uL: st.uL, uR: st.uR, uLc, uRc, alarm: alarm ? 1 : 0, D, missAge: missAgeAt(t),
        P00: P[0][0], P01: P[0][1], P11: P[1][1], ...last };
      for (const key of LOG_KEYS) L[key].push(row[key]);
      if (moving && pr.s >= path.total - 0.05) { finish = t; break; }
    }

    // 5. plant
    robot.step(t, vc, wc);
  }
  return { p, path, arm: p.arm, seed, log: L, cam, finish, steerJumps, alarmOn, bCal, tF };
}
