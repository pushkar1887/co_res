// sensors.js: quantized encoders, gyro noise model, overhead camera, camera-loss channels (PLAN §4.2).
import { makeRng, streamSeed, STREAM, wrap } from './util.js';

export function makeEncoders(p, robot) {
  const radPerCount = 2 * Math.PI / p.cpr;
  let cL = 0, cR = 0;
  // count deltas since the last read and the wheel travel they imply (nominal radius)
  return function read() {
    const nL = Math.floor(robot.st.phiL / radPerCount), nR = Math.floor(robot.st.phiR / radPerCount);
    const dnL = nL - cL, dnR = nR - cR; cL = nL; cR = nR;
    return { dnL, dnR, dL: dnL * radPerCount * p.wheelR, dR: dnR * radPerCount * p.wheelR };
  };
}

// omega_g = omega + turn-on + warm-up + Gauss-Markov bias + fault step + white noise
export function makeGyro(p, seed, faults) {
  const g = makeRng(streamSeed(seed, STREAM.gyro)), dtS = 1 / p.sensHz, k = p.gyroNoiseScale;
  const turnOn = p.gyroTurnOn * g.n(), warmSign = g.u() < 0.5 ? -1 : 1;
  const phi = Math.exp(-dtS / p.gyroGMtau), sGM = k * p.gyroGMsig, drive = sGM * Math.sqrt(1 - phi * phi);
  const sig = k * p.gyroARW / Math.sqrt(dtS);
  let gm = sGM * g.n();
  return function sample(trueW, t) {
    gm = phi * gm + drive * g.n();
    const fault = faults.gyroStep && t >= faults.gyroAt - 1e-9 ? faults.gyroStep : 0;
    const bias = turnOn + warmSign * p.gyroWarmAmp * (1 - Math.exp(-t / p.gyroWarmTau)) + gm + fault;
    return { w: trueW + bias + sig * g.n(), bias };
  };
}

// Pose + Gaussian noise. Noise is drawn for EVERY slot, delivered or not, to keep streams paired.
export function makeCamera(p, seed) {
  const g = makeRng(streamSeed(seed, STREAM.camera)), k = p.camNoiseScale;
  return (x, y, th) => [x + k * p.camSigXY * g.n(), y + k * p.camSigXY * g.n(), wrap(th + k * p.camSigTh * g.n())];
}

// loss = {kind:'none'} | {kind:'slots', slots:[j...]} | {kind:'ge', lambda, meanBurst, from}
export function makeLoss(loss, seed, p) {
  if (!loss || loss.kind === 'none') return () => false;
  if (loss.kind === 'slots') { const set = new Set(loss.slots); return (j) => set.has(j); }
  if (loss.kind === 'ge') {   // Gilbert channel: every report in the bad state is lost (PLAN §4.2)
    const g = makeRng(streamSeed(seed, STREAM.channel)), r = 1 / loss.meanBurst, q = loss.lambda * r / (1 - loss.lambda);
    const from = Math.round(loss.from * p.camHz), cache = [];
    let bad = false;
    return (j) => {
      while (cache.length <= j) { const i = cache.length; if (i >= from) bad = bad ? g.u() >= r : g.u() < q; cache.push(i >= from && bad); }
      return cache[j];
    };
  }
  throw new Error('unknown loss kind ' + loss.kind);
}

// Stationary gyro log for the Allan-variance check (V6)
export function staticGyroLog(p, seconds, seed) {
  const sample = makeGyro(p, seed, {}), n = Math.round(seconds * p.sensHz), w = new Float64Array(n);
  for (let i = 0; i < n; i++) w[i] = sample(0, i / p.sensHz).w;
  return w;
}
