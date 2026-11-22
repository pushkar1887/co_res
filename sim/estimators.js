// estimators.js: dead reckoning step and the 4-state EKF [x, y, theta, gyro bias] (PLAN §5.1).
import { wrap, inv3 } from './util.js';

export function drStep(X, ds, w, dtS) {
  const c = Math.cos(X[2]), s = Math.sin(X[2]);
  X[0] += ds * c; X[1] += ds * s; X[2] = wrap(X[2] + w * dtS);
}

export function ekfInitP() {
  return [[1e-4, 0, 0, 0], [0, 1e-4, 0, 0], [0, 0, (Math.PI / 180) ** 2, 0], [0, 0, 0, 1e-6]];
}

export function ekfPredict(X, P, ds, wg, dtS, p) {
  const c = Math.cos(X[2]), s = Math.sin(X[2]);
  X[0] += ds * c; X[1] += ds * s; X[2] = wrap(X[2] + (wg - X[3]) * dtS);
  const f02 = -ds * s, f12 = ds * c, f23 = -dtS;
  // P <- F P F^T with F = I + [0 0 f02 0; 0 0 f12 0; 0 0 0 f23; 0 0 0 0]
  const FP = [0, 1, 2, 3].map((i) => P[i].slice());
  for (let j = 0; j < 4; j++) { FP[0][j] += f02 * P[2][j]; FP[1][j] += f12 * P[2][j]; FP[2][j] += f23 * P[3][j]; }
  for (let i = 0; i < 4; i++) {
    const r = FP[i];
    P[i][0] = r[0] + r[2] * f02; P[i][1] = r[1] + r[2] * f12; P[i][2] = r[2] + r[3] * f23; P[i][3] = r[3];
  }
  // + G diag(sigma_ds^2, N^2 dtS) G^T + diag(0,0,0,q_b)
  const res = 2 * Math.PI * p.wheelR / p.cpr;
  const qds = res * res / 12 + (p.ekfScaleSig * ds) ** 2, qth = (p.gyroNoiseScale * p.gyroARW) ** 2 * dtS;
  P[0][0] += c * c * qds; P[0][1] += c * s * qds; P[1][0] += c * s * qds; P[1][1] += s * s * qds;
  P[2][2] += qth; P[3][3] += p.ekfBiasRW * p.ekfBiasRW;
}

export function ekfUpdate(X, P, z, Rd) {             // H = [I3 0]
  const S = [0, 1, 2].map((i) => [0, 1, 2].map((j) => P[i][j] + (i === j ? Rd[i] : 0)));
  const Si = inv3(S), inn = [z[0] - X[0], z[1] - X[1], wrap(z[2] - X[2])];
  const K = [0, 1, 2, 3].map((i) => [0, 1, 2].map((j) => P[i][0] * Si[0][j] + P[i][1] * Si[1][j] + P[i][2] * Si[2][j]));
  for (let i = 0; i < 4; i++) X[i] += K[i][0] * inn[0] + K[i][1] * inn[1] + K[i][2] * inn[2];
  X[2] = wrap(X[2]);
  const top = [P[0].slice(), P[1].slice(), P[2].slice()];
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) P[i][j] -= K[i][0] * top[0][j] + K[i][1] * top[1][j] + K[i][2] * top[2][j];
  for (let i = 0; i < 4; i++) for (let j = 0; j < i; j++) { const m = 0.5 * (P[i][j] + P[j][i]); P[i][j] = m; P[j][i] = m; }
}
