// params.js: every model parameter, with units and the PLAN section that justifies it.
// Change values ONLY here (PLAN §0 conventions). All data produced with these values is synthetic.

export const D2R = Math.PI / 180;

export const DEFAULTS = Object.freeze({
  // route (PLAN §4.1): straight L1 -> 90 deg left arc of radius R -> straight L3   [m]
  R: 1.0, L1: 3.0, L3: 5.0,
  // robot + actuators (PLAN §4.1, §4.2)
  v: 0.5,            // nominal forward speed [m/s]
  Ld: 0.30,          // Pure Pursuit lookahead, fixed per experiment [m]
  track: 0.20,       // wheel spacing b [m]
  wheelR: 0.033,     // nominal wheel radius r [m]
  cpr: 360,          // encoder counts per wheel revolution
  motorTau: 0.12,    // first-order motor time constant [s]
  wheelVmax: 1.0,    // wheel speed saturation [m/s]
  tStill: 1.0,       // stand still first; gyro bias calibrated over this interval [s]
  // clocks (PLAN §4.1): physics step, sensors, camera, steering (same for every arm)
  dt: 0.01, sensHz: 50, camHz: 10, steerHz: 10,
  // camera (PLAN §4.1)
  camSigXY: 0.010, camSigTh: 1.0 * D2R, camNoiseScale: 1.0,    // S2 doubles it
  // gyro (assumed MEMS-class, PLAN §4.1)
  gyroARW: 0.015 * D2R,                     // angle random walk [rad/sqrt(s)]
  gyroGMsig: 0.004 * D2R, gyroGMtau: 100,   // bias instability, Gauss-Markov [rad/s], [s]
  gyroTurnOn: 1.0 * D2R,                    // turn-on bias sigma [rad/s] (removed by calibration)
  gyroWarmAmp: 0.2 * D2R, gyroWarmTau: 120, // warm-up drift [rad/s], [s]
  gyroNoiseScale: 1.0,                      // S1 doubles ARW and bias instability
  // EKF tuning (PLAN §5.1)
  ekfScaleSig: 0.01,   // unmodelled wheel-scale allowance (fraction of ds)
  ekfBiasRW: 3e-4,     // bias random walk per sensor step [rad/s]. Tuned in M1 (PLAN §5.1, §20): smallest tested
                       // value whose camera-available reference recovers from a +1 deg/s step (< 10 mm extra error).
                       // 3e-6 left the filter so sure of its bias that the reference itself drifted 60 mm.
  // consistency monitor (PLAN §5.3, §6.8; frozen after CAL)
  monWin: 2.0,         // window T_w [s]
  monThr: 0.6 * D2R,   // threshold h [rad]
  monGate: 0.3,        // gate g: time since the first missed report [s]
  monRefTau: 10.0,     // v2 offset low-pass [s]: quantization averages to ~0.03 deg/s; warm-up lag < 0.02 deg/s
  // speed policies (PLAN §5.4)
  alpha: 0.3,          // speed fraction while slowed
  // scoring (PLAN §7, §8)
  corridor: 0.030,     // e_tol [m]
  scoreTail: 3.0,      // window extends this long after the camera returns [s]
  tMax: 200,           // hard stop [s]
});

// Arms = (estimator, speed policy). PLAN §5.1, §5.4.
export const ARMS = Object.freeze({
  hold: { est: 'hold', pol: 'N' },
  dr:   { est: 'dr',   pol: 'N' },
  ekf:  { est: 'ekf',  pol: 'N' },
  N:    { est: 'ekf',  pol: 'N' },
  SB:   { est: 'ekf',  pol: 'SB' },
  SA:   { est: 'ekf',  pol: 'SA' },
  SO:   { est: 'ekf',  pol: 'SO' },
});

// 20-slot (2 s) loss patterns: each drops 8 reports, mean dropped slot 9.5 (PLAN §7, Part A)
export const PATTERNS = Object.freeze({
  iso8:   [1, 3, 5, 7, 12, 14, 16, 18],
  '2x4':  [2, 3, 4, 5, 14, 15, 16, 17],
  burst8: [6, 7, 8, 9, 10, 11, 12, 13],
});
