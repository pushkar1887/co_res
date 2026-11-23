// estimator.h: float32 port of the browser simulator's estimator and controller for the ESP32 replay
// (PLAN §8, check V8). Each function mirrors one simulator file:
//   Route::project/point   <- sim/route.js      Ekf::predict/update <- sim/estimators.js
//   Monitor::update        <- sim/monitor.js    purePursuit         <- sim/control.js
//   Replay::step           <- the step order of sim/simulate.js at a 20 ms sensor tick:
//                             sensors -> camera (every 5th tick) -> steering (every 5th tick)
// Time is counted in integer ticks, so gate and calibration decisions match the simulator exactly.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "sim_params.h"

// One 20 ms sensor step of the recorded run. cam: 0 = no camera slot, 1 = report delivered, 2 = report lost.
struct TraceRow { int16_t dnL, dnR; float gyro; uint8_t cam; float cx, cy, cth; };
// The browser's state after the same step: pose, residual D, wheel commands, speed and turn commands, alarm.
struct RefRow { float x, y, th, D, uL, uR, vc, wc; uint8_t alarm; };

namespace est {

const float PI_F = 3.14159265358979f;
inline float clampf(float a, float lo, float hi) { return a < lo ? lo : (a > hi ? hi : a); }
inline float wrapf(float a) { a = fmodf(a + PI_F, 2.0f * PI_F); if (a < 0) a += 2.0f * PI_F; return a - PI_F; }

// Straight L1 -> 90 deg left arc of radius R -> straight L3.
struct Route {
  float R, L1, L3, La, cx, cy;
  void init(float r, float l1, float l3) { R = r; L1 = l1; L3 = l3; La = PI_F / 2 * R; cx = L1; cy = R; }
  void point(float s, float& x, float& y) const {
    if (s <= L1) { x = s; y = 0; return; }
    if (s <= L1 + La) { float a = -PI_F / 2 + (s - L1) / R; x = cx + R * cosf(a); y = cy + R * sinf(a); return; }
    x = L1 + R; y = R + (s - L1 - La);
  }
  float project(float x, float y) const {          // arc length of the closest route point
    float s1 = clampf(x, 0, L1), bestD = hypotf(x - s1, y), bestS = s1;
    float a = clampf(atan2f(y - cy, x - cx), -PI_F / 2, 0);
    float d2 = hypotf(x - (cx + R * cosf(a)), y - (cy + R * sinf(a)));
    if (d2 < bestD) { bestD = d2; bestS = L1 + (a + PI_F / 2) * R; }
    float s3 = clampf(y - R, 0, L3), d3 = hypotf(x - (L1 + R), y - (R + s3));
    if (d3 < bestD) { bestD = d3; bestS = L1 + La + s3; }
    return bestS;
  }
};

inline void inv3(const float m[3][3], float o[3][3]) {
  float a = m[0][0], b = m[0][1], c = m[0][2], d = m[1][0], e = m[1][1], f = m[1][2], g = m[2][0], h = m[2][1], i = m[2][2];
  float A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g, det = a * A + b * B + c * C;
  o[0][0] = A / det; o[0][1] = -(b * i - c * h) / det; o[0][2] = (b * f - c * e) / det;
  o[1][0] = B / det; o[1][1] = (a * i - c * g) / det;  o[1][2] = -(a * f - c * d) / det;
  o[2][0] = C / det; o[2][1] = -(a * h - b * g) / det; o[2][2] = (a * e - b * d) / det;
}

// 4-state EKF: [x, y, theta, gyro bias] (PLAN §5.1).
struct Ekf {
  float X[4];
  float P[4][4];
  void init() {
    memset(X, 0, sizeof(X)); memset(P, 0, sizeof(P));
    P[0][0] = 1e-4f; P[1][1] = 1e-4f; P[2][2] = (PI_F / 180) * (PI_F / 180); P[3][3] = 1e-6f;
  }
  void predict(float ds, float wg, float dtS) {
    float c = cosf(X[2]), s = sinf(X[2]);
    X[0] += ds * c; X[1] += ds * s; X[2] = wrapf(X[2] + (wg - X[3]) * dtS);
    float f02 = -ds * s, f12 = ds * c, f23 = -dtS;
    float FP[4][4];
    memcpy(FP, P, sizeof(P));
    for (int j = 0; j < 4; j++) { FP[0][j] += f02 * P[2][j]; FP[1][j] += f12 * P[2][j]; FP[2][j] += f23 * P[3][j]; }
    for (int i = 0; i < 4; i++) {
      P[i][0] = FP[i][0] + FP[i][2] * f02; P[i][1] = FP[i][1] + FP[i][2] * f12;
      P[i][2] = FP[i][2] + FP[i][3] * f23; P[i][3] = FP[i][3];
    }
    const float res = 2.0f * PI_F * P_WHEEL_R / P_CPR;
    float qds = res * res / 12.0f + (P_EKF_SCALE_SIG * ds) * (P_EKF_SCALE_SIG * ds), qth = P_GYRO_ARW * P_GYRO_ARW * dtS;
    P[0][0] += c * c * qds; P[0][1] += c * s * qds; P[1][0] += c * s * qds; P[1][1] += s * s * qds;
    P[2][2] += qth; P[3][3] += P_EKF_BIAS_RW * P_EKF_BIAS_RW;
  }
  void update(const float z[3], const float Rd[3]) {   // H = [I3 0]
    float S[3][3], Si[3][3];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) S[i][j] = P[i][j] + (i == j ? Rd[i] : 0.0f);
    inv3(S, Si);
    float inn[3] = { z[0] - X[0], z[1] - X[1], wrapf(z[2] - X[2]) };
    float K[4][3];
    for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) K[i][j] = P[i][0] * Si[0][j] + P[i][1] * Si[1][j] + P[i][2] * Si[2][j];
    for (int i = 0; i < 4; i++) X[i] += K[i][0] * inn[0] + K[i][1] * inn[1] + K[i][2] * inn[2];
    X[2] = wrapf(X[2]);
    float top[3][4];
    memcpy(top, P, sizeof(top));
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) P[i][j] -= K[i][0] * top[0][j] + K[i][1] * top[1][j] + K[i][2] * top[2][j];
    for (int i = 0; i < 4; i++) for (int j = 0; j < i; j++) { float m = 0.5f * (P[i][j] + P[j][i]); P[i][j] = m; P[j][i] = m; }
  }
};

// Gyro-encoder consistency monitor v2 (PLAN §5.3): own offset o, frozen while the camera is missing.
struct Monitor {
  float ring[P_MON_WIN_N];
  float o, cum;
  int i;
  bool full;
  void init(float oInit) { memset(ring, 0, sizeof(ring)); o = oInit; cum = 0; i = 0; full = false; }
  float update(float wg, float dL, float dR, bool fresh, float dtS) {
    float r = wg - (dR - dL) / (P_TRACK * dtS);
    if (fresh) o += (r - o) * dtS / P_MON_REF_TAU;
    cum += (r - o) * dtS;
    float old = ring[i];
    ring[i] = cum; i = (i + 1) % P_MON_WIN_N; if (i == 0) full = true;
    return full ? cum - old : 0.0f;
  }
};

inline float purePursuit(const Route& rt, const float X[4], float Ld, float vc) {
  float s = rt.project(X[0], X[1]), gx, gy;
  rt.point(s + Ld, gx, gy);
  float dx = gx - X[0], dy = gy - X[1], yg = -sinf(X[2]) * dx + cosf(X[2]) * dy;
  return 2.0f * vc * yg / (dx * dx + dy * dy);      // omega_c = 2 v_c y_g / d^2
}

// The robot's estimator + policy SA ("slow on alarm"), driven by replayed sensor data.
struct Replay {
  Route route; Ekf ekf; Monitor mon;
  bool calibrated, alarm;
  double calSum;
  int calN, tFixTick, missed;
  float bCal, D, vc, wc;
  void reset() {
    route.init(P_R, P_L1, P_L3); ekf.init();
    calibrated = false; alarm = false; calSum = 0; calN = 0; tFixTick = 0; missed = 0; bCal = 0; D = 0; vc = 0; wc = 0;
  }
  void step(int i, const TraceRow& row, float gyroExtra) {
    const float dtS = 1.0f / P_SENS_HZ, radPerCount = 2.0f * PI_F / P_CPR;
    const bool moving = i >= P_TSTILL_TICKS;
    const float dL = row.dnL * radPerCount * P_WHEEL_R, dR = row.dnR * radPerCount * P_WHEEL_R, wg = row.gyro + gyroExtra;
    // 1. sensors: stationary calibration, then prediction and monitor
    if (!calibrated) {
      if (!moving) { calSum += wg; calN++; }
      else {
        int n = calN > 0 ? calN : 1;
        bCal = (float)(calSum / n); calibrated = true; ekf.X[3] = bCal; mon.init(bCal);
        ekf.P[3][3] = P_GYRO_ARW * P_GYRO_ARW / (dtS * n);
      }
    }
    if (calibrated) {
      ekf.predict(0.5f * (dL + dR), wg, dtS);
      D = mon.update(wg, dL, dR, missed == 0, dtS);
      int missAge = missed > 0 ? i - (tFixTick + P_CAM_EVERY) : -1;   // ticks since the first missed report
      if (moving && !alarm && missAge >= P_MON_GATE_TICKS && fabsf(D) > P_MON_THR) alarm = true;
    }
    // 2. camera (10 Hz)
    if (i % P_CAM_EVERY == 0) {
      if (row.cam == 1) {
        tFixTick = i; missed = 0; alarm = false;
        const float z[3] = { row.cx, row.cy, row.cth };
        const float Rd[3] = { P_CAM_SIG_XY * P_CAM_SIG_XY, P_CAM_SIG_XY * P_CAM_SIG_XY, P_CAM_SIG_TH * P_CAM_SIG_TH };
        ekf.update(z, Rd);
      } else if (row.cam == 2) missed++;
    }
    // 3. steering (10 Hz): policy SA, then Pure Pursuit
    if (i % P_STEER_EVERY == 0 && moving) {
      vc = alarm ? P_ALPHA * P_V : P_V;
      wc = purePursuit(route, ekf.X, P_LD, vc);
    }
  }
  bool cameraFresh() const { return missed == 0; }
  void wheelCmd(float& uL, float& uR) const {
    uL = clampf(vc - wc * P_TRACK / 2, -P_WHEEL_VMAX, P_WHEEL_VMAX);
    uR = clampf(vc + wc * P_TRACK / 2, -P_WHEEL_VMAX, P_WHEEL_VMAX);
  }
};

}  // namespace est
