// sketch.ino: ESP32 (Wokwi) replay of two simulated runs from the browser simulator (PLAN §8, check V8).
//
// Pass 1 "healthy": 4 s camera blackout, no fault. Press the button DURING the blackout to inject a
//                   +1 deg/s gyro fault into the replayed gyro; the alarm LED should light ~0.5-0.6 s later.
// Pass 2 "gyro_fault": the recorded run with a +1 deg/s gyro fault starting 0.5 s into the blackout.
// After each pass the firmware compares its own estimate with the browser's at every 20 ms step and prints
// "V8 PARITY: PASS/FAIL" (pose < 1 mm, identical alarm states, wheel commands < 1e-3 m/s).
// Replay is open loop: the recorded robot does not respond to these commands (PLAN §8 limitation).
//
// LEDs: GPIO25 = left-wheel command, GPIO26 = right-wheel command (brightness = speed),
//       GPIO27 = camera fresh (off during the blackout), GPIO14 = monitor alarm. Button: GPIO13 to GND.
#include "estimator.h"
#include "trace_healthy.h"
#include "trace_gyro_fault.h"

const int PIN_LED_L = 25, PIN_LED_R = 26, PIN_LED_CAM = 27, PIN_LED_ALARM = 14, PIN_BTN = 13;

struct Pass { const char* name; const TraceRow* tr; const RefRow* ref; int n, onTick, retTick, faultTick; };
const Pass PASSES[2] = {
  { "healthy", TRACE_HEALTHY, REF_HEALTHY, TRACE_HEALTHY_N, TRACE_HEALTHY_ON_TICK, TRACE_HEALTHY_RET_TICK, TRACE_HEALTHY_FAULT_TICK },
  { "gyro_fault", TRACE_GYRO_FAULT, REF_GYRO_FAULT, TRACE_GYRO_FAULT_N, TRACE_GYRO_FAULT_ON_TICK, TRACE_GYRO_FAULT_RET_TICK, TRACE_GYRO_FAULT_FAULT_TICK },
};

est::Replay rp;
int passIdx = 0, tick = 0, injectTick = -1, alarmDiff = 0, lastBtn = HIGH;
bool prevAlarm = false;
float maxPos = 0, maxTh = 0, maxCmd = 0, maxD = 0;
uint32_t nextUs = 0, sumUs = 0, maxUs = 0, lastBtnMs = 0;

int duty(float u) { return (int)(255.0f * fminf(fabsf(u) / P_WHEEL_VMAX, 1.0f) + 0.5f); }

void startPass(int k) {
  passIdx = k; tick = 0; injectTick = -1; alarmDiff = 0; prevAlarm = false;
  maxPos = maxTh = maxCmd = maxD = 0; sumUs = maxUs = 0;
  rp.reset();
  const Pass& ps = PASSES[k];
  Serial.printf("\n# ===== PASS %d: %s (%d steps of 20 ms) =====\n", k + 1, ps.name, ps.n);
  Serial.printf("# blackout %.2f-%.2f s%s\n", ps.onTick * 0.02f, ps.retTick * 0.02f,
                ps.faultTick >= 0 ? "; recorded gyro fault +1 deg/s" : "; press the button during the blackout to inject a gyro fault");
  if (ps.faultTick >= 0) Serial.printf("# recorded fault onset t=%.2f s\n", ps.faultTick * 0.02f);
  Serial.println("t_s,x_m,y_m,th_deg,D_deg,alarm,cam,vc_mps,uL_mps,uR_mps,pose_err_mm");
  nextUs = micros();
}

void endPass() {
  const Pass& ps = PASSES[passIdx];
  Serial.printf("# PASS %s done: max pose diff %.4f mm, max heading diff %.5f deg, max |dD| %.5f deg, alarm rows differing %d of %d, max wheel-cmd diff %.6f m/s\n",
                ps.name, maxPos * 1000.0f, maxTh * 180.0f / est::PI_F, maxD * 180.0f / est::PI_F, alarmDiff, ps.n, maxCmd);
  Serial.printf("# estimator+monitor+steering step: mean %lu us, max %lu us (Wokwi timing is indicative)\n", (unsigned long)(sumUs / ps.n), (unsigned long)maxUs);
  if (injectTick >= 0) {
    Serial.printf("# fault injected by button at t=%.2f s: the input differs from the browser run, so parity is not expected\n", injectTick * 0.02f);
  } else {
    bool pass = maxPos < 0.001f && alarmDiff == 0 && maxCmd < 0.001f;
    Serial.printf("# V8 PARITY (%s): %s\n", ps.name, pass ? "PASS" : "FAIL");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_CAM, OUTPUT); pinMode(PIN_LED_ALARM, OUTPUT); pinMode(PIN_BTN, INPUT_PULLUP);
  Serial.println("# co_res ESP32 replay: EKF + gyro-encoder monitor + Pure Pursuit (float32). All data simulated.");
  startPass(0);
}

void loop() {
  if ((int32_t)(micros() - nextUs) < 0) return;        // 50 Hz pacing
  nextUs += 20000;
  const Pass& ps = PASSES[passIdx];

  int b = digitalRead(PIN_BTN);                          // falling edge = press (debounced)
  if (b == LOW && lastBtn == HIGH && millis() - lastBtnMs > 200 && injectTick < 0) {
    injectTick = tick; lastBtnMs = millis();
    Serial.printf("# BUTTON: injecting +1 deg/s gyro fault from t=%.2f s\n", tick * 0.02f);
  }
  lastBtn = b;

  uint32_t t0 = micros();
  rp.step(tick, ps.tr[tick], injectTick >= 0 ? P_FAULT_STEP : 0.0f);
  float uL, uR;
  rp.wheelCmd(uL, uR);
  uint32_t us = micros() - t0;
  sumUs += us; if (us > maxUs) maxUs = us;

  const RefRow& R = ps.ref[tick];                        // parity against the browser, same timestamp
  float pe = hypotf(rp.ekf.X[0] - R.x, rp.ekf.X[1] - R.y);
  maxPos = fmaxf(maxPos, pe);
  maxTh = fmaxf(maxTh, fabsf(est::wrapf(rp.ekf.X[2] - R.th)));
  maxD = fmaxf(maxD, fabsf(rp.D - R.D));
  maxCmd = fmaxf(maxCmd, fmaxf(fabsf(uL - R.uL), fabsf(uR - R.uR)));
  if ((rp.alarm ? 1 : 0) != R.alarm) alarmDiff++;

  analogWrite(PIN_LED_L, duty(uL));
  analogWrite(PIN_LED_R, duty(uR));
  digitalWrite(PIN_LED_CAM, rp.cameraFresh() ? HIGH : LOW);
  digitalWrite(PIN_LED_ALARM, rp.alarm ? HIGH : LOW);

  const float t = tick * 0.02f;
  if (tick == ps.onTick) Serial.printf("# CAMERA LOST t=%.2f s\n", t);
  if (rp.alarm && !prevAlarm) Serial.printf("# ALARM t=%.2f s -> speed capped at %.2f m/s\n", t, P_ALPHA * P_V);
  if (tick == ps.retTick) Serial.printf("# CAMERA BACK t=%.2f s\n", t);
  prevAlarm = rp.alarm;
  if (tick % 5 == 0)
    Serial.printf("%.2f,%.4f,%.4f,%.3f,%.4f,%d,%d,%.3f,%.4f,%.4f,%.4f\n", t, rp.ekf.X[0], rp.ekf.X[1], rp.ekf.X[2] * 180.0f / est::PI_F,
                  rp.D * 180.0f / est::PI_F, rp.alarm ? 1 : 0, rp.cameraFresh() ? 1 : 0, rp.vc, uL, uR, pe * 1000.0f);

  if (++tick >= ps.n) {
    endPass();
    delay(1500);
    startPass((passIdx + 1) % 2);
  }
}
