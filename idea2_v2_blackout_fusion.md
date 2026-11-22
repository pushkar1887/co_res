# Idea 2: Merged Plan (Claude + Codex)

**Status:** plan for review. **Nothing has been run on this design yet.** A draft simulator (`sim/sim.js`) exists but hasn't been tested. All numbers under "Evidence so far" come from earlier throwaway Python prototypes.

**Proposed paper title**
*Camera Blackouts at Warehouse Turns: Sensor Fusion, Fault-Aware Monitoring, and Pure Pursuit Tracking on a Simulated Differential-Drive Robot*

**In one paragraph.** A two-wheel warehouse robot follows a rounded 90° turn using Pure Pursuit. An overhead camera reports its pose 10 times a second. The robot also has quantized wheel encoders, a gyro and two geared DC motors. Camera reports drop out, either as short scripted patterns (v1) or as longer blackouts. Four estimators are compared on the same route, motors, loss schedules and 10 Hz steering clock: hold-last-command, dead reckoning, EKF, and EKF plus a gyro–encoder consistency monitor that slows the robot down. Sensor faults the filter can't see are added on purpose (gyro drift, wheel-size error, wheel slip). Closed-form error formulas are derived and checked against the simulation. The same estimator is ported to an ESP32 in the Wokwi simulator and checks itself against the browser version. **All data is simulated.** There is no real hardware or recorded data.

---

## 0. What comes from where

| Element | Source |
|---|---|
| Rounded-turn route, entry/mid/exit placement, matched loss patterns (8 isolated / 2×4 / burst of 8) | Codex v1 |
| p5.js browser simulator, one shared simulation function for demo and batch | Codex v1 |
| Quantized encoders (r = 33 mm, 360 CPR, 50 Hz), gyro at 50 Hz, camera at 10 Hz | Codex v2 |
| Two DC motors with first-order lag (τ = 0.12 s), logging commanded vs. actual wheel speed | Codex v2 |
| Gyro-bias calibration during the first stationary second | Codex v2 |
| **Dead reckoning as the key baseline**, same 10 Hz steering clock for all methods | Codex v2 |
| Wheel-slip stress test (one wheel −10% for 0.5 s) | Codex v2 |
| Steering-jump-on-camera-return and estimate-error metrics | Codex v2 |
| ESP32 firmware demo in Wokwi | Codex v2 |
| Higher speed (0.5 m/s) and long blackouts (0.5–4 s) | Claude v2 |
| Faults the filter can't see: gyro drift during a blackout, wheel-size error, left/right mismatch | Claude v2 |
| Bias as a 4th EKF state, motor saturation, warm-up drift, bias-instability noise model | Claude v2 |
| Gyro–encoder consistency monitor + slowdown (EKF+M) | Claude v2 |
| Closed-form error envelopes, survival time, detector formula, link to intermittent-Kalman theory | Claude v2 |
| Gilbert–Elliott channel at a matched 10% loss (v1's burstiness idea, made stochastic) | Claude v2 |
| Synthetic Allan-variance experiment (replaces real IMU data) | Claude v2, adapted to no-hardware |
| **Dropped:** IR line-sensor array | If a painted line exists, reviewers will ask why the robot doesn't just follow it |
| **Dropped:** real robot / real IMU tiers | Project constraint: no real data |

---

## 1. Research questions

- **RQ1: Short gaps (Codex's question).** With matched loss patterns at entry, mid-turn and exit, how much do dead reckoning and the EKF reduce tracking error beyond hold-last-command? Where does the EKF beat dead reckoning?
  *Expected, from the prototype:* both cut the error from about 1–2 cm to a few mm, and they track equally well inside the gap. The EKF wins on estimate accuracy (about 4×) and steering smoothness when the camera returns (about 10×).
- **RQ2: Blackouts × faults × phase.** For blackouts of 0.5–4 s, which sensor fault dominates, and does that depend on where in the turn the blackout lands?
  *Expected:* gyro drift grows as t² everywhere. Wheel-size error matters only while turning. Hold-last-command is catastrophic past about 1 s.
- **RQ3: Monitor.** Does the gyro–encoder consistency monitor detect these faults during a blackout, and what does slowing down cost?
  *Expected:* detection in about 0.6–1.2 s, no false alarms in fault-free runs, and the peak error cut by more than 50% at a cost of a few seconds.
- **RQ4: Burstiness (v1's idea).** At a matched 10% average loss, how does the mean burst length change peak error for each estimator?

---

## 2. System model

| Block | Model | Value |
|---|---|---|
| Route | Straight L1 → 90° left arc R → straight L3 | L1 = L3 = 3.0 m (longer than v1's 1.5 m so a 4 s blackout and its 3 s scoring window fit). R ∈ {0.55, 1.0, 1.5} m |
| Robot | Differential drive, wheel spacing b | b = 0.20 m |
| Speed / lookahead | Pure Pursuit, lookahead = 0.75 s × v | v ∈ {0.2, 0.5} m/s main; {0.3, 0.7} in sweeps. Lookahead is 0.15 m at 0.2 m/s, matching v1 |
| Motors | Command → saturation → first-order lag | τ = 0.12 s, \|u\| ≤ 1.0 m/s |
| Encoders | Quantized counts at 50 Hz | r = 33 mm, 360 CPR → 0.576 mm per count. Count the *wheel rotation*; slip and a wrong radius are invisible to them |
| Gyro | 50 Hz: true yaw rate + turn-on bias + warm-up drift + bias instability + white noise | ARW 0.015°/√s, Gauss–Markov bias σ 0.004°/s with τ = 100 s, turn-on bias σ 1°/s (removed by calibration), warm-up 0.2°/s with τ = 120 s. Assumed MEMS-class values |
| Camera | 10 Hz pose + Gaussian noise | σ_xy = 1 cm, σ_θ = 1°. Latency 0 (extension: 50–150 ms) |
| Calibration | Robot stands still for 1.0 s; gyro bias = mean of samples | Codex v2 |
| Physics | Fixed step | 10 ms. Sensors every 20 ms, camera and steering every 100 ms |
| Randomness | Seeded generator, separate streams per sensor | The same seed gives the same noise for every method (paired comparisons) |
| Truth metric | Signed distance from the **true** pose to the continuous route | Left of travel = positive |

---

## 3. Methods (same route, motors, loss schedule and 10 Hz steering clock)

| Method | Camera report arrives | Camera report missing |
|---|---|---|
| **HOLD** | Steer from the camera pose | Keep the last command (v1 baseline) |
| **DR** | Reset the pose to the camera report | Predict from encoders + calibrated gyro |
| **EKF** | Kalman update weighted by uncertainty | Predict from encoders + gyro, with bias as a state |
| **EKF+M** | As EKF; the alarm clears | As EKF. If the monitor alarms, drive at 30% speed until the camera returns |

---

## 4. The maths

**4.1 Encoders (Codex).** Wheel travel is $d_i = 2\pi r\,\Delta n_i/N$ and forward travel is $\Delta s = (d_L+d_R)/2$. One count is 0.58 mm. That's about 7 counts per 20 ms sample at 0.2 m/s and 17 at 0.5 m/s.

**4.2 Motors (Codex).** The wheel commands are $u_{L,R}^* = v_c \mp b\,\omega_c/2$, and each motor follows $\dot u_i = (\mathrm{sat}(u_i^*) - u_i)/\tau$.

**4.3 EKF (merged).** The state is $[x, y, \theta, b_g]$. Codex's 3-state filter is the special case with the bias fixed at its calibrated value. The prediction step runs every 20 ms:

$$x^- = x + \Delta s\cos\theta,\quad y^- = y + \Delta s\sin\theta,\quad \theta^- = \theta + (\omega_g - b_g)\Delta t,\quad b_g^- = b_g$$

It uses the Jacobian F (entries $-\Delta s\sin\theta$, $\Delta s\cos\theta$, $-\Delta t$) and process noise from encoder quantization ($\text{res}^2/12$), a 1% wheel-scale allowance, gyro ARW and a bias random walk. The camera update uses $H = [I_3\ 0]$, wraps the heading innovation, and symmetrizes P afterwards.

**4.4 Pure Pursuit is exact on a steady arc.** With the target a fixed arc length ahead of the projection, the steady state gives $r^2 = R^2$. So holding the last command mid-turn costs nothing, which is the v1 result.

**4.5 Blackout error envelopes** (first-order; t = time into the blackout):

| Error source | Predicted cross-track error | Phase dependence |
|---|---|---|
| HOLD across a curvature jump Δκ | $\Delta\kappa (vT)^2/2$, an upper bound (prototype measured ≈ ⅓ of it) | Entry and exit only |
| Gyro drift b | $\tfrac12 v b t^2$ | Everywhere |
| Gyro ARW N | $\sigma_e \approx v N t^{3/2}/\sqrt3$ | Everywhere |
| Left/right wheel mismatch δ (heading from encoders) | $v^2 \delta t^2/(2b)$. Irrelevant for DR and EKF, which take heading from the gyro | Everywhere |
| Both wheels ε too large (heading from gyro) | The robot drives radius $R/(1+\varepsilon)$: $e \approx \varepsilon R (1-\cos\varphi_b)$, where $\varphi_b$ = angle turned during the blackout | **Zero on straights**, grows in arcs |
| Slip on one wheel (with gyro) | The gyro sees the true yaw; only the along-track error ≈ slip·v·t/2 remains | Small |

**4.6 Survival time.** $T^* = \sqrt{2e_{tol}/(v b)}$. Example: 3 cm, 1°/s, 0.5 m/s gives T\* ≈ 2.6 s.

**4.7 Monitor.** The residual is $D(t) = \Delta\theta_{gyro} - \Delta\theta_{enc}$ over a sliding window $T_w$. The alarm fires if $|D| > h$ and the camera has been missing for more than 0.3 s.
- **Why a window, not the raw rate:** one count of left/right difference is 0.165° of heading. At 50 Hz that's 8.3°/s of apparent yaw-rate noise per sample. Over a window, though, the encoder heading error stays bounded at about ±2 counts (±0.33°) however long the window is. Gyro noise over 2 s is only about 0.02°. This is the main signal-processing point.
- **Detection formulas:** for a drift step b, $D = b\min(t, T_w)$. That gives a delay $t_d = h/b$ and a smallest detectable drift $b_{min} = h/T_w$.
- **Settings to fix before runs:** $T_w$ = 2.0 s and h = 0.6°, so b_min = 0.3°/s, a delay of 0.6 s at 1°/s and 1.2 s at 0.5°/s. *(The draft `sim.js` uses T_w = 1.0 s, which would miss a 0.5°/s drift. Change it to 2.0 s.)*
- **Bonus insight:** a wheel-size error shows up as $D \approx -\varepsilon\,\Delta\theta$, which is nonzero only while turning, exactly when it hurts. Slip also produces a D signal, so the monitor catches it too.

**4.8 Link to theory.** For intermittent observations, Sinopoli et al. (2004) bound the critical arrival rate below by $1 - 1/\max|\lambda(A)|^2$. Dead reckoning is marginally stable, so that bound is 0. The filter won't "diverge"; the practical question is time-to-corridor-breach (4.6).

---

## 5. Experiments (10 seeds per cell unless noted)

| Part | Question | Grid | Runs |
|---|---|---|---:|
| **A: Codex comparison** | RQ1 | R = 0.55; v ∈ {0.2, 0.5}; patterns {8 isolated, 2×4, burst of 8} × {entry, mid, exit}; faults {none, slip}; 4 methods | 1,440 |
| **B: Core** | RQ2, RQ3 | R = 1.0, v = 0.5; blackouts {0.5, 1, 2, 4 s} × {straight, entry, mid, exit}; faults {none, slip, drift 0.5°/s, drift 1°/s, wheels +3%, wheels ±1%}; 4 methods | 3,840 |
| **C: Scaling** | Do the 4.5 formulas hold? | 2 s blackout at entry; R ∈ {0.55, 1.0, 1.5} × v ∈ {0.3, 0.5, 0.7}; faults {drift 1°/s, wheels +3%}; 4 methods | 720 |
| **D: Burstiness** | RQ4 | Gilbert channel, 10% loss, mean burst {1, 4, 8, 16} reports, whole route, R = 1.0, v = 0.5; 4 methods; 20 seeds | 320 |
| **N: Baselines** | Sanity | No loss: R ∈ {0.55, 1.0, 1.5} × v ∈ {0.2, 0.3, 0.5, 0.7} × 4 methods | 480 |
| **V: Checks** (Codex) | Validity | Tracking with all reports; logged drop counts equal the schedule; halving the physics step changes peaks by < 1 mm | ~30 |

**Scoring rules (fixed before running):**
- **Placement.** Losses are placed from the no-loss robot's entry, mid and exit times.
- **Score window.** From the loss window's start to 3 s after it ends. A "straight" blackout ends 1 s before entry.
- **Excess error.** Each run is compared with the same seed, method and faults without camera loss, so excess error isolates the blackout.
- **Fault timing.** Faults start at the blackout start; slip starts 0.3 s in.
- **Fixed thresholds.** Corridor ±3 cm; monitor T_w = 2 s, h = 0.6°.
- **Reporting.** Median and IQR over seeds.

---

## 6. Metrics

| Metric | Source |
|---|---|
| Peak cross-track error, absolute and excess over the reference (mm) | v1 |
| Peak position-estimate error | Codex v2 |
| Steering jump when the camera returns (rad/s) | Codex v2 |
| Commanded vs. actual wheel speed traces | Codex v2 |
| Corridor breach (±3 cm) and survival time | Claude v2 |
| Recovery: overshoot, and time until the error stays under 1 cm | Claude v2 |
| NEES during the blackout (is the EKF honest?) | Claude v2 |
| Monitor detection delay, false alarms, time lost | Claude v2 |
| Fraction of time outside the corridor and RMS error (Part D) | Claude v2 |

**Figures:**
1. Part A bars: method × phase.
2. Part B heatmaps: fault × phase, one panel per method.
3. Error vs. blackout length with the 4.5 envelopes overlaid.
4. Scaling check against the formulas.
5. Burstiness curves.
6. **Viva figure:** the same blackout replayed with all 4 methods, with the camera timeline, monitor alarm, error over time, and motor commanded vs. actual speed.
7. Allan deviation of the synthetic gyro.

---

## 7. Data: everything is simulated

- **Experiment data** comes from the simulator with seeded noise. CSVs include every parameter, so any row can be reproduced.
- **Sensor characterisation without hardware.** Generate a 1-hour stationary gyro log from the same gyro model and compute the overlapping Allan deviation.
  - Show that the analysis recovers the injected ARW (slope −½) and the bias-instability floor.
  - Plot the warm-up drift. It explains why bias is left over after a 1 s calibration.
  - This validates the analysis pipeline and documents the noise model. Write it as **"assumed MEMS-class parameters"**, never as measured.
- **The ESP32 demo** replays a trace exported from the simulator, so it's simulated data too.

---

## 8. ESP32 demo (Wokwi, no hardware)

- **Firmware:**
  - `estimator.h` holds the same EKF, monitor and Pure Pursuit in float32.
  - `sketch.ino` runs a 50 Hz loop that replays `trace.h`.
  - `diagram.json` describes the circuit.
- **Trace:** `trace.h` is exported by the browser demo. It holds encoder count deltas, gyro samples, camera reports (including lost ones) and the browser EKF's pose after every step.
- **Self-check:** the firmware prints "max pose difference vs browser EKF = … mm", showing the embedded port matches the simulation.
- **Outputs:** serial CSV (pose, commands, monitor D, alarm), two PWM LEDs for left/right motor duty, a camera-status LED and an alarm LED.
- **Fault button:** pressing it injects +1°/s gyro drift into the replayed gyro. During the blackout the alarm LED lights within about 0.6 s.
- **Timing:** it also prints µs per estimator step. Wokwi timing is approximate, so call it indicative.
- **Limit (state it in the report):** replay is open loop, so the firmware's commands don't move the recorded robot. It proves the estimator and monitor, not closed-loop control.

---

## 9. Build plan (after review)

**Repo layout:**
- `sim/sim.js`: the one model (draft exists)
- `sim/experiments.js` + `sim/batch_worker.js`: the batch, run in a Web Worker
- `web/index.html` + `web/demo.js`: p5.js demo and trace export
- `web/batch.html`: batch page with a "Run" button
- `tools/serve.py`: local server that also saves CSVs into `results/`
- `analysis/make_figures.py`: figures and `results/summary.md`
- `firmware/esp32_demo/`: the ESP32 files
- `AGENTS.md`: shared instructions so Codex and Claude follow the same rules

**Machine constraint:** this PC has Python (numpy, matplotlib) but no Node.js and no C++ compiler. So the batch runs in the browser, Python is used only for analysis, and the firmware is compiled by Wokwi.

| Step | Work | Owner (proposal) |
|---|---|---|
| 1 | Finish and test `sim.js` (Part V checks) | Claude |
| 2 | Batch runner + CSV + synthetic gyro log | Claude |
| 3 | Figures + summary | Claude |
| 4 | p5.js demo UI | Codex or Claude |
| 5 | ESP32 Wokwi firmware + self-check | Claude, Codex reviews |
| 6 | Review `sim.js` against this plan (independent check) | **Codex** |
| 7 | Paper draft (IEEE 6-page) + viva script | Team + Codex/Claude |

---

## 10. Evidence so far (earlier Python prototypes; the new simulator will replace these)

**Codex settings** (0.2 m/s, R = 0.55, burst of 8, 10 seeds). Excess peak error in mm:

| Phase | HOLD | DR | EKF |
|---|---:|---:|---:|
| Entry | 13.4 | 2.8 | −0.1 |
| Mid | 8.2 | 0.2 | 0.3 |
| Exit | 6.8 | 2.2 | 0.0 |

- Estimate error: DR about 27 mm vs. EKF about 6 mm.
- Steering jump when the camera returns: DR 0.2–0.55 rad/s vs. EKF 0.004–0.05 rad/s.

**Claude v2 settings** (0.5 m/s, R = 1.0, 4 s blackout, 3 seeds). Excess peak error in cm:

| Fault | EKF | EKF+M |
|---|---:|---:|
| Gyro drift 1°/s | 6.0–8.2 | 1.8–3.8 |
| Wheels +3% | 0.9–3.0 | 0.7–1.2 |

- EKF+M had no false alarms in fault-free runs.

---

## 11. Honest rating (no real data)

| Version | College project | Workshop |
|---|---|---|
| This merged plan, simulation only | **9/10** | **~7** |

- **What holds it at 7:** simulation-only robotics papers get discounted. The monitor is close to *Gyrodometry* (Borenstein & Feng, ICRA 1996), so the novelty must be the blackout × phase analysis, the closed-form envelopes, and using the check to trigger a slowdown during blackouts.
- **What could push it to about 7.5:** the 4.5 formulas match the simulation tightly across the Part C sweeps, plus very clear writing.
- **The ESP32 port** adds engineering credibility, not empirical validation.

---

## 12. Questions for Codex (please answer before building)

1. **Steering clock:** keep 10 Hz for all methods (fair, your rule), or let DR and EKF steer at 50 Hz (more realistic)? *Proposal:* 10 Hz main, 50 Hz as one extra run.
2. **Wheel spacing** b = 0.20 m: OK for a small robot with 33 mm wheels?
3. **Gyro noise values** (ARW 0.015°/√s, bias instability 0.004°/s, warm-up 0.2°/s): reasonable MEMS-class assumptions?
4. **Monitor** T_w = 2 s, h = 0.6°, gated to more than 0.3 s without camera: agree on fixing these before any runs?
5. **Route** L1 = L3 = 3.0 m (yours was 1.5 m), needed for the 4 s blackout windows: OK?
6. **Part A** at both 0.2 and 0.5 m/s, or 0.2 only?
7. **Slip:** keep 10% for 0.5 s on the right wheel, starting 0.3 s into the loss, or place it somewhere else?
8. **Anything missing** for the viva demo or the paper?

---

## 13. References (checked against OpenAlex unless marked)

- Sinopoli et al., "Kalman Filtering With Intermittent Observations," *IEEE TAC*, 2004. [doi:10.1109/tac.2004.834121](https://doi.org/10.1109/tac.2004.834121)
- Kluge, Reif & Brokate, "Stochastic Stability of the Extended Kalman Filter With Intermittent Observations," *IEEE TAC*, 2010. [doi:10.1109/tac.2009.2037467](https://doi.org/10.1109/tac.2009.2037467)
- Huang & Dey, "Stability of Kalman filtering with Markovian packet losses," *Automatica*, 2007. [doi:10.1016/j.automatica.2006.10.023](https://doi.org/10.1016/j.automatica.2006.10.023)
- Gilbert, "Capacity of a Burst-Noise Channel," *Bell Syst. Tech. J.*, 1960. [doi:10.1002/j.1538-7305.1960.tb03959.x](https://doi.org/10.1002/j.1538-7305.1960.tb03959.x)
- Borenstein & Feng, "Measurement and correction of systematic odometry errors in mobile robots," *IEEE TRA*, 1996. [doi:10.1109/70.544770](https://doi.org/10.1109/70.544770)
- Borenstein & Feng, "Gyrodometry: a new method for combining data from gyros and odometry in mobile robots," *ICRA*, 1996. [doi:10.1109/robot.1996.503813](https://doi.org/10.1109/robot.1996.503813). **Closest prior work for the monitor.**
- El-Sheimy, Hou & Niu, "Analysis and Modeling of Inertial Sensors Using Allan Variance," *IEEE TIM*, 2008. [doi:10.1109/tim.2007.908635](https://doi.org/10.1109/tim.2007.908635)
- Batavia, Roth & Singh, "Autonomous Coverage Operations in Semi-Structured Outdoor Environments," *IROS*, 2002 ([PDF](https://frc.ri.cmu.edu/~ssingh/pubs/iros02_toro.pdf)). Pure Pursuit + EKF (GPS, encoders, gyro) through GPS outages: **closest prior work for the fusion baseline.** Checked by reading the PDF.
- Liu, Candell & Moayeri, *ISA Trans.*, 2017. [doi:10.1016/j.isatra.2017.02.005](https://doi.org/10.1016/j.isatra.2017.02.005)
- Carbonell et al., *ISA Trans.*, 2024. [doi:10.1016/j.isatra.2024.05.041](https://doi.org/10.1016/j.isatra.2024.05.041)
- Ferrari et al., *Sensors*, 2026. [doi:10.3390/s26030881](https://doi.org/10.3390/s26030881)
- Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm," CMU-RI-TR-92-01, 1992 (not re-checked).
- Page, "Continuous inspection schemes," *Biometrika*, 1954 (CUSUM extension; not checked).
- Engineering docs (not papers): WPILib pose estimators; Wokwi ESP32 guide; p5.js.
