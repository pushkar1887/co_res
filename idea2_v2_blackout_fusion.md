# Idea 2 v2: Localization Blackouts at Turns

**Proposed paper title**
*When the Position Fix Drops Out Mid-Turn: Error Envelopes, Sensor Ablation, and a Gyro–Odometry Consistency Monitor for Pure Pursuit on a Differential-Drive Robot*

**In one paragraph.** A two-wheel warehouse robot follows a rounded 90° turn. It gets an absolute position fix 10 times a second (from an overhead camera, UWB or lidar localization over Wi-Fi). It also has its own wheel encoders and a gyro, fused in an Extended Kalman Filter (EKF). When the fix drops out for 0.5–4 s (the robot passes behind a rack, Wi-Fi roams, the camera is blocked), the robot keeps going on dead reckoning. The paper answers three questions. **Which sensor error actually hurts during a blackout? Does it depend on where in the turn the blackout lands? And can a cheap monitor that compares two sensors catch it in time to slow down?** Each answer comes with a closed-form prediction that the simulation checks, and optionally with a small real robot.

---

## 0. What changed from v1

| | v1 (Codex) | v2 (this brief) |
|---|---|---|
| Sensors | Absolute pose only | Absolute fix + wheel encoders + gyro (IMU) |
| Actuators | Ideal: the commanded turn rate is applied instantly | Motor first-order lag (τ = 0.1 s) plus wheel-speed saturation |
| Estimator | None: hold the last steering command | 4-state EKF (x, y, θ, gyro bias), compared against 3 simpler options |
| Signals content | none | Allan-variance sensor characterisation, residual-based fault detection (threshold / CUSUM) |
| Maths | Pure Pursuit formula | Kinematics, EKF equations, 5 closed-form blackout error envelopes, detector delay formula, link to intermittent-observation Kalman theory |
| Result | Predictable: "only bursts at a curvature change matter" (~1 cm at 0.2 m/s) | Fault type × turn phase interaction, plus a mitigation with a measured cost (5–8 cm errors cut by 50–75% in the prototype) |
| Workshop rating | ~4/10 | ~7 sim-only, **7.5–8 with real-sensor data** (see §11) |

**Will a Kalman filter work here? Yes, but on its own it kills the paper.** In my prototype, an EKF with a gyro and a learned gyro bias kept a **4-second blackout to 1–3 cm** of error. So "we added a Kalman filter and it helped" is not a finding. The finding is **what the filter cannot see during a blackout** (a gyro bias it hasn't learned, wheel-size errors) and **where on the path those errors turn into sideways error**.

---

## 1. Research questions and hypotheses

- **RQ1: Error envelopes.** For each sensor-error type, how does cross-track error grow with blackout length? Do simple closed-form formulas predict it?
  *H1:* An unlearned gyro bias and a left/right wheel mismatch grow as t². A common wheel-size error grows only while the robot is turning.
- **RQ2: Phase interaction.** Does the same blackout cause different error at straight / entry / mid-turn / exit, and does that depend on the error type?
  *H2:* Yes. Wheel-size error is harmless on straights and bad in arcs. Gyro bias is bad everywhere. Holding the last command with no fusion is bad only at curvature jumps (the v1 result).
- **RQ3: Mitigation.** Can a gyro-vs-encoder consistency monitor detect these faults during a blackout and trigger a slowdown? What does it cost in time and false alarms?
  *H3:* It catches both fault types with zero false alarms in nominal runs and cuts the peak error by more than 50%.

---

## 2. System model (sensors, actuators, signals)

| Block | Model | Prototype value (replace with datasheet / measured values) |
|---|---|---|
| Path | 3.0 m straight → 90° arc R → 2.0 m straight | R ∈ {0.55, 1.0, 1.5} m. The long first straight lets the gyro-bias estimate converge. |
| Robot | Differential drive, track width W | W = 0.30 m, v ∈ {0.3, 0.5, 0.7} m/s |
| Actuators | Wheel-speed command → saturation → first-order lag | τ = 0.10 s, \|v_wheel\| ≤ 0.9 m/s |
| Wheel encoders | 100 Hz, v_meas = v_true·(1+ε) + noise | σ = 0.005 m/s per sample. Faults: common ε = +3%, differential ε = ±1% |
| Gyro (yaw) | 100 Hz, ω_meas = ω + b + noise | σ = 0.003 rad/s per sample, residual bias b_res ∈ {0, 0.5, 1.0}°/s |
| Absolute fix | 10 Hz, pose + Gaussian noise | σ_xy = 1 cm, σ_θ = 1°. Extension: latency 50–150 ms |
| Channel | Scripted blackouts + Gilbert–Elliott two-state Markov loss | Blackouts 0.5, 1, 2, 4 s |
| Controller | Pure Pursuit on the *estimated* pose, 100 Hz | Lookahead 0.30 m (≈ 0.6 s × v) |
| Truth / metric | Cross-track error from the **true** pose to the continuous path | Physics step 10 ms |

**Kinematics and actuators**

$$v=\tfrac{v_R+v_L}{2},\quad \omega=\tfrac{v_R-v_L}{W},\qquad \dot x=v\cos\theta,\ \dot y=v\sin\theta,\ \dot\theta=\omega$$

$$\dot v_{R,L}=\tfrac{1}{\tau}\big(\mathrm{sat}(v_{R,L}^{cmd})-v_{R,L}\big),\qquad v_{R,L}^{cmd}=v\pm\tfrac{\omega W}{2}$$

---

## 3. Estimators compared (sensor ablation)

| Name | Uses | What it represents |
|---|---|---|
| **ZOH** | Fix only, holds last command | v1 baseline, a robot with no onboard sensing |
| **EKF-enc** | Fix + encoders (heading from the wheel difference) | Cheap robot with no IMU |
| **EKF-full** | Fix + encoders + gyro, bias as a state | Typical AMR |
| **EKF + covariance governor** | Slows down if the EKF's lateral 3σ is large near a curvature change | "Trust the filter" mitigation |
| **EKF + consistency monitor** (contribution) | Slows down if gyro and encoder yaw rates disagree during a blackout | "Cross-check two sensors" mitigation |

**EKF** (state $\mathbf{x}=[x,y,\theta,b_g]^T$, inputs: encoder speed $v_m$ and gyro rate $\omega_g$):

$$\mathbf{x}_{k+1}=\begin{bmatrix}x+v_m\cos\theta\,\Delta t\\ y+v_m\sin\theta\,\Delta t\\ \theta+(\omega_g-b_g)\Delta t\\ b_g\end{bmatrix},\quad
F=\begin{bmatrix}1&0&-v_m\sin\theta\Delta t&0\\0&1&v_m\cos\theta\Delta t&0\\0&0&1&-\Delta t\\0&0&0&1\end{bmatrix}$$

$P\leftarrow FPF^T+GQG^T$. When a fix arrives, update with $H=[I_3\ \ 0]$ (wrap the heading innovation). During a blackout only the predict step runs, so P grows.

---

## 4. The maths: derive these, then check them in simulation

**M1. Pure Pursuit is exact on a steady arc.** With the target taken a fixed arc length ahead of the robot's projection onto the path, the steady-state condition reduces to $r^2=R^2$. So in steady state the robot sits exactly on the arc, and holding the last command mid-turn is free. This explains the v1 mid-turn result.

**M2. Blackout error envelopes** (first-order, small-angle; *t* = time since the blackout started):

| Error source | Predicted cross-track error | Phase dependence |
|---|---|---|
| No fusion, command held across a curvature jump Δκ | $e \approx \Delta\kappa\,(vT)^2/2$ (upper bound; prototype measured ≈ ⅓ of this because of lookahead preview) | Only at entry/exit |
| Residual gyro bias $b$ | $e \approx \tfrac12 v\,b\,t^2$ | Everywhere |
| Gyro white noise (angle random walk $N$) | $\sigma_e \approx v N t^{3/2}/\sqrt3$ | Everywhere |
| Left/right wheel mismatch $\delta=\varepsilon_R-\varepsilon_L$ (no gyro) | $e \approx v^2\delta\,t^2/(2W)$ | Everywhere |
| Common wheel-size error $\varepsilon$ (with gyro) | Robot drives a circle of radius $R/(1+\varepsilon)$: $e \approx \varepsilon R\,(1-\cos\varphi_b)$, where $\varphi_b$ = arc angle turned during the blackout | **Zero on straights**, grows in arcs |

**M3. Survival time** (time until the corridor $e_{tol}$ is breached), from the bias row:

$$T^*=\sqrt{\frac{2e_{tol}}{v\,b}}$$

Example: $e_{tol}$ = 3 cm, b = 1°/s, v = 0.5 m/s → **T\* ≈ 2.6 s**. The prototype crossed 3 cm between 2 and 3 s. Inverting it gives a speed limit for a given expected blackout length, which is the governor's design rule.

**M4. Detector design.** The residual is $r=(\omega_g-\hat b_g)-\frac{\omega_R^{enc}-\omega_L^{enc}}{W}$, low-pass filtered with time constant τ_r. Set the threshold at $h=4\sigma_r$ for a known false-alarm rate. For a bias step $b>h$, the detection delay is

$$t_d=-\tau_r\ln\!\left(1-\tfrac{h}{b}\right)$$

Example: τ_r = 0.5 s, h = 0.6°/s, b = 1°/s → t_d ≈ 0.46 s. **Bonus insight:** a common wheel-size error shows up in r as −εω, which is nonzero *only while turning*. So the monitor sees that fault exactly when it matters. Extension: replace the threshold with a CUSUM test (Page, 1954) and compare the delay against the false-alarm trade-off.

**M5. Link to Kalman-with-packet-loss theory.** Sinopoli et al. (2004) show that the expected EKF covariance stays bounded only above a critical arrival rate λ_c, with lower bound $1-1/\max|\lambda(A)|^2$. Dead reckoning is marginally stable (eigenvalues on the unit circle), so that lower bound is **zero**. For this system, "does the filter diverge?" is the wrong question. The right one is "how long until it leaves the corridor?" (M3). That is why the paper measures peak error and survival time, not covariance stability.

---

## 5. Experiment design

**Core grid (simulation, R = 1.0 m, v = 0.5 m/s):**

4 blackout lengths {0.5, 1, 2, 4 s} × 4 placements {straight, entry, mid, exit} × 5 sensor conditions {none, b_res 0.5°/s, b_res 1°/s, wheels +3% common, ±1% differential} × 5 estimators × **10 noise seeds** = **4,000 runs**. That's minutes in Python.

**Sweeps** (2 s blackout only): radius {0.55, 1.0, 1.5} and speed {0.3, 0.5, 0.7}. These test the R and v scaling that M2 predicts.

**v1 survives as one experiment.** A Gilbert–Elliott channel with a **matched 10% average loss** and mean burst length {1, 4, 8, 16} reports, ZOH vs EKF-full, with Monte Carlo seeds. This keeps v1's "same loss rate, different burstiness" idea, but stochastic and with confidence intervals.

**Fairness rules (write them in the paper):**
1. Compare each blackout run with the **same-seed no-blackout run** (excess error).
2. Let the gyro-bias estimate converge before any blackout: start blackouts ≥ 4 s into the run. In the prototype, a blackout at 1 s gave half the error because the bias estimate was still noisy.
3. Fix the corridor (±3 cm) and the detector threshold **before** looking at the results.
4. Report the median and IQR over seeds, not single runs.

---

## 6. Metrics

| Metric | Why |
|---|---|
| Peak excess cross-track error (cm) | Main outcome |
| Corridor violation rate (±3 cm) and survival time T\* | Safety framing, checks M3 |
| Recovery: overshoot and time back within 1 cm after the fix returns | The peak sometimes comes *after* the blackout |
| NEES during the blackout | Is the EKF honest about its own error? Expect honest for noise, overconfident for faults. That's why the covariance governor fails and the monitor is needed. |
| Detector delay and false alarms per hour (no-fault runs) | Cost of the monitor |
| Time lost vs. nominal run | Cost of slowing down |

**Main figure:** a heatmap of fault type × turn phase, coloured by median excess error, one panel per estimator.
**Explanation figure:** error vs. time for one blackout, with the M2 envelope overlaid and the monitor's alarm time marked.

---

## 7. Prototype evidence (already measured, Python, 3 seeds)

Rough simulation (`prototype/blackout_sim.py`) at R = 1.0 m, v = 0.5 m/s, with a **4 s blackout centred on the phase**. Values are the median excess peak error in cm:

| Sensor condition | Phase | EKF-enc | EKF-full | EKF + monitor | Monitor time cost |
|---|---|---:|---:|---:|---:|
| No fault | entry / mid / exit | 0.3 / 0.5 / 0.4 | 0.4 / 1.2 / 0.8 | 0.4 / 1.2 / 0.8 | **0 s (no false alarms)** |
| Gyro drift 1°/s (unlearned) | entry / mid / exit | 0.3 / 0.5 / 0.4\* | 6.0 / 7.1 / 8.2 | **1.8 / 2.3 / 3.8** | ~2.5 s |
| Wheels +3% common | entry / mid / exit | 2.8 / 1.4 / 0.8 | 3.0 / 2.1 / 0.9 | **1.2 / 1.2 / 0.7** | 0.9–2.8 s |

\* EKF-enc looks good here only because its encoders had no fault in these rows. With a ±1% left/right mismatch, EKF-enc reached **8–10 cm for 2 s and 17–23 cm for 4 s blackouts** (single seed).

Other prototype observations:
- **ZOH without fusion is catastrophic for long blackouts:** 32 cm for a 2 s mid-turn blackout, and 0.7–1.8 m for a 4 s blackout at entry, mid or exit (holding a command through a curve with motor lag and fix noise).
- **The covariance-only governor barely helped** (for example 9.2 → 9.2 cm at exit). The EKF doesn't know about the unmodelled fault, so its covariance stays small. That supports the NEES argument in §6.

These are **prototype numbers with assumed noise levels**, not results. They show that the full study will produce effects big enough to measure and explain.

---

## 8. Hardware tiers (optional, but this is what moves the rating to 7.5–8)

**Tier A: real sensor data (≈ ₹300, ~4 hours).** Put an MPU6050 or BMI160 on an Arduino. Log the gyro for 1–2 hours sitting still, and also over the first 10 minutes of warm-up. Compute the **Allan variance** to get the real angle random walk and bias instability, plus the real warm-up bias drift. Plug those values into the simulator in place of my guesses. Reviewers trust "noise parameters measured from our IMU" far more than "assumed".

**Tier B: real robot validation (≈ ₹4–6k, 25–40 hours).**
- ESP32, 2 × N20 gear motors with Hall encoders, TB6612FNG driver, MPU6050, and an ArUco marker on top.
- An overhead webcam or phone runs OpenCV ArUco detection and sends the pose over Wi-Fi UDP at 10 Hz. Tape the path on the floor.
- **The trick:** drop the packets *in software* on the same scripted schedule as the simulation, but **log every camera frame**. You get exact blackouts and free ground truth.
- Inject the gyro-bias fault in firmware by adding a constant.
- 20–40 runs are enough to validate 2–3 predictions: t² growth, wheel-size error appearing only in the arc, and the monitor's error reduction.

---

## 9. Related work to cite (all checked against OpenAlex)

| Paper | Use in your paper |
|---|---|
| Sinopoli et al., "Kalman Filtering With Intermittent Observations," *IEEE TAC* 2004, [doi:10.1109/tac.2004.834121](https://doi.org/10.1109/tac.2004.834121) | Theory anchor for M5 |
| Kluge, Reif & Brokate, "Stochastic Stability of the EKF With Intermittent Observations," *IEEE TAC* 2010, [doi:10.1109/tac.2009.2037467](https://doi.org/10.1109/tac.2009.2037467) | EKF version of the same |
| Huang & Dey, "Stability of Kalman filtering with Markovian packet losses," *Automatica* 2007, [doi:10.1016/j.automatica.2006.10.023](https://doi.org/10.1016/j.automatica.2006.10.023) | Bursty (Markov) loss theory |
| Gilbert, "Capacity of a Burst-Noise Channel," *Bell Syst. Tech. J.* 1960, [doi:10.1002/j.1538-7305.1960.tb03959.x](https://doi.org/10.1002/j.1538-7305.1960.tb03959.x) | Gilbert–Elliott channel model |
| Borenstein & Feng, "Measurement and correction of systematic odometry errors," *IEEE TRA* 1996, [doi:10.1109/70.544770](https://doi.org/10.1109/70.544770) | Wheel-size and wheel-mismatch error model |
| Borenstein & Feng, "Gyrodometry," *ICRA* 1996, [doi:10.1109/robot.1996.503813](https://doi.org/10.1109/robot.1996.503813) | **Closest prior work for the monitor.** It already switches between gyro and odometry when they disagree. Say clearly that your novelty is using that check *during localization blackouts to trigger a slowdown*, analysed by turn phase. |
| El-Sheimy, Hou & Niu, "Analysis and Modeling of Inertial Sensors Using Allan Variance," *IEEE TIM* 2008, [doi:10.1109/tim.2007.908635](https://doi.org/10.1109/tim.2007.908635) | Tier A method |
| Liu, Candell & Moayeri, *ISA Trans.* 2017, [doi:10.1016/j.isatra.2017.02.005](https://doi.org/10.1016/j.isatra.2017.02.005) | Bursty vs. random loss at a matched rate (industrial control) |
| Carbonell et al., *ISA Trans.* 2024, [doi:10.1016/j.isatra.2024.05.041](https://doi.org/10.1016/j.isatra.2024.05.041) | Networked path following with packet drops |
| Ferrari et al., *Sensors* 2026, [doi:10.3390/s26030881](https://doi.org/10.3390/s26030881) | Tracking accuracy under Wi-Fi timing |
| Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm," CMU-RI-TR-92-01, 1992 | Controller (from v1; not re-checked) |
| Page, "Continuous inspection schemes," *Biometrika* 1954 | CUSUM extension (classic; not checked with the tool) |

**Still to do before writing:** search for "localization outage" + "speed reduction / safe speed" and "IMU–odometry fault detection" for AMRs. Uncertainty-aware speed limiting exists, so you need to know the closest one and say how yours differs.

---

## 10. Work plan (3 people)

| Week | Work | Team-hours |
|---|---|---:|
| 1 | Simulator: kinematics, actuators, sensors, EKF; reproduce the no-loss baseline | 15–20 |
| 2 | Blackout schedules, Gilbert–Elliott channel, monitor, batch runner, CSV | 15–20 |
| 2–3 | Derive M1–M5, overlay the envelopes on the simulation, NEES | 10–15 |
| 3 | Core grid + sweeps, figures | 10–15 |
| 3–4 | Paper (6 pages, IEEE format), viva demo | 15–20 |
| (opt.) | Tier A Allan variance / Tier B robot | +4 / +25–40 |
| | **Total sim-only** | **~65–90** |

**Split:** teammate 1 builds the simulator and EKF. Teammate 2 builds the channel, monitor, batch runner and plots (plus the hardware if you do Tier B). You do the maths derivations, related work and the paper.

**Tools:** Python (NumPy and Matplotlib) for the experiments, because matrix maths is easier than in JavaScript. Keep the p5.js replay only as the viva demo, or skip it.

**Viva moment:** the same 4 s blackout, the same gyro drift, two runs. Without the monitor the robot drifts about 6 cm outside the corridor. With it, the alarm fires about 0.5 s in, the robot slows, and it stays inside. Show the error-vs-time plot with the t² envelope drawn over it.

---

## 11. Honest rating

| Version | Workshop score | What it needs |
|---|---|---|
| v1 (Codex) | ~4 | n/a |
| v2 simulation only: grid + maths + monitor | **~7** | Clean M2 envelope validation and the Gyrodometry comparison stated honestly |
| v2 + Tier A (real IMU noise via Allan variance) | **~7.5** | 4 extra hours, ₹300 |
| v2 + Tier B (real robot, 20–40 runs) | **~8** | Validates 2–3 predictions on hardware |

**Why not higher:**
- The monitor idea is close to Gyrodometry (1996). The novelty is the blackout × turn-phase analysis and the closed-form envelopes, not the cross-check itself.
- Simulation-only robotics papers get discounted.

**Why not lower:** this version has a falsifiable model (M2–M4), an ablation, a mitigation with a measured cost, and a reason why the obvious mitigation (trusting the covariance) fails.

These scores assume good execution and writing. They're a reviewer's estimate, not a promise of acceptance.

## 12. Threats to validity (put these in the paper)

- Noise values are assumed until Tier A replaces them.
- The motor model is first order and has no wheel slip. Slip on turns would add a phase-dependent error of its own (a good future-work line).
- One path shape and one controller. Mention Stanley or MPC as future work.
- The fix has no latency in the core grid. Latency is the first extension.
