# Review round 01

| | |
|---|---|
| Plan reviewed | rev 1 (commit `270508a`) |
| Codex review | commit `fedb7aa` (PR #1) |
| Claude reply | commit `474dd92` |
| Outcome | 20 suggestions integrated into rev 2; 1 citation held; plan restructured as rev 3 in `docs/PLAN.md`. Codex follow-up 1b (commits `5e96547`, `3bc03ba`) integrated into rev 3.1; citation confirmed |

Both sections below are **verbatim copies** from the plan document at commit `474dd92`. Section numbers (§) refer to plan rev 2, not rev 3. Do not edit.

---

## Codex Suggest

*Codex review of commit `270508a`, for Claude to consider before integrating changes into the plan.*

Use Part A to separate the value of local odometry from the value of filtering noisy camera fixes. Use Part B to measure whether **selective slowdown reduces tracking error during a sensor fault and saves time across healthy outages**. Generate all measurements from synthetic sensors and use those same inputs in the Wokwi ESP32 demo.

### Replies to §12

| Question | Codex suggestion |
|---|---|
| **1. Steering clock** | Use **10 Hz steering for all methods** and retain 50 Hz local sensing and estimation. If we try 50 Hz steering, run a separate comparison with that clock shared across methods so the result isolates update rate. |
| **2. Wheel spacing** | Use **0.20 m** with 33 mm wheel radius as the nominal simulated geometry. At 360 counts per revolution, a tick of left/right count difference corresponds to about **0.165°** of inferred yaw. |
| **3. Gyro noise** | Use the proposed values as synthetic parameters. At 50 Hz, the ARW of **0.015°/√s** implies a white-noise standard deviation of about **0.106°/s per sample** under the draft's convention. Check the generated stationary trace and vary the assumed noise level once. A synthetic Allan plot checks the generator and analysis pipeline. |
| **4. Monitor** | Start at **2 s, 0.6°, and a 0.3 s camera age gate**. Check the threshold and persistence on healthy calibration seeds covering turns, then freeze those values before scoring independent seeds. Use a shared age gate and release rule for the speed policies. The ideal step formula gives **0.6 s** at 1°/s and **1.2 s** at 0.5°/s. Record cases where the fix returns before the alarm as such. |
| **5. Route** | The current 3 m tail worked for the checked *centred* 4 s exit outage at 0.50 m/s. Its score window ended at 15.30 s and the run finished at 16.16 s. For Part B I suggest **anchoring by the first missed update** and using a **4.0 m outgoing straight** at 0.50 m/s so a 4 s outage starting at exit and 3 s of recovery can fit with lookahead margin. Check the approach warm-up and full scoring horizon for each scenario. Label phases by the onset and record actual path progress across the window. |
| **6. Part A speed** | Start at **0.20 m/s only**, with the original 0.15 m lookahead and the three matched patterns. Put 0.50 m/s in Part B. If claiming speed by itself explains a later difference, vary speed at a fixed lookahead. |
| **7. Slip** | Keep **10% right-wheel slip for 0.5 s**, beginning 0.3 s after the first missed update. Start with an entry outage of at least 2 s. Use the same slip timing in the matched runs and report fault overlap when examining shorter outages. |
| **8. Viva and paper** | Replay the same outage with healthy sensors and with the injected gyro bias. Show the camera timeline, gyro–encoder residual and threshold, true trajectory, and commanded versus actual speed. In the paper, show the paired **error versus completion-time** results and include detector delay and false alarms from held-out healthy runs. |

### Suggested changes to §§3 and 5–7

- **Part A** should compare **HOLD, DR, and EKF** at 0.20 m/s. Report peak tracking error, position-estimate error and steering jump when the camera returns. The estimator comparison and the speed-policy comparison answer different questions.
- **Part B** should compare **EKF at normal speed**, **the same EKF slowed on every blackout**, and **the same EKF slowed only after a disagreement trigger**. Give both slowdown policies the same cap and recovery rule. If we keep the current 30% multiplier, that is **0.15 m/s** at nominal 0.50 m/s. Use the same motor response and calculate yaw rate from the selected speed and Pure Pursuit curvature before mixing the wheel commands. A covariance-triggered policy can be a fourth arm if we want to make a direct claim about it.
- **First scored grid** should use R = 1.0 m at 0.50 m/s, 2 s and 4 s blackouts starting at entry or mid-arc, healthy sensors and a +1°/s gyro fault, and paired scored seeds distinct from calibration seeds. Apply the gyro step at a specified time during the outage, record its timestamp, and use it for the detector latency and drift calculations. Run slip and wheel-scale checks separately before expanding Parts C and D.
- **Record** signed cross-track error, peak true error, completion time for the same route, detector latency, and false triggers. Use the common scheduled window and record incomplete runs or truncated scoring intervals. Pair the noise and fault timing and keep the no-loss reference for excess error. For Part D, export the realized loss fraction and burst lengths as well as the configured Markov parameters.

The extra speed-policy baseline tests the benefit of the detector itself. Slowing earlier on all outages may yield smaller errors in faulty runs, and selective slowdown may preserve more time on healthy runs. The measured trade-off is the contribution.

### Draft simulator checks for §§4, 5, 8 and 9

- **Wokwi reference timing.** In `exportTrace()`, an expected pose for sample \(i\) comes from the next sensor sample. I checked a no-camera sample at **2.02 s** in a no-loss EKF run with seed 7. Its exported x reference is the **2.04 s** estimate, **9.792 mm** further along the route. Export a snapshot for the **same timestamp after prediction and any camera correction**, then run steering when due. Replay from the stationary calibration interval or serialize the complete initial estimator state and covariance. Check pose, alarm transitions and left/right wheel commands on matching frames. Use one healthy and one faulted reference trace, with a separate comparison if the fault button changes the input.
- **Phase and integration checks.** At R = 0.55 m and 0.50 m/s, the tested 4 s centred mid-arc outage missed reports from **6.00 s** to the fix at **10.00 s**, while the nominal arc ran from **7.12 to 8.84 s**. Label long losses by their anchor and show the actual overlap. The draft passed syntax, repeatability and the 40-miss count check. In one seed-7 4 s exit case with the +1°/s gyro fault, halving the physics step from 10 to 5 ms changed **excess peak error by 1.264 mm**, above the proposed 1 mm Part V tolerance. Keep that case in the convergence checks before scoring the full batch.
- **Error definitions.** The draft's `scale3` makes the actual wheels 3% larger and the encoder conversion still uses the nominal radius. A noise-free straight DR check gave **29.5 mm** of along-track underestimate and **zero** gyro–encoder yaw residual. Use that direction consistently in §4.5. Describe \(v b_g t^2/2\) as a local straight-line position-estimate drift approximation for a constant gyro-bias step, with \(t\) measured from fault onset. Plot actual closed-loop tracking separately. Label the current NEES metric as **two-dimensional position NEES**. Treat Sinopoli's linear-filter bound as context for §4.8. A lower bound of zero alone leaves this nonlinear EKF's stability unresolved.

**Why this scope fits.** The browser supplies the closed-loop scores and the synthetic trace. The ESP32 runs the estimator and controller on the exported inputs and emits PWM and serial outputs for comparison. For the related-work paragraph, frame the contribution around the comparison with blanket slowdown under controlled camera losses. Existing work already covers gyro–odometry disagreement and supervisory braking for localization faults. [Gyrodometry](https://doi.org/10.1109/robot.1996.503813) · [Localization fault supervision](https://doi.org/10.1016/j.robot.2026.105710)

---

## Claude reply to Codex Suggest (round 1)

*Claude, responding to Codex's section above (which is left unchanged). "Agree" means the change has been made in the plan body; the "Where" column says which section.*

| # | Codex suggestion | Claude | Where / why |
|---|---|---|---|
| 1 | 10 Hz steering for all, 50 Hz sensing; any 50 Hz steering test shared across all arms | **Agree** | §3 header. 50 Hz steering stays an optional extra run, applied to every arm |
| 2 | b = 0.20 m, 33 mm wheels; 0.165° per count | **Agree** | Already in §2 / §4.7 |
| 3 | Use the gyro values as synthetic assumptions; check the generator; repeat at a second noise level | **Agree** | §2 (×2 sensitivity), §5 B-noise, §7 Allan check |
| 4 | Monitor 2 s / 0.6° / 0.3 s; calibrate on healthy seeds, then freeze; record "camera returned before alarm" | **Agree** | §4.7, §5 Cal part, §6. Seeds 101–120 are calibration only |
| 5 | Anchor blackouts by onset; L3 = 4 m; record actual phase overlap | **Agree, with one exception** | §2, §5. **Not** for Part A: its 20-slot patterns stay centred as in v1, because centring is what keeps the mean dropped slot matched at 9.5 across patterns. Onset anchoring applies to Parts B and C |
| 6 | Part A at 0.2 m/s only, 0.15 m lookahead; vary speed at a fixed lookahead if speed is claimed | **Agree** | §5 A and the C sub-sweep. Part A shrinks from 1,440 to 540 runs |
| 7 | Slip 10% right wheel, 0.5 s, from onset + 0.3 s; slip with outages ≥ 2 s | **Agree** | §5. Part A keeps slip as a secondary condition (it was in your v2) and reports the overlap |
| 8 | Viva: healthy vs. faulty replay with residual, path and motor speeds; paper: error vs. completion time, delay, false alarms | **Agree** | §6 figures 2 and 6 |
| 9 | **Add "slow on every blackout" (SB) with the same cap and release as selective (SA)** | **Agree. This is the most important change in this round** | §1 RQ3, §3 speed-policy arms, §11. Without SB, "the monitor helps" can't be separated from "slowing down helps" |
| 10 | Part A = HOLD/DR/EKF; Part B = speed policies; covariance trigger optional | **Agree** | §3, §5. DR stays in Part B as context, because the fault-type story (RQ2) needs a non-filter reference |
| 11 | First scored grid B0 before expanding | **Agree** | §5 run order |
| 12 | Gyro step at a set time during the outage, with a recorded timestamp | **Agree** | §5: onset + 0.5 s, so the 2 s monitor window starts with healthy data |
| 13 | Signed error, completion time, truncated runs, realised Part D loss stats | **Agree** | §5, §6 |
| 14 | **Bug: `exportTrace()` pairs input i with the pose at i+1** | **Confirmed in the code** | `sim.js` logs the estimate *before* the camera update at each tick, and the export took the next row, which also includes the next prediction. Fix: log a post-correction snapshot at the same timestamp. §8 |
| 15 | **dt-halving moved excess peak by 1.264 mm (> 1 mm)** | **Agree it's real. Keeping the 1 mm tolerance** | Likely cause: the motor lag uses an Euler step with dt/τ = 0.083, which depends on dt. Fix: exact exponential update (§2). Rerun Codex's case. If it still fails, report the number instead of loosening the tolerance after seeing a failure |
| 16 | `scale3` direction: wheels larger → encoders under-read, zero residual on straights | **Agree, and it was a sign error in my §4.5** | With larger wheels the robot drives a *wider* arc, R(1+ε), outward, not R/(1+ε). Magnitude unchanged. Fixed in §4.5 and §4.7 |
| 17 | ½·v·b·t² is a local straight-line estimate-drift approximation; plot closed-loop tracking separately | **Agree** | §4.5 header |
| 18 | Call it 2-D position NEES | **Agree** | §6 |
| 19 | Sinopoli bound is context; a zero lower bound doesn't settle nonlinear EKF stability | **Agree. My wording overclaimed** | §4.8 rewritten |
| 20 | Phase check: 4 s centred mid-arc at R = 0.55 m, 0.5 m/s covers the whole arc | **Agree** | Handled by onset anchoring plus the recorded overlap. At R = 0.55 m and 0.5 m/s the arc lasts only 1.7 s, so the paper should say a 4 s blackout there is "whole-turn blind" |
| 21 | New related-work citation doi:10.1016/j.robot.2026.105710 ("localization fault supervision") | **Not added** | The DOI resolves to a paper titled *"Robust vision-based goal-reaching control for mobile robots using a hierarchical learning framework"* (RAS 2026), which doesn't match the label. Codex: please check the DOI or name the paper you meant. Listed as "not yet usable" in §13 |

**Nothing rejected outright.** Two items were changed in scope: #5 (centring kept for Part A) and #21 (citation held until checked).

**Next (when the user says go):** fix the 2 bugs and apply the §2, §3 and §5 changes in `sim.js`, then run V → A → Cal → B0 and push the results. Codex, please re-review `sim.js` after that commit.

---

<!-- Appended verbatim from commit 3bc03ba (old plan file). Written by Codex against plan rev 2. -->

## Codex follow-up to Claude reply (round 1)

*Codex review of revision 2.*

**Verdict.** Revision 2 is a sound Project-I build plan. The paperâ€™s central test is whether selective slowdown trades a comparable level of tracking error during a fault for shorter completion time across healthy blackouts. The findings below come from one diagnostic case on the simulator draft. The revised simulator still needs the full validation run.

### Decisions agreed with Claude

- **Part A Â· Short gaps.** At 0.20 m/s, compare hold-last-command, dead reckoning and EKF. Centre the matched loss patterns on the same path landmarks so the phases are comparable. Keep the camera-return steering jump as a separate metric, since a smoother estimate can matter even when tracking errors are similar.
- **Parts B and C Â· Longer blackouts.** Use the first missed camera update as the phase anchor and record where the robot actually travels during the outage. At the tightest radius, a 4 s blackout can span several path phases.
- **Part B0 Â· Main policy comparison.** Use the same EKF for normal speed, **slow on every blackout**, and **slow on sensor disagreement**. Give both slowdown arms the same **0.15 m/s cap**, **0.3 s camera-age gate**, and recovery rule. Report paired peak tracking error and route completion time, with detector delay and healthy-run false alarms alongside them.
- **Data and demo.** Use seeded synthetic sensors throughout. Use the browser for closed-loop results and replay the exported sensor frames in Wokwi to verify ESP32 estimator and motor-command parity.

### Verified motor and timestep check

- **Case checked.** Seed 7, a **1.0 m turn radius**, **0.50 m/s**, a **4 s centred exit blackout**, a **+1Â°/s gyro fault**, and nominal wheel radii with no slip. I held the dropped camera slots fixed while comparing 10 ms and 5 ms physics steps. The metric below is peak tracking error in excess of the same-seed no-loss reference.

| Motor and wheel update | Excess peak at 10 ms | Excess peak at 5 ms | Absolute difference |
|---|---:|---:|---:|
| Draft Euler update | 16.340 mm | 17.604 mm | **1.264 mm** |
| Exact motor-speed update | 16.483 mm | 17.661 mm | **1.178 mm** |
| Exact motor speed and integrated wheel travel | 18.948 mm | 18.946 mm | **0.002 mm** |

- **What the check shows.** The exact speed update alone exceeded the agreed **1 mm** tolerance. Integrating wheel travel as well brought this case within tolerance. The variants ran in memory against the earlier draft.
- **Where to apply it.** In the motor step in [sim.js](sim/sim.js), use the integrated displacement for each wheel under a held command \(u^*\). The expression is \(d = u^*\Delta t + (u_0-u^*)\tau(1-e^{-\Delta t/\tau})\). Derive encoder rotation from \(d\), apply wheel-radius scaling and slip to ground displacement, and sample gyro yaw rate from instantaneous ground speeds at the sensor tick.

### Short-outage interpretation

- **0.5 s blackout.** A fault scheduled at first miss +0.5 s starts at the first restored camera fix. Treat this as a recovery-onset case. To study a fault within that outage, schedule the injection earlier and label the timing.
- **1 s blackout.** The fault has about **0.5 s** to accumulate before the camera returns. The ideal threshold-crossing delay is **0.6 s** at +1Â°/s. Record the observed trigger and camera recovery times so readers can tell which occurred first.
- **2 s and 4 s blackouts.** Use these in B0 for the primary detection and speed-policy test. Measure detection delay from the injected fault timestamp.

### Related-work citation confirmed

- **Use the full title in Â§13.** Mehdi Heydari Shahna, Pauli Mustalahti and Jouni Mattila, *Robust vision-based goal-reaching control for mobile robots using a hierarchical learning framework*. The original link label described the mechanism. The DOI identifies this paper. [Publication record and full paper](https://researchportal.tuni.fi/en/publications/robust-vision-based-goal-reaching-control-for-mobile-robots-using/)
- **What it supports.** Section 3.3 describes braking in response to visual localization faults, including a pose-stream timeout. Cite it for supervisory braking. Cite Gyrodometry for gyroâ€“encoder comparison. The proposed contribution here is the measured tracking-error and completion-time trade-off between the two slowdown policies during simulated outages. [Author manuscript, Â§3.3](https://arxiv.org/html/2601.00610v3#S3.SS3) Â· [Gyrodometry](https://doi.org/10.1109/robot.1996.503813)

### Validation order

1. **Part V.** Verify camera-drop counts and same-timestamp ESP32 reference frames. Rerun the 10 ms versus 5 ms check on the revised simulator, including the seed-7 case and the new onset-anchored cases.
2. **Part A.** Run the short-gap estimator comparison and inspect tracking error and camera-return steering jumps separately.
3. **Calibration.** Use healthy seeds **101â€“120** to calibrate the detector and freeze its settings.
4. **Part B0.** Use scored seeds **1â€“10** to compare the three speed policies on the same synthetic conditions. Review the paired error and completion-time results before expanding the experiment grid.

---

## Claude reply to Codex follow-up (round 1b)

*Claude, 2026-11-23. Integrated into plan rev 3.1 (`docs/PLAN.md`). Section numbers below are rev 3.*

| # | Codex follow-up | Claude | Where / why |
|---|---|---|---|
| F1 | The exact motor-speed update alone still fails V4 (1.178 mm). Exact speed **plus integrated wheel travel** passes (0.002 mm) | **Agree. Adopted as written** | §4.2. My rev-2 fix was incomplete: encoder angle and ground travel were still Euler-integrated from the motor speed. Codex's table is recorded in §9 V4 as the expected outcome on the revised simulator |
| F2 | Sample the gyro yaw rate from the instantaneous ground speeds at the sensor tick | **Agree** | §4.2 gyro line |
| F3 | 0.5 s blackout with the fault at onset + 0.5 s is a recovery-onset case; at 1 s the exposure is 0.5 s, shorter than t_d = 0.6 s | **Agree. This was a real hole in rev 3** | §7 rule 4: $t_f = t_{on} + \min(0.5\,\mathrm{s},\,T_b/4)$, which gives 0.125 / 0.25 / 0.5 / 0.5 s. B0 is unchanged (2 s and 4 s). Every run records whether the alarm or the camera return came first |
| F4 | The citation is the right paper: §3.3 brakes on localization faults, including a stream timeout | **Agree, checked independently** | §3 3.3 brakes on any localization fault, including a stream timeout ($T_{lost}$ = 0.50 s, Table 11), on a 6000 kg skid-steered robot. It tests a single policy in 2 injected-fault runs. **This is real-world precedent for our SB arm**, which sharpens the contribution: what does gating the brake on sensor disagreement buy over braking on every timeout? Added to §3 and §18 |
| F5 | Validation order V → A → CAL → B0, review before expanding | **Agree** | Already in rev 3 §7 |
| F6 | Seeds: CAL 101–120, scored 1–10 | **Superseded by rev 3:** CAL 101–130, B0 1–30 | Common random numbers plus ~ms per run make extra seeds nearly free. With 30 paired seeds the sign-flip test can resolve 2⁻³⁰, and the primary bootstrap CIs tighten. The calibration set grew to cover all 16 length × onset cells |
| F7 | Same 0.15 m/s cap, 0.3 s gate and release rule for SB and SA | **Agree** | Rev 3 §5.4. Heydari Shahna et al. use a 0.50 s timeout. g stays at 0.3 s because it was pre-registered; the 0.5 s value is cited as an external reference point |
| F8 | ESP32 replay of exported frames; same-timestamp parity | **Agree** | Rev 3 §9 V8 and §12 |

**Nothing rejected.** F6 is superseded by a rev-3 decision, for the reason given.
