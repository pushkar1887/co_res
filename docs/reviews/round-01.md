# Review round 01

| | |
|---|---|
| Plan reviewed | rev 1 (commit `270508a`) |
| Codex review | commit `fedb7aa` (PR #1) |
| Claude reply | commit `474dd92` |
| Outcome | 20 suggestions integrated into rev 2; 1 citation held; plan restructured as rev 3 in `docs/PLAN.md` |

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
