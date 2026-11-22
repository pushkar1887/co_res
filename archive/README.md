# archive/

These are superseded throwaway prototypes, kept only because `docs/PLAN.md` §11 cites their numbers. Nothing in the project imports them, and their numbers are replaced once `sim/` runs.

| File | What it was | Cited result |
|---|---|---|
| `prototypes/proto1_hold_short_gaps.py` | v1: hold-last-command only, no sensor noise, 0.2 m/s | ≤ 1 cm peak error; mid-turn ≈ 0 |
| `prototypes/proto2_ekf_faults_monitor.py` | Claude v2: encoders + gyro + EKF + monitor, 0.5 m/s | Library for the test below |
| `prototypes/proto2_monitor_test.py` | Runs proto2: 4 s blackout, faults, 3 seeds | Gyro 1°/s: EKF 60–82 mm vs. monitor 18–38 mm |
| `prototypes/proto3_codex_setup_check.py` | Codex v2 settings: HOLD / DR / EKF, burst of 8, slip, 10 seeds | DR ≈ EKF in tracking; EKF wins on estimate error and steering jump |

Run from inside `archive/prototypes/`, for example `python proto2_monitor_test.py`.
