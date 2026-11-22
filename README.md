# Consistency-gated slowdown during camera blackouts

A simulated differential-drive warehouse robot follows a 90° turn with Pure Pursuit. It uses an overhead camera (10 Hz), quantized wheel encoders and a gyro (50 Hz), fused in an EKF. When camera reports drop out, a gyro–encoder consistency monitor decides whether to slow down. The study compares slowing **only when the sensors disagree** against never slowing, slowing on every blackout, and an oracle, with closed-form predictions and an ESP32 (Wokwi) port. **All data is simulated.**

| | |
|---|---|
| Plan | [`docs/PLAN.md`](docs/PLAN.md), rev 3 |
| Status | Design waiting for Codex review round 2. No experiment run yet; `sim/sim.js` is an untested draft |
| Reviews | [`docs/reviews/`](docs/reviews/) |
| Rules for Codex + Claude | [`AGENTS.md`](AGENTS.md) |

## Layout

```
docs/       plan, review log, (later) paper and viva
sim/        the one simulation model (JavaScript ES modules)
web/        browser demo (p5.js) and one-click batch runner
tools/      local server that saves results; Node CLI runner
analysis/   Python statistics, figures and tables (reads results/ only)
results/    generated data, one folder per experiment
figures/    generated figures
firmware/   ESP32 Wokwi demo
archive/    superseded prototypes
```

The full annotated tree is in PLAN §12.

## How to run (after milestone M1)

```
python tools/serve.py        # then open http://localhost:8000/web/
python analysis/make_all.py  # figures and tables from results/
```
