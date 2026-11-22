# AGENTS.md: rules for Codex and Claude on this repo

Both agents work on the same GitHub repo (`pushkar1887/co_res`) from different machines. Codex runs in the cloud and can't see Claude's local disk, so **the repo is the only shared state**.

## 1. Sync (every session, no exceptions)

1. **Before any work:** `git pull --ff-only`. Then run `git log --oneline -10` and read every commit the other agent made since your last one.
2. **After every change:** commit and push at once. Never leave work only on your local disk.
3. Never force-push, never rewrite another agent's commits, never delete their files without a plan reference.
4. If a push is rejected: `git pull --rebase`, resolve, push. If a conflict touches the other agent's work, keep both versions and flag it in your next review.

## 2. Plan protocol

- `docs/PLAN.md` is the single design document. **Claude edits sections 1–18.**
- **Codex writes only in §19 "Codex Suggest (round N)"**: append-only, marked "Codex".
- Claude answers each round in `docs/reviews/round-NN.md`. For each suggestion it records **agree** (integrated; says where) or **disagree** (with a reason). It copies the Codex section there verbatim, integrates the agreed changes into §1–18, resets §19 for the next round, and logs the revision in §17.
- Once a design decision is frozen (§13 M0), changing it requires a new review round and a change-log entry.

## 3. File ownership (until M2; see PLAN §12–13)

| Path | Owner | Other agent |
|---|---|---|
| `docs/PLAN.md` §1–18 | Claude | suggests in §19 |
| `docs/PLAN.md` §19 | Codex | reads |
| `sim/`, `web/`, `tools/`, `firmware/` | Claude | reviews; proposes changes in §19 or a PR |
| `analysis/reference_check.py` | **Codex** (independent re-implementation, V9) | must not read `sim/` internals beyond the exported trace format |
| `analysis/` (everything else) | Claude | reviews |
| `results/`, `figures/` | generated only; nobody hand-edits | — |
| `docs/paper/`, `docs/viva/` | Pushkar + both | coordinate per section |

## 4. Engineering conventions

- **One model.** The simulator exists only in `sim/` (ES modules, no DOM). Python in `analysis/` reads `results/` and never re-simulates. The one exception is `reference_check.py`, which deliberately re-implements the estimator for cross-checking.
- **Parameters** change only in `sim/params.js`, with a comment citing the PLAN section that justifies them.
- **Randomness** comes only from the seeded streams in `sim/util.js`. The same seed means the same noise for every arm (common random numbers).
- **Units:** SI in code, radians internally, degrees only in labels and docs.
- **Results** go in `results/<experiment>/` with `runs.csv` and `meta.json` (git commit, date, parameter hash, experiment id). Regenerate; never edit.
- **Names:** snake_case files; figures `fig_<topic>.pdf/.png`; experiment ids as in PLAN §7.
- **Line endings:** LF (enforced by `.gitattributes`).

## 5. Data policy

All data is simulated. Never describe anything as measured. Sensor parameters are "assumed MEMS-class".

## 6. Commits

- Message format: `<area>: <what changed>`, where area is one of `plan`, `review`, `sim`, `web`, `analysis`, `results`, `firmware`, `docs`, `repo`.
- Reference the PLAN section when a change implements or alters the design.

## 7. Environment facts

- Claude's machine (Windows): Python 3 with numpy, scipy and matplotlib. **No Node.js, no C++ compiler.** Batch runs in the browser via `tools/serve.py`; firmware compiles in Wokwi.
- Codex (cloud, Linux): may have Node. `tools/run_experiment.mjs` (planned) lets it run the same grids headlessly.
