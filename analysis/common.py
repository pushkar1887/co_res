"""Shared loaders and the pre-registered statistics of PLAN §8.

Unit of analysis = seed. Every test is on paired differences (same seed, same cell, different arm).
"""
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats

ROOT = Path(__file__).resolve().parent.parent
RESULTS = ROOT / "results"
FIGURES = ROOT / "figures"
RNG_SEED = 20261123          # fixed so bootstrap / permutation numbers are reproducible


def load(folder):
    return pd.read_csv(RESULTS / folder / "runs.csv")


def paired(df, cell_cols, arm_a, arm_b, metric):
    """Per-seed difference metric(arm_a) - metric(arm_b) within each cell. Returns a DataFrame."""
    a = df[df.arm == arm_a].set_index(cell_cols + ["seed"])[metric]
    b = df[df.arm == arm_b].set_index(cell_cols + ["seed"])[metric]
    d = (a - b).dropna().rename("diff").reset_index()
    return d


def bootstrap_ci(x, conf=0.95, n=10_000, stat=np.median, rng=None):
    rng = rng or np.random.default_rng(RNG_SEED)
    x = np.asarray(x, float)
    idx = rng.integers(0, len(x), size=(n, len(x)))
    boots = stat(x[idx], axis=1)
    lo, hi = np.percentile(boots, [(1 - conf) / 2 * 100, (1 + conf) / 2 * 100])
    return float(stat(x)), float(lo), float(hi)


def signflip_p(x, n=100_000, rng=None):
    """Two-sided paired sign-flip permutation test on the median (exact if 2^n_seeds <= n)."""
    rng = rng or np.random.default_rng(RNG_SEED)
    x = np.asarray(x, float)
    obs = abs(np.median(x))
    if 2 ** len(x) <= n:
        signs = ((np.arange(2 ** len(x))[:, None] >> np.arange(len(x))) & 1) * 2 - 1
    else:
        signs = rng.choice([-1.0, 1.0], size=(n, len(x)))
    null = np.abs(np.median(signs * x, axis=1))
    return float((np.sum(null >= obs - 1e-15) + 1) / (len(null) + 1))


def tost(x, delta):
    """Equivalence: 90% CI of the paired median inside [-delta, +delta]."""
    m, lo, hi = bootstrap_ci(x, conf=0.90)
    return {"median": m, "ci90_lo": lo, "ci90_hi": hi, "delta": delta, "equivalent": bool(lo > -delta and hi < delta)}


def holm(pvals):
    order = np.argsort(pvals)
    adj = np.empty(len(pvals))
    running = 0.0
    for rank, i in enumerate(order):
        running = max(running, min(1.0, (len(pvals) - rank) * pvals[i]))
        adj[i] = running
    return adj


def clopper_pearson_upper(k, n, conf=0.95):
    """One-sided upper bound on a binomial rate."""
    return 1.0 if k >= n else float(stats.beta.ppf(conf, k + 1, n - k))
