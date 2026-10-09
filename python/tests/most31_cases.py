"""Shared MOST31 angle grids and coefficient comparisons.

Not a test module. Translator tests import ``ALPHA_DEG``, ``BETA_DEG``, and
``assert_close``.
"""
from __future__ import annotations

import numpy as np

ALPHA_DEG = (-30, -12, -10, -7.5, 0, 3, 22.5, 45, 47, 60, 85)
BETA_DEG = (-35, -30, -7, 0, 4, 30, 35)


def assert_close(tc, got, ref, rel, msg=""):
    """Fail ``tc`` unless ``|got - ref| <= rel * max(1, |ref|)`` elementwise."""
    got_a = np.asarray(got, dtype=np.float64)
    ref_a = np.asarray(ref, dtype=np.float64)
    if got_a.shape != ref_a.shape:
        prefix = f"{msg}: " if msg else ""
        tc.fail(f"{prefix}shape got {got_a.shape} ref {ref_a.shape}")
    error = np.abs(got_a - ref_a)
    limit = rel * np.maximum(1.0, np.abs(ref_a))
    if not np.all(np.isfinite(got_a)) or np.any(error > limit):
        prefix = f"{msg}: " if msg else ""
        tc.fail(
            f"{prefix}got {got_a.tolist()} ref {ref_a.tolist()} "
            f"|err| {error.tolist()} tol {limit.tolist()}"
        )
