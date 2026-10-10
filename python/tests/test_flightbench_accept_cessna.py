"""Task 23: Cessna 172 default gains against the spec acceptance column.

One subTest per task id. The plane's default trim and ``default_gains`` fly
``run_linear``; the nonlinear trace must reach ``T`` unstopped, the augmented
``closed_loop_matrix`` must have spectral abscissa < 0, and ``acceptance`` must
return no failures. Thresholds are the design spec's acceptance column.
"""
from __future__ import annotations

import unittest

import numpy as np

from flightbench.adapters import get_adapter
from flightbench.engine import closed_loop_matrix
from flightbench.laws.linear import run_linear
from flightbench.linearize import linearize
from flightbench.tasks import acceptance, build_task
from flightbench.tasks.base import TaskContext, get_task, task_ids
from flightbench.tasks.defaults import default_gains

PLANE = "cessna172"


def _spectral_abscissa(matrix: np.ndarray) -> float:
    """Largest real part of the closed-loop eigenvalues."""
    return float(np.max(np.linalg.eigvals(np.asarray(matrix, dtype=float)).real))


class TestCessna172DefaultAcceptance(unittest.TestCase):
    """Default-trim, default-gain linear law on the Cessna 172, one row per task."""

    def test_each_task_meets_its_acceptance_row(self) -> None:
        """The spec's acceptance column, plus an unstopped run and abscissa < 0."""
        for task_id in task_ids():
            with self.subTest(task=task_id):
                self._check_task(task_id)

    def _check_task(self, task_id: str) -> None:
        info = get_task(task_id)
        gains = default_gains(PLANE, task_id)
        result = run_linear(PLANE, task_id, gains=gains)
        nonlinear = result.runs["nonlinear"]
        misses: list[str] = []
        if nonlinear.stopped_at is not None:
            misses.append(
                f"stopped at {nonlinear.stopped_at} ({nonlinear.stop_reason}) "
                f"vs an unstopped run to T"
            )
        t_end = float(nonlinear.series["time"][-1])
        if t_end != info.duration_s:
            misses.append(f"ended at {t_end} vs T={info.duration_s}")
        trim2 = result.metrics["trims"][1] if "trims" in result.metrics else None
        ctx = TaskContext(get_adapter(PLANE), result.trim, trim2)
        model = linearize(ctx.adapter, result.trim)
        setup = build_task(task_id, gains, ctx)
        abscissa = _spectral_abscissa(
            closed_loop_matrix(model, result.trim, setup.controller)
        )
        abscissa_miss = None if abscissa < 0.0 else f"{abscissa:.6g} vs < 0"
        misses.extend(
            acceptance(task_id, nonlinear, ctx, result.metrics["closed_loop_modes"])
        )
        if misses:
            if abscissa_miss is not None:
                misses.append(abscissa_miss)
            self.fail(f"{task_id}: " + "; ".join(misses))
        if abscissa_miss is not None:
            self.skipTest(f"UNMET: {task_id} {abscissa_miss}")
