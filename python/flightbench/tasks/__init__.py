"""Build and check a task: the entry points the API and the CLI use.

``build_task`` turns (task id, gains, context) into the controller, disturbance
and reference the engine integrates; ``acceptance`` turns a finished run into the
list of acceptance-row failures. Both dispatch to the per-family module
(``tasks.longitudinal`` / ``tasks.lateral``) lazily:

    BUILDERS[task_id](gains, ctx) -> TaskSetup
    ACCEPT[task_id](trace, ctx, closed_loop_modes) -> list[str]

so importing the registry never pulls a plane or a loop in. The gains are
validated against ``TASKS[task_id].gains`` before that import: an unknown name, a
non-numeric value or a non-finite value is a bad request whatever the task.
Missing gains are not defaulted here — callers pass a complete dict, filled from
``tasks.defaults.default_gains`` when they want defaults.
"""

from __future__ import annotations

import importlib
import math

from flightbench.common import FlightbenchError, Mode, Trace
from flightbench.tasks.base import (
    GainSpec,
    TaskContext,
    TaskInfo,
    TaskSetup,
    get_task,
    task_ids,
)

# family -> module holding that family's BUILDERS / ACCEPT dicts.
FAMILY_MODULES: dict[str, str] = {
    "longitudinal": "flightbench.tasks.longitudinal",
    "lateral": "flightbench.tasks.lateral",
}

__all__ = [
    "FAMILY_MODULES",
    "GainSpec",
    "TaskContext",
    "TaskInfo",
    "TaskSetup",
    "acceptance",
    "build_task",
    "get_task",
    "task_ids",
]


def build_task(
    task_id: str, gains: dict[str, float], ctx: TaskContext
) -> TaskSetup:
    """Build one task's controller, disturbance and reference."""
    info = _with_checked_gains(task_id, gains)
    return _family(info).BUILDERS[info.id](gains, ctx)


def acceptance(
    task_id: str,
    trace: Trace,
    ctx: TaskContext,
    closed_loop_modes: list[Mode],
) -> list[str]:
    """The spec's acceptance row for a finished run: one string per failure."""
    info = get_task(task_id)
    return _family(info).ACCEPT[info.id](trace, ctx, closed_loop_modes)


def _family(info: TaskInfo):
    """The module holding one task's builder and acceptance row, imported now."""
    try:
        module_name = FAMILY_MODULES[info.family]
    except KeyError:
        raise FlightbenchError(
            f"task {info.id!r} has family {info.family!r}, which no module builds; "
            f"legal families: {tuple(FAMILY_MODULES)}"
        ) from None
    return importlib.import_module(module_name)


def _with_checked_gains(task_id: str, gains: dict[str, float]) -> TaskInfo:
    """The task's row, refusing unknown, non-numeric or non-finite gain values."""
    info = get_task(task_id)
    legal = tuple(spec.name for spec in info.gains)
    for name in gains:
        if name not in legal:
            raise FlightbenchError(
                f"unknown gain {name!r} for task {task_id!r}; legal gains: {legal}"
            )
    for name, value in gains.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise FlightbenchError(
                f"gain {name!r} for task {task_id!r} must be a number, got {value!r}"
            )
        if not math.isfinite(float(value)):
            raise FlightbenchError(
                f"gain {name!r} for task {task_id!r} must be finite, got {value!r}"
            )
    return info
