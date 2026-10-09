"""Per-plane default gains: ``data/flightbench/<plane>.json``, seeds as fallback.

The tuner (``python -m flightbench.tune``) writes this file per plane; a missing
file or a missing task row means the seeds in ``tasks.base``. Only the ``linear``
law lives here: ``lqr`` and ``ndi`` defaults come from code.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from flightbench.common import FlightbenchError
from flightbench.tasks.base import get_task

# <repo>/data/flightbench: this file is <repo>/python/flightbench/tasks/defaults.py.
DEFAULTS_DIR = Path(__file__).resolve().parents[3] / "data" / "flightbench"

# The law whose tuned gains the file carries.
LAW = "linear"


def defaults_path(plane: str) -> Path:
    """Where a plane's defaults document lives."""
    return DEFAULTS_DIR / f"{plane}.json"


def load_defaults(plane: str) -> dict:
    """A plane's defaults document; a missing file is an empty mapping."""
    path = defaults_path(plane)
    if not path.is_file():
        return {}
    try:
        document = json.loads(path.read_text())
    except ValueError as error:
        raise FlightbenchError(
            f"defaults for plane {plane!r} are not valid JSON: {path} ({error})"
        ) from None
    if not isinstance(document, dict):
        raise FlightbenchError(
            f"defaults for plane {plane!r} are not a JSON object: {path}"
        )
    return document


def default_gains(plane: str, task: str) -> dict[str, float]:
    """The task's gains for a plane: the file's tuned value, else the seed."""
    info = get_task(task)
    row = _row(load_defaults(plane), plane, task)
    source = f"plane {plane!r} defaults for task {task!r}"
    return {
        gain.name: _gain_value(
            gain.name, row.get(gain.name, gain.seed), source
        )
        for gain in info.gains
    }


def write_defaults(
    plane: str,
    aero: str,
    trim: tuple[float, float],
    table: dict[str, dict[str, float]],
    generated_by: str,
) -> Path:
    """Write a plane's tuned document in the spec's shape and return its path."""
    vt_mps, altitude_m = trim
    document = {
        "plane": plane,
        "aero": aero,
        "trim": {
            "vt_mps": float(vt_mps),
            "altitude_m": float(altitude_m),
        },
        "generated_by": generated_by,
        LAW: _checked_table(table, plane),
    }
    DEFAULTS_DIR.mkdir(parents=True, exist_ok=True)
    path = defaults_path(plane)
    path.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")
    return path


def _row(document: dict, plane: str, task: str) -> dict:
    """The ``{"<gain>": <value>}`` row of one task, empty when the file has none."""
    law = document.get(LAW, {})
    if not isinstance(law, dict):
        raise FlightbenchError(
            f"plane {plane!r} defaults: {LAW!r} is not a JSON object"
        )
    row = law.get(task, {})
    if not isinstance(row, dict):
        raise FlightbenchError(
            f"plane {plane!r} defaults: task {task!r} row is not a JSON object"
        )
    return row


def _checked_table(
    table: dict[str, dict[str, float]], plane: str
) -> dict[str, dict[str, float]]:
    """Validate a tuned table against the registry: known tasks, known gains."""
    checked: dict[str, dict[str, float]] = {}
    for task, row in table.items():
        info = get_task(task)
        legal = tuple(gain.name for gain in info.gains)
        source = f"plane {plane!r} defaults for task {task!r}"
        checked[task] = {}
        for name, value in row.items():
            if name not in legal:
                raise FlightbenchError(
                    f"unknown gain {name!r} for task {task!r}; legal gains: {legal}"
                )
            checked[task][name] = _gain_value(name, value, source)
    return checked


def _gain_value(name: str, value: object, source: str) -> float:
    """A gain value as a finite float; anything else is a bad request."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FlightbenchError(
            f"gain {name!r} in {source} must be a number, got {value!r}"
        )
    number = float(value)
    if not math.isfinite(number):
        raise FlightbenchError(
            f"gain {name!r} in {source} must be finite, got {value!r}"
        )
    return number


__all__ = ["DEFAULTS_DIR", "default_gains", "defaults_path", "load_defaults", "write_defaults"]
