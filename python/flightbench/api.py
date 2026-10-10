"""Flightbench HTTP API.

Four routes over the plane registry, the task registry and ``laws.run``.
Values stay SI and radians. A plant stop is a 200 with ``stopped_at`` set.
``TrimError`` is 422; every other ``FlightbenchError`` is 400.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import uvicorn
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from flightbench.adapters import get_adapter, laws_for, plane_ids
from flightbench.common import (
    ALPHA,
    CHANNELS,
    SERIES,
    THETA,
    FlightbenchError,
    Mode,
    RunResult,
    Trace,
    TrimError,
    TrimPoint,
)
from flightbench.laws import gain_specs, run
from flightbench.tasks.base import TASKS

# Spec bounds for a requested trim. vt is exclusive at 0; both caps are inclusive.
_VT_MPS_MAX = 400.0
_ALTITUDE_M_MIN = 0.0
_ALTITUDE_M_MAX = 15000.0

_UI_DIST = Path(__file__).resolve().parents[2] / "web" / "flightbench" / "dist"


class TrimRequest(BaseModel):
    """Optional trim condition on a run request, SI."""

    vt_mps: float
    altitude_m: float


class RunRequest(BaseModel):
    """One run. Omitted gains keep the law's defaults for that task."""

    plane: str
    law: str
    task: str
    aero: str | None = None
    trim: TrimRequest | None = None
    gains: dict[str, float] | None = None


app = FastAPI(title="flightbench")


@app.exception_handler(TrimError)
def _trim_error(_request, exc: TrimError) -> JSONResponse:
    return JSONResponse(status_code=422, content={"detail": str(exc)})


@app.exception_handler(FlightbenchError)
def _flightbench_error(_request, exc: FlightbenchError) -> JSONResponse:
    return JSONResponse(status_code=400, content={"detail": str(exc)})


def _numbers(values) -> list[float]:
    """A series column as JSON numbers."""
    return np.asarray(values, dtype=float).reshape(-1).tolist()


def _trim(point: TrimPoint) -> dict:
    """A trim point in the spec's Trim shape."""
    state = np.asarray(point.x, dtype=float)
    controls = np.asarray(point.u, dtype=float)
    return {
        "vt_mps": float(point.vt_mps),
        "altitude_m": float(point.altitude_m),
        "alpha_rad": float(state[ALPHA]),
        "theta_rad": float(state[THETA]),
        "controls": {
            name: float(controls[index]) for index, name in enumerate(CHANNELS)
        },
    }


def _mode(mode: Mode) -> dict:
    return {
        "name": mode.name,
        "wn": float(mode.wn),
        "zeta": float(mode.zeta),
        "real": float(mode.real),
        "imag": float(mode.imag),
    }


def _series(trace: Trace) -> dict:
    return {key: _numbers(trace.series[key]) for key in SERIES}


def _reference(reference) -> dict | None:
    if reference is None:
        return None
    signal, time, values = reference
    return {"signal": signal, "time": _numbers(time), "values": _numbers(values)}


def _metrics(metrics: dict) -> dict:
    encoded = {}
    for key, value in metrics.items():
        if key in ("open_loop_modes", "closed_loop_modes"):
            encoded[key] = [_mode(mode) for mode in value]
        elif key == "trims":
            encoded[key] = [_trim(point) for point in value]
        else:
            encoded[key] = value
    return encoded


def to_json(result: RunResult) -> dict:
    """``RunResult`` as the spec's RunResponse. Numpy arrays become lists."""
    nonlinear = result.runs["nonlinear"]
    runs = {"nonlinear": _series(nonlinear)}
    for name, trace in result.runs.items():
        if name != "nonlinear":
            runs[name] = _series(trace)
    stopped_at = nonlinear.stopped_at
    return {
        "plane": result.plane,
        "law": result.law,
        "task": result.task,
        "aero": result.aero,
        "trim": _trim(result.trim),
        "runs": runs,
        "reference": _reference(result.reference),
        "metrics": _metrics(result.metrics),
        "stopped_at": None if stopped_at is None else float(stopped_at),
        "stop_reason": nonlinear.stop_reason,
    }


def _reject_nonfinite_gains(gains: dict[str, float] | None) -> None:
    """Non-finite gains are a bad request, refused before the plant runs."""
    for name, value in (gains or {}).items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise FlightbenchError(f"gain {name!r} must be finite, got {value!r}")


def _reject_trim(trim: TrimRequest | None) -> None:
    """Trim outside ``0 < vt_mps <= 400`` and ``0 <= altitude_m <= 15000``."""
    if trim is None:
        return
    if not math.isfinite(trim.vt_mps) or not (0.0 < trim.vt_mps <= _VT_MPS_MAX):
        raise FlightbenchError(
            f"trim vt_mps must satisfy 0 < vt_mps <= {_VT_MPS_MAX:g}, got {trim.vt_mps}"
        )
    if (
        not math.isfinite(trim.altitude_m)
        or not (_ALTITUDE_M_MIN <= trim.altitude_m <= _ALTITUDE_M_MAX)
    ):
        raise FlightbenchError(
            "trim altitude_m must satisfy "
            f"{_ALTITUDE_M_MIN:g} <= altitude_m <= {_ALTITUDE_M_MAX:g}, "
            f"got {trim.altitude_m}"
        )


@app.get("/api/planes")
def get_planes() -> list[dict]:
    planes = []
    for plane_id in plane_ids():
        adapter = get_adapter(plane_id)
        vt_mps, altitude_m = adapter.default_trim
        planes.append({
            "id": adapter.id,
            "label": adapter.label,
            "laws": list(laws_for(plane_id)),
            "aero_models": list(adapter.aero_models),
            "default_trim": {"vt_mps": float(vt_mps), "altitude_m": float(altitude_m)},
            "channel_labels": {
                name: adapter.channel_labels[name] for name in CHANNELS
            },
        })
    return planes


@app.get("/api/tasks")
def get_tasks() -> list[dict]:
    return [
        {
            "id": info.id,
            "label": info.label,
            "family": info.family,
            "lesson": info.lesson,
            "duration_s": float(info.duration_s),
            "description": info.description,
        }
        for info in TASKS.values()
    ]


@app.get("/api/gains")
def get_gains(plane: str, law: str, task: str) -> list[dict]:
    return gain_specs(plane, law, task)


@app.post("/api/runs")
def post_run(body: RunRequest) -> dict:
    _reject_nonfinite_gains(body.gains)
    _reject_trim(body.trim)
    trim = None if body.trim is None else (body.trim.vt_mps, body.trim.altitude_m)
    result = run(
        body.plane,
        body.law,
        body.task,
        gains=body.gains,
        aero=body.aero,
        trim=trim,
    )
    return to_json(result)


if _UI_DIST.is_dir():
    app.mount("/", StaticFiles(directory=_UI_DIST, html=True), name="ui")


def main() -> None:
    """Serve the app on the spec's loopback port."""
    uvicorn.run(app, host="127.0.0.1", port=8765)


if __name__ == "__main__":
    main()
