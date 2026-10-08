"""GCAS and sequenced waypoint guidance for the host-only X-31.

The vendored controllers take one slow command, ``{"V", "Chi", "Gamma"}``,
and they own the seven surfaces. These maneuvers only change that command.
A step in ``Gamma``, or a heading step, is already known to hit the port's
85 deg alpha limit, so the pull is a lead of a few degrees on the measured
flight path and the heading command is slewed. Both are held for ``HOLD_S``
and then refreshed. The plant, the actuators and both controllers stay the
port's.

GCAS is the F-16 mode machine (standby, roll, pull) on that command. The
deck is the F-16 deck, 1000 ft. The X-31 trims at 50 m/s and cannot spend a
5 g pull the way the F-16's Nz loop can, so the upright case starts high
enough that this gentler pull bottoms above the ground. Waypoint following
is the AeroBench F-16 list: ordered NED points, a capture radius, then the
next point. It is not the SITL ahead-waypoint line.
"""
from __future__ import annotations

import math

import numpy as np

from f16.units import GCAS_FLOOR_M, M_PER_FT
from x31.actuators import derivative as act_derivative
from x31.dynamics import AngleLimitError
from x31.ode45 import Ode45Error, ode45
from x31.quaternion import body_321_to_q, q_to_body_321, rotate_body_to_earth
from x31.simulate import _held_measurement, _pack, _plant_sample, _split
from x31.types import ActuatorState
from x31_sim import column_name, controller_states, surfaces

import x31.actuators as actuators
import x31.gain_schedule as gain_schedule
import x31.ndi as ndi
import x31.simulate as simulate

# Guidance period the pull and the heading slew were flown at. Refreshing the
# gamma lead much faster holds a constant error and steepens the pull.
HOLD_S = 0.5
_V_MPS = 50.0
_TRIM_ALPHA_RAD = math.radians(4.8648645161451505)
_GAMMA_LEAD_DEG = 2.0
_CHI_SLEW_DPS = 5.0
_GAMMA_SLEW_DPS = 1.0
_PULL_GAMMA_DEG = 8.0
_ALPHA_HOLD_DEG = 22.0
_WINGS_DEG = 8.0
_ROLL_RATE_DPS = 15.0
_MIN_PULL_S = 2.0
_CAPTURE_M = 250.0 * M_PER_FT

# AeroBench ``run_waypoint.py`` lists (east ft, north ft, altitude ft).
_WAYPOINTS_FT = (
    (1000.0, 3000.0, 4000.0),
    (3000.0, 8000.0, 3900.0),
    (1000.0, 23000.0, 3650.0),
    (500.0, 28000.0, 300.0),
)

_MISSIONS = {
    "gcas_upright": {
        "kind": "gcas",
        "duration_s": 26.0,
        "phi": -math.pi / 8.0,
        "theta": -0.3 * math.pi / 2.0,
        "psi": 0.0,
        "n_m": 0.0,
        "e_m": 0.0,
        "h_m": 600.0,
    },
    "gcas_inverted": {
        "kind": "gcas",
        "duration_s": 12.0,
        "phi": -0.9 * math.pi,
        "theta": -0.01 * math.pi / 2.0,
        "psi": 0.0,
        "n_m": 0.0,
        "e_m": 0.0,
        "h_m": 330.0,
    },
    "gcas_long": {
        "kind": "gcas",
        "duration_s": 120.0,
        "phi": -math.pi / 8.0,
        "theta": -0.3 * math.pi / 2.0,
        "psi": 0.0,
        "n_m": 0.0,
        "e_m": 0.0,
        "h_m": 600.0,
    },
    "waypoint": {
        "kind": "waypoint",
        "duration_s": 40.0,
        "phi": 0.0,
        "theta": _TRIM_ALPHA_RAD,
        "psi": math.pi / 8.0,
        "n_m": 0.0,
        "e_m": 0.0,
        "h_m": 3800.0 * M_PER_FT,
    },
}


def guided_names() -> tuple[str, ...]:
    return tuple(_MISSIONS)


def mission_duration(name: str) -> float:
    return float(_mission(name)["duration_s"])


def _mission(name: str) -> dict:
    try:
        return _MISSIONS[name]
    except KeyError:
        raise ValueError(
            f"unknown guided maneuver {name!r}; known guided maneuvers are "
            f"{', '.join(guided_names())}"
        ) from None


def _wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def _wrap_rad(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _lead(measured_deg: float, target_deg: float, limit_deg: float) -> float:
    err = _wrap_deg(target_deg - measured_deg)
    err = max(-limit_deg, min(limit_deg, err))
    return measured_deg + err


def _slew(current: float, target: float, rate_dps: float, dt: float) -> float:
    step = max(-rate_dps * dt, min(rate_dps * dt, _wrap_deg(target - current)))
    return _wrap_deg(current + step)


class Gcas:
    """Standby, roll, then a lead-limited pull. Same modes as the F-16 GCAS."""

    def __init__(self) -> None:
        self.mode = "standby"
        self.pull_t: float | None = None

    def advance(self, t: float, state: dict) -> None:
        above = state["h"] >= GCAS_FLOOR_M
        nose_high = state["gamma"] > 0.0
        wings = abs(state["phi"]) < _WINGS_DEG and abs(state["p"]) < _ROLL_RATE_DPS
        if self.mode == "standby":
            if (not nose_high) and (not above):
                self.mode = "roll"
        elif self.mode == "roll":
            if wings:
                self.mode = "pull"
                self.pull_t = float(t)
        elif nose_high and self.pull_t is not None and t >= self.pull_t + _MIN_PULL_S:
            self.mode = "standby"

    def command(self, state: dict, dt: float) -> dict:
        del dt
        # Standby holds a dive. Once the nose is up it eases back toward
        # level; holding the measured climb lets the alpha loop run away.
        if self.mode == "pull" and state["alpha"] <= _ALPHA_HOLD_DEG:
            gamma = _lead(state["gamma"], _PULL_GAMMA_DEG, _GAMMA_LEAD_DEG)
        elif self.mode == "standby" and state["gamma"] > 0.0 and state["alpha"] <= _ALPHA_HOLD_DEG:
            gamma = _lead(state["gamma"], 0.0, _GAMMA_LEAD_DEG)
        else:
            gamma = state["gamma"]
        return {"V": _V_MPS, "Chi": state["chi"], "Gamma": gamma}


class Waypoints:
    """AeroBench waypoint list. Capture one point, then fly the next."""

    def __init__(self) -> None:
        self.points = tuple(
            (n_ft * M_PER_FT, e_ft * M_PER_FT, h_ft * M_PER_FT)
            for e_ft, n_ft, h_ft in _WAYPOINTS_FT
        )
        self.index = 0
        self.chi_cmd: float | None = None
        self.gamma_cmd: float | None = None

    @property
    def mode(self) -> str:
        if self.index >= len(self.points):
            return "done"
        return f"waypoint {self.index + 1}"

    def advance(self, t: float, state: dict) -> None:
        del t
        if self.index >= len(self.points):
            return
        n, e, h = self.points[self.index]
        slant = math.sqrt(
            (n - float(state["pos"][0])) ** 2
            + (e - float(state["pos"][1])) ** 2
            + (h - state["h"]) ** 2
        )
        if slant < _CAPTURE_M:
            self.index += 1

    def command(self, state: dict, dt: float) -> dict:
        # The command is slewed toward the bearing. A fixed lead on the
        # measured heading never falls to zero, the chi integrator holds
        # bank, and the aircraft orbits the point.
        if self.chi_cmd is None or self.gamma_cmd is None:
            self.chi_cmd = state["chi"]
            self.gamma_cmd = state["gamma"]
        if self.index >= len(self.points):
            bearing = self.chi_cmd
            target = 0.0
        else:
            n, e, h = self.points[self.index]
            bearing = math.degrees(
                math.atan2(e - float(state["pos"][1]), n - float(state["pos"][0]))
            )
            target = max(-5.0, min(8.0, 0.02 * (h - state["h"])))
        assert self.chi_cmd is not None and self.gamma_cmd is not None
        self.chi_cmd = _slew(self.chi_cmd, bearing, _CHI_SLEW_DPS, dt)
        rate = 0.0 if state["alpha"] > _ALPHA_HOLD_DEG and target > self.gamma_cmd else _GAMMA_SLEW_DPS
        self.gamma_cmd = self.gamma_cmd + max(
            -rate * dt, min(rate * dt, target - self.gamma_cmd)
        )
        return {"V": _V_MPS, "Chi": self.chi_cmd, "Gamma": self.gamma_cmd}


def _guidance(name: str):
    kind = _mission(name)["kind"]
    if kind == "gcas":
        return Gcas()
    return Waypoints()


def _initial(name: str):
    body = _mission(name)
    q = body_321_to_q(float(body["phi"]), float(body["theta"]), float(body["psi"]))
    speed = _V_MPS
    inertial = np.array([
        speed * math.cos(_TRIM_ALPHA_RAD),
        0.0,
        speed * math.sin(_TRIM_ALPHA_RAD),
    ])
    vel = rotate_body_to_earth(q, inertial)
    pos = np.array([float(body["n_m"]), float(body["e_m"]), -float(body["h_m"])])
    return pos, vel, q, np.zeros(3)


def _sense(y: np.ndarray, n_act: int, n_ctrl: int) -> dict:
    pos, _vel, q, _w, _act, _ctrl = _split(y, n_act, n_ctrl)
    qn, _surf, _rates, measured = _plant_sample(0.0, pos, _vel, q, _w, _act)
    phi, _theta, _psi = q_to_body_321(qn)
    return {
        "pos": pos,
        "h": -float(pos[2]),
        "phi": math.degrees(_wrap_rad(float(phi))),
        "gamma": math.degrees(float(measured["gamma_path"])),
        "chi": math.degrees(float(measured["chi"])),
        "alpha": math.degrees(float(measured["alpha"])),
        "p": math.degrees(float(measured["p"])),
    }


def _module(controller: str):
    if controller == "gain_schedule":
        return gain_schedule
    if controller == "ndi":
        return ndi
    raise ValueError(
        f"guided maneuvers run gain_schedule or ndi, not {controller!r}"
    )


def run_guided(
    controller: str,
    name: str,
    duration: float | None = None,
    step: float = 1.0 / 30.0,
) -> dict:
    """Fly one guided maneuver and return the runner's column dict.

    ``mode`` is the controller name. ``modes`` is one guidance label per
    sample (``standby``, ``roll``, ``pull``, ``waypoint 1``, ``done``),
    which is what the CSV ``mode`` column carries for these maneuvers.
    """
    _mission(name)
    module = _module(controller)
    if duration is None:
        duration = mission_duration(name)
    if not np.isfinite(duration) or duration <= 0.0:
        raise ValueError(f"unphysical duration {duration!r} for {name!r}")
    if not np.isfinite(step) or step <= 0.0:
        raise ValueError(f"unphysical step {step!r}")

    guidance = _guidance(name)
    pos, vel, q, w = _initial(name)
    n_act = int(actuators.initial_state().x.size)
    n_ctrl = controller_states(controller)
    y = _pack(pos, vel, q, w, n_ctrl=n_ctrl)
    delay = (
        simulate._MeasurementDelay(float(np.linalg.norm(vel)))
        if controller == "gain_schedule"
        else None
    )
    channels = surfaces()
    times: list[float] = []
    modes: list[str] = []
    columns: dict[str, list[float]] = {
        key: []
        for key in (
            "n_m", "e_m", "d_m", "vt_mps", "alpha", "beta", "phi", "theta", "psi",
            *(column_name(surface, command) for command in (True, False) for surface in channels),
        )
    }
    stopped_at = None
    stop_reason = None

    def record(t: float, state: np.ndarray, command: dict, label: str) -> None:
        p, v, qi, wi, act, ctrl = _split(state, n_act, n_ctrl)
        qn, surf, _rates, measured = _plant_sample(float(t), p, v, qi, wi, act)
        if delay is not None:
            held = _held_measurement(delay, float(t), measured)
        else:
            held = measured
        cmd, _dots = module.surface(float(t), held, command, ctrl)
        euler = q_to_body_321(qn)
        times.append(float(t))
        modes.append(label)
        columns["n_m"].append(float(p[0]))
        columns["e_m"].append(float(p[1]))
        columns["d_m"].append(float(p[2]))
        columns["vt_mps"].append(float(measured["V"]))
        columns["alpha"].append(float(measured["alpha"]))
        columns["beta"].append(float(measured["beta"]))
        columns["phi"].append(float(euler[0]))
        columns["theta"].append(float(euler[1]))
        columns["psi"].append(float(euler[2]))
        for surface in channels:
            columns[column_name(surface, command=True)].append(float(getattr(cmd, surface)))
            columns[column_name(surface, command=False)].append(float(getattr(surf, surface)))

    def hold(t0: float, dt: float, command: dict):
        def rhs(tt, yy, command=command):
            p, v, qi, wi, act, ctrl = _split(yy, n_act, n_ctrl)
            qn, _surf, rates, measured = _plant_sample(float(tt), p, v, qi, wi, act)
            if delay is not None:
                delay.record(float(tt), measured, q_to_body_321(qn))
                measured = _held_measurement(delay, float(tt), measured)
            cmd, dots = module.surface(float(tt), measured, command, ctrl)
            act_dot = act_derivative(ActuatorState(x=act), cmd)
            return np.concatenate([
                rates.pos,
                rates.vel,
                rates.q,
                rates.w,
                act_dot,
                np.asarray(dots, dtype=float).reshape(-1),
            ])

        grid = _log_grid(t0, t0 + dt, step)
        solution = ode45(
            rhs, (t0, t0 + dt), y, rtol=1e-3, atol=1e-6, t_eval=grid,
        )
        return solution.t, solution.y

    t = 0.0
    label = guidance.mode
    command: dict | None = None
    try:
        while True:
            dt = min(HOLD_S, duration - t)
            sensed = _sense(y, n_act, n_ctrl)
            guidance.advance(t, sensed)
            command = guidance.command(sensed, dt)
            if not times:
                record(t, y, command, guidance.mode)
            if t >= duration - 1e-9:
                break
            label = guidance.mode
            logged_t, logged_y = hold(t, dt, command)
            y = np.asarray(logged_y[-1], dtype=float)
            for sample_t, sample_y in zip(logged_t, logged_y):
                record(float(sample_t), np.asarray(sample_y, dtype=float), command, label)
            t += dt
    except AngleLimitError as exc:
        partial = getattr(exc, "partial", None)
        if partial is not None and partial.t.size > 0 and command is not None:
            for sample_t, sample_y in zip(partial.t, partial.y):
                record(float(sample_t), np.asarray(sample_y, dtype=float), command, label)
            y = np.asarray(partial.y[-1], dtype=float)
            t = float(partial.t[-1])
        stopped_at = t
        stop_reason = f"{exc} (limit {exc.limit_deg:g} deg)"
    except Ode45Error as exc:
        partial = getattr(exc, "partial", None)
        if partial is not None and partial.t.size > 0 and command is not None:
            for sample_t, sample_y in zip(partial.t, partial.y):
                record(float(sample_t), np.asarray(sample_y, dtype=float), command, label)
            y = np.asarray(partial.y[-1], dtype=float)
            t = float(partial.t[-1])
        stopped_at = t
        stop_reason = f"integrator: {exc}"

    out = {
        "t": np.asarray(times, dtype=float),
        "mode": controller,
        "modes": modes,
        "stopped_at": stopped_at,
        "stop_reason": stop_reason,
    }
    for name_key, values in columns.items():
        out[name_key] = np.asarray(values, dtype=float)
    return out


def _log_grid(t0: float, t1: float, step: float) -> np.ndarray:
    """Sample times in ``(t0, t1]``. The start is already recorded."""
    count = max(1, int(round((t1 - t0) / step)))
    grid = t0 + (t1 - t0) * np.arange(1, count + 1, dtype=float) / count
    grid[-1] = t1
    return grid
