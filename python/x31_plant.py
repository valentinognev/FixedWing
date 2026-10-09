"""X-31 rigid-body plant step with MOST31 aero in place of the port table."""
from __future__ import annotations

from pathlib import Path

import numpy as np

import x31_numpy_compat  # noqa: F401  restores numpy short trig aliases before x31
import most31
from x31 import actuators, dynamics
from x31.params import physical
from x31.quaternion import multiply, rotate_body_to_earth
from x31.simulate import _ZERO, _measured, _split
from x31.types import ActuatorState, PlantState

MOST31_PATH = Path(__file__).resolve().parents[1] / "data" / "planes" / "x31" / "most31.json"

_ALPHA_LIMIT = 85.0 * np.pi / 180.0
_BETA_LIMIT = 80.0 * np.pi / 180.0
_COEFFICIENTS = most31.load(MOST31_PATH)


def aero_force_moment(v_inf, alpha, beta, rates, surface):
    """Body force and moment. alpha and beta are radians; surface deflections are degrees."""
    v_inf = float(v_inf)
    rates = np.asarray(rates, dtype=float)
    geom = physical()
    sref = float(geom["Sref"])
    span = float(geom["b"])
    cbar = float(geom["Cbar"])
    cx, cy, cz, cl, cm, cn = most31.evaluate(
        _COEFFICIENTS,
        float(alpha),
        float(beta),
        0.0,
        np.deg2rad(float(surface.aileron)),
        np.deg2rad(float(surface.rudder)),
        np.deg2rad(float(surface.canard)),
        float(rates[0]),
        float(rates[1]),
        float(rates[2]),
        v_inf,
        span,
        cbar,
        0.0,
        0.0,
    )
    qbar = (dynamics._RHO * v_inf**2) / 2.0
    force = qbar * sref * np.array([cx, cy, cz], dtype=float)
    moment = qbar * sref * np.array([span * cl, cbar * cm, span * cn], dtype=float)
    return force, moment


def rates(t, state, surface):
    """Port `dynamics.rates` sequence with MOST31 aero."""
    del t
    geom = physical()
    q = np.asarray(state.q, dtype=float)
    v_inf, alpha, beta = dynamics._air_data(q, state.vel)
    rates = np.asarray(state.w, dtype=float)
    force_b, moment_b = aero_force_moment(v_inf, alpha, beta, rates, surface)
    thrust_force, thrust_moment = dynamics._thrust_body(surface, geom)
    force_e = rotate_body_to_earth(q, force_b + thrust_force)
    mass = geom["mass"]
    vel_dot = force_e / mass + np.array([0.0, 0.0, dynamics._G])
    pos_dot = np.asarray(state.vel, dtype=float).copy()
    q_dot = 0.5 * multiply(q, np.array([0.0, rates[0], rates[1], rates[2]]))
    inertia = dynamics._inertia(geom)
    w_dot = np.linalg.solve(
        inertia, moment_b + thrust_moment - np.cross(rates, inertia @ rates)
    )
    return PlantState(pos=pos_dot, vel=vel_dot, q=q_dot, w=w_dot)


def derivative(t, state, surface):
    """`rates` plus the port's 85° alpha / 80° beta stop."""
    _v_inf, alpha, beta = dynamics._air_data(
        np.asarray(state.q, dtype=float), np.asarray(state.vel, dtype=float)
    )
    if abs(alpha) > _ALPHA_LIMIT:
        raise dynamics.AngleLimitError("85 degree angle limit", 85.0)
    if abs(beta) > _BETA_LIMIT:
        raise dynamics.AngleLimitError("80 degree angle limit", 80.0)
    return rates(t, state, surface)


def plant_sample(t, pos, vel, q, w, act, surface=None):
    """Rates use the integrator quaternion. Logs and measurements use q/|q|."""
    q_int = np.asarray(q, dtype=float)
    qn = dynamics.normalize(q_int)
    surf = actuators.output(ActuatorState(x=act), _ZERO) if surface is None else surface
    body_rates = derivative(t, PlantState(pos=pos, vel=vel, q=q_int, w=w), surf)
    return qn, surf, body_rates, _measured(qn, vel, w, surf, body_rates)


def plant_rhs(t, y, n_act, command):
    pos, vel, q, w, act, _ctrl = _split(y, n_act, 0)
    _qn, _surf, body_rates, _sample = plant_sample(float(t), pos, vel, q, w, act)
    act_dot = actuators.derivative(ActuatorState(x=act), command)
    return np.concatenate(
        [body_rates.pos, body_rates.vel, body_rates.q, body_rates.w, act_dot]
    )
