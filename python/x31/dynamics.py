"""X-31 Python port. Copyright (C) 2016 Hsin-Yi Kang. GPLv2.

Vendored verbatim into FixedWing from
Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft at commit
c03cc39. Only this provenance note differs from the upstream file.
"""

import numpy as np

from x31.aero import body_force_moment
from x31.params import physical
from x31.quaternion import multiply, rotate_body_to_earth, rotate_earth_to_body
from x31.types import PlantState

_G = 9.81
_RHO = 1.23
_ALPHA_LIMIT = 85.0 * np.pi / 180.0
_BETA_LIMIT = 80.0 * np.pi / 180.0


class AngleLimitError(RuntimeError):
    """The plant `Angle limit` subsystem assertion.

    The diagram wires this to Stop Simulation, so a log ends at the limit
    rather than failing. It stays a RuntimeError so callers that assert the
    stop keep working.
    """

    def __init__(self, message, limit_deg):
        self.limit_deg = float(limit_deg)
        super().__init__(message)


def normalize(q):
    """Quaternion Normalization: q / signedSqrt(sum(q.^2))."""
    q = np.asarray(q, dtype=float)
    total = float(np.sum(np.square(q)))
    norm = float(np.sign(total) * np.sqrt(abs(total)))
    return q / norm


def _inertia(geom):
    ixx = geom["Ixx"]
    iyy = geom["Iyy"]
    izz = geom["Izz"]
    ixy = geom["Ixy"]
    ixz = geom["Ixz"]
    iyz = geom["Iyz"]
    return np.array(
        [
            [ixx, -ixy, -ixz],
            [-ixy, iyy, -iyz],
            [-ixz, -iyz, izz],
        ],
        dtype=float,
    )


def _thrust_body(surface, geom):
    thrust = float(surface.thrust)
    pitch = np.deg2rad(float(surface.thrust_pitch))
    yaw = np.deg2rad(float(surface.thrust_yaw))
    axial = thrust * np.cos(pitch) * np.cos(yaw)
    side = thrust * np.cos(pitch) * np.sin(yaw)
    normal = thrust * np.sin(pitch)
    lever = geom["Xt"]
    force = 1000.0 * np.array([axial, side, normal])
    moment = 1000.0 * np.array([0.0, normal * lever, -side * lever])
    return force, moment


def _air_data(q, vel):
    body = rotate_earth_to_body(q, vel)
    v_inf = float(np.linalg.norm(body))
    beta = float(np.asin(body[1] / v_inf))
    alpha = float(np.asin(body[2] / (v_inf * np.cos(beta))))
    return v_inf, alpha, beta


def derivative(t, state, surface, table=None):
    """Rigid-body rates. t is the integrator time and is unused by the vector field.

    This is the integrator input, so it carries the `Angle limit` assertion.
    Use `rates` to log a state that already sits at the limit.
    `table` is the Figure 2.2 set; `None` keeps v0.3.1.
    """
    _v_inf, alpha, beta = _air_data(
        np.asarray(state.q, dtype=float), np.asarray(state.vel, dtype=float)
    )
    if abs(alpha) > _ALPHA_LIMIT:
        raise AngleLimitError("85 degree angle limit", 85.0)
    if abs(beta) > _BETA_LIMIT:
        raise AngleLimitError("80 degree angle limit", 80.0)
    return rates(t, state, surface, table=table)


def rates(t, state, surface, table=None):
    """The same vector field without the angle-limit assertion.

    Simulink wires the limit to Stop Simulation, not to the To Workspace
    blocks, so the last samples of a log that ends at the limit still exist.
    `table` is forwarded to `body_force_moment`; `None` keeps v0.3.1.
    """
    del t
    geom = physical()
    q = np.asarray(state.q, dtype=float)
    v_inf, alpha, beta = _air_data(q, state.vel)
    rates = np.asarray(state.w, dtype=float)
    force_b, moment_b = body_force_moment(
        v_inf, np.rad2deg(alpha), np.rad2deg(beta), rates, surface, _RHO, table=table
    )
    thrust_force, thrust_moment = _thrust_body(surface, geom)
    force_e = rotate_body_to_earth(q, force_b + thrust_force)
    mass = geom["mass"]
    vel_dot = force_e / mass + np.array([0.0, 0.0, _G])
    pos_dot = np.asarray(state.vel, dtype=float).copy()
    q_dot = 0.5 * multiply(q, np.array([0.0, rates[0], rates[1], rates[2]]))
    inertia = _inertia(geom)
    w_dot = np.linalg.solve(
        inertia, moment_b + thrust_moment - np.cross(rates, inertia @ rates)
    )
    return PlantState(pos=pos_dot, vel=vel_dot, q=q_dot, w=w_dot)
