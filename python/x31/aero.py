"""X-31 Python port. Copyright (C) 2016 Hsin-Yi Kang. GPLv2.

Vendored verbatim into FixedWing from
Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft at commit
c03cc39. Only this provenance note differs from the upstream file.
"""

import numpy as np

from x31.params import fig22, physical
from x31.quaternion import body_321_to_q, rotate_body_to_earth


def eval_poly(coeff8, angle_deg):
    c = np.asarray(coeff8, dtype=float)
    angle = float(angle_deg)
    a = abs(angle)
    value = (
        c[0] * a
        + c[1] * a**2
        + c[2] * a**3
        + c[3] * a**4
        + c[4] * a**5
        + c[5] * a**6
        + c[6]
    )
    if c[7] == 1.0 and angle < 0.0:
        value = -value + 2.0 * c[6]
    return float(value)


def _coeff(table, index, column, alpha_deg):
    return eval_poly(table[index, column], alpha_deg)


def body_force_moment(v_inf, alpha_deg, beta_deg, rates, surface, rho, table=None):
    """Body-axis aerodynamic force and moment from the Forces and Moment subsystems.

    `table` is the Figure 2.2 coefficient set; `None` keeps the v0.3.1 table, so
    every default path is unchanged. A caller replaying a stored log passes that
    log's own `stored_table`.
    """
    v_inf = float(v_inf)
    alpha_deg = float(alpha_deg)
    beta_deg = float(beta_deg)
    rates = np.asarray(rates, dtype=float)
    rho = float(rho)
    p, q, r = rates
    alpha = np.deg2rad(alpha_deg)
    beta = np.deg2rad(beta_deg)
    table = fig22() if table is None else table

    cl = _coeff(table, 0, 0, alpha_deg)
    cldc = _coeff(table, 1, 0, alpha_deg)
    lift_offset = _coeff(table, 1, 1, alpha_deg)
    cd = _coeff(table, 0, 1, alpha_deg)
    cy = _coeff(table, 2, 0, alpha_deg)
    cyda = 0.0
    cydr = _coeff(table, 2, 1, alpha_deg)
    clb = _coeff(table, 3, 0, alpha_deg)
    clp = _coeff(table, 4, 0, alpha_deg)
    clr = _coeff(table, 5, 0, alpha_deg)
    clda = _coeff(table, 6, 0, alpha_deg)
    cldr = _coeff(table, 7, 0, alpha_deg)
    cm0 = _coeff(table, 8, 0, alpha_deg)
    cmq = _coeff(table, 8, 1, alpha_deg)
    cmdc = _coeff(table, 9, 0, alpha_deg)
    moment_offset = _coeff(table, 9, 1, alpha_deg)
    cnb = _coeff(table, 3, 1, alpha_deg)
    cnp = _coeff(table, 4, 1, alpha_deg)
    cnr = _coeff(table, 5, 1, alpha_deg)
    cnda = _coeff(table, 6, 1, alpha_deg)
    cndr = _coeff(table, 7, 1, alpha_deg)

    aileron = float(surface.aileron)
    rudder = float(surface.rudder)
    canard = float(surface.canard)
    cl_tot = cl + cldc * (canard - lift_offset)
    cy_tot = cy * beta + cyda * aileron + cydr * rudder

    geom = physical()
    sref = geom["Sref"]
    span = geom["b"]
    cbar = geom["Cbar"]
    qbar = (rho * v_inf**2) / 2.0
    lift = qbar * sref * cl_tot
    drag = qbar * sref * cd
    side = qbar * sref * cy_tot

    lift_b = np.array([lift * np.sin(alpha), 0.0, lift * (-np.cos(alpha))])
    side_b = np.array([side * (-np.sin(beta)), side * np.cos(beta), 0.0])
    drag_b = rotate_body_to_earth(
        body_321_to_q(0.0, -alpha, beta), np.array([-drag, 0.0, 0.0])
    )
    force = lift_b + drag_b + side_b

    two_v = 2.0 * v_inf
    roll = (
        clb * beta
        + clp * p * span / two_v
        + clr * r * span / two_v
        + clda * aileron
        + cldr * rudder
    )
    pitch = cm0 + cmq * q * cbar / two_v + cmdc * (canard - moment_offset)
    yaw = (
        cnb * beta
        + cnp * p * span / two_v
        + cnr * r * span / two_v
        + cnda * aileron
        + cndr * rudder
    )
    moment = np.array(
        [qbar * sref * span * roll, qbar * sref * cbar * pitch, qbar * sref * span * yaw]
    )
    return force, moment
