"""Thirteen-state host derivative."""
from __future__ import annotations

import math

import numpy as np

from f16.aero_morelli import morelli_coefficients
from f16.units import G_MPS2, aerobench_poly_rad
from plane.aircraft import Aircraft, thrust_n
from plane.atmosphere import air


def plane_derivative(x: np.ndarray, u: np.ndarray, aircraft: Aircraft) -> np.ndarray:
    if np.shape(x) != (13,) or np.shape(u) != (4,):
        raise ValueError(f"expected x shape (13,) and u shape (4,), got {np.shape(x)} and {np.shape(u)}")
    if float(x[0]) <= 0.0:
        raise ValueError("vt must be > 0")
    vt, alpha, beta, phi, theta, psi, p, q, r = (float(v) for v in x[:9])
    alt = float(x[11])
    power = float(x[12])
    throttle, de, da, dr = (float(v) for v in u)
    _, qbar, _ = air(vt, alt)
    cx, cy, cz, cl, cm, cn = morelli_coefficients(
        aerobench_poly_rad(alpha), aerobench_poly_rad(beta),
        de, da, dr, p, q, r,
        aircraft.cbar_m, aircraft.b_m, vt, aircraft.xcg, aircraft.xcg_ref,
        root=aircraft.coeff_dir,
        model=aircraft.model,
    )
    thrust = thrust_n(power, vt, aircraft)
    cb = math.cos(beta)
    ub = vt * math.cos(alpha) * cb
    vb = vt * math.sin(beta)
    wb = vt * math.sin(alpha) * cb
    sth, cth = math.sin(theta), math.cos(theta)
    sph, cph = math.sin(phi), math.cos(phi)
    spsi, cpsi = math.sin(psi), math.cos(psi)
    qs = qbar * aircraft.s_m2
    qsb = qs * aircraft.b_m
    inv_m = 1.0 / aircraft.mass_kg
    ax = inv_m * (qs * cx + thrust)
    ay = inv_m * qs * cy
    az = inv_m * qs * cz
    udot = r * vb - q * wb - G_MPS2 * sth + ax
    vdot = p * wb - r * ub + G_MPS2 * cth * sph + ay
    wdot = q * ub - p * vb + G_MPS2 * cth * cph + az
    dum = ub * ub + wb * wb
    xd = np.zeros(13)
    xd[0] = (ub * udot + vb * vdot + wb * wdot) / vt
    xd[1] = (ub * wdot - wb * udot) / dum
    xd[2] = (vt * vdot - vb * xd[0]) * cb / dum
    xd[3] = p + (sth / cth) * (q * sph + r * cph)
    xd[4] = q * cph - r * sph
    xd[5] = (q * sph + r * cph) / cth
    he = aircraft.he
    xd[6] = (aircraft.c2 * p + aircraft.c1 * r + aircraft.c4 * he) * q + qsb * (aircraft.c3 * cl + aircraft.c4 * cn)
    xd[7] = (aircraft.c5 * p - aircraft.c7 * he) * r + aircraft.c6 * (r * r - p * p) + qs * aircraft.cbar_m * aircraft.c7 * cm
    xd[8] = (aircraft.c8 * p - aircraft.c2 * r + aircraft.c9 * he) * q + qsb * (aircraft.c4 * cl + aircraft.c9 * cn)
    t1 = sph * cpsi
    t2 = cph * sth
    t3 = sph * spsi
    s1 = cth * cpsi
    s2 = cth * spsi
    s3 = t1 * sth - cph * spsi
    s4 = t3 * sth + cph * cpsi
    s5 = sph * cth
    s6 = t2 * cpsi + t3
    s7 = t2 * spsi - t1
    s8 = cph * cth
    xd[9] = ub * s1 + vb * s3 + wb * s6
    xd[10] = ub * s2 + vb * s4 + wb * s7
    xd[11] = ub * sth - vb * s5 - wb * s8
    xd[12] = (throttle - power) / aircraft.tau_s
    return xd
