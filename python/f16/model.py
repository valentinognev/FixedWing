"""Morelli F-16 state derivative (SI state, controls, and force)."""
from __future__ import annotations

from math import ceil, cos, floor, pi, sin, sqrt

import numpy as np

from f16.aero_morelli import morelli_coefficients
from f16.aero_stevens import stevens_coefficients
from f16.units import (
    ALT_FLOOR_M,
    B_M,
    C3,
    C4,
    C7,
    C9,
    CBAR_M,
    G_MPS2,
    H_STRAT_M,
    H_THRUST_STEP_M,
    HE_KGM2,
    LAPSE_PER_M,
    LBF_TO_N,
    R_AIR,
    RHO0_KG_M3,
    RM_PER_KG,
    S_M2,
    T0_K,
    T_STRAT_K,
    XA_M,
    aerobench_poly_rad,
)


def subf16_derivative(x13, u4, model: str = "morelli"):
    x13 = np.asarray(x13, dtype=float)
    u4 = np.asarray(u4, dtype=float)
    if x13.shape != (13,) or u4.shape != (4,):
        raise ValueError(f"expected x13 (13,) and u (4,), got {x13.shape} {u4.shape}")
    xd, Nz, Ny, _, _ = _subf16(x13, u4, model=model)
    return xd, float(Nz), float(Ny)


def _fix(ele):
    """Round towards zero."""
    assert isinstance(ele, float)
    if ele > 0:
        return int(floor(ele))
    return int(ceil(ele))


def _adc(vt, alt):
    tfac = 1 - LAPSE_PER_M * alt
    if alt >= H_STRAT_M:
        t = T_STRAT_K
    else:
        t = T0_K * tfac
    rho = RHO0_KG_M3 * tfac**4.14
    a = sqrt(1.4 * R_AIR * t)
    amach = vt / a
    qbar = .5 * rho * vt * vt
    return amach, qbar


def _tgear(thtl):
    if thtl <= .77:
        tg = 64.94 * thtl
    else:
        tg = 217.38 * thtl - 117.38
    return tg


def _rtau(dp):
    if dp <= 25:
        rt = 1.0
    elif dp >= 50:
        rt = .1
    else:
        rt = 1.9 - .036 * dp
    return rt


def _pdot(p3, p1):
    if p1 >= 50:
        if p3 >= 50:
            t = 5
            p2 = p1
        else:
            p2 = 60
            t = _rtau(p2 - p3)
    else:
        if p3 >= 50:
            t = 5
            p2 = 40
        else:
            p2 = p1
            t = _rtau(p2 - p3)
    return t * (p2 - p3)


def _thrust(power, alt, rmach):
    a = np.array([[1060, 670, 880, 1140, 1500, 1860],
                  [635, 425, 690, 1010, 1330, 1700],
                  [60, 25, 345, 755, 1130, 1525],
                  [-1020, -710, -300, 350, 910, 1360],
                  [-2700, -1900, -1300, -247, 600, 1100],
                  [-3600, -1400, -595, -342, -200, 700]], dtype=float).T * LBF_TO_N
    b = np.array([[12680, 9150, 6200, 3950, 2450, 1400],
                  [12680, 9150, 6313, 4040, 2470, 1400],
                  [12610, 9312, 6610, 4290, 2600, 1560],
                  [12640, 9839, 7090, 4660, 2840, 1660],
                  [12390, 10176, 7750, 5320, 3250, 1930],
                  [11680, 9848, 8050, 6100, 3800, 2310]], dtype=float).T * LBF_TO_N
    c = np.array([[20000, 15000, 10800, 7000, 4000, 2500],
                  [21420, 15700, 11225, 7323, 4435, 2600],
                  [22700, 16860, 12250, 8154, 5000, 2835],
                  [24240, 18910, 13760, 9285, 5700, 3215],
                  [26070, 21075, 15975, 11115, 6860, 3950],
                  [28886, 23319, 18300, 13484, 8642, 5057]], dtype=float).T * LBF_TO_N
    if alt < ALT_FLOOR_M:
        alt = ALT_FLOOR_M
    h = alt / H_THRUST_STEP_M
    i = _fix(h)
    if i >= 5:
        i = 4
    dh = h - i
    rm = 5 * rmach
    m = _fix(rm)
    if m >= 5:
        m = 4
    elif m <= 0:
        m = 0
    dm = rm - m
    cdh = 1 - dh
    s = b[i, m] * cdh + b[i + 1, m] * dh
    t = b[i, m + 1] * cdh + b[i + 1, m + 1] * dh
    tmil = s + (t - s) * dm
    if power < 50:
        s = a[i, m] * cdh + a[i + 1, m] * dh
        t = a[i, m + 1] * cdh + a[i + 1, m + 1] * dh
        tidl = s + (t - s) * dm
        thrst = tidl + (tmil - tidl) * power * .02
    else:
        s = c[i, m] * cdh + c[i + 1, m] * dh
        t = c[i, m + 1] * cdh + c[i + 1, m + 1] * dh
        tmax = s + (t - s) * dm
        thrst = tmil + (tmax - tmil) * (power - 50) * .02
    return thrst


def _coefficients(model, alpha, beta, de, da, dr, p, q, r, cbar, b, vt, xcg, xcgref):
    if model == "morelli":
        return morelli_coefficients(
            aerobench_poly_rad(alpha), aerobench_poly_rad(beta), de, da, dr,
            p, q, r, cbar, b, vt, xcg, xcgref)
    if model == "stevens":
        return stevens_coefficients(
            alpha, beta, de, da, dr, p, q, r, cbar, b, vt, xcg, xcgref)
    raise ValueError(f"model {model!r} is not implemented")


def _subf16(x, u, model: str = "morelli"):
    """SI F-16 dynamics with selectable aero model (Morelli poly or Stevens tables)."""
    xcg = 0.35

    thtlc, el, ail, rdr = u

    s = S_M2
    b = B_M
    cbar = CBAR_M
    rm = RM_PER_KG
    xcgr = .35
    he = HE_KGM2
    c1 = -.770
    c2 = .02755
    c3 = C3
    c4 = C4
    c5 = .9604
    c6 = 1.759e-2
    c7 = C7
    c8 = -.7336
    c9 = C9
    g = G_MPS2

    xd = x.copy()
    vt = x[0]
    phi = x[3]
    theta = x[4]
    psi = x[5]
    p = x[6]
    q = x[7]
    r = x[8]
    alt = x[11]
    power = x[12]

    amach, qbar = _adc(vt, alt)
    cpow = _tgear(thtlc)
    xd[12] = _pdot(power, cpow)
    t = _thrust(power, alt, amach)

    cxt, cyt, czt, clt, cmt, cnt = _coefficients(
        model, x[1], x[2], el, ail, rdr, p, q, r, cbar, b, vt, xcg, xcgr)
    cbta = cos(x[2])
    u = vt * cos(x[1]) * cbta
    v = vt * sin(x[2])
    w = vt * sin(x[1]) * cbta
    sth = sin(theta)
    cth = cos(theta)
    sph = sin(phi)
    cph = cos(phi)
    spsi = sin(psi)
    cpsi = cos(psi)
    qs = qbar * s
    qsb = qs * b
    rmqs = rm * qs
    gcth = g * cth
    qsph = q * sph
    ay = rmqs * cyt
    az = rmqs * czt

    udot = r * v - q * w - g * sth + rm * (qs * cxt + t)
    vdot = p * w - r * u + gcth * sph + ay
    wdot = q * u - p * v + gcth * cph + az
    dum = (u * u + w * w)

    xd[0] = (u * udot + v * vdot + w * wdot) / vt
    xd[1] = (u * wdot - w * udot) / dum
    xd[2] = (vt * vdot - v * xd[0]) * cbta / dum

    xd[3] = p + (sth / cth) * (qsph + r * cph)
    xd[4] = q * cph - r * sph
    xd[5] = (qsph + r * cph) / cth

    xd[6] = (c2 * p + c1 * r + c4 * he) * q + qsb * (c3 * clt + c4 * cnt)
    xd[7] = (c5 * p - c7 * he) * r + c6 * (r * r - p * p) + qs * cbar * c7 * cmt
    xd[8] = (c8 * p - c2 * r + c9 * he) * q + qsb * (c4 * clt + c9 * cnt)

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
    xd[9] = u * s1 + v * s3 + w * s6
    xd[10] = u * s2 + v * s4 + w * s7
    xd[11] = u * sth - v * s5 - w * s8

    xa = XA_M
    az = az - xa * xd[7]
    ay = ay + xa * xd[8]

    Nz = (-az / g) - 1
    Ny = ay / g
    return xd, Nz, Ny, az, ay
