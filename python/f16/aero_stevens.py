"""Stevens & Lewis F-16 aerodynamic coefficient tables (AeroBench port)."""
from __future__ import annotations

from math import ceil, floor
from pathlib import Path

import numpy as np

from f16.aero_data import load_aero_coefficients
from f16.units import aerobench_deg


def _fix(ele):
    """Round towards zero."""
    assert isinstance(ele, float)
    if ele > 0:
        return int(floor(ele))
    return int(ceil(ele))


def _sign(ele):
    if ele < 0:
        return -1
    if ele == 0:
        return 0
    return 1


def _table(coeff, name):
    return np.array(coeff[name], dtype=float)


def cx_table(alpha, el, coeff):
    'cx definition'

    a = _table(coeff, "cx")

    s = .2 * alpha
    k = _fix(s)
    if k <= -2:
        k = -1

    if k >= 9:
        k = 8

    da = s - k
    l = k + _fix(1.1 * _sign(da))
    s = el / 12
    m = _fix(s)
    if m <= -2:
        m = -1

    if m >= 2:
        m = 1

    de = s - m
    n = m + _fix(1.1 * _sign(de))
    k = k + 3
    l = l + 3
    m = m + 3
    n = n + 3
    t = a[k-1, m-1]
    u = a[k-1, n-1]
    v = t + abs(da) * (a[l-1, m-1] - t)
    w = u + abs(da) * (a[l-1, n-1] - u)
    cxx = v + (w - v) * abs(de)

    return cxx

def cy_table(beta, ail, rdr, coeff):
    'cy function'

    return (
        coeff["cy_beta"] * beta
        + coeff["cy_aileron"] * (ail / coeff["aileron_norm_deg"])
        + coeff["cy_rudder"] * (rdr / coeff["rudder_norm_deg"])
    )

def cz_table(alpha, beta, el, coeff):
    'cz function'

    a = _table(coeff, "cz")

    s = .2 * alpha
    k = _fix(s)

    if k <= -2:
        k = -1

    if k >= 9:
        k = 8

    da = s - k
    l = k + _fix(1.1 * _sign(da))
    l = l + 3
    k = k + 3
    s = a[k-1] + abs(da) * (a[l-1] - a[k-1])

    return s * (1 - (beta / coeff["beta_norm_deg"])**2) + coeff["cz_elevator"] * (el / coeff["elevator_norm_deg"])

def cl_table(alpha, beta, coeff):
    'cl function'

    a = _table(coeff, "cl")

    s = .2 * alpha
    k = _fix(s)

    if k <= -2:
        k = -1

    if k >= 9:
        k = 8

    da = s - k
    l = k + _fix(1.1 * _sign(da))
    s = .2 * abs(beta)
    m = _fix(s)
    if m == 0:
        m = 1

    if m >= 6:
        m = 5

    db = s - m
    n = m + _fix(1.1 * _sign(db))
    l = l + 3
    k = k + 3
    m = m + 1
    n = n + 1
    t = a[k-1, m-1]
    u = a[k-1, n-1]
    v = t + abs(da) * (a[l-1, m-1] - t)
    w = u + abs(da) * (a[l-1, n-1] - u)
    dum = v + (w - v) * abs(db)

    return dum * _sign(beta)

def cm_table(alpha, el, coeff):
    'cm function'

    a = _table(coeff, "cm")

    s = .2 * alpha
    k = _fix(s)

    if k <= -2:
        k = -1

    if k >= 9:
        k = 8

    da = s - k
    l = k + _fix(1.1 * _sign(da))
    s = el / 12
    m = _fix(s)

    if m <= -2:
        m = -1

    if m >= 2:
        m = 1

    de = s - m
    n = m + _fix(1.1 * _sign(de))
    k = k + 3
    l = l + 3
    m = m + 3
    n = n + 3
    t = a[k-1, m-1]
    u = a[k-1, n-1]
    v = t + abs(da) * (a[l-1, m-1] - t)
    w = u + abs(da) * (a[l-1, n-1] - u)

    return v + (w - v) * abs(de)

def cn_table(alpha, beta, coeff):
    'cn function'

    a = _table(coeff, "cn")

    s = .2 * alpha
    k = _fix(s)

    if k <= -2:
        k = -1

    if k >= 9:
        k = 8

    da = s - k
    l = k + _fix(1.1 * _sign(da))
    s = .2 * abs(beta)
    m = _fix(s)

    if m == 0:
        m = 1

    if m >= 6:
        m = 5

    db = s - m
    n = m + _fix(1.1 * _sign(db))
    l = l + 3
    k = k + 3
    m = m + 1
    n = n + 1
    t = a[k-1, m-1]
    u = a[k-1, n-1]

    v = t + abs(da) * (a[l-1, m-1] - t)
    w = u + abs(da) * (a[l-1, n-1] - u)
    dum = v + (w - v) * abs(db)

    return dum * _sign(beta)

def dlda_table(alpha, beta, coeff):
    'dlda function'

    a = _table(coeff, "dlda")

    s = .2 * alpha
    k = _fix(s)
    if k <= -2:
        k = -1

    if k >= 9:
        k = 8

    da = s - k
    l = k + _fix(1.1 * _sign(da))
    s = .1 * beta
    m = _fix(s)
    if m <= -3:
        m = -2

    if m >= 3:
        m = 2

    db = s - m
    n = m + _fix(1.1 * _sign(db))
    l = l + 3
    k = k + 3
    m = m + 4
    n = n + 4
    t = a[k-1, m-1]
    u = a[k-1, n-1]
    v = t + abs(da) * (a[l-1, m-1] - t)
    w = u + abs(da) * (a[l-1, n-1] - u)

    return v + (w - v) * abs(db)

def dldr_table(alpha, beta, coeff):
    'dldr function'

    a = _table(coeff, "dldr")

    s = .2 * alpha
    k = _fix(s)
    if k <= -2:
        k = -1

    if k >= 9:
        k = 8

    da = s - k
    l = k + _fix(1.1 * _sign(da))
    s = .1 * beta
    m = _fix(s)

    if m <= -3:
        m = -2

    if m >= 3:
        m = 2

    db = s - m
    n = m + _fix(1.1 * _sign(db))
    l = l + 3
    k = k + 3
    m = m + 4
    n = n + 4
    t = a[k-1, m-1]
    u = a[k-1, n-1]

    v = t + abs(da) * (a[l-1, m-1] - t)
    w = u + abs(da) * (a[l-1, n-1] - u)

    return v + (w - v) * abs(db)

def dnda_table(alpha, beta, coeff):
    'dnda function'

    a = _table(coeff, "dnda")

    s = .2 * alpha
    k = _fix(s)

    if k <= -2:
        k = -1

    if k >= 9:
        k = 8

    da = s - k
    l = k + _fix(1.1 * _sign(da))
    s = .1 * beta
    m = _fix(s)
    if m <= -3:
        m = -2

    if m >= 3:
        m = 2

    db = s - m
    n = m + _fix(1.1 * _sign(db))
    l = l + 3
    k = k + 3
    m = m + 4
    n = n + 4
    t = a[k-1, m-1]
    u = a[k-1, n-1]
    v = t + abs(da) * (a[l-1, m-1] - t)
    w = u + abs(da) * (a[l-1, n-1] - u)

    return v + (w - v) * abs(db)

def dndr_table(alpha, beta, coeff):
    'dndr function'

    a = _table(coeff, "dndr")

    s = .2 * alpha
    k = _fix(s)
    if k <= -2:
        k = -1

    if k >= 9:
        k = 8

    da = s - k
    l = k + _fix(1.1 * _sign(da))
    s = .1 * beta
    m = _fix(s)
    if m <= -3:
        m = -2

    if m >= 3:
        m = 2

    db = s - m
    n = m + _fix(1.1 * _sign(db))
    l = l + 3
    k = k + 3
    m = m + 4
    n = n + 4
    t = a[k-1, m-1]
    u = a[k-1, n-1]
    v = t + abs(da) * (a[l-1, m-1] - t)
    w = u + abs(da) * (a[l-1, n-1] - u)
    return v + (w - v) * abs(db)

def damp_table(alpha_rad, coeff=None):
    """Stevens damping table. Argument is radians.

    The paper grid is 5 degrees. ``0.2 * aerobench_deg`` is that grid with
    AeroBench's 57.29578, matching ``subf16_model``.
    """
    if coeff is None:
        coeff = load_aero_coefficients("stevens")
    a = _table(coeff, "damp")
    s = 0.2 * aerobench_deg(alpha_rad)
    k = _fix(s)
    if k <= -2:
        k = -1
    if k >= 9:
        k = 8
    da = s - k
    l = k + _fix(1.1 * _sign(da))
    k = k + 3
    l = l + 3
    d = np.zeros((9,))
    for i in range(9):
        d[i] = a[k - 1, i] + abs(da) * (a[l - 1, i] - a[k - 1, i])
    return d


def stevens_coefficients(alpha_rad, beta_rad, de_rad, da_rad, dr_rad, p, q, r, cbar, b, vt, xcg, xcgref, *, root: Path | None = None):
    coeff = load_aero_coefficients("stevens", root=root)
    alpha = aerobench_deg(alpha_rad)
    beta = aerobench_deg(beta_rad)
    de = aerobench_deg(de_rad)
    da = aerobench_deg(da_rad)
    dr = aerobench_deg(dr_rad)
    dail = da / coeff["aileron_norm_deg"]
    drdr = dr / coeff["rudder_norm_deg"]
    cx = cx_table(alpha, de, coeff)
    cy = cy_table(beta, da, dr, coeff)
    cz = cz_table(alpha, beta, de, coeff)
    cl = cl_table(alpha, beta, coeff) + dlda_table(alpha, beta, coeff) * dail + dldr_table(alpha, beta, coeff) * drdr
    cm = cm_table(alpha, de, coeff)
    cn = cn_table(alpha, beta, coeff) + dnda_table(alpha, beta, coeff) * dail + dndr_table(alpha, beta, coeff) * drdr
    tvt = 0.5 / vt
    b2v = b * tvt
    cq = cbar * q * tvt
    d = damp_table(alpha_rad, coeff)
    cx = cx + cq * d[0]
    cy = cy + b2v * (d[1] * r + d[2] * p)
    cz = cz + cq * d[3]
    cl = cl + b2v * (d[4] * r + d[5] * p)
    cm = cm + cq * d[6] + cz * (xcgref - xcg)
    cn = cn + b2v * (d[7] * r + d[8] * p) - cy * (xcgref - xcg) * cbar / b
    return cx, cy, cz, cl, cm, cn
