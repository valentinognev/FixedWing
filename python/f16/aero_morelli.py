"""Pure Morelli F-16 aerodynamic force and moment coefficients."""
from __future__ import annotations

from pathlib import Path

from f16.aero_data import load_aero_coefficients


def morelli_coefficients(
    alpha, beta, de, da, dr, p, q, r, cbar, b, V, xcg, xcgref, *, root: Path | None = None,
):
    phat = p * b / (2 * V)
    qhat = q * cbar / (2 * V)
    rhat = r * b / (2 * V)

    coeff = load_aero_coefficients("morelli", root=root)
    cx = coeff["cx"]
    cxq = coeff["cxq"]
    cy = coeff["cy"]
    cyp = coeff["cyp"]
    cyr = coeff["cyr"]
    cz = coeff["cz"]
    czq = coeff["czq"]
    cl = coeff["cl"]
    clp = coeff["clp"]
    clr = coeff["clr"]
    clda = coeff["clda"]
    cldr = coeff["cldr"]
    cm = coeff["cm"]
    cmq = coeff["cmq"]
    cn = coeff["cn"]
    cnp = coeff["cnp"]
    cnr = coeff["cnr"]
    cnda = coeff["cnda"]
    cndr = coeff["cndr"]

    Cx0 = cx[0] + cx[1] * alpha + cx[2] * de**2 + cx[3] * de + cx[4] * alpha * de + cx[5] * alpha**2 + cx[6] * alpha**3
    Cxq = cxq[0] + cxq[1] * alpha + cxq[2] * alpha**2 + cxq[3] * alpha**3 + cxq[4] * alpha**4
    Cy0 = cy[0] * beta + cy[1] * da + cy[2] * dr
    Cyp = cyp[0] + cyp[1] * alpha + cyp[2] * alpha**2 + cyp[3] * alpha**3
    Cyr = cyr[0] + cyr[1] * alpha + cyr[2] * alpha**2 + cyr[3] * alpha**3
    Cz0 = (cz[0] + cz[1] * alpha + cz[2] * alpha**2 + cz[3] * alpha**3 + cz[4] * alpha**4) * (1 - beta**2) + cz[5] * de
    Czq = czq[0] + czq[1] * alpha + czq[2] * alpha**2 + czq[3] * alpha**3 + czq[4] * alpha**4
    Cl0 = cl[0] * beta + cl[1] * alpha * beta + cl[2] * alpha**2 * beta + cl[3] * beta**2 + cl[4] * alpha * beta**2 + cl[5] * \
        alpha**3 * beta + cl[6] * alpha**4 * beta + cl[7] * alpha**2 * beta**2
    Clp = clp[0] + clp[1] * alpha + clp[2] * alpha**2 + clp[3] * alpha**3
    Clr = clr[0] + clr[1] * alpha + clr[2] * alpha**2 + clr[3] * alpha**3 + clr[4] * alpha**4
    Clda = clda[0] + clda[1] * alpha + clda[2] * beta + clda[3] * alpha**2 + clda[4] * alpha * beta + clda[5] * alpha**2 * beta + clda[6] * alpha**3
    Cldr = cldr[0] + cldr[1] * alpha + cldr[2] * beta + cldr[3] * alpha * beta + cldr[4] * alpha**2 * beta + cldr[5] * alpha**3 * beta + cldr[6] * beta**2
    Cm0 = cm[0] + cm[1] * alpha + cm[2] * de + cm[3] * alpha * de + cm[4] * de**2 + cm[5] * alpha**2 * de + cm[6] * de**3 + cm[7] * \
        alpha * de**2
    Cmq = cmq[0] + cmq[1] * alpha + cmq[2] * alpha**2 + cmq[3] * alpha**3 + cmq[4] * alpha**4 + cmq[5] * alpha**5
    Cn0 = cn[0] * beta + cn[1] * alpha * beta + cn[2] * beta**2 + cn[3] * alpha * beta**2 + cn[4] * alpha**2 * beta + cn[5] * \
        alpha**2 * beta**2 + cn[6] * alpha**3 * beta
    Cnp = cnp[0] + cnp[1] * alpha + cnp[2] * alpha**2 + cnp[3] * alpha**3 + cnp[4] * alpha**4
    Cnr = cnr[0] + cnr[1] * alpha + cnr[2] * alpha**2
    Cnda = cnda[0] + cnda[1] * alpha + cnda[2] * beta + cnda[3] * alpha * beta + cnda[4] * alpha**2 * beta + cnda[5] * alpha**3 * beta + cnda[6] * \
        alpha**2 + cnda[7] * alpha**3 + cnda[8] * beta**3 + cnda[9] * alpha * beta**3
    Cndr = cndr[0] + cndr[1] * alpha + cndr[2] * beta + cndr[3] * alpha * beta + cndr[4] * alpha**2 * beta + cndr[5] * alpha**2

    Cx = Cx0 + Cxq * qhat
    Cy = Cy0 + Cyp * phat + Cyr * rhat
    Cz = Cz0 + Czq * qhat
    Cl = Cl0 + Clp * phat + Clr * rhat + Clda * da + Cldr * dr
    Cm = Cm0 + Cmq * qhat + Cz * (xcgref - xcg)
    Cn = Cn0 + Cnp * phat + Cnr * rhat + Cnda * da + Cndr * dr - Cy * (xcgref - xcg) * (cbar / b)
    return Cx, Cy, Cz, Cl, Cm, Cn
