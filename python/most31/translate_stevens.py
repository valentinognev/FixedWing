"""Translate Stevens & Lewis F-16 tables into MOST31 coefficients."""
from __future__ import annotations

import numpy as np

from most31.schema import (
    ANGLE_DEG_PER_RAD,
    MULTIPLIERS,
    OUTPUTS,
    Most31Coefficients,
    zeros,
)

_DAMPING = (
    ("Cx", "qhat"),
    ("Cy", "rhat"),
    ("Cy", "phat"),
    ("Cz", "qhat"),
    ("Cl", "rhat"),
    ("Cl", "phat"),
    ("Cm", "qhat"),
    ("Cn", "rhat"),
    ("Cn", "phat"),
)


def translate_stevens(coeff: dict, source: str) -> Most31Coefficients:
    """Map a Stevens ``coefficients`` object into MOST31. Angles stay radians."""
    out = zeros(source)
    cx = OUTPUTS.index("Cx")
    cy = OUTPUTS.index("Cy")
    cz = OUTPUTS.index("Cz")
    cl = OUTPUTS.index("Cl")
    cm = OUTPUTS.index("Cm")
    cn = OUTPUTS.index("Cn")
    one = MULTIPLIERS.index("1")
    da = MULTIPLIERS.index("da")
    dr = MULTIPLIERS.index("dr")
    factor = ANGLE_DEG_PER_RAD
    aileron = factor / float(coeff["aileron_norm_deg"])
    rudder = factor / float(coeff["rudder_norm_deg"])
    beta = factor / float(coeff["beta_norm_deg"])

    out.table_ae[cx, one] = np.asarray(coeff["cx"], dtype=np.float64)
    out.table_ae[cm, one] = np.asarray(coeff["cm"], dtype=np.float64)

    cz_alpha = np.asarray(coeff["cz"], dtype=np.float64)
    out.table_a[cz, one, 0, :] = cz_alpha
    out.table_a[cz, one, 2, :] = -(beta ** 2) * cz_alpha
    out.poly[cz, one, 0, 0, 0, 1] = float(coeff["cz_elevator"]) * factor / float(
        coeff["elevator_norm_deg"]
    )

    out.table_aabsb[cl, one] = np.asarray(coeff["cl"], dtype=np.float64)
    out.table_aabsb[cn, one] = np.asarray(coeff["cn"], dtype=np.float64)
    out.table_ab[cl, da] = np.asarray(coeff["dlda"], dtype=np.float64) * aileron
    out.table_ab[cn, da] = np.asarray(coeff["dnda"], dtype=np.float64) * aileron
    out.table_ab[cl, dr] = np.asarray(coeff["dldr"], dtype=np.float64) * rudder
    out.table_ab[cn, dr] = np.asarray(coeff["dndr"], dtype=np.float64) * rudder

    out.poly[cy, one, 0, 0, 1, 0] = float(coeff["cy_beta"]) * factor
    out.poly[cy, da, 0, 0, 0, 0] = float(coeff["cy_aileron"]) * aileron
    out.poly[cy, dr, 0, 0, 0, 0] = float(coeff["cy_rudder"]) * rudder

    damp = np.asarray(coeff["damp"], dtype=np.float64)
    for column, (output, multiplier) in enumerate(_DAMPING):
        out.table_a[OUTPUTS.index(output), MULTIPLIERS.index(multiplier), 0, :] = damp[:, column]
    return out
