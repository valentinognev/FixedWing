"""Translate a Morelli coefficient dict into a MOST31 polynomial."""
from __future__ import annotations

from f16.units import AEROBENCH_RTOD, DEG_TO_RAD
from most31.schema import MULTIPLIERS, OUTPUTS, Most31Coefficients, zeros

# Morelli alpha and beta are angle_rad * AEROBENCH_RTOD * pi/180.
_R = AEROBENCH_RTOD * DEG_TO_RAD

_CX = OUTPUTS.index("Cx")
_CY = OUTPUTS.index("Cy")
_CZ = OUTPUTS.index("Cz")
_CL = OUTPUTS.index("Cl")
_CM = OUTPUTS.index("Cm")
_CN = OUTPUTS.index("Cn")

_ONE = MULTIPLIERS.index("1")
_PHAT = MULTIPLIERS.index("phat")
_QHAT = MULTIPLIERS.index("qhat")
_RHAT = MULTIPLIERS.index("rhat")
_DA = MULTIPLIERS.index("da")
_DR = MULTIPLIERS.index("dr")

# (coefficient index, alpha power n, beta power j, elevator power k)
_CX0 = (
    (0, 0, 0, 0),
    (1, 1, 0, 0),
    (2, 0, 0, 2),
    (3, 0, 0, 1),
    (4, 1, 0, 1),
    (5, 2, 0, 0),
    (6, 3, 0, 0),
)
_CL0 = (
    (0, 0, 1, 0),
    (1, 1, 1, 0),
    (2, 2, 1, 0),
    (3, 0, 2, 0),
    (4, 1, 2, 0),
    (5, 3, 1, 0),
    (6, 4, 1, 0),
    (7, 2, 2, 0),
)
_CLDA = (
    (0, 0, 0, 0),
    (1, 1, 0, 0),
    (2, 0, 1, 0),
    (3, 2, 0, 0),
    (4, 1, 1, 0),
    (5, 2, 1, 0),
    (6, 3, 0, 0),
)
_CLDR = (
    (0, 0, 0, 0),
    (1, 1, 0, 0),
    (2, 0, 1, 0),
    (3, 1, 1, 0),
    (4, 2, 1, 0),
    (5, 3, 1, 0),
    (6, 0, 2, 0),
)
_CM0 = (
    (0, 0, 0, 0),
    (1, 1, 0, 0),
    (2, 0, 0, 1),
    (3, 1, 0, 1),
    (4, 0, 0, 2),
    (5, 2, 0, 1),
    (6, 0, 0, 3),
    (7, 1, 0, 2),
)
_CN0 = (
    (0, 0, 1, 0),
    (1, 1, 1, 0),
    (2, 0, 2, 0),
    (3, 1, 2, 0),
    (4, 2, 1, 0),
    (5, 2, 2, 0),
    (6, 3, 1, 0),
)
_CNDA = (
    (0, 0, 0, 0),
    (1, 1, 0, 0),
    (2, 0, 1, 0),
    (3, 1, 1, 0),
    (4, 2, 1, 0),
    (5, 3, 1, 0),
    (6, 2, 0, 0),
    (7, 3, 0, 0),
    (8, 0, 3, 0),
    (9, 1, 3, 0),
)
_CNDR = (
    (0, 0, 0, 0),
    (1, 1, 0, 0),
    (2, 0, 1, 0),
    (3, 1, 1, 0),
    (4, 2, 1, 0),
    (5, 2, 0, 0),
)


def translate_morelli(coeff: dict, source: str) -> Most31Coefficients:
    """Map every Morelli monomial onto ``poly[output, multiplier, n % 2, n, j, k]``."""
    out = zeros(source)
    poly = out.poly
    _monomials(poly, _CX, _ONE, coeff["cx"], _CX0)
    _alpha_series(poly, _CX, _QHAT, coeff["cxq"])
    cy = coeff["cy"]
    _store(poly, _CY, _ONE, 0, 1, 0, cy[0])
    _store(poly, _CY, _DA, 0, 0, 0, cy[1])
    _store(poly, _CY, _DR, 0, 0, 0, cy[2])
    _alpha_series(poly, _CY, _PHAT, coeff["cyp"])
    _alpha_series(poly, _CY, _RHAT, coeff["cyr"])
    _cz0(poly, coeff["cz"])
    _alpha_series(poly, _CZ, _QHAT, coeff["czq"])
    _monomials(poly, _CL, _ONE, coeff["cl"], _CL0)
    _alpha_series(poly, _CL, _PHAT, coeff["clp"])
    _alpha_series(poly, _CL, _RHAT, coeff["clr"])
    _monomials(poly, _CL, _DA, coeff["clda"], _CLDA)
    _monomials(poly, _CL, _DR, coeff["cldr"], _CLDR)
    _monomials(poly, _CM, _ONE, coeff["cm"], _CM0)
    _alpha_series(poly, _CM, _QHAT, coeff["cmq"])
    _monomials(poly, _CN, _ONE, coeff["cn"], _CN0)
    _alpha_series(poly, _CN, _PHAT, coeff["cnp"])
    _alpha_series(poly, _CN, _RHAT, coeff["cnr"])
    _monomials(poly, _CN, _DA, coeff["cnda"], _CNDA)
    _monomials(poly, _CN, _DR, coeff["cndr"], _CNDR)
    return out


def _store(poly, output: int, multiplier: int, n: int, j: int, k: int, c: float) -> None:
    poly[output, multiplier, n % 2, n, j, k] = float(c) * _R ** (n + j)


def _monomials(poly, output: int, multiplier: int, values, terms) -> None:
    for index, n, j, k in terms:
        _store(poly, output, multiplier, n, j, k, values[index])


def _alpha_series(poly, output: int, multiplier: int, values) -> None:
    for n, c in enumerate(values):
        _store(poly, output, multiplier, n, 0, 0, c)


def _cz0(poly, cz) -> None:
    """``(sum cz[n] alpha^n) * (1 - beta^2) + cz[5] de``."""
    for n, c in enumerate(cz[:5]):
        _store(poly, _CZ, _ONE, n, 0, 0, c)
        _store(poly, _CZ, _ONE, n, 2, 0, -c)
    _store(poly, _CZ, _ONE, 0, 0, 1, cz[5])
