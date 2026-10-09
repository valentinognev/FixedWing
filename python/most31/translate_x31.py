"""Translate an X-31 Figure 2.2 coefficient table into MOST31."""
from __future__ import annotations

import numpy as np

from most31.schema import MULTIPLIERS, OUTPUTS, Most31Coefficients, zeros

# Per-degree polynomial powers and degree surface deflections become radians.
_K = 180.0 / np.pi
_DEGREE = 13


def translate_x31(table: np.ndarray, source: str) -> Most31Coefficients:
    """Map a `(10, 2, 8)` Figure 2.2 table onto MOST31 polynomial slots.

    Flags other than 0 (even in alpha) or 1 (odd in alpha) raise ``ValueError``.
    """
    rows = np.asarray(table, dtype=np.float64)
    if rows.shape != (10, 2, 8):
        raise ValueError(f"table: expected shape (10, 2, 8), got {rows.shape}")
    if np.any((rows[:, :, 7] != 0.0) & (rows[:, :, 7] != 1.0)):
        raise ValueError("Figure 2.2 odd/even flag must be 0 or 1")

    function = [[_alpha_function(rows[row, col]) for col in range(2)] for row in range(10)]
    cldc = function[1][0]
    lift_offset = function[1][1]
    cmdc = function[9][0]
    moment_offset = function[9][1]

    coefficients = zeros(source)
    poly = coefficients.poly
    _store(poly, "CL", "1", function[0][0] - _product(cldc, lift_offset))
    _store(poly, "CD", "1", function[0][1])
    _store(poly, "CL", "dc", cldc * _K)
    _store(poly, "CY", "1", function[2][0], beta_power=1)
    _store(poly, "CY", "dr", function[2][1] * _K)
    _store(poly, "Cl", "1", function[3][0], beta_power=1)
    _store(poly, "Cn", "1", function[3][1], beta_power=1)
    _store(poly, "Cl", "phat", function[4][0])
    _store(poly, "Cn", "phat", function[4][1])
    _store(poly, "Cl", "rhat", function[5][0])
    _store(poly, "Cn", "rhat", function[5][1])
    _store(poly, "Cl", "da", function[6][0] * _K)
    _store(poly, "Cn", "da", function[6][1] * _K)
    _store(poly, "Cl", "dr", function[7][0] * _K)
    _store(poly, "Cn", "dr", function[7][1] * _K)
    _store(poly, "Cm", "1", function[8][0] - _product(cmdc, moment_offset))
    _store(poly, "Cm", "qhat", function[8][1])
    _store(poly, "Cm", "dc", cmdc * _K)
    return coefficients


def _store(
    poly: np.ndarray,
    output: str,
    multiplier: str,
    values: np.ndarray,
    beta_power: int = 0,
) -> None:
    poly[OUTPUTS.index(output), MULTIPLIERS.index(multiplier), :, :, beta_power, 0] = values


def _alpha_function(row: np.ndarray) -> np.ndarray:
    """Even/odd parts of one port row, powers of radians through degree 6."""
    values = np.zeros((2, _DEGREE), dtype=np.float64)
    values[0, 0] = row[6]
    part = 1 if row[7] == 1.0 else 0
    for degree in range(1, 7):
        values[part, degree] = row[degree - 1] * _K**degree
    return values


def _product(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Product of two sign-split alpha functions, kept through degree 12."""
    even = np.convolve(left[0], right[0]) + np.convolve(left[1], right[1])
    odd = np.convolve(left[0], right[1]) + np.convolve(left[1], right[0])
    assert np.all(even[_DEGREE:] == 0.0) and np.all(odd[_DEGREE:] == 0.0)
    out = np.empty((2, _DEGREE), dtype=np.float64)
    out[0] = even[:_DEGREE]
    out[1] = odd[:_DEGREE]
    return out
