"""Minimal SISO card: unreduced state space plus the cancelled transfer function."""
from __future__ import annotations

import numpy as np
from scipy.signal import ss2tf, tf2zpk

from flightbench.analysis.bode import bode_response

# Cancel p with z when |p − z| is at most this fraction of max(1, |p|).
_CANCEL_REL = 1e-6


def siso_card(
    A: np.ndarray,
    b: np.ndarray,
    c: np.ndarray,
    d: float,
    *,
    input: str,
    output: str,
    states: tuple[str, ...],
) -> dict:
    """One open-loop channel as matrices, a cancelled monic G(s), and Bode samples.

    ``A`` is the caller's state matrix, copied as nested lists. Cancelled
    pole/zero pairs are removed from ``G(s)`` only. ``bode`` is
    ``bode_response`` of those unreduced matrices.
    """
    system = np.asarray(A, dtype=float)
    n = system.shape[0]
    column = _as_column(b, n)
    row = _as_row(c, n)
    feedthrough = float(d)

    num, den = ss2tf(system, column, row, feedthrough)
    zeros, poles, gain = tf2zpk(_structural_numerator(num), den)
    kept_poles, kept_zeros = _cancel(poles, zeros)

    if kept_zeros:
        num_poly = complex(gain) * np.atleast_1d(np.poly(kept_zeros))
    else:
        num_poly = np.array([complex(gain)])
    den_poly = np.atleast_1d(np.poly(kept_poles))
    den_poly = den_poly / den_poly[0]

    return {
        "input": input,
        "output": output,
        "states": [str(name) for name in states],
        "A": _matrix(system),
        "B": _matrix(column),
        "C": _matrix(row),
        "D": [[feedthrough]],
        "numerator": _real_coeffs(num_poly),
        "denominator": _real_coeffs(den_poly),
        "poles": _pairs(kept_poles),
        "zeros": _pairs(kept_zeros),
        "bode": bode_response(system, column, row, feedthrough),
    }


def _as_column(vector: np.ndarray, n: int) -> np.ndarray:
    array = np.asarray(vector, dtype=float).reshape(-1)
    if array.size != n:
        raise ValueError(f"b has {array.size} entries, expected {n}")
    return array.reshape(n, 1)


def _as_row(vector: np.ndarray, n: int) -> np.ndarray:
    array = np.asarray(vector, dtype=float).reshape(-1)
    if array.size != n:
        raise ValueError(f"c has {array.size} entries, expected {n}")
    return array.reshape(1, n)


def _structural_numerator(num) -> np.ndarray:
    """Drop the degree-n padding ``ss2tf`` leaves on a strictly proper channel.

    Those entries are exact zeros or a few ulps. Leaving them in makes
    ``tf2zpk`` treat the numerator as badly conditioned.
    """
    coeffs = np.ravel(np.asarray(num, dtype=float))
    if coeffs.size == 0:
        return np.array([0.0])
    scale = float(np.max(np.abs(coeffs)))
    if scale == 0.0:
        return np.array([0.0])
    tol = 1e3 * float(np.finfo(float).eps) * scale
    start = 0
    while start < coeffs.size - 1 and abs(float(coeffs[start])) <= tol:
        start += 1
    return coeffs[start:]


def _cancel(poles, zeros) -> tuple[list[complex], list[complex]]:
    """Drop pairs with ``|p − z| ≤ 1e-6 * max(1, |p|)``, closest pair first."""
    pole_list = [complex(pole) for pole in np.ravel(np.asarray(poles, dtype=complex))]
    zero_list = [complex(zero) for zero in np.ravel(np.asarray(zeros, dtype=complex))]
    candidates: list[tuple[float, int, int]] = []
    for zero_index, zero in enumerate(zero_list):
        for pole_index, pole in enumerate(pole_list):
            distance = abs(pole - zero)
            limit = _CANCEL_REL * max(1.0, abs(pole))
            if distance <= limit:
                candidates.append((distance, zero_index, pole_index))
    candidates.sort()
    used_poles: set[int] = set()
    used_zeros: set[int] = set()
    for _distance, zero_index, pole_index in candidates:
        if zero_index in used_zeros or pole_index in used_poles:
            continue
        used_zeros.add(zero_index)
        used_poles.add(pole_index)
    kept_poles = [pole for index, pole in enumerate(pole_list) if index not in used_poles]
    kept_zeros = [zero for index, zero in enumerate(zero_list) if index not in used_zeros]
    return kept_poles, kept_zeros


def _pairs(values) -> list[dict]:
    ordered = sorted(values, key=lambda value: (-value.real, -value.imag))
    return [{"real": float(value.real), "imag": float(value.imag)} for value in ordered]


def _real_coeffs(coeffs) -> list[float]:
    values = np.atleast_1d(np.asarray(coeffs, dtype=complex))
    scale = 1.0 if values.size == 0 else max(1.0, float(np.max(np.abs(values))))
    if np.any(np.abs(values.imag) > 1e-8 * scale):
        raise ValueError("transfer-function coefficients are not real")
    return [float(value.real) for value in values]


def _matrix(array: np.ndarray) -> list[list[float]]:
    return [[float(value) for value in row] for row in np.asarray(array, dtype=float)]
