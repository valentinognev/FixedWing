"""Frequency response of the return ratio L(jω)."""
from __future__ import annotations

import numpy as np

NYQUIST_OMEGA = np.logspace(-2, 2, 400)
NYQUIST_CAP = 1e6


def nyquist(
    A: np.ndarray,
    b: np.ndarray,
    c: np.ndarray,
    d: float,
    gain_c: float,
    omega: np.ndarray | None = None,
) -> dict:
    """Samples of ``L(jω) = -gain_c * (c (jωI − A)^{-1} b + d)``.

    One ``numpy.linalg.solve`` per finite frequency. A sample that is
    non-finite, whose modulus exceeds ``NYQUIST_CAP``, or whose resolvent
    is singular, is omitted. ``omega``, ``real``, and ``imag`` have equal length.
    """
    if omega is None:
        omega = NYQUIST_OMEGA
    matrix = np.asarray(A, dtype=float)
    input_vec = np.asarray(b, dtype=float).reshape(-1)
    output_vec = np.asarray(c, dtype=float).reshape(-1)
    identity = np.eye(matrix.shape[0], dtype=float)
    kept_omega: list[float] = []
    kept_real: list[float] = []
    kept_imag: list[float] = []
    for frequency in np.asarray(omega, dtype=float).reshape(-1):
        if not np.isfinite(frequency):
            continue
        try:
            state = np.linalg.solve(1j * float(frequency) * identity - matrix, input_vec)
        except np.linalg.LinAlgError:
            continue
        value = -float(gain_c) * (output_vec @ state + d)
        if not np.isfinite(value) or abs(value) > NYQUIST_CAP:
            continue
        kept_omega.append(float(frequency))
        kept_real.append(float(np.real(value)))
        kept_imag.append(float(np.imag(value)))
    return {"omega": kept_omega, "real": kept_real, "imag": kept_imag}
