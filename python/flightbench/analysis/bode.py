"""Bode samples of G(jω)."""
from __future__ import annotations

import math

import numpy as np

from flightbench.analysis.nyquist import NYQUIST_OMEGA


def bode_response(
    A: np.ndarray,
    b: np.ndarray,
    c: np.ndarray,
    d: float,
    omega: np.ndarray | None = None,
) -> dict:
    """Samples of ``G(jω) = c (jωI − A)^{-1} b + d``.

    One ``numpy.linalg.solve`` per finite frequency. A singular solve, a
    non-finite ``G``, or ``|G| == 0`` is omitted. Magnitude is
    ``20 * log10(|G|)``. Phase is ``degrees(atan2(imag, real))`` wrapped
    into ``(-180, 180]``. ``omega``, ``magnitude_db``, and ``phase_deg``
    have equal length.
    """
    if omega is None:
        omega = NYQUIST_OMEGA
    matrix = np.asarray(A, dtype=float)
    input_vec = np.asarray(b, dtype=float).reshape(-1)
    output_vec = np.asarray(c, dtype=float).reshape(-1)
    identity = np.eye(matrix.shape[0], dtype=float)
    kept_omega: list[float] = []
    kept_magnitude: list[float] = []
    kept_phase: list[float] = []
    for frequency in np.asarray(omega, dtype=float).reshape(-1):
        if not np.isfinite(frequency):
            continue
        try:
            state = np.linalg.solve(1j * float(frequency) * identity - matrix, input_vec)
        except np.linalg.LinAlgError:
            continue
        value = output_vec @ state + d
        modulus = abs(value)
        if not np.isfinite(value) or modulus == 0.0:
            continue
        magnitude = 20.0 * math.log10(float(modulus))
        phase = math.degrees(math.atan2(float(np.imag(value)), float(np.real(value))))
        if phase <= -180.0:
            phase += 360.0
        kept_omega.append(float(frequency))
        kept_magnitude.append(float(magnitude))
        kept_phase.append(float(phase))
    return {
        "omega": kept_omega,
        "magnitude_db": kept_magnitude,
        "phase_deg": kept_phase,
    }
