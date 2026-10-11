"""Root-locus grids and nearest-eigenvalue branches.

An ordinary gain sweeps the affine matrix ``M(v) = M(0) + v·(M(1) − M(0))``.
Branches are the eigenvalues of that sweep, joined greedily in the complex plane.
"""

from __future__ import annotations

import numpy as np


def affine_grid(k: float) -> np.ndarray:
    """Unique ascending sweep: ``0``, ``k``, and 80 decades around ``max(|k|, 1)``.

    A negative ``k`` adds the same logarithmic grid on the negative side.
    """
    scale = max(abs(k), 1.0)
    positive = np.logspace(-3.0, 3.0, 80) * scale
    parts = [np.array([0.0, float(k)]), positive]
    if k < 0.0:
        parts.append(-positive)
    return np.unique(np.concatenate(parts))


def parameter_grid(k: float, *, skip: float | None = None) -> np.ndarray:
    """Unique ascending sweep from ``0.05·k`` to ``20·k``, always including ``k``.

    ``k`` is positive. A value equal to ``skip`` is removed when ``skip`` is set.
    """
    span = np.geomspace(0.05 * k, 20.0 * k, 80)
    grid = np.unique(np.concatenate([span, np.array([float(k)])]))
    if skip is not None:
        grid = grid[grid != skip]
    return grid


def _point(z: complex) -> dict:
    return {"real": float(np.real(z)), "imag": float(np.imag(z))}


def join_branches(eigs: np.ndarray) -> list[list[dict]]:
    """Join ``eigs`` of shape ``(n_values, n_modes)`` into locus branches.

    Column 0 orders the branches by increasing real part, then increasing
    imaginary part. Each later column gives every branch, in that order, the
    unmatched eigenvalue closest in the complex plane. A distance tie takes the
    smaller real part, then the smaller imaginary part.
    """
    spectrum = np.asarray(eigs, dtype=complex)
    n_values, n_modes = spectrum.shape
    if n_values == 0:
        return [[] for _ in range(n_modes)]

    first = spectrum[0]
    order = sorted(range(n_modes), key=lambda i: (first[i].real, first[i].imag))
    branches: list[list[dict]] = [[_point(first[i])] for i in order]
    previous = [first[i] for i in order]

    for col in range(1, n_values):
        column = spectrum[col]
        unmatched = list(range(n_modes))
        followed: list[complex] = []
        for branch, src in zip(branches, previous):
            best = min(
                unmatched,
                key=lambda j, src=src: (
                    (column[j].real - src.real) ** 2 + (column[j].imag - src.imag) ** 2,
                    column[j].real,
                    column[j].imag,
                ),
            )
            unmatched.remove(best)
            chosen = column[best]
            branch.append(_point(chosen))
            followed.append(chosen)
        previous = followed
    return branches


def locus_from_eigs(values: np.ndarray, eigs: np.ndarray) -> dict:
    """Drop samples with a non-finite entry, then pair the rest into branches."""
    samples = np.asarray(values, dtype=float).reshape(-1)
    spectrum = np.asarray(eigs, dtype=complex)
    keep = np.isfinite(samples) & np.isfinite(spectrum).all(axis=1)
    kept = samples[keep]
    return {
        "values": [float(v) for v in kept],
        "branches": join_branches(spectrum[keep]),
    }


def affine_locus(m0: np.ndarray, m1: np.ndarray, k: float) -> dict:
    """Eigenvalues of ``m0 + v·(m1 − m0)`` on ``affine_grid(k)``."""
    grid = affine_grid(k)
    base = np.asarray(m0, dtype=float)
    delta = np.asarray(m1, dtype=float) - base
    spectrum = np.empty((grid.size, base.shape[0]), dtype=complex)
    for index, value in enumerate(grid):
        spectrum[index] = np.linalg.eigvals(base + value * delta)
    return locus_from_eigs(grid, spectrum)
