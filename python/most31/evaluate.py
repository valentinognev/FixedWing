"""Evaluate MOST31 coefficients to body-axis force and moment totals."""
from __future__ import annotations

import numpy as np

from most31.schema import Most31Coefficients


def _powers(base: float, count: int) -> np.ndarray:
    """``base**0 .. base**(count-1)`` with ``0**0 = 1``."""
    powers = np.empty(count, dtype=np.float64)
    powers[0] = 1.0
    if count > 1:
        np.multiply.accumulate(np.full(count - 1, base, dtype=np.float64), out=powers[1:])
    return powers


def _segment(grid: np.ndarray, x: float) -> tuple[int, float]:
    """End-segment index and unclipped weight (extrapolates outside the grid)."""
    i = int(np.clip(np.searchsorted(grid, x, "right") - 1, 0, len(grid) - 2))
    w = (x - grid[i]) / (grid[i + 1] - grid[i])
    return i, w


def _interp_alpha(table: np.ndarray, index: int, weight: float) -> np.ndarray:
    return (1.0 - weight) * table[:, :, index, :] + weight * table[:, :, index + 1, :]


def _interp_second(samples: np.ndarray, index: int, weight: float) -> np.ndarray:
    return (1.0 - weight) * samples[:, :, index] + weight * samples[:, :, index + 1]


def nine_outputs(
    c: Most31Coefficients,
    alpha: float,
    beta: float,
    de: float,
    da: float,
    dr: float,
    dc: float,
    p: float,
    q: float,
    r: float,
    V: float,
    b: float,
    cbar: float,
) -> np.ndarray:
    """Nine coefficients in ``OUTPUTS`` order, before CG shift and wind rotation."""
    if V <= 0.0:
        raise ValueError("V must be positive")
    inv_two_v = 1.0 / (2.0 * V)
    multipliers = np.array(
        (
            1.0,
            p * b * inv_two_v,
            q * cbar * inv_two_v,
            r * b * inv_two_v,
            da,
            dr,
            dc,
        ),
        dtype=np.float64,
    )
    # sgn(0)**0 = 1 and sgn(0)**1 = 0; |alpha|**0 = 1, including at alpha = 0.
    s_pow = _powers(float(np.sign(alpha)), 2)
    n_pow = _powers(abs(float(alpha)), 13)
    beta_pow = _powers(float(beta), 4)
    de_pow = _powers(float(de), 4)
    basis = (
        s_pow[:, None, None, None]
        * n_pow[None, :, None, None]
        * beta_pow[None, None, :, None]
        * de_pow[None, None, None, :]
    )
    poly_part = c.poly.reshape(9, 7, -1) @ basis.reshape(-1)

    ia, wa = _segment(c.grids["alpha_rad"], alpha)
    table_a = (1.0 - wa) * c.table_a[:, :, :, ia] + wa * c.table_a[:, :, :, ia + 1]
    t_a = table_a @ beta_pow
    along_ae = _interp_alpha(c.table_ae, ia, wa)
    along_ab = _interp_alpha(c.table_ab, ia, wa)
    along_aabsb = _interp_alpha(c.table_aabsb, ia, wa)
    ie, we = _segment(c.grids["elevator_rad"], de)
    ib, wb = _segment(c.grids["beta_rad"], beta)
    iabs, wabs = _segment(c.grids["abs_beta_rad"], abs(float(beta)))
    t_ae = _interp_second(along_ae, ie, we)
    t_ab = _interp_second(along_ab, ib, wb)
    t_aabsb = _interp_second(along_aabsb, iabs, wabs) * np.sign(beta)
    return (poly_part + t_a + t_ae + t_ab + t_aabsb) @ multipliers


def evaluate(
    c: Most31Coefficients,
    alpha: float,
    beta: float,
    de: float,
    da: float,
    dr: float,
    dc: float,
    p: float,
    q: float,
    r: float,
    V: float,
    b: float,
    cbar: float,
    xcg: float,
    xcgref: float,
) -> tuple[float, float, float, float, float, float]:
    """Body totals ``(CX, CY, CZ, Cl, Cm, Cn)`` after CG shift and wind rotation."""
    cx, cy, cz, cl, cm, cn, lift, drag, side = nine_outputs(
        c, alpha, beta, de, da, dr, dc, p, q, r, V, b, cbar
    )
    shift = xcgref - xcg
    cm = cm + cz * shift
    cn = cn - cy * shift * cbar / b
    sa = np.sin(alpha)
    ca = np.cos(alpha)
    sb = np.sin(beta)
    cb = np.cos(beta)
    cx_w = lift * sa - side * sb - drag * ca * cb
    cy_w = side * cb - drag * ca * sb
    cz_w = -lift * ca - drag * sa
    return (
        float(cx + cx_w),
        float(cy + cy_w),
        float(cz + cz_w),
        float(cl),
        float(cm),
        float(cn),
    )
