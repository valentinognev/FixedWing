"""ISA atmosphere for the host airplane, using the F-16 constants."""
from __future__ import annotations

from math import sqrt

from f16.units import H_STRAT_M, LAPSE_PER_M, R_AIR, RHO0_KG_M3, T0_K, T_STRAT_K


def air(vt_mps: float, alt_m: float) -> tuple[float, float, float]:
    """Return ``(mach, qbar_pa, rho)`` from true airspeed and geometric altitude."""
    if vt_mps <= 0 or alt_m < 0:
        raise ValueError("vt must be positive and altitude must be non-negative")
    tfac = 1 - LAPSE_PER_M * alt_m
    if alt_m >= H_STRAT_M:
        t = T_STRAT_K
    else:
        t = T0_K * tfac
    rho = RHO0_KG_M3 * tfac**4.14
    mach = vt_mps / sqrt(1.4 * R_AIR * t)
    qbar_pa = 0.5 * rho * vt_mps * vt_mps
    return mach, qbar_pa, rho
