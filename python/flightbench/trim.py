"""Wings-level trim: the steady point every bench run starts from.

The trim solves the longitudinal balance only. The other states and channels
are not free parameters but trim constraints: beta = phi = p = q = r = 0,
theta = alpha, roll and yaw at their zero deflection, the plane extras at their
equilibrium for the solved throttle. That leaves three unknowns -- angle of
attack, pitch channel, throttle -- against the three residuals (vt_dot,
alpha_dot, q_dot) of the common derivative, so the pilot who asks for one speed
at one altitude gets one state back and the sign conventions stay the
adapter's business.

The solver tolerances are scipy's defaults (``ftol = xtol = gtol = 1e-8``).
The three unknowns are well scaled: an angle in radians, a surface in radians
and a throttle fraction each move their own residual by many orders of
magnitude more than the residual itself, so the finite-difference Jacobian is
never the limit and the design's 1e-8 acceptance is reached on every plane.
The X-31's common derivative is a central difference of the common state whose
rounding floor is around 1e-10, two orders under that acceptance, so a tighter
tolerance would buy nothing. The initial guess is documented in the plan and
starts inside every plane's channel box, so the X-31's asymmetric canard limits
(-90 deg to +30 deg) cost the trust-region solver nothing.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares

from flightbench.common import (
    ALPHA,
    ALT,
    FlightbenchError,
    PITCH,
    PlantStop,
    Q,
    THETA,
    THROTTLE,
    TrimError,
    TrimPoint,
    VT,
)
from flightbench.adapters.base import PlaneAdapter

# Unknowns, in the order the solver sees them: angle of attack, pitch channel,
# throttle. Everything else about the state and the controls is fixed by the
# trim constraints above, so the problem is three equations in three unknowns.
_ALPHA0 = 0.05
_THROTTLE0 = 0.3

# Acceptance: the design fails the trim when any residual is not effectively
# zero, checked per component so no single balance can hide behind the others.
_RESIDUAL_TOL = 1e-8
_SOLVER_TOL = 1e-8
_MAX_NFEV = 200


def _pitch_zero(adapter: PlaneAdapter) -> float:
    """Neutral pitch: 0 when mid-stick is legal, else the middle of the range."""
    lo, hi = (float(v) for v in np.asarray(adapter.limits)[PITCH])
    if lo <= 0.0 <= hi:
        return 0.0
    return 0.5 * (lo + hi)


def _trim_state(adapter: PlaneAdapter, vt_mps: float, altitude_m: float,
                alpha: float, throttle: float) -> np.ndarray:
    """The common state a candidate trim (alpha, pitch, throttle) implies."""
    x = np.zeros(12 + len(adapter.extras), dtype=float)
    x[VT] = vt_mps
    x[ALPHA] = alpha
    x[THETA] = alpha  # gamma = 0: the flight path is the body x-axis
    x[ALT] = altitude_m
    x[12:] = adapter.extras_equilibrium(throttle)
    return x


def _trim_controls(throttle: float, pitch: float) -> np.ndarray:
    """Wings level and no rudder: only the two solved channels deflect."""
    u = np.zeros(4, dtype=float)
    u[THROTTLE] = throttle
    u[PITCH] = pitch
    return u


def trim_level(adapter: PlaneAdapter, vt_mps: float, altitude_m: float) -> TrimPoint:
    """Wings-level trim at one speed and altitude.

    Solves (alpha, pitch, throttle) against (vt_dot, alpha_dot, q_dot) with
    ``scipy.optimize.least_squares`` over the adapter's common derivative, with
    beta = phi = p = q = r = 0, theta = alpha, roll and yaw at zero and the
    plane extras at their equilibrium for the throttle.

    Raises ``FlightbenchError`` for a condition that is not a flight condition
    at all (non-finite, non-positive speed, negative altitude) and
    ``TrimError`` when the solver cannot reach a steady balance or the answer
    needs a channel beyond the adapter's limits.
    """
    try:
        vt = float(vt_mps)
        alt = float(altitude_m)
    except (TypeError, ValueError):
        raise FlightbenchError(
            f"trim condition must be numeric, got vt_mps={vt_mps!r} altitude_m={altitude_m!r}"
        ) from None
    if not np.isfinite(vt) or vt <= 0.0:
        raise FlightbenchError(f"vt_mps must be positive and finite, got {vt}")
    if not np.isfinite(alt) or alt < 0.0:
        raise FlightbenchError(f"altitude_m must be non-negative and finite, got {alt}")

    def state_of(z: np.ndarray) -> np.ndarray:
        alpha, _, throttle = z
        return _trim_state(adapter, vt, alt, alpha, throttle)

    def residuals(z: np.ndarray) -> np.ndarray:
        alpha, pitch, throttle = z
        try:
            xd = adapter.derivative(
                _trim_state(adapter, vt, alt, alpha, throttle),
                _trim_controls(throttle, pitch),
            )
        except PlantStop:
            # A candidate the plant cannot fly at all (ground, angle limit,
            # non-finite) is not a zero of the balance: report it as one and
            # let the acceptance check below turn it into a TrimError.
            return np.full(3, np.nan)
        return np.array([xd[VT], xd[ALPHA], xd[Q]], dtype=float)

    z0 = np.array([_ALPHA0, _pitch_zero(adapter), _THROTTLE0], dtype=float)
    try:
        found = least_squares(residuals, z0, ftol=_SOLVER_TOL, xtol=_SOLVER_TOL,
                              gtol=_SOLVER_TOL, max_nfev=_MAX_NFEV)
    except ValueError as exc:
        # Residuals that are NaN from the start make scipy refuse to run; the
        # condition (a trim on the ground, say) is a failed trim, not a crash.
        raise TrimError(
            f"no wings-level trim at {vt} m/s, {alt} m: the solver could not run: {exc}"
        ) from None
    _, pitch, throttle = (float(v) for v in found.x)

    u = _trim_controls(throttle, pitch)
    x = state_of(found.x)

    lo = np.asarray(adapter.limits, dtype=float)[:, 0]
    hi = np.asarray(adapter.limits, dtype=float)[:, 1]
    outside = [f"{ch}={value!r} not in [{low!r}, {high!r}]"
               for ch, value, low, high in zip(("throttle", "pitch", "roll", "yaw"), u, lo, hi)
               if not low <= value <= high]
    if outside:
        raise TrimError(
            f"no wings-level trim at {vt} m/s, {alt} m: a channel is outside its limits: "
            + "; ".join(outside)
        )

    residual = residuals(found.x)
    worst = float(np.max(np.abs(residual))) if np.all(np.isfinite(residual)) else float("inf")
    if worst > _RESIDUAL_TOL:
        raise TrimError(
            f"no wings-level trim at {vt} m/s, {alt} m: residual component {worst:.3e} "
            f"exceeds the {_RESIDUAL_TOL:.0e} acceptance"
        )

    return TrimPoint(x=x, u=u, vt_mps=vt, altitude_m=alt)
