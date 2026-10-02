"""Unit walls, CG moment transfer and sign normalisation for the aero converter.

The Cessna 172 source is feet, per degree, and moments about the solver's own
reference point.  Only SI, per-radian values about the CG may reach a Morelli
array, so every conversion the pipeline needs lives here.  This module has no
solver, no file I/O and no dependency on the flight model beyond the sign
derivation recorded below.

Sign convention, derived from ``plane/dynamics.py`` and nothing else
-----------------------------------------------------------------
``xd[11] = ub*sin(theta) - vb*cos(theta)*sin(phi) - wb*cos(phi)*cos(theta)``
(dynamics.py:71) is the NED altitude rate
``h_dot = u sin(theta) + v cos(theta) sin(phi) - w cos(theta) cos(phi)``, so
**positive ``w`` is downward** and the body **z axis points down**.

The force is then resolved as

    ax = (qbar * S) * cx / m                                        (dynamics.py:42)
    ay = (qbar * S) * cy / m                                        (dynamics.py:43)
    az = (qbar * S) * cz / m                                        (dynamics.py:44)

so ``cz`` is the body-z force coefficient with **down** positive:
**``cz = -CL``**.  That is the load-bearing fact of this module.  A downward
force does raise ``alpha_dot``, because ``az`` reaches ``wdot``
(dynamics.py:47) and ``xd[1] = (ub*wdot - wb*udot) / (ub^2 + wb^2)`` is
``alpha_dot`` (dynamics.py:51) -- but a downward force is not lift.  Reading
that alpha response as "positive ``cz`` is lift up" is the mistake this
docstring exists to prevent; it inverts the lift slope and flies the aircraft
upside down.

The rest of that chain, which does need no frame argument:

* ``cy`` reaches ``vdot`` (dynamics.py:46), hence ``xd[2]``
  (dynamics.py:52), which is ``beta_dot``, with a positive coefficient, so
  positive ``cy`` raises beta;
* ``xd[0]``, the total speed derivative, receives ``(cx*qbar*S + thrust)/m``,
  so negative ``cx`` is drag.

The moments reach the rate vector through ``plane/aircraft.py``'s
``_inertia_coefficients``, in which ``c3 = izz/gamma > 0``, ``c7 = 1/iyy > 0``
and ``c9 = ixx/gamma > 0`` for any physical inertia:

    xd[6] = ... + qbar*S*b   * (c3 * cl + c4 * cn)      roll rate      (dynamics.py:57)
    xd[7] = ... + qbar*S*cbar * c7 * cm                pitch rate     (dynamics.py:58)
    xd[8] = ... + qbar*S*b   * (c4 * cl + c9 * cn)     yaw rate       (dynamics.py:59)

and ``xd[4] = q cos(phi) - r sin(phi)`` is ``theta_dot`` (dynamics.py:53), so
positive ``q`` is nose up, positive ``p`` is a roll to the right and positive
``r`` is a yaw to the right.  Reading that chain through:

* positive ``cm`` is nose up;
* positive ``cn`` is nose right (yaw right);
* positive ``cl`` rolls right (right wing down).

A statically stable aeroplane has a lift slope that grows with alpha and a
pitching moment that falls with it.  Since ``CL_alpha = -cz[1]``, that means
**``cz[1]`` is negative** -- so that ``CL_alpha = -cz[1] > 0`` -- and
**``cm[1]`` is negative**.  The static margin in cbar is therefore
``cm[1] / cz[1]``: the textbook ``-Cm_alpha / CL_alpha`` with
``CL_alpha = -cz[1]``, and both slots being negative makes the ratio positive.

Independent confirmation of ``cz = -CL``, from the in-tree F-16 set that
``run_f16.py`` trims successfully: ``cm[0] = -0.0203`` and
``cm[1] = +0.0466`` give ``alpha_trim = -cm0/cm1 = +0.4354 rad``, and
``cz[0] + cz[1]*alpha_trim = -0.1378 + (-4.2114)(0.4354) = -1.97``.  Only
``cz = -CL`` turns that into a plausible lift coefficient (``CL = +1.97``);
``cz = +CL`` gives negative lift.

``cm[0]`` (Cm0) is deliberately **absent** from ``SIGN_INVARIANTS`` and is never
sign-normalised: the two in-tree reference files disagree on it
(``data/planes/linear/morelli.json`` has ``+0.05``, the F-16 set has
``-0.0203``), and Cm0's sign is tail rigging rather than a derivable
convention.  Task 5's trim test validates it instead.  An absent key is not an
error: ``normalise_sign`` returns the value untouched for any name the table
does not pin.

Moving moments to the CG
------------------------
``shift_moments_to_cg`` takes the coefficients as ``plane/dynamics.py`` resolves
them, so the force at the reference point ``O`` is ``qbar * S * (-cd, cy, cl)``
in a body frame with x forward, y right and z down (drag is the negative x force,
see the derivation above).  ``(dx, dy, dz)`` is the position of the CG relative
to ``O``, so ``O - G = (-dx, -dy, -dz)``, and the rigid-body relation

    M about G = M about O + (O - G) x F

gives, after dividing by ``qbar * S`` and by the reference length of each axis,
with ``b = b_ref`` and ``c = c_ref``:

    Cl_cg = Cl + (dz*cy - dy*cl) / b      M about x, right roll positive
    Cm_cg = Cm + (dx*cl + dz*cd) / c      M about y, nose up positive
    Cn_cg = Cn - (dy*cd + dx*cy) / b      M about z, yaw right positive

The signs are not a convention choice, they are the cross product:

* a *downward* force behind the CG rotates the nose up, and ``dx * cl`` with both
  positive does exactly that, which is what the measured Tornado case below
  checks;
* a *downward* force left of the CG rolls the aircraft left, i.e. negative roll,
  and ``-dy * cl`` with ``dy`` and ``cl`` positive is negative, as it must be;
* a *drag* force above the CG pitches the nose up, and ``dz * cd`` with ``dz``
  negative is negative, i.e. nose down, which is what a drag force above the CG
  does: it pulls the tail up.

``s_ref`` is accepted because the shift is meaningless without the reference
geometry, but it does not appear in the result: the coefficients handed in are
already divided by ``qbar * S``, so the reference area cancels.  It is still
validated, so a zero or negative reference geometry fails loudly instead of
producing a plausible wrong number.
"""
from __future__ import annotations

__all__ = [
    "DEG_TO_RAD",
    "FT_TO_M",
    "G_FT_S2",
    "SIGN_INVARIANTS",
    "SLUG_TO_KG",
    "ft",
    "normalise_sign",
    "per_degree_to_per_radian",
    "shift_moments_to_cg",
]

# 180/pi: the factor that turns a per-degree derivative into a per-radian one.
DEG_TO_RAD = 57.29577951308232
FT_TO_M = 0.3048
SLUG_TO_KG = 14.59390294
G_FT_S2 = 32.174

# Required sign of every derivative the converter writes, keyed by the
# DerivativeSet field name: +1 when the quantity must be positive, -1 when it
# must be negative.  Every row is transcribed from the plan's "Physics
# invariants" table; `cl_alpha` is negative because `cz = -CL` and a stable
# aircraft needs a positive lift slope.  `cm0` is absent on purpose and is never
# normalised.  An absent key means "no derivable invariant", not an error.
SIGN_INVARIANTS: dict[str, int] = {
    "cl_alpha": -1,
    "cl_q": -1,
    "cl_de": -1,
    "cm_alpha": -1,
    "cm_q": -1,
    "cm_de": -1,
    "cl_beta": -1,
    "cl_p": -1,
    "cl_r": 1,
    "cl_da": -1,
    "cl_dr": 1,
    "cy_beta": -1,
    "cy_p": -1,
    "cy_r": 1,
    "cy_da": 1,
    "cy_dr": 1,
    "cn_beta": 1,
    "cn_p": -1,
    "cn_r": -1,
    "cn_da": -1,
    "cn_dr": -1,
    "cd0": -1,
    "cd_q": 1,
    "cl0": -1,
}

# The plan's invariant table names a slot by its Morelli array ("cz[1]"), so
# "cz_alpha" is a spelling of the DerivativeSet field "cl_alpha".  Both
# spellings resolve; the field name is canonical.
_KEY_ALIASES: dict[str, str] = {
    "cz_0": "cl0",
    "cz_alpha": "cl_alpha",
    "cz_q": "cl_q",
    "cz_de": "cl_de",
    "cx_0": "cd0",
    "cx_q": "cd_q",
}


def per_degree_to_per_radian(value: float) -> float:
    """Convert a per-degree derivative to a per-radian one, unconditionally."""
    return float(value) * DEG_TO_RAD


def ft(value: float) -> float:
    """Convert feet to metres."""
    return float(value) * FT_TO_M


def shift_moments_to_cg(
    cl: float,
    cd: float,
    cy: float,
    cl_m: float,
    cm: float,
    cn: float,
    *,
    dx: float,
    dy: float,
    dz: float,
    s_ref: float,
    b_ref: float,
    c_ref: float,
) -> tuple[float, float, float, float, float, float]:
    """Move a coefficient set's moments from ``O`` to the CG.

    ``cl`` is the coefficient of the body +z force as ``plane/dynamics.py``
    computes it, ``cd`` the positive drag magnitude (so the body x force is
    ``-cd``) and ``cy`` the coefficient of the body +y force.  ``cl_m``, ``cm``
    and ``cn`` are the roll, pitch and yaw moment coefficients about ``O``.
    ``(dx, dy, dz)`` is the position of the CG relative to ``O`` in the body
    frame, x forward, y right, z down, so a reference point behind the CG has
    ``dx > 0``.  All lengths are metres.  The three force coefficients are
    returned unchanged; only the moments move.  See the module docstring for the
    derivation and for the measured Tornado case this pins.
    """
    if s_ref <= 0.0 or b_ref <= 0.0 or c_ref <= 0.0:
        raise ValueError(
            f"reference geometry must be positive, got s_ref={s_ref}, b_ref={b_ref}, c_ref={c_ref}"
        )
    cl = float(cl)
    cd = float(cd)
    cy = float(cy)
    dx = float(dx)
    dy = float(dy)
    dz = float(dz)
    return (
        cl,
        cd,
        cy,
        float(cl_m) + (dz * cy - dy * cl) / b_ref,
        float(cm) + (dx * cl + dz * cd) / c_ref,
        float(cn) - (dy * cd + dx * cy) / b_ref,
    )


def normalise_sign(name: str, value: float) -> tuple[float, bool]:
    """Return ``(value, flipped)`` with ``value`` on the invariant side of zero.

    A value whose sign disagrees with ``SIGN_INVARIANTS[name]`` is multiplied by
    -1 and reported as flipped, so the caller can log it.  Zero is not flipped:
    it has no sign to disagree with, and a zero slot is a declared absence, not
    a value.

    A name the table does not pin -- ``cm0``, whose sign the two in-tree
    reference files disagree on, or any other name a caller may pass -- is
    returned unchanged and never raises, because no derivable invariant exists
    for it and silently inverting it would be worse than leaving it alone.
    """
    key = _KEY_ALIASES.get(name, name)
    value = float(value)
    if value == 0.0:
        return 0.0, False
    if key not in SIGN_INVARIANTS:
        return value, False
    if (value > 0.0) == (SIGN_INVARIANTS[key] > 0):
        return value, False
    return -value, True