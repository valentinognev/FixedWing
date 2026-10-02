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
convention.  Task 5's trim test validates it instead.  It is named explicitly in
``NO_INVARIANT`` so that an absent key is never silently treated as "no
invariant": ``normalise_sign`` normalises a name in ``SIGN_INVARIANTS``,
returns a name in ``NO_INVARIANT`` untouched, and raises ``KeyError`` for
anything else, so a misspelled slot fails at the call site.

Moving moments to the CG
------------------------
The three force coefficients are named exactly as ``plane/dynamics.py`` names
them, because a "lift"/"drag" spelling invites reading a drag magnitude where a
signed body force belongs.  Each argument is that file's variable divided by
``qbar * S``:

* ``cz`` -- the ``az`` coefficient.  It is the body **+z** force with **z down**,
  so it is down-positive and ``cz = -CL``: lift up is a negative ``cz``.
* ``cx`` -- the ``ax`` coefficient.  It is the body **+x** force, so drag is a
  negative ``cx``.
* ``cy`` -- the ``ay`` coefficient, the body **+y** force.

``(dx, dy, dz)`` is the position of the CG relative to the reference point ``O``,
in the same body frame, so ``O - G = (-dx, -dy, -dz)`` and the rigid-body
relation

    M about G = M about O + (O - G) x F

gives, after dividing by ``qbar * S`` and by the reference length of each axis,
with ``b = b_ref`` and ``c = c_ref``:

    Cl_cg = Cl + (dz*cy - dy*cz) / b      M about x, right roll positive
    Cm_cg = Cm + (dx*cz - dz*cx) / c      M about y, nose up positive
    Cn_cg = Cn + (dy*cx - dx*cy) / b      M about z, yaw right positive

Each moment component collects exactly the force components that have an arm
about its own axis -- roll (about x) from the y and z forces, pitch (about y)
from the x and z forces, yaw (about z) from the x and y forces -- which is why
``dz*cz``, ``dy*cz``'s counterpart in pitch, and the other impossible pairings
appear nowhere.  With the source geometry of the C172 that reads as follows, and
each bullet is pinned by a one-arm test:

* lift up (``cz < 0``) behind the CG (``dx > 0``) pitches the nose **down**, and
  ``dx*cz`` is then negative, as it must be;
* lift up left of the CG (``dy > 0``) rolls the aircraft **right**, and
  ``-dy*cz`` is then positive;
* a lift force cannot yaw or pitch about its own axis, so ``dy*cz`` is absent
  from the yaw line and a force along the pitch axis has no arm about that axis,
  so ``dz*cz`` is absent from the pitch line;
* **drag** (``cx < 0``) above the CG (``dz > 0``) pitches the nose **up**, and
  ``-dz*cx`` is then positive; below the CG (``dz < 0``) it pitches the nose
  down, because ``-dz*cx`` is negative;
* drag left of the CG (``dy > 0``) pulls the tail left, so the nose goes left,
  and ``dy*cx`` is then negative, as it must be.

The sign of the ``dx`` term comes from that cross product and from nothing else.
It is deliberately **not** cross-checked against ``f16/aero_morelli.py`` line 65
(``Cm = Cm0 + Cmq*qhat + Cz*(xcgref - xcg)``): that term is identically zero
everywhere in this repository, because every in-tree reference sets
``xcg == xcg_ref`` -- ``data/planes/linear/morelli.json`` lines 8-9,
``f16/model.py:164`` (``xcg = 0.35``) with ``:172`` (``xcgr = .35``),
``f16/aero_stevens.py:411``, and the four test fixtures
(``test_plane_aircraft.py:25``, ``test_plane_dynamics.py:44``,
``test_plane_model_select.py:24``, ``test_plane_sim.py:27``).  Nothing in this
repository documents Morelli's own x-axis direction for those station numbers, so
that expression cannot witness a convention here.  The unit tests are what pin
it, one arm at a time.

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
    "NO_INVARIANT",
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
# aircraft needs a positive lift slope.  `cm0` is deliberately not here; see
# NO_INVARIANT below.  A name in neither table is a caller error, not a licence
# to pass a value through, so normalise_sign raises for it.
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

# Derivative fields that deliberately carry no sign invariant.  `cm0` (Cm0) is
# absent from the plan's "Physics invariants" table: the two in-tree reference
# files disagree on it (`data/planes/linear/morelli.json` has +0.05, the F-16 set
# has -0.0203) and Cm0's sign is tail rigging, not a derivable convention.  These
# names are listed so a misspelling is still an error; the value passes through.
NO_INVARIANT: frozenset[str] = frozenset({"cm0"})

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
    cz: float,
    cx: float,
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

    ``r = CG - O`` is expressed in the ``python/plane/dynamics.py`` frame:
    **x forward, y right, z down**, and all three of ``dx``, ``dy``, ``dz`` are metres.
    The three force coefficients are that file's variables divided by ``qbar * S``
    -- ``cz`` the ``az`` coefficient (body +z, down, so ``cz = -CL``), ``cx`` the
    ``ax`` coefficient (body +x, so drag is a negative ``cx``), ``cy`` the ``ay``
    coefficient -- and ``cl_m``, ``cm``, ``cn`` are the roll, pitch and yaw moment
    coefficients about ``O``.  ``b_ref`` and ``c_ref`` are the reference span and
    mean chord in metres; ``s_ref`` is the reference area in square metres, is
    validated, and cancels out of the result (the coefficients are already divided
    by ``qbar * S``).  The three force coefficients are returned unchanged, in the
    order they came in; only the moments move.  The shift is exact for a coefficient
    set that is linear in alpha and it applies unchanged to the **slopes**, because
    differentiating the shift differentiates the same expression -- which is how a
    ``Cm_alpha`` reported about a non-CG reference is corrected, with the lift slope
    passed as ``cz``.

    **The Cessna 172 source frame is x AFT positive; this function's is x FORWARD
    positive.**  The source file's stations are in feet from the nose with x growing
    aft -- ``AERO.XW = 2.2`` (wing), ``AERO.XCG = 2.94`` (CG), ``AERO.XH = 8.75``
    (tail) -- and Tornado's ``geo["CG"] = [2.94, 0, 0]`` against its
    ``geo["ref_point"] = [0, 0, 0]`` therefore means the CG is 2.94 ft *aft* of the
    reference, i.e. the reference is ``ft(2.94) = 0.896112`` m *ahead* of the CG
    (``ft()`` being this module's feet-to-metres helper) and

        dx = -ft(2.94) = -0.896112      NOT  +0.896112
        dy = 0,  dz = 0                (both stations have y = z = 0)

    A solver that reports lift with an up-positive sign passes ``cz = -CL``, so the
    same case is ``cz = -4.363``.  The x axis is the only one the aft-positive flip
    touches for this aircraft, but the general rule above governs all three.

    Those two signs multiply into ``dx*cz``, so **a wrong pairing still returns a
    plausible number**.  Worked with ``cm = -7.720`` per radian as measured about
    ``ref_point``, ``s_ref = ft(24.0)`` from ``AERO.SREF``, ``b_ref = ft(12.0)`` from
    ``AERO.BLREF`` and ``c_ref = ft(2.0) = 0.6096`` from ``AERO.CBARR``:

    ==========================  ======================  ================
    call                        shifted Cm_a             static margin
    ==========================  ======================  ================
    ``cz=-4.363, dx=-0.896112``  ``-1.3063899999999986``  0.2994 cbar  correct
    ``cz=-4.363, dx=+0.896112``  ``-14.133610000000001``  3.2394 cbar  visibly wrong
    ``cz=+4.363, dx=+0.896112``  ``-1.3063899999999986``  0.2994 cbar  the trap
    ``cz=+4.363, dx=-0.896112``  ``-14.133610000000001``  3.2394 cbar  wrong
    ==========================  ======================  ================

    So sanity-check the result rather than the arguments: after shifting, the static
    margin ``cm_alpha / cl_alpha`` must land in 0.05 ... 0.45 cbar -- in the written
    arrays that is ``coefficients["cm"][1] / coefficients["cz"][1]``, where ``cl_alpha``
    is the ``DerivativeSet`` slot that becomes ``cz[1]``, hence **negative** for a
    stable aircraft and ``-1`` in ``SIGN_INVARIANTS``; the physical ``CL_alpha`` is
    ``-cz[1]`` and positive, and using it here would invert the ratio and condemn the
    correct answer.  A value outside the band means the frame or the ``cz`` sign is
    wrong, not that the aircraft is unusual -- and the third row above passes the
    number check with the convention still wrong, which is why the geometry has to be
    right first.  See the module docstring for the derivation and the per-arm physical
    reading of all six terms.
    """
    if s_ref <= 0.0 or b_ref <= 0.0 or c_ref <= 0.0:
        raise ValueError(
            f"reference geometry must be positive, got s_ref={s_ref}, b_ref={b_ref}, c_ref={c_ref}"
        )
    cz = float(cz)
    cx = float(cx)
    cy = float(cy)
    dx = float(dx)
    dy = float(dy)
    dz = float(dz)
    return (
        cz,
        cx,
        cy,
        float(cl_m) + (dz * cy - dy * cz) / b_ref,
        float(cm) + (dx * cz - dz * cx) / c_ref,
        float(cn) + (dy * cx - dx * cy) / b_ref,
    )


def normalise_sign(name: str, value: float) -> tuple[float, bool]:
    """Return ``(value, flipped)`` with ``value`` on the invariant side of zero.

    A value whose sign disagrees with ``SIGN_INVARIANTS[name]`` is multiplied by
    -1 and reported as flipped, so the caller can log it.  Zero is never flipped
    and never returns negative zero: it has no sign to disagree with, and a zero
    slot is a declared absence rather than a value.

    A name in ``NO_INVARIANT`` -- ``cm0``, whose sign the two in-tree reference
    files disagree on -- is returned unchanged.  Any other name the tables do not
    know raises ``KeyError``: Tasks 3 and 4 call this directly to log flips into
    ``provenance``, so a misspelled slot must fail loudly instead of shipping an
    inverted sign that no test would catch.
    """
    key = _KEY_ALIASES.get(name, name)
    if key not in SIGN_INVARIANTS and key not in NO_INVARIANT:
        canonical = sorted(set(SIGN_INVARIANTS) | set(NO_INVARIANT))
        raise KeyError(
            f"no sign invariant for {name!r}; accepted names are {canonical} "
            f"or the array-named aliases {sorted(_KEY_ALIASES)}"
        )
    value = float(value)
    if value == 0.0:
        return 0.0, False
    if key in NO_INVARIANT:
        return value, False
    if (value > 0.0) == (SIGN_INVARIANTS[key] > 0):
        return value, False
    return -value, True