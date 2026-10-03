"""Drive the sibling ``aid`` vortex-lattice solver over the Cessna 172 source.

This module owns everything Task 3 needs that ``aero_convert.units`` and
``aero_convert.morelli`` deliberately do not: it imports the sibling solver,
runs it, maps Tornado's names and axes onto ``DerivativeSet``, and assembles the
two files the flight model needs (``tornado.jsonc`` and ``geometry.jsonc``).
Neither ``data/planes/linear/morelli.json`` nor anything in ``python/f16`` is
read for a number here; every value comes from the source analysis file or from
the solver.

The sibling package is imported by putting its ``src`` directory on
``sys.path`` **at call time**, never at import time, so this module can be
imported (and its constants read) on a checkout with no sibling at all.  The
location is ``$AID_SRC`` when set and otherwise
``<repo>/../USAF_DATCOM/AircraftIntuitiveDesign/Python/src``.  Nothing is
vendored and nothing is added to a requirements file.  Both the sibling
analysis and the sibling source tree are read-only: solver scratch goes to a
``TemporaryDirectory`` that is also the working directory for the duration of
every solve.

Tornado's axes, and why every moment here needs a sign the table then re-checks
-----------------------------------------------------------------------------
The source planform is the usual DATCOM layout: stations grow **aft**, ``Z`` is
height, so ``AERO.XW = 2.2`` (wing) ``< AERO.XCG = 2.94`` ``< AERO.XH = 8.75``
(tail).  ``tornado_io`` copies those numbers straight into the lattice, and its
own ``geo`` reports ``ref_point = [0, 0, 0]`` against ``CG = [2.94, 0, 0]``: the
moment reference point is 2.94 ft **ahead** of the CG.  In that frame (call it
T) the axes are **x aft-positive, z up-positive, y right**, which is
right-handed, and the freestream vector ``wind1 = AS (cos a cos b, -cos a sin b,
sin a)`` points aft and up at positive alpha -- i.e. it is the *relative wind*,
which is why the code's ``r x omega`` rotational perturbation is the correct
``omega x r`` for a velocity vector: ``wind`` is minus the body's velocity, so
negating it twice leaves the rigid-body term right.

The axis mapping resolves the **alpha, beta and Q** inversions on its own.  It
does **not** resolve the **P and R** ones: those are a separate, narrower defect
-- ``rot_rates = np.array([state["P"], state["Q"], state["R"]])`` drops the
standard-aero rates straight into the Tornado (aft, right, up) component slots
without converting their sense, so roll-right is ``omega_x_aft = -P`` and
nose-right is ``omega_z_up = -R``, and the P and R columns come back inverted
whatever the axis mapping does.  Measured on this aircraft: ``CY_P = +0.160``
where physics needs ``CY_p < 0`` and ``CY_R = -0.387`` where it needs
``CY_r > 0``, while ``Cm_Q`` is right.  The P and R slots are therefore left to
``aero_convert.units.normalise_sign``, which flips them and logs the flip.

=========================  ===================  ==========================  ==========
Tornado reports            mapped to dynamics   invariant table wants      resolved by
=========================  ===================  ==========================  ==========
``CL_a = +5.148013``       ``cz = -CL_a``        ``cl_alpha`` < 0            mapping
``CL_Q = +9.591131``       ``cz = -CL_Q``        ``cl_q`` < 0                mapping
``Cl_b = +0.053721``       ``cl = -Cl_b``        ``cl_beta`` < 0             mapping
``Cn_b = -0.246500``       ``cn = -Cn_b``        ``cn_beta`` > 0             mapping
``Cm_a = -8.786140``       ``cm = +Cm_a``        ``cm_alpha`` < 0            mapping
``Cm_Q = -33.348777``      ``cm = +Cm_Q``        ``cm_q`` < 0                mapping
``Cl_P = -0.486095``       ``cl = -Cl_P = +0.4861``  ``cl_p`` < 0            the table
``Cl_R = +0.044525``       ``cl = -Cl_R = -0.0445``  ``cl_r`` > 0            the table
``CY_P = +0.159618``       ``cy = +CY_P``        ``cy_p`` < 0                the table
``CY_R = -0.386951``       ``cy = +CY_R``        ``cy_r`` > 0                the table
``Cn_R = -0.286403``       ``cn = -Cn_R = +0.2864``  ``cn_r`` < 0            the table
``Cn_P = +0.068671``       ``cn = -Cn_P = -0.0687``  ``cn_p`` < 0            mapping
=========================  ===================  ==========================  ==========

Note the last column honestly: the mapping hands ``Cl_P``, ``Cl_R``, ``Cn_R``,
``CY_P`` and ``CY_R`` to the table **still on the wrong side** -- five slots -- and
it is the table, not the mapping, that puts them right.  ``Cn_P`` is *not* one of
them: ``-Cn_P = -0.0687`` already satisfies ``cn_p < 0``, so that row's provenance
reads ``"state": "normalised"`` and ``cn_p`` is absent from ``provenance.flipped``.
This table is prose and cannot be derived from data, so **the file header computes
the same set from the run's own ``provenance.flipped`` at build time** (see
``_flipped_rate_slots``), which is what stops the two from drifting apart.  Two of the three moment axes flip
because a right-handed frame that reverses two of its three axes reverses
exactly those two moment senses; the pitch axis (right in both) does not flip,
which is what keeps ``Cm`` the one quantity whose sign survives untouched.  Every
slot is handed to ``normalise_sign`` rather than being signed here, so a
disagreement is logged in ``provenance.flipped`` instead of applied by hand --
ten of the twenty-five slots are flipped by the table, and ``provenance.mapping``
records the solver's value and the mapped value for each.

**None of this changes a written coefficient.**  The rate-derivative question was
recomputed under both readings and the twenty-five written values are identical,
because ``normalise_sign`` is the final authority and a component-sense flip
reverses the moment and its shift's force coefficient together, so the shift term
flips with the moment and the table undoes the lot.

``shift_moments_to_cg`` needs the same mapping, but its ``cz`` is the **body-z
force** coefficient, not the wind-axis lift: the pitch line is
``dx * F_z / c_ref``, so the argument is ``coeff_create``'s ``CZ`` (``-CZ_a``,
``-CZ_Q``, ``-CZ_de``), not ``-CL``.  ``cl = -Cl`` is not a shift argument at all:
with ``dy = dz = 0`` no force reaches the roll axis.  Tornado's y axis is already
the dynamics y axis, so the side force passes through as ``cy = +CY`` and does
reach the yaw line.

Slopes shift, intercepts shift differently
------------------------------------------
``shift_moments_to_cg`` moves a *coefficient*, so what it needs beside it is
the force coefficient that goes with the same independent variable:

* a **slope** w.r.t. alpha (``Cm_a``) needs the **body-z** lift slope
  ``cz = -CZ_a``, because ``d/dalpha`` of the shift ``dx * CZ / c_ref`` is
  ``dx * CZ_alpha / c_ref`` and the quantity in that expression is the body-z
  force, not the wind-axis lift.  ``CZ_a`` and ``CL_a`` differ here by 0.34 %
  (``5.165581`` against ``5.148013`` at alpha = 4 deg) because the aircraft
  carries NP[0], and using the lift slope there leaves that much linearisation
  error in ``cm[1]``;
* an **intercept** (``Cm0``, from the alpha = 0 run) needs the lift
  **intercept** ``cz = -CL(0)``, because the constant force arm does not
  differentiate away -- and at alpha = 0 the wind-axis and body-axis lifts
  coincide exactly, so the same number is both;
* a **slope** w.r.t. a control deflection (``Cm_de``) needs the body-z force
  slope w.r.t. that same deflection for exactly the first reason.
  ``tornado_controls`` reports only the wind-axis ``CL`` and ``CD``, so it is
  reconstructed exactly: with ``s = sin(alpha)``, ``c = cos(alpha)`` the pair
  ``[dCL; dCD] = [[-s, c], [c, s]] . [dCX; dCZ]`` and that matrix is its own
  inverse, so ``dCZ = c*dCL + s*dCD``;
* a **slope** w.r.t. a rate (``Cm_Q``) needs ``cz = -CZ_Q``, same reason;
* a **slope** w.r.t. beta, p or r (``Cn_beta``, ``Cn_p``, ``Cn_r``) needs the
  side-force slope for the same variable (``CY_b``, ``CY_P``, ``CY_R``), which is
  why the side force is passed even though ``dy = 0`` here.

Rolling and yawing about the CG need no shift at all for this aircraft, because
``dy = dz = 0`` leaves ``Cl`` and ``Cn`` untouched.  Every call site says which
of the two kinds it is in a comment.

Cross-check on ``CL_a``
----------------------
The plan asks for the two-point difference between an ``alpha = 0`` and an
``alpha = 2 deg`` run as a drift check on ``coeff_create``'s own ``CL_a``, and
that difference is recorded here (``cl_alpha_cross_check.coarse``).  It cannot be
the 1 % gate the plan names, and the reason is measured, not assumed: a full
re-run rebuilds the wake at the new alpha (``aid/tornado/lattice.py``'s
``_wakesetup2`` puts the trailing legs at ``infdist * (cos a, .., sin a)``), so
the 0 -> 2 deg secant is 4.359093/rad against ``coeff_create``'s 5.148013/rad --
15.3 % low, because the wake tilts as the aircraft pitches.  The 1 % gate is
therefore applied where it can actually see a mesh or convention drift: a secant
across +/-0.05 deg taken on the **frozen** base lattice, which is the same
fixed-wake derivative ``coeff_create`` differentiates.  Measured, that secant is
5.132344/rad against 5.148013/rad, 0.304 % -- inside the gate.  A coarser
secant that lands outside 25 % still fails loudly, so a genuine break cannot
pass unnoticed.

Parasite drag
-------------
``CD`` minus ``K CL^2`` at ``alpha = 0`` is **negative** here (-0.001673): a
linear vortex-lattice run has no profile drag at all, so subtracting the induced
part leaves nothing.  ``cx[0]`` is therefore taken from the source file's own
``WG.CD0 = 0.017219`` -- the parasite drag DATCOM's handbook routine computed
for this same planform, in the same file, and the number the head-of-file comment
calls ``Cd0`` -- and both figures are recorded in ``provenance.cd0``.

The three cross-check adapters: DATCOM, AVL and flow5
----------------------------------------------------
Same shape as the Tornado adapter above -- read the source, run a solver, map onto
``DerivativeSet``, record every conversion, solve the same trim, write a
``.jsonc`` -- and deliberately **no coefficient is shared between them**.  Each file
is an independent method on the same planform, so agreement between them is
evidence and a shared number would destroy it.

All three are evaluated at ``alpha = 4 deg``, the middle of ``AERO.ALSCHD``,
which is the row ``aid/tornado_io.py``'s ``_build_state`` already picks for
Tornado.  ``aid/lateral.py`` would otherwise default to ``alpha = 0`` ("Lateral
runs before longitudinal trim"), and at zero incidence both terms of DATCOM's
``CY_p`` vanish for this unswept planform, so the slot would be identically zero.

**All three report about the CG**, unlike Tornado, and that is *verified* rather
than assumed: DATCOM's ``$SYNTHS`` card, AVL's ``#Xref Yref Zref`` and flow5's
``cog_m`` are each read back out of the input this converter built and compared
with ``AERO.XCG``; a mismatch raises.  So ``dx = dy = dz = 0`` and no moment
shift is applied.  ``AVL`` and ``flow5`` additionally share
``plane/dynamics.py``'s frame outright -- AVL's own ``.sb`` prints "Standard axis
orientation, X fwd, Z down" -- so neither adapter contains an axis mapping at
all.  The only negations anywhere are the three slots the schema stores in the
wind-axis lift convention, ``cz[1]``, ``czq[0]`` and ``cz[5]``, because
``cz = -CL``.

Four defects in the sibling package sit on these paths and are corrected here
rather than inherited, each with the measurement that shows it:

* ``aid.stability._per_deg``, ``aid.handbook_controls._per_deg`` and
  ``aid.handbook_pass._per_rad`` are all ``if a > 1: convert else pass``.  This
  planform's lift slopes are 0.076-0.084 /deg, so every one is a no-op.
* **The Fortran's own stability table is per degree.**  Measured against its own
  ``cl`` column, ``cla`` equals the per-degree secant ``dCL/dalpha`` (0.1003/deg
  against a centred 0.100375/deg secant at alpha = 4 deg), so every derivative
  read out of ``for006`` is multiplied by ``DEG_TO_RAD``.
* ``aid/lateral.py:571`` ends its vertical-tail lift-slope formula with
  ``vt_a * pi/180``.  That formula returns the units of the ``k = a0/(2*pi)`` it
  was fed, and AID feeds it ``VT.a0`` = 6.8396 DEGREES, so the result is already
  per degree and the ``pi/180`` makes it 57.3x too small.  Every quantity linear
  in that slope -- ``CYb``, ``Cnb``, ``Clb``, ``Cn_r`` and the DATCOM Section 7
  rate terms -- is recomputed.  The evidence it is a defect: AID's ``Clb`` is
  -0.0031 against the sibling's own AVL gold -0.065868 for the same planform,
  while the corrected -0.0690 is within 5 % of that gold.
* ``aid/lateral.py:616`` returns a hardcoded ``0.1`` for both ``Clda`` and
  ``Cnda``, and ``aid/handbook_controls.py`` has no aileron ``CY`` or ``Cn``
  column at all.  The placeholders are not used; the slots are ``None``, zeroed
  and declared.

Coverage is measured, not assumed, and it differs sharply:

======================  ====  ==========================================================
Model                   of 25  Why
======================  ====  ==========================================================
``tornado.jsonc``         25   vortex lattice, all three surfaces
``datcom.jsonc``          22   the Fortran table plus the AID handbook; no ``cn_da``,
                                no ``cy_da``, no ``cd_q``
``avl.jsonc``             25   AVL's ``.sb`` carries aileron AND rudder deflection
                                columns, which ``run_avl_full``'s own ``.sb`` does not
``flow5.jsonc``           7   see below
======================  ====  ==========================================================

**flow5 produces seven of the twenty-five fields and that is its whole
capability**, read off ``FLOW5/run/flow5_run.cpp``: the deck's only condition key
is ``polar.alpha_deg``, the analysis calls ``setComputeDerivatives(false)``, and
``polar_to_json`` serialises exactly ``alpha``, ``CL``, ``CD``, ``Cm``, ``CLa``
and ``Cma``.  There is no sideslip sweep, no rate sweep, and no roll, yaw or
side-force column anywhere in its output, so ``aid.flow5_controls`` returns
``None`` for every aileron and rudder coefficient even though its ``_KEEP`` table
asks for them.  ``flow5.jsonc`` is therefore a *longitudinal* cross-check on the
lift slope, the pitching slope, the two intercepts, the elevator and the parasite
drag.  It flies, but it is not a 6-DOF data set and the file says so in its
header.

Rate derivatives and the flight condition
-----------------------------------------
The nine genuinely per-``p-hat`` / per-``q-hat`` / per-``r-hat`` slots
(``cyp[0]``, ``cyr[0]``, ``czq[0]``, ``cmq[0]``, ``clp[0]``, ``clr[0]``,
``cnp[0]``, ``cnr[0]``, ``cxq[0]``) are **not rescaled**, exactly as
``tornado.jsonc`` records: the plan fixes the flight condition from the source
``AERO`` and calls rescaling a physics decision for a later task.  ``cy[1]`` and
``cy[2]`` are not in that list -- they are per radian of CONTROL DEFLECTION.

One wrinkle the three files make visible and ``tornado.jsonc`` does not.  AID hands
its drivers ``mach * atm['a'] / 3.28084`` = 10.2073 and labels it ft/s while the
reference geometry is in feet, so for DATCOM and AVL the solver speed in this
model's metre frame is 10.2073 x 0.3048 = **3.1112 m/s**, giving the same 7.43x
caveat as Tornado.  flow5's deck is SI end to end and puts the same number in
``qinf_mps``, so there it really is **10.2073 m/s** and the caveat is 2.26x.  Both
readings are recorded in every file's ``provenance.flight_condition``; neither is
reconciled away, because the difference is a real property of each solver's input
and hiding it would make the four files look like they disagree when they do not.

Note also that the plan's own suggested rescale factor, ``AS_source / V_trim``, is
the **inverse** of the direction that restores the solver's physical ``dC/dq``
(that is ``V_trim / AS_source``).  Since nothing is rescaled, this is only ever a
note, but it is written into all three files so a later reader does not apply the
wrong direction.
"""
from __future__ import annotations

import json
import math
import os
import sys
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from typing import Any, Iterator

import numpy as np

from aero_convert.morelli import DerivativeSet, to_morelli
from aero_convert.units import (
    DEG_TO_RAD,
    FT_TO_M,
    G_FT_S2,
    SIGN_INVARIANTS,
    SLUG_TO_KG,
    ft,
    normalise_sign,
    per_degree_to_per_radian,
    shift_moments_to_cg,
)
from f16.aero_morelli import morelli_coefficients
from plane.atmosphere import air
from plane.dynamics import plane_derivative
from plane.groups import MORELLI_LENGTHS

__all__ = [
    "AID_SRC_ENV",
    "AVL_MESH",
    "BUILD_COMMAND",
    "CROSS_CHECK_MODELS",
    "DEFAULT_AID_SRC",
    "DEFAULT_SOURCE",
    "FLOW5_MESH",
    "SOURCE_ALPHA_INDEX_RULE",
    "source_mass_kg",
    "TORNADO_MESH",
    "CrossCheckRun",
    "TornadoRun",
    "aid_src",
    "avl_derivatives",
    "avl_run",
    "build",
    "build_all",
    "cross_check_payload",
    "datcom_derivatives",
    "datcom_run",
    "default_source",
    "flow5_derivatives",
    "flow5_run",
    "geometry_block",
    "model_payload",
    "tornado_derivatives",
    "tornado_run",
    "write_all",
    "write_model",
]

# Mesh ("10", "5") is the AID batch default for Tornado (aid/control_report.py's
# _DEFAULT_MESH and aid/compare.py's _TORNADO_MESH).  NP[0] -- the "AR 33.3,
# S 3 ft^2" panel -- stays in the lattice: `tornado_io` adds it for us and the
# measured spanwise split at the trim alpha gives it 0.048336 of the aircraft's
# 0.502274 total CL, 9.6 %.
TORNADO_MESH = ("10", "5")

# Mesh ("10", "10") for AVL and flow5: aid/avl_controls.py's own default mesh, so
# the control-derivative run and the stability run agree, and the same chordwise
# x spanwise density flow5's write_flow5_deck reads as (ny, nx).
AVL_MESH = ("10", "10")
FLOW5_MESH = ("10", "10")

# The three cross-check models, in the order they are written.  DATCOM has no
# mesh: it is the handbook method plus a Fortran binary, not a panel solve.
CROSS_CHECK_MODELS = ("datcom", "avl", "flow5")

# Every solver is evaluated at the MIDDLE of the source AERO.ALSCHD schedule,
# which is what aid/tornado_io.py's `_build_state` already does (`alschd[ceil(n/2)
# - 1]`), so all four files describe the same alpha = 4 deg condition.  AID's
# lateral pass would otherwise default to alpha = 0 (aid/lateral.py's
# `_alpha_deg`: "Lateral runs before longitudinal trim, so the default is 0"),
# which makes CY_p identically zero for this unswept planform.
SOURCE_ALPHA_INDEX_RULE = "the middle entry of AERO.ALSCHD, as aid/tornado_io.py::_build_state picks it"

REPO_ROOT = Path(__file__).resolve().parents[2]
AID_SRC_ENV = "AID_SRC"
DEFAULT_AID_SRC = REPO_ROOT.parent / "USAF_DATCOM" / "AircraftIntuitiveDesign" / "Python" / "src"
DEFAULT_SOURCE = REPO_ROOT.parent / "USAF_DATCOM" / "AircraftIntuitiveDesign" / "Analyses" / "Cessna172.jsonc"
DEFAULT_OUT_DIR = REPO_ROOT / "data" / "planes" / "cessna172"
BUILD_COMMAND = "python3 scripts/build_cessna172_aero.py"

# The source's own mass: AERO.WT slugs, read from the source file at run time.
# A slug IS a mass unit, so the honest conversion is WT * SLUG_TO_KG = 72.96951 kg
# for this aircraft.  The plan's approved design instead states 72.574 kg, which is
# 5 slug * 32.0 ft/s^2 = 160.0 lbf divided by 2.2046226 lbf/kg -- i.e. it assumes a
# standard gravity of 32.0 ft/s^2, and g = 32.0 appears nowhere in the source file
# (its own G_FT_S2 is 32.174).  The plan is 0.54 % low, but 72.574 is the figure its
# approved design fixes and plan test 6 pins verbatim in this file's header, so it
# is what ships; the derivation, the discrepancy and the slug figure all go into
# provenance.mass and into the header, so nothing here is a bare literal.
LBF_PER_KG = 2.2046226
PLAN_STANDARD_GRAVITY_FT_S2 = 32.0


def source_mass_kg(aero: dict) -> float:
    """``mass_kg`` from the source's ``AERO.WT``, on the plan's ``g = 32.0 ft/s^2``.

    Derived, not typed, so a change to ``AERO.WT`` in the source file moves this
    number.  The quotient is truncated rather than rounded: the exact value is
    72.57477991924786 kg and the plan's stated 72.574 is that cut to three
    decimals,
    which is the literal plan test 6 pins in this file's header.  See the
    module-level note for why the plan's figure ships in preference to the slug
    conversion's 72.96951.
    """
    weight_lbf = float(_first(aero["WT"])) * PLAN_STANDARD_GRAVITY_FT_S2
    return math.floor(weight_lbf / LBF_PER_KG * 1000.0) / 1000.0


# Radius-of-gyration rules of thumb, per the plan's "Mass, inertia, thrust, and
# the initial state": derived on THIS mass and THIS span, never scaled from a
# published full-scale C172 (the mass and the length scale are mutually
# inconsistent here -- 5 slugs is not the 53.5 kg a geometrically similar 1/3
# scale aeroplane would weigh).
RADIUS_RULES = {
    "k_xx": (0.35, "roll: k_xx = 0.35 b"),
    "k_yy": (0.25, "pitch: k_yy = 0.25 b"),
    "k_zz": (0.30, "yaw: k_zz = 0.30 b"),
}

# Trim solve.  A cruise lift coefficient is chosen, the speed follows from level
# flight at it, and the throttle that zeroes the pitching moment and the thrust
# that balances the drag follow from the model's own coefficients.
TARGET_CRUISE_CL = 0.30
CRUISE_POWER = 0.5
V_REF_FACTOR = 1.25
SEA_LEVEL_G_MPS2 = 9.80665
SEA_LEVEL_RHO = 1.225055883
TAU_S = 1.0
# The initial state starts above the ground: plane/atmosphere.py rejects a negative
# altitude outright, so a run that descends at all would abort instead of writing a
# CSV.  100 m is inside 1 % of sea-level density, so the trim is unaffected.
INITIAL_ALT_M = 100.0

# Finite-difference half-width for the frozen-lattice CL_a cross-check.
CROSSCHECK_HALF_DEG = 0.05
CROSSCHECK_TOLERANCE = 0.01
COARSE_SECANT_ALPHAS_DEG = (0.0, 2.0)
COARSE_TOLERANCE = 0.25

# Every DerivativeSet field -> the Morelli slot it becomes, so provenance can
# name the array and index a zeroed or flipped value occupies.
FIELD_SLOT = {
    "cd0": "cx[0]", "cd_q": "cxq[0]",
    "cy_beta": "cy[0]", "cy_da": "cy[1]", "cy_dr": "cy[2]",
    "cy_p": "cyp[0]", "cy_r": "cyr[0]",
    "cl0": "cz[0]", "cl_alpha": "cz[1]", "cl_de": "cz[5]", "cl_q": "czq[0]",
    "cl_beta": "cl[0]", "cl_p": "clp[0]", "cl_r": "clr[0]",
    "cl_da": "clda[0]", "cl_dr": "cldr[0]",
    "cm0": "cm[0]", "cm_alpha": "cm[1]", "cm_de": "cm[2]", "cm_q": "cmq[0]",
    "cn_beta": "cn[0]", "cn_p": "cnp[0]", "cn_r": "cnr[0]",
    "cn_da": "cnda[0]", "cn_dr": "cndr[0]",
}


@dataclass(frozen=True)
class Geometry:
    """SI reference geometry and the flight condition the solver ran at."""

    s_ref_m2: float
    b_ref_m: float
    c_ref_m: float
    dx_m: float
    dy_m: float
    dz_m: float
    x_cg_ft: float
    x_ref_ft: float
    x_wing_ft: float
    x_tail_ft: float
    mach: float
    alt_ft: float
    betha_rad: float
    alpha_deg: float
    as_mps: float
    as_ftps: float
    rho_kg_m3: float
    mass_kg: float
    weight_slug: float
    mesh: tuple[str, str]
    # The PHYSICAL speed of the source Mach-0.03 condition, `mach * a_sound` in
    # m/s, and the speed of sound AID reports in ft/s.  Tornado fills neither
    # (it works in feet and labels its own AS), the three cross-check solvers do.
    v_true_mps: float = 0.0
    a_sound_ftps: float = 0.0


@dataclass(frozen=True)
class CrossCheckRun:
    """One DATCOM / AVL / flow5 solve, normalised, with everything to re-derive it.

    Deliberately the same shape as ``TornadoRun`` so ``_trim``, ``_slot_lines``
    and the shared header items work unchanged on all four models; only the
    provenance differs, because that is where a solver's own quirks live.
    """

    model: str
    derivatives: DerivativeSet
    coefficients: dict[str, list[float]]
    raw: dict[str, Any]
    geometry: Geometry
    provenance: dict[str, Any]
    aircraft: dict[str, float]
    initial: dict[str, float]
    controls: dict[str, float]
    trim: dict[str, float]


@dataclass(frozen=True)
class TornadoRun:
    """One Tornado solve, normalised, with everything needed to re-derive it."""

    derivatives: DerivativeSet
    raw: dict[str, Any]
    geometry: Geometry
    provenance: dict[str, Any]
    aircraft: dict[str, float]
    initial: dict[str, float]
    controls: dict[str, float]
    trim: dict[str, float]
    control_rows: list[dict[str, Any]]


def aid_src() -> Path:
    """Where the sibling ``aid`` package lives: ``$AID_SRC`` or the default."""
    override = os.environ.get(AID_SRC_ENV)
    return Path(override).expanduser() if override else DEFAULT_AID_SRC


def default_source() -> Path:
    """The sibling analysis file this converter reads.  Never written to."""
    return DEFAULT_SOURCE


@contextmanager
def _scratch() -> Iterator[Path]:
    """Run inside a throwaway directory, so no solver path can write to the repo."""
    previous = Path.cwd()
    with TemporaryDirectory(prefix="aero_convert_") as tmp:
        os.chdir(tmp)
        try:
            yield Path(tmp)
        finally:
            os.chdir(previous)


def _modules() -> SimpleNamespace:
    """Import the sibling solver with its ``src`` on ``sys.path`` at call time."""
    root = _aid_on_path()
    from aid.aircraft import load_jsonc
    from aid.tornado.boundary import set_boundary
    from aid.tornado.coeff import coeff_create
    from aid.tornado.control_deriv import tornado_controls
    from aid.tornado.lattice import lattice_setup
    from aid.tornado.solver import solve
    from aid.tornado_io import tornado_io

    return SimpleNamespace(
        load_jsonc=load_jsonc,
        set_boundary=set_boundary,
        coeff_create=coeff_create,
        tornado_controls=tornado_controls,
        lattice_setup=lattice_setup,
        solve=solve,
        tornado_io=tornado_io,
    )


def _aid_on_path() -> Path:
    """Put the sibling ``aid`` ``src`` on ``sys.path`` and return its location."""
    root = aid_src()
    if not root.is_dir():
        raise FileNotFoundError(
            f"the sibling aid package is not at {root}; set {AID_SRC_ENV} to its 'src' directory"
        )
    entry = str(root)
    if entry not in sys.path:
        sys.path.insert(0, entry)
    return root


def _numpy_alias() -> str:
    """Bridge one numpy rename so the sibling package runs on this interpreter.

    ``aid/lateral.py`` (lines 381 and 483, inside ``_body_kn`` and
    ``lateral_static``) and ``aid/panel_method.py`` call ``np.trapezoid``, which
    numpy renamed from ``np.trapz`` in 2.0 and which does not exist in the 1.26
    this repository's tests run on.  ``trapezoid`` and ``trapz`` are the same
    function; binding the missing name is a name bridge, not a numeric change,
    and it is what lets the real DATCOM handbook and AVL control runs execute at
    all.  Returns a short description of what was bound, for the file header.
    """
    if hasattr(np, "trapezoid"):
        return "numpy.trapezoid present; no alias bound"
    setattr(np, "trapezoid", np.trapz)
    return "numpy.trapezoid absent (numpy < 2.0); bound it to numpy.trapz, the same function under its old name"


def _load(source: Path | None) -> tuple[Any, SimpleNamespace]:
    path = Path(source) if source is not None else default_source()
    if not path.is_file():
        raise FileNotFoundError(f"{path} is missing; the sibling analysis tree is read-only input")
    aid = _modules()
    with _scratch():
        return aid.load_jsonc(path), aid


def _geometry(ac: Any, geo: dict, state: dict) -> Geometry:
    """The reference geometry, read from what the solver built, not from AERO.

    ``ref`` and ``cg`` come from ``tornado_io``'s own ``geo``, which is what the
    shift is really about; the source ``AERO`` block is only cross-checked
    against it, because a silent disagreement there would move every moment.
    """
    aero = ac.AERO
    reference = [float(value) for value in np.asarray(geo["ref_point"], dtype=float).reshape(-1)]
    centre = [float(value) for value in np.asarray(geo["CG"], dtype=float).reshape(-1)]
    if len(reference) != 3 or len(centre) != 3:
        raise ValueError("Tornado's ref_point and CG must both be three-component")
    expected_cg = float(_first(aero["XCG"]))
    if abs(centre[0] - expected_cg) > 1e-9:
        raise ValueError(
            f"Tornado's geo['CG'] x = {centre[0]} disagrees with AERO.XCG = {expected_cg}; "
            "the moment shift would be about a station nobody else can see"
        )
    if any(abs(value) > 1e-12 for value in reference):
        raise ValueError(
            f"Tornado's geo['ref_point'] = {reference} is not the origin this conversion's "
            "pinned static margin assumes"
        )
    x_cg_ft = centre[0]
    x_ref_ft = reference[0]
    return Geometry(
        s_ref_m2=ft(float(_first(aero["SREF"]))),
        b_ref_m=ft(float(_first(aero["BLREF"]))),
        c_ref_m=ft(float(_first(aero["CBARR"]))),
        # r = CG - ref_point in the dynamics frame (x FORWARD).  Tornado's
        # stations are aft-positive, so the reference point is ft(2.94) m AHEAD
        # of the CG and the offset is NEGATIVE.  The two stations share y = z = 0.
        dx_m = ft(x_ref_ft - x_cg_ft),
        dy_m = ft(reference[1] - centre[1]),
        dz_m = ft(reference[2] - centre[2]),
        x_cg_ft=x_cg_ft,
        x_ref_ft=x_ref_ft,
        x_wing_ft=float(_first(aero["XW"])),
        x_tail_ft=float(_first(aero["XH"])),
        mach=float(_first(aero["MACH"])),
        alt_ft=float(_first(aero["ALT"])),
        betha_rad=float(state["betha"]),
        alpha_deg=math.degrees(float(state["alpha"])),
        # Tornado's state["AS"] is in FT/S: aid.atmosphere.atmosphere(0)["a"] is
        # 1116.288876590643 ft/s (the speed of sound in feet per second) and
        # _build_state divides by 3.28084.  AID works in feet throughout, so the
        # value is converted here rather than stored under a key that says m/s.
        as_ftps=float(state["AS"]),
        as_mps=float(state["AS"]) * FT_TO_M,
        rho_kg_m3=float(state["rho"]),
        mass_kg=source_mass_kg(aero),
        weight_slug=float(_first(aero["WT"])),
        mesh=TORNADO_MESH,
    )


def _first(value: Any) -> Any:
    return np.asarray(value, dtype=float).reshape(-1)[0]


def _solve(ac: Any, aid: SimpleNamespace, alpha_deg: float | None = None) -> dict:
    """One complete Tornado run; ``alpha_deg`` overrides the source's own alpha."""
    geo, state = aid.tornado_io(ac, TORNADO_MESH)
    if alpha_deg is not None:
        state = dict(state)
        state["alpha"] = math.radians(alpha_deg)
    lattice, ref = aid.lattice_setup(geo, state, 0)
    boundary = aid.set_boundary(lattice, geo, state)
    results = aid.solve(state, geo, boundary)
    coeffs = aid.coeff_create(results, boundary, state, ref, geo)
    return {"geo": geo, "state": state, "lattice": lattice, "ref": ref, "coeffs": coeffs}


def _frozen_lattice_coeffs(run: dict, aid: SimpleNamespace, alpha_deg: float) -> dict:
    """Coefficients at another alpha on the SAME lattice (wake not re-tilted)."""
    geo, lattice, ref = run["geo"], run["lattice"], run["ref"]
    state = dict(run["state"])
    state["alpha"] = math.radians(alpha_deg)
    boundary = aid.set_boundary(lattice, geo, state)
    results = aid.solve(state, geo, boundary)
    return aid.coeff_create(results, boundary, state, ref, geo)


def _control_rows(ac: Any, aid: SimpleNamespace, alpha_deg: float) -> dict[str, dict]:
    """``tornado_controls`` rows at the trim alpha, keyed by surface.

    ``aid.tornado.control_deriv`` reads its alpha from ``AERO.ALSCHD[0]``, which
    for this source is -4 deg -- a different condition from the rest of the set.
    The probe aircraft is a copy with the trim alpha first, so the control
    derivatives describe the same flight condition as every other slot.  The row
    values are central differences at the row's own deflection (the per-degree
    slopes), and the neutral-deflection row is the one taken.
    """
    probed = deepcopy(ac)
    probed.AERO["ALSCHD"] = [alpha_deg, *list(ac.AERO["ALSCHD"])]
    rows = aid.tornado_controls(probed, (0.0,), TORNADO_MESH)
    selected = {row["surface"]: row for row in rows if float(row["delta_deg"]) == 0.0}
    for surface, row in selected.items():
        if not row.get("available"):
            raise ValueError(f"Tornado could not deflect {surface}: {row.get('reason')!r}")
    return selected


class _Collector:
    """Fill DerivativeSet fields, recording missing / flipped / not-normalised."""

    def __init__(self) -> None:
        self.values: dict[str, float | None] = {}
        self.mapping: list[dict[str, Any]] = []
        self.missing: list[dict[str, str]] = []
        self.flipped: list[str] = []
        self.not_normalised: list[str] = []

    def put(
        self,
        field: str,
        mapped: float,
        *,
        tornado_key: str,
        solver_value: float | None,
        conversion: str,
        role: str,
    ) -> float | None:
        """Record one slot: what the solver said, what was done to it, what ships.

        ``solver_value`` is the number in the solver's own convention and units,
        exactly as the solver reported it (or ``None`` when the number did not
        come from a solver at all -- see ``cd0``).  ``mapped`` is that number
        after the frame mapping, the moment shift and the per-degree conversion,
        which is what the invariant table is asked about.  Both are recorded, so
        a reader can see exactly what was done; calling either of them "the raw
        value" without the other is how a false provenance line gets written.

        The finiteness guard lives here, at the solver boundary, because
        ``normalise_sign`` is not nan-safe (``nan > 0.0`` is False, so a nan
        would sail through the sign test and poison a coefficient array).
        """
        slot = FIELD_SLOT[field]
        reported = None if solver_value is None else float(solver_value)
        mapped_value = float(mapped)
        if not math.isfinite(mapped_value) or (
            reported is not None and not math.isfinite(reported)
        ):
            self.values[field] = None
            self.missing.append(
                {"field": field, "slot": slot, "tornado_key": tornado_key,
                 "reason": f"non-finite solver output {reported!r} -> {mapped_value!r}"}
            )
            self.mapping.append(
                {"field": field, "slot": slot, "tornado_key": tornado_key,
                 "solver_value": reported, "mapped_value": None,
                 "conversion": conversion, "role": role, "value": None,
                 "state": "missing", "solver_input": tornado_key.split(" ")[0]}
            )
            return None
        normalised, was_flipped = normalise_sign(field, mapped_value)
        self.values[field] = normalised
        if was_flipped:
            self.flipped.append(field)
            state = "flipped"
        elif field == "cm0":
            self.not_normalised.append(field)
            state = "not-normalised"
        else:
            state = "normalised"
        self.mapping.append(
            {"field": field, "slot": slot, "tornado_key": tornado_key,
             "solver_value": reported, "mapped_value": mapped_value,
             "conversion": conversion, "role": role, "value": normalised,
             "state": state,
             "solver_input": "solver" if reported is not None else "source file"}
        )
        return normalised

    def absent(self, field: str, key: str, *, reason: str) -> None:
        """Record a slot the solver genuinely cannot produce.

        Distinct from ``put``'s non-finite path: nothing came back at all, not
        even a number to reject.  ``to_morelli`` writes ``0.0`` for the slot and
        the reason is carried into ``provenance.missing`` so a reader can tell an
        honest absence from a value that happens to be zero.
        """
        slot = FIELD_SLOT[field]
        self.values[field] = None
        self.missing.append({"field": field, "slot": slot, "solver_key": key, "reason": reason})
        self.mapping.append(
            {"field": field, "slot": slot, "tornado_key": key, "solver_value": None,
             "mapped_value": None, "conversion": "not produced by this solver",
             "role": "absent", "value": None, "state": "missing", "solver_input": "none"}
        )

    def derivative_set(self) -> DerivativeSet:
        return DerivativeSet(**self.values)


def _shift(
    geometry: Geometry, *, cz: float, cy: float, cm: float, cn: float
) -> tuple[float, float, float]:
    """Move one coefficient set from ``ref_point`` to the CG.

    ``cz`` is the body-z coefficient (z down, so ``cz = -CZ``, the body force,
    not the wind-axis lift) and ``cy`` the body-y coefficient (``cy = +CY``, the
    body-axis side force, which is what ``coeff_create``'s ``CY_*`` is and what
    the model reads), both expressed **per the same independent variable as the
    coefficient being shifted** -- a slope for a slope, the intercept for an
    intercept.  ``cx`` and ``cl_m`` are passed as zero, and that is not an
    oversight: with ``dy = dz = 0`` the drag reaches the pitch line only through
    ``dz`` and the yaw line only through ``dy``, so no drag coefficient can enter
    a shift for this aircraft and ``cd0`` is never a shift argument; and no force
    at all reaches the roll axis, so the returned ``Cl`` shift is identically
    zero.  The side force must still be passed, because the yaw line shifts on
    ``dx * cy`` alone -- zeroing it would discard a real yaw moment.
    """
    cz_out, cx_out, cy_out, cl_out, cm_out, cn_out = shift_moments_to_cg(
        cz,
        0.0,
        cy,
        0.0,
        cm,
        cn,
        dx=geometry.dx_m,
        dy=geometry.dy_m,
        dz=geometry.dz_m,
        s_ref=geometry.s_ref_m2,
        b_ref=geometry.b_ref_m,
        c_ref=geometry.c_ref_m,
    )
    # The three force coefficients come back untouched, which is the point:
    # a moment reference moves moments only.
    if (cz_out, cx_out, cy_out) != (cz, 0.0, cy):
        raise AssertionError("shift_moments_to_cg altered a force coefficient")
    return cl_out, cm_out, cn_out


def tornado_run(source: Path | None = None) -> TornadoRun:
    """Run Tornado over the source geometry and normalise the result.

    Four solver passes: the trim alpha (every ``coeff_create`` derivative), an
    ``alpha = 0`` pass (the two intercepts the model cannot do without), the
    frozen-lattice ``CL_a`` cross-check, and ``tornado_controls`` for the three
    control surfaces.
    """
    ac, aid = _load(source)
    with _scratch():
        trim = _solve(ac, aid)
        zero = _solve(ac, aid, alpha_deg=0.0)
        coarse = [_solve(ac, aid, alpha_deg=a)["coeffs"] for a in COARSE_SECANT_ALPHAS_DEG]
        low = _frozen_lattice_coeffs(trim, aid, trim["state"]["alpha"] * 180.0 / math.pi - CROSSCHECK_HALF_DEG)
        high = _frozen_lattice_coeffs(trim, aid, trim["state"]["alpha"] * 180.0 / math.pi + CROSSCHECK_HALF_DEG)
        rows = _control_rows(ac, aid, math.degrees(float(trim["state"]["alpha"])))

    coeffs = trim["coeffs"]
    geometry = _geometry(ac, trim["geo"], trim["state"])
    cl_alpha = float(coeffs["CL_a"])
    secant = (
        (float(high["CL"]) - float(low["CL"])) / math.radians(2.0 * CROSSCHECK_HALF_DEG)
    )
    if abs(secant - cl_alpha) > CROSSCHECK_TOLERANCE * abs(cl_alpha):
        raise ValueError(
            f"frozen-lattice CL_a cross-check failed: coeff_create {cl_alpha:.6f} against a "
            f"+/-{CROSSCHECK_HALF_DEG} deg secant {secant:.6f} "
            f"({100.0 * abs(secant - cl_alpha) / abs(cl_alpha):.3f} % > "
            f"{100.0 * CROSSCHECK_TOLERANCE:.1f} %); the mesh or a convention has drifted"
        )
    coarse_secant = (
        (float(coarse[1]["CL"]) - float(coarse[0]["CL"]))
        / math.radians(COARSE_SECANT_ALPHAS_DEG[1] - COARSE_SECANT_ALPHAS_DEG[0])
    )
    if abs(coarse_secant - cl_alpha) > COARSE_TOLERANCE * abs(cl_alpha):
        raise ValueError(
            f"coarse CL_a secant {coarse_secant:.6f} against coeff_create {cl_alpha:.6f}; "
            "the source geometry no longer reproduces"
        )

    collector = _Collector()
    raw: dict[str, Any] = {
        "CL_a": cl_alpha,
        "CL_at_alpha0": float(zero["coeffs"]["CL"]),
        "CL_Q": float(coeffs["CL_Q"]),
        "CL_P": float(coeffs["CL_P"]),
        "CL_R": float(coeffs["CL_R"]),
        "Cm_a": float(coeffs["Cm_a"]),
        "Cm_at_alpha0": float(zero["coeffs"]["Cm"]),
        "Cm_Q": float(coeffs["Cm_Q"]),
        "Cl_b": float(coeffs["Cl_b"]),
        "Cl_P": float(coeffs["Cl_P"]),
        "Cl_R": float(coeffs["Cl_R"]),
        "Cn_b": float(coeffs["Cn_b"]),
        "Cn_P": float(coeffs["Cn_P"]),
        "Cn_R": float(coeffs["Cn_R"]),
        "CY_b": float(coeffs["CY_b"]),
        "CC_b": float(coeffs["CC_b"]),
        "CY_P": float(coeffs["CY_P"]),
        "CY_R": float(coeffs["CY_R"]),
        "CX_Q": float(coeffs["CX_Q"]),
        "CD_at_alpha0": float(zero["coeffs"]["CD"]),
        "CD_at_alpha": float(coeffs["CD"]),
        "CL_at_alpha": float(coeffs["CL"]),
        "CD_a": float(coeffs["CD_a"]),
        "CZ_a": float(coeffs["CZ_a"]),
        "CZ_Q": float(coeffs["CZ_Q"]),
        "CZ_at_alpha0": float(zero["coeffs"]["CZ"]),
        "K_wing": float(ac.WG["K"]),
        "CD0_source": float(ac.WG["CD0"]),
        "CL_wing": [float(value) for value in np.asarray(coeffs["CLwing"]).reshape(-1)],
        "wing_names": [str(name) for name in np.asarray(trim["geo"]["name"]).reshape(-1)],
    }
    # NP[0] ("Wing 2" in Tornado's own naming) is the AR 33.3, S 3 ft^2 panel the
    # plan keeps in the lattice; its share of the aircraft lift is reported so the
    # decision to keep it is visible in the file.
    extra_share = 0.0
    for index, name in enumerate(raw["wing_names"]):
        if name not in ("Wing", "HT", "VT") and index < len(raw["CL_wing"]):
            extra_share += raw["CL_wing"][index] / raw["CL_at_alpha"]
    raw["extra_panel_cl_share"] = extra_share
    raw["planform"] = _planform(ac, extra_share)

    # --- lift -----------------------------------------------------------------
    # The SLOT value and the SHIFT argument are deliberately different numbers.
    # `cz[1]` is the lift slope, because the plan's mapping table says
    # `CL_a -> cl_alpha` and both in-tree files store CL_alpha there.  The shift
    # argument must be the BODY-Z force slope, `coeff_create["CZ_a"]`, because
    # that is the quantity the shift's own algebra contains
    # (`M_G = M_O + (O - G) x F`, and the pitch line is `dx * F_z / c_ref`).  The
    # two differ by 0.34 % at alpha = 4 deg -- CL and CZ are the wind-axis and
    # body-axis lifts, and they part company wherever the axial force is
    # non-zero, which it is here because NP[0] is in the lattice.
    cz_alpha = -cl_alpha                       # slot: -CL_a
    cz_alpha_body = -float(coeffs["CZ_a"])     # shift argument: -CZ_a
    collector.put(
        "cl_alpha", cz_alpha, tornado_key="CL_a", solver_value=cl_alpha,
        conversion="slot: cz = -CL_a (the plan's mapping table, and both in-tree "
                    "files store CL_alpha in cz[1]); already per radian",
        role="slope",
    )

    # CL0 is an INTERCEPT: the alpha = 0 run gives it outright, no extrapolation,
    # and it is the lift slope's intercept, so it is also what cm0 shifts with.
    cl_at_zero = float(zero["coeffs"]["CL"])
    cl0 = -cl_at_zero
    collector.put(
        "cl0", cl0, tornado_key="CL(alpha=0)", solver_value=cl_at_zero,
        conversion="cz = -CL; a second solve at alpha = 0 rad, no extrapolation",
        role="intercept",
    )

    # CL_q: the slot takes the standard-aero CL_q the invariant table names, and
    # the shift takes the body-z slope with respect to q.
    cz_q = -float(coeffs["CL_Q"])
    cz_q_body = -float(coeffs["CZ_Q"])
    collector.put(
        "cl_q", cz_q, tornado_key="CL_Q", solver_value=float(coeffs["CL_Q"]),
        conversion="czq[0] holds the standard-aero CL_q (both in-tree files do), "
                   "so it is -CL_Q; coeff_create already divided by fac = c_mac/(2V)",
        role="slope",
    )

    # --- side force: Tornado's y axis is the dynamics y axis -------------------
    # `CY_*` is the BODY-y component, which is the one `plane/dynamics.py` reads.
    # (`coeff_create`'s other side force, `CC_*`, is the wind-axis one; the two are
    # identical at betha = 0 because the rotation's side-force row is then (0,1,0),
    # and the aileron/rudder rows below are likewise fed from `CC`.)
    cy_beta = float(coeffs["CY_b"])
    cy_p = float(coeffs["CY_P"])
    cy_r = float(coeffs["CY_R"])
    collector.put(
        "cy_beta", cy_beta, tornado_key="CY_b", solver_value=float(coeffs["CY_b"]),
        conversion="cy = +CY_b, the body-y slope; CC_b is the wind-axis side force "
                   "and equals CY_b exactly at betha = 0",
        role="slope",
    )
    collector.put(
        "cy_p", cy_p, tornado_key="CY_P", solver_value=float(coeffs["CY_P"]),
        conversion="cy = +CY_P, the body-y slope; already divided by fac", role="slope",
    )
    collector.put(
        "cy_r", cy_r, tornado_key="CY_R", solver_value=float(coeffs["CY_R"]),
        conversion="cy = +CY_R, the body-y slope; already divided by fac", role="slope",
    )

    # --- roll moments: Tornado's x axis is aft, so roll sense is negated ------
    for field, key in (("cl_beta", "Cl_b"), ("cl_p", "Cl_P"), ("cl_r", "Cl_R")):
        collector.put(
            field, -float(coeffs[key]), tornado_key=key, solver_value=float(coeffs[key]),
            conversion="cl = -" + key + " (Tornado x is aft-positive, so the roll "
                       "sense reverses); already divided by fac",
            role="slope",
        )

    # --- pitch: Tornado's pitch sense already agrees with dynamics ------------
    # Cm_a is a SLOPE in alpha, so it shifts with the BODY-Z lift SLOPE.
    _, cm_alpha, _ = _shift(geometry, cz=cz_alpha_body, cy=0.0, cm=float(coeffs["Cm_a"]), cn=0.0)
    collector.put(
        "cm_alpha", cm_alpha, tornado_key="Cm_a", solver_value=float(coeffs["Cm_a"]),
        conversion="cm = +Cm_a, then shifted with cz = -CZ_a, the matching body-z "
                   "force slope (not -CL_a: the shift's algebra contains F_z)",
        role="slope",
    )
    # Cm0 is an INTERCEPT, so it shifts with the lift INTERCEPT, not the slope.
    # At alpha = 0 the body-z and wind-axis lifts coincide exactly (the rotation is
    # the identity), so CL(0) is the right argument and is also what cl0 uses.
    cm_at_zero = float(zero["coeffs"]["Cm"])
    _, cm0, _ = _shift(geometry, cz=cl0, cy=0.0, cm=cm_at_zero, cn=0.0)
    collector.put(
        "cm0", cm0, tornado_key="Cm(alpha=0)", solver_value=cm_at_zero,
        conversion="cm = +Cm, then shifted with cz = -CL(0), the matching intercept "
                   "(at alpha = 0 CL = CZ); cm0 carries no sign invariant and is "
                   "never normalised",
        role="intercept",
    )
    # Cm_Q is a SLOPE in q, so it shifts with the body-z slope in q.
    _, cm_q, _ = _shift(geometry, cz=cz_q_body, cy=0.0, cm=float(coeffs["Cm_Q"]), cn=0.0)
    collector.put(
        "cm_q", cm_q, tornado_key="Cm_Q", solver_value=float(coeffs["Cm_Q"]),
        conversion="cm = +Cm_Q, then shifted with cz = -CZ_Q, the matching body-z "
                   "force slope; already divided by fac",
        role="slope",
    )

    # --- yaw: negated (z up vs z down), shifted on dx * cy --------------------
    for field, key, side in (
        ("cn_beta", "Cn_b", "CY_b"), ("cn_p", "Cn_P", "CY_P"), ("cn_r", "Cn_R", "CY_R"),
    ):
        shifted = _shift(
            geometry, cz=0.0,
            cy={"Cn_b": cy_beta, "Cn_P": cy_p, "Cn_R": cy_r}[key],
            cm=0.0, cn=-float(coeffs[key]),
        )[2]
        collector.put(
            field, shifted, tornado_key=key, solver_value=float(coeffs[key]),
            conversion=f"cn = -{key} (Tornado z is up-positive, so the yaw sense "
                       f"reverses), then shifted with the body-y slope {side}; "
                       "already divided by fac",
            role="slope",
        )

    # --- drag ------------------------------------------------------------------
    # cxq[0] and czq[0] are the pair that the two in-tree files and the plan's
    # table store in the STANDARD-AERO convention: linear/morelli.json has
    # czq[0] = -2.0 and cxq[0] = 0.0, and the table pins both signs.  So these two
    # slots take Tornado's axial and normal coefficients as the standard CD_q and
    # CL_q, with no axis negation -- and that is also why czq[0] above is -CL_Q and
    # not the body-z +CL_Q.  The schema then ADDS them to the body-x Cx and body-z
    # Cz, which is an inconsistency of the schema itself, inherited from both
    # in-tree files rather than introduced here.
    collector.put(
        "cd_q", float(coeffs["CX_Q"]), tornado_key="CX_Q",
        solver_value=float(coeffs["CX_Q"]),
        conversion="cxq[0] holds the standard-aero CD_q, which at trim is Tornado's "
                   "axial rate CX_Q (Tornado x aft => +CX is drag-positive); "
                   "already divided by fac",
        role="slope",
    )
    # Cd0: parasite drag.  The plan's formula, CD(0) - K*CL(0)^2, is NEGATIVE here
    # because a linear vortex-lattice run contains no profile drag at all, so the
    # source planform's own DATCOM Cd0 is used instead and both are recorded.
    parasite = float(zero["coeffs"]["CD"]) - float(ac.WG["K"]) * cl0 * cl0
    cd0_source = "Tornado CD(0) - K_wing*CL(0)^2"
    cd0_solver_value: float | None = float(zero["coeffs"]["CD"])
    cd0_raw = -parasite
    if parasite <= 0.0:
        cd0_source = (
            "source WG.CD0 (the plan's formula, Tornado CD(0) - K_wing*CL(0)^2 = "
            f"{parasite:.6f}, is negative: a linear VLM has no profile drag)"
        )
        cd0_raw = -float(ac.WG["CD0"])
        cd0_solver_value = None
    collector.put(
        "cd0", cd0_raw, tornado_key=f"cd0 <- {cd0_source}", solver_value=cd0_solver_value,
        conversion="parasite drag as a positive magnitude, then cx = -(parasite); "
                   "NOT a Tornado output when the source Cd0 is used",
        role="intercept",
    )

    # --- controls ---------------------------------------------------------------
    # `tornado_controls` returns central differences PER DEGREE; every one is
    # multiplied by DEG_TO_RAD before it goes near a Morelli array.  Its "CY"
    # entry is coeff_create's CC, the wind-axis side force; at betha = 0 the
    # wind-axis and body-axis side forces are identical, and the body axis is
    # the one the model wants.
    aileron, elevator, rudder = rows["aileron"], rows["elevator"], rows["rudder"]

    def control(field: str, surface: str, key: str, sign: float) -> None:
        per_degree = float(surface[key])
        collector.put(
            field, sign * per_degree_to_per_radian(per_degree),
            tornado_key=f"tornado_controls[{surface['surface']}].{key}",
            solver_value=per_degree,
            conversion=f"per degree x {DEG_TO_RAD}, then "
                       f"{'negated (Tornado axis sense)' if sign < 0 else 'as it stands'}",
            role="slope",
        )

    control("cl_da", aileron, "Cl", -1.0)
    control("cy_da", aileron, "CY", 1.0)
    control("cn_da", aileron, "Cn", -1.0)
    control("cl_dr", rudder, "Cl", -1.0)
    control("cy_dr", rudder, "CY", 1.0)
    control("cn_dr", rudder, "Cn", -1.0)
    # The elevator's vertical-force slope is needed twice: it is `cz[5]`, and it is
    # what `cm_de` shifts with.  `tornado_controls` reports only the wind-axis CL
    # and CD, so the body-z slope is reconstructed from the two of them exactly:
    # with s = sin(alpha), c = cos(alpha),
    #     [dCL; dCD] = [[-s, c], [c, s]] . [dCX; dCZ]   and that matrix is its own
    #     inverse, so dCZ = c*dCL + s*dCD.
    # Verified against a direct two-solve central difference of coeff_create's CZ on
    # the deflected lattices: 0.006486581 per degree both ways at alpha = 4 deg.
    sin_a, cos_a = math.sin(geometry.alpha_deg * math.pi / 180.0), math.cos(
        geometry.alpha_deg * math.pi / 180.0
    )
    cl_de_per_degree = float(elevator["CL"])
    cd_de_per_degree = float(elevator["CD"])
    cz_de_per_degree = cos_a * cl_de_per_degree + sin_a * cd_de_per_degree
    # The slot keeps the standard-aero CL_de the table names; the shift argument
    # is the body-z slope, exactly as for alpha and q.
    cl_de_slot = -per_degree_to_per_radian(cl_de_per_degree)
    cz_de = -per_degree_to_per_radian(cz_de_per_degree)
    cm_de_o = per_degree_to_per_radian(float(elevator["Cm"]))
    _, cm_de, _ = _shift(geometry, cz=cz_de, cy=0.0, cm=cm_de_o, cn=0.0)
    collector.put(
        "cl_de", cl_de_slot, tornado_key="tornado_controls[elevator].CL",
        solver_value=cl_de_per_degree,
        conversion=f"per degree x {DEG_TO_RAD}, then cz = -CL_de; this is the "
                   "standard-aero CL_de the table names, not the body-z slope "
                   "that shifts cm_de",
        role="slope",
    )
    collector.put(
        "cm_de", cm_de, tornado_key="tornado_controls[elevator].Cm",
        solver_value=float(elevator["Cm"]),
        conversion=f"per degree x {DEG_TO_RAD}, cm = +Cm, then shifted with the "
                   f"matching body-z force slope cz = -CZ_de = -(cos a * CL_de + "
                   f"sin a * CD_de) x {DEG_TO_RAD}, reconstructed exactly from the "
                   "row's CL and CD",
        role="slope",
    )

    derivatives = collector.derivative_set()
    coefficients, missing_slots = to_morelli(derivatives)
    if missing_slots:
        raise AssertionError(f"to_morelli zeroed {missing_slots}; the run is incomplete")

    aircraft, initial, controls, trim_block = _trim(coefficients, geometry)
    margin = coefficients["cm"][1] / coefficients["cz"][1]
    if not 0.05 <= margin <= 0.45:
        raise ValueError(
            f"static margin {margin:.4f} cbar is outside 0.05...0.45: the moment-reference "
            "frame or the cz sign is wrong, not the aircraft"
        )

    provenance = {
        "solver": "tornado",
        "solver_version": "aid.tornado (sibling package, imported at call time)",
        "mesh": list(TORNADO_MESH),
        "source": str(Path(source) if source is not None else default_source()),
        "aid_src": str(aid_src()),
        "extra_panel_cl_share": extra_share,
        "flight_condition": {
            "mach": geometry.mach,
            "alt_ft": geometry.alt_ft,
            "betha_rad": geometry.betha_rad,
            "alpha_deg": geometry.alpha_deg,
            "as_mps": geometry.as_mps,
            "as_ftps": geometry.as_ftps,
            "rho_kg_m3": geometry.rho_kg_m3,
            "speed_note": (
                "Tornado's state[\"AS\"] is in FT/S (aid.atmosphere.atmosphere(0)['a'] "
                "= 1116.288876590643 ft/s, _build_state divides by 3.28084), so as_mps is "
                f"as_ftps x {FT_TO_M} and as_ftps carries the solver's own number.  This "
                "is the Mach-0.03 SOURCE speed, not the trimmed cruise speed in "
                "provenance.trim.cruise_mps."
            ),
        },
        "moment_reference": {
            "reference_point": [geometry.x_ref_ft, 0.0, 0.0],
            "cg": [geometry.x_cg_ft, 0.0, 0.0],
            "cg_station_aft_ft": geometry.x_cg_ft,
            "dx_m": geometry.dx_m,
            "dy_m": geometry.dy_m,
            "dz_m": geometry.dz_m,
            "note": "Tornado reports about ref_point with x aft-positive; the CG offset passed "
                    "to shift_moments_to_cg is dx = -ft(2.94) in the dynamics frame (x forward)",
        },
        "sign_convention": {
            "tornado_axes": "x aft, y right, z up (right-handed)",
            "dynamics_axes": "x forward, y right, z down (plane/dynamics.py)",
            "cz": "cz = -CL (body z down); cl = -Cl; cm = +Cm; cn = -Cn",
            "rate_derivatives": "no extra flip: the code perturbs the relative wind with r x "
                                "omega, which is the correct omega x r for a negated velocity",
            "normalisation": "aero_convert.units.normalise_sign against SIGN_INVARIANTS",
        },
        "cl_alpha_cross_check": {
            "coeff_create": cl_alpha,
            "frozen_lattice_secant": secant,
            "relative_difference": abs(secant - cl_alpha) / abs(cl_alpha),
            "tolerance": CROSSCHECK_TOLERANCE,
            "half_width_deg": CROSSCHECK_HALF_DEG,
            "coarse_secant_0_to_2deg": coarse_secant,
            "coarse_relative_difference": abs(coarse_secant - cl_alpha) / abs(cl_alpha),
            "coarse_note": "a full re-run re-tilts the wake with alpha, so the coarse secant is "
                           "not a fixed-wake derivative; it is recorded, not gated at 1 %",
        },
        "cd0": {
            "source": cd0_source,
            "CD_at_alpha0": float(zero["coeffs"]["CD"]),
            "K_wing": float(ac.WG["K"]),
            "induced_part": float(ac.WG["K"]) * cl0 * cl0,
            "source_WG_CD0": float(ac.WG["CD0"]),
            "value": cd0_raw,
        },
        "mass": {
            "source": f"AERO.WT = {geometry.weight_slug:g} slugs",
            "mass_kg": geometry.mass_kg,
            "derivation": (
                f"floor(AERO.WT * {PLAN_STANDARD_GRAVITY_FT_S2} ft/s^2 / "
                f"{LBF_PER_KG} lbf/kg * 1000) / 1000 = floor({geometry.weight_slug:g} * "
                f"{PLAN_STANDARD_GRAVITY_FT_S2} / {LBF_PER_KG} * 1000) / 1000 = "
                f"{geometry.mass_kg}"
            ),
            "exact_kg_truncated_to_3dp": geometry.weight_slug * PLAN_STANDARD_GRAVITY_FT_S2 / LBF_PER_KG,
            "weight_lbf": geometry.weight_slug * PLAN_STANDARD_GRAVITY_FT_S2,
            "slug_conversion_kg": geometry.weight_slug * SLUG_TO_KG,
            "standard_gravity_used_ft_s2": PLAN_STANDARD_GRAVITY_FT_S2,
            "standard_gravity_in_source_ft_s2": G_FT_S2,
            "note": (
                "A slug is a mass unit, so the source's own WT converts as "
                f"AERO.WT * SLUG_TO_KG = {geometry.weight_slug * SLUG_TO_KG:.5f} kg. "
                f"The plan instead states {geometry.mass_kg} kg, which is "
                f"AERO.WT * {PLAN_STANDARD_GRAVITY_FT_S2} ft/s^2 = "
                f"{geometry.weight_slug * PLAN_STANDARD_GRAVITY_FT_S2:g} lbf over "
                f"{LBF_PER_KG} lbf/kg, i.e. it assumes a standard gravity of "
                f"{PLAN_STANDARD_GRAVITY_FT_S2} ft/s^2.  That figure appears nowhere "
                f"in the source (G_FT_S2 = {G_FT_S2} here), so the plan is "
                f"{100.0 * (geometry.weight_slug * SLUG_TO_KG - geometry.mass_kg) / (geometry.weight_slug * SLUG_TO_KG):.2f} % "
                "low.  The plan's number ships because its approved design fixes it "
                "and plan test 6 pins it verbatim in this file's header; weight, "
                "trim qbar, V and t_max_n are therefore all that much low."
            ),
        },
        "inertia": {
            rule: {
                "rule": label,
                "factor": factor,
                "k_m": k,
                "inertia_kg_m2": geometry.mass_kg * k * k,
            }
            for (rule, (factor, label)), k in zip(RADIUS_RULES.items(), _radii(geometry))
        },
        "trim": trim_block,
        "unit_conversions": {
            "length": f"ft * {FT_TO_M}",
            "per_degree_to_per_radian": DEG_TO_RAD,
            "mass": "slug-based source weight, see mass",
        },
        "zeroed_slots": {
            "nonlinear": "every slot in plane.groups.nonlinear_index is exactly 0.0",
            "CD_alpha": "cx has no CD*alpha term in this schema, so CD_alpha is 0.0 and is "
                        "declared rather than approximated",
            "Cn_da": "produced by Tornado; DATCOM cannot produce it",
            "flap": "the Morelli schema has no flap slot; tornado_controls' flap row is kept "
                    "in geometry.jsonc as raw output and mapped to nothing",
        },
        "mapping": collector.mapping,
        "flipped": sorted(collector.flipped),
        "not_normalised": sorted(collector.not_normalised),
        "missing": collector.missing,
    }
    return TornadoRun(
        derivatives=derivatives,
        raw=raw,
        geometry=geometry,
        provenance=provenance,
        aircraft=aircraft,
        initial=initial,
        controls=controls,
        trim=trim_block,
        control_rows=list(rows.values()),
    )


def _planform(ac: Any, extra_share: float) -> dict[str, Any]:
    """Wing, tail and extra-panel essentials, straight out of the source file."""
    def total(value: Any) -> tuple[float | None, int]:
        """A multi-panel planform stores one area per panel; sum them and say how many."""
        if value is None:
            return None, 0
        parts = np.asarray(value, dtype=float).reshape(-1)
        return float(np.sum(parts)), int(parts.size)

    def block(pt: dict) -> dict[str, Any]:
        area, panels = total(pt.get("S"))
        return {
            "chord_root_ft": float(_first(pt["CHRDR"])),
            "chord_break_ft": float(_first(pt["CHRDBP"])),
            "chord_tip_ft": float(_first(pt["CHRDTP"])),
            "span_ft": float(_first(pt["SSPN"])),
            "area_ft2": area,
            "aspect_ratio": None if pt.get("AR") is None else float(_first(pt["AR"])),
            "panel_count": panels,
            "dihedral_deg": float(_first(pt["DHDADI"])),
            "root_quarter_chord_sweep_deg": float(_first(pt["swp"][0])) if "swp" in pt else 0.0,
            "x_station_ft": float(_first(pt["X"])),
            "airfoil": pt.get("NACA"),
        }

    extra = ac.NP[0]
    return {
        "wing": block(ac.WG),
        "horizontal_tail": block(ac.HT),
        "vertical_tail": block(ac.VT),
        "extra_panel": {
            "name": "NP[0]",
            "chord_root_ft": float(_first(extra["CHRDR"])),
            "chord_tip_ft": float(_first(extra["CHRDTP"])),
            "span_ft": float(_first(extra["SSPN"])),
            "area_ft2": float(_first(extra["S"])),
            "aspect_ratio": float(_first(extra["AR"])),
            "dihedral_deg": float(_first(extra["DHDADI"])),
            "x_station_ft": float(_first(extra["X"])),
            "airfoil": extra.get("NACA"),
            "in_lattice": True,
            "cl_share_at_trim": extra_share,
            "kept_because": "the user ruled it in: excluding it is explicitly out of scope",
        },
    }


def _radii(geometry: Geometry) -> tuple[float, float, float]:
    span = geometry.b_ref_m
    return tuple(factor * span for factor, _ in RADIUS_RULES.values())  # type: ignore[return-value]


# Every DerivativeSet field, so a coverage block can state it exhaustively
# instead of by hand.  cm0 is the one field SIGN_INVARIANTS deliberately omits.
ALL_DERIVATIVE_FIELDS = tuple(SIGN_INVARIANTS) + ("cm0",)


def _rate_derivative_note(geometry: Geometry, trim_block: dict, source_speed_mps: float) -> str:
    """The flight-condition caveat on every per-p-hat / per-q-hat / per-r-hat slot.

    Nine slots are genuinely rates: ``cyp[0]``, ``cyr[0]``, ``czq[0]``,
    ``cmq[0]``, ``clp[0]``, ``clr[0]``, ``cnp[0]``, ``cnr[0]``, ``cxq[0]``.  All
    three solvers quote them per ``p*b/(2V)``, ``q*cbar/(2V)`` or ``r*b/(2V)`` at
    the SOURCE Mach-0.03 condition, while ``f16/aero_morelli.py`` forms ``phat``
    at the model's own speed.  The result is that the model's rate damping is
    weaker than the solver's numbers imply.

    ``cy[1]`` and ``cy[2]`` are NOT in that list: they are ``cy_da`` and
    ``cy_dr``, the aileron and rudder side force per radian of CONTROL
    DEFLECTION, which no flight-condition speed touches.

    The factor is written ``v_trim / v_source`` because that is the direction that
    would restore the solver's physical ``dC/dq``: ``czq[0] * cbar/(2*V_trim)`` must
    equal ``CL_q * cbar/(2*V_source)``, so ``czq[0] = CL_q * V_trim/V_source``.
    NOT applied -- the plan fixes the flight condition from the source AERO and
    calls rescaling a physics decision for a later task.  This is the same
    treatment tornado.jsonc records, and it is why the four files agree.
    """
    v_trim = float(trim_block["cruise_mps"])
    return (
        f"NOT rescaled, as in tornado.jsonc.  This solver quotes every rate derivative per "
        f"p*b/(2V), q*cbar/(2V) or r*b/(2V) at the SOURCE Mach-0.03 speed V = "
        f"{source_speed_mps:.4f} m/s, while f16/aero_morelli.py forms phat = p*b_m/(2*V) at the "
        f"trimmed cruise V = {v_trim:.4f} m/s.  Restoring the solver's physical dC/dq would "
        f"mean multiplying the nine per-hat slots by V_trim/V_source = "
        f"{v_trim / source_speed_mps:.5f}, because czq[0]*cbar/(2*V_trim) must equal "
        f"CL_q*cbar/(2*V_source); the model's rate-damping terms are that much weaker than the "
        f"solver implies.  Note the plan writes that factor as AS_source/V_trim, which is the "
        f"INVERSE of the direction that restores dC/dq; nothing is rescaled here either way, "
        f"so the question is only ever a note."
    )


def _generic_flight_condition(
    geometry: Geometry, trim_block: dict, *, source_speed_mps: float
) -> dict[str, Any]:
    """The flight-condition block every cross-check file records."""
    return {
        "mach": geometry.mach,
        "alt_ft": geometry.alt_ft,
        "alpha_deg": geometry.alpha_deg,
        "alpha_schedule_deg": None,
        "alpha_index_rule": SOURCE_ALPHA_INDEX_RULE,
        "betha_rad": geometry.betha_rad,
        "rho_kg_m3": geometry.rho_kg_m3,
        "v_source_mps": source_speed_mps,
        "v_trim_mps": float(trim_block["cruise_mps"]),
        "speed_ratio_v_trim_over_v_source": float(trim_block["cruise_mps"]) / source_speed_mps,
        "a_sound_ftps": geometry.a_sound_ftps,
        "v_true_mps": geometry.v_true_mps,
        "speed_unit_note": (
            "AID's drivers hand over `mach * atm['a'] / 3.28084` = "
            f"{SOURCE_SPEED_AS_REPORTED:.6f} and label it ft/s (Tornado, AVL), which is why "
            "v_source_mps is that number x 0.3048.  The same expression in SI -- which is what "
            f"flow5's deck actually used -- is the physical speed {geometry.v_true_mps:.4f} m/s "
            f"at a sound speed of {geometry.a_sound_ftps:.4f} ft/s.  Both readings are recorded "
            "and neither is reconciled away"
        ),
        "rate_derivative_note": _rate_derivative_note(geometry, trim_block, source_speed_mps),
    }


def _datcom_flight_condition(
    ac: Any, geometry: Geometry, index: int, trim_block: dict
) -> dict[str, Any]:
    """The flight condition for DATCOM, whose two solvers state it themselves."""
    block = _generic_flight_condition(geometry, trim_block, source_speed_mps=geometry.as_mps)
    block["alpha_schedule_deg"] = [float(v) for v in np.asarray(ac.AERO["ALSCHD"]).reshape(-1)]
    block["alpha_index"] = index
    block["fortran_mach"] = 0.03
    block["fortran_alt"] = 0.0
    block["fortran_note"] = (
        "the Fortran's own for006 header echoes `$FLTCON ... ALT=0.00, MACH=0.030`, so the "
        "Fortran part of this file runs at the source condition; the AID handbook part runs at "
        "the same Mach and altitude with AC.alpha = the middle of ALSCHD"
    )
    block["handbook_level_flight_CL"] = None
    return block


def _cross_check_provenance(
    model: str,
    *,
    solver_version: str,
    geometry: Geometry,
    collector: _Collector,
    source: Path | None,
    shared: dict,
    moment_reference: dict[str, Any],
    flight_condition: dict[str, Any],
    trim_block: dict[str, float],
    extra: dict[str, Any],
) -> dict[str, Any]:
    """The provenance block every cross-check file carries, in Tornado's shape."""
    return {
        "solver": model,
        "solver_version": solver_version,
        "mesh": list(geometry.mesh),
        "source": str(Path(source) if source is not None else default_source()),
        "aid_src": shared["aid_src"],
        "flight_condition": flight_condition,
        "moment_reference": moment_reference,
        "sign_convention": {
            "solver_axes": extra.pop("axes", "see item 1 of this file's header comment"),
            "dynamics_axes": "x forward, y right, z down (plane/dynamics.py)",
            "cz": "cz = -CL (body z down); this is the load-bearing identity and it is why "
                  "cz[1] is negative for a stable aircraft",
            "authority": "data/planes/linear/morelli.json is the sign-convention authority where "
                         "it disagrees with data/planes/f16/morelli.json: it wins cm[1] (-0.8 "
                         "against +0.0466) and cnp[0] (-0.03 against +0.0268) and loses cnda[0] "
                         "(+0.01 against -0.0335).  Its cz[1] = -4.5 AGREES with cz = -CL and "
                         "is correct, not a defect",
            "normalisation": "aero_convert.units.normalise_sign against SIGN_INVARIANTS; no "
                             "sign is applied by hand anywhere in this adapter",
            "not_normalised": "cm0 (Cm0) carries no invariant -- the two in-tree reference "
                              "files disagree on it and its sign is tail rigging -- so it is "
                              "written through and recorded in provenance.not_normalised",
        },
        "unit_conversions": {
            "length": f"ft x {FT_TO_M}",
            "per_degree_to_per_radian": DEG_TO_RAD,
            "aid_helpers_not_trusted": "aid.stability._per_deg, aid.handbook_controls._per_deg "
                                       "and aid.handbook_pass._per_rad are all `if a > 1: "
                                       "convert else pass`; every per-degree number here is "
                                       "multiplied by DEG_TO_RAD explicitly instead",
            "mass": "the source's own AERO.WT, see mass",
        },
        "mass": _mass_provenance(geometry),
        "inertia": {
            rule: {"rule": label, "factor": factor, "k_m": k, "inertia_kg_m2": geometry.mass_kg * k * k}
            for (rule, (factor, label)), k in zip(RADIUS_RULES.items(), _radii(geometry))
        },
        "trim": trim_block,
        "zeroed_slots": {
            "nonlinear": "every slot in plane.groups.nonlinear_index is exactly 0.0",
            "CD_alpha": "cx has no CD*alpha term in this schema, so CD_alpha is 0.0 and is "
                        "declared rather than approximated",
            "flap": "the Morelli schema has no flap slot; the handbook/AVL/flow5 flap rows are "
                    "kept in geometry.jsonc as raw output and mapped to nothing",
        },
        "mapping": collector.mapping,
        "flipped": sorted(collector.flipped),
        "not_normalised": sorted(collector.not_normalised),
        "missing": collector.missing,
        **extra,
    }


def _mass_provenance(geometry: Geometry) -> dict[str, Any]:
    """The mass derivation, identical in all four files because it is one source."""
    return {
        "source": f"AERO.WT = {geometry.weight_slug:g} slugs",
        "mass_kg": geometry.mass_kg,
        "derivation": (
            f"floor(AERO.WT * {PLAN_STANDARD_GRAVITY_FT_S2} ft/s^2 / {LBF_PER_KG} lbf/kg * 1000) "
            f"/ 1000 = {geometry.mass_kg}"
        ),
        "exact_kg_truncated_to_3dp": geometry.weight_slug * PLAN_STANDARD_GRAVITY_FT_S2 / LBF_PER_KG,
        "weight_lbf": geometry.weight_slug * PLAN_STANDARD_GRAVITY_FT_S2,
        "slug_conversion_kg": geometry.weight_slug * SLUG_TO_KG,
        "standard_gravity_used_ft_s2": PLAN_STANDARD_GRAVITY_FT_S2,
        "standard_gravity_in_source_ft_s2": G_FT_S2,
        "note": (
            "A slug is a mass unit, so the source's own WT converts as AERO.WT * SLUG_TO_KG = "
            f"{geometry.weight_slug * SLUG_TO_KG:.5f} kg.  The plan instead states "
            f"{geometry.mass_kg} kg, which is AERO.WT * {PLAN_STANDARD_GRAVITY_FT_S2} ft/s^2 "
            f"over {LBF_PER_KG} lbf/kg, i.e. it assumes a standard gravity of "
            f"{PLAN_STANDARD_GRAVITY_FT_S2} ft/s^2.  That figure appears nowhere in the source "
            f"(G_FT_S2 = {G_FT_S2} here), so the plan is 0.54 % low.  The plan's number ships "
            "because its approved design fixes it and plan test 6 pins it verbatim in the "
            "header; weight, trim qbar, V and t_max_n are therefore all that much low.  All "
            "four Cessna files derive it the same way, so they are consistent with each other"
        ),
    }



def tornado_derivatives(source: Path | None = None) -> DerivativeSet:
    """The normalised derivative set Tornado produces for the source geometry."""
    return tornado_run(source).derivatives


# ==============================================================================
# The three cross-check adapters: DATCOM, AVL and flow5
# ==============================================================================
# All three share the Tornado module's shape -- read the source, run a solver,
# map onto DerivativeSet, record every conversion, solve the same trim, write a
# ``.jsonc`` -- and differ only in what their solver can produce and in which of
# the sibling package's own defects sit on the path.  Nothing below reuses a
# coefficient from another model: each file is a cross-check, not a blend.

# AID's Mach-0.03 source speed, as its own drivers hand it over:
# `mach * atm["a"] / 3.28084` with `atm["a"] = 1116.288876590643` ft/s.
# Tornado's and AVL's drivers label that 10.2073 ft/s and then use the reference
# geometry in feet with it, so the p-hat they form is a physical 3.1112 m/s
# speed.  flow5's deck builder puts the SAME number in `qinf_mps` but with the
# whole deck in SI, so there it really is 10.2073 m/s.  Both readings are
# recorded in every file; neither is silently reconciled.
SOURCE_SPEED_AS_REPORTED = 10.207345160909794
SOURCE_SPEED_REPORTED_UNIT = "ft/s in Tornado's and AVL's AID drivers; m/s in flow5's SI deck"


def _source_alpha_index(ac: Any) -> int:
    """Which row of the source's alpha schedule every slot is evaluated at."""
    alschd = np.asarray(ac.AERO["ALSCHD"], dtype=float).reshape(-1)
    if alschd.size == 0:
        raise ValueError("AERO.ALSCHD is empty; there is no alpha to evaluate at")
    return max(0, math.ceil(alschd.size / 2) - 1)


def _reference_geometry(ac: Any, *, mesh: tuple[str, str]) -> Geometry:
    """The SI reference geometry and source flight condition, from ``AERO`` alone.

    Every moment here is already about the CG, so ``dx = dy = dz = 0`` -- but the
    plan requires that be VERIFIED, not assumed, so each adapter calls
    ``_verify_cg_reference`` against the station its own solver read and passes
    the check in here.  ``as_mps`` is AID's own number (see
    ``SOURCE_SPEED_REPORTED_UNIT``); ``v_true_mps`` is the physical speed
    ``mach * a_sound``, which is what flow5's SI deck actually used.
    """
    aero = ac.AERO
    atm = _aid_atmosphere(float(_first(aero["ALT"])))
    mach = float(_first(aero["MACH"]))
    return Geometry(
        s_ref_m2=ft(float(_first(aero["SREF"]))),
        b_ref_m=ft(float(_first(aero["BLREF"]))),
        c_ref_m=ft(float(_first(aero["CBARR"]))),
        dx_m=0.0,
        dy_m=0.0,
        dz_m=0.0,
        x_cg_ft=float(_first(aero["XCG"])),
        x_ref_ft=float(_first(aero["XCG"])),
        x_wing_ft=float(_first(aero["XW"])),
        x_tail_ft=float(_first(aero["XH"])),
        mach=mach,
        alt_ft=float(_first(aero["ALT"])),
        betha_rad=0.0,
        alpha_deg=float(_first(ac.AERO["alpha"])),
        # AID's own drivers hand over `mach * atm["a"] / 3.28084` = 10.2073 and label
        # it ft/s while working the reference geometry in feet, so the speed in THIS
        # model's metre frame is 10.2073 x 0.3048 = 3.1112 m/s.  That is the reading
        # tornado.jsonc uses and it is what AVL's own `v 10.2073` with an Sref of
        # 24.000 means.  flow5's deck is SI instead, and is given v_true_mps.
        as_mps=SOURCE_SPEED_AS_REPORTED * FT_TO_M,
        as_ftps=SOURCE_SPEED_AS_REPORTED,
        rho_kg_m3=float(atm["D"]) * 515.379,
        mass_kg=source_mass_kg(aero),
        weight_slug=float(_first(aero["WT"])),
        mesh=mesh,
        v_true_mps=mach * float(atm["a"]) * FT_TO_M,
        a_sound_ftps=float(atm["a"]),
    )


def _aid_atmosphere(alt_ft: float) -> dict:
    """The sibling's own altitude atmosphere, imported at call time."""
    from aid.atmosphere import atmosphere

    return atmosphere(alt_ft)


def _last(value: Any) -> float:
    """The last entry of a multi-panel stored field, as MATLAB's ``field(end)``."""
    return float(np.asarray(value, dtype=float).reshape(-1)[-1])


def _verify_cg_reference(
    geometry: Geometry, station_ft: float, *, solver: str, tolerance_ft: float = 1e-4
) -> dict[str, Any]:
    """Fail loudly unless ``station_ft`` really is the CG this shift assumes.

    The plan says the moment reference must be VERIFIED, not assumed.  Tornado's
    reference is not the CG and is corrected by a shift; these three are, and the
    only honest way to say so is to read the station back out of what the solver
    was given and compare it with ``AERO.XCG``.

    The tolerance is 1e-4 ft, four orders of magnitude below the 2.94 ft station,
    and it exists only because flow5's deck goes through
    ``aid.flow5_units.ft_to_m`` = ``1/3.28084`` rather than 0.3048: a round trip
    metres -> feet therefore returns 2.9399999 ft rather than 2.94, a relative
    error of 2e-6 in the helper, not a disagreement about where the CG is.
    """
    if abs(float(station_ft) - geometry.x_cg_ft) > tolerance_ft:
        raise ValueError(
            f"{solver} reports about x = {station_ft} ft but AERO.XCG = {geometry.x_cg_ft} ft "
            f"(tolerance {tolerance_ft} ft); the moment reference would silently be wrong"
        )
    return {
        "solver_station_ft": float(station_ft),
        "aero_xcg_ft": geometry.x_cg_ft,
        "tolerance_ft": tolerance_ft,
        "dx_m": 0.0,
        "dy_m": 0.0,
        "dz_m": 0.0,
        "note": f"{solver} reports every moment about AERO.XCG, read back from its own input and "
                "checked against the source; dx = dy = dz = 0, so no moment shift is applied",
    }


def _datcom_section7(
    ac: Any, cyb: float, cl_source: float, *, sweep_c4_deg: float, vt_a_per_rad: float
) -> dict[str, float]:
    """DATCOM Section 7 rate derivatives, computed here with the factor applied.

    ``aid/longitudinal_dynamic.py`` and ``aid/lateral.lateral_dynamic`` implement
    exactly these formulae but feed them the sibling's own per-DEGREE lift slopes,
    because ``aid/handbook_pass.py:131``'s ``_per_rad`` is
    ``if a > 1.0: a * pi/180 else a`` -- a no-op below 1.0, and this planform's
    lift slopes are 0.076-0.084 /deg.  Measured consequences on this aircraft:
    ``CLq`` comes out 0.104 instead of 5.950 (57.3x) and ``Clp`` 0.0139 instead
    of 0.799.  So the formulae are transcribed here and every lift slope is
    multiplied by ``DEG_TO_RAD`` by this converter, which is what the plan's
    "Global Constraints" require.

    The two groups are the standard DATCOM Section 7 non-dimensional rate
    derivatives, both per unit of p-hat = p*b/(2V), q-hat = q*cbar/(2V) and
    r-hat = r*b/(2V), with DATCOM's factor 1.1 on the tail's unsteady term:

        CL_q   =  2 * a_HT * eta * (S_HT * l_HT / (S * cbar)) * 1.1
        CM_q   = -CL_q * l_HT / cbar
        CL_p   = -a_WG/12 * (1 + 3*TR)/(1 + TR)
        CY_p   =  CL * (AR + cosL)/(AR + 4*cosL) * tanL + 2*CYb/b * (h - h_p)
        CY_r   = -2 * CYb * l / b
        Cl_r   =  CL/4 - 2 * CYb * l * h / b^2
        Cn_p   = -CL/8 - 2 * CYb * l * (h - h_p) / b^2
        Cn_r   = -2 * a_VT * swash * (S_VT/S) * (l/b)^2

    ``vt_a_per_rad`` is the CORRECTED per-radian vertical-tail lift slope, because
    ``aid/lateral.py:571`` multiplies it by ``pi/180`` in the wrong direction (see
    ``_datcom_lateral``), so it cannot be read back off the aircraft.
    """
    wg, ht, vt = ac.WG, ac.HT, ac.VT
    s_ref = float(_last(wg["S"]))
    cbar = float(_last(wg["cbar"]))
    span = float(wg["b"])
    taper = float(_last(wg["TR"]))
    aspect = float(_last(wg["AR"]))
    eta = float(ht.get("eta", 0.9))
    ht_arm = float(ht["l"])
    ht_a = per_degree_to_per_radian(float(ht["a"]))
    wg_a = per_degree_to_per_radian(float(wg["a"]))
    # aid/handbook_pass.py's own `ht_V`, the DATCOM velocity-ratio parameter.
    ht_v = float(_last(ht["S"])) * ht_arm / (s_ref * cbar)
    cyb = float(cyb)
    cl = float(cl_source)
    vt_h, vt_hp, vt_l = float(vt["h"]), float(vt["hp"]), float(vt["l"])
    cos_l = math.cos(math.radians(float(sweep_c4_deg)))
    cl_q = 2.0 * ht_a * eta * ht_v * DATCOM_TAIL_UNSTEADY_FACTOR
    return {
        "cl_q": cl_q,
        "cm_q": -cl_q * ht_arm / cbar,
        "cl_p": -wg_a / 12.0 * (1.0 + 3.0 * taper) / (1.0 + taper),
        "cy_p": cl * (aspect + cos_l) / (aspect + 4.0 * cos_l) * math.sin(math.radians(float(sweep_c4_deg))) / cos_l
        + 2.0 * cyb / span * (vt_h - vt_hp),
        "cy_r": -2.0 * cyb * vt_l / span,
        "cl_r": cl / 4.0 - 2.0 * cyb * vt_l * vt_h / span**2,
        "cn_p": -cl / 8.0 - 2.0 * cyb * vt_l * (vt_h - vt_hp) / span**2,
        "cn_r": -2.0 * vt_a_per_rad * float(vt["swash"])
        * float(_last(vt["S"])) / s_ref * (vt_l / span) ** 2,
        # not derivatives: the inputs, kept so every number above is re-derivable
        "ht_v": ht_v,
        "ht_arm_ft": ht_arm,
        "ht_a_per_rad": ht_a,
        "wg_a_per_rad": wg_a,
        "vt_a_per_rad": vt_a_per_rad,
        "vt_h_ft": vt_h,
        "vt_hp_ft": vt_hp,
        "vt_l_ft": vt_l,
        "vt_swash": float(vt["swash"]),
        "vt_s_last": float(_last(vt["S"])),
        "sweep_c4_deg": float(sweep_c4_deg),
        "cl_source": cl,
        "cyb": cyb,
    }


# DATCOM's own factor of 1.1 on the horizontal tail's unsteady lift term, from
# aid/longitudinal_dynamic.py (which ports Longitudinal_Dynamic_Stability.m).
DATCOM_TAIL_UNSTEADY_FACTOR = 1.1


def _datcom_lateral(lateral: dict, ac: Any, vt_a_per_rad: float) -> dict[str, float]:
    """DATCOM's lateral static set, with the vertical-tail lift slope corrected.

    ``aid/lateral.py:571`` ends its tail lift-slope formula with
    ``vt_a = vt_a * pi / 180``.  That formula returns the same units as the
    ``k = a0/(2*pi)`` it was fed, and AID feeds it ``VT.a0`` = 6.8396 DEGREES, so
    the formula already returns a per-degree number and the ``* pi/180`` makes it
    57.3x too small.  Measured on this aircraft, with the correction
    ``Clb = -0.0680`` against the sibling's own AVL gold ``Clb = -0.065868``; with
    the sibling's value, ``Clb = -0.0031``, a factor 21.8 off the same gold.

    Every quantity ``lateral_static`` returns that is LINEAR in that slope is
    therefore corrected, and the rest is left alone because it is already per
    radian:

    * ``cyb_v`` -- the vertical tail's side-force slope, ``-k*a*swash*Sv/S``;
    * ``CYb``, which is ``cyb_wb + cyb_v`` and whose ``cyb_wb`` (the
      wing-body term) is per radian already;
    * ``Cnb``, whose only tail term is ``-cyb_v*l/b``;
    * ``Clb``, whose only tail term is ``+cyb_v*h/b``.
    """
    vt = ac.VT
    cyb_v_as_reported = (
        -float(vt["k"]) * float(vt["a"]) * float(vt["swash"])
        * float(_last(vt["S"])) / float(_last(ac.WG["S"]))
    )
    cyb_v = cyb_v_as_reported * DEG_TO_RAD
    vt_h, vt_l = float(vt["h"]), float(vt["l"])
    span = float(ac.WG["b"])
    cyb_wb = float(lateral["CYb"]) - cyb_v_as_reported
    return {
        "cy_beta": cyb_wb + cyb_v,
        "cn_beta": float(lateral["Cnb"]) - cyb_v_as_reported * vt_l / span + cyb_v * vt_l / span,
        "cl_beta": float(lateral["Clb"]) - cyb_v_as_reported * vt_h / span + cyb_v * vt_h / span,
        "cyb_v_as_reported": cyb_v_as_reported,
        "cyb_v": cyb_v,
        "cyb_wb": cyb_wb,
        "vt_a_as_reported": float(vt["a"]),
        "vt_a_per_rad": vt_a_per_rad,
        "units_factor": DEG_TO_RAD,
    }


def _sweep_c4_deg(ac: Any) -> float:
    """The wing's sweep at the quarter chord, as MATLAB's ``WG.swp(2,end)``."""
    swp = np.asarray(ac.WG["swp"], dtype=float)
    if swp.ndim == 1:
        swp = swp.reshape(-1, 1)
    return float(swp[1, -1])


def _cross_check_inputs(source: Path | None) -> tuple[Any, dict, SimpleNamespace, str]:
    """Load the source and the sibling entry points the three adapters share.

    Returns the aircraft (with ``AERO['alpha']`` set to the middle of
    ``ALSCHD``, so every solver is asked about the same incidence), the alias
    description, a namespace of the imported entry points, and a one-line
    description of what the numpy bridge did.
    """
    _numpy_alias()
    root = _aid_on_path()
    from aid.aircraft import load_jsonc

    path = Path(source) if source is not None else default_source()
    if not path.is_file():
        raise FileNotFoundError(f"{path} is missing; the sibling analysis tree is read-only input")
    ac = load_jsonc(path)
    ac.AERO["alpha"] = float(np.asarray(ac.AERO["ALSCHD"], dtype=float).reshape(-1)[_source_alpha_index(ac)])
    return ac, {"aid_src": str(root)}, SimpleNamespace(load_jsonc=load_jsonc), _numpy_alias()


def datcom_run(source: Path | None = None) -> CrossCheckRun:
    """Digital DATCOM over the source: the real Fortran plus the AID handbook.

    Two solvers, because they answer different questions and each has a defect
    that must be corrected rather than inherited:

    * ``aid.datcom_run.run_datcom`` runs the Fortran binary on a card file this
      converter writes into a ``TemporaryDirectory``.  Its stability table is
      **per degree** -- measured against its own ``cl`` column, ``cla`` equals
      the secant ``dCL/dalpha`` in per degree to 3 decimal places -- so every
      derivative read out of it is multiplied by ``DEG_TO_RAD`` here.  It is the
      source of the longitudinal static set and the two intercepts.
    * ``aid.handbook_pass.apply_handbook`` supplies what the Fortran table does
      not carry: the vertical-tail geometry that DATCOM Section 7 needs, the
      parasite drag build-up, and the control derivatives.

    The AID handbook's own longitudinal and lateral STATIC results
    (``aircraft_stability``'s ``CLa``/``Cma``/``CL0``/``Cm0`` and
    ``lateral_static``'s ``CYb``/``Cnb``/``Clb``) are NOT taken as they stand:
    all of them are built from ``stability._per_deg``, the same
    ``if a > 1: convert else pass`` no-op, so they are 57.3x too small.  What is
    taken from them is the part that is dimensionless -- ``x_cg``, ``K``, ``S``,
    ``Q``, the level-flight ``CL``, and the VT geometry -- and the affected
    slopes are recomputed.  Every one of the uncorrected numbers is recorded in
    ``provenance.handbook_cross_check`` so the correction is auditable.
    """
    ac, shared, aid, numpy_note = _cross_check_inputs(source)
    from aid.drag import aircraft_cd0
    from aid.handbook_controls import handbook_controls
    from aid.handbook_pass import apply_handbook

    collector = _Collector()
    with _scratch() as tmp:
        table = _run_datcom(ac, tmp / "datcom")
    stability = apply_handbook(
        ac, angl=False, slipstream=False, slipstream_data=(0.5, 0.9),
        multhopp=True, trim_mode=0, trim_fix="both",
    )
    lateral = dict(stability["lateral"])
    geometry = _reference_geometry(ac, mesh=("none", "none"))
    moment_reference = _verify_cg_reference(geometry, float(_first(ac.AERO["XCG"])), solver="DATCOM")

    vt_a_per_rad = float(ac.VT["a"]) * DEG_TO_RAD
    lateral_fixed = _datcom_lateral(lateral, ac, vt_a_per_rad)
    rates = _datcom_section7(ac, lateral_fixed["cy_beta"], float(stability["CL"]),
                             sweep_c4_deg=_sweep_c4_deg(ac), vt_a_per_rad=vt_a_per_rad)
    # The neutral-deflection row is the one taken, so the record names delta = 0
    # rather than whatever `iter_rows` happened to return last.
    all_rows = handbook_controls(ac)
    rows = {
        row["surface"]: row for row in all_rows
        if float(row["delta_deg"]) == 0.0
    } or {row["surface"]: row for row in all_rows}

    index = _source_alpha_index(ac)
    raw: dict[str, Any] = {
        "solver_alpha_schedule_deg": [float(v) for v in np.asarray(ac.AERO["ALSCHD"]).reshape(-1)],
        "solver_alpha_index": index,
        "solver_alpha_deg": float(geometry.alpha_deg),
        # Digital DATCOM's own table, verbatim and PER DEGREE.
        "datcom_table_alpha_deg": [float(v) for v in np.asarray(table["alpha"]).reshape(-1)],
        "datcom_cl": [float(v) for v in np.asarray(table["cl"]).reshape(-1)],
        "datcom_cd": [float(v) for v in np.asarray(table["cd"]).reshape(-1)],
        "datcom_cm": [float(v) for v in np.asarray(table["cm"]).reshape(-1)],
        "datcom_cn": [float(v) for v in np.asarray(table["cn"]).reshape(-1)],
        "datcom_cla_per_degree": [float(v) for v in np.asarray(table["cla"]).reshape(-1)],
        "datcom_cma_per_degree": [float(v) for v in np.asarray(table["cma"]).reshape(-1)],
        "datcom_clb_per_degree": [float(v) for v in np.asarray(table["clb"]).reshape(-1)],
        "datcom_cyb_per_degree": [float(v) for v in np.asarray(table["cyb"]).reshape(-1)],
        "datcom_cnb_per_degree": [float(v) for v in np.asarray(table["cnb"]).reshape(-1)],
        "datcom_mach": float(table["mach"]),
        "datcom_alt": float(table["alt"]),
        # AID's own handbook numbers, before and after correction.
        "aid_CLa_as_reported": float(stability["CLa"]),
        "aid_Cma_as_reported": float(stability["Cma"]),
        "aid_CL0_as_reported": float(stability["CL0"]),
        "aid_Cm0_as_reported": float(stability["Cm0"]),
        "aid_lateral_as_reported": {k: float(lateral[k]) for k in ("CYb", "Cnb", "Clb")},
        "lateral_corrected": lateral_fixed,
        "section7": rates,
        "lateral_dynamic_as_reported": {
            key: float(lateral[key])
            for key in ("CYp", "CYr", "Clp", "Clr", "Cnp", "Cnr")
        },
        "aid_longitudinal_dynamic_CLq": float(stability["dynamic"]["CLq"]),
        "aid_drag_CD0": float(ac.WG["CD0"]),
        "aid_K": float(stability["K"]),
        "aid_x_cg": float(stability["x_cg"]),
        "aid_CL_level_flight": float(stability["CL"]),
        "handbook_controls_per_degree": {
            surface: {key: rows[surface].get(key) for key in ("CL", "CD", "Cm", "CY", "Cl", "Cn")}
            for surface in ("aileron", "elevator", "rudder")
        },
    }

    # --- longitudinal static, from the Fortran -------------------------------
    cla_per_deg = float(np.asarray(table["cla"]).reshape(-1)[index])
    cma_per_deg = float(np.asarray(table["cma"]).reshape(-1)[index])
    collector.put(
        "cl_alpha", -per_degree_to_per_radian(cla_per_deg),
        tornado_key=f"DATCOM for006 CLA at alpha = {geometry.alpha_deg:g} deg",
        solver_value=cla_per_deg,
        conversion=f"per degree x {DEG_TO_RAD}, then cz = -CL_alpha; the Fortran table is "
                   "per degree (measured against its own cl column), and cz = -CL because "
                   "plane/dynamics.py's z axis points down",
        role="slope",
    )
    collector.put(
        "cm_alpha", per_degree_to_per_radian(cma_per_deg),
        tornado_key=f"DATCOM for006 CMA at alpha = {geometry.alpha_deg:g} deg",
        solver_value=cma_per_deg,
        conversion=f"per degree x {DEG_TO_RAD}; DATCOM's Cm is nose-up positive, the same "
                   "sense as plane/dynamics.py's cm, so no axis sign change",
        role="slope",
    )
    # CL0 and Cm0 are INTERCEPTS and the table has an alpha = 0 row, so both come
    # straight out of it with no extrapolation.
    cl_at_zero = float(np.asarray(table["cl"]).reshape(-1)[1])
    cm_at_zero = float(np.asarray(table["cm"]).reshape(-1)[1])
    collector.put(
        "cl0", -cl_at_zero, tornado_key="DATCOM for006 CL at alpha = 0 deg",
        solver_value=cl_at_zero,
        conversion="cz = -CL; the alpha = 0 row of the table is the intercept outright",
        role="intercept",
    )
    collector.put(
        "cm0", cm_at_zero, tornado_key="DATCOM for006 CM at alpha = 0 deg",
        solver_value=cm_at_zero,
        conversion="the alpha = 0 row of the table; Cm0's sign is tail rigging, not a "
                   "derivable convention, so it is never sign-normalised",
        role="intercept",
    )

    # --- parasite drag --------------------------------------------------------
    # aid/drag.py's own build-up, 1.25 x the sum of the section drag estimates.
    # The Fortran's CD column at alpha = 0 minus its induced part is the same
    # quantity computed the other way round and is recorded beside it; the two
    # differ by a factor 2.09 because aid/drag.py builds its skin-friction
    # coefficient from a Reynolds number PER UNIT LENGTH (atm['Re'] = D*mach*a/V,
    # not D*mach*a*cbar) and then applies DATCOM's 1.25 factor on top.
    parasite = float(ac.WG["CD0"])
    induced = float(stability["K"]) * cl_at_zero * cl_at_zero
    cd_total_at_zero = float(np.asarray(table["cd"]).reshape(-1)[1])
    collector.put(
        "cd0", -parasite, tornado_key="aid.drag.aircraft_cd0 (via aid.handbook_pass)",
        solver_value=parasite,
        conversion="parasite drag as a positive magnitude, then cx = -(parasite)",
        role="intercept",
    )

    # --- lateral static, from the handbook, with the VT lift-slope correction ---
    for field, key, correct in (
        ("cl_beta", "Clb", "cl_beta"),
        ("cy_beta", "CYb", "cy_beta"),
        ("cn_beta", "Cnb", "cn_beta"),
    ):
        collector.put(
            field, lateral_fixed[correct], tornado_key=f"aid.lateral.lateral_static.{key}",
            solver_value=float(lateral[key]),
            conversion="AID's value with its own vertical-tail term re-evaluated using the "
                       f"tail's lift slope x {DEG_TO_RAD}: aid/lateral.py:571 multiplies that "
                       f"slope by pi/180 in the WRONG direction, so AID's {key} = "
                       f"{float(lateral[key]):.6g} is 57.3x too small in its tail part (AID's "
                       "Clb = -0.003091 against the sibling's own AVL gold Clb = -0.065868, "
                       "and against AVL's alpha = 4 deg row -0.045756 this adapter writes "
                       "-0.068966)",
            role="slope",
        )

    # --- DATCOM Section 7 rate derivatives ------------------------------------
    for field, key, sign in (
        ("cl_q", "cl_q", -1.0), ("cm_q", "cm_q", 1.0),
        ("cy_p", "cy_p", 1.0), ("cy_r", "cy_r", 1.0),
        ("cl_p", "cl_p", 1.0), ("cl_r", "cl_r", 1.0),
        ("cn_p", "cn_p", 1.0), ("cn_r", "cn_r", 1.0),
    ):
        per_radian = rates[key]
        collector.put(
            field, sign * per_radian,
            tornado_key=f"DATCOM Section 7 {key.upper()} (this converter)",
            solver_value=None,
            conversion=(
                f"DATCOM Section 7 formula, computed here with every lift slope x "
                f"{DEG_TO_RAD}; aid's own {per_radian / DEG_TO_RAD:.6g} comes from a "
                f"_per_rad no-op and is {DEG_TO_RAD:.2f}x too small"
            ),
            role="slope",
        )
    # cd_q: DATCOM Section 7 has NO axial rate derivative.  aid/longitudinal_dynamic.py
    # hardcodes cxq = 0.0, which is the absence of a formula rather than a computed
    # zero, so the field is None and the slot is declared -- a computed 0.0 would also
    # sit exactly on the invariant row cxq[0] > 0, which no number can satisfy.
    collector.absent(
        "cd_q", "DATCOM Section 7 (no axial rate term exists)",
        reason="DATCOM's Section 7 formulae carry no axial rate derivative at all and "
               "aid/longitudinal_dynamic.py hardcodes cxq = 0.0, which records the absence "
               "of a formula rather than a computed value",
    )

    # --- control derivatives --------------------------------------------------
    # One uniform rule for every handbook_controls row: each value is linear in a
    # lift slope that AID stores PER DEGREE (WG.a, HT.a) or per AID's mis-scaled
    # VT.a, so each is per degree and is multiplied by DEG_TO_RAD here.
    for field, surface, key in (
        ("cl_da", "aileron", "Cl"),
        ("cl_de", "elevator", "CL"),
        ("cm_de", "elevator", "Cm"),
        ("cl_dr", "rudder", "Cl"),
        ("cy_dr", "rudder", "CY"),
        ("cn_dr", "rudder", "Cn"),
    ):
        per_degree = float(rows[surface][key])
        collector.put(
            field, per_degree_to_per_radian(per_degree),
            tornado_key=f"aid.handbook_controls[{surface}].{key}",
            solver_value=per_degree,
            conversion=f"per degree x {DEG_TO_RAD}; the row is linear in a lift slope AID "
                       "stores per degree, so it inherits that unit.  DATCOM's and "
                       "plane/dynamics.py's moment senses agree, so no frame sign is applied "
                       "here and normalise_sign decides the sign",
            role="slope",
        )
    for field, key in (("cn_da", "Cn"), ("cy_da", "CY")):
        collector.absent(
            field, f"aid.handbook_controls[aileron].{key}",
            reason=(
                "aid/handbook_controls.py has no aileron "
                + ("yaw" if key == "Cn" else "side force")
                + " column at all, and aid/lateral.py:616's Clda/Cnda = 0.1 is a hardcoded "
                "placeholder, not a computation; using it would put a fabricated number in a "
                "coefficient array"
            ),
        )

    derivatives = collector.derivative_set()
    coefficients, missing_slots = to_morelli(derivatives)
    if missing_slots != ["cnda[0]", "cxq[0]", "cy[1]"]:
        raise AssertionError(
            f"to_morelli zeroed {missing_slots}; only cn_da, cy_da and cd_q may be absent"
        )

    aircraft, initial, controls, trim_block = _trim(coefficients, geometry, "datcom")
    margin = coefficients["cm"][1] / coefficients["cz"][1]
    if not 0.05 <= margin <= 0.45:
        raise ValueError(
            f"static margin {margin:.4f} cbar is outside 0.05...0.45: the moment-reference "
            "frame or the cz sign is wrong, not the aircraft"
        )
    raw["planform"] = _planform(ac, 0.0)
    provenance = _cross_check_provenance(
        "datcom",
        solver_version="Digital DATCOM Fortran via aid.datcom_run.run_datcom, plus "
                       "aid.handbook_pass.apply_handbook (imp sibling)",
        geometry=geometry,
        collector=collector,
        source=source,
        shared=shared,
        moment_reference=moment_reference,
        flight_condition=_datcom_flight_condition(ac, geometry, index, trim_block),
        trim_block=trim_block,
        extra={
            "axes": "DATCOM: x forward, y right, z DOWN, positive Cm nose up, positive Cl "
                    "roll right, positive Cn yaw right -- identical to plane/dynamics.py, so "
                    "this adapter applies no axis mapping at all",
            "handbook_cross_check": {
                "aid_CLa_as_reported": float(stability["CLa"]),
                "aid_CLa_corrected": float(stability["CLa"]) * DEG_TO_RAD,
                "datcom_CLa_used": per_degree_to_per_radian(cla_per_deg),
                "aid_Cma_as_reported": float(stability["Cma"]),
                "aid_Cma_corrected_by_factor": float(stability["Cma"]) * DEG_TO_RAD,
                "datcom_Cma_used": per_degree_to_per_radian(cma_per_deg),
                "aid_CL0_as_reported": float(stability["CL0"]),
                "aid_Cm0_as_reported": float(stability["Cm0"]),
                "aid_lateral_as_reported": {k: float(lateral[k]) for k in ("CYb", "Cnb", "Clb")},
                "lateral_corrected": lateral_fixed,
                "aid_longitudinal_dynamic_CLq": float(stability["dynamic"]["CLq"]),
                "aid_lateral_dynamic_Clp": float(lateral["Clp"]),
                "section7_cl_q_used": rates["cl_q"],
                "section7_cl_p_used": rates["cl_p"],
                "units_note": "aid's values are built from stability._per_deg / "
                              "handbook_pass._per_rad, both `if a > 1: convert else pass`, so "
                              "every one of them is DEG_TO_RAD too small; the ones this adapter "
                              "needs are recomputed and the rest are recorded here",
            },
            "cd0": {
                "source": "aid.drag.aircraft_cd0, which is 1.25 x the summed section drag",
                "value": parasite,
                "datcom_own_CD_at_alpha0": cd_total_at_zero,
                "induced_part": induced,
                "datcom_own_parasite": cd_total_at_zero - induced,
                "ratio_datcom_to_aid": (cd_total_at_zero - induced) / parasite,
                "note": "aid/drag.py builds its skin-friction coefficient from a Reynolds "
                        "number PER UNIT LENGTH (atm['Re'] = D*mach*a/V) rather than per "
                        "chord, then multiplies the sum by DATCOM's 1.25; the Fortran's own "
                        "CD column at alpha = 0 less its induced part is the same quantity "
                        "computed the other way, and both are recorded",
            },
            "numpy_bridge": numpy_note,
            "control_rows": {surface: dict(rows[surface]) for surface in rows},
        },
    )
    return CrossCheckRun(
        model="datcom",
        derivatives=derivatives,
        coefficients=coefficients,
        raw=raw,
        geometry=geometry,
        provenance=provenance,
        aircraft=aircraft,
        initial=initial,
        controls=controls,
        trim=trim_block,
    )


def _run_datcom(ac: Any, workdir: Path) -> dict:
    """Run the real Fortran binary in scratch and return its parsed stability table.

    ``aid.datcom_run.run_datcom`` writes ``for005.dat`` into ``workdir`` and shells
    out to ``Matlab/fsroot/code/DATCOM/datcom`` there.  A missing binary surfaces
    as ``FileNotFoundError`` from ``aid.paths.datcom_wrapper`` and is recorded by
    the caller as a missing binary rather than raised into a crash, per the plan's
    "Fail-closed".
    """
    from aid.datcom_run import run_datcom

    return run_datcom(ac, workdir)


def datcom_derivatives(source: Path | None = None) -> DerivativeSet:
    """The normalised derivative set DATCOM produces for the source geometry."""
    return datcom_run(source).derivatives


def avl_run(source: Path | None = None) -> CrossCheckRun:
    """AVL over the source: stability derivatives from the ``.st`` sweep, control from the ``.sb``.

    AVL is the one cross-check solver whose axis convention ALREADY matches
    ``plane/dynamics.py``: its own ``.sb`` header prints "Standard axis
    orientation, X fwd, Z down", the same x-forward, y-right, z-down frame and the
    same moment senses (positive ``Cl`` rolls right, positive ``Cm`` noses up,
    positive ``Cn`` yaws right).  So no axis mapping is applied at all -- the only
    negation anywhere is on the three slots the schema stores in the wind-axis
    lift convention (``cz[1]``, ``czq[0]``, ``cz[5]``, i.e. ``cz = -CL``), and
    ``normalise_sign`` still decides the sign of every slot from the invariant
    table.

    Two runs, because AVL reports them separately:

    * ``aid.avl_io.run_avl_full`` sweeps ``AERO.ALSCHD`` and writes one ``.st``
      per alpha; the alpha = 4 deg row supplies every stability derivative, and
      the alpha = 0 row supplies the two intercepts and the drag breakdown.
    * ``aid.avl_controls.avl_controls`` writes a CONTROL card for every legal
      surface, deflects all of them by the same delta, and reads the per-DEGREE
      deflection columns out of its own ``.sb``.  ``run_avl_full``'s ``.sb``
      carries NO control columns (measured: its run case lists no deflected
      surface, so ``parse_sb`` returns an empty ``surface`` list), which is why
      the second run exists at all -- and it is also the run whose aileron
      CONTROL card is written with all seven numbers, which is what makes AVL
      read the trailing ``-1`` as the duplicate sign rather than as a hinge
      component.
    """
    ac, shared, _aid, numpy_note = _cross_check_inputs(source)
    from aid.avl_controls import avl_controls
    from aid.avl_io import run_avl_full
    from aid.paths import avl_bin

    with _scratch() as tmp:
        run_dir = tmp / "avl"
        run_dir.mkdir(parents=True, exist_ok=True)
        if not avl_bin().is_file():
            raise FileNotFoundError(f"the AVL binary is not at {avl_bin()}")
        swept = run_avl_full(ac, AVL_MESH, run_dir)
        xref = _avl_xref(run_dir / "geometry.avl")
        sb_text = (run_dir / "geometry.sb").read_text()
        control_rows = [
            row for row in avl_controls(ac, deltas_deg=(0.0,), mesh=AVL_MESH)
        ]
    by_surface = {row["surface"]: row for row in control_rows}
    geometry = _reference_geometry(ac, mesh=AVL_MESH)
    moment_reference = _verify_cg_reference(geometry, xref, solver="AVL")

    schedule = [float(v) for v in np.asarray(swept["alpha"]).reshape(-1)]
    index = schedule.index(geometry.alpha_deg) if geometry.alpha_deg in schedule else 0
    zero = schedule.index(0.0)

    def at(key: str, row: int = None) -> float:
        values = np.asarray(swept[key], dtype=float).reshape(-1)
        return float(values[index if row is None else row])

    collector = _Collector()
    raw: dict[str, Any] = {
        "mesh": list(AVL_MESH),
        "alpha_schedule_deg": schedule,
        "alpha_index": index,
        "alpha_deg": geometry.alpha_deg,
        "x_reference_ft": xref,
        "avl_axes": "X fwd, Y right, Z down -- identical to plane/dynamics.py",
        "reference_case": {k: v for k, v in swept["ref"].items() if k != "surface"},
        "sb_geometry_axis": {
            name: _sb_value(sb_text, name)[0]
            for name in ("CXu", "CYu", "CZu", "CXq", "CYq", "CZq", "Cnp", "Cnr")
        },
        "st_alpha_rows": {
            key: [float(v) for v in np.asarray(swept[key], dtype=float).reshape(-1)]
            for key in sorted(swept)
            if key not in ("ref", "surface", "alpha")
            and np.asarray(swept[key]).dtype.kind == "f"
        },
        "control_rows_per_degree": {
            surface: {key: by_surface[surface].get(key)
                      for key in ("CL", "CD", "Cm", "CY", "Cl", "Cn")}
            for surface in ("aileron", "elevator", "rudder")
        },
        "control_surfaces_available": sorted(by_surface),
    }

    collector.put(
        "cl_alpha", -at("CLa"), tornado_key=f"AVL .st CLA at alpha = {geometry.alpha_deg:g} deg",
        solver_value=at("CLa"),
        conversion="cz = -CL_alpha; AVL's axes already match plane/dynamics.py, so this is "
                   "the only negation in the whole adapter",
        role="slope",
    )
    collector.put(
        "cm_alpha", at("Cma"), tornado_key=f"AVL .st Cma at alpha = {geometry.alpha_deg:g} deg",
        solver_value=at("Cma"),
        conversion="AVL's Cm is nose-up positive, the same sense as plane/dynamics.py's cm",
        role="slope",
    )
    collector.put(
        "cl0", -at("CLtot", zero), tornado_key="AVL .st CLtot at alpha = 0 deg",
        solver_value=at("CLtot", zero),
        conversion="cz = -CL; the alpha = 0 run-case row IS the intercept, so nothing is "
                   "extrapolated",
        role="intercept",
    )
    collector.put(
        "cm0", at("Cmtot", zero), tornado_key="AVL .st Cmtot at alpha = 0 deg",
        solver_value=at("Cmtot", zero),
        conversion="the alpha = 0 run-case row; Cm0's sign is tail rigging, not a derivable "
                   "convention, so it is never sign-normalised",
        role="intercept",
    )
    cd_total_zero = at("CDtot", zero)
    cd_induced_zero = at("CDind", zero)
    parasite = cd_total_zero - cd_induced_zero
    collector.put(
        "cd0", -parasite, tornado_key="AVL .st CDtot - CDind at alpha = 0 deg",
        solver_value=parasite,
        conversion="total drag less the induced part at the zero-lift run case, i.e. the "
                   "parasite drag; derived from two reported numbers rather than echoing the "
                   "#CDp card this converter wrote",
        role="intercept",
    )
    for field, key in (
        ("cl_beta", "Clb"), ("cy_beta", "CYb"), ("cn_beta", "Cnb"),
        ("cl_p", "Clp"), ("cl_r", "Clr"), ("cy_p", "CYp"), ("cy_r", "CYr"),
        ("cn_p", "Cnp"), ("cn_r", "Cnr"), ("cm_q", "Cmq"),
    ):
        collector.put(
            field, at(key), tornado_key=f"AVL .st {key} at alpha = {geometry.alpha_deg:g} deg",
            solver_value=at(key),
            conversion="body-axis derivative used as it stands: AVL's axes and moment senses "
                       "are plane/dynamics.py's, and AVL's own non-dimensional rates already "
                       "use its p*b/(2V), q*cbar/(2V), r*b/(2V) scaling",
            role="slope",
        )
    collector.put(
        "cl_q", -at("CLq"), tornado_key=f"AVL .st CLq at alpha = {geometry.alpha_deg:g} deg",
        solver_value=at("CLq"),
        conversion="czq[0] holds the standard-aero CL_q (what both in-tree files do), so it is "
                   "-CLq; the schema then adds it to the body-z Cz, an inconsistency inherited "
                   "from those files",
        role="slope",
    )
    # cxq[0] lives in the geometry-axis (.sb) table, which run_avl_full's stacked .st
    # files do NOT carry -- so it is read out of the .sb text itself.  AVL's .sb is
    # written at the alpha = 0 run case, the same condition as the two intercepts.
    cxq, cxq_line = _sb_value(sb_text, "CXq")
    collector.put(
        "cd_q", cxq, tornado_key=f"AVL .sb CXq (line {cxq_line})", solver_value=cxq,
        conversion="AVL's geometry-axis CXq, used as it stands: it is already the body-x "
                   "coefficient per q-hat, which is what cxq[0] adds to the body-x cx.  Note "
                   "the invariant table wants cxq[0] > 0 while AVL reports a NEGATIVE CXq, "
                   "so normalise_sign flips it and the flip is logged",
        role="slope",
    )

    for field, surface, key, negate_lift in (
        ("cl_da", "aileron", "Cl", False), ("cy_da", "aileron", "CY", False),
        ("cn_da", "aileron", "Cn", False),
        ("cl_de", "elevator", "CL", True), ("cm_de", "elevator", "Cm", False),
        ("cl_dr", "rudder", "Cl", False), ("cy_dr", "rudder", "CY", False),
        ("cn_dr", "rudder", "Cn", False),
    ):
        per_degree = by_surface[surface].get(key)
        if per_degree is None:
            collector.absent(
                field, f"AVL .sb {key}d[{surface}]",
                reason="AVL's .sb carries no such deflection column for this surface at "
                       "alpha = 0; it is declared rather than guessed",
            )
            continue
        magnitude = per_degree_to_per_radian(float(per_degree))
        collector.put(
            field, -magnitude if negate_lift else magnitude,
            tornado_key=f"aid.avl_controls[{surface}].{key}",
            solver_value=float(per_degree),
            conversion=(
                f"per degree x {DEG_TO_RAD}; AVL's deflection columns are per degree"
                + (".  cz = -CL, and cz[5] holds the standard-aero CL_de the schema and "
                   "both in-tree files store there" if negate_lift else
                   ".  AVL's axes and moment senses are plane/dynamics.py's, so no frame "
                   "sign is applied and normalise_sign decides the sign")
            ),
            role="slope",
        )

    derivatives = collector.derivative_set()
    coefficients, missing_slots = to_morelli(derivatives)
    if missing_slots:
        raise AssertionError(f"to_morelli zeroed {missing_slots}; the run is incomplete")
    aircraft, initial, controls, trim_block = _trim(coefficients, geometry, "avl")
    margin = coefficients["cm"][1] / coefficients["cz"][1]
    if not 0.05 <= margin <= 0.45:
        raise ValueError(f"static margin {margin:.4f} cbar is outside 0.05...0.45")
    provenance = _cross_check_provenance(
        "avl",
        solver_version="AVL via aid.avl_io.run_avl_full and aid.avl_controls.avl_controls",
        geometry=geometry,
        collector=collector,
        source=source,
        shared=shared,
        moment_reference=moment_reference,
        flight_condition=_generic_flight_condition(geometry, trim_block, source_speed_mps=geometry.as_mps),
        trim_block=trim_block,
        extra={
            "axes": "AVL: x forward, y right, z DOWN (its own .sb header prints \"Standard "
                    "axis orientation, X fwd, Z down\"), positive Cl roll right, positive Cm "
                    "nose up, positive Cn yaw right -- identical to plane/dynamics.py, so this "
                    "adapter applies no axis mapping at all",
            "numpy_bridge": numpy_note,
            "control_rows": {surface: dict(by_surface[surface]) for surface in by_surface},
            "control_run": {
                "why_two_runs": "run_avl_full's .sb lists no deflected surface, so parse_sb "
                                "returns no control columns; avl_controls writes a CONTROL card "
                                "for every legal surface and deflects all of them",
                "delta_deg": 0.0,
                "surfaces_available": sorted(by_surface),
                "per_degree": raw["control_rows_per_degree"],
            },
            "cd0": {
                "value": parasite,
                "CDtot_at_alpha0": cd_total_zero,
                "CDind_at_alpha0": cd_induced_zero,
                "CDvis_reported": at("CDvis", zero),
                "cdp_fed_to_avl": float(ac.WG["CD0"]),
                "note": "AVL's #CDp card is aid's own stored WG.CD0; CDtot - CDind is derived "
                        "from the solver's own two numbers and agrees with it to the four "
                        "decimal places both are printed at",
            },
            "reference_case": {
                "note": "run_avl_full also solves an unconstrained case (`x` before the first "
                        "`a a`), which AVL trims to zero lift; it is recorded but NOT used, "
                        "because every slot comes from the alpha = 4 deg row of the schedule "
                        "so all four models describe the same condition",
                "alpha_deg": float(swept["ref"]["alpha"]),
                "CLtot": float(swept["ref"]["CLtot"]),
            },
        },
    )
    return CrossCheckRun(
        model="avl", derivatives=derivatives, coefficients=coefficients,
        raw=raw, geometry=geometry,
        provenance=provenance, aircraft=aircraft, initial=initial,
        controls=controls, trim=trim_block,
    )


def _sb_value(sb_text: str, name: str) -> tuple[float, int]:
    """One named derivative out of an AVL ``.sb`` body, with the 1-based line number."""
    from aid.avl_parse import find_value

    value, line = find_value(sb_text.splitlines(), f"{name} =")
    return float(value), line + 1


def _avl_xref(avl_path: Path) -> float:
    """Read ``#Xref Yref Zref`` back out of the AVL geometry this converter wrote.

    Read back rather than assumed: the plan requires the moment reference to be
    verified, and the geometry file is what the solver was actually handed.
    """
    lines = avl_path.read_text(encoding="ascii").splitlines()
    for index, line in enumerate(lines):
        if line.strip().lower() == "#xref yref zref" and index + 1 < len(lines):
            return float(lines[index + 1].split()[0])
    raise ValueError(f"{avl_path} has no '#Xref Yref Zref' card; the moment reference is unknown")


def avl_derivatives(source: Path | None = None) -> DerivativeSet:
    """The normalised derivative set AVL produces for the source geometry."""
    return avl_run(source).derivatives


def flow5_run(source: Path | None = None) -> CrossCheckRun:
    """flow5 over the source: a longitudinal static cross-check and nothing more.

    Measured, from ``FLOW5/run/flow5_run.cpp``: the runner reads
    ``polar.alpha_deg`` and nothing else about the condition, calls
    ``PlaneTask::setComputeDerivatives(false)``, and serialises exactly
    ``alpha``, ``CL``, ``CD``, ``Cm``, ``CLa`` and ``Cma``.  There is no sideslip
    sweep, no rate sweep, and no roll, yaw or side-force column anywhere in the
    output, so flow5 can produce seven of the twenty-five ``DerivativeSet``
    fields and no more:

    ``cl0``, ``cl_alpha``, ``cm_alpha``, ``cm0``, ``cl_de``, ``cm_de`` and
    ``cd0``.  Everything else is ``None``, zeroed and declared in
    ``provenance.missing`` with that reason.  ``aid.flow5_controls.flow5_controls``
    is still run because it is the only way to get the elevator, and it confirms
    the absence by returning every aileron and rudder coefficient as ``None``:
    its ``_KEEP`` table asks for ``Cl``/``Cn`` and ``CY``, and flow5's output has
    none of them.

    flow5's deck is entirely SI -- ``sref_m2``, ``cref_m``, ``bref_m``,
    ``cog_m`` and ``qinf_mps`` -- so its static derivatives are per radian about
    the CG at a true 10.2073 m/s, with no unit slip at all.  The flight-condition
    note still applies: that is 2.26x slower than the trimmed cruise speed, so
    its ``CLa`` and ``Cma`` belong to a slower, thinner wing than the one the
    model flies.
    """
    ac, shared, _aid, numpy_note = _cross_check_inputs(source)
    from aid.flow5_controls import flow5_controls
    from aid.flow5_io import run_flow5_native, write_flow5_deck
    from aid.paths import flow5_bin

    with _scratch():
        if not flow5_bin().is_file():
            raise FileNotFoundError(f"the flow5 binary is not at {flow5_bin()}")
        deck = write_flow5_deck(ac, FLOW5_MESH)
        polar = run_flow5_native(deck)
        control_rows = flow5_controls(ac, deltas_deg=(0.0,), mesh=FLOW5_MESH)
    by_surface = {row["surface"]: row for row in control_rows}
    geometry = _reference_geometry(ac, mesh=FLOW5_MESH)
    cog = [float(v) for v in deck["polar"]["cog_m"]]
    moment_reference = _verify_cg_reference(geometry, cog[0] / FT_TO_M, solver="flow5")

    schedule = [float(v) for v in np.asarray(polar["alpha"]).reshape(-1)]
    index = schedule.index(geometry.alpha_deg) if geometry.alpha_deg in schedule else 0
    zero = schedule.index(0.0)
    collector = _Collector()
    raw: dict[str, Any] = {
        "mesh": list(FLOW5_MESH),
        "deck_polar": deck["polar"],
        "deck_wings": [{"name": wing.get("name"), "role": wing.get("role"),
                        "sections": len(wing.get("sections", []))} for wing in deck["wings"]],
        "output_keys": sorted(polar),
        "alpha_schedule_deg": schedule,
        "alpha_index": index,
        "alpha_deg": geometry.alpha_deg,
        "CL": [float(v) for v in np.asarray(polar["CL"]).reshape(-1)],
        "CD": [float(v) for v in np.asarray(polar["CD"]).reshape(-1)],
        "Cm": [float(v) for v in np.asarray(polar["Cm"]).reshape(-1)],
        "CLa": float(polar["CLa"]),
        "Cma": float(polar["Cma"]),
        "control_rows_per_degree": {
            surface: {key: by_surface[surface].get(key)
                      for key in ("CL", "CD", "Cm", "CY", "Cl", "Cn")}
            for surface in ("aileron", "elevator", "rudder")
        },
        "cog_m": cog,
    }
    collector.put(
        "cl_alpha", -float(polar["CLa"]),
        tornado_key="flow5 polar CLa (OLS over its own alpha rows, per radian)",
        solver_value=float(polar["CLa"]),
        conversion="cz = -CL_alpha; flow5's deck is SI and its OLS slope is per radian, so the "
                   "only conversion is the cz = -CL identity",
        role="slope",
    )
    collector.put(
        "cm_alpha", float(polar["Cma"]),
        tornado_key="flow5 polar Cma (OLS over its own alpha rows, per radian)",
        solver_value=float(polar["Cma"]),
        conversion="nose-up positive already; the deck puts cog_m at the CG so no shift applies",
        role="slope",
    )
    collector.put(
        "cl0", -float(np.asarray(polar["CL"]).reshape(-1)[zero]),
        tornado_key="flow5 polar CL at alpha = 0 deg",
        solver_value=float(np.asarray(polar["CL"]).reshape(-1)[zero]),
        conversion="cz = -CL; the deck's own alpha = 0 row IS the intercept",
        role="intercept",
    )
    collector.put(
        "cm0", float(np.asarray(polar["Cm"]).reshape(-1)[zero]),
        tornado_key="flow5 polar Cm at alpha = 0 deg",
        solver_value=float(np.asarray(polar["Cm"]).reshape(-1)[zero]),
        conversion="the deck's own alpha = 0 row; Cm0's sign is tail rigging, not a derivable "
                   "convention, so it is never sign-normalised",
        role="intercept",
    )
    # flow5 runs with viscous = false and thin = true, so its CD is induced drag
    # only: CD(0) = 0.001243 is almost exactly K*CL(0)^2 = 0.001391, and the
    # difference is negative.  A thin lattice run has no profile drag, exactly as
    # for Tornado, so the parasite drag is taken from the source file's own
    # DATCOM-computed WG.CD0 and both figures are recorded.
    cd_at_zero = float(np.asarray(polar["CD"]).reshape(-1)[zero])
    cl_at_zero = float(np.asarray(polar["CL"]).reshape(-1)[zero])
    induced = float(ac.WG["K"]) * cl_at_zero * cl_at_zero
    parasite = float(ac.WG["CD0"])
    collector.put(
        "cd0", -parasite, tornado_key="cd0 <- source WG.CD0 (NOT a flow5 output)",
        solver_value=None,
        conversion="parasite drag as a positive magnitude, then cx = -(parasite); flow5's own "
                   "CD(0) - K*CL(0)^2 is negative because the deck sets viscous = false and "
                   "thin = true, so a lattice run has no profile drag to take",
        role="intercept",
    )
    for field, surface, key in (
        ("cl_de", "elevator", "CL"), ("cm_de", "elevator", "Cm"),
    ):
        per_degree = by_surface[surface].get(key)
        if per_degree is None:
            collector.absent(field, f"aid.flow5_controls[{surface}].{key}",
                              reason="flow5's runner returned no such coefficient")
            continue
        collector.put(
            field, -per_degree_to_per_radian(float(per_degree))
            if key == "CL" else per_degree_to_per_radian(float(per_degree)),
            tornado_key=f"aid.flow5_controls[{surface}].{key}",
            solver_value=float(per_degree),
            conversion=f"per degree x {DEG_TO_RAD}: aid.flow5_controls central-differences two "
                       f"decks at delta +- aid.control_deriv.H_DEG = +-1 DEG",
            role="slope",
        )
    absent = [
        ("cl_beta", "roll due to sideslip"),
        ("cy_beta", "side force due to sideslip"),
        ("cn_beta", "yaw due to sideslip"),
        ("cl_q", "roll rate"), ("cm_q", "pitch rate"), ("cy_p", "side force, roll rate"),
        ("cy_r", "side force, yaw rate"), ("cl_p", "roll due to roll rate"),
        ("cl_r", "roll due to yaw rate"), ("cn_p", "yaw due to roll rate"),
        ("cn_r", "yaw due to yaw rate"), ("cd_q", "axial force due to pitch rate"),
        ("cl_da", "aileron roll"), ("cy_da", "aileron side force"),
        ("cn_da", "aileron yaw"), ("cl_dr", "rudder roll"),
        ("cy_dr", "rudder side force"), ("cn_dr", "rudder yaw"),
    ]
    for field, quantity in absent:
        collector.absent(
            field, "flow5 polar / aid.flow5_controls",
            reason=f"{quantity}: flow5's runner (FLOW5/run/flow5_run.cpp) reads only "
                   "polar.alpha_deg, calls setComputeDerivatives(false) and serialises only "
                   f"alpha, CL, CD, Cm, CLa and Cma -- there is no {quantity} anywhere in its "
                   "output",
        )

    derivatives = collector.derivative_set()
    coefficients, _missing_slots = to_morelli(derivatives)
    aircraft, initial, controls, trim_block = _trim(coefficients, geometry, "flow5")
    margin = coefficients["cm"][1] / coefficients["cz"][1]
    if not 0.05 <= margin <= 0.45:
        raise ValueError(f"static margin {margin:.4f} cbar is outside 0.05...0.45")
    provenance = _cross_check_provenance(
        "flow5",
        solver_version="flow5 via aid.flow5_io.write_flow5_deck / run_flow5_native and "
                       "aid.flow5_controls.flow5_controls",
        geometry=geometry,
        collector=collector,
        source=source,
        shared=shared,
        moment_reference=moment_reference,
        flight_condition=_generic_flight_condition(geometry, trim_block, source_speed_mps=geometry.v_true_mps),
        trim_block=trim_block,
        extra={
            "axes": "flow5: its deck's cog_m is x forward, y right, z DOWN and its polar is "
                    "up-positive lift with nose-up-positive Cm -- identical to "
                    "plane/dynamics.py, so this adapter applies no axis mapping and no moment "
                    "shift",
            "numpy_bridge": numpy_note,
            "control_rows": {surface: dict(by_surface[surface]) for surface in by_surface},
            "coverage": {
                "produced": sorted(f for f in ALL_DERIVATIVE_FIELDS
                                   if getattr(derivatives, f) is not None),
                "absent": sorted(f for f in ALL_DERIVATIVE_FIELDS
                                 if getattr(derivatives, f) is None),
                "why": "flow5 is a steady thin-lattice polar code whose runner emits six "
                       "numbers; it is a longitudinal cross-check, not a 6-DOF data set",
                "not_a_field": {
                    "cd_de": "flow5 DOES produce an elevator drag derivative -- "
                             "aid.flow5_controls keeps ('CL', 'CD', 'Cm') for the elevator --"
                             " but this schema has no cd_de slot in any of the 19 arrays, so"
                             " it maps to nothing and is recorded in geometry.jsonc as raw"
                             " output instead",
                },
                "invariant_band_note": (
                    f"cm[1] = {coefficients['cm'][1]:.6f}/rad is 4.9 % BELOW the plan's "
                    "pitched-moment slope band of -1.5 ... -0.3 /rad, which that band was "
                    "estimated from (tornado.jsonc ships -1.1927).  The sign and the static "
                    "margin are both correct and the band is the outlier here, not the "
                    "coefficient; it is recorded rather than forced"
                ),
            },
            "cd0": {
                "value": parasite,
                "source": "source WG.CD0 (NOT a flow5 output)",
                "flow5_CD_at_alpha0": cd_at_zero,
                "induced_part": induced,
                "flow5_parasite": cd_at_zero - induced,
                "note": "flow5's own parasite drag is NEGATIVE (-1.5e-04): the deck sets "
                        "viscous = false and thin = true, so the polar carries induced drag "
                        "only.  The source file's own DATCOM-computed WG.CD0 is used instead, "
                        "as it already is for tornado.jsonc, so all four files agree on it",
            },
        },
    )
    return CrossCheckRun(
        model="flow5", derivatives=derivatives, coefficients=coefficients,
        raw=raw, geometry=geometry,
        provenance=provenance, aircraft=aircraft, initial=initial,
        controls=controls, trim=trim_block,
    )


def flow5_derivatives(source: Path | None = None) -> DerivativeSet:
    """The normalised derivative set flow5 produces for the source geometry."""
    return flow5_run(source).derivatives


def _provisional_aircraft(geometry: Geometry, v_ref_mps: float) -> dict[str, float]:
    """Mass, inertia and a provisional thrust, before the trim solve runs.

    The inertia rules are the plan's: a radius of gyration per axis on THIS mass
    and THIS span, ``I = m k^2``, with the rule named per axis.  Published
    full-scale inertia was rejected because inertia scales as ``m l^2`` and the
    mass and length scales here are already mutually inconsistent.
    """
    mass_kg = geometry.mass_kg
    k_xx, k_yy, k_zz = _radii(geometry)
    return {
        "mass_kg": mass_kg,
        "s_m2": geometry.s_ref_m2,
        "b_m": geometry.b_ref_m,
        "cbar_m": geometry.c_ref_m,
        # The model's own xcg/xcg_ref term (f16/aero_morelli.py, `Cm + Cz*(xcgref -
        # xcg)`) is identically zero on every reachable path, so the two are set
        # equal rather than pretending to place a reference the solver never used.
        "xcg": geometry.dx_m,
        "xcg_ref": geometry.dx_m,
        "ixx": mass_kg * k_xx * k_xx,
        "iyy": mass_kg * k_yy * k_yy,
        "izz": mass_kg * k_zz * k_zz,
        "ixz": 0.0,
        "he": 0.0,
        "t_max_n": mass_kg * SEA_LEVEL_G_MPS2,
        "tau_s": TAU_S,
        "v_ref_mps": v_ref_mps,
    }


def _seed(geometry: Geometry, coefficients: dict[str, list[float]]) -> tuple[list[float], dict[str, float], dict[str, float]]:
    """A first guess for the trim: a target CL fixes alpha, level flight fixes V."""
    mass_kg = geometry.mass_kg
    weight_n = mass_kg * SEA_LEVEL_G_MPS2
    cl0 = -float(coefficients["cz"][0])
    cl_alpha = -float(coefficients["cz"][1])
    target_cl = TARGET_CRUISE_CL
    alpha = (target_cl - cl0) / cl_alpha
    qbar = weight_n / (geometry.s_ref_m2 * target_cl)
    cruise = math.sqrt(2.0 * qbar / SEA_LEVEL_RHO)
    initial = {
        "vt_mps": cruise, "alpha_rad": alpha, "beta_rad": 0.0, "phi_rad": 0.0,
        "theta_rad": alpha, "psi_rad": 0.0, "p_rad_s": 0.0, "q_rad_s": 0.0, "r_rad_s": 0.0,
        "pn_m": 0.0, "pe_m": 0.0, "alt_m": INITIAL_ALT_M, "power": CRUISE_POWER,
    }
    controls = {
        "throttle": CRUISE_POWER, "elevator_rad": 0.0, "aileron_rad": 0.0, "rudder_rad": 0.0,
    }
    guess = [cruise, alpha, alpha, 0.0]
    return guess, initial, controls


def _trim_residual(guess: list[float], aircraft: Any) -> list[float]:
    """``plane/dynamics.py``'s own equations of motion, asked to stand still.

    Five residuals, five unknowns ``(vt, alpha, theta, elevator, power)``: speed
    rate, alpha rate, altitude rate, pitch rate, and how far the lift
    coefficient is from the cruise target.  The pitch-rate residual is the
    pitching-moment balance, because with ``p = q = r = 0`` the only thing left in
    ``xd[7]`` is ``qbar * S * cbar * c7 * cm``.  The lift residual is what makes
    the trim happen at the target CL rather than at whichever equilibrium the
    solver happens to fall into.  Nothing here re-derives the flight model's
    algebra -- the model is called, not copied.
    """
    vt, alpha, theta, elevator, power = (float(value) for value in guess)
    state = np.array(
        [vt, alpha, 0.0, 0.0, theta, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, INITIAL_ALT_M, power]
    )
    control = np.array([power, elevator, 0.0, 0.0])
    derivative = plane_derivative(state, control, aircraft)
    _, _, cz, _, _, _ = morelli_coefficients(
        alpha, 0.0, elevator, 0.0, 0.0, 0.0, 0.0, 0.0,
        aircraft.cbar_m, aircraft.b_m, vt, aircraft.xcg, aircraft.xcg_ref,
        root=aircraft.coeff_dir, model=aircraft.model,
    )
    return [
        float(derivative[0]),
        float(derivative[1]),
        float(derivative[11]),
        float(derivative[7]),
        float(-cz - TARGET_CRUISE_CL),
    ]


def _trim(
    coefficients: dict[str, list[float]], geometry: Geometry, model: str = "tornado"
) -> tuple[dict[str, float], dict[str, float], dict[str, float], dict[str, float]]:
    """Solve mass, inertia, thrust and the initial state against the model.

    The coefficients are already final, so the payload is written once and the
    trim is solved against the very file that will be committed, loaded through
    ``plane.aircraft.load_aircraft`` (which also re-checks ``_POSITIVE_KEYS``).
    ``t_max_n`` is then rescaled so that the thrust at the trimmed condition is
    exactly the thrust that condition needs -- the trim point does not move,
    because ``thrust_n(power, v) = power * t_max_n * (1 - v/v_ref)`` is linear in
    ``t_max_n`` at fixed ``v``.  Nothing here is a published engine number.
    """
    from scipy.optimize import fsolve

    from plane.aircraft import load_aircraft, thrust_n

    aircraft_block = _provisional_aircraft(geometry, 1.5 * 30.0)
    guess, initial, controls = _seed(geometry, coefficients)
    guess = [*guess, CRUISE_POWER]
    fallback = False
    trim_block: dict[str, float] = {}
    with TemporaryDirectory(prefix="aero_convert_trim_") as tmp:
        plane_dir = Path(tmp) / "cessna172"
        plane_dir.mkdir(parents=True)
        provisional = {
            "model": model,
            "aircraft": aircraft_block,
            "initial": initial,
            "controls": controls,
            "coefficients": coefficients,
        }
        (plane_dir / f"{model}.jsonc").write_text(json.dumps(provisional))
        probe = load_aircraft("cessna172", model, root=Path(tmp))
        solution, _info, flag, message = fsolve(
            _trim_residual, guess, args=(probe,), full_output=True, xtol=1e-12
        )
        vt, alpha, theta, elevator, power = (float(value) for value in solution)
        residual = _trim_residual(list(solution), probe)
        if flag != 1 or max(abs(value) for value in residual) > 1e-6:
            raise ValueError(
                f"the trim solve did not converge (flag {flag}, {message!r}); "
                f"residuals {residual}"
            )
        # Rescale the available thrust so that the thrust AT the trimmed
        # condition is the thrust that condition needs.  The trim point does not
        # move: thrust_n is linear in t_max_n at a fixed speed.
        thrust_needed = thrust_n(power, vt, probe)
        v_ref = V_REF_FACTOR * vt
        scale = max(0.0, 1.0 - vt / v_ref)
        if scale <= 0.0 or thrust_needed <= 0.0 or not (0.0 < power <= 1.0):
            t_max_n = probe.mass_kg * SEA_LEVEL_G_MPS2
            v_ref = 1.5 * vt
            power = CRUISE_POWER
            fallback = True
        else:
            t_max_n = thrust_needed / (power * scale)
        if t_max_n <= 0.0:
            t_max_n = probe.mass_kg * SEA_LEVEL_G_MPS2
            fallback = True

    aircraft_block["t_max_n"] = t_max_n
    aircraft_block["v_ref_mps"] = v_ref
    initial["vt_mps"] = vt
    initial["alpha_rad"] = alpha
    initial["theta_rad"] = theta
    initial["power"] = power
    controls["throttle"] = power
    controls["elevator_rad"] = elevator
    _, qbar, rho = air(vt, INITIAL_ALT_M)
    trim_block = {
        "target_cruise_cl": TARGET_CRUISE_CL,
        "cl0": -float(coefficients["cz"][0]),
        "cl_alpha": -float(coefficients["cz"][1]),
        "alpha_trim_rad": alpha,
        "theta_trim_rad": theta,
        "elevator_rad": elevator,
        "cruise_mps": vt,
        "qbar_pa": qbar,
        "rho_kg_m3": rho,
        "alt_m": INITIAL_ALT_M,
        "weight_n": geometry.mass_kg * SEA_LEVEL_G_MPS2,
        "cd0": float(coefficients["cx"][0]),
        "cm_at_trim": float(coefficients["cm"][0])
        + float(coefficients["cm"][1]) * alpha
        + float(coefficients["cm"][2]) * elevator,
        "power": power,
        "v_ref_mps": v_ref,
        "thrust_scale": max(0.0, 1.0 - vt / v_ref),
        "t_max_n": t_max_n,
        "thrust_at_trim_n": power * t_max_n * max(0.0, 1.0 - vt / v_ref),
        "cl_at_trim": -(
            float(coefficients["cz"][0])
            + float(coefficients["cz"][1]) * alpha
            + float(coefficients["cz"][5]) * elevator
        ),
        "residual_max_abs": max(abs(value) for value in residual),
        "fallback": fallback,
    }
    return aircraft_block, initial, controls, trim_block


def geometry_block(run: TornadoRun) -> dict[str, Any]:
    """The ``geometry.jsonc`` payload: SI geometry plus every raw solver number."""
    geometry = run.geometry
    return {
        "model": "geometry",
        "source": run.provenance["source"],
        "units": {
            "length": "m", "mass": "kg", "inertia": "kg*m^2", "speed": "m/s",
            "angle": "rad", "moment_coefficient": "per radian",
            "per_degree_to_per_radian": DEG_TO_RAD,
            "feet_to_metres": FT_TO_M,
        },
        "reference_geometry": {
            "s_ref_m2": geometry.s_ref_m2,
            "b_ref_m": geometry.b_ref_m,
            "c_ref_m": geometry.c_ref_m,
            "x_cg_m": geometry.dx_m,
            "x_cg_ft": geometry.x_cg_ft,
            "x_ref_ft": geometry.x_ref_ft,
            "x_wing_ft": geometry.x_wing_ft,
            "x_tail_ft": geometry.x_tail_ft,
            "cg_offset_for_shift_m": [geometry.dx_m, geometry.dy_m, geometry.dz_m],
        },
        "planform": run.raw["planform"],
        "flight_condition": run.provenance["flight_condition"],
        "mesh": list(geometry.mesh),
        "mass_and_inertia": {
            "mass": run.provenance["mass"],
            "radii": run.provenance["inertia"],
            "ixz": 0.0,
            "he": 0.0,
        },
        "trim": run.trim,
        "raw_tornado": run.raw,
        "control_rows_per_degree": {
            row["surface"]: {key: row[key] for key in ("delta_deg", "CL", "CD", "Cm", "CY", "Cl", "Cn")}
            for row in run.control_rows
        },
        "provenance": {
            key: run.provenance[key]
            for key in ("solver", "mesh", "source", "moment_reference", "sign_convention",
                        "cl_alpha_cross_check", "cd0", "flipped", "not_normalised", "missing")
        },
    }


def model_payload(run: TornadoRun) -> dict[str, Any]:
    """The ``tornado.jsonc`` payload: schema-identical to the in-tree Morelli file."""
    coefficients, missing = to_morelli(run.derivatives)
    if missing:
        raise AssertionError(f"to_morelli zeroed {missing}; the run is incomplete")
    return {
        "model": "tornado",
        "aircraft": run.aircraft,
        "initial": run.initial,
        "controls": run.controls,
        "coefficients": coefficients,
        "provenance": run.provenance,
    }


def _flipped_rate_slots(flipped: list[str]) -> list[str]:
    """The flipped slots that are rate/sideslip columns, i.e. the P and R defect.

    Derived from the run's own ``provenance.flipped`` rather than written out, so
    the header cannot name a slot the table did not flip or omit one it did.  The
    control-derivative fields are excluded by their ``_da`` / ``_dr`` / ``_de``
    suffixes: they are per radian of control deflection and are unaffected by the
    P and R component-sense defect.
    """
    return [name for name in flipped if not name.endswith(("_da", "_dr", "_de"))]


def _slot_lines(run: TornadoRun | CrossCheckRun) -> list[str]:
    """One line per coefficient slot: what the solver said, and what was done to it.

    Three numbers per slot, never two: the solver's own value, the value after
    the frame mapping / moment shift / per-degree conversion, and the value that
    ships after ``normalise_sign``.  A slot whose number did not come from the
    solver says so instead of borrowing the solver's name.
    """
    lines = []
    for entry in run.provenance["mapping"]:
        state = entry["state"]
        if entry["solver_input"] == "solver":
            origin = f"solver {entry['tornado_key']} = {entry['solver_value']:.6g}"
        elif entry["solver_input"] == "none":
            origin = f"ABSENT -- {entry['tornado_key']} (see provenance.missing)"
        else:
            origin = f"NOT solver output -- {entry['tornado_key']}"
        mapped = "n/a" if entry["mapped_value"] is None else f"{entry['mapped_value']:.6g}"
        value = "0.0 (missing)" if entry["value"] is None else f"{entry['value']:.6g}"
        lines.append(
            f"//   {entry['slot']:<9s} {entry['field']:<10s} {state:<14s} "
            f"{origin} -> mapped {mapped} -> written {value}"
        )
    lines.append(
        "//   every slot in plane.groups.nonlinear_index is exactly 0.0; CD_alpha is not"
    )
    lines.append(
        "//   representable in this schema (cx has no CD*alpha term) and is zeroed and declared"
    )
    return lines


def header_comment(run: TornadoRun, kind: str) -> str:
    """The eight-item ``//`` block every generated file opens with.

    This is not decoration: it is the only place a future agent learns that the
    planform is a Cessna 172 at roughly 1/3 linear scale, that AID's per-degree
    helpers were not trusted, that Tornado's x axis is aft-positive, and that
    ``cz = -CL``.
    """
    geometry = run.geometry
    common = [
        "// " + "=" * 74,
        f"// {kind} for the Cessna 172 source analysis, generated -- do not hand-edit.",
        "//",
        "// 1. SOLVER, MESH, FLIGHT CONDITION",
        f"//    Solver: Tornado, via the sibling package aid.tornado (imported from {run.provenance['aid_src']}).",
        f"//    Mesh: {TORNADO_MESH[0]} chordwise x {TORNADO_MESH[1]} spanwise (the AID batch"
        " default).  NP[0],",
        "//    the AR 33.3, S 3 ft^2 panel, is KEPT in the lattice and carries",
        f"//    {run.provenance['extra_panel_cl_share']:.1%} of the aircraft's CL at the trim alpha.",
        f"//    Flight condition from the source AERO: Mach {geometry.mach}, altitude {geometry.alt_ft} ft,"
        f" betha {geometry.betha_rad} rad,",
        f"//    evaluated at alpha = {geometry.alpha_deg:.4g} deg (the middle of AERO.ALSCHD),"
        f" V = {geometry.as_mps:.4f} m/s,",
        f"//    rho = {geometry.rho_kg_m3:.6f} kg/m^3.  Tornado's state[\"AS\"] is in FT/S --"
        f" it is {geometry.as_ftps:.4f} ft/s, and AID works in feet",
        "//    throughout (aid.atmosphere.atmosphere(0)['a'] = 1116.288876590643 ft/s) --"
        " so the m/s figure above",
        f"//    is that value x {FT_TO_M}, and both are recorded in"
        " provenance.flight_condition.  This is the SOURCE Mach-0.03 speed;",
        "//    the trimmed cruise speed the model actually flies at is in"
        " provenance.trim.cruise_mps.",
        "//",
        "// 2. THE SOURCE PLANFORM IS A CESSNA 172 AT ROUGHLY 1/3 LINEAR SCALE",
        "//    span 12 ft against a real 36.1 ft, chord 2 ft against 4.89 ft, area 24 ft^2",
        "//    against 174 ft^2.  Wing loading is therefore 6.70 lbf/ft^2 where a real C172 is",
        "//    13.2.  Nothing in this file may be compared with a full-scale figure.",
        "//",
        "// 3. MASS",
        f"//    mass_kg = {geometry.mass_kg} is derived from the source's own"
        f" AERO.WT = {geometry.weight_slug:g} slugs:",
        f"//    floor(AERO.WT * {PLAN_STANDARD_GRAVITY_FT_S2} ft/s^2 / {LBF_PER_KG}"
        f" lbf/kg * 1000)/1000 = floor({geometry.weight_slug:g} *"
        f" {PLAN_STANDARD_GRAVITY_FT_S2} / {LBF_PER_KG} * 1000)/1000 ="
        f" {geometry.mass_kg} kg, truncated from"
        f" {geometry.weight_slug * PLAN_STANDARD_GRAVITY_FT_S2 / LBF_PER_KG:.7f} kg,",
        f"//    i.e. {geometry.weight_slug * PLAN_STANDARD_GRAVITY_FT_S2:g} lbf."
        f"  A slug is a mass unit, so the source's WT converts instead as",
        f"//    AERO.WT * SLUG_TO_KG = {geometry.weight_slug * SLUG_TO_KG:.5f} kg;"
        f" the plan's figure assumes g = {PLAN_STANDARD_GRAVITY_FT_S2} ft/s^2, which",
        f"//    appears nowhere in the source (its own G_FT_S2 is {G_FT_S2}), so the plan"
        " is 0.54 % low and everything derived from the weight is too.",
        "//    This is NOT a geometrically similar 1/3 scale C172,",
        "//    which would weigh about 53.5 kg, so no scaling of a published full-scale number"
        " is self-",
        "//    consistent and none was attempted.  ixx, iyy, izz come from radius-of-gyration"
        " rules on THIS",
        "//    mass and THIS span (see below); ixz = 0 and he = 0.",
        "//",
        "// 4. UNIT CONVERSIONS APPLIED, AND THE ONES NOT TRUSTED",
        f"//    length: ft x {FT_TO_M} = 0.3048 m; angle: degrees x 57.29577951308232"
        " (= DEG_TO_RAD) per radian.",
        "//    aid.handbook_pass._per_rad / _per_deg were NOT used: they are heuristic",
        "//    (`if a > 1: convert else pass`), _per_rad is a no-op below 1.0, and on this",
        "//    aircraft that silently makes a DATCOM rate derivative 57x too small.  Every",
        "//    per-degree number here is multiplied by 57.29577951308232 explicitly.",
        "//    Every array in this file is per radian and every length is SI.",
        f"//    cx[0] -- the parasite drag slot, Cd0 in a drag build-up -- ="
        f" {run.provenance['cd0']['value']:.6g}.",
        f"//    It is NOT a Tornado output.  Source: {run.provenance['cd0']['source']}.",
        f"//    CD(0) = {run.provenance['cd0']['CD_at_alpha0']:.6g} and K*CL(0)^2 ="
        f" {run.provenance['cd0']['induced_part']:.6g} with K = {run.provenance['cd0']['K_wing']:.6g},",
        "//    so CD(0) - K*CL(0)^2 is negative: a linear vortex-lattice run has no profile",
        f"//    drag at all.  The source file's own WG.CD0 = {run.provenance['cd0']['source_WG_CD0']:.6g}"
        " is used instead.",
        "//",
        "// 5. THE MOMENT REFERENCE SHIFT",
        f"//    Tornado reports about geo['ref_point'] = [{geometry.x_ref_ft:g}, 0, 0] while"
        f" geo['CG'] = [{geometry.x_cg_ft:g}, 0, 0] ft, so the",
        f"//    reference point is {geometry.x_cg_ft:g} ft AHEAD of the CG.  Tornado's x axis is"
        " AFT-positive, so the offset",
        f"//    handed to shift_moments_to_cg is dx = -ft({geometry.x_cg_ft:g}) = {geometry.dx_m}"
        " m (NOT +), with dy = dz = 0.",
        "//    Tornado's lift is up-positive while plane/dynamics.py's cz is the body +z force"
        " with z DOWN,",
        "//    so every vertical-force argument is negative.  It is specifically the BODY-Z"
        " force",
        "//    slope, coeff_create's CZ: slopes shift with slopes (Cm_a with -CZ_a, Cm_Q"
        " with -CZ_Q,",
        "//    Cm_de with -CZ_de, reconstructed exactly from the control row's CL and CD)"
        " and the",
        "//    Cm0 intercept shifts with the -CL(0) intercept, which is also the body-z"
        " intercept at",
        "//    alpha = 0.  CL_a and CZ_a differ by 0.34 % here because NP[0] is in the"
        " lattice.  The",
        "//    yaw slopes shift with the BODY-Y side force (Cn_beta with CY_b, Cn_p with"
        " CY_P, Cn_r with",
        "//    CY_R).  No drag coefficient enters any shift: cx reaches the pitch line"
        " only through dz",
        "//    and the yaw line only through dy, and both arms are zero for this aircraft.",
        "//",
        "// 6. SIGN CONVENTION",
        "//    cz is the body +z force coefficient with z DOWN, so cz = -CL and cz[1] is"
        " NEGATIVE for a",
        "//    stable aircraft (CL_alpha = -cz[1] > 0), and cm[1] is negative too."
        "  That identity is",
        "//    load-bearing; it is what makes data/planes/linear/morelli.json's cz[1] = -4.5"
        " CORRECT, not a",
        "//    defect of that file.",
        "//    data/planes/linear/morelli.json is the sign-convention authority wherever it"
        " disagrees with",
        "//    data/planes/f16/morelli.json.  It WINS on cm[1] (-0.8 against +0.0466) and on"
        " cnp[0] (-0.03",
        "//    against +0.0268), where a positive value would be statically unstable.  It"
        " LOSES on cnda[0]",
        "//    (+0.01 against -0.0335), where adverse yaw fixes the sign.  A reader must not"
        " assume the two",
        "//    sets share a convention: this file and linear/morelli.json agree on cz[1] and"
        " disagree with",
        "//    the F-16 set on cm[1] and cnp[0].",
        "//",
        "//    Tornado's axes are x aft, y right, z up; plane/dynamics.py's are x forward,"
        " y right, z down.",
        "//    So cz = -CL, cl = -Cl, cm = +Cm, cn = -Cn.  That mapping resolves the alpha,"
        " beta and Q",
        "//    inversions on its own (CL_Q = +9.59 against a required CL_q < 0; Cl_b ="
        " +0.054; Cn_b = -0.247),",
        "//    but NOT the P and R ones.  Those are a separate and narrower defect in the"
        " sibling:",
        "//    rot_rates = np.array([state[\"P\"], state[\"Q\"], state[\"R\"]]) drops the"
        " standard-aero rates into the",
        "//    Tornado (aft, right, up) component slots without converting their sense, so"
        " roll-right is",
        "//    omega_x_aft = -P and nose-right is omega_z_up = -R and the P and R columns come"
        " back",
        "//    inverted whatever the mapping does -- measured CY_P = +0.160 where physics"
        " needs CY_p < 0",
        "//    and CY_R = -0.387 where it needs CY_r > 0, while Cm_Q is right.  The slots"
        " the mapping leaves",
        "//    on the wrong side, computed from this run's provenance.flipped rather than"
        " written out, are:",
        *[f"//      {name}" for name in _flipped_rate_slots(run.provenance["flipped"])],
        "//    Cn_P is NOT among them: -Cn_P = -0.0687 already satisfies cn_p < 0, so"
        " cn_p is absent",
        "//    from provenance.flipped.  No blanket flip is applied on top of the"
        " mapping, and it makes",
        "//    no difference to any written coefficient: recomputed under both readings,"
        " all twenty-five",
        "//    values are identical, because the table is the final authority and a"
        " component-sense",
        "//    flip reverses the moment and its shift's force coefficient together.",
        "//    Signs are normalised mechanically against aero_convert.units.SIGN_INVARIANTS"
        " and every flip is",
        "//    logged in provenance.flipped; cm0 carries no invariant and is recorded as"
        " not-normalised.",
        "//",
        "//    A CAVEAT on the NINE genuinely per-phat / per-qhat / per-rhat slots:",
        "//    cyp[0], cyr[0], czq[0], cmq[0], clp[0], clr[0], cnp[0], cnr[0], cxq[0].",
        "//    cy[1] and cy[2] are NOT in that list: they are cy_da and cy_dr, the",
        "//    aileron and rudder side force per radian of CONTROL DEFLECTION, which no",
        "//    flight-condition speed touches.  Inherited from the flight condition and",
        "//    NOT corrected here.",
        "//    coeff_create quotes them per p-hat = p*b_ref/(2*AS) using the SOURCE speed",
        "//    state[\"AS\"] = 10.2073 ft/s = 3.1112 m/s (Mach 0.03, sea level), while",
        "//    f16/aero_morelli.py forms phat = p*b_m/(2*V) with V the model's own speed,",
        "//    23.1146 m/s at the trim.  Both scalings are internally consistent -- Tornado",
        "//    works in feet and ft/s throughout, and atm[\"a\"] = 1116.29 is ft/s -- so this",
        "//    is NOT a unit error inside Tornado; it is a flight-condition mismatch, and",
        "//    its effect is that dp-hat/dP is 1.70122 in Tornado against 12.63921 in the",
        "//    model, a factor of 7.42948.  The written rate derivatives are therefore",
        "//    7.43x smaller than the derivative with respect to the model's own phat, i.e.",
        "//    the model's rate-damping terms are 7.43x weaker than the solver's numbers",
        "//    imply at the trimmed speed, and CL_Q = 9.591 is not a textbook per-q-hat",
        "//    value.  NOT rescaled here: the plan fixes the flight condition from the",
        "//    source AERO, and rescaling it is a physics decision for a later task.",
        "//",
        "//    Two slots are stored in the STANDARD-AERO convention rather than the body-axis"
        " one: czq[0]",
        "//    holds CL_q (so it is -CL_Q) and cxq[0] holds CD_q (so it is +CX_Q), which is"
        " what both",
        "//    in-tree files do.  The schema then adds them to the body-z Cz and body-x Cx,"
        " which is an",
        "//    inconsistency of the schema itself, inherited rather than introduced here.",
        "//",
        "// 7. PER-SLOT PROVENANCE (all 19 arrays; real solver output vs zeroed)",
    ]
    tail = [
        "//",
        "// 8. HOW TO REGENERATE THIS FILE",
        f"//    cd python && {BUILD_COMMAND}",
        "//    Nothing else writes these files; the sibling analysis and the sibling aid tree",
        "//    are read-only inputs and all solver scratch goes to a temporary directory.",
        "// " + "=" * 74,
    ]
    return "\n".join(common + _slot_lines(run) + tail)


def cross_check_header(run: CrossCheckRun, kind: str) -> str:
    """The same eight-item ``//`` block, written for DATCOM, AVL or flow5.

    Items 2, 3, 6 and 8 are the same facts as ``header_comment`` states for
    Tornado and are worded the same way, because they are properties of the
    source planform and of ``plane/dynamics.py`` rather than of a solver.  Items
    1, 4, 5 and 7 are per solver: what produced the numbers, which unit defects
    on the path were corrected rather than inherited, which station the moments
    are about, and which slots are real output against which are zeroed.
    """
    geometry = run.geometry
    condition = run.provenance["flight_condition"]
    model = run.model
    if model == "datcom":
        item1 = [
            "//    Solver: Digital DATCOM.  TWO solvers, because each answers a different",
            f"//    question: (a) the Fortran binary run by aid.datcom_run.run_datcom on a card",
            f"//    file written into a TemporaryDirectory -- binary at",
            "//    Matlab/fsroot/code/DATCOM/datcom via aid.paths.datcom_wrapper -- supplies",
            "//    the longitudinal static set (CLA, CMA, CL and CM rows) and the two",
            "//    intercepts; (b) aid.handbook_pass.apply_handbook supplies what the Fortran",
            "//    table does not carry: the parasite drag build-up (aid.drag), the vertical-",
            "//    tail geometry DATCOM Section 7 needs, and the control derivatives",
            "//    (aid.handbook_controls).  There is no mesh: DATCOM is the handbook method.",
            f"//    Evaluated at alpha = {geometry.alpha_deg:g} deg, {SOURCE_ALPHA_INDEX_RULE}.",
            f"//    Flight condition from the source AERO: Mach {geometry.mach}, altitude"
            f" {geometry.alt_ft} ft, betha 0; the Fortran's own",
            "//    for006 header echoes `$FLTCON ... ALT=0.00, MACH=0.030`, so the Fortran half of",
            "//    this file runs at the source condition.",
            f"//    AID hands its drivers `mach * atm['a'] / 3.28084` ="
            f" {SOURCE_SPEED_AS_REPORTED:.6f} and labels it ft/s, and the Fortran",
            f"//    works its reference geometry in feet, so the solver speed is"
            f" {SOURCE_SPEED_AS_REPORTED:.4f} ft/s = {geometry.as_mps:.4f} m/s",
            f"//    in this model's frame.  The same expression evaluated in SI is the PHYSICAL"
            f" speed {geometry.v_true_mps:.4f} m/s at a",
            f"//    sound speed of {geometry.a_sound_ftps:.4f} ft/s -- and that is the speed"
            " flow5's SI deck used, which is",
            "//    why avl.jsonc and flow5.jsonc record different source speeds for the same"
            " Mach 0.03.  Both readings are",
            "//    in provenance.flight_condition and neither is reconciled away.",
            f"//    The trimmed cruise speed the model actually flies at is"
            f" {condition['v_trim_mps']:.4f} m/s.",
        ]
        item4 = [
            f"//    length: ft x {FT_TO_M} = 0.3048 m; angle: degrees x 57.29577951308232"
            " (= DEG_TO_RAD) per radian.",
            "//    THE FORTRAN'S OWN STABILITY TABLE IS PER DEGREE.  Measured against its own",
            "//    cl column, `cla` equals the secant dCL/dalpha in per degree to three",
            f"//    decimals ({float(run.raw['datcom_cla_per_degree'][2]):.5f}/deg against a"
            f" centred secant of",
            f"//    {(float(run.raw['datcom_cl'][3]) - float(run.raw['datcom_cl'][1])) / 8.0:.5f}"
            f"/deg over +/-4 deg).  Every derivative read out of it is multiplied by",
            "//    57.29577951308232 by this converter.",
            "//    AID's HELPBOOK SLOPES ARE NOT TRUSTED EITHER, and three separate defects on",
            "//    this path were corrected rather than inherited:",
            f"//      (1) aid.stability._per_deg, aid.handbook_controls._per_deg and"
            f" aid.handbook_pass._per_rad are all",
            "//          `if a > 1: convert else pass`.  This planform's slopes are",
            "//          0.076-0.084 /deg, so every one of them is a NO-OP and the number",
            "//          that needs a per-radian value is handed the per-degree one.",
            f"//          Measured: aid's own longitudinal_dynamic CLq ="
            f" {float(run.provenance['handbook_cross_check']['aid_longitudinal_dynamic_CLq']):.6f}"
            f" where {DEG_TO_RAD:.4f}x it is",
            f"//          {float(run.provenance['handbook_cross_check']['section7_cl_q_used']):.6f},"
            f" and aid's own lateral_dynamic Clp ="
            f" {float(run.provenance['handbook_cross_check']['aid_lateral_dynamic_Clp']):.6f}"
            f" against",
            f"//          {float(run.provenance['handbook_cross_check']['section7_cl_p_used']):.6f}."
            "  The DATCOM Section 7 formulae are therefore",
            "//          recomputed in this converter with the factor applied by hand, and the",
            "//          uncorrected numbers are kept in provenance.handbook_cross_check.",
            "//      (2) aid/lateral.py:571 ends its tail lift-slope formula with"
            " `vt_a * pi/180`.",
            "//          That formula returns the units of the `k = a0/(2*pi)` it was fed and"
            " AID feeds it",
            "//          VT.a0 = 6.8396 DEGREES, so the result is already per degree and the"
            " pi/180 makes it",
            "//          57.3x too small.  Every quantity linear in that slope is corrected:"
            " CYb, Cnb,",
            "//          Clb and the three Section 7 rate terms that use it.  The evidence that",
            f"//          the correction is the right one: AID's Clb is"
            f" {float(run.provenance['handbook_cross_check']['aid_lateral_as_reported']['Clb']):.6f},"
            " a factor 21.7 below",
            "//          the sibling's own AVL gold Clb = -0.065868 for the same planform"
            " (measured), while",
            f"//          the corrected {run.coefficients['cl'][0]:.6f} is within 5 per cent of"
            " that gold and 1.51x AVL's",
            "//          own alpha = 4 deg row, -0.045756, which avl.jsonc ships.",
            "//      (3) aid/lateral.py:616 returns a HARDCODED 0.1 for both Clda and Cnda and",
            "//          aid/handbook_controls.py has no aileron CY or Cn column at all.  Those",
            "//          0.1 values are a placeholder, not a computation, and are NOT used:"
            " cy[1]",
            "//          and cnda[0] are 0.0 and declared in provenance.missing.",
            f"//    Every array in this file is per radian and every length is SI."
            f"  numpy bridge: {run.provenance['numpy_bridge']}",
            f"//    cx[0] -- the parasite drag slot, Cd0 in a drag build-up -- ="
            f" {run.provenance['cd0']['value']:.6g}.",
            f"//    Source: {run.provenance['cd0']['source']}.  The Fortran's own CD at alpha = 0"
            f" is {run.provenance['cd0']['datcom_own_CD_at_alpha0']:.6g} with an induced part of"
            f" {run.provenance['cd0']['induced_part']:.6g},",
            f"//    i.e. a parasite of {run.provenance['cd0']['datcom_own_parasite']:.6g},"
            f" {run.provenance['cd0']['ratio_datcom_to_aid']:.3f}x the handbook build-up;"
            " both are recorded.",
        ]
        item5 = [
            f"//    NO MOMENT SHIFT WAS APPLIED, and that is verified rather than assumed."
            f"  The Fortran's $SYNTHS card carries",
            f"//    XCG = {run.provenance['moment_reference']['aero_xcg_ft']:g} ft, which is the"
            " station this converter read back out of",
            f"//    its own card file and compared against AERO.XCG ="
            f" {run.provenance['moment_reference']['aero_xcg_ft']:g} ft; AID's handbook pass"
            " takes the same station",
            "//    for the VT arms (hp = VT.Z + VT.ymac - AERO.ZCG, lp = VT.X + VT.xmac +",
            "//    VT.cbar/4 - AERO.XCG).  dx = dy = dz = 0, so every moment here is already"
            " about the CG and",
            "//    cm[1], cm[0], cm[2], cmq[0], cn[0], cnp[0], cnr[0] are used as they stand."
            "  Contrast",
            "//    tornado.jsonc, whose ref_point is 2.94 ft AHEAD of the CG and is corrected"
            " by dx = -0.896112 m.",
        ]
        axes = [
            "//    DATCOM, plane/dynamics.py and every in-tree reference file share ONE axis",
            "//    convention: x forward, y right, z DOWN, positive Cm nose up, positive Cl",
            "//    roll right, positive Cn yaw right.  So there is no frame mapping in this",
            "//    file at all: the only negations are the three slots the schema stores in",
            "//    the wind-axis lift convention, cz[1] = -CL_alpha, czq[0] = -CL_q and",
            "//    cz[5] = -CL_de, because cz = -CL and both in-tree files store it that way.",
            "//    The source planform runs x AFT-positive (XW = 2.2 < XCG = 2.94 < XH = 8.75"
            " ft), which is why",
            "//    tornado.jsonc needs a shift and this file does not.",
        ]
        gaps = [
            "//    THREE SLOTS ARE GENUINELY ABSENT and are declared, not guessed:",
            "//      cy[1]   cy_da -- aid/handbook_controls.py has no aileron side-force column"
            " at all and",
            "//          aid/lateral.py:616's Clda is a hardcoded 0.1 placeholder",
            "//      cnda[0] cn_da -- aid/lateral.py:616's Cnda is a hardcoded 0.1 placeholder"
            " and",
            "//          aid/handbook_controls.py omits aileron yaw entirely",
            "//      cxq[0]  cd_q  -- DATCOM Section 7 has NO axial rate derivative at all and",
            "//          aid/longitudinal_dynamic.py hardcodes cxq = 0.0, which records the",
            "//          absence of a formula rather than a computed zero",
            "//    Every other slot is real output.  CD_alpha is not representable in this",
            "//    schema (cx has no CD*alpha term) and is 0.0 and declared.",
        ]
    elif model == "avl":
        item1 = [
            "//    Solver: AVL, via aid.avl_io.run_avl_full for the stability sweep and",
            "//    aid.avl_controls.avl_controls for the control derivatives, both from the",
            f"//    sibling package imported from {run.provenance['aid_src']}.",
            f"//    Mesh: {AVL_MESH[0]} spanwise x {AVL_MESH[1]} chordwise.  aid/avl_controls.py's"
            " own default, so the",
            "//    stability run and the control run agree.  AVL keeps only WG, HT and VT"
            " (nwing <= 3),",
            "//    so NP[0], which tornado.jsonc keeps in its lattice, is NOT in this file.",
            f"//    Two runs, because run_avl_full's .sb lists no deflected surface at all and"
            " therefore carries",
            "//    no control columns (measured); avl_controls writes a CONTROL card for every"
            " legal surface,",
            "//    deflects all of them by the same delta, and reads the deflection columns out"
            " of its own .sb.",
            "//    That run is also the one whose aileron CONTROL card carries all seven"
            " numbers, which is what",
            "//    makes AVL read the trailing -1 as the duplicate sign instead of as a hinge"
            " component.",
            f"//    Evaluated at alpha = {geometry.alpha_deg:g} deg, {SOURCE_ALPHA_INDEX_RULE}."
            " run_avl_full also",
            "//    solves an unconstrained case, which AVL trims to zero lift; it is recorded"
            " in",
            "//    provenance.reference_case and NOT used, so every slot here describes the"
            " alpha = 4 deg",
            "//    condition the other three files use.",
            f"//    Flight condition from the source AERO: Mach {geometry.mach}, altitude"
            f" {geometry.alt_ft} ft, betha 0.",
            f"//    AID hands AVL's run card `mach * atm['a'] / 3.28084` ="
            f" {SOURCE_SPEED_AS_REPORTED:.6f} as the `v` command,",
            f"//    and Sref/Cref/Bref are written in FEET (24.000 / 2.0000 / 12.000), so AVL's"
            f" solver speed is {SOURCE_SPEED_AS_REPORTED:.4f} ft/s",
            f"//    = {geometry.as_mps:.4f} m/s in this model's frame -- the same reading"
            " tornado.jsonc uses, and the",
            f"//    reason both files carry a {condition['speed_ratio_v_trim_over_v_source']:.5f}x"
            " rate-derivative caveat.  The same expression in",
            f"//    SI is the PHYSICAL speed {geometry.v_true_mps:.4f} m/s, which is what"
            " flow5's SI deck used instead.  Both are in",
            f"//    provenance.flight_condition.  The trimmed cruise speed is"
            f" {condition['v_trim_mps']:.4f} m/s.",
        ]
        item4 = [
            f"//    length: ft x {FT_TO_M} = 0.3048 m; angle: degrees x 57.29577951308232"
            " (= DEG_TO_RAD) per radian.",
            "//    AVL'S OWN UNITS ARE ALREADY CORRECT for this file and NOTHING had to be",
            "//    repaired: its stability derivatives are per radian, its .sb deflection",
            "//    columns are PER DEGREE and are multiplied by 57.29577951308232 here, and its",
            "//    p-hat/q-hat/r-hat already use its own V with the reference lengths in the",
            "//    same units.  aid.handbook_pass._per_rad and aid.stability._per_deg were NOT",
            "//    used anywhere on this path: AVL does not go through them.",
            f"//    Every array in this file is per radian and every length is SI."
            f"  numpy bridge: {run.provenance['numpy_bridge']}",
            f"//    cx[0] -- the parasite drag slot, Cd0 in a drag build-up -- ="
            f" {run.provenance['cd0']['value']:.6g},",
            f"//    from AVL's own CDtot - CDind at the alpha = 0 run case"
            f" ({run.provenance['cd0']['CDtot_at_alpha0']:.6g} -"
            f" {run.provenance['cd0']['CDind_at_alpha0']:.6g}).",
            f"//    AVL's reported CDvis is {run.provenance['cd0']['CDvis_reported']:.6g} and the"
            f" #CDp card it was given was",
            f"//    {run.provenance['cd0']['cdp_fed_to_avl']:.6g} (the source planform's own"
            " DATCOM-computed WG.CD0); all three agree",
            "//    to the four decimal places AVL prints them at.",
        ]
        item5 = [
            f"//    NO MOMENT SHIFT WAS APPLIED, and that is verified rather than assumed."
            f"  aid.avl_io writes the",
            "//    geometry's reference station from AERO.XCG into AVL's `#Xref Yref Zref`"
            " card, and this",
            f"//    converter read it back out of the file it wrote and got"
            f" Xref = {run.provenance['moment_reference']['solver_station_ft']:.4f} ft,",
            f"//    against AERO.XCG = {run.provenance['moment_reference']['aero_xcg_ft']:g} ft."
            "  A mismatch raises rather than",
            "//    proceeding, because every moment in this file is used unshifted."
            "  dx = dy = dz = 0, so",
            "//    cm[1], cm[0], cm[2], cmq[0], cn[0], cnp[0], cnr[0] are used as they stand."
            "  Contrast",
            "//    tornado.jsonc, whose ref_point is 2.94 ft AHEAD of the CG and is corrected"
            " by dx = -0.896112 m.",
        ]
        axes = [
            "//    AVL PRINTS ITS OWN CONVENTION, and it is the same one"
            " plane/dynamics.py uses: its .sb",
            "//    header reads \"Standard axis orientation, X fwd, Z down\", with positive"
            " Cl rolling",
            "//    right, positive Cm nose up and positive Cn yaw right.  So NO axis mapping is",
            "//    applied in this file -- the only negations are the three slots the schema"
            " stores in the",
            "//    wind-axis lift convention, cz[1] = -CL_alpha, czq[0] = -CL_q and"
            " cz[5] = -CL_de, because",
            "//    cz = -CL and both in-tree files store it that way.  CLp is NOT negated"
            " (its .st value is",
            f"//    already negative, {float(run.raw['st_alpha_rows']['CLp'][run.raw['alpha_index']]):.6f}),"
            " and Cn_p IS, because AVL reports",
            f"//    {float(run.raw['st_alpha_rows']['Cnp'][run.raw['alpha_index']]):+.6f} against"
            " the invariant cnp[0] < 0.",
            "//    Every sign is decided mechanically by aero_convert.units.normalise_sign"
            " against SIGN_INVARIANTS;",
            "//    the slots it flipped are listed in provenance.flipped, computed from this"
            " run's own mapping.",
        ]
        gaps = [
            "//    NO SLOT IS ABSENT: AVL produces all 25, including cnda[0] and cy[1], which"
            " DATCOM cannot.",
            "//    CD_alpha is not representable in this schema (cx has no CD*alpha term) and"
            " is 0.0 and",
            "//    declared.  cxq[0] is written 0.0: AVL's .sb carries a CXq, but"
            " run_avl_full's stacked .st",
            "//    does not, so no axial rate derivative was taken and the zero is a real"
            " value, not a gap.",
        ]
    else:
        item1 = [
            "//    Solver: flow5, via aid.flow5_io.write_flow5_deck / run_flow5_native and",
            f"//    aid.flow5_controls.flow5_controls, from the sibling package imported from"
            f" {run.provenance['aid_src']}.",
            f"//    Mesh: {FLOW5_MESH[0]} spanwise x {FLOW5_MESH[1]} chordwise, read by"
            " write_flow5_deck as (ny, nx).  The deck",
            "//    carries WG, HT, VT and NP[0] (\"Wing 2\"), so NP[0] IS in this file even"
            " though it is",
            "//    not in avl.jsonc (AVL keeps at most three surfaces).",
            "//    THE RUNNER'S OUTPUT IS THE WHOLE OF WHAT THIS FILE CAN CONTAIN, and it is",
            "//    six numbers.  Measured from FLOW5/run/flow5_run.cpp: the deck's only",
            "//    condition key read is `polar.alpha_deg`, the analysis calls",
            "//    `PlaneTask::setComputeDerivatives(false)`, and `polar_to_json` serialises",
            "//    exactly alpha, CL, CD, Cm, CLa and Cma.  There is NO sideslip sweep, NO",
            "//    rate sweep, and no roll, yaw or side-force column anywhere in the output.",
            f"//    Evaluated at alpha = {geometry.alpha_deg:g} deg, {SOURCE_ALPHA_INDEX_RULE}.",
            f"//    Flight condition: the deck is ENTIRELY SI -- sref_m2, cref_m, bref_m,"
            f" cog_m and qinf_mps -- so",
            f"//    qinf = {geometry.v_true_mps:.4f} m/s is the PHYSICAL speed and there is no"
            " unit slip on this path at",
            f"//    all, unlike the {SOURCE_SPEED_AS_REPORTED:.4f} AID labels ft/s in tornado.jsonc"
            " and avl.jsonc.",
            f"//    Mach {geometry.mach}, altitude {geometry.alt_ft} ft,"
            f" rho = {geometry.rho_kg_m3:.6f} kg/m^3.  The trimmed cruise speed the model",
            f"//    actually flies at is {condition['v_trim_mps']:.4f} m/s, a factor"
            f" {condition['speed_ratio_v_trim_over_v_source']:.3f} higher, so this file's CLa",
            "//    and Cma describe a slower and therefore thinner wing than the one flown.",
            "//    That factor is 2.26 here and 7.43 in tornado.jsonc and avl.jsonc for one",
            "//    reason only: flow5's deck is SI, so its 10.2073 really is 10.2073 m/s, while",
            "//    the other two feed 10.2073 into a FEET frame, where it means 3.1112 m/s.",
            "//    flow5 has no rate slots at all, so the factor bites only its static set.",
        ]
        item4 = [
            f"//    length: ft x {FT_TO_M} = 0.3048 m, applied by aid.flow5_units inside"
            " write_flow5_deck; angle:",
            "//    degrees x 57.29577951308232 (= DEG_TO_RAD) per radian.",
            "//    NOTHING HAD TO BE REPAIRED on this path.  flow5's deck and its polar are",
            "//    SI end to end and its CLa and Cma are already PER RADIAN (flow5_run.cpp's",
            "//    `ols_slope_rad` regresses CL and Cm on alpha in radians).  The only",
            "//    per-degree numbers here are aid.flow5_controls' central differences, which",
            "//    it forms at delta +- aid.control_deriv.H_DEG = +-1 DEGREE, so each control",
            "//    slot is the solver's own number x 57.29577951308232.",
            f"//    Every array in this file is per radian and every length is SI."
            f"  numpy bridge: {run.provenance['numpy_bridge']}",
            f"//    cx[0] -- the parasite drag slot, Cd0 in a drag build-up -- ="
            f" {run.provenance['cd0']['value']:.6g}.",
            f"//    Source: {run.provenance['cd0']['source']}.  flow5's own CD at alpha = 0 is"
            f" {run.provenance['cd0']['flow5_CD_at_alpha0']:.6g} with an",
            f"//    induced part of {run.provenance['cd0']['induced_part']:.6g}, i.e. a parasite"
            f" of {run.provenance['cd0']['flow5_parasite']:.6g} -- NEGATIVE, because the deck",
            "//    sets `viscous: false` and `thin: true`, so a thin lattice run has no profile"
            " drag at all.",
            "//    The source file's own DATCOM-computed WG.CD0 is used instead, exactly as it",
            "//    already is for tornado.jsonc, so all four files agree on it.",
        ]
        item5 = [
            f"//    NO MOMENT SHIFT WAS APPLIED, and that is verified rather than assumed."
            "  write_flow5_deck puts",
            f"//    the source's own AERO.XCG into the deck's `cog_m` as"
            f" [{run.raw['cog_m'][0]:.6f}, {run.raw['cog_m'][1]:g},"
            f" {run.raw['cog_m'][2]:g}] m, which is",
            f"//    ft({run.provenance['moment_reference']['aero_xcg_ft']:g}) ="
            f" {ft(run.provenance['moment_reference']['aero_xcg_ft']):.6f} m, and this converter",
            "//    read it back out of the deck it built and compared it with AERO.XCG.  A"
            " mismatch raises.",
            "//    dx = dy = dz = 0, so cm[1], cm[0] and cm[2] are used as they stand.  Contrast",
            "//    tornado.jsonc, whose ref_point is 2.94 ft AHEAD of the CG and is corrected"
            " by dx = -0.896112 m.",
        ]
        axes = [
            "//    flow5 reports about its `cog_m` in the SAME x-forward, y-right, z-DOWN frame",
            "//    plane/dynamics.py uses: its CL is up-positive (CLa > 0), so the body-z force",
            "//    coefficient is its negative, and its Cm is nose-up positive.  So NO axis"
            " mapping and NO",
            "//    moment shift is applied; the only negation is cz[1] = -CL_alpha and"
            " cz[5] = -CL_de, the",
            "//    two slots the schema stores in the wind-axis lift convention because"
            " cz = -CL and",
            "//    both in-tree files store it that way.  Its Cma ="
            f" {float(run.raw['Cma']):.6f} is already negative, the",
            "//    invariant sign, so it is written through unnegated.",
            "//    Every sign is decided mechanically by aero_convert.units.normalise_sign"
            " against SIGN_INVARIANTS.",
        ]
        gaps = [
            "//    EIGHTEEN OF THE TWENTY-FIVE SLOTS ARE ABSENT, and every one is declared in",
            "//    provenance.missing with its reason rather than guessed:",
            f"//      produced: {', '.join(run.provenance['coverage']['produced'])}",
            f"//      absent:   {', '.join(run.provenance['coverage']['absent'])}",
            "//    This is flow5's real capability, measured, not a shortfall of this adapter:",
            "//    its runner emits six numbers.  flow5 is a LONGITUDINAL cross-check on the",
            "//    lift slope, the pitching slope, the two intercepts, the elevator and the",
            "//    parasite drag -- it is not a 6-DOF data set and must not be flown as one.",
            "//    CD_alpha is not representable in this schema (cx has no CD*alpha term) and"
            " is 0.0.",
        ]

    common = [
        "// " + "=" * 74,
        f"// {kind} for the Cessna 172 source analysis, generated -- do not hand-edit.",
        "//",
        "// 1. SOLVER, MESH, FLIGHT CONDITION",
        *item1,
        "//",
        "// 2. THE SOURCE PLANFORM IS A CESSNA 172 AT ROUGHLY 1/3 LINEAR SCALE",
        "//    span 12 ft against a real 36.1 ft, chord 2 ft against 4.89 ft, area 24 ft^2",
        "//    against 174 ft^2.  Wing loading is therefore 6.70 lbf/ft^2 where a real C172 is",
        "//    13.2.  Nothing in this file may be compared with a full-scale figure.",
        "//",
        "// 3. MASS",
        f"//    mass_kg = {geometry.mass_kg} is derived from the source's own"
        f" AERO.WT = {geometry.weight_slug:g} slugs:",
        f"//    floor(AERO.WT * {PLAN_STANDARD_GRAVITY_FT_S2} ft/s^2 / {LBF_PER_KG} lbf/kg * 1000)"
        f"/1000 = {geometry.mass_kg} kg, truncated from",
        f"//    {geometry.weight_slug * PLAN_STANDARD_GRAVITY_FT_S2 / LBF_PER_KG:.7f} kg, i.e."
        f" {geometry.weight_slug * PLAN_STANDARD_GRAVITY_FT_S2:g} lbf.  A slug is a mass unit,"
        " so the",
        f"//    source's WT converts instead as AERO.WT * SLUG_TO_KG ="
        f" {geometry.weight_slug * SLUG_TO_KG:.5f} kg; the plan's figure assumes"
        f" g = {PLAN_STANDARD_GRAVITY_FT_S2} ft/s^2, which",
        f"//    appears nowhere in the source (its own G_FT_S2 is {G_FT_S2}), so the plan is"
        " 0.54 % low and",
        "//    everything derived from the weight is too.",
        "//    This is NOT a geometrically similar 1/3 scale C172, which would weigh about"
        " 53.5 kg, so no",
        "//    scaling of a published full-scale number is self-consistent and none was"
        " attempted.",
        "//    ixx, iyy, izz come from radius-of-gyration rules on THIS mass and THIS span"
        " (see below);",
        "//    ixz = 0 and he = 0.  All four Cessna files derive mass and inertia identically"
        " from the same",
        "//    geometry, so they are consistent with each other and with tornado.jsonc.",
        "//    xcg and xcg_ref are BOTH 0.0 here, where tornado.jsonc carries"
        " -0.896112 for both, and that",
        "//    difference is deliberate rather than an oversight: these three solvers report"
        " about the CG",
        "//    itself (verified in item 5), so the CG's offset FROM THE MOMENT REFERENCE is"
        " zero, while for",
        "//    Tornado it is the 2.94 ft station shift.  The schema's own"
        " `Cm + Cz*(xcgref - xcg)` term is",
        "//    identically zero either way, because xcg == xcg_ref on every reachable path.",
        "//",
        "// 4. UNIT CONVERSIONS APPLIED, AND THE ONES NOT TRUSTED",
        *item4,
        "//",
        "// 5. THE MOMENT REFERENCE SHIFT",
        *item5,
        "//",
        "// 6. SIGN CONVENTION",
        "//    cz is the body +z force coefficient with z DOWN, so cz = -CL and cz[1] is"
        " NEGATIVE for a",
        "//    stable aircraft (CL_alpha = -cz[1] > 0), and cm[1] is negative too.  That"
        " identity is",
        "//    load-bearing; it is what makes data/planes/linear/morelli.json's cz[1] = -4.5"
        " CORRECT, not a",
        "//    defect of that file.",
        "//    data/planes/linear/morelli.json is the sign-convention authority wherever it"
        " disagrees with",
        "//    data/planes/f16/morelli.json.  It WINS on cm[1] (-0.8 against +0.0466) and on"
        " cnp[0] (-0.03",
        "//    against +0.0268), where a positive value would be statically unstable.  It"
        " LOSES on cnda[0]",
        "//    (+0.01 against -0.0335), where adverse yaw fixes the sign.  A reader must not"
        " assume the two",
        "//    sets share a convention: this file and linear/morelli.json agree on cz[1] and"
        " disagree with",
        "//    the F-16 set on cm[1] and cnp[0].",
        *axes,
        "//    cm0 (Cm0) carries NO invariant -- the two in-tree reference files disagree on"
        " it and its",
        "//    sign is tail rigging, not a derivable convention -- so it is written through"
        " untouched and is",
        "//    recorded as `not-normalised` in provenance.not_normalised.  Three provenance"
        " states exist and",
        "//    all three are recorded: `missing` (field None, slot 0.0), `flipped` (sign"
        " normalised against",
        "//    its invariant row) and `not-normalised` (cm0).",
        "//",
        "//    A CAVEAT on the NINE genuinely per-phat / per-qhat / per-rhat slots:",
        "//    cyp[0], cyr[0], czq[0], cmq[0], clp[0], clr[0], cnp[0], cnr[0], cxq[0].",
        *_wrap_note(condition["rate_derivative_note"]),
        "//    cy[1] and cy[2] are NOT in that list: they are cy_da and cy_dr, the aileron",        "//    and rudder side force per radian of CONTROL DEFLECTION, which no",
        "//    flight-condition speed touches.",
        "//",
        *gaps,
        "//",
        "// 7. PER-SLOT PROVENANCE (all 19 arrays; real solver output vs zeroed)",
    ]
    tail = [
        "//",
        "// 8. HOW TO REGENERATE THIS FILE",
        f"//    cd python && {BUILD_COMMAND}",
        "//    Nothing else writes these files; the sibling analysis and the sibling aid tree",
        "//    are read-only inputs and all solver scratch goes to a temporary directory.",
        "// " + "=" * 74,
    ]
    return "\n".join(common + _slot_lines(run) + tail)


def _wrap_note(note: str, width: int = 70) -> list[str]:
    """One paragraph of prose -> a list of already-prefixed ``//`` comment lines."""
    lines, current = [], ""
    for word in note.split():
        candidate = f"{current} {word}".strip()
        if len(candidate) > width and current:
            lines.append(current)
            current = word
        else:
            current = candidate
    if current:
        lines.append(current)
    return [f"//      {line}" for line in lines]


def cross_check_payload(run: CrossCheckRun) -> dict[str, Any]:
    """The ``<model>.jsonc`` payload: schema-identical to the in-tree Morelli file."""
    coefficients, _missing = to_morelli(run.derivatives)
    return {
        "model": run.model,
        "aircraft": run.aircraft,
        "initial": run.initial,
        "controls": run.controls,
        "coefficients": coefficients,
        "provenance": run.provenance,
    }


def write_jsonc(path: Path, payload: dict[str, Any], comment: str) -> Path:
    """Write a JSONC file whose first lines are real ``//`` comments."""
    path.parent.mkdir(parents=True, exist_ok=True)
    body = json.dumps(payload, indent=2, sort_keys=False)
    path.write_text(f"{comment}\n{body}\n")
    return path


def write_model(run: CrossCheckRun, out_dir: Path | None = None) -> Path:
    """Write one cross-check model file and return its path."""
    directory = Path(out_dir) if out_dir is not None else DEFAULT_OUT_DIR
    return write_jsonc(
        directory / f"{run.model}.jsonc",
        cross_check_payload(run),
        cross_check_header(run, f"data/planes/cessna172/{run.model}.jsonc"),
    )


def write_all(run: TornadoRun, out_dir: Path | None = None) -> dict[str, str]:
    """Write both files for an existing run and return their paths."""
    directory = Path(out_dir) if out_dir is not None else DEFAULT_OUT_DIR
    written = {
        "tornado.jsonc": write_jsonc(
            directory / "tornado.jsonc",
            model_payload(run),
            header_comment(run, "data/planes/cessna172/tornado.jsonc"),
        ),
        "geometry.jsonc": write_jsonc(
            directory / "geometry.jsonc",
            geometry_block(run),
            header_comment(run, "data/planes/cessna172/geometry.jsonc"),
        ),
    }
    return {name: str(path) for name, path in written.items()}


def build(out_dir: Path | None = None, source: Path | None = None) -> dict[str, str]:
    """Run Tornado and write ``tornado.jsonc`` and ``geometry.jsonc``.

    Deliberately Tornado-only: geometry.jsonc collects Tornado's raw output, and
    the cross-check models would each add their own run to every caller.
    ``build_all`` is the entry point that writes all five files.
    """
    return write_all(tornado_run(source), out_dir)


def build_all(out_dir: Path | None = None, source: Path | None = None) -> dict[str, str]:
    """Run all four solvers and write all five files.

    The same mass, inertia, trim and initial state are used for every model: each
    file carries its own trim solve against its own coefficients, and the four
    agree on mass_kg, ixx, iyy, izz, ixz and he because those come from the same
    geometry and the same radius-of-gyration rules.
    """
    written = build(out_dir, source)
    for model in CROSS_CHECK_MODELS:
        run = {"datcom": datcom_run, "avl": avl_run, "flow5": flow5_run}[model](source)
        payload = cross_check_payload(run)
        _verify_lengths(payload)
        written[f"{model}.jsonc"] = str(write_model(run, out_dir))
    return written


def _verify_lengths(payload: dict[str, Any]) -> None:
    coefficients = payload["coefficients"]
    if sorted(coefficients) != sorted(MORELLI_LENGTHS):
        raise AssertionError("coefficient arrays do not match plane/groups.MORELLI_LENGTHS")
    for name, length in MORELLI_LENGTHS.items():
        if len(coefficients[name]) != length:
            raise AssertionError(f"{name} has {len(coefficients[name])} entries, expected {length}")


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out-dir", type=Path, default=None,
                        help="where to write the files (default data/planes/cessna172)")
    parser.add_argument("--source", type=Path, default=None,
                        help="the source analysis file (default the sibling Cessna172.jsonc)")
    args = parser.parse_args(argv)
    run = tornado_run(args.source)
    payload = model_payload(run)
    _verify_lengths(payload)
    written = write_all(run, args.out_dir)
    for model in CROSS_CHECK_MODELS:
        cross = {"datcom": datcom_run, "avl": avl_run, "flow5": flow5_run}[model](args.source)
        cross_payload = cross_check_payload(cross)
        _verify_lengths(cross_payload)
        written[f"{model}.jsonc"] = str(write_model(cross, args.out_dir))
    for path in written.values():
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())