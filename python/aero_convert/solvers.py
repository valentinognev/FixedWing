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
every solve.  The location is **recorded** in every generated file relative to the
repository root (``../USAF_DATCOM/AircraftIntuitiveDesign/Python/src``), never as an
absolute path: this repo is reachable through two mount points on the machine that
generated them and ``Path.resolve()`` follows the symlink between them, so an
absolute path would make the committed bytes machine-dependent.

The sibling normalises every solver's output into Forward-Right-Down, and that
is ``plane/dynamics.py``'s own frame
-------------------------------------------------------------------------------
``aid/axes.py`` is the sibling's single home for solver sign conventions and
``aid/tornado/coeff.py``'s last statement is ``return to_frd("tornado", out)``, so
every number this adapter reads is **already** x forward, y right, z DOWN, with
positive ``Cl`` rolling right, positive ``Cm`` nose up and positive ``Cn`` yawing
right -- ``plane/dynamics.py``'s exact frame and senses.  **There is therefore no
frame mapping anywhere in this adapter, and no moment needs its sign changed for
frame reasons.**  Tornado's control rows are already Forward-Right-Down too, and
``aid/tornado/control_deriv.py`` says on purpose that it does not re-wrap them
("wrapping this dict would flip them twice").

This adapter previously mapped ``cz = -CL; cl = -Cl; cm = +Cm; cn = -Cn; cy = +CY``
and carried three tables justifying each negation by Tornado's aft-positive x or
up-positive z.  Those tables are gone: the negations they justified are gone with
them.  What remains is not a mapping but an IDENTITY derived from the flight model,
and it is worth being precise about the difference, because only one of the two is
a conversion:

* ``cz = -CL``.  ``plane/dynamics.py`` applies ``az = +qbar*S*cz/m``, so ``cz`` is the
  body-+z coefficient with **z down**: up-positive lift enters as a negative ``cz``.
  That follows from ``xd[11]`` being the NED altitude rate, from the flight model
  alone, and it is what makes ``data/planes/linear/morelli.json``'s ``cz[1] = -4.5``
  CORRECT rather than a defect.  It applies to every slot whose solver channel is a
  ``CL`` lift channel -- which for Tornado is ``cz[1]`` (``-CL_a``), ``cz[0]``
  (``-CL(0)``) and ``cz[5]`` (``-CL_de``), not the two this paragraph used to name:
  ``cz[0]`` was missing, and item 5 of the same generated file already said so while
  item 6 denied it.  ``identity_negated`` now derives the set from
  ``provenance.mapping`` and each header prints its own, because it differs per model
  (Tornado three, DATCOM three, AVL five, flow5 one).
* **Everything else is read as the solver reports it.**  ``cl``, ``cm``, ``cn``, ``cy``
  and ``cx`` are body coefficients whose senses already match, so ``Cl_b``, ``Cl_P``,
  ``Cn_R``, ``CX_Q``, ``Cm_a`` and the control rows all enter the arrays unaltered.
  In particular ``czq[0]`` and ``cxq[0]`` now hold Tornado's **body-axis** ``CZ_Q`` and
  ``CX_Q`` rather than the wind-axis ``CL_Q`` the plan's mapping table names, because
  the schema ADDS both slots to the body-axis ``cz`` and ``cx``.  The wind-axis numbers
  are still recorded, in ``provenance.cd_q_convention``, because both in-tree
  reference files store the standard-aero ``CL_q``/``CD_q`` in these two slots and a
  reader comparing against them needs to know which is which.

What this re-base changed in the written numbers: ONE slot, and it is deliberate.
``czq[0]`` now holds the body-axis ``CZ_Q`` instead of the wind-axis ``CL_Q``, which
the schema's own arithmetic requires (it adds the slot to the body-axis ``cz``); the
two differ by about a quarter of a per cent here because CL and CZ part company
wherever the axial force is non-zero.  Every other written coefficient is unchanged,
because every slot that used to be negated and then flipped by the invariant table is
now already on the invariant side of zero.  Both numbers are kept in
``provenance.cd_q_convention``, so no count needs stating here: read
``len(provenance.flipped)`` from the run rather than trusting a sentence, which is
what the header now does.

``shift_moments_to_cg``'s ``cz`` is the **body-z force** coefficient, not the
wind-axis lift: the pitch line is ``dx * F_z / c_ref``, so the argument is
``coeff_create``'s ``CZ_a``, ``CZ_Q`` and the reconstructed ``CZ_de`` -- each read
straight, none negated, because ``CZ`` is already the body-z channel in
Forward-Right-Down.  ``Cl`` is not a shift argument at all: with ``dy = dz = 0`` no
force reaches the roll axis.  ``CY`` is in no solver's sign map, so the side force
passes through unaltered and does reach the yaw line.

Slopes shift, intercepts shift differently
------------------------------------------
``shift_moments_to_cg`` moves a *coefficient*, so what it needs beside it is
the force coefficient that goes with the same independent variable:

* a **slope** w.r.t. alpha (``Cm_a``) needs the **body-z** lift slope
  ``cz = CZ_a``, because ``d/dalpha`` of the shift ``dx * CZ / c_ref`` is
  ``dx * CZ_alpha / c_ref`` and the quantity in that expression is the body-z
  force, not the wind-axis lift.  ``|CZ_a|`` and ``CL_a`` differ here by 0.34 %
  (``5.165581`` against ``5.148013`` at alpha = 4 deg) because the aircraft carries
  NP[0], and using the lift slope there leaves that much linearisation error in
  ``cm[1]``.  Note the SIGN: ``CZ_a`` arrives NEGATIVE, because z is down, so the
  identity and the frame agree for once and no negation is needed;
* an **intercept** (``Cm0``, from the alpha = 0 run) needs the **body-z** lift
  **intercept** ``cz = CZ(0)``, because the constant force arm does not
  differentiate away -- and at alpha = 0 ``CZ(0)`` and ``-CL(0)`` are the same
  number, so the identity that gives ``cz[0]`` its sign is also the right shift
  argument;
* a **slope** w.r.t. a control deflection (``Cm_de``) needs the body-z force
  slope w.r.t. that same deflection for exactly the first reason.
  ``tornado_controls`` reports the wind-axis ``CL`` and ``CD`` rather than a body
  ``CZ``, so it is reconstructed exactly: with ``s = sin(alpha)``, ``c = cos(alpha)``
  the pair ``[dCL; dCD] = [[-s, c], [c, s]] . [dCX; dCZ]`` and that matrix is its own
  inverse, so ``dCZ = c*dCL + s*dCD`` -- positive here, and again needing no
  negation;
* a **slope** w.r.t. a rate (``Cm_Q``) needs ``cz = CZ_Q``, same reason;
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
all.  The sign-convention negations that remain are the slots the schema itself
stores in the wind-axis lift convention, because ``cz = -CL``; which ones those are
differs per model (Tornado's ``cz[0]``/``cz[1]``/``cz[5]``, AVL's plus ``czq[0]`` and
``cx[0]``, flow5's ``cz[5]`` alone), so each header now derives its own list from
``provenance.mapping`` via ``identity_negated`` rather than asserting a shared one.

Four defects in the sibling package sit on these paths and are corrected here
rather than inherited, each with the measurement that shows it:

* ``aid.stability._per_deg``, ``aid.handbook_controls._per_deg`` and
  ``aid.handbook_pass._per_rad`` are all ``if a > 1: convert else pass``.  This
  planform's lift slopes are 0.076-0.084 /deg, so every one is a no-op.
* **The Fortran's own stability table is per degree.**  Measured against its own
  ``cl`` column, ``cla`` equals the per-degree secant ``dCL/dalpha`` (0.1003/deg
  against a centred 0.100375/deg secant at alpha = 4 deg), so every derivative
  read out of ``for006`` is multiplied by ``DEG_TO_RAD``.
* ``aid/lateral.py:570`` ends its vertical-tail lift-slope formula with
  ``vt_a * pi/180``, which is the wrong conversion in the wrong direction.  The
  formula's ``k`` comes from ``aid/lateral.py:565``, which reads ``HT["a0"]`` --
  not ``VT.a0``, which appears nowhere in the sibling tree -- and ``apply_handbook``
  rewrites it to **6.8396 per radian**, so ``k = 1.0886/rad`` and the formula's
  answer is **3.5004 per radian**.  That is the right size for a fin of AR ~ 1.8:
  51 per cent of the 2-D 6.8396/rad, the reduction a low-aspect-ratio tail must
  show, where a per-degree reading would imply about 200/rad.  The trailing
  ``* pi/180`` then divides a per-radian number by 180 and stores 0.0610931, so
  every quantity linear in that slope -- ``CYb``, ``Cnb``, ``Clb``, ``Cn_r`` and
  the DATCOM Section 7 rate terms -- is ``DEG_TO_RAD`` too small and is recomputed.
  The correction is a multiply by ``DEG_TO_RAD`` because that is the numerical
  reciprocal of the bad division; it is **not** a per-degree-to-per-radian
  conversion, and the two descriptions only coincide numerically.  The evidence it
  is a defect: AID's ``Clb`` is -0.0031 against the sibling's own AVL gold -0.065868
  for the same planform, while the corrected -0.0690 is within 5 % of that gold.
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
``flow5.jsonc``           21   the twelve ``StabDerivatives``, the lateral control
                                rows and the ``CDvis``/``CDind`` split; no q-derivative,
                                no ``cy_da``
======================  ====  ==========================================================

**flow5 fills twenty-one of the twenty-five fields**, read off the 26 keys
``aid.flow5_io.run_flow5`` returns at mesh ``("10", "10")``: the six longitudinal
polar channels, the twelve ``StabDerivatives`` (``CZa``, ``CXa``, ``CYb``,
``CYp``, ``CYr``, ``Clb``, ``Clp``, ``Clr``, ``Cnb``, ``Cnp``, ``Cnr``, ``XNP``),
``CLa``/``Cma``, a ``CDvis``/``CDind`` split, the body ``Cx``/``Cz``/``Cl``/``Cn``
point channels and the alpha/beta schedules.  So every sideslip and every roll- and
yaw-rate derivative is real, and all five aileron/rudder coefficients are real.

The four it cannot fill are absent because its *runner* declines to forward them, not
because the panel method cannot compute them: ``FLOW5/run/flow5_run.cpp``'s
``stab_derivative_fields()`` does not pass on ``CXq``/``CZq``/``Cmq``, so ``cl_q``,
``cm_q`` and ``cd_q`` are empty; and ``aid.flow5_controls``' ``_KEEP`` table asks the
aileron for ``Cl`` and ``Cn`` but never for ``CY``, so ``cy_da`` is empty.  All four
are written as ``None`` and DECLARED, never as zeros, so an honestly empty slot cannot
be read as a measurement.

Two flow5 facts constrain how far its numbers can be trusted, and both are recorded
in the file rather than left to a reader to rediscover.  Its twelve
``StabDerivatives`` are **beta-flat** -- ``computeStabilityDerivatives`` and
``computeAngularDerivatives`` both hardcode ``double beta(0.0)`` -- while its
``CLa``/``Cma`` are polar OLS slopes that *do* follow beta; we fly beta = 0, so
nothing is wrong today.  And its ``Clb`` is **mesh-dependent to the point of
inverting sign**: -0.0736 at ``("5","3")``, **+0.0018** at ``("10","5")``, -0.0530 at
``("10","10")``, -0.0593 at ``("20","10")``, against AVL's -0.045756.  The positive
value at ``("10","5")`` would pass a sign-only invariant table while being badly
wrong, which is why ``FLOW5_MESH`` is pinned by a test rather than left to a comment.

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
import re
import sys
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from typing import Any, Iterator, Sequence

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

# Path.resolve() follows the symlink between the two mount points this repo is
# reachable from on this machine, so an absolute path baked into a generated file
# would differ between them and two builds on the two paths would not produce the
# same bytes.  Every path a generated file records therefore goes through
# `_stable_path`, which expresses it relative to REPO_ROOT.
REPO_ROOT = Path(__file__).resolve().parents[2]
AID_SRC_ENV = "AID_SRC"
DEFAULT_AID_SRC = REPO_ROOT.parent / "USAF_DATCOM" / "AircraftIntuitiveDesign" / "Python" / "src"
DEFAULT_SOURCE = REPO_ROOT.parent / "USAF_DATCOM" / "AircraftIntuitiveDesign" / "Analyses" / "Cessna172.jsonc"
DEFAULT_OUT_DIR = REPO_ROOT / "data" / "planes" / "cessna172"
BUILD_COMMAND = "python3 scripts/build_cessna172_aero.py"

# Where build_all puts each cross-check solver's rest-of-output.  Named once so a
# model file's header and its provenance cannot disagree about the location.
RAW_OUTPUT_LOCATION = "geometry.jsonc, under raw_cross_checks.{model}"

# The three cross-check runs, keyed by model name, so build_all and main() share
# one dispatch instead of two copies of it.
_CROSS_CHECK_RUNNERS = {"datcom": "datcom_run", "avl": "avl_run", "flow5": "flow5_run"}

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
    """One Tornado solve, normalised, with everything needed to re-derive it.

    Deliberately the same shape as ``CrossCheckRun``: ``_slot_lines`` reads
    ``run.coefficients`` and is shared by the Tornado and cross-check headers, so
    the two run types have to carry the same attributes.  Tornado's header build
    raised ``AttributeError: 'TornadoRun' object has no attribute 'coefficients'``
    before this field was stored.
    """

    derivatives: DerivativeSet
    coefficients: dict[str, list[float]]
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


def _aid_commit() -> str | None:
    """The sibling package's HEAD commit, read with its own git, read-only.

    Recorded because this project has been broken by an upstream commit TWICE, both
    times silently and both times found only by a failing test rather than by a
    diff: once when ``aid.tornado.coeff.py`` began returning Forward-Right-Down, so
    the moment shift double-negated, and once when ``flow5_run`` began emitting
    lateral channels, so a file that declared eighteen slots missing stopped being
    true.  A rebuild that moves the sibling should then be a one-line diff in this
    field rather than a mystery.

    Read with ``git -C <dir> rev-parse HEAD`` through the sibling's own repository.
    Nothing is written there, nothing is vendored, and no requirements entry is
    added.  ``None`` when the path is not a git checkout, which is the case for an
    ``$AID_SRC`` override pointing at a plain directory.
    """
    import subprocess

    root = aid_src().resolve()
    for candidate in (root, *root.parents):
        if (candidate / ".git").exists():
            try:
                out = subprocess.run(
                    ["git", "-C", str(candidate), "rev-parse", "HEAD"],
                    check=True, capture_output=True, text=True, timeout=20,
                )
            except (subprocess.SubprocessError, OSError):
                return None
            return out.stdout.strip() or None
    return None


def _stable_path(path: Path | str | None) -> str:
    """A path string that does not change with the mount point.

    This repository is reachable as both ``/home/valentin/Projects/FlightSimulation/
    FixedWing`` and ``/mnt/VirtualMachine/FlightSimulation/FixedWing`` on the machine
    that generated these files -- the first is a symlink onto the second -- and
    ``Path.resolve()`` follows it.  A generated file that recorded an absolute path
    would therefore carry the mount prefix into the committed bytes and would not
    be reproducible on the other path.  Everything inside or beside the repo is
    recorded relative to the repo root instead, which is where a reader is standing
    anyway; the sibling is a sibling, so it reads ``../USAF_DATCOM/...``.

    ``os.path.relpath`` is used rather than a hand-rolled prefix strip so that an
    ``$AID_SRC`` override pointing anywhere still gets a deterministic string, even
    if that string is a long chain of ``..``.
    """
    return os.path.relpath(Path(path if path is not None else DEFAULT_SOURCE).resolve(), REPO_ROOT)


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
    # `coeff_create` returns Forward-Right-Down (aid/tornado/coeff.py's last line is
    # `return to_frd("tornado", out)`), which is x forward, y right, z DOWN with
    # positive Cl rolling right, positive Cm nose up and positive Cn yawing right --
    # exactly `plane/dynamics.py`'s frame, so NOTHING here is converted for frame
    # reasons and `aid/axes.py` owns every sign.
    #
    # Two negations SURVIVE and neither is a frame conversion:
    #
    # 1. `cz = -CL`.  `plane/dynamics.py` applies `az = +qbar*S*cz/m`, so `cz` is the
    #    body-+z coefficient with z DOWN, i.e. up-positive lift enters as a negative
    #    `cz`.  That identity is derived from the flight model, not from a solver, and
    #    both in-tree reference files store CL_alpha in `cz[1]`.  Tornado's `CZ_a` is
    #    the body-z coefficient itself and is read WITHOUT this negation.
    # 2. `cz[5] = -CL_de`, the same identity applied to the elevator.
    #
    # The SLOT value and the SHIFT argument are still deliberately different numbers:
    # `cz[1]` is the wind-axis lift slope `CL_a`, while the shift's algebra contains
    # the body-z force (`M_G = M_O + (O - G) x F`, pitch line `dx * F_z / c_ref`), so
    # it shifts with `CZ_a`.  The two differ by 0.34 % at alpha = 4 deg because CL and
    # CZ part company wherever the axial force is non-zero, which it is here -- NP[0]
    # is in the lattice.
    cz_alpha = -cl_alpha                       # slot: -CL_a, the cz = -CL identity
    cz_alpha_body = float(coeffs["CZ_a"])      # shift argument: CZ_a, already body-z
    collector.put(
        "cl_alpha", cz_alpha, tornado_key="CL_a", solver_value=cl_alpha,
        conversion="cz = -CL_alpha, and that is the plane/dynamics.py identity (its "
                    "body +z axis points down), NOT a frame conversion -- coeff_create "
                    "already returns Forward-Right-Down.  Per radian already",
        role="slope",
    )

    # CL0 is an INTERCEPT: the alpha = 0 run gives it outright, no extrapolation,
    # and it is the lift slope's intercept, so it is also what cm0 shifts with.
    cl_at_zero = float(zero["coeffs"]["CL"])
    cl0 = -cl_at_zero
    collector.put(
        "cl0", cl0, tornado_key="CL(alpha=0)", solver_value=cl_at_zero,
        conversion="cz = -CL, the plane/dynamics.py identity; a second solve at "
                   "alpha = 0 rad, no extrapolation",
        role="intercept",
    )

    # czq[0]: the schema ADDS this slot to the body-z `cz`, and coeff_create's `CZ_Q`
    # IS the body-z pitch-rate coefficient, already Forward-Right-Down, so it is read
    # directly with no negation of any kind.  The wind-axis `CL_Q` is recorded beside
    # it because both in-tree files store the standard-aero CL_q in this slot and a
    # reader comparing against them needs to know which number is which; the two
    # differ by 0.25 % here, since CL and CZ part company wherever the axial force is
    # non-zero (NP[0] is in the lattice).
    cz_q = float(coeffs["CZ_Q"])
    cz_q_body = cz_q
    collector.put(
        "cl_q", cz_q, tornado_key="CZ_Q", solver_value=float(coeffs["CZ_Q"]),
        conversion="read directly: CZ_Q is the body-z pitch-rate coefficient and "
                   "czq[0] is added to the body-z cz, and coeff_create already returns "
                   "Forward-Right-Down, so no identity and no conversion apply.  "
                   f"Wind-axis CL_Q = {float(coeffs['CL_Q']):.6f} is recorded as "
                   "provenance.cd_q_convention.wind_axis_cl_q; the two differ by 0.25 %",
        role="slope",
    )

    # --- side force -----------------------------------------------------------
    # `CY` is never in any solver's sign map -- aid/axes.py: "the CY channel and its
    # flow-angle derivatives are never flipped, by any solver" -- so it is read
    # exactly as reported.  It is the body-y coefficient, which is what
    # `plane/dynamics.py` reads; coeff_create's other side force, `CC`, is the
    # wind-axis one, and the two are identical at betha = 0 because the rotation's
    # side-force row is then (0,1,0).
    cy_beta = float(coeffs["CY_b"])
    cy_p = float(coeffs["CY_P"])
    cy_r = float(coeffs["CY_R"])
    collector.put(
        "cy_beta", cy_beta, tornado_key="CY_b", solver_value=float(coeffs["CY_b"]),
        conversion="read directly: CY is in no solver's sign map (aid/axes.py), so it "
                   "is already the body-y slope plane/dynamics.py reads; CC_b is the "
                   "wind-axis side force and equals CY_b exactly at betha = 0",
        role="slope",
    )
    collector.put(
        "cy_p", cy_p, tornado_key="CY_P", solver_value=float(coeffs["CY_P"]),
        conversion="read directly; CY_P is already Forward-Right-Down and already "
                   "divided by fac = c_mac/(2V)", role="slope",
    )
    collector.put(
        "cy_r", cy_r, tornado_key="CY_R", solver_value=float(coeffs["CY_R"]),
        conversion="read directly; CY_R is already Forward-Right-Down and already "
                   "divided by fac = c_mac/(2V)", role="slope",
    )

    # --- roll moments ---------------------------------------------------------
    # `Cl` is in Tornado's sign map with a -1, so coeff_create's roll channel is
    # already F-R-D and is read directly: no negation.  The old `cl = -Cl_*` mapping
    # existed only to reverse Tornado's aft-positive x, and the sibling now does it.
    for field, key in (("cl_beta", "Cl_b"), ("cl_p", "Cl_P"), ("cl_r", "Cl_R")):
        collector.put(
            field, float(coeffs[key]), tornado_key=key, solver_value=float(coeffs[key]),
            conversion="read directly: coeff_create returns Forward-Right-Down and "
                       "positive Cl already rolls right, which is what "
                       "plane/dynamics.py's cl means.  Already divided by fac",
            role="slope",
        )

    # --- pitch ----------------------------------------------------------------
    # `Cm` takes NO sign-map entry (it is about the one shared axis, y, and
    # coeff.py:108 states it is the body-y moment, not a wind-axis one), so it is read
    # directly and was already un-negated before the F-R-D work.  Cm_a is a SLOPE in
    # alpha, so it shifts with the BODY-Z lift SLOPE.
    _, cm_alpha, _ = _shift(geometry, cz=cz_alpha_body, cy=0.0, cm=float(coeffs["Cm_a"]), cn=0.0)
    collector.put(
        "cm_alpha", cm_alpha, tornado_key="Cm_a", solver_value=float(coeffs["Cm_a"]),
        conversion="read directly, then shifted with cz = +CZ_a, the body-z force "
                   "slope the shift's algebra contains (not CL_a: F_z, not the "
                   "wind-axis lift)",
        role="slope",
    )
    # Cm0 is an INTERCEPT, so it shifts with the lift INTERCEPT, not the slope.
    # At alpha = 0 the body-z and wind-axis lifts coincide exactly (the rotation is
    # the identity), so CL(0) is the right argument and is also what cl0 uses.
    cm_at_zero = float(zero["coeffs"]["Cm"])
    _, cm0, _ = _shift(geometry, cz=cl0, cy=0.0, cm=cm_at_zero, cn=0.0)
    collector.put(
        "cm0", cm0, tornado_key="Cm(alpha=0)", solver_value=cm_at_zero,
        conversion="read directly, then shifted with the matching body-z intercept "
                   "cz = CZ(0) = -CL(0) (at alpha = 0 the body and wind lifts "
                   "coincide); cm0 carries no sign invariant and is never normalised",
        role="intercept",
    )
    # Cm_Q is a SLOPE in q, so it shifts with the body-z slope in q.
    _, cm_q, _ = _shift(geometry, cz=cz_q_body, cy=0.0, cm=float(coeffs["Cm_Q"]), cn=0.0)
    collector.put(
        "cm_q", cm_q, tornado_key="Cm_Q", solver_value=float(coeffs["Cm_Q"]),
        conversion="read directly, then shifted with cz = +CZ_Q, the body-z force "
                   "slope in q; already divided by fac",
        role="slope",
    )

    # --- yaw ------------------------------------------------------------------
    # `Cn` is in Tornado's sign map with a -1, so coeff_create's yaw channel is
    # already F-R-D with positive Cn yawing right, which is what `plane/dynamics.py`'s
    # cn means: read directly.  The old `cn = -Cn_*` mapping existed only to reverse
    # Tornado's up-positive z.
    for field, key, side in (
        ("cn_beta", "Cn_b", "CY_b"), ("cn_p", "Cn_P", "CY_P"), ("cn_r", "Cn_R", "CY_R"),
    ):
        shifted = _shift(
            geometry, cz=0.0,
            cy={"Cn_b": cy_beta, "Cn_P": cy_p, "Cn_R": cy_r}[key],
            cm=0.0, cn=float(coeffs[key]),
        )[2]
        collector.put(
            field, shifted, tornado_key=key, solver_value=float(coeffs[key]),
            conversion=f"read directly (coeff_create returns Forward-Right-Down, and "
                       f"positive Cn already yaws right), then shifted on dx*cy with the "
                       f"body-y slope {side}; already divided by fac",
            role="slope",
        )

    # --- drag ------------------------------------------------------------------
    # `CX` is in Tornado's sign map with a -1, so `CX_Q` arrives already F-R-D with
    # drag negative, and `cxq[0]` is added to the body-x `cx`: read directly, no
    # negation.  The schema ADDS `cxq[0]` to the body-x coefficient, so this slot and
    # `czq[0]` above both now hold the body-axis coefficient the schema actually wants;
    # see provenance.cd_q_convention for the wind-axis numbers beside them, since both
    # in-tree files store the standard-aero CD_q / CL_q in these two slots.
    collector.put(
        "cd_q", float(coeffs["CX_Q"]), tornado_key="CX_Q",
        solver_value=float(coeffs["CX_Q"]),
        conversion="read directly: CX_Q is the body-x pitch-rate coefficient and "
                   "cxq[0] is added to the body-x cx, and coeff_create already returns "
                   "Forward-Right-Down.  Already divided by fac",
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
    # multiplied by DEG_TO_RAD before it goes near a Morelli array.  Its `Cl`/`Cn`/
    # `Cm` come straight out of `coeff_create`, which is already F-R-D, and the
    # sibling deliberately does NOT re-wrap them (aid/tornado/control_deriv.py's own
    # comment: "No to_frd(\"tornado\", ...) here on purpose ... wrapping this dict
    # would flip them twice").  So all three are read directly with no negation.
    # Its `CY` is coeff_create's CC, the wind-axis side force; at betha = 0 that
    # equals the body-y force the model reads, and `CY` is in no solver's sign map.
    # `CL` and `CD` remain wind-axis, which is why `cz[5]` below still carries the
    # `cz = -CL` identity.
    aileron, elevator, rudder = rows["aileron"], rows["elevator"], rows["rudder"]

    def control(field: str, surface: str, key: str, negate: bool = False) -> None:
        per_degree = float(surface[key])
        magnitude = per_degree_to_per_radian(per_degree)
        collector.put(
            field, -magnitude if negate else magnitude,
            tornado_key=f"tornado_controls[{surface['surface']}].{key}",
            solver_value=per_degree,
            conversion=(
                f"per degree x {DEG_TO_RAD}, then read directly: the row's "
                f"{key} comes out of coeff_create, which already returns "
                "Forward-Right-Down, and control_deriv.py deliberately does not "
                "re-wrap it"
                + (
                    ".  The one negation that survives is the plane/dynamics.py "
                    "identity cz = -CL, the same one that applies to cz[1]"
                    if negate else
                    "; no identity and no frame conversion apply"
                )
            ),
            role="slope",
        )

    control("cl_da", aileron, "Cl")
    control("cy_da", aileron, "CY")
    control("cn_da", aileron, "Cn")
    control("cl_dr", rudder, "Cl")
    control("cy_dr", rudder, "CY")
    control("cn_dr", rudder, "Cn")
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
    # `cz[5]` keeps the plane/dynamics.py identity cz = -CL, the same one that gives
    # cz[1] its sign; the shift argument is the body-z slope, exactly as for alpha
    # and q.
    #
    # That reconstruction's SIGN is measured, not reasoned.  Two solves at elevator
    # delta = +-1 deg on the deflected lattices, reading coeff_create's own CZ, give
    # dCZ/ddelta = -0.006487/deg -- the negative of the +0.006472 the rotation above
    # produces, because that rotation turns the wind-axis CL into a z-UP normal force
    # and coeff_create's CZ is z DOWN.  So the rotation is negated:
    cl_de_slot = -per_degree_to_per_radian(cl_de_per_degree)
    cz_de = -per_degree_to_per_radian(cz_de_per_degree)
    cm_de_o = per_degree_to_per_radian(float(elevator["Cm"]))
    _, cm_de, _ = _shift(geometry, cz=cz_de, cy=0.0, cm=cm_de_o, cn=0.0)
    collector.put(
        "cl_de", cl_de_slot, tornado_key="tornado_controls[elevator].CL",
        solver_value=cl_de_per_degree,
        conversion=f"per degree x {DEG_TO_RAD}, then cz = -CL_de, the "
                   "plane/dynamics.py identity (its body +z axis points down) and "
                   "NOT a frame conversion -- the row's CL is wind-axis, which is why "
                   "the identity applies here at all; the body-z slope that shifts "
                   "cm_de is reconstructed from the row's CL and CD and recorded as "
                   "provenance.raw CZ_de",
        role="slope",
    )
    collector.put(
        "cm_de", cm_de, tornado_key="tornado_controls[elevator].Cm",
        solver_value=float(elevator["Cm"]),
        conversion=f"per degree x {DEG_TO_RAD}, Cm read directly (it comes out of "
                   f"coeff_create, already F-R-D), then shifted with the matching "
                   f"body-z force slope cz = CZ_de = (cos a * CL_de + "
                   f"sin a * CD_de) x {DEG_TO_RAD}, reconstructed exactly from the "
                   "row's wind-axis CL and CD",
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
        "source": _stable_path(source),
        "aid_src": _stable_path(aid_src()),
        "aid_src_from_env": bool(os.environ.get(AID_SRC_ENV)),
        "aid_src_commit": _aid_commit(),
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
            "note": "Tornado reports about ref_point and the sibling re-expresses every "
                    "coefficient in Forward-Right-Down, where the reference station is 2.94 ft "
                    "AHEAD of the CG in the x-FORWARD sense; the CG offset passed to "
                    "shift_moments_to_cg is therefore dx = -ft(2.94) m",
        },
        "sign_convention": {
            "tornado_axes": "x forward, y right, z DOWN, as RETURNED -- coeff_create ends "
                            "with `return to_frd(\"tornado\", out)` and aid/axes.py is the "
                            "sibling's single home for solver signs",
            "frame_conversion_applied": "none: the sibling already returns "
                                        "plane/dynamics.py's own frame, so no moment and no "
                                        "force needs its sign changed for frame reasons",
            "only_surviving_negations": "cz = -CL on cz[1] and cz[5], which is the "
                                        "plane/dynamics.py identity (its body +z axis points "
                                        "down) and not a conversion",
            "dynamics_axes": "x forward, y right, z down (plane/dynamics.py)",
            "cz": "cz = -CL on cz[1] and cz[5] only; cl, cm, cn, cy and cx are read exactly as "
                  "reported",
            "rate_derivatives": "none: coeff_create returns F-R-D for every channel, including "
                                "the p/r columns, and Tornado's own Cl_P and Cn_R now AGREE "
                                "with AVL's (which is unmapped and therefore natively F-R-D). "
                                "The former claim that Tornado's P and R columns came back "
                                "component-sense inverted no longer applies and was removed",
            "normalisation": "aero_convert.units.normalise_sign against SIGN_INVARIANTS",
        },
        "cd_q_convention": {
            "shipped": "czq[0] = coeff_create['CZ_Q'] and cxq[0] = coeff_create['CX_Q'], the "
                       "BODY-axis pitch-rate coefficients, read unnegated",
            "why": "the schema ADDS both slots to the body-axis cz and cx, so the body-axis "
                   "coefficient is what they mean.  The sibling returns both in "
                   "Forward-Right-Down, so no identity and no conversion apply",
            "wind_axis_cl_q": float(coeffs["CL_Q"]),
            "wind_axis_cl_q_slot_if_standard_aero": -float(coeffs["CL_Q"]),
            "body_axis_cz_q_shipped": float(coeffs["CZ_Q"]),
            "difference_percent": 100.0
            * (float(coeffs["CL_Q"]) - abs(float(coeffs["CZ_Q"])))
            / abs(float(coeffs["CL_Q"])),
            "in_tree_files_store": "the standard-aero CL_q and CD_q (data/planes/linear/"
                                   "morelli.json has czq[0] = -2.0, cxq[0] = 0.0), so a reader "
                                   "comparing against them needs wind_axis_cl_q_slot_if_"
                                   "standard_aero above; the two differ here by about a "
                                   "quarter of a per cent because CL and CZ part company "
                                   "wherever the axial force is non-zero, which it is -- NP[0] "
                                   "is in the lattice",
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
        coefficients=coefficients,
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
        "source": _stable_path(source),
        "aid_src": _stable_path(aid_src()),
        "aid_src_from_env": bool(os.environ.get(AID_SRC_ENV)),
        "aid_src_commit": _aid_commit(),
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
            "flap": "the Morelli schema has no flap slot, so the handbook's flap row maps to "
                    "nothing.  It is NOT carried in this file; provenance.raw_output_location "
                    "says where the build put it, and provenance.control_rows holds it",
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
# The V_trim/V_source ratio the two FEET-frame solvers carry, for the flow5 header to
# quote beside its own.  Computed here from the same constant rather than typed, so the
# two cannot drift apart.
SPEED_FACTOR_FEET_FRAME = 23.1146 / (SOURCE_SPEED_AS_REPORTED * FT_TO_M)


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
        "note": f"{solver} reports every moment about AERO.XCG, and the station was read back "
                "out of the input the SOLVER was given rather than out of the same in-memory "
                "value, then compared with the source; dx = dy = dz = 0, so no moment shift "
                "is applied",
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

    ``vt_a_per_rad`` is the CORRECTED per-radian vertical-tail lift slope: the one
    on the aircraft has been divided by ``pi/180`` at ``aid/lateral.py:570`` (see
    ``_datcom_lateral``), so it cannot be read back off it.
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

    ``aid/lateral.py:570`` ends its tail lift-slope formula with
    ``vt_a = vt_a * pi / 180``, which is the wrong conversion in the wrong
    direction.  The formula itself,

        vt_a = 2*pi*AR_eff / (2 + sqrt((AR_eff*B/k)^2*(1+tan^2(L)/B^2) + 4)),
        k   = a0 / (2*pi),                        <- aid/lateral.py:565, reads HT["a0"]

    returns a slope in the units of the Helmbold ``k`` it was fed, and AID feeds it
    ``apply_handbook``'s rewritten ``HT.a0`` -- 6.8396 PER RADIAN on this planform,
    so ``k = 1.0886/rad`` and the formula's answer is **3.5004 per radian**.  That
    is the physically right size for a fin of AR ~ 1.8: 3.5004 is 51 per cent of the
    2-D 6.8396/rad, the reduction a low-aspect-ratio tail must show.  Reading it as
    per DEGREE instead would imply about 200/rad, which is not a lift slope anything
    has.  The trailing ``* pi/180`` then divides a per-radian number by 180 and
    stores 0.0610931, i.e. ``DEG_TO_RAD`` too small, and every quantity linear in
    that slope inherits it.

    So the correction here is numerically a multiply by ``DEG_TO_RAD`` -- undoing
    that division -- and NOT a per-degree-to-per-radian conversion.  The two
    descriptions coincide numerically because the error happens to be a factor of
    180 the wrong way, and only the second one is true.

    Measured, which is what settles it: with the sibling's value ``Clb =
    -0.003091`` against the sibling's own AVL gold ``Clb = -0.065868`` for the same
    planform, a factor 21.7; with the correction ``Clb = -0.068966``, within 5 per
    cent of that gold.

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
    # Undo aid/lateral.py:570's `* pi/180`, which divided a per-radian slope by
    # 180.  DEG_TO_RAD is the numerical reciprocal of that division; it is NOT a
    # per-degree-to-per-radian conversion here -- see the docstring.
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
        "units_factor_meaning": "aid/lateral.py:570 divided a PER RADIAN slope by pi/180, "
                                "so this multiplies it back by DEG_TO_RAD.  It is the "
                                "numerical reciprocal of that division, not a "
                                "per-degree-to-per-radian conversion",
        "ht_a0_per_radian_as_read": float(_last(ac.HT["a0"])),
        "helmbold_k_per_radian": float(_last(ac.HT["a0"])) / (2.0 * math.pi),
        "mechanism": "aid/lateral.py:565 reads HT['a0'] (rewritten by apply_handbook to "
                     "6.8396 PER RADIAN), so k = a0/(2*pi) = 1.0886/rad and the tail slope "
                     "formula returns 3.5004 PER RADIAN -- 51 per cent of the 2-D 6.8396/rad, "
                     "the reduction a fin of AR ~ 1.8 must show.  aid/lateral.py:570 then "
                     "multiplies that by pi/180, which is the defect",
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
    return ac, {}, SimpleNamespace(load_jsonc=load_jsonc), _numpy_alias()


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
        table, cards = _run_datcom(ac, tmp / "datcom")
    solver_station, station_evidence = _datcom_cg_station(cards)
    stability = apply_handbook(
        ac, angl=False, slipstream=False, slipstream_data=(0.5, 0.9),
        multhopp=True, trim_mode=0, trim_fix="both",
    )
    lateral = dict(stability["lateral"])
    geometry = _reference_geometry(ac, mesh=("none", "none"))
    moment_reference = _verify_cg_reference(geometry, solver_station, solver="DATCOM")
    moment_reference.update(station_evidence)

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
                       f"tail's lift slope x {DEG_TO_RAD}.  aid/lateral.py:570 divides that "
                       f"PER RADIAN slope by pi/180, so AID's {key} = "
                       f"{float(lateral[key]):.6g} has a tail part 57.3x too small (AID's "
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


def _run_datcom(ac: Any, workdir: Path) -> tuple[dict, dict[str, Any]]:
    """Run the real Fortran binary in scratch; return its table and its echo.

    ``aid.datcom_run.run_datcom`` writes ``for005.dat`` into ``workdir`` and shells
    out to ``Matlab/fsroot/code/DATCOM/datcom`` there, which leaves both the card it
    was given (``for005.dat``) and the Fortran's own listing of the cards it read
    (``datcom.out``, copied to ``for006.dat`` by the wrapper) behind.  A missing
    binary surfaces as ``FileNotFoundError`` from ``aid.paths.datcom_wrapper`` and
    is recorded by the caller as a missing binary rather than raised into a crash,
    per the plan's "Fail-closed".

    Both files are returned because the second one is what makes the moment
    reference VERIFIABLE rather than assumed: the Fortran echoes its ``$SYNTHS``
    namelist back, and ``$SYNTHS XCG=`` is the station its moments are taken about.
    """
    from aid.datcom_run import run_datcom

    table = run_datcom(ac, workdir)
    return table, {
        "for005": _read_card(workdir / "for005.dat"),
        "fortran_echo": _read_card(workdir / "datcom.out"),
    }


def _read_card(path: Path) -> list[str]:
    """Every ``$`` namelist line a DATCOM card file or listing contains."""
    if not path.is_file():
        return []
    return [line for line in path.read_text(errors="replace").splitlines() if "$" in line]


def _card_value(lines: list[str], key: str) -> list[float]:
    """Every ``KEY=number`` on the ``$`` namelist lines, one per occurrence.

    A regex rather than a comma split, because a Fortran namelist's FIRST key is
    preceded by the namelist's own name -- ``$SYNTHS XCG=2.94,...`` -- so
    ``"SYNTHS XCG"`` would never compare equal to ``"XCG"``.  ``\b`` also stops
    ``XCG=`` from matching inside a longer name.
    """
    pattern = re.compile(r"\b" + re.escape(key) + r"\s*=\s*([-+0-9.eE]+)")
    return [float(match.group(1)) for line in lines for match in pattern.finditer(line)]


def _datcom_cg_station(cards: dict[str, list[str]]) -> tuple[float, dict[str, Any]]:
    """The moment station DATCOM actually used, read back out of its own output.

    ``aid/datcom_io.py``'s ``write_synths`` puts ``XCG={aero['XCG']}`` on the
    ``$SYNTHS`` namelist, and the Fortran echoes the whole namelist back into
    ``datcom.out``.  Every occurrence is collected and required to agree, so the
    number compared against ``AERO.XCG`` is the station the SOLVER read rather than
    the one this converter happened to be holding -- which is the whole point of the
    plan's "verify, do not assume".
    """
    written = _card_value(cards["for005"], "XCG")
    echoed = _card_value(cards["fortran_echo"], "XCG")
    evidence: dict[str, Any] = {
        "for005_synths_xcg": written,
        "fortran_echo_synths_xcg": echoed,
        "card_line": "the Fortran echoes its $SYNTHS namelist back into datcom.out; "
                     "$SYNTHS XCG is the station its moments are taken about",
    }
    if not echoed:
        raise ValueError(
            "the Fortran's own output carries no $SYNTHS XCG=, so the station its "
            "moments are taken about cannot be read back; comparing AERO.XCG with "
            "itself would prove nothing"
        )
    if any(value != echoed[0] for value in echoed):
        raise ValueError(f"the Fortran echoed $SYNTHS XCG= as {echoed}; it did not agree with itself")
    if written and any(value != written[0] for value in written):
        raise ValueError(f"for005.dat carries $SYNTHS XCG= as {written}, not one value")
    if written and written[0] != echoed[0]:
        raise ValueError(
            f"for005.dat has $SYNTHS XCG={written[0]} but the Fortran echoed "
            f"{echoed[0]}; it did not read the card this converter wrote"
        )
    evidence["source"] = "the Fortran's own echo of $SYNTHS XCG in datcom.out"
    return echoed[0], evidence


def datcom_derivatives(source: Path | None = None) -> DerivativeSet:
    """The normalised derivative set DATCOM produces for the source geometry."""
    return datcom_run(source).derivatives


def avl_run(source: Path | None = None) -> CrossCheckRun:
    """AVL over the source: stability derivatives from the ``.st`` sweep, control from the ``.sb``.

    AVL is the one cross-check solver whose axis convention ALREADY matches
    ``plane/dynamics.py``: its own ``.sb`` header prints "Standard axis
    orientation, X fwd, Z down", the same x-forward, y-right, z-down frame and the
    same moment senses (positive ``Cl`` rolls right, positive ``Cm`` noses up,
    positive ``Cn`` yaws right).  So no axis mapping is applied at all.  The
    negations that remain are only the schema's own sign-convention identities --
    ``cz = -CL`` on the lift slots and ``cx = -CD`` on the drag slot -- and which
    slots those touch is DERIVED per file by ``identity_negated`` rather than
    listed here, because the set is different for every model and a hand-written
    list is exactly the kind of sentence that goes stale.  ``normalise_sign``
    still decides the sign of every slot from the invariant table.

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


# flow5's twelve StabDerivative channels, read off FLOW5/run/flow5_run.cpp's
# stab_derivative_fields() field list rather than typed: a name the runner stops
# emitting then shows up as absent here instead of silently missing from the record.
FLOW5_STAB_DERIVATIVE_KEYS = (
    "CZa", "CXa", "CYb", "CYp", "CYr", "Clb", "Clp", "Clr",
    "Cnb", "Cnp", "Cnr", "XNP",
)


def _flow5_deck_for_record(ac: Any) -> dict[str, Any]:
    """The deck this converter hands flow5, kept for the raw record in the file.

    flow5 has no other record of what it was asked for: the runner reads the deck and
    reports 26 numbers, so the deck's polar -- ``qinf_mps``, ``cog_m``, ``sref_m2``,
    the alpha schedule -- is the only place the flight condition it actually ran at
    survives.  ``cog_m`` is what the moment-reference check reads back.
    """
    from aid.flow5_io import write_flow5_deck

    return write_flow5_deck(ac, FLOW5_MESH)


def flow5_run(source: Path | None = None) -> CrossCheckRun:
    """flow5 over the source, through the ONE call that makes it Forward-Right-Down.

    ``aid.flow5_io.run_flow5(ac, mesh)`` is ``to_frd("flow5", run_flow5_native(...))``,
    and using it is the whole frame question here -- but it is worth saying why the
    obvious shortcut is wrong, because it is a trap:

    * flow5's **per-point polar channels** ``Cx``, ``Cz``, ``Cl``, ``Cn`` ARE in
      flow5's sign map, all ``-1``.  Measured before the flip, raw ``Cz`` tracked raw
      ``CL`` (``Cz`` = [+0.2349, -0.1315, +0.4967, ...] against ``CL`` = [-0.2353,
      +0.1315, +0.4970, ...]) -- i.e. up-positive, so raw ``Cz`` is *not* a body-z
      coefficient and needs the ``-1``.
    * flow5's **twelve StabDerivatives** (``CZa``, ``CXa``, ``CYb``, ``CYp``, ``CYr``,
      ``Clb``, ``Clp``, ``Clr``, ``Cnb``, ``Cnp``, ``Cnr``, ``XNP``) take **NO**
      sign-map entry, because ``PanelAnalysis::computeStabilityDerivatives``
      projects them onto the stability axes, which are already F-R-D.  Measured:
      ``CZa`` = -5.256228 while ``CLa`` = +5.169735, so ``CZa`` already carries the
      body-z sign and is what ``cz[1]`` wants.
    * ``CLa`` and ``Cma`` are OLS slopes over the polar and are **per radian**
      already, whatever their sign convention.

    So the two halves need opposite treatment and only ``run_flow5`` gets both right
    in one place.  Hand-applying ``to_frd`` to the derivative scalars would flip
    ``CZa`` positive and invert the lift slope.

    Control derivatives come from ``aid.flow5_controls``, whose default runner is the
    RAW one, so the same ``to_frd`` is injected here rather than left implicit --
    measured, the flipped aileron row is ``Cl`` = +0.005592/deg where raw is
    -0.005592/deg.

    What flow5 cannot produce, measured: no q-derivative at all (the library computes
    ``CXq``/``CZq``/``Cmq`` but ``FLOW5/run/flow5_run.cpp`` does not forward them),
    and no aileron side force, because ``aid/flow5_controls``'s ``_KEEP`` table asks
    the aileron for ``Cl`` and ``Cn`` only.  Those four slots are ``None``, zeroed and
    declared.
    """
    ac, shared, _aid, numpy_note = _cross_check_inputs(source)
    from aid.axes import to_frd
    from aid.flow5_controls import flow5_controls
    from aid.flow5_io import run_flow5, run_flow5_native
    from aid.paths import flow5_bin
    from aid.solver_overlay import FLOW5_BETA_FLAT

    with _scratch():
        if not flow5_bin().is_file():
            raise FileNotFoundError(f"the flow5 binary is not at {flow5_bin()}")
        polar = run_flow5(ac, FLOW5_MESH)
        deck = _flow5_deck_for_record(ac)
        control_rows = flow5_controls(
            ac, deltas_deg=(0.0,), mesh=FLOW5_MESH,
            run=lambda built: to_frd("flow5", run_flow5_native(built)),
        )
    by_surface = {row["surface"]: row for row in control_rows}
    geometry = _reference_geometry(ac, mesh=FLOW5_MESH)
    # `cog_m` is metres in the deck; `_verify_cg_reference` speaks feet, like
    # AERO.XCG.  The conversion goes through flow5's own 1/3.28084 on the way out and
    # back through this module's FT_TO_M, which is why the tolerance is 1e-4 ft.
    cog = [float(v) for v in deck["polar"]["cog_m"]]
    moment_reference = _verify_cg_reference(geometry, cog[0] / FT_TO_M, solver="flow5")

    schedule = [float(v) for v in np.asarray(polar["alpha"]).reshape(-1)]
    index = schedule.index(geometry.alpha_deg) if geometry.alpha_deg in schedule else 0
    zero = schedule.index(0.0)
    # `Cz` and `Cm` are the BODY-axis channels and are what the two intercept slots
    # mean; `CL` and `Cma` are the wind-axis / stability-slope spellings of the same
    # alpha dependence and are recorded beside them.
    cz_at_zero = float(np.asarray(polar["Cz"]).reshape(-1)[zero])
    collector = _Collector()
    raw: dict[str, Any] = {
        "mesh": list(FLOW5_MESH),
        "frame": "to_frd('flow5', ...) via aid.flow5_io.run_flow5 -- the per-point polar "
                 "channels Cx/Cz/Cl/Cn are in flow5's sign map with -1, the twelve "
                 "StabDerivatives take no entry because they are already stability-axis",
        "entry_point": "aid.flow5_io.run_flow5",
        "deck_polar": deck["polar"],
        "deck_wings": [
            {"name": wing.get("name"), "role": wing.get("role"),
             "sections": len(wing.get("sections", []))}
            for wing in deck["wings"]
        ],
        "output_keys": sorted(polar),
        "CLa": float(polar["CLa"]),
        "Cma": float(polar["Cma"]),
        "alpha_schedule_deg": schedule,
        "alpha_index": index,
        "alpha_deg": geometry.alpha_deg,
        "CL": [float(v) for v in np.asarray(polar["CL"]).reshape(-1)],
        "Cz": [float(v) for v in np.asarray(polar["Cz"]).reshape(-1)],
        "Cx": [float(v) for v in np.asarray(polar["Cx"]).reshape(-1)],
        "Cm": [float(v) for v in np.asarray(polar["Cm"]).reshape(-1)],
        "CD": [float(v) for v in np.asarray(polar["CD"]).reshape(-1)],
        "CDvis": [float(v) for v in np.asarray(polar["CDvis"]).reshape(-1)],
        "CDind": [float(v) for v in np.asarray(polar["CDind"]).reshape(-1)],
        "stab_derivatives": {
            name: float(polar[name]) for name in sorted(FLOW5_STAB_DERIVATIVE_KEYS)
            if name in polar
        },
        "control_rows_per_degree": {
            surface: {key: by_surface[surface].get(key)
                      for key in ("CL", "CD", "Cm", "CY", "Cl", "Cn")}
            for surface in ("aileron", "elevator", "rudder")
        },
        "control_row_frame": "to_frd('flow5', ...) injected as flow5_controls' runner; its "
                             "default runner is the RAW one, so without the injection the "
                             "aileron Cl and Cn would arrive with the wrong sign",
        "cog_m": cog,
        "beta_flat": sorted(FLOW5_BETA_FLAT),
    }
    # --- longitudinal static ---------------------------------------------------
    cza = float(polar["CZa"])
    collector.put(
        "cl_alpha", cza, tornado_key="flow5 CZa (StabDerivative, already stability-axis)",
        solver_value=cza,
        conversion="read directly: CZa is one of the twelve StabDerivatives, which take NO "
                   "sign-map entry because computeStabilityDerivatives projects onto the "
                   "stability axes, and it is already the body-z alpha slope with z DOWN.  "
                   "Measured corroboration: CLa = +5.169735 while CZa = -5.256228, so CZa "
                   "already carries this slot's sign and no negation applies",
        role="slope",
    )
    collector.put(
        "cm_alpha", float(polar["Cma"]), tornado_key="flow5 Cma",
        solver_value=float(polar["Cma"]),
        conversion="read directly; nose-up positive already, and cog_m puts the polar at the CG "
                   "so no moment shift applies",
        role="slope",
    )
    collector.put(
        "cl0", cz_at_zero, tornado_key="flow5 Cz at alpha = 0 deg",
        solver_value=cz_at_zero,
        conversion="read directly: Cz is the BODY-z normal force and is the per-point polar "
                   "channel flow5's sign map DOES flip with -1, so after to_frd it is already "
                   "the body-z coefficient this slot means.  The alpha = 0 row IS the "
                   "intercept",
        role="intercept",
    )
    collector.put(
        "cm0", float(np.asarray(polar["Cm"]).reshape(-1)[zero]),
        tornado_key="flow5 Cm at alpha = 0 deg",
        solver_value=float(np.asarray(polar["Cm"]).reshape(-1)[zero]),
        conversion="the alpha = 0 row; Cm0's sign is tail rigging, not a derivable "
                   "convention, so it is never sign-normalised",
        role="intercept",
    )
    # Parasite drag: flow5 now SPLITS CD, and the split is the measurement that settles
    # this slot.  `CDvis` is identically 0.0 on every alpha and `CDind` identically `CD`,
    # because `flow5_run.cpp` calls `pPlPolar->setViscous(false)`: the deck asks for no
    # viscous model, so flow5 reports ZERO profile drag and the whole of its CD is
    # induced.  Neither channel is informative about parasite drag, so `cx[0]` is NOT
    # taken from them; the source file's own DATCOM-computed WG.CD0 is used, exactly as
    # tornado.jsonc does, so all four files agree on it.  All three numbers are recorded.
    cd_at_zero = float(np.asarray(polar["CD"]).reshape(-1)[zero])
    cd_vis_zero = float(np.asarray(polar["CDvis"]).reshape(-1)[zero])
    cd_ind_zero = float(np.asarray(polar["CDind"]).reshape(-1)[zero])
    parasite = float(ac.WG["CD0"])
    collector.put(
        "cd0", -parasite, tornado_key="cd0 <- source WG.CD0 (NOT a flow5 output)",
        solver_value=None,
        conversion="parasite drag as a positive magnitude, then cx = -(parasite); NOT a "
                   "flow5 output",
        role="intercept",
    )
    # --- lateral static and the rate derivatives flow5 does have ----------------
    for field, key, note_text in (
        ("cl_beta", "Clb", "roll due to sideslip"),
        ("cy_beta", "CYb", "side force due to sideslip"),
        ("cn_beta", "Cnb", "yaw due to sideslip"),
        ("cy_p", "CYp", "side force, roll rate"),
        ("cy_r", "CYr", "side force, yaw rate"),
        ("cl_p", "Clp", "roll due to roll rate"),
        ("cl_r", "Clr", "roll due to yaw rate"),
        ("cn_p", "Cnp", "yaw due to roll rate"),
        ("cn_r", "Cnr", "yaw due to yaw rate"),
    ):
        collector.put(
            field, float(polar[key]),
            tornado_key=f"flow5 {key} (StabDerivative, already stability-axis)",
            solver_value=float(polar[key]),
            conversion=f"read directly: {note_text}; {key} is one of the twelve "
                       "StabDerivatives and takes NO sign-map entry",
            role="slope",
        )
    # --- controls --------------------------------------------------------------
    for field, surface, key, negate_lift in (
        ("cl_da", "aileron", "Cl", False), ("cn_da", "aileron", "Cn", False),
        ("cl_de", "elevator", "CL", True), ("cm_de", "elevator", "Cm", False),
        ("cl_dr", "rudder", "Cl", False), ("cy_dr", "rudder", "CY", False),
        ("cn_dr", "rudder", "Cn", False),
    ):
        per_degree = by_surface[surface].get(key)
        if per_degree is None:
            collector.absent(
                field, f"aid.flow5_controls[{surface}].{key}",
                reason="flow5's control row does not carry this coefficient",
            )
            continue
        magnitude = per_degree_to_per_radian(float(per_degree))
        collector.put(
            field, -magnitude if negate_lift else magnitude,
            tornado_key=f"aid.flow5_controls[{surface}].{key}",
            solver_value=float(per_degree),
            conversion=(
                f"per degree x {DEG_TO_RAD}: aid.flow5_controls central-differences two decks"
                f" at delta +- aid.control_deriv.H_DEG = +-1 DEGREE.  The row is already "
                f"Forward-Right-Down because to_frd is injected as the runner, so it is "
                f"read directly"
                + (
                    ".  The one negation that survives is the plane/dynamics.py identity "
                    "cz = -CL, the same one that gives cz[1] its sign"
                    if negate_lift else "; no identity and no frame conversion apply"
                )
            ),
            role="slope",
        )
    for field, quantity in (
        ("cl_q", "pitch-rate derivative of roll"),
        ("cm_q", "pitch-rate derivative of Cm"),
        ("cd_q", "pitch-rate derivative of the axial force"),
        ("cy_da", "aileron side force"),
    ):
        collector.absent(
            field, "flow5 polar / aid.flow5_controls",
            reason=(
                f"{quantity}: flow5's library DOES compute CXq / CZq / Cmq, but "
                "FLOW5/run/flow5_run.cpp's stab_derivative_fields() does not forward them, so "
                "they never reach the JSON.  No q-derivative of any kind is available"
                if field.endswith("_q") else
                "aileron side force: aid/flow5_controls' _KEEP table asks the aileron for "
                "Cl and Cn only, and does not ask for CY at all, so no aileron side-force "
                "coefficient is computed"
            ),
        )

    derivatives = collector.derivative_set()
    coefficients, _missing_slots = to_morelli(derivatives)
    aircraft, initial, controls, trim_block = _trim(coefficients, geometry, "flow5")
    margin = coefficients["cm"][1] / coefficients["cz"][1]
    if not 0.05 <= margin <= 0.45:
        raise ValueError(f"static margin {margin:.4f} cbar is outside 0.05...0.45")
    produced = sorted(f for f in ALL_DERIVATIVE_FIELDS if getattr(derivatives, f) is not None)
    absent = sorted(f for f in ALL_DERIVATIVE_FIELDS if getattr(derivatives, f) is None)
    provenance = _cross_check_provenance(
        "flow5",
        solver_version="flow5 via aid.flow5_io.run_flow5 (to_frd) and "
                       "aid.flow5_controls.flow5_controls with to_frd injected as the runner",
        geometry=geometry,
        collector=collector,
        source=source,
        shared=shared,
        moment_reference=moment_reference,
        flight_condition=_generic_flight_condition(
            geometry, trim_block, source_speed_mps=geometry.v_true_mps),
        trim_block=trim_block,
        extra={
            "axes": "flow5 after to_frd: its per-point polar channels Cx/Cz/Cl/Cn take a -1 "
                    "and its twelve StabDerivatives take none, because they are already "
                    "stability-axis -- so the adapter contains no frame mapping at all and no "
                    "moment shift",
            "numpy_bridge": numpy_note,
            "control_rows": {surface: dict(by_surface[surface]) for surface in by_surface},
            "coverage": {
                "produced": produced,
                "absent": absent,
                "why": "flow5 emits 26 keys: the six longitudinal polar channels, the twelve "
                       "StabDerivatives, CLa/Cma, a CDvis/CDind split and XNP.  The four absent "
                       "slots are the three q-derivatives the runner does not forward and the "
                       "aileron side force its control helper does not ask for",
            },
            "beta_flat": {
                "keys": sorted(FLOW5_BETA_FLAT),
                "note": "flow5's StabDerivatives are frozen at beta = 0: "
                        "panelanalysis.cpp's computeStabilityDerivatives and "
                        "computeAngularDerivatives both hardcode `double beta(0.0)`, while "
                        "CLa and Cma are polar OLS slopes that DO follow beta.  aid's "
                        "FLOW5_BETA_FLAT names the frozen set and "
                        "flow5_emitted_stab_derivatives() checks it against the C++ field list, "
                        "so a 13th derivative would show up as a gap rather than silently "
                        "claiming to have flown a sideslip",
                "harmless_here": "we fly beta = 0, so no channel in this file is being read at "
                                 "a condition it did not fly at",
            },
            "mesh": {
                "value": list(FLOW5_MESH),
                "why_pinned": "flow5's Clb is MESH-DEPENDENT and the sibling records it: "
                              "-0.0736 at ('5','3'), +0.0018 at ('10','5'), -0.0530 at "
                              "('10','10'), -0.0593 at ('20','10'), against AVL's -0.059 and "
                              "Tornado's -0.054.  At ('10','5') it goes POSITIVE and |Clb| is "
                              "~30x too small, which reads as a sign flip rather than a "
                              "convergence failure.  ('10','10') is the sibling's own default "
                              "and the only one of the four that agrees with AVL, so it is "
                              "pinned by test_16_flow5_mesh_is_pinned because the hazard is a "
                              "silent sign inversion, not a precision loss",
            },
            "drag_split": {
                "CD_at_alpha0": cd_at_zero,
                "CDvis_at_alpha0": cd_vis_zero,
                "CDind_at_alpha0": cd_ind_zero,
                "note": "NOT informative about parasite drag: flow5_run.cpp calls "
                        "setViscous(false), so CDvis is identically 0.0 on every alpha and "
                        "CDind is identically CD -- the whole of flow5's drag is induced.  This "
                        "is why cx[0] is NOT taken from either channel",
            },
            "cd0": {
                "source": "the source analysis file's own DATCOM-computed WG.CD0, the same "
                          "value tornado.jsonc, datcom.jsonc and avl.jsonc all use",
                "value": parasite,
                "source_WG_CD0": parasite,
                "note": "cx[0] is a POSITIVE parasite-drag magnitude in Morelli's sign "
                        "convention (the collector applies the cx = -CD0 identity), and it is "
                        "NOT a flow5 output.  flow5's own CDvis/CDind split cannot supply "
                        "it; see provenance.drag_split.  Recorded here because "
                        "cross_check_header's item 4 reads provenance['cd0']['value'] and "
                        "['source'] for every model, and flow5's absence of it raised "
                        "KeyError during the file write",
            },
            "not_a_field": {
                "cd_de": "flow5 DOES produce an elevator drag derivative -- "
                         "aid.flow5_controls keeps ('CL', 'CD', 'Cm') for the elevator -- but "
                         "this schema has no cd_de slot in any of the 19 arrays, so it maps to "
                         "nothing.  It is NOT carried in this file; see "
                         "provenance.raw_output_location for where the build put it",
            },
            "cross_solver_disagreements": _cross_solver_disagreements(
                collector.mapping, collector.flipped,
            ),
            "invariant_band_note": (
                f"cm[1] = {coefficients['cm'][1]:.6f}/rad is 4.9 % BELOW the plan's "
                "pitched-moment slope band of -1.5 ... -0.3 /rad, which that band was "
                "estimated from (tornado.jsonc ships -1.1927).  The sign and the static "
                "margin are both correct and the band is the outlier here, not the "
                "coefficient; it is recorded rather than forced, and Task 5's suite is where "
                "magnitudes are asserted"
            ),
        },
    )
    return CrossCheckRun(
        model="flow5", derivatives=derivatives, coefficients=coefficients,
        raw=raw, geometry=geometry, provenance=provenance, aircraft=aircraft,
        initial=initial, controls=controls, trim=trim_block,
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


def geometry_block(
    run: TornadoRun, cross_runs: Sequence[CrossCheckRun] = ()
) -> dict[str, Any]:
    """The ``geometry.jsonc`` payload: SI geometry plus every raw solver number.

    ``cross_runs`` are the DATCOM / AVL / flow5 runs, and their ``raw`` blocks land
    here under ``raw_cross_checks``.  That is the plan's requirement that this file
    "carries the SI reference geometry and the unconverted per-degree solver
    output": the Fortran's own ``for006`` table, AID's pre-correction handbook rows,
    AVL's per-alpha ``.st`` rows and flow5's deck and polar are otherwise not
    recoverable from the committed tree at all, because a model file's
    ``provenance.mapping`` carries only the one raw number behind each slot.

    With ``cross_runs`` empty -- which is what the Tornado-only ``build()`` does --
    the key is absent rather than empty, so nothing can claim a block is here when
    it is not.
    """
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
            for key in ("solver", "mesh", "source", "aid_src", "aid_src_from_env",
                        "aid_src_commit",
                        "moment_reference",
                        "sign_convention", "cl_alpha_cross_check", "cd0", "flipped",
                        "not_normalised", "missing")
        },
        **({"raw_cross_checks": {cross.model: cross.raw for cross in cross_runs}}
           if cross_runs else {}),
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
    """The flipped slots that are rate or sideslip columns, for the header to name.

    Derived from the run's own ``provenance.flipped`` rather than written out, so
    the header cannot name a slot the invariant table did not flip or omit one it
    did.  The control-derivative fields are excluded by their ``_da`` / ``_dr`` /
    ``_de`` suffixes: they are per radian of control deflection.

    Since the sibling began returning Forward-Right-Down this set is normally EMPTY
    -- the whole class of "the P and R columns come back component-sense inverted"
    defects it was built to report no longer exists -- so an empty result is the
    expected answer and the header says so rather than listing something.

    The header now prints the FULL flipped list beside this subset and labels the
    subset as a subset.  It used to print only this subset under the words "this
    run's own flipped list", which answered a seven-element question with two
    elements.
    """
    return [name for name in flipped if not name.endswith(("_da", "_dr", "_de"))]


def _q_convention_lines(run: TornadoRun | CrossCheckRun) -> list[str]:
    """Item 6's pitch-rate convention paragraph, derived from the run's own data.

    This paragraph used to assert, in hand-written words, that ``czq[0]`` holds
    ``CL_q`` and ``cxq[0]`` holds ``CD_q`` "in the STANDARD-AERO convention rather than
    the body-axis one".  After the re-base that was false -- the file ships the
    body-axis ``CZ_Q`` -- and the SAME file contradicted it 35 lines earlier and in
    ``provenance.cd_q_convention``.  Every number here now comes out of that
    provenance block, so the sentence cannot outlive the computation.
    """
    convention = run.provenance.get("cd_q_convention")
    if not convention:
        return [
            "//    No cd_q_convention block: this run has no separate wind-axis/body-axis",
            "//    distinction to record for the rate slots.",
        ]
    lines = [
        "//    THE PITCH-RATE SLOT IS BODY-AXIS, not standard-aero.  Written out from",
        "//    provenance.cd_q_convention so it cannot drift from what shipped:",
        f"//      shipped:      {convention['shipped']}",
        f"//      why:          {convention['why']}",
        f"//      czq[0] writes {float(convention['body_axis_cz_q_shipped']):.6f}"
        f" (CZ_Q, body axis)",
    ]
    if "wind_axis_cl_q" in convention:
        lines += [
            f"//      the WIND-axis CL_Q is {float(convention['wind_axis_cl_q']):.6f}, i.e."
            f" {float(convention['wind_axis_cl_q_slot_if_standard_aero']):.6f}"
            " in this slot if standard-aero",
            f"//      convention were used -- it is NOT what this file writes.  The two"
            f" differ by {abs(float(convention['difference_percent'])):.3f} per cent,",
            "//      because CL and CZ part company wherever the axial force is non-zero.",
            "//      Any sentence in this header quoting a wind-axis CL_Q number means the",
            "//      WIND-axis one above, not the value in the arrays.",
        ]
    return lines


def _cross_solver_disagreements(
    mapping: list[dict[str, Any]], flipped: list[str],
) -> dict[str, str]:
    """One entry per slot where this solver's own sign fights the invariant table.

    DERIVED from ``provenance.mapping`` and ``provenance.flipped``, never written out.
    This block used to name two slots by hand -- ``cn_p`` and a mislabelled ``cz_5`` --
    and both were wrong in the same way: ``cn_p`` was the only one of four such
    disagreements that got an entry, and ``cz_5`` claimed "the flip recorded in
    provenance.flipped is that identity plus the table" when ``cl_de`` is state
    `normalised` and absent from ``flipped``, because the ``cz = -CL`` identity
    supplies that sign before the table ever runs.

    So the rule is mechanical: a slot belongs here exactly when it is in ``flipped``,
    which by construction means ``normalise_sign`` reversed it, which by construction
    means the solver's own sign was on the wrong side of the invariant.  Anything the
    sign-convention identities handled is NOT here, and says so.
    """
    from aero_convert.units import SIGN_INVARIANTS

    by_field = {entry["field"]: entry for entry in mapping}
    out: dict[str, str] = {}
    for field in sorted(flipped):
        invariant = SIGN_INVARIANTS.get(field)
        entry = by_field.get(field, {})
        solver_value = entry.get("solver_value")
        if invariant is None:
            out[field] = (
                f"{field} was flipped but SIGN_INVARIANTS has no row for it, which should "
                "be impossible; recorded rather than dropped"
            )
            continue
        sign = "negative" if invariant < 0 else "positive"
        if solver_value is None:
            out[field] = (
                f"the invariant table wants {field} {sign}, and this slot was flipped to "
                "match, but its mapping carries no solver value to compare against"
            )
            continue
        out[field] = (
            f"{field}: this solver's own {entry.get('tornado_key')} is "
            f"{float(solver_value):+.6f}, where morelli.json wants {field} {sign} "
            f"(SIGN_INVARIANTS[{field!r}] = {invariant:+d}).  The sign is established -- "
            "roll, pitch and yaw damping each have one sign, and "
            "data/planes/linear/morelli.json is the named authority -- so normalise_sign "
            "flips it mechanically and provenance.flipped records it.  Recorded here as a "
            "CROSS-SOLVER DISAGREEMENT rather than routine normalisation, because the "
            "other solvers sit on the invariant side of zero"
        )
    # And the near-miss worth stating explicitly, because it looks like one and is not.
    identity_only = [
        field for field in ("cl_de", "cl_alpha", "cl0", "cd0")
        if field in by_field and field not in flipped
    ]
    if identity_only:
        out["_not_a_disagreement"] = (
            "For the record, " + ", ".join(sorted(identity_only))
            + " are NOT in this list: the sign-convention identities (`cz = -CL`, "
            "`cx = -CD`) already put them on the invariant side, so normalise_sign leaves "
            "them alone and provenance.flipped does not name them.  An earlier version of "
            "this block listed cz[5] here anyway and described a flip that was never "
            "recorded."
        )
    return out


def identity_negated(run: TornadoRun | CrossCheckRun) -> list[dict[str, Any]]:
    """Every slot whose ONLY transformation is the ``cz = -CL`` sign identity.

    Derived from ``provenance.mapping``, never written out.  A field qualifies when
    its mapped value is exactly minus its solver value, optionally scaled by
    ``DEG_TO_RAD`` for a per-degree control row, AND it is neither CG-shifted nor
    present in ``provenance.flipped``.  Anything else in the file went through the
    moment shift or the invariant table, which is a different thing with a different
    justification.

    This exists because the header used to say "TWO negations survive ... cz[1] and
    cz[5]" while the same file's item 5 already admitted ``cz[0]`` carries it too,
    so one file contradicted itself.  The count is now whatever the data says: three
    for Tornado, one for flow5, zero for the two solvers whose ``.st``/``.sb`` are
    already body-axis.
    """
    from aero_convert.morelli import SLOT_MAP

    field_slot = {
        name: (array, index)
        for array, slots in SLOT_MAP.items()
        for index, name in slots
    }
    flipped = set(run.provenance["flipped"])
    found: list[dict[str, Any]] = []
    for entry in run.provenance["mapping"]:
        field = entry["field"]
        solver_value = entry.get("solver_value")
        mapped_value = entry.get("mapped_value")
        if solver_value is None or mapped_value is None or field in flipped:
            continue
        if solver_value == 0.0:
            continue
        ratio = mapped_value / -solver_value
        per_degree = abs(ratio - DEG_TO_RAD) < 1e-9
        if abs(ratio - 1.0) < 1e-12 or per_degree:
            array, index = field_slot[field]
            found.append({
                "field": field,
                "slot": f"{array}[{index}]",
                "solver_key": entry.get("tornado_key"),
                "solver_value": float(solver_value),
                "mapped_value": float(mapped_value),
                "per_degree": per_degree,
            })
    return found


def identity_lines(run: TornadoRun | CrossCheckRun) -> list[str]:
    """Header lines naming the ``cz = -CL`` survivors, with their real numbers."""
    found = identity_negated(run)
    if not found:
        return [
            "//    NO negation survives here, and that is the point: this solver's channels",
            "//    are already body-axis, so every slot is read exactly as reported.",
        ]
    lines = [
        f"//    {len(found)} negation"
        + ("" if len(found) == 1 else "s")
        + (" survives" if len(found) == 1 else " survive")
        + ", and none is a frame conversion.  Each is a SIGN-CONVENTION identity"
        " that the schema",
        "//    itself requires -- `cz = -CL` for the lift slots, its body +z axis pointing"
        " down, and",
        "//    `cx = -CD` for the drag slot -- not a change of axis.  Listed from"
        " provenance.mapping:",
    ]
    for item in found:
        if item["per_degree"]:
            per_deg = item["solver_value"]
            lines.append(
                f"//      {item['slot']} from {item['solver_key']} = {per_deg:.6f} per degree"
                f" -> {item['mapped_value']:.6f} per radian"
            )
        else:
            lines.append(
                f"//      {item['slot']} from {item['solver_key']}"
                f" = {item['solver_value']:.6f} -> {item['mapped_value']:.6f} per radian"
            )
    return lines


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


def header_comment(
    run: TornadoRun, kind: str, cross_runs: Sequence[CrossCheckRun] = ()
) -> str:
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
        f"//    Solver: Tornado, via the sibling package aid.tornado (imported from"
        f" {run.provenance['aid_src']},",
        "//    a path relative to THIS repository's root so the bytes do not depend on which",
        "//    mount point the repo is reached through; see provenance.aid_src.",
        f"//    Sibling commit: {run.provenance.get('aid_src_commit') or 'not a git checkout'}."
        "  It is recorded",
        "//    because an upstream commit has broken this converter twice, silently both"
        " times -- a rebuild",
        "//    that moves it should be a one-line diff in this field and nothing else.",
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
        f"//    reference point is {geometry.x_cg_ft:g} ft AHEAD of the CG in the x-FORWARD"
        " sense the sibling returns.",
        f"//    The offset handed to shift_moments_to_cg is dx = -ft({geometry.x_cg_ft:g}) ="
        f" {geometry.dx_m} m (NOT +), with dy = dz = 0.",
        "//    The shift's cz is the BODY-Z force coefficient, and coeff_create's CZ IS that",
        "//    coefficient, already in plane/dynamics.py's frame, so it is used as it stands:",
        "//    slopes shift with slopes (Cm_a with CZ_a, Cm_Q with CZ_Q, Cm_de with CZ_de",
        "//    reconstructed exactly from the control row's wind-axis CL and CD) and the Cm0",
        "//    intercept shifts with the body-z intercept CZ(0), which at alpha = 0 is the same",
        "//    number as the -CL(0) that gives cz[0] its sign.  CL_a and CZ_a differ by 0.34 %"
        " here because",
        "//    NP[0] is in the lattice.  The yaw slopes shift with the BODY-Y side force (Cn_beta",
        "//    with CY_b, Cn_p with CY_P, Cn_r with CY_R).  No drag coefficient enters any"
        " shift: cx reaches",
        "//    the pitch line only through dz and the yaw line only through dy, and both arms"
        " are zero",
        "//    for this aircraft.",
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
        "//    NO FRAME CONVERSION IS APPLIED IN THIS FILE, and that is the new normal.",
        "//    aid/axes.py is the sibling's single home for solver signs and",
        "//    aid/tornado/coeff.py's last statement is `return to_frd(\"tornado\", out)`, so",
        "//    everything read here is ALREADY x forward, y right, z DOWN with positive Cl",
        "//    rolling right, positive Cm nose up and positive Cn yawing right --",
        "//    plane/dynamics.py's own frame and senses.  The control rows are already",
        "//    Forward-Right-Down too, and aid/tornado/control_deriv.py says on purpose that it",
        "//    does not re-wrap them.  The earlier mapping `cz = -CL; cl = -Cl; cm = +Cm;",
        "//    cn = -Cn; cy = +CY` existed only to convert Tornado's former aft-positive x and",
        "//    up-positive z, and it is GONE: cl, cm, cn, cy and cx are read exactly as reported,",
        "//    and the P and R columns need no flip because they no longer arrive inverted.",
        "//",
        *identity_lines(run),
        "//",
        "//    Everything else is unnegated, including czq[0] = CZ_Q and cxq[0] = CX_Q, which are",
        "//    the body-axis coefficients the schema actually adds to cz and cx.",
        "//",
        "//    Signs are normalised mechanically against aero_convert.units.SIGN_INVARIANTS and",
        "//    every flip is logged in provenance.flipped.  Computed from this run's own flipped",
        "//    list rather than written out, it is ALL of them:",
        f"//      {run.provenance['flipped'] or '(none)'}",
        "//    of which the rate and sideslip columns -- the ones this header used to print on",
        "//    their own, under words that claimed they were the whole list -- are:",
        f"//      {_flipped_rate_slots(run.provenance['flipped']) or '(none)'}",
        "//    Those are convention differences, not frame errors: the invariant table is the",
        "//    final authority, and each is a slot whose SIGN the two in-tree reference files",
        "//    disagree on.  cm0 carries no invariant and is recorded as not-normalised.",
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
        *_q_convention_lines(run),
        "//",
        "// 7. PER-SLOT PROVENANCE (all 19 arrays; real solver output vs zeroed)",
    ]
    tail = [
        "//",
        "// 8. HOW TO REGENERATE THIS FILE",
        f"//    cd python && {BUILD_COMMAND}",
        "//    Nothing else writes these files; the sibling analysis and the sibling aid tree",
        "//    are read-only inputs and all solver scratch goes to a temporary directory.",
        *_raw_output_lines(cross_runs),
        "// " + "=" * 74,
    ]
    return "\n".join(common + _slot_lines(run) + tail)


def cross_check_header(
    run: CrossCheckRun, kind: str, raw_output_location: str | None = None
) -> str:
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
            "//      (2) aid/lateral.py:570 ends its tail lift-slope formula with"
            " `vt_a * pi/180`,",
            "//          which is the wrong conversion in the wrong direction.  The formula's",
            "//          `k` comes from aid/lateral.py:565, which reads HT[\'a0\'] -- NOT",
            "//          VT.a0, which appears nowhere in the sibling tree -- and",
            f"//          apply_handbook rewrites that to"
            f" {float(run.provenance['handbook_cross_check']['lateral_corrected']['ht_a0_per_radian_as_read']):.4f}"
            " PER RADIAN, so",
            f"//          k = a0/(2*pi) ="
            f" {float(run.provenance['handbook_cross_check']['lateral_corrected']['helmbold_k_per_radian']):.4f}"
            "/rad and the formula answers",
            f"//          {float(run.provenance['handbook_cross_check']['lateral_corrected']['vt_a_per_rad']):.4f}"
            " PER RADIAN -- 51 per cent of the 2-D",
            "//          6.8396/rad, the reduction a fin of AR ~ 1.8 must show, where a",
            "//          per-degree reading would imply about 200/rad.  The trailing pi/180",
            "//          then divides a per-radian number by 180 and stores 0.0610931, so",
            "//          every quantity linear in that slope is 57.3x too small and is"
            " corrected: CYb, Cnb,",
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
        echoed = run.provenance["moment_reference"]["fortran_echo_synths_xcg"]
        written_card = run.provenance["moment_reference"]["for005_synths_xcg"]
        item5 = [
            "//    NO MOMENT SHIFT WAS APPLIED, and that is VERIFIED, not assumed: the station",
            "//    is read back out of what the Fortran itself wrote, not out of the same",
            "//    in-memory value it was given.",
            f"//    aid/datcom_io.py's write_synths puts XCG={{aero['XCG']}} on the $SYNTHS"
            f" namelist, so for005.dat",
            f"//    carries $SYNTHS XCG = {written_card[0]:g} ft and the Fortran ECHOES that"
            " namelist back into",
            f"//    datcom.out, {len(echoed)} time(s), all {echoed[0]:g} ft.  That echo is the"
            " number compared against",
            f"//    AERO.XCG = {run.provenance['moment_reference']['aero_xcg_ft']:g} ft; a"
            " disagreement between the card",
            "//    and the echo, an echo that disagrees with itself, or a MISSING echo each"
            " raise, so this check",
            "//    cannot pass by comparing AERO.XCG with itself.  Both lists are in"
            " provenance.moment_reference.",
            "//    AID's handbook pass takes the same station for the VT arms",
            "//    (hp = VT.Z + VT.ymac - AERO.ZCG, lp = VT.X + VT.xmac + VT.cbar/4 - AERO.XCG).",
            "//    dx = dy = dz = 0, so every moment here is already about the CG and cm[1],",
            "//    cm[0], cm[2], cmq[0], cn[0], cnp[0], cnr[0] are used as they stand.  Contrast",
            "//    tornado.jsonc, whose ref_point is 2.94 ft AHEAD of the CG and is corrected"
            " by dx = -0.896112 m.",
        ]
        axes = [
            "//    DATCOM, plane/dynamics.py and every in-tree reference file share ONE axis",
            "//    convention: x forward, y right, z DOWN, positive Cm nose up, positive Cl",
            "//    roll right, positive Cn yaw right.  So there is no frame mapping in this",
            "//    file at all.  Which slots, if any, the `cz = -CL` identity still negates"
            " is derived:",
            *identity_lines(run),
            "//    The source planform runs x AFT-positive (XW = 2.2 < XCG = 2.94 < XH = 8.75"
            " ft), which is why",
            "//    tornado.jsonc needs a shift and this file does not.",
        ]
        absent = len(run.provenance["missing"])
        plural = "SLOT IS" if absent == 1 else "SLOTS ARE"
        gaps = [
            f"//    {absent} {plural} GENUINELY ABSENT -- zeroed and declared, with the"
            " reason generated from",
            "//    provenance.missing so this list cannot go stale against the slot table"
            " below:",
            *_gap_lines(run),
            "//    Every other slot is real output.  CD_alpha is not representable in this",
            "//    schema (cx has no CD*alpha term) and is 0.0 and declared.",
        ]
    elif model == "avl":
        item1 = [
            "//    Solver: AVL, via aid.avl_io.run_avl_full for the stability sweep and",
            "//    aid.avl_controls.avl_controls for the control derivatives, both from the",
            f"//    sibling package imported from {run.provenance['aid_src']}, a path relative to"
            " this",
            "//    repository's root so the bytes do not depend on which mount point the repo is"
            " reached",
            "//    through; see provenance.aid_src.",
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
            "//    applied in this file.  Which slots, if any, the `cz = -CL` identity still"
            " negates is derived:",
            *identity_lines(run),
            "//    CLp is NOT negated"
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
            "//    declared.",
            *_gap_lines(run),
        ]
    else:
        item1 = [
            "//    Solver: flow5, via aid.flow5_io.write_flow5_deck / run_flow5(ac, mesh) and",
            f"//    aid.flow5_controls.flow5_controls, from the sibling package imported from"
            f" {run.provenance['aid_src']},",
            "//    a path relative to this repository's root so the bytes do not depend on which",
            "//    mount point the repo is reached through; see provenance.aid_src.",
        f"//    Sibling commit: {run.provenance.get('aid_src_commit') or 'not a git checkout'}."
        "  It is recorded",
        "//    because an upstream commit has broken this converter twice, silently both"
        " times -- a rebuild",
        "//    that moves it should be a one-line diff in this field and nothing else.",
            f"//    Mesh: {FLOW5_MESH[0]} spanwise x {FLOW5_MESH[1]} chordwise, read by"
            " write_flow5_deck as (ny, nx).  The deck",
            "//    carries WG, HT, VT and NP[0] (\"Wing 2\"), so NP[0] IS in this file even"
            " though it is",
            "//    not in avl.jsonc (AVL keeps at most three surfaces).",
            f"//    The runner emits {len(run.raw['output_keys'])} keys and this file fills"
            f" {len(run.provenance['coverage']['produced'])} of the 25 slots from them:",
            f"//      {', '.join(run.raw['output_keys'])}",
            "//    The frame is settled in ONE call, aid.flow5_io.run_flow5, which is",
            "//    to_frd('flow5', run_flow5_native(...)).  Hand-applying the sign map would be",
            "//    wrong in a way that is worth stating: flow5's per-point polar channels",
            "//    Cx/Cz/Cl/Cn ARE in its sign map with a -1 (raw Cz tracks raw CL, i.e. it is",
            "//    up-positive and is NOT a body-z coefficient), while its twelve",
            "//    StabDerivatives take NO entry because computeStabilityDerivatives projects",
            "//    them onto the stability axes.  Measured: CZa = -5.256228 while CLa = +5.169735,",
            "//    so CZa already carries this slot's sign.  Negating the derivative scalars would",
            "//    invert the lift slope.",
            "//    aid/flow5_controls' DEFAULT runner is the RAW one, so to_frd is injected as",
            "//    the runner here; without it the aileron Cl and Cn arrive with the wrong sign.",
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
            f"//    That factor is {condition['speed_ratio_v_trim_over_v_source']:.2f} here"
            f" and {SPEED_FACTOR_FEET_FRAME:.2f} in tornado.jsonc and",
            "//    avl.jsonc for one reason only: flow5's deck is SI, so its"
            f" {geometry.v_true_mps:.4f} really is {geometry.v_true_mps:.4f} m/s,",
            f"//    while the other two feed {SOURCE_SPEED_AS_REPORTED:.4f} into a FEET frame,"
            f" where it means {geometry.as_mps:.4f} m/s.",
            "//    The rate factor is 1 for three of the four rate slots this file does have,",
            "//    because flow5's StabDerivatives are per unit of p-hat / r-hat taken at the",
            "//    source speed and are NOT rescaled -- the same treatment as the other three",
            "//    files, for the same reason, and recorded the same way.",
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
            f"//    Source: {run.provenance['cd0']['source']}, and it is NOT a flow5 output."
            " flow5 now SPLITS",
            "//    its drag and the split is the measurement that settles this slot:",
            f"//      CD(0)    = {run.provenance['drag_split']['CD_at_alpha0']:.6g}",
            f"//      CDvis(0) = {run.provenance['drag_split']['CDvis_at_alpha0']:.6g}"
            "   <- identically 0.0 on EVERY alpha",
            f"//      CDind(0) = {run.provenance['drag_split']['CDind_at_alpha0']:.6g}"
            "   <- identically CD on every alpha",
            "//    because FLOW5/run/flow5_run.cpp calls `pPlPolar->setViscous(false)`: the deck",
            "//    asks for no viscous model, so flow5 reports ZERO profile drag and the whole"
            " of its CD",
            "//    is induced.  Neither channel is informative about parasite drag, so cx[0] is"
            " not taken from",
            "//    either.  The source file's own DATCOM-computed WG.CD0 is used instead,"
            " exactly as it already is",
            "//    for tornado.jsonc, so all four files agree on it.",
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
            "//    moment shift is applied.  Which slots that leaves negated is DERIVED from",
            "//    provenance.mapping rather than asserted -- this item used to claim",
            "//    `cz[1] = -CL_alpha`, which was true while cz[1] came from CLa and stopped",
            "//    being true when the source became the body-axis CZa, which needs no",
            "//    negation at all:",
            *identity_lines(run),
            f"//    Its Cma = {float(run.raw['Cma']):.6f} is already negative, the",
            "//    invariant sign, so it is written through unnegated.",
            "//    Every sign is decided mechanically by aero_convert.units.normalise_sign"
            " against SIGN_INVARIANTS.",
        ]
        produced = run.provenance["coverage"]["produced"]
        coverage = run.provenance["coverage"]
        gaps = [
            f"//    {len(run.provenance['missing'])} OF THE {len(ALL_DERIVATIVE_FIELDS)} SLOTS"
            " ARE ABSENT -- zeroed and declared",
            "//    in provenance.missing with a reason rather than guessed.  Both lists below are",
            "//    GENERATED from the run's own coverage block, so they cannot go stale against",
            "//    the slot table:",
            f"//      produced ({len(produced)}): {', '.join(produced)}",
            f"//      absent   ({len(coverage['absent'])}): {', '.join(coverage['absent'])}",
            *_gap_lines(run),
            "//    This is flow5's real capability, measured, not a shortfall of this adapter:",
            f"//    its runner emits {len(run.raw['output_keys'])} keys over the alpha sweep"
            f" ({', '.join(run.raw['output_keys'])}),",
            "//    and of those only CL, CD, Cm, CLa and Cma are longitudinal coefficients the",
            "//    schema has a slot for.  flow5 is a LONGITUDINAL cross-check on the lift",
            "//    slope, the pitching slope, the two intercepts, the elevator and the parasite",
            "//    drag -- it is not a 6-DOF data set and must not be flown as one.",
            "//    CD_alpha is not representable in this schema (cx has no CD*alpha term) and"
            " is 0.0.",
            "//    cd_de is not a FIELD at all: flow5 does produce an elevator drag derivative,"
            " but this",
            "//    schema has no slot for it in any of the 19 arrays, so it maps to nothing.  It"
            " is NOT",
            "//    carried in this file; the note at the end of item 7 says where the build put"
            " it.",
            f"//    One magnitude sits outside the plan's band and is RECORDED rather than forced:"
            f" cm[1] = {run.coefficients['cm'][1]:.6f}",
            "//    /rad is 4.9 % below the plan's -1.5 ... -0.3 pitched-moment-slope band, which"
            " that band was",
            "//    estimated from (tornado.jsonc ships -1.1927).  The sign and the static margin"
            " are both",
            "//    correct and the band is the outlier here, not the coefficient.  Task 5's",
            "//    invariant suite is where magnitudes are asserted, so this excursion is",
            "//    declared here rather than forced into the band.",
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
    if raw_output_location is None:
        where = [
            "//    THE REST OF THIS SOLVER'S OUTPUT IS NOT IN THE COMMITTED TREE.  This file was",
            "//    written on its own, without the geometry.jsonc raw block, so its for006 table /",
            "//    .st rows / polar and its pre-correction handbook rows exist only in the run",
            "//    that produced this file.  Re-run scripts/build_cessna172_aero.py, which writes",
            "//    all five files together, to get them into geometry.jsonc.",
        ]
    else:
        where = [
            f"//    THE REST OF THIS SOLVER'S RAW, UNCONVERTED OUTPUT IS IN {raw_output_location}",
            "//    -- the Fortran's own for006 table and AID's pre-correction handbook rows for",
            "//    DATCOM, the per-alpha .st rows for AVL, the deck and the polar for flow5.",
            "//    A model file's provenance.mapping carries only the one raw number behind each",
            "//    slot, so that block is what makes the rest re-derivable without re-running.",
        ]
    tail = [
        *where,
        "//",
        "// 8. HOW TO REGENERATE THIS FILE",
        f"//    cd python && {BUILD_COMMAND}",
        "//    Nothing else writes these files; the sibling analysis and the sibling aid tree",
        "//    are read-only inputs and all solver scratch goes to a temporary directory.",
        "// " + "=" * 74,
    ]
    return "\n".join(common + _slot_lines(run) + tail)


def _gap_lines(run: CrossCheckRun) -> list[str]:
    """One ``//`` line per declared gap, GENERATED from the run, never hand-written.

    The first version of this was prose that went stale: it said ``cxq[0]`` was
    "written 0.0" while the same file wrote 0.140889, because the sentence was a
    draft from before ``cd_q`` started being read out of AVL's ``.sb``.  Item 7 is
    the required "which slots are real solver output and which are zeroed", so it is
    now derived from ``provenance.missing`` and ``provenance.flipped`` -- the same
    trick ``header_comment`` already uses for Tornado's flipped-rate-slot list -- and
    it cannot disagree with the slot table printed just above it.

    The two consistency assertions ARE the point: a slot declared missing that is
    non-zero, or a field declared missing whose mapping carries a value, is a
    contradiction, and now it stops the build instead of shipping a file that says
    one thing and holds another.
    """
    entries = {entry["field"]: entry for entry in run.provenance["mapping"]}
    lines = []
    for item in sorted(run.provenance["missing"], key=lambda entry: entry["slot"]):
        slot, field = item["slot"], item["field"]
        array, index = slot.split("[")
        index = int(index.rstrip("]"))
        if run.coefficients[array][index] != 0.0:
            raise AssertionError(
                f"{run.model}: {slot} is in provenance.missing but writes "
                f"{run.coefficients[array][index]!r}, so this header would be claiming a "
                "declared gap that is not one"
            )
        if field in entries and entries[field]["value"] is not None:
            raise AssertionError(
                f"{run.model}: {field} is in provenance.missing but its mapping carries a "
                "value, so the header and the mapping disagree"
            )
        lines.append(
            f"//      {slot:<9s} {field:<7s} ZEROED AND DECLARED -- no value exists:"
            f" {item['reason']}"
        )
    if not lines:
        lines.append("//      (none: this solver filled every slot in the schema)")
    flipped = sorted(run.provenance["flipped"])
    if flipped:
        # This used to be appended with no lead-in, so it read as a FOURTH absent slot
        # inside the "N SLOTS ARE GENUINELY ABSENT" list -- and an absent slot and a
        # flipped slot are different states entirely.  It now opens its own labelled
        # paragraph, and names the count so it cannot disagree with the slot table.
        lines.append("")
        lines.append(
            f"//    SEPARATELY, {len(flipped)} SLOT(S) ARE FILLED BUT FLIPPED -- these are NOT"
            " absent:"
        )
        lines.append(
            "//    they carry a real solver value whose sign the invariant table reversed, so"
        )
        lines.append(
            "//    the solver's own sign sits on the other side of zero: " + ", ".join(flipped)
        )
        lines.append(
            "//    Every one is in the slot table above with state `flipped`."
        )
    return lines


def _raw_output_lines(cross_runs: Sequence[CrossCheckRun]) -> list[str]:
    """Item 9 of ``geometry.jsonc``: where every solver's raw output is.

    Written only when the block is actually present, so the file can never claim a
    block that the run which produced it did not write.  This is the plan's
    requirement that ``geometry.jsonc`` "carries the SI reference geometry and the
    unconverted per-degree solver output": ``raw_tornado`` is Tornado's and
    ``raw_cross_checks`` is the other three's.
    """
    if not cross_runs:
        return []
    lines = [
        "//",
        "// 9. THE OTHER THREE SOLVERS' RAW OUTPUT",
        "//    This file also carries `raw_cross_checks`, keyed by model, because a model"
        " file's",
        "//    provenance.mapping holds only the ONE raw number behind each slot.  Without"
        " this block",
        "//    the rest of each solver's output would not be re-derivable from the committed",
        "//    tree without re-running the solver:",
    ]
    for cross in cross_runs:
        lines.append(
            f"//      {cross.model:<7s} raw_cross_checks.{cross.model}: {len(cross.raw)} keys --"
            f" {', '.join(sorted(cross.raw))}"
        )
    lines += [
        "//    Every per-degree number in them is UNCONVERTED: the model files carry the",
        "//    x 57.29577951308232 step and the sign normalisation, those blocks do not.",
        "//    flow5 has none -- its deck and polar are already SI and per radian -- which is",
        "//    itself part of that solver's record.",
    ]
    return lines


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


def _cross_check_run(model: str, source: Path | None = None) -> CrossCheckRun:
    """Run one named cross-check solver."""
    try:
        runner = globals()[_CROSS_CHECK_RUNNERS[model]]
    except KeyError:
        raise ValueError(f"{model!r} is not a cross-check model; {sorted(_CROSS_CHECK_RUNNERS)}") from None
    return runner(source)


def cross_check_payload(
    run: CrossCheckRun, raw_output_location: str | None = None
) -> dict[str, Any]:
    """The ``<model>.jsonc`` payload: schema-identical to the in-tree Morelli file.

    ``raw_output_location`` is set by ``write_model`` and records where the rest of
    this solver's output went.  It is a parameter rather than prose inside the
    adapter because whether it is true depends on what the WRITER did, not on what
    the adapter produced: only ``build_all`` puts the cross-check raw blocks into
    ``geometry.jsonc``.  A model file written on its own therefore says so instead of
    pointing at a block that is not in the tree.
    """
    coefficients, _missing = to_morelli(run.derivatives)
    provenance = dict(run.provenance)
    if raw_output_location is not None:
        provenance["raw_output_location"] = raw_output_location
    return {
        "model": run.model,
        "aircraft": run.aircraft,
        "initial": run.initial,
        "controls": run.controls,
        "coefficients": coefficients,
        "provenance": provenance,
    }


def write_jsonc(path: Path, payload: dict[str, Any], comment: str) -> Path:
    """Write a JSONC file whose first lines are real ``//`` comments."""
    path.parent.mkdir(parents=True, exist_ok=True)
    body = json.dumps(payload, indent=2, sort_keys=False)
    path.write_text(f"{comment}\n{body}\n")
    return path


def write_model(
    run: CrossCheckRun, out_dir: Path | None = None, *, raw_output_location: str | None = None
) -> Path:
    """Write one cross-check model file and return its path.

    ``raw_output_location`` is forwarded to both the payload and the header comment,
    so the file's item 7 can only state where the rest of the solver's output is if
    the caller actually put it there.
    """
    directory = Path(out_dir) if out_dir is not None else DEFAULT_OUT_DIR
    return write_jsonc(
        directory / f"{run.model}.jsonc",
        cross_check_payload(run, raw_output_location),
        cross_check_header(run, f"data/planes/cessna172/{run.model}.jsonc", raw_output_location),
    )


def write_all(
    run: TornadoRun, out_dir: Path | None = None, cross_runs: Sequence[CrossCheckRun] = ()
) -> dict[str, str]:
    """Write ``tornado.jsonc`` and ``geometry.jsonc``; return their paths.

    ``cross_runs`` are folded into ``geometry.jsonc`` as ``raw_cross_checks``, which
    is what makes the plan's "the unconverted per-degree solver output" requirement
    true for the other three solvers and not only for Tornado.
    """
    directory = Path(out_dir) if out_dir is not None else DEFAULT_OUT_DIR
    written = {
        "tornado.jsonc": write_jsonc(
            directory / "tornado.jsonc",
            model_payload(run),
            header_comment(run, "data/planes/cessna172/tornado.jsonc"),
        ),
        "geometry.jsonc": write_jsonc(
            directory / "geometry.jsonc",
            geometry_block(run, cross_runs),
            header_comment(run, "data/planes/cessna172/geometry.jsonc", cross_runs),
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

    Every solver is run BEFORE anything is written, so ``geometry.jsonc`` can carry
    all four solvers' raw output and each model file can name where its own went.
    """
    tornado = tornado_run(source)
    cross_runs = [_cross_check_run(model, source) for model in CROSS_CHECK_MODELS]
    for cross in cross_runs:
        _verify_lengths(cross_check_payload(cross))
    written = write_all(tornado, out_dir, cross_runs)
    for cross in cross_runs:
        written[f"{cross.model}.jsonc"] = str(
            write_model(cross, out_dir, raw_output_location=RAW_OUTPUT_LOCATION.format(
                model=cross.model))
        )
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
    written = build_all(args.out_dir, args.source)
    for path in written.values():
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())