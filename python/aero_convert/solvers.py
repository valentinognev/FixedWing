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
negating it twice leaves the rigid-body term right.  **No extra sign flip
belongs on the rate derivatives.**  Measured on this aircraft, and all four of
these follow from the axis mapping alone:

=========================  ===================  ================================
Tornado reports            mapped to dynamics     required by the invariant table
=========================  ===================  ================================
``CL_a = +5.148013``       ``cz = -CL_a``        ``cl_alpha`` < 0
``CL_Q = +9.591131``       ``cz = -CL_Q``        ``cl_q`` < 0
``Cl_b = +0.053721``       ``cl = -Cl_b``        ``cl_beta`` < 0
``Cn_b = -0.246500``       ``cn = -Cn_b``        ``cn_beta`` > 0
``Cn_P = +0.068671``       ``cn = -Cn_P``        ``cn_p`` < 0
``Cl_P = -0.486095``       ``cl = -Cl_P``        ``cl_p`` < 0
``Cm_a = -8.786140``       ``cm = +Cm_a``        ``cm_alpha`` < 0
``Cm_Q = -33.348777``      ``cm = +Cm_Q``        ``cm_q`` < 0
=========================  ===================  ================================

Two of the three moment axes flip because a right-handed frame that reverses two
of its three axes reverses exactly those two moment senses; the pitch axis
(right in both) does not flip, which is what keeps ``Cm`` the one quantity whose
sign must survive untouched for the aircraft to be statically stable.  Every
slot is still handed to ``aero_convert.units.normalise_sign`` rather than being
signed here, so a disagreement is logged in ``provenance.flipped`` instead of
being applied by hand -- ten of the twenty-five slots are flipped by the table.

``cz = -CL`` and ``cl = -Cl`` are also the arguments ``shift_moments_to_cg``
needs, because that function takes the ``plane/dynamics.py`` body frame
(x forward, y right, z down) and ``cz`` is its body-z coefficient.  Tornado's
y axis is already the dynamics y axis, so the side force passes through as
``cy = +CY``.

Slopes shift, intercepts shift differently
------------------------------------------
``shift_moments_to_cg`` moves a *coefficient*, so what it needs beside it is
the force coefficient that goes with the same independent variable:

* a **slope** w.r.t. alpha (``Cm_a``) needs the lift **slope** ``cz = -CL_a``,
  because ``d/dalpha`` of the shift ``dx * CZ / c_ref`` is ``dx * CZ_alpha /
  c_ref``;
* an **intercept** (``Cm0``, from the alpha = 0 run) needs the lift
  **intercept** ``cz = -CL(0)``, because the constant force arm does not
  differentiate away;
* a **slope** w.r.t. a control deflection (``Cm_de``) needs the vertical-force
  slope w.r.t. that same deflection, ``cz = -CL_de``, for exactly the first
  reason;
* a **slope** w.r.t. a rate (``Cm_Q``) needs ``cz = -CL_Q``, same reason;
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
    "BUILD_COMMAND",
    "DEFAULT_AID_SRC",
    "DEFAULT_SOURCE",
    "SOURCE_MASS_KG",
    "TORNADO_MESH",
    "TornadoRun",
    "aid_src",
    "build",
    "default_source",
    "geometry_block",
    "model_payload",
    "tornado_derivatives",
    "tornado_run",
    "write_all",
]

# Mesh ("10", "5") is the AID batch default for Tornado (aid/control_report.py's
# _DEFAULT_MESH and aid/compare.py's _TORNADO_MESH).  NP[0] -- the "AR 33.3,
# S 3 ft^2" panel -- stays in the lattice: `tornado_io` adds it for us and the
# measured spanwise split at the trim alpha gives it 0.048336 of the aircraft's
# 0.502274 total CL, 9.6 %.
TORNADO_MESH = ("10", "5")

REPO_ROOT = Path(__file__).resolve().parents[2]
AID_SRC_ENV = "AID_SRC"
DEFAULT_AID_SRC = REPO_ROOT.parent / "USAF_DATCOM" / "AircraftIntuitiveDesign" / "Python" / "src"
DEFAULT_SOURCE = REPO_ROOT.parent / "USAF_DATCOM" / "AircraftIntuitiveDesign" / "Analyses" / "Cessna172.jsonc"
DEFAULT_OUT_DIR = REPO_ROOT / "data" / "planes" / "cessna172"
BUILD_COMMAND = "python3 scripts/build_cessna172_aero.py"

# The source's own mass: AERO.WT = 5 slugs.  5 slug * 32 ft/s^2 = 160.0 lbf, and
# 160.0 / 2.2046226 = 72.574 kg.  Using G_FT_S2 = 32.174 ft/s^2 instead would give
# 72.967 kg, 0.54 % higher; the plan's approved design fixes 72.574 and the head
# -of-file comment has to carry that figure, so it is used verbatim and both are
# recorded here.
SOURCE_MASS_KG = 72.574

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
    rho_kg_m3: float
    mesh: tuple[str, str]


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
    root = aid_src()
    if not root.is_dir():
        raise FileNotFoundError(
            f"the sibling aid package is not at {root}; set {AID_SRC_ENV} to its 'src' directory"
        )
    entry = str(root)
    if entry not in sys.path:
        sys.path.insert(0, entry)
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
        as_mps=float(state["AS"]),
        rho_kg_m3=float(state["rho"]),
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
        value: float,
        *,
        tornado_key: str,
        conversion: str,
        role: str,
    ) -> float | None:
        """Normalise one raw solver value, or record why it is absent.

        The finiteness guard lives here, at the solver boundary, because
        ``normalise_sign`` is not nan-safe (``nan > 0.0`` is False, so a nan
        would sail through the sign test and poison a coefficient array).
        """
        slot = FIELD_SLOT[field]
        raw = float(value)
        if not math.isfinite(raw):
            self.values[field] = None
            self.missing.append(
                {"field": field, "slot": slot, "tornado_key": tornado_key,
                 "reason": f"non-finite solver output {raw!r}"}
            )
            self.mapping.append(
                {"field": field, "slot": slot, "tornado_key": tornado_key,
                 "conversion": conversion, "role": role, "raw": None,
                 "value": None, "state": "missing"}
            )
            return None
        normalised, was_flipped = normalise_sign(field, raw)
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
             "conversion": conversion, "role": role, "raw": raw,
             "value": normalised, "state": state}
        )
        return normalised

    def derivative_set(self) -> DerivativeSet:
        return DerivativeSet(**self.values)


def _shift(
    geometry: Geometry, *, cz: float, cy: float, cm: float, cn: float
) -> tuple[float, float, float]:
    """Move one coefficient set from ``ref_point`` to the CG.

    ``cz`` is the body-z coefficient (z down, so ``cz = -CL``) and ``cy`` the
    body-y coefficient, both expressed **per the same independent variable as
    the coefficient being shifted** -- a slope for a slope, the intercept for an
    intercept.  ``cx``, ``cl_m`` are zero here because ``dy = dz = 0`` for this
    aircraft, so no force can reach the roll axis and the pitch/yaw lines see
    ``cx`` only through the ``dz`` and ``dy`` arms, which are zero; the returned
    ``Cl`` and ``Cn`` shifts are therefore identically zero, and the side force
    still has to be passed because the yaw line shifts on ``dx * cy`` alone.
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
    # Cl_a is a SLOPE: the vertical-force coefficient that goes with it is the
    # lift slope, cz = -CL_a, and the same slope is what Cm_a must be shifted
    # with.  Tornado's CL_a is already per radian (state["alpha"] is radians).
    cz_alpha = -cl_alpha
    collector.put("cl_alpha", cz_alpha, tornado_key="CL_a",
                  conversion="cz = -CL_a (body z down); already per radian", role="slope")

    # CL0 is an INTERCEPT: the alpha = 0 run gives it outright, no extrapolation.
    cl0 = -float(zero["coeffs"]["CL"])
    collector.put("cl0", cl0, tornado_key="CL(alpha=0)",
                  conversion="cz = -CL; second solve at alpha = 0 rad", role="intercept")

    # CL_q is the standard-aero CL_q the table asks for: Tornado's lift rate, with
    # the same body-z negation the rest of the vertical force gets.
    cz_q = -float(coeffs["CL_Q"])
    collector.put("cl_q", cz_q, tornado_key="CL_Q",
                  conversion="cz = -CL_Q; coeff_create already divided by fac = c_mac/(2V)",
                  role="slope")

    # --- side force: Tornado's y axis is the dynamics y axis -------------------
    cy_beta = float(coeffs["CY_b"])
    cy_p = float(coeffs["CY_P"])
    cy_r = float(coeffs["CY_R"])
    collector.put("cy_beta", cy_beta, tornado_key="CY_b",
                  conversion="cy = +CY_b (body y); CC_b is the wind-axis side force and "
                              "equals CY_b exactly at betha = 0", role="slope")
    collector.put("cy_p", cy_p, tornado_key="CY_P",
                  conversion="cy = +CY_P; already divided by fac", role="slope")
    collector.put("cy_r", cy_r, tornado_key="CY_R",
                  conversion="cy = +CY_R; already divided by fac", role="slope")

    # --- roll moments: Tornado's x axis is aft, so roll sense is negated ------
    collector.put("cl_beta", -float(coeffs["Cl_b"]), tornado_key="Cl_b",
                  conversion="cl = -Cl_b (Tornado x is aft-positive)", role="slope")
    collector.put("cl_p", -float(coeffs["Cl_P"]), tornado_key="Cl_P",
                  conversion="cl = -Cl_P; already divided by fac", role="slope")
    collector.put("cl_r", -float(coeffs["Cl_R"]), tornado_key="Cl_R",
                  conversion="cl = -Cl_R; already divided by fac", role="slope")

    # --- pitch: Tornado's pitch sense already agrees with dynamics ------------
    # Cm_a is a SLOPE in alpha, so it shifts with the lift SLOPE.
    _, cm_alpha, _ = _shift(geometry, cz=cz_alpha, cy=0.0, cm=float(coeffs["Cm_a"]), cn=0.0)
    collector.put("cm_alpha", cm_alpha, tornado_key="Cm_a",
                  conversion="cm = +Cm_a, then shifted with cz = -CL_a (the matching slope)",
                  role="slope")
    # Cm0 is an INTERCEPT, so it shifts with the lift INTERCEPT, not the slope.
    _, cm0, _ = _shift(geometry, cz=cl0, cy=0.0, cm=float(zero["coeffs"]["Cm"]), cn=0.0)
    collector.put("cm0", cm0, tornado_key="Cm(alpha=0)",
                  conversion="cm = +Cm, then shifted with cz = -CL(0) (the matching intercept); "
                             "cm0 carries no sign invariant and is never normalised",
                  role="intercept")
    # Cm_Q is a SLOPE in q, so it shifts with the lift slope with respect to q.
    _, cm_q, _ = _shift(geometry, cz=cz_q, cy=0.0, cm=float(coeffs["Cm_Q"]), cn=0.0)
    collector.put("cm_q", cm_q, tornado_key="Cm_Q",
                  conversion="cm = +Cm_Q, then shifted with cz = -CL_Q (the matching slope)",
                  role="slope")

    # --- yaw: negated (z up vs z down), shifted on dx * cy --------------------
    _, _, cn_beta = _shift(geometry, cz=0.0, cy=cy_beta, cm=0.0, cn=-float(coeffs["Cn_b"]))
    collector.put("cn_beta", cn_beta, tornado_key="Cn_b",
                  conversion="cn = -Cn_b (Tornado z is up-positive), then shifted with cy = CY_b",
                  role="slope")
    _, _, cn_p = _shift(geometry, cz=0.0, cy=cy_p, cm=0.0, cn=-float(coeffs["Cn_P"]))
    collector.put("cn_p", cn_p, tornado_key="Cn_P",
                  conversion="cn = -Cn_P, then shifted with cy = CY_P", role="slope")
    _, _, cn_r = _shift(geometry, cz=0.0, cy=cy_r, cm=0.0, cn=-float(coeffs["Cn_R"]))
    collector.put("cn_r", cn_r, tornado_key="Cn_R",
                  conversion="cn = -Cn_R, then shifted with cy = CY_R", role="slope")

    # --- drag ------------------------------------------------------------------
    # CD_q: Tornado's x axis is aft, so its axial force component is already
    # drag-positive; the table wants CD_q positive and this is that number.
    collector.put("cd_q", float(coeffs["CX_Q"]), tornado_key="CX_Q",
                  conversion="CX_Q read as CD_q (Tornado x aft => +CX is drag); "
                              "already divided by fac", role="slope")
    parasite = float(zero["coeffs"]["CD"]) - float(ac.WG["K"]) * cl0 * cl0
    cd0_source = "CD(0) - K*CL(0)^2"
    cd0_raw = -parasite
    if parasite <= 0.0:
        cd0_source = "source WG.CD0 (the VLM has no profile drag: CD(0) - K*CL(0)^2 = " \
                     f"{parasite:.6f})"
        cd0_raw = -float(ac.WG["CD0"])
    collector.put("cd0", cd0_raw, tornado_key="CD at alpha = 0",
                  conversion=f"parasite drag from {cd0_source}, then cx = -(parasite)",
                  role="intercept")

    # --- controls ---------------------------------------------------------------
    # `tornado_controls` returns central differences PER DEGREE; every one is
    # multiplied by DEG_TO_RAD before it goes near a Morelli array.  Its "CY"
    # entry is coeff_create's CC, the wind-axis side force; at betha = 0 the
    # wind-axis and body-axis side forces are identical, and the body axis is
    # the one the model wants.
    aileron, elevator, rudder = rows["aileron"], rows["elevator"], rows["rudder"]
    cl_da = -per_degree_to_per_radian(float(aileron["Cl"]))
    cy_da = per_degree_to_per_radian(float(aileron["CY"]))
    cn_da = -per_degree_to_per_radian(float(aileron["Cn"]))
    collector.put("cl_da", cl_da, tornado_key="tornado_controls[aileron].Cl",
                  conversion="per degree x 57.29577951308232, then cl = -Cl", role="slope")
    collector.put("cy_da", cy_da, tornado_key="tornado_controls[aileron].CY (= CC)",
                  conversion="per degree x 57.29577951308232, then cy = +CY", role="slope")
    collector.put("cn_da", cn_da, tornado_key="tornado_controls[aileron].Cn",
                  conversion="per degree x 57.29577951308232, then cn = -Cn", role="slope")
    cl_dr = -per_degree_to_per_radian(float(rudder["Cl"]))
    cy_dr = per_degree_to_per_radian(float(rudder["CY"]))
    cn_dr = -per_degree_to_per_radian(float(rudder["Cn"]))
    collector.put("cl_dr", cl_dr, tornado_key="tornado_controls[rudder].Cl",
                  conversion="per degree x 57.29577951308232, then cl = -Cl", role="slope")
    collector.put("cy_dr", cy_dr, tornado_key="tornado_controls[rudder].CY (= CC)",
                  conversion="per degree x 57.29577951308232, then cy = +CY", role="slope")
    collector.put("cn_dr", cn_dr, tornado_key="tornado_controls[rudder].Cn",
                  conversion="per degree x 57.29577951308232, then cn = -Cn", role="slope")
    # CL_de is a force slope in de; it is the same number that shifts Cm_de.
    cz_de = -per_degree_to_per_radian(float(elevator["CL"]))
    cm_de_o = per_degree_to_per_radian(float(elevator["Cm"]))
    _, cm_de, _ = _shift(geometry, cz=cz_de, cy=0.0, cm=cm_de_o, cn=0.0)
    collector.put("cl_de", cz_de, tornado_key="tornado_controls[elevator].CL",
                  conversion="per degree x 57.29577951308232, then cz = -CL", role="slope")
    collector.put("cm_de", cm_de, tornado_key="tornado_controls[elevator].Cm",
                  conversion="per degree x 57.29577951308232, cm = +Cm, then shifted with the "
                             "matching vertical-force slope cz = -CL_de", role="slope")

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
            "rho_kg_m3": geometry.rho_kg_m3,
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
            "source": "AERO.WT = 5 slugs",
            "mass_kg": SOURCE_MASS_KG,
            "weight_lbf_at_32_ft_s2": 5.0 * 32.0,
            "note": f"G_FT_S2 = {G_FT_S2} would give "
                    f"{5.0 * G_FT_S2 / 2.2046226:.3f} kg; the plan fixes {SOURCE_MASS_KG}",
        },
        "inertia": {
            rule: {
                "rule": label,
                "factor": factor,
                "k_m": k,
                "inertia_kg_m2": SOURCE_MASS_KG * k * k,
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


def tornado_derivatives(source: Path | None = None) -> DerivativeSet:
    """The normalised derivative set Tornado produces for the source geometry."""
    return tornado_run(source).derivatives


def _provisional_aircraft(geometry: Geometry, v_ref_mps: float) -> dict[str, float]:
    """Mass, inertia and a provisional thrust, before the trim solve runs.

    The inertia rules are the plan's: a radius of gyration per axis on THIS mass
    and THIS span, ``I = m k^2``, with the rule named per axis.  Published
    full-scale inertia was rejected because inertia scales as ``m l^2`` and the
    mass and length scales here are already mutually inconsistent.
    """
    mass_kg = SOURCE_MASS_KG
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
    mass_kg = SOURCE_MASS_KG
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


def _trim(coefficients: dict[str, list[float]], geometry: Geometry) -> tuple[dict[str, float], dict[str, float], dict[str, float], dict[str, float]]:
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
            "model": "tornado",
            "aircraft": aircraft_block,
            "initial": initial,
            "controls": controls,
            "coefficients": coefficients,
        }
        (plane_dir / "tornado.jsonc").write_text(json.dumps(provisional))
        probe = load_aircraft("cessna172", "tornado", root=Path(tmp))
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
        "weight_n": SOURCE_MASS_KG * SEA_LEVEL_G_MPS2,
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


def _slot_lines(run: TornadoRun) -> list[str]:
    """One line per coefficient slot: what it is and where it came from."""
    lines = []
    for entry in run.provenance["mapping"]:
        state = entry["state"]
        raw = "n/a" if entry["raw"] is None else f"{entry['raw']:.6g}"
        value = "0.0 (missing)" if entry["value"] is None else f"{entry['value']:.6g}"
        lines.append(
            f"//   {entry['slot']:<9s} {entry['field']:<10s} {state:<14s} "
            f"tornado {entry['tornado_key']} = {raw} -> {value}"
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
        f" V = {geometry.as_mps:.4f} m/s, rho = {geometry.rho_kg_m3:.6f} kg/m^3.",
        "//",
        "// 2. THE SOURCE PLANFORM IS A CESSNA 172 AT ROUGHLY 1/3 LINEAR SCALE",
        "//    span 12 ft against a real 36.1 ft, chord 2 ft against 4.89 ft, area 24 ft^2",
        "//    against 174 ft^2.  Wing loading is therefore 6.70 lbf/ft^2 where a real C172 is",
        "//    13.2.  Nothing in this file may be compared with a full-scale figure.",
        "//",
        "// 3. MASS",
        f"//    mass_kg = {SOURCE_MASS_KG} is the source file's own AERO.WT = 5 slugs"
        f" (5 x 32 ft/s^2 = 160.0 lbf,",
        f"//    160.0 / 2.2046226 = {SOURCE_MASS_KG} kg).  This is NOT a geometrically similar"
        " 1/3 scale C172,",
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
        f"//    cx[0] -- Cd0 in Tornado's own build-up -- = {run.provenance['cd0']['value']:.6g}.",
        f"//    Source of it: {run.provenance['cd0']['source']}.",
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
        "//    so every vertical-force argument is cz = -CL.  Slopes shift with slopes"
        " (Cm_a with -CL_a,",
        "//    Cm_Q with -CL_Q, Cm_de with -CL_de) and the Cm0 intercept shifts with the -CL(0)"
        " intercept;",
        "//    the yaw slopes shift with the side force (Cn_beta with CY_b, Cn_p with CY_P,"
        " Cn_r with CY_R).",
        "//",
        "// 6. SIGN CONVENTION",
        "//    Tornado's axes are x aft, y right, z up; plane/dynamics.py's are x forward,"
        " y right, z down.",
        "//    So cz = -CL, cl = -Cl, cm = +Cm, cn = -Cn.  The rate derivatives need NO extra"
        " flip: the",
        "//    code perturbs the relative wind with r x omega, which is the correct omega x r"
        " once the",
        "//    velocity is negated.  Signs are then normalised mechanically against"
        " aero_convert.units.SIGN_INVARIANTS",
        "//    (data/planes/linear/morelli.json is the convention authority) and every flip is"
        " logged in",
        "//    provenance.flipped.  cm0 carries no invariant and is recorded as"
        " not-normalised.",
        "//    data/planes/linear/morelli.json's cz[1] sign is treated as that file's defect:"
        " the load-bearing",
        "//    identity is cz = -CL, so a stable aircraft has cz[1] < 0 and cm[1] < 0.",
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


def write_jsonc(path: Path, payload: dict[str, Any], comment: str) -> Path:
    """Write a JSONC file whose first lines are real ``//`` comments."""
    path.parent.mkdir(parents=True, exist_ok=True)
    body = json.dumps(payload, indent=2, sort_keys=False)
    path.write_text(f"{comment}\n{body}\n")
    return path


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
    """Run the solver and write ``tornado.jsonc`` and ``geometry.jsonc``."""
    return write_all(tornado_run(source), out_dir)


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
    for path in write_all(run, args.out_dir).values():
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())