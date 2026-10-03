"""The four solver adapters over the real Cessna 172 source geometry.

These tests run the sibling ``aid`` solvers -- the vortex-lattice code, the
Digital DATCOM Fortran binary, AVL and flow5 -- so they are the only ones in
this directory that are not pure arithmetic.  They pin what the plan's
"Physics invariants" table demands of the flight model and of its three
cross-checks: every slot the schema can represent is filled, every filled slot
is on the invariant side of zero, ``cn_da`` is real for Tornado and AVL (the
slot DATCOM cannot produce), and the static margin lands inside 5-45 % MAC.
Test 6 pins the head-of-file comment, which is the only place a future agent
learns the eight facts needed to re-derive the file.

The three cross-check adapters have DIFFERENT coverage, and that is measured
rather than assumed, so each test states the solver's real capability:

* DATCOM runs the Fortran plus the AID handbook, and produces 22 of the 25
  ``DerivativeSet`` fields.  ``cn_da`` and ``cy_da`` are ``None`` because AID's
  ``lateral.py:616`` hardcodes ``0.1`` for both and ``handbook_controls`` omits
  aileron yaw and aileron side force entirely; ``cd_q`` is ``None`` because
  DATCOM Section 7 has no axial-rate formula at all and
  ``longitudinal_dynamic.py`` hardcodes ``cxq = 0.0``, which records the absence
  of a formula rather than a computed zero.
* AVL produces all 25, because its ``.sb`` carries aileron and rudder
  deflection columns.
* flow5's runner emits only ``CL``, ``CD`` and ``Cm`` against alpha
  (``FLOW5/run/flow5_run.cpp`` reads ``polar.alpha_deg`` and nothing else, and
  calls ``setComputeDerivatives(false)``), so it has no sideslip, no rate and
  no lateral-control derivative at all: 7 of 25.  That is the fail-closed
  answer, not a gap to be papered over.
"""
from __future__ import annotations

import math
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from aero_convert.morelli import to_morelli
from aero_convert.solvers import build, tornado_run
from aero_convert.units import DEG_TO_RAD, SIGN_INVARIANTS, normalise_sign
from plane.groups import MORELLI_LENGTHS, nonlinear_index

# The three cross-check models, and the names their adapters must expose.  The
# names are looked up through ``getattr`` at call time rather than imported at
# module scope so a missing adapter fails inside its own test instead of
# breaking the import of this whole module.
CROSS_MODELS = ("datcom", "avl", "flow5")
MODEL_NAMES = CROSS_MODELS

# The 19 slots the plan's Task 3 RED list names, in the plan's order.
REQUIRED_FIELDS = (
    "cl_alpha", "cm_alpha", "cl_q", "cm_q", "cl_beta", "cy_beta", "cn_beta",
    "cl_p", "cl_r", "cn_p", "cn_r", "cl_da", "cy_da", "cn_da", "cl_dr",
    "cy_dr", "cn_dr", "cl_de", "cm_de",
)

# The eight substrings the head-of-file comment must carry (plan, Task 3, RED 6).
HEADER_SUBSTRINGS = (
    "Tornado",
    "1/3",
    "72.574",
    "57.29577951308232",
    "ref_point",
    "Cd0",
    "CD_alpha",
    "build_cessna172_aero.py",
)

STATIC_MARGIN_BAND = (0.05, 0.45)

# The plan's Task 4 RED list, verbatim, for the two adapters that can produce all
# eight of those slots.
CROSS_STATIC_AND_CONTROL = (
    "cl_alpha", "cm_alpha", "cl_beta", "cy_beta", "cn_beta",
    "cl_da", "cl_de", "cm_de",
)

# flow5 can produce four of those eight.  The other four are absent because the
# runner has no sideslip sweep and no lateral coefficients at all, and that is
# measured from FLOW5/run/flow5_run.cpp, not assumed.
FLOW5_ABSENT = ("cl_beta", "cy_beta", "cn_beta", "cl_da")

# Every DerivativeSet field, so the coverage test can state it exhaustively.
ALL_FIELDS = tuple(SIGN_INVARIANTS) + ("cm0",)

# The eight head-of-file items each cross-check file must carry.  Item 5 names
# the reference each solver actually reads, so it differs per solver; the rest
# are shared.
CROSS_HEADER_SUBSTRINGS = {
    "datcom": ("DATCOM", "1/3", "72.574", "57.29577951308232", "XCG", "Cd0",
               "CD_alpha", "build_cessna172_aero.py"),
    "avl": ("AVL", "1/3", "72.574", "57.29577951308232", "Xref", "Cd0",
            "CD_alpha", "build_cessna172_aero.py"),
    "flow5": ("flow5", "1/3", "72.574", "57.29577951308232", "cog_m", "Cd0",
              "CD_alpha", "build_cessna172_aero.py"),
}

# Plan Task 4 RED 7: every model's cl_alpha (i.e. cz[1], negative) must be in this
# band, so a unit or reference slip in any single adapter is visible.
CL_ALPHA_BAND = (-6.5, -4.0)


class TornadoAdapterTest(unittest.TestCase):
    """One solver run shared by every test in the class."""

    solver_run = None

    @classmethod
    def setUpClass(cls) -> None:
        cls.solver_run = tornado_run()

    def coefficients(self) -> dict[str, list[float]]:
        coefficients, missing = to_morelli(self.solver_run.derivatives)
        self.assertEqual(missing, [], f"slots zeroed because the solver missed them: {missing}")
        return coefficients

    def test_1_required_slots_are_present(self) -> None:
        """Test 1: none of the 19 named slots is ``None``."""
        for field in REQUIRED_FIELDS:
            with self.subTest(field=field):
                value = getattr(self.solver_run.derivatives, field)
                self.assertIsNotNone(value, f"{field} was not produced by the solver")
                self.assertTrue(
                    math.isfinite(float(value)),
                    f"{field} is not finite: {value!r}",
                )

    def test_2_every_filled_slot_satisfies_its_invariant(self) -> None:
        """Test 2: each of the 19 sits on the sign its invariant row requires."""
        for field in REQUIRED_FIELDS:
            with self.subTest(field=field):
                value = float(getattr(self.solver_run.derivatives, field))
                required = SIGN_INVARIANTS[field]
                if required > 0:
                    self.assertGreater(value, 0.0, f"{field} must be > 0, got {value!r}")
                else:
                    self.assertLess(value, 0.0, f"{field} must be < 0, got {value!r}")

    def test_3_cn_da_is_non_zero(self) -> None:
        """Test 3: ``cn_da`` is the slot DATCOM cannot produce, so it proves completeness."""
        cn_da = self.solver_run.derivatives.cn_da
        self.assertIsNotNone(cn_da)
        self.assertNotEqual(float(cn_da), 0.0, "cn_da must be real solver output, not a placeholder")

    def test_4_arrays_are_well_formed(self) -> None:
        """Test 4: right lengths, every nonlinear slot exactly 0.0, all finite."""
        coefficients = self.coefficients()
        self.assertEqual(sorted(coefficients), sorted(MORELLI_LENGTHS))
        for name, length in MORELLI_LENGTHS.items():
            with self.subTest(array=name):
                self.assertEqual(len(coefficients[name]), length)
                for index in nonlinear_index(name):
                    self.assertEqual(coefficients[name][index], 0.0)
                for value in coefficients[name]:
                    self.assertTrue(math.isfinite(float(value)), f"{name} carries {value!r}")

    def test_5_static_margin_is_in_band(self) -> None:
        """Test 5: ``cm[1] / cz[1]`` in cbar, both slots negative, so the ratio is positive."""
        coefficients = self.coefficients()
        margin = coefficients["cm"][1] / coefficients["cz"][1]
        low, high = STATIC_MARGIN_BAND
        self.assertGreater(margin, low, f"static margin {margin} cbar is below {low}")
        self.assertLess(margin, high, f"static margin {margin} cbar is above {high}")

    def test_6_written_files_carry_the_eight_header_facts(self) -> None:
        """Test 6: the build writes a ``//`` comment block naming the eight facts.

        Checked on BOTH files: the plan requires every generated file to carry
        them, and a header that only the model file has is half a record.
        """
        with TemporaryDirectory() as tmp:
            written = build(Path(tmp))
            self.assertIn("tornado.jsonc", written)
            self.assertIn("geometry.jsonc", written)
            for name in ("tornado.jsonc", "geometry.jsonc"):
                with self.subTest(file=name):
                    self.assert_header(Path(written[name]).read_text())

    def assert_header(self, text: str) -> None:
        lines = text.splitlines()
        self.assertTrue(lines[0].lstrip().startswith("//"), "the file must open with a // comment")
        brace = next(i for i, line in enumerate(lines) if line.strip().startswith("{"))
        self.assertGreater(brace, 0, "the comment block must come before the opening brace")
        header = "\n".join(lines[:brace])
        for needle in HEADER_SUBSTRINGS:
            with self.subTest(needle=needle):
                self.assertIn(needle, header, f"the header comment must mention {needle!r}")

    def test_7_committed_files_carry_the_eight_header_facts(self) -> None:
        """The committed deliverables keep their headers, checked without a build."""
        directory = Path(__file__).resolve().parents[2] / "data" / "planes" / "cessna172"
        for name in ("tornado.jsonc", "geometry.jsonc"):
            with self.subTest(file=name):
                path = directory / name
                self.assertTrue(path.exists(), f"{path} is missing; run scripts/build_cessna172_aero.py")
                self.assert_header(path.read_text())

    def test_8_provenance_records_three_states(self) -> None:
        """missing / flipped / not-normalised are three distinct, separately recorded states.

        Unlike Tornado, none of the three cross-check solvers needs a flip for
        ``cm0`` and the three states are still all recorded: DATCOM flips four
        slots, AVL seven, and flow5 none -- flow5's mapped values already land on
        the invariant side, which is a fact about the data, not a missing record.
        """
        provenance = self.solver_run.provenance
        self.assertEqual(provenance["missing"], [])
        self.assertIn("flipped", provenance)
        self.assertEqual(provenance["not_normalised"], ["cm0"])
        self.assertEqual(sorted(provenance["flipped"]), sorted(set(provenance["flipped"])))
        for field in provenance["flipped"]:
            with self.subTest(field=field):
                value = float(getattr(self.solver_run.derivatives, field))
                required = SIGN_INVARIANTS[field]
                self.assertEqual((value > 0.0), (required > 0))
        self.assertTrue(provenance["flipped"], "Tornado's raw signs are inverted; flips must be logged")

    def test_10_control_magnitudes_pin_the_per_degree_factor(self) -> None:
        """A doubled DEG_TO_RAD, or a slot mix-up inside a control group, must fail.

        Signs alone cannot catch either, so every control-derivative magnitude is
        checked against the solver's own per-degree number multiplied by
        ``DEG_TO_RAD`` exactly.  ``raw_tornado`` is the untouched ground truth.
        """
        raw = self.solver_run.raw
        rows = {
            row["surface"]: row for row in self.solver_run.control_rows
        }
        coefficients = self.coefficients()
        cases = (
            ("clda", 0, "aileron", "Cl", -1.0),
            ("cy", 1, "aileron", "CY", 1.0),
            ("cnda", 0, "aileron", "Cn", -1.0),
            ("cldr", 0, "rudder", "Cl", -1.0),
            ("cy", 2, "rudder", "CY", 1.0),
            ("cndr", 0, "rudder", "Cn", -1.0),
            ("cz", 5, "elevator", "CL", -1.0),
        )
        for array, index, surface, key, sign in cases:
            with self.subTest(array=array, surface=surface, key=key):
                per_degree = float(rows[surface][key])
                self.assertGreater(abs(per_degree), 0.0, "the solver row must be non-zero")
                # Magnitude only: normalise_sign may flip the sign afterwards, and
                # test 2 already pins that.  What must not change is the size.
                expected = abs(per_degree) * DEG_TO_RAD
                self.assertAlmostEqual(
                    abs(coefficients[array][index]), expected, places=12,
                    msg=f"{array}[{index}] magnitude must be |{per_degree}| x {DEG_TO_RAD}",
                )
                self.assertNotAlmostEqual(
                    abs(coefficients[array][index]), 2.0 * expected, places=6,
                    msg="a doubled DEG_TO_RAD would produce this",
                )
        # cm[2] is shifted, so it is pinned against its own shift, not raw.
        self.assertNotAlmostEqual(
            coefficients["cm"][2],
            float(rows["elevator"]["Cm"]) * DEG_TO_RAD,
            places=3,
            msg="cm_de is the CG-shifted value and must differ from the about-ref_point one",
        )
        # A group mix-up (aileron numbers landing in the rudder slots) must fail.
        self.assertNotAlmostEqual(
            abs(coefficients["clda"][0]), abs(coefficients["cldr"][0]), places=6,
        )
        self.assertNotAlmostEqual(
            abs(coefficients["cy"][1]), abs(coefficients["cy"][2]), places=6,
        )
        self.assertIn("CY_b", raw)
        self.assertIn("CZ_Q", raw, "the shift argument's source must be re-derivable")

    def test_11_shift_uses_the_body_z_force_slope(self) -> None:
        """cm[1] must come from CZ_a, not from CL_a: they differ, and it shows."""
        raw = self.solver_run.raw
        geometry = self.solver_run.geometry
        from aero_convert.units import shift_moments_to_cg

        def shifted(cz: float) -> float:
            return shift_moments_to_cg(
                cz, 0.0, 0.0, 0.0, raw["Cm_a"], 0.0,
                dx=geometry.dx_m, dy=geometry.dy_m, dz=geometry.dz_m,
                s_ref=geometry.s_ref_m2, b_ref=geometry.b_ref_m, c_ref=geometry.c_ref_m,
            )[4]

        cm_from_cz = shifted(-raw["CZ_a"])
        cm_from_cl = shifted(-raw["CL_a"])
        self.assertNotAlmostEqual(cm_from_cz, cm_from_cl, places=3)
        self.assertAlmostEqual(
            self.coefficients()["cm"][1], cm_from_cz, places=12,
            msg="cm[1] must be the shift of Cm_a with cz = -CZ_a",
        )
        # and the q slope likewise
        self.assertAlmostEqual(
            self.coefficients()["cmq"][0],
            shift_moments_to_cg(
                -raw["CZ_Q"], 0.0, 0.0, 0.0, raw["Cm_Q"], 0.0,
                dx=geometry.dx_m, dy=geometry.dy_m, dz=geometry.dz_m,
                s_ref=geometry.s_ref_m2, b_ref=geometry.b_ref_m, c_ref=geometry.c_ref_m,
            )[4],
            places=12,
            msg="cmq[0] must be the shift of Cm_Q with cz = -CZ_Q",
        )

    def test_12_mapping_separates_the_solver_value_from_the_mapped_one(self) -> None:
        """Provenance must show what the solver said AND what was done to it."""
        entries = {entry["field"]: entry for entry in self.solver_run.provenance["mapping"]}
        # (field, frame sign, per-degree?) -- the static fields are already per
        # radian, so only the frame sign separates them from the solver's number;
        # the control rows are per degree and must carry DEG_TO_RAD as well.
        for field, frame_sign, per_degree in (
            ("cl_alpha", -1.0, False), ("cl_q", -1.0, False), ("cl_beta", -1.0, False),
            ("cy_p", 1.0, False), ("cy_beta", 1.0, False),
            ("cl_da", -1.0, True), ("cn_da", -1.0, True), ("cy_da", 1.0, True),
            ("cl_de", -1.0, True), ("cl_dr", -1.0, True),
        ):
            with self.subTest(field=field):
                entry = entries[field]
                self.assertIsNotNone(entry["solver_value"])
                factor = DEG_TO_RAD if per_degree else 1.0
                expected = frame_sign * entry["solver_value"] * factor
                self.assertAlmostEqual(
                    entry["mapped_value"], expected, places=12,
                    msg=f"{field}: the mapped value must be the solver value with the "
                        f"documented frame sign{' and DEG_TO_RAD' if per_degree else ''}",
                )

        # cd0 is the one slot whose number is not a solver output, and it says so.
        self.assertIsNone(entries["cd0"]["solver_value"])
        self.assertEqual(entries["cd0"]["solver_input"], "source file")
        self.assertIn("WG.CD0", entries["cd0"]["tornado_key"])
        for field, entry in entries.items():
            with self.subTest(field=field):
                self.assertNotIn("raw", entry, "the key 'raw' is the misleading name")

    def test_13_trim_residual_is_zero(self) -> None:
        """The initial state must be an equilibrium of plane/dynamics.py itself."""
        self.assertLess(self.solver_run.trim["residual_max_abs"], 1e-9)
        self.assertFalse(self.solver_run.trim["fallback"])
        self.assertAlmostEqual(self.solver_run.trim["cl_at_trim"], 0.30, places=9)
        self.assertAlmostEqual(
            self.solver_run.trim["cm_at_trim"], 0.0, places=9,
        )

    def test_14_pr_column_flip_changes_no_written_coefficient(self) -> None:
        """The P/R component-sense defect is immaterial, and that is pinned here.

        The amended plan records that Tornado's P and R columns come back
        component-sense inverted independently of the axis mapping.  Negating
        those six solver values -- and, with them, the force coefficients their
        yaw shift consumes -- must leave every written coefficient untouched,
        because normalise_sign is the final authority.  If a future change ever
        relies on the P/R signs, this fails.
        """
        from aero_convert.units import shift_moments_to_cg

        raw = self.solver_run.raw
        geometry = self.solver_run.geometry
        shipped = self.coefficients()

        def shift(cz: float, cy: float, cm: float, cn: float) -> tuple[float, float]:
            out = shift_moments_to_cg(
                cz, 0.0, cy, 0.0, cm, cn,
                dx=geometry.dx_m, dy=geometry.dy_m, dz=geometry.dz_m,
                s_ref=geometry.s_ref_m2, b_ref=geometry.b_ref_m, c_ref=geometry.c_ref_m,
            )
            return out[4], out[5]

        # (a) as shipped, (b) with the P and R columns negated end to end.
        variants = {}
        for label, flip in (("shipped", 1.0), ("pr_flipped", -1.0)):
            cy_p = flip * raw["CY_P"]
            cy_r = flip * raw["CY_R"]
            cm_alpha, _ = shift(-raw["CZ_a"], 0.0, raw["Cm_a"], 0.0)
            cm_q, _ = shift(-raw["CZ_Q"], 0.0, raw["Cm_Q"], 0.0)
            _, cn_beta = shift(0.0, raw["CY_b"], 0.0, -raw["Cn_b"])
            _, cn_p = shift(0.0, cy_p, 0.0, -flip * raw["Cn_P"])
            _, cn_r = shift(0.0, cy_r, 0.0, -flip * raw["Cn_R"])
            variants[label] = {
                "cl[0]": -flip * raw["Cl_b"],
                "cy[0]": raw["CY_b"],
                "clp[0]": -flip * raw["Cl_P"],
                "clr[0]": -flip * raw["Cl_R"],
                "cyp[0]": cy_p,
                "cyr[0]": cy_r,
                "cnp[0]": cn_p,
                "cnr[0]": cn_r,
                "cn[0]": cn_beta,
                "cm[1]": cm_alpha,
                "cmq[0]": cm_q,
            }
        fields = {
            "cl[0]": "cl_beta", "cy[0]": "cy_beta", "clp[0]": "cl_p", "clr[0]": "cl_r",
            "cyp[0]": "cy_p", "cyr[0]": "cy_r", "cnp[0]": "cn_p", "cnr[0]": "cn_r",
            "cn[0]": "cn_beta", "cm[1]": "cm_alpha", "cmq[0]": "cm_q",
        }
        for slot, mapped in variants["pr_flipped"].items():
            with self.subTest(slot=slot):
                array, index = slot.split("[")
                index = int(index.rstrip("]"))
                expected = normalise_sign(fields[slot], mapped)[0]
                self.assertAlmostEqual(expected, shipped[array][index], places=12)
                self.assertAlmostEqual(
                    normalise_sign(fields[slot], variants["shipped"][slot])[0],
                    shipped[array][index], places=12,
                )

    def test_15_dropping_the_shift_moves_exactly_seven_slots(self) -> None:
        """The other half of the materiality claim: the SHIFT is not immaterial.

        The P/R sign dispute changes nothing (test_14), but discarding the CG shift
        as well is a different hypothesis and moves exactly the seven slots the
        shift touches -- cm[0], cm[1], cm[2], cmq[0], cn[0], cnp[0], cnr[0].  This
        pins the boundary between the two claims, so "the sign dispute is
        immaterial" can never be read as "the moment reference is".
        """
        from aero_convert.units import shift_moments_to_cg

        run = self.solver_run
        raw = run.raw
        geometry = run.geometry
        shipped = self.coefficients()
        rows = {row["surface"]: row for row in run.control_rows}
        elevator = rows["elevator"]

        def shift(cz: float, cy: float, cm: float, cn: float) -> tuple[float, float]:
            out = shift_moments_to_cg(
                cz, 0.0, cy, 0.0, cm, cn,
                dx=geometry.dx_m, dy=geometry.dy_m, dz=geometry.dz_m,
                s_ref=geometry.s_ref_m2, b_ref=geometry.b_ref_m, c_ref=geometry.c_ref_m,
            )
            return out[4], out[5]

        sin_a = math.sin(math.radians(geometry.alpha_deg))
        cos_a = math.cos(math.radians(geometry.alpha_deg))
        cl_de = float(elevator["CL"]) * DEG_TO_RAD
        cd_de = float(elevator["CD"]) * DEG_TO_RAD
        cz_de = -(cos_a * cl_de + sin_a * cd_de)
        cm_de = float(elevator["Cm"]) * DEG_TO_RAD

        cases = {
            "cm[1]": ("cm_alpha", shift(-raw["CZ_a"], 0.0, raw["Cm_a"], 0.0)[0], raw["Cm_a"]),
            "cm[0]": ("cm0", shift(-raw["CL_at_alpha0"], 0.0, raw["Cm_at_alpha0"], 0.0)[0],
                      raw["Cm_at_alpha0"]),
            "cmq[0]": ("cm_q", shift(-raw["CZ_Q"], 0.0, raw["Cm_Q"], 0.0)[0], raw["Cm_Q"]),
            "cm[2]": ("cm_de", shift(cz_de, 0.0, cm_de, 0.0)[0], cm_de),
            "cn[0]": ("cn_beta", shift(0.0, raw["CY_b"], 0.0, -raw["Cn_b"])[1], -raw["Cn_b"]),
            "cnp[0]": ("cn_p", shift(0.0, raw["CY_P"], 0.0, -raw["Cn_P"])[1], -raw["Cn_P"]),
            "cnr[0]": ("cn_r", shift(0.0, raw["CY_R"], 0.0, -raw["Cn_R"])[1], -raw["Cn_R"]),
        }
        moved = set()
        for slot, (field, mapped, unshifted) in cases.items():
            with self.subTest(slot=slot):
                array, index = slot.split("[")
                index = int(index.rstrip("]"))
                self.assertAlmostEqual(
                    normalise_sign(field, mapped)[0], shipped[array][index], places=12,
                    msg=f"{slot} must be the CG-shifted value the file ships",
                )
                if normalise_sign(field, mapped)[0] != normalise_sign(field, unshifted)[0]:
                    moved.add(slot)
        self.assertEqual(moved, set(cases), "exactly the seven shifted slots must move")

    def test_9_trimmed_aircraft_is_positive_where_it_must_be(self) -> None:
        """plane/aircraft.py's _POSITIVE_KEYS: nothing may be zero or negative."""
        aircraft = self.solver_run.aircraft
        for key in (
            "mass_kg", "s_m2", "b_m", "cbar_m", "ixx", "iyy", "izz", "tau_s",
        ):
            with self.subTest(key=key):
                self.assertGreater(aircraft[key], 0.0)
        self.assertGreaterEqual(aircraft["t_max_n"], 0.0)
        self.assertEqual(aircraft["ixz"], 0.0)
        self.assertEqual(aircraft["he"], 0.0)


class CrossCheckAdapterTest(unittest.TestCase):
    """DATCOM, AVL and flow5 over the same source geometry.

    One solver run per model, shared by every test in the class: the DATCOM
    Fortran and the AID handbook are cheap, AVL is one small run, and flow5's
    control derivatives cost eight panel runs, so repeating them per test would
    dominate the suite.
    """

    runs: dict = {}

    @classmethod
    def setUpClass(cls) -> None:
        import aero_convert.solvers as solvers

        cls.runs = {name: getattr(solvers, f"{name}_run")() for name in CROSS_MODELS}

    @staticmethod
    def write_model(run, out_dir: Path) -> Path:
        from aero_convert.solvers import write_model

        return write_model(run, out_dir)

    @staticmethod
    def cross_check_payload(run) -> dict:
        from aero_convert.solvers import cross_check_payload

        return cross_check_payload(run)

    def coefficients(self, model: str) -> dict[str, list[float]]:
        return to_morelli(self.runs[model].derivatives)[0]

    def test_1_datcom_and_avl_produce_the_named_slots(self) -> None:
        """Plan Task 4 RED 1: the eight named slots are real for both."""
        for model in ("datcom", "avl"):
            for field in CROSS_STATIC_AND_CONTROL:
                with self.subTest(model=model, field=field):
                    value = getattr(self.runs[model].derivatives, field)
                    self.assertIsNotNone(value, f"{model} did not produce {field}")
                    self.assertTrue(math.isfinite(float(value)), f"{field} = {value!r}")

    def test_1b_flow5_produces_only_what_its_runner_emits(self) -> None:
        """Plan Task 4 RED 1, corrected for what flow5 can actually do.

        ``FLOW5/run/flow5_run.cpp`` sweeps ``polar.alpha_deg`` only, hardcodes
        ``setComputeDerivatives(false)``, and serialises ``CL``, ``CD`` and
        ``Cm`` alone -- so flow5 has no sideslip row, no rate row and no roll or
        yaw coefficient to difference.  The four slots the plan's RED 1 names
        must therefore be ``None`` and declared, and the four it can produce
        must be real.
        """
        derivatives = self.runs["flow5"].derivatives
        for field in ("cl_alpha", "cm_alpha", "cl_de", "cm_de"):
            with self.subTest(field=field):
                value = getattr(derivatives, field)
                self.assertIsNotNone(value, f"flow5 did not produce {field}")
                self.assertTrue(math.isfinite(float(value)), f"{field} = {value!r}")
        for field in FLOW5_ABSENT:
            with self.subTest(field=field):
                self.assertIsNone(
                    getattr(derivatives, field),
                    f"flow5 has no {field}; a value here would be invented",
                )
                declared = {entry["field"] for entry in self.runs["flow5"].provenance["missing"]}
                self.assertIn(field, declared, f"{field} must be declared in provenance.missing")

    def test_2_every_filled_slot_satisfies_its_invariant(self) -> None:
        """Plan Task 4 RED 2, narrowed to the SIGN column, over every field produced.

        **DECLARED NARROWING.**  The plan's "Physics invariants" table has two
        columns per row: a required sign and a plausible magnitude.  This test
        asserts the SIGN column only.  The magnitude column is asserted in
        Task 5's suite, ``python/tests/test_plane_cessna172_json.py``, which the
        plan makes its home ("every row of the Physics invariants table holds,
        magnitudes included").  Asserting both here would give one requirement two
        homes and two places to keep in sync, so this file says so out loud rather
        than quietly covering half of it.

        Why the narrowing matters here: ``flow5``'s ``cm[1] = -1.573326`` sits
        4.9 % outside the plan's ``-1.5 ... -0.3`` pitched-moment-slope band, which
        a magnitude assertion would surface as a failure.  It is RECORDED rather
        than forced into the band -- in ``provenance.coverage.invariant_band_note``
        and in item 6 of the file's own header -- because the sign is correct, the
        static margin (0.304334 cbar) is correct, and the band was evidently
        estimated from ``tornado.jsonc``'s -1.1927.  The controller has accepted
        that: recording it beats forcing it.
        """
        for model in MODEL_NAMES:
            derivatives = self.runs[model].derivatives
            for field in ALL_FIELDS:
                value = getattr(derivatives, field)
                if value is None:
                    continue
                with self.subTest(model=model, field=field):
                    value = float(value)
                    self.assertNotEqual(value, 0.0, f"{model} {field} is exactly zero")
                    required = SIGN_INVARIANTS.get(field)
                    if required is None:
                        continue  # cm0 carries no invariant; test 8 pins that
                    if required > 0:
                        self.assertGreater(value, 0.0, f"{model} {field} must be > 0, got {value!r}")
                    else:
                        self.assertLess(value, 0.0, f"{model} {field} must be < 0, got {value!r}")

    def test_3_flow5_has_no_rate_derivatives(self) -> None:
        """Plan Task 4 RED 3: a steady panel code has no rate rows at all."""
        derivatives = self.runs["flow5"].derivatives
        for field in ("cl_q", "cm_q", "cy_p", "cy_r", "cl_p", "cl_r", "cn_p",
                      "cn_r", "cd_q"):
            with self.subTest(field=field):
                self.assertIsNone(getattr(derivatives, field))

    def test_4_datcom_has_no_cn_da_or_cy_da(self) -> None:
        """Plan Task 4 RED 4: AID's 0.1 placeholders must not reach the file.

        ``aid/lateral.py:616`` returns a hardcoded ``0.1`` for both, and
        ``aid/handbook_controls.py`` has no aileron CY or Cn column at all.  The
        slots are ``None``, zeroed and declared; ``0.1`` must appear nowhere.
        """
        derivatives = self.runs["datcom"].derivatives
        slots = {"cn_da": ("cnda", 0), "cy_da": ("cy", 1)}
        for field, (array, index) in slots.items():
            with self.subTest(field=field):
                self.assertIsNone(getattr(derivatives, field))
                self.assertEqual(self.coefficients("datcom")[array][index], 0.0)
        # The placeholder must not be hiding as a reported solver value either.
        for entry in self.runs["datcom"].provenance["mapping"]:
            if entry["field"] in slots:
                self.assertIsNone(entry["solver_value"])
        lateral = self.runs["datcom"].provenance["handbook_cross_check"]["aid_lateral_as_reported"]
        self.assertNotEqual(float(lateral["Clb"]), 0.0, "Clb is a real handbook result")
        self.assertAlmostEqual(float(lateral["Clb"]), -0.003090622882802272, places=12,
                               msg="AID's Clb is the 57.3x-too-small number it reports")
        # ...while AVL does produce both, from its own .sb.
        for field in ("cn_da", "cy_da"):
            with self.subTest(model="avl", field=field):
                value = getattr(self.runs["avl"].derivatives, field)
                self.assertIsNotNone(value)
                self.assertNotAlmostEqual(float(value), 0.0, places=6)

    def test_5_static_margin_is_in_band(self) -> None:
        """Plan Task 4 RED 5: ``cm[1] / cz[1]`` in cbar for all three."""
        for model in MODEL_NAMES:
            with self.subTest(model=model):
                coefficients = self.coefficients(model)
                margin = coefficients["cm"][1] / coefficients["cz"][1]
                low, high = STATIC_MARGIN_BAND
                self.assertGreater(margin, low, f"{model} margin {margin} below {low}")
                self.assertLess(margin, high, f"{model} margin {margin} above {high}")

    def test_6_written_files_carry_the_eight_header_facts(self) -> None:
        """Plan Task 4 RED 6: a ``//`` block naming the eight facts, both fresh and committed."""
        with TemporaryDirectory() as tmp:
            for model in MODEL_NAMES:
                run = self.runs[model]
                path = self.write_model(run, Path(tmp))
                with self.subTest(model=model, source="fresh"):
                    self.assertEqual(path.name, f"{model}.jsonc")
                    self.assert_header(model, path.read_text())
        directory = Path(__file__).resolve().parents[2] / "data" / "planes" / "cessna172"
        for model in MODEL_NAMES:
            with self.subTest(model=model, source="committed"):
                path = directory / f"{model}.jsonc"
                self.assertTrue(path.is_file(), f"{path} is missing; run the build script")
                self.assert_header(model, path.read_text())

    def assert_header(self, model: str, text: str) -> None:
        lines = text.splitlines()
        self.assertTrue(lines[0].lstrip().startswith("//"), "the file must open with a // comment")
        brace = next(i for i, line in enumerate(lines) if line.strip().startswith("{"))
        self.assertGreater(brace, 0, "the comment block must come before the opening brace")
        header = "\n".join(lines[:brace])
        for needle in CROSS_HEADER_SUBSTRINGS[model]:
            with self.subTest(model=model, needle=needle):
                self.assertIn(needle, header, f"the header comment must mention {needle!r}")

    def test_7_cross_model_lift_slope_agrees(self) -> None:
        """Plan Task 4 RED 7: every model's cl_alpha sits in the same band.

        Tornado, DATCOM, AVL and flow5 are four independent methods on one
        planform, so a unit or reference slip in any single adapter moves its
        lift slope out of a band the other three agree on.
        """
        band_low, band_high = CL_ALPHA_BAND
        values = {}
        for model in ("tornado", *MODEL_NAMES):
            value = float(to_morelli(self.runs[model].derivatives
                                     if model in self.runs else tornado_run().derivatives)[0]["cz"][1])
            values[model] = value
            with self.subTest(model=model):
                self.assertGreater(value, band_low, f"{model} cz[1] = {value} is not below {band_low}")
                self.assertLess(value, band_high, f"{model} cz[1] = {value} is not above {band_high}")
        worst = max(values.values()) / min(values.values())
        self.assertLess(worst, 1.6, f"lift slopes {values} disagree by {worst:.3f}")

    def test_8_provenance_records_three_states(self) -> None:
        """missing / flipped / not-normalised are three distinct, separately recorded states.

        Unlike Tornado, none of the three cross-check solvers needs a flip for
        ``cm0`` and the three states are still all recorded: DATCOM flips four
        slots, AVL seven, and flow5 none -- flow5's mapped values already land on
        the invariant side, which is a fact about the data, not a missing record.
        """
        for model in MODEL_NAMES:
            run = self.runs[model]
            provenance = run.provenance
            with self.subTest(model=model):
                self.assertEqual(sorted(provenance["not_normalised"]), ["cm0"])
                self.assertEqual(
                    sorted(provenance["flipped"]), sorted(set(provenance["flipped"]))
                )
                for field in provenance["flipped"]:
                    value = float(getattr(run.derivatives, field))
                    self.assertEqual((value > 0.0), (SIGN_INVARIANTS[field] > 0))
                # every None field is declared, and nothing declared is non-zero
                declared = {entry["field"] for entry in provenance["missing"]}
                for field in ALL_FIELDS:
                    value = getattr(run.derivatives, field)
                    with self.subTest(model=model, field=field):
                        self.assertEqual(value is None, field in declared)
                        if field in declared:
                            self.assertIsNotNone(
                                next(e for e in provenance["missing"] if e["field"] == field)["reason"],
                                "a declared gap must say why",
                            )

    def test_9_arrays_are_well_formed(self) -> None:
        """Right lengths, every nonlinear slot exactly 0.0, every element finite."""
        for model in MODEL_NAMES:
            coefficients = self.coefficients(model)
            with self.subTest(model=model):
                self.assertEqual(sorted(coefficients), sorted(MORELLI_LENGTHS))
            for name, length in MORELLI_LENGTHS.items():
                with self.subTest(model=model, array=name):
                    self.assertEqual(len(coefficients[name]), length)
                    for index in nonlinear_index(name):
                        self.assertEqual(coefficients[name][index], 0.0)
                    for value in coefficients[name]:
                        self.assertTrue(math.isfinite(float(value)), f"{name} carries {value!r}")

    def test_10_payload_schema_matches_the_in_tree_file(self) -> None:
        """Each payload is schema-identical to data/planes/linear/morelli.json."""
        for model in MODEL_NAMES:
            payload = self.cross_check_payload(self.runs[model])
            with self.subTest(model=model):
                self.assertEqual(payload["model"], model)
                self.assertEqual(sorted(payload["coefficients"]), sorted(MORELLI_LENGTHS))
                for block in ("aircraft", "initial", "controls"):
                    self.assertIsInstance(payload[block], dict)
                    self.assertTrue(payload[block])
                self.assertIn("provenance", payload)
            coefficients = payload["coefficients"]
            for array in ("cx", "cy", "cz", "cm"):
                for value in coefficients[array]:
                    self.assertTrue(math.isfinite(float(value)), f"{array} carries {value!r}")

    def test_11_files_open_through_load_aircraft(self) -> None:
        """The GREEN condition: every file opens as a flight model."""
        from plane.aircraft import load_aircraft

        for model in MODEL_NAMES:
            with self.subTest(model=model):
                aircraft = load_aircraft("cessna172", model)
                self.assertGreater(aircraft.mass_kg, 0.0)
                self.assertEqual(aircraft.model, model)
                coefficients = to_morelli(self.runs[model].derivatives)[0]
                self.assertEqual(
                    coefficients["cm"][1] / coefficients["cz"][1] > 0.05, True
                )

    # Which raw per-solver number each rate slot is derived from, so the assertion
    # below can compare against the SOLVER's value instead of against the value the
    # adapter computed from it.  DATCOM's Section 7 rates live in the run's own raw
    # block; AVL's are the alpha = 4 deg row of its .st sweep.
    RATE_SOURCES = {
        "datcom": ("section7", None),
        "avl": ("st_alpha_rows", "Clp"),
    }
    RATE_FIELDS = ("cl_q", "cm_q", "cl_p", "cl_r", "cy_p", "cy_r", "cn_p", "cn_r")

    def test_12_rate_derivatives_are_not_rescaled_and_the_factor_is_recorded(self) -> None:
        """The nine per-p-hat slots keep the SOLVER's value, and the ratio is written down.

        Three separate claims, each with an assertion that can fail:

        1. the recorded ratio is self-consistent -- ``speed_ratio_v_trim_over_v_source``
           really is ``v_trim_mps / v_source_mps``, and it is the ratio to the right
           direction (the plan's own ``AS_source / V_trim`` is the inverse and would
           make the damping weaker still, not restore it);
        2. every shipped rate coefficient equals ``normalise_sign`` of the SOLVER's
           own per-``p-hat`` number, taken out of ``run.raw`` -- an independent path
           through the data, not the value the adapter computed from it;
        3. that shipped value is NOT the solver's number multiplied by the ratio.
           Claim 3 is the one that fails if the rescale is ever applied, and it is
           why the previous version of this test -- which compared a number with a
           strictly larger bound built from itself -- could never fail at all.
        """
        for model in MODEL_NAMES:
            run = self.runs[model]
            provenance = run.provenance
            condition = provenance["flight_condition"]
            with self.subTest(model=model, claim="the ratio is self-consistent"):
                self.assertIn("v_source_mps", condition)
                self.assertIn("v_trim_mps", condition)
                self.assertIn("speed_ratio_v_trim_over_v_source", condition)
                self.assertIn("rate_derivative_note", condition)
                factor = condition["v_trim_mps"] / condition["v_source_mps"]
                self.assertGreater(factor, 1.0)
                self.assertAlmostEqual(
                    condition["speed_ratio_v_trim_over_v_source"], factor, places=12,
                )
                # the note must quote the number that is actually recorded
                self.assertIn(f"{factor:.5f}", condition["rate_derivative_note"])
                self.assertIn("V_trim/V_source", condition["rate_derivative_note"])

            entries = {e["field"]: e for e in provenance["mapping"]}
            solver_rates = self._solver_rate_values(model, run)
            with self.subTest(model=model, claim="no rescale was applied"):
                checked = 0
                for field in self.RATE_FIELDS:
                    entry = entries.get(field)
                    if entry is None or entry["state"] == "missing":
                        continue
                    raw_value = solver_rates.get(field)
                    if raw_value is None:
                        continue
                    checked += 1
                    shipped = float(entry["value"])
                    self.assertAlmostEqual(
                        shipped, normalise_sign(field, raw_value)[0], places=12,
                        msg=f"{model} {field} must be the solver's own per-p-hat number",
                    )
                    rescaled = normalise_sign(field, raw_value * factor)[0]
                    self.assertNotAlmostEqual(
                        shipped, rescaled, places=6,
                        msg=f"{model} {field} was multiplied by the flight-condition ratio "
                            f"{factor:.5f}; the plan fixes the source condition and records "
                            "the ratio instead",
                    )
                # DATCOM and AVL have eight rate slots each; flow5 has no rate rows
                # at all, which test_3 already pins, so zero is its CORRECT answer and
                # anything else would mean flow5 had started inventing them.
                self.assertEqual(
                    checked, 0 if model == "flow5" else len(self.RATE_FIELDS),
                    f"{model}: expected {0 if model == 'flow5' else len(self.RATE_FIELDS)} "
                    "rate slots to be checked against the solver's own numbers",
                )

    @classmethod
    def _solver_rate_values(cls, model: str, run) -> dict[str, float]:
        """Each rate slot's value straight out of the solver's own raw output.

        DATCOM's Section 7 rates are not printed by anything, so they are read from
        the run's ``raw["section7"]`` block, which is the converter's transcription
        of the formula -- the same source ``provenance.mapping`` names, reached by a
        different route.  AVL's are the alpha = 4 deg row of its own ``.st`` sweep.
        flow5 has no rate rows and yields nothing, which is why it is excluded above.
        """
        if model == "datcom":
            section7 = run.raw["section7"]
            return {
                "cl_q": -section7["cl_q"], "cm_q": section7["cm_q"],
                "cl_p": section7["cl_p"], "cl_r": section7["cl_r"],
                "cy_p": section7["cy_p"], "cy_r": section7["cy_r"],
                "cn_p": section7["cn_p"], "cn_r": section7["cn_r"],
            }
        if model == "avl":
            rows, index = run.raw["st_alpha_rows"], run.raw["alpha_index"]
            return {
                "cl_q": -rows["CLq"][index], "cm_q": rows["Cmq"][index],
                "cl_p": rows["Clp"][index], "cl_r": rows["Clr"][index],
                "cy_p": rows["CYp"][index], "cy_r": rows["CYr"][index],
                "cn_p": rows["Cnp"][index], "cn_r": rows["Cnr"][index],
            }
        return {}

    def test_13_control_slots_are_per_radian_of_deflection(self) -> None:
        """No per-degree control number may reach a Morelli array.

        AID's ``_per_deg``/``_per_rad`` are no-ops below 1.0, so DATCOM's
        handbook control rows, AVL's ``.sb`` deflection columns and flow5's
        ``H_DEG`` central differences all arrive PER DEGREE.  Every control slot
        is the solver's own number times ``DEG_TO_RAD``, exactly.
        """
        for model, surface_fields in (
            ("datcom", (("cl_da", "aileron", "Cl"), ("cl_de", "elevator", "CL"),
                        ("cm_de", "elevator", "Cm"), ("cl_dr", "rudder", "Cl"),
                        ("cy_dr", "rudder", "CY"), ("cn_dr", "rudder", "Cn"))),
            ("avl", (("cl_da", "aileron", "Cl"), ("cl_de", "elevator", "CL"),
                     ("cm_de", "elevator", "Cm"), ("cl_dr", "rudder", "Cl"),
                     ("cy_dr", "rudder", "CY"), ("cn_dr", "rudder", "Cn"),
                     ("cy_da", "aileron", "CY"), ("cn_da", "aileron", "Cn"))),
            ("flow5", (("cl_de", "elevator", "CL"), ("cm_de", "elevator", "Cm"))),
        ):
            entries = {e["field"]: e for e in self.runs[model].provenance["mapping"]}
            rows = self.runs[model].provenance["control_rows"]
            for field, surface, key in surface_fields:
                with self.subTest(model=model, field=field):
                    per_degree = float(rows[surface][key])
                    self.assertNotEqual(per_degree, 0.0)
                    expected = abs(per_degree) * DEG_TO_RAD
                    self.assertAlmostEqual(
                        abs(float(entries[field]["mapped_value"])), expected, places=12,
                        msg=f"{model} {field} must be |{per_degree}| x {DEG_TO_RAD}",
                    )

    def test_14_datcom_rates_are_not_the_broken_handbook_numbers(self) -> None:
        """The 57x ``_per_rad`` defect must not survive into the DATCOM rates.

        ``aid/handbook_pass.py:131``'s ``_per_rad`` is ``if a > 1: convert else
        pass``, so ``_per_rad(0.0756)`` returns 0.0756 where a per-radian lift
        slope is required.  ``aid/longitudinal_dynamic`` and
        ``aid/lateral.lateral_dynamic`` use it, which makes DATCOM's ``CLq``
        0.103 instead of 5.880 and its ``Clp`` 57x too small.  This adapter
        applies the factor by hand, so the shipped values must be 57x the
        handbook's -- and nowhere near them.
        """
        raw = self.runs["datcom"].provenance["handbook_cross_check"]
        broken_clq = float(raw["aid_longitudinal_dynamic_CLq"])
        shipped = abs(float(self.coefficients("datcom")["czq"][0]))
        self.assertGreater(broken_clq, 0.0)
        self.assertGreater(shipped / broken_clq, 50.0)
        self.assertAlmostEqual(shipped / broken_clq, 57.29577951308232, places=6)
        broken_clp = abs(float(raw["aid_lateral_dynamic_Clp"]))
        shipped_clp = abs(float(self.coefficients("datcom")["clp"][0]))
        self.assertGreater(shipped_clp / broken_clp, 50.0)
        self.assertAlmostEqual(shipped_clp / broken_clp, 57.29577951308232, places=6)

    def test_15_static_and_control_derivatives_agree_across_solvers(self) -> None:
        """A shared cross-check on the DATCOM VT lift-slope units correction.

        ``aid/lateral.py:571`` multiplies the DATCOM tail lift slope by
        ``pi/180`` in the wrong direction, which makes every vertical-tail
        contribution 57x too small -- ``Clb`` lands at -0.0031 where the
        sibling's own AVL gold says -0.0659.  Re-applying the per-radian factor
        puts DATCOM's ``cl[0]`` within 25 % of AVL's.
        """
        datcom_clb = abs(float(self.coefficients("datcom")["cl"][0]))
        avl_clb = abs(float(self.coefficients("avl")["cl"][0]))
        aid_clb = abs(float(
            self.runs["datcom"].provenance["handbook_cross_check"]["aid_lateral_as_reported"]["Clb"]
        ))
        self.assertGreater(datcom_clb, 0.0)
        ratio = datcom_clb / avl_clb
        self.assertLess(ratio, 2.0, f"DATCOM cl_beta {datcom_clb} vs AVL {avl_clb}")
        self.assertGreater(ratio, 0.5, f"DATCOM cl_beta {datcom_clb} vs AVL {avl_clb}")
        # The correction is MATERIAL and it is the correction that lands in the band:
        # AID's uncorrected value is 57x smaller and nowhere near AVL.
        self.assertLess(
            aid_clb / avl_clb, 0.5,
            f"aid/lateral.py:571's uncorrected Clb ({aid_clb}) should NOT agree with AVL "
            f"({avl_clb}); if it now does, the units correction has been undone",
        )


if __name__ == "__main__":
    unittest.main()