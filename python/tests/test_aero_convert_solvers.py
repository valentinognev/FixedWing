"""The Tornado adapter over the real Cessna 172 source geometry.

These tests run the sibling ``aid`` vortex-lattice solver, so they are the only
ones in this directory that are not pure arithmetic.  They pin what the plan's
"Physics invariants" table demands of the flight model: every slot the schema
can represent is filled, every filled slot is on the invariant side of zero,
``cn_da`` is real (the slot DATCOM cannot produce), and the static margin lands
inside 5-45 % MAC.  Test 6 pins the head-of-file comment, which is the only
place a future agent learns the eight facts needed to re-derive the file.
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
        """missing / flipped / not-normalised are three distinct, separately recorded states."""
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


if __name__ == "__main__":
    unittest.main()