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
from aero_convert.units import SIGN_INVARIANTS
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

    def test_6_written_file_carries_the_eight_header_facts(self) -> None:
        """Test 6: the build writes a ``//`` comment block naming the eight facts."""
        with TemporaryDirectory() as tmp:
            written = build(Path(tmp))
            self.assertIn("tornado.jsonc", written)
            self.assertIn("geometry.jsonc", written)
            text = Path(written["tornado.jsonc"]).read_text()
        lines = text.splitlines()
        self.assertTrue(lines[0].lstrip().startswith("//"), "the file must open with a // comment")
        brace = next(i for i, line in enumerate(lines) if line.strip().startswith("{"))
        self.assertGreater(brace, 0, "the comment block must come before the opening brace")
        header = "\n".join(lines[:brace])
        for needle in HEADER_SUBSTRINGS:
            with self.subTest(needle=needle):
                self.assertIn(needle, header, f"the header comment must mention {needle!r}")

    def test_7_committed_file_carries_the_eight_header_facts(self) -> None:
        """The committed deliverable keeps the header, so it is checked without a build."""
        path = Path(__file__).resolve().parents[2] / "data" / "planes" / "cessna172" / "tornado.jsonc"
        self.assertTrue(path.exists(), f"{path} is missing; run scripts/build_cessna172_aero.py")
        text = path.read_text()
        brace = next(i for i, line in enumerate(text.splitlines()) if line.strip().startswith("{"))
        header = "\n".join(text.splitlines()[:brace])
        for needle in HEADER_SUBSTRINGS:
            with self.subTest(needle=needle):
                self.assertIn(needle, header)

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