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
* flow5 produces 21 of 25, read off the 26 keys ``aid.flow5_io.run_flow5``
  returns at mesh ``("10", "10")``: the six longitudinal polar channels, the
  twelve ``StabDerivatives``, ``CLa``/``Cma``, a ``CDvis``/``CDind`` split, the
  body ``Cx``/``Cz``/``Cl``/``Cn`` point channels and the alpha/beta
  schedules.  So every sideslip and every roll/yaw-rate derivative is real,
  and all five aileron/rudder coefficients are real.  The four it cannot fill
  are absent because its *runner* declines to forward them: ``flow5_run.cpp``'s
  ``stab_derivative_fields()`` omits ``CXq``/``CZq``/``Cmq`` (so ``cl_q``,
  ``cm_q``, ``cd_q``) and ``aid.flow5_controls`` never asks the aileron for
  ``CY`` (so ``cy_da``).  They are ``None`` and declared, never zeros -- that
  is the fail-closed answer, not a gap to be papered over.

The coverage counts in this paragraph are recorded in each generated header,
not checked from here: ``test_18_hand_written_counts_in_the_headers_must_match_the_data``
parses the built ``//`` block and checks the claims it still makes -- the
flipped-slot list, the identity-negated set, and the header's
``N OF THE m SLOTS ARE ABSENT`` count -- against the run.  The per-solver
coverage figures above are printed by the build into item 1 of each file
instead, so a stale count here cannot silently disagree with a generated one.
"""
from __future__ import annotations

import inspect
import json
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

# flow5's four genuinely absent slots, measured against its 26 emitted keys rather
# than assumed: its library computes CXq/CZq/Cmq but FLOW5/run/flow5_run.cpp's
# stab_derivative_fields() does not forward them, and aid/flow5_controls' _KEEP
# table asks the aileron for Cl and Cn only and never for CY.
FLOW5_ABSENT = ("cl_q", "cm_q", "cd_q", "cy_da")

# The slots flow5 gained when its runner began emitting the twelve
# StabDerivatives and the lateral control rows.  Named so that a regression which
# silently stops reading one of them is a named failure, not just a coverage count.
FLOW5_NEWLY_PRODUCED = (
    "cl_beta", "cy_beta", "cn_beta", "cy_p", "cy_r", "cl_p", "cl_r",
    "cn_p", "cn_r", "cl_da", "cn_da", "cl_dr", "cy_dr", "cn_dr",
)

# flow5's Clb against four meshes, as the sibling records it.  At ("10","5") it goes
# POSITIVE and |Clb| is ~30x too small, which reads as a silent sign inversion
# rather than as a convergence failure -- that is why FLOW5_MESH is pinned by a
# test instead of left to a comment.  How each mesh compares with the OTHER runs is
# a comparison between runs, so it is not written here; the pin is against the one
# mesh this table records.
FLOW5_CLB_BY_MESH = {
    ("5", "3"): -0.0736,
    ("10", "5"): 0.0018,
    ("10", "10"): -0.0530,
    ("20", "10"): -0.0593,
}

def slot_of(field: str) -> tuple[str, int]:
    """Which ``coefficients`` array and index a derivative field lands in.

    Derived from ``morelli.SLOT_MAP`` -- the same table ``to_morelli`` writes
    through -- rather than hand-maintained here.  An earlier hand-written copy in
    this file got ``cl_beta`` wrong (it is ``cl[0]``, not ``cl[1]``) and had no
    entry at all for the control fields, which are not array-leading: ``cl_da``
    lives in ``clda[0]``, not ``cl[0]``.
    """
    from aero_convert.morelli import SLOT_MAP

    for array, slots in SLOT_MAP.items():
        for index, name in slots:
            if name == field:
                return array, index
    raise KeyError(f"{field!r} is in no SLOT_MAP entry")


# The six rate-derivative columns Tornado's test_14 checks slot by slot.
SLOT_OF = {field: slot_of(field) for field in
           ("cl_p", "cl_r", "cy_p", "cy_r", "cn_p", "cn_r")}

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


def sibling_aid_present() -> bool:
    """Whether the sibling analysis tree and ``aid`` package these tests drive exist.

    Both are read-only inputs outside this repository, so they are absent on any
    clone that does not carry them, and ``solvers`` then raises ``FileNotFoundError``
    from ``_load``/``_aid_on_path``.  A non-SkipTest exception in ``setUpClass`` is
    reported as an ERROR for the WHOLE class, which would make the documented
    ``cd python && python3 -m unittest discover -s tests`` fail for reasons that
    have nothing to do with this change.
    """
    from aero_convert.solvers import aid_src, default_source

    return aid_src().is_dir() and default_source().is_file()


SIBLING_ABSENT = (
    "the sibling aid package and Cessna172 analysis this module drives are absent; "
    "set AID_SRC to the aid 'src' directory and place Analyses/Cessna172.jsonc "
    "beside this repository"
)


@unittest.skipUnless(sibling_aid_present(), SIBLING_ABSENT)
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

    @classmethod
    def avl_run(cls) -> object:
        """AVL, the natively-F-R-D yardstick ``test_14`` checks Tornado against.

        ``aid/axes.py``'s AVL entry is ``{}``, so AVL's coefficients need no
        conversion at all and are already in ``plane/dynamics.py``'s frame.  That
        makes them the one honest reference against which Tornado's frame claim can
        be checked without appealing to our own invariant table -- and ``test_14``
        compares ``provenance.mapping`` PRE-``normalise_sign`` values against it, not
        the shipped arrays, because on the shipped arrays the comparison is
        ``table == table`` and cannot fail.
        """
        if getattr(cls, "_avl", None) is None:
            from aero_convert.solvers import avl_run as _avl_run

            cls._avl = _avl_run()
        return cls._avl

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
        ``cm0`` and the three states are still all recorded.  How many slots each
        solver had to reverse is a fact about the data, so it is read back from
        ``provenance.flipped`` and asserted below rather than written here -- which
        run flips how many is exactly the kind of number a stale sentence gets
        wrong.
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

        # The shift argument is the BODY-z force slope.  Under the sibling's
        # Forward-Right-D boundary ``CZ_a`` already IS that body-z coefficient, so it
        # is read directly -- the pre-F-R-D ``-raw["CZ_a"]`` here was the old frame
        # conversion, and the new ``cz[1] = -CL_a`` slot identity does NOT apply to
        # the shift argument.  CL_a is still the wrong number to shift with, because
        # CL and CZ part company wherever the axial force is non-zero.
        cm_from_cz = shifted(raw["CZ_a"])
        cm_from_cl = shifted(-raw["CL_a"])
        self.assertNotAlmostEqual(
            cm_from_cz, cm_from_cl, places=3,
            msg="CZ_a and CL_a must differ enough for this test to mean anything",
        )
        self.assertAlmostEqual(
            self.coefficients()["cm"][1], cm_from_cz, places=12,
            msg="cm[1] must be the shift of Cm_a with cz = +CZ_a (already body-z)",
        )
        # and the q slope likewise
        self.assertAlmostEqual(
            self.coefficients()["cmq"][0],
            shift_moments_to_cg(
                raw["CZ_Q"], 0.0, 0.0, 0.0, raw["Cm_Q"], 0.0,
                dx=geometry.dx_m, dy=geometry.dy_m, dz=geometry.dz_m,
                s_ref=geometry.s_ref_m2, b_ref=geometry.b_ref_m, c_ref=geometry.c_ref_m,
            )[4],
            places=12,
            msg="cmq[0] must be the shift of Cm_Q with cz = +CZ_Q (already body-z)",
        )

    def test_12_mapping_separates_the_solver_value_from_the_mapped_one(self) -> None:
        """Provenance must show what the solver said AND what was done to it."""
        entries = {entry["field"]: entry for entry in self.solver_run.provenance["mapping"]}
        # (field, frame sign, per-degree?) -- the static fields are already per
        # radian, so only the frame sign separates them from the solver's number;
        # the control rows are per degree and must carry DEG_TO_RAD as well.
        #
        # Under the sibling's Forward-Right-D boundary the frame sign is +1 for
        # EVERY field except the two `cz = -CL` identities, and that is the whole
        # point of the re-base.  `cl_alpha`, `cl0` and `cl_de` keep -1 because their
        # negation is the plane/dynamics.py identity (its body +z axis points down),
        # NOT a frame conversion.  Everything else -- `cl_q`, `cl_beta`, `cy_beta`,
        # `cy_p`, and all six control rows -- is read straight out of coeff_create.
        # This used to assert -1 for nine of these; the frame conversion it was
        # checking for is now the sibling's job, in aid/axes.py.
        for field, frame_sign, per_degree in (
            ("cl_alpha", -1.0, False), ("cl_q", 1.0, False), ("cl_beta", 1.0, False),
            ("cy_p", 1.0, False), ("cy_beta", 1.0, False),
            ("cl_da", 1.0, True), ("cn_da", 1.0, True), ("cy_da", 1.0, True),
            ("cl_de", -1.0, True), ("cl_dr", 1.0, True), ("cy_dr", 1.0, True),
            ("cn_dr", 1.0, True),
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

        # An INDEPENDENT leg, so this is not merely a restatement of the table above:
        # for every field the adapter reads without a shift, the number provenance
        # calls "mapped" must be the number the file actually ships, after the
        # invariant table.  Provenance and the written array cannot drift apart, and
        # a reinstated negation would have to be fixed in both places to pass.
        shipped = self.coefficients()
        for field in ("cl_q", "cl_beta", "cy_beta", "cy_p", "cl_da", "cy_da",
                      "cn_da", "cl_de", "cl_dr", "cy_dr", "cn_dr"):
            with self.subTest(field=field, leg="provenance agrees with the shipped array"):
                array, index = slot_of(field)
                self.assertAlmostEqual(
                    shipped[array][index],
                    normalise_sign(field, entries[field]["mapped_value"])[0],
                    places=12,
                    msg=f"{field}: provenance.mapped_value and the shipped slot disagree",
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

    def test_14_pr_columns_are_already_frd_and_the_shift_is_what_makes_two_of_them_material(self) -> None:
        """The old P/R component-sense "defect" is GONE -- but not uniformly, and
        the old test's reason for calling it immaterial was wrong.

        The previous version of this test asserted that negating Tornado's P and R
        columns end to end changed no written coefficient, i.e. that the
        component-sense inversion was immaterial.  ``aid/tornado/coeff.py:247`` now
        returns ``to_frd("tornado", out)``, so those six columns arrive already
        Forward-Right-Down -- the frame AVL is natively in (``aid/axes.py``'s AVL
        entry is ``{}``) and the frame ``plane/dynamics.py`` flies in.

        Measured, the six slots split into two groups, and the split is the whole
        point:

        * Four are UNSHIFTED and IMMATERIAL under ``normalise_sign``.  The table pins
          their sign from the Morelli invariant, so ``normalise_sign(field, -raw)``
          and ``normalise_sign(field, raw)`` land on the same number.  A negation
          there really is invisible -- and for these four that is fine, because the
          frame claim is not what establishes their sign; the invariant table is.
        * Two are MATERIAL: ``cn_p`` and ``cn_r``.  They are not bare solver values
          but CG-shift outputs, and ``shift_moments_to_cg`` MIXES ``CY_P``/``CY_R``
          into ``Cn`` before ``normalise_sign`` ever runs.  So negating ``Cn_P`` or
          ``Cn_R`` does not merely flip a sign the table can undo -- it changes the
          shifted magnitude, and no table undoes that.  These two ARE worth getting
          right, and the old code negated them before shifting.

        So this test pins the partition itself rather than asserting a blanket
        immateriality that is false for two of the six, and it pins the shipped value
        of the material pair against the shift of the RAW F-R-D column.  If someone
        reinstates a ``-1`` on ``Cn_P``/``Cn_R``, both this and ``test_15``'s shift
        accounting move.
        """
        raw = self.solver_run.raw
        geometry = self.solver_run.geometry
        shipped = self.coefficients()

        def shifted_cn(cy_rate: float, cn_rate: float) -> float:
            """The CG shift's contribution to ``Cn_p``/``Cn_r`` from the yaw rates."""
            from aero_convert.units import shift_moments_to_cg

            return shift_moments_to_cg(
                0.0, 0.0, cy_rate, 0.0, 0.0, cn_rate,
                dx=geometry.dx_m, dy=geometry.dy_m, dz=geometry.dz_m,
                s_ref=geometry.s_ref_m2, b_ref=geometry.b_ref_m,
                c_ref=geometry.c_ref_m,
            )[5]

        unshifted = {"cl_p": "Cl_P", "cl_r": "Cl_R", "cy_p": "CY_P", "cy_r": "CY_R"}
        shifted = {"cn_p": ("CY_P", "Cn_P"), "cn_r": ("CY_R", "Cn_R")}

        # The partition, measured rather than assumed.
        material = set()
        for field, raw_key in unshifted.items():
            _array, _position = SLOT_OF[field]
            value = float(raw[raw_key])
            if normalise_sign(field, -value)[0] != normalise_sign(field, value)[0]:
                material.add(field)
            with self.subTest(field=field, leg="unshifted, table-pinned"):
                self.assertAlmostEqual(
                    shipped[_array][_position], value, places=12,
                    msg=f"{field} is not a CG-shift output; it must be the raw F-R-D value",
                )
        for field, (cy_key, cn_key) in shifted.items():
            array, position = SLOT_OF[field]
            good = shifted_cn(float(raw[cy_key]), float(raw[cn_key]))
            bad = shifted_cn(float(raw[cy_key]), -float(raw[cn_key]))
            if normalise_sign(field, good)[0] != normalise_sign(field, bad)[0]:
                material.add(field)
            with self.subTest(field=field, leg="material: shift consumes the F-R-D Cn"):
                self.assertAlmostEqual(
                    shipped[array][position], normalise_sign(field, good)[0], places=12,
                    msg=f"{field}: the CG shift must consume the un-negated Cn column",
                )
        self.assertEqual(
            material, {"cn_p", "cn_r"},
            "only the two CG-shifted yaw-rate slots are material; if this set changed, "
            "the sign of every other P/R column is table-pinned rather than frame-derived",
        )

        # And the material pair must not reproduce if the negation comes back.
        for field, (cy_key, cn_key) in shifted.items():
            array, position = SLOT_OF[field]
            with self.subTest(field=field, leg="material: negation would be visible"):
                wrong = shifted_cn(float(raw[cy_key]), -float(raw[cn_key]))
                self.assertNotAlmostEqual(
                    shipped[array][position], normalise_sign(field, wrong)[0], places=6,
                    msg=f"shift(-{cn_key}) reproduced the shipped {field}; the negation is back",
                )

        # Cross-solver sign agreement -- on the PRE-table values, which is the only
        # version of this comparison that can fail.
        #
        # The obvious version, shipped-versus-shipped, is ``table == table``:
        # ``normalise_sign`` has already forced both numbers onto the invariant side of
        # zero, so the assertion can only trip on an exact zero or a missing invariant
        # row.  Its failure message used to claim a FRAME error, which it cannot detect.
        # ``provenance.mapping[*]["mapped_value"]`` is the value BEFORE the table runs, so
        # two solvers in one shared frame genuinely have to agree there.
        avl_pre = {e["field"]: e for e in self.avl_run().provenance["mapping"]}
        ours_pre = {e["field"]: e for e in self.solver_run.provenance["mapping"]}
        for field in (*unshifted, *shifted):
            with self.subTest(field=field, leg="pre-table sign agrees with AVL"):
                ours = float(ours_pre[field]["mapped_value"])
                theirs = float(avl_pre[field]["mapped_value"])
                self.assertNotEqual(ours, 0.0)
                self.assertNotEqual(theirs, 0.0)
                self.assertEqual(
                    ours > 0.0, theirs > 0.0,
                    f"{field}: PRE-normalise_sign Tornado {ours:+.6g} vs AVL "
                    f"{theirs:+.6g} disagree in sign.  These are the values before the "
                    "invariant table, so this IS a frame statement",
                )

    def test_15_dropping_the_shift_moves_exactly_the_seven_shifted_slots(self) -> None:
        """The other half of the materiality claim: the SHIFT is not immaterial.

        ``test_14`` shows the P/R column signs are frame-material only where the CG
        shift mixes them.  Discarding the shift altogether is a different hypothesis
        again, and it moves the seven slots the shift touches: ``cm[0]``, ``cm[1]``,
        ``cm[2]``, ``cmq[0]``, ``cn[0]``, ``cnp[0]``, ``cnr[0]``.  Pinning the
        boundary means "the sign question is narrow" can never be read as "the moment
        reference is arbitrary".

        This was the second inherited gap in the re-base: the case table still built
        every shifted argument from the OLD negated columns (``-raw["CZ_a"]``,
        ``-raw["CZ_Q"]``, ``-raw["Cn_b"]``, ``-raw["Cn_P"]``, ``-raw["Cn_R"]``), so it
        asserted a mapping the adapter stopped using and failed against correct code.
        Every argument is now the Forward-Right-D column, read directly.

        The distinction that matters and is easy to lose: the SLOT and the SHIFT
        ARGUMENT are still deliberately different numbers.  ``cz[1]`` ships the
        wind-axis ``-CL_a`` (the ``cz = -CL`` identity derived from the flight model),
        but the shift consumes the body-z ``CZ_a``, because the shift's algebra
        contains a body-z force.  They differ by 0.34 % here.  Likewise ``cz[5]``
        ships ``-CL_de`` while the shift takes the reconstructed body-z
        ``cz_de = -(cos_a * cl_de + sin_a * cd_de)``.
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
            # slot: (field, shifted value the file must ship, same value unshifted)
            "cm[1]": ("cm_alpha", shift(float(raw["CZ_a"]), 0.0, raw["Cm_a"], 0.0)[0],
                      raw["Cm_a"]),
            "cm[0]": ("cm0", shift(-float(raw["CL_at_alpha0"]), 0.0, raw["Cm_at_alpha0"], 0.0)[0],
                      raw["Cm_at_alpha0"]),
            "cmq[0]": ("cm_q", shift(float(raw["CZ_Q"]), 0.0, raw["Cm_Q"], 0.0)[0],
                       raw["Cm_Q"]),
            "cm[2]": ("cm_de", shift(cz_de, 0.0, cm_de, 0.0)[0], cm_de),
            "cn[0]": ("cn_beta", shift(0.0, raw["CY_b"], 0.0, raw["Cn_b"])[1], raw["Cn_b"]),
            "cnp[0]": ("cn_p", shift(0.0, raw["CY_P"], 0.0, raw["Cn_P"])[1], raw["Cn_P"]),
            "cnr[0]": ("cn_r", shift(0.0, raw["CY_R"], 0.0, raw["Cn_R"])[1], raw["Cn_R"]),
        }
        moved = set()
        for slot, (field, mapped, unshifted) in cases.items():
            with self.subTest(slot=slot, claim="ships the shifted value"):
                array, index = slot.split("[")
                index = int(index.rstrip("]"))
                self.assertAlmostEqual(
                    normalise_sign(field, mapped)[0], shipped[array][index], places=12,
                    msg=f"{slot} must be the CG-shifted value the file ships",
                )
            if normalise_sign(field, mapped)[0] != normalise_sign(field, unshifted)[0]:
                moved.add(slot)
        self.assertEqual(
            moved, set(cases),
            "all seven CG-shifted slots must actually move when the shift is dropped",
        )

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


class BuildGuardTest(unittest.TestCase):
    """``build()`` must not be able to overwrite the committed deliverables.

    It writes Tornado only, so with the default out_dir it would replace the
    committed ``geometry.jsonc`` -- the only copy of the Fortran ``for006`` table,
    the AVL per-alpha rows and the flow5 deck and polar -- with a version missing
    every ``raw_cross_checks`` key.  The guard raises instead.  No solver run is
    needed to check that, so this class is independent of the sibling analysis tree.
    """

    def test_build_refuses_the_committed_data_directory(self) -> None:
        import aero_convert.solvers as solvers

        self.assertIn("build", solvers.__all__)
        with self.assertRaises(ValueError):
            solvers.build()
        signature = inspect.signature(solvers.build)
        self.assertIs(
            signature.parameters["out_dir"].default, None,
            "an explicit out_dir must stay the only way in",
        )


@unittest.skipUnless(sibling_aid_present(), SIBLING_ABSENT)
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

    def test_1b_flow5_produces_twenty_one_of_the_twenty_five(self) -> None:
        """flow5's coverage, measured against its 26 emitted keys rather than assumed.

        Its runner now emits the six longitudinal polar channels, the twelve
        ``StabDerivatives`` (``CZa``, ``CXa``, ``CYb``, ``CYp``, ``CYr``, ``Clb``,
        ``Clp``, ``Clr``, ``Cnb``, ``Cnp``, ``Cnr``, ``XNP``), ``CLa``/``Cma``, a
        ``CDvis``/``CDind`` split, the body ``Cx``/``Cz``/``Cl``/``Cn`` point
        channels and the alpha/beta schedules.  So it fills 21 of the 25 slots,
        including every sideslip and every roll/yaw-rate derivative.

        The four it cannot fill are named in ``FLOW5_ABSENT`` with their reasons in
        ``provenance.missing``.  They are ``None`` and DECLARED rather than written
        as zeros, so a slot that is honestly empty can never be read as a
        measurement -- which matters here, because three of them are q-derivatives
        the library computes and the runner declines to forward.
        """
        run = self.runs["flow5"]
        derivatives = run.derivatives
        declared = {entry["field"] for entry in run.provenance["missing"]}
        absent = sorted(f for f in ALL_FIELDS if getattr(derivatives, f) is None)
        produced = sorted(f for f in ALL_FIELDS if getattr(derivatives, f) is not None)
        self.assertEqual(len(produced), 21, f"flow5 produced {produced}")
        self.assertEqual(absent, sorted(FLOW5_ABSENT))
        self.assertEqual(set(absent), declared, "every absent slot must be declared")
        self.assertEqual(
            set(run.provenance["coverage"]["produced"]), set(produced),
            "provenance.coverage must agree with the derivative set it describes",
        )
        self.assertEqual(set(run.provenance["coverage"]["absent"]), set(absent))
        for field in FLOW5_NEWLY_PRODUCED:
            with self.subTest(field=field, claim="newly produced"):
                value = getattr(derivatives, field)
                self.assertIsNotNone(
                    value,
                    f"flow5 emits {field}; a None here means the adapter stopped reading it",
                )
                self.assertTrue(math.isfinite(float(value)))
                self.assertNotEqual(float(value), 0.0)

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

    def test_3_flow5_has_no_pitch_rate_derivatives(self) -> None:
        """flow5 has p- and r-rate derivatives now, but still NO q-derivative.

        Its twelve StabDerivatives include ``CYp``, ``CYr``, ``Clp``, ``Clr``,
        ``Cnp`` and ``Cnr`` -- the roll-rate and yaw-rate columns.  It has no
        pitch-rate column at all: the library computes ``CXq``/``CZq``/``Cmq`` and
        ``FLOW5/run/flow5_run.cpp`` simply does not forward them, which the
        sibling's own record notes.  So the three q-slots stay ``None`` and
        declared, and this pins that they are honestly empty rather than merely
        unwritten -- while the six p/r slots must be real, because a regression
        that lost them would otherwise look identical from coverage alone.
        """
        run = self.runs["flow5"]
        derivatives = run.derivatives
        declared = {e["field"] for e in run.provenance["missing"]}
        for field in ("cl_q", "cm_q", "cd_q"):
            with self.subTest(field=field, claim="absent"):
                self.assertIsNone(getattr(derivatives, field))
                self.assertIn(field, declared)
        for field in ("cy_p", "cy_r", "cl_p", "cl_r", "cn_p", "cn_r"):
            with self.subTest(field=field, claim="present"):
                value = getattr(derivatives, field)
                self.assertIsNotNone(value, f"flow5 emits {field}")
                self.assertNotEqual(float(value), 0.0)

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
        ``cm0`` and the three states are still all recorded.  How many slots each
        solver had to reverse is a fact about the data, so it is read back from
        ``provenance.flipped`` and asserted below rather than written here -- which
        run flips how many is exactly the kind of number a stale sentence gets
        wrong.
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
            ("flow5", (("cl_da", "aileron", "Cl"), ("cn_da", "aileron", "Cn"),
                        ("cl_dr", "rudder", "Cl"), ("cn_dr", "rudder", "Cn"),
                        ("cy_dr", "rudder", "CY"), ("cl_de", "elevator", "CL"),
                        ("cm_de", "elevator", "Cm"))),
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

    def test_13b_control_slots_cannot_detect_a_frame_error_so_dont_pretend_they_can(self) -> None:
        """A control slot's SHIPPED sign is pinned by the table, not by the frame.

        This started life as "flow5's control signs must match AVL's", on the theory
        that AVL -- whose ``aid/axes.py`` entry is ``{}``, so natively Forward-Right-Down
        -- would catch a double ``to_frd`` on flow5's rows.  It does not, and the
        injection test is why: negating flow5's ``Cl``, ``Cn`` and ``CL`` control rows
        a second time leaves every shipped coefficient IDENTICAL.

        The reason is structural, and it is the same masking that made the old
        ``test_14`` wrong.  Negating a control row preserves its magnitude, and
        ``normalise_sign`` then pins the sign from the Morelli invariant -- so the
        shipped file cannot distinguish the two.  A sign-only assertion here is
        therefore vacuous, which is precisely the I4 defect: a test that cannot fail
        is worse than no test, because it reads like coverage.

        So this asserts the finding instead of hiding it, and it makes the limit
        explicit for whoever writes the next frame guard:

        1. For every control slot, ``normalise_sign`` provably returns the same value
           for the row and for its negation -- the masking is demonstrated, not
           assumed.
        2. Therefore the ONLY places a Tornado/flow5 frame error is detectable in the
           shipped file are where the CG shift MIXES columns and changes a magnitude,
           which ``test_14`` shows is ``cn_p`` and ``cn_r``.
        3. The AVL sign agreement is still asserted -- it is a genuine cross-solver
           consistency check on the invariant table itself, and Task 5 wants it -- but
           it is labelled as that, not as a frame guard.
        """
        masked = []
        rows = self.runs["flow5"].provenance["control_rows"]
        for field in ("cl_da", "cn_da", "cl_dr", "cy_dr", "cn_dr", "cl_de", "cm_de"):
            entries = {e["field"]: e for e in self.runs["flow5"].provenance["mapping"]}
            mapped = float(entries[field]["mapped_value"])
            with self.subTest(field=field):
                self.assertNotEqual(mapped, 0.0)
                if normalise_sign(field, -mapped)[0] == normalise_sign(field, mapped)[0]:
                    masked.append(field)
                else:
                    self.fail(
                        f"{field} is frame-sensitive after all: normalise_sign does not "
                        "pin its sign, so this test's premise is wrong and the slot needs "
                        "its own frame guard",
                    )
        self.assertEqual(
            masked, ["cl_da", "cn_da", "cl_dr", "cy_dr", "cn_dr", "cl_de", "cm_de"],
            "every flow5 control slot is expected to be sign-pinned by the table; if this "
            "set shrinks, a control slot has become frame-sensitive and needs its own guard",
        )

        # (3) WHY the table being the final authority matters here, stated as a measured
        # fact rather than as a cross-solver check that cannot fail.  The obvious
        # comparison -- ``derivatives.<field>`` against ``coefficients("avl")`` -- is
        # ``table == table`` and can only trip on an exact zero; an earlier version of this
        # test called it "a genuine cross-solver consistency check", which is true and
        # also nearly content-free.  So this asserts the opposite of agreement: the solvers
        # really do disagree in sign BEFORE the table runs, on control derivatives, which is
        # why ``normalise_sign`` -- not a cross-solver vote -- is what settles the sign.
        pre = {
            model: {e["field"]: e["mapped_value"]
                    for e in self.runs[model].provenance["mapping"]}
            for model in ("flow5", "datcom", "avl")
        }
        disagreements = {}
        for field in ("cl_da", "cl_de", "cm_de", "cl_dr", "cy_dr", "cn_dr",
                      "cy_da", "cn_da"):
            avl_value = pre["avl"].get(field)
            if avl_value is None:
                continue
            for model in ("flow5", "datcom"):
                value = pre[model].get(field)
                if value is None or value == 0.0 or avl_value == 0.0:
                    continue
                if (value > 0.0) != (avl_value > 0.0):
                    disagreements.setdefault(field, []).append(model)
        self.assertTrue(
            disagreements,
            "the solvers are expected to disagree in sign before normalise_sign runs on "
            "the control derivatives.  If they no longer do, the table is doing less work "
            "than the headers say and item 6 needs rewriting",
        )
        for field, models in disagreements.items():
            with self.subTest(field=field, note="pre-table sign disagreement, table settles it"):
                self.assertTrue(models)
                for model in models:
                    with self.subTest(model=model):
                        self.assertNotEqual(pre[model][field], 0.0)

    def test_17_every_written_file_records_the_sibling_commit_it_was_built_from(self) -> None:
        """Part 7: ``aid_src_commit`` in all five files, checked against real git.

        This project has been broken by an upstream commit TWICE, both times
        silently, both times found only by a failing test.  So every generated file
        records the sibling's HEAD, and the expectation here is INDEPENDENT: this test
        shells out to ``git rev-parse HEAD`` in the sibling itself rather than calling
        the adapter's own ``_aid_commit()``.  A test that asserted
        ``provenance["aid_src_commit"] == _aid_commit()`` would pass even if
        ``_aid_commit`` were broken, which is the same blindness as I4.

        The sibling is only ever READ.  This test does not write to it, does not
        vendor it, and adds no requirements entry for it.
        """
        import json
        import subprocess
        import tempfile
        from pathlib import Path as _Path

        from aero_convert.solvers import aid_src, write_model, write_all

        sibling = aid_src()
        head = subprocess.run(
            ["git", "-C", str(sibling), "rev-parse", "HEAD"],
            capture_output=True, text=True, check=True,
        ).stdout.strip()
        self.assertEqual(len(head), 40, f"not a full SHA: {head!r}")
        self.assertEqual(
            subprocess.run(["git", "-C", str(sibling), "status", "--porcelain"],
                           capture_output=True, text=True, check=True).stdout, "",
            "the sibling must be clean; a dirty tree would make the commit a lie",
        )

        with tempfile.TemporaryDirectory() as tmp:
            out = _Path(tmp)
            write_all(tornado_run(), out)
            for model in ("datcom", "avl", "flow5"):
                write_model(self.runs[model], out)

            def provenance_of(name: str) -> dict:
                text = (out / name).read_text()
                body = "\n".join(
                    line for line in text.splitlines()
                    if not line.lstrip().startswith("//")
                )
                return json.loads(body)["provenance"]

            for name in ("tornado.jsonc", "geometry.jsonc", "datcom.jsonc",
                         "avl.jsonc", "flow5.jsonc"):
                with self.subTest(file=name):
                    recorded = provenance_of(name).get("aid_src_commit")
                    self.assertEqual(
                        recorded, head,
                        f"{name} does not record the sibling commit it was built from; "
                        "an upstream move would then be a mystery instead of a one-line diff",
                    )

    def test_18_hand_written_counts_in_the_headers_must_match_the_data(self) -> None:
        """The durable guard for this defect class: prose that rots when the data moves.

        Three rounds running, the same finding kept arriving -- a header sentence
        asserting a count, a slot name, a key set or a capability, true when written and
        false after the computation changed.  From this round alone: item 6 printed a
        2-element flipped list while ``provenance.flipped`` held seven; it claimed
        ``czq[0]`` held ``CL_q`` in the same file that shipped ``CZ_Q``; it said "TWO
        negations survive" while the same file's item 5 admitted a third; the module
        docstring called a 0.249 % move "nothing measurable"; and this test module's own
        docstring still described flow5's retired 7-of-25 runner.

        The generator fix was to derive those numbers.  This test is what stops the next
        one rotting: it PARSES the rendered header back out of the freshly built files and
        checks each claim against the run's own data.  It does not re-derive the prose --
        it checks the prose against ``len()``.

        Cheap on purpose.  Three mechanical claims, each against a GENERATED artifact
        this module's own build writes:

        1. the flipped-list literal in item 6 equals ``provenance.flipped``;
        2. the identity-negation lines name exactly ``identity_negated(run)``;
        3. the "N OF THE m SLOTS ARE ABSENT" count equals ``len(provenance["missing"])``
           and ``m`` equals the real schema size.

        It used to carry a fourth claim, over the coverage table in this module's own
        docstring.  That one asserted over implementation source rather than over a
        generated artifact, so a behaviour-preserving reword of the prose broke it and
        it proved nothing about the adapters; the coverage claim each generated file
        already makes in its own header is the one that belongs here.
        """
        import re
        import tempfile
        from pathlib import Path as _Path

        from aero_convert import solvers
        from aero_convert.solvers import build_all, identity_negated, tornado_run

        runs = {"tornado": tornado_run(), "datcom": self.runs["datcom"],
                "avl": self.runs["avl"], "flow5": self.runs["flow5"]}

        with tempfile.TemporaryDirectory() as tmp:
            build_all(_Path(tmp))
            for model, run in runs.items():
                name = "tornado.jsonc" if model == "tornado" else f"{model}.jsonc"
                header = "\n".join(
                    line for line in (_Path(tmp) / name).read_text().splitlines()
                    if line.lstrip().startswith("//")
                )

                # (1) the WHOLE flipped list, in whichever of the two formats the
                # header uses.  Tornado's item 6 prints it as a list literal; the
                # cross-check files print it in item 7 as a comma-separated run.
                with self.subTest(model=model, claim="header prints ALL of provenance.flipped"):
                    expected = list(run.provenance["flipped"])
                    literal = re.search(
                        r"it is ALL of them:\s*\n//\s+(\[.*?\]|\(none\))", header)
                    inline = re.search(
                        r"the other side of zero: ([a-z_]+(?:, [a-z_]+)*)", header)
                    self.assertTrue(
                        literal or inline,
                        f"{name}: the header prints no flipped list at all",
                    )
                    if literal:
                        printed_list = (
                            [] if literal.group(1) == "(none)"
                            else json.loads(literal.group(1).replace("'", '"'))
                        )
                    else:
                        printed_list = inline.group(1).split(", ")
                    self.assertEqual(
                        printed_list, expected,
                        f"{name}: the header prints {printed_list}, "
                        f"provenance.flipped is {expected}",
                    )

                # (2) every identity-negated slot, one per line
                with self.subTest(model=model, claim="item 6 names every identity slot"):
                    expected = {item["slot"] for item in identity_negated(run)}
                    named_slots = set(re.findall(
                        r"^//\s+([a-z]+\[\d+\]) from ", header, re.M))
                    self.assertEqual(
                        named_slots, expected,
                        f"{name}: item 6 lists {sorted(named_slots)} as the "
                        f"identity negations, provenance.mapping says {sorted(expected)}",
                    )

                # (2b) The SAME fact by a second, independent path.  Claims (1)-(2) compare
                # the header against the generator, so a bug INSIDE the generator's
                # `identity_negated` would move both sides together and pass -- I checked,
                # by dropping `cl0` from the predicate, and the guard stayed green.  So
                # recompute the negated set straight from item 7's rendered slot table,
                # which prints solver/mapped/written per slot: a slot is identity-negated
                # when mapped == -solver and the table did not then flip it.  Two paths to
                # one answer, so one being wrong is now a failure.
                with self.subTest(model=model, claim="identity set agrees with item 7"):
                    rows = re.findall(
                        r"^//\s+(\S+\[\d+\])\s+\S+\s+\S+\s+solver .*? = "
                        r"(-?[\d.eE+-]+) -> mapped (-?[\d.eE+-]+) -> written (-?[\d.eE+-]+)",
                        header, re.M,
                    )
                    self.assertTrue(rows, f"{name}: item 7's slot table did not parse")
                    from_table = set()
                    for slot, solver_v, mapped_v, written_v in rows:
                        solver_f, mapped_f = float(solver_v), float(mapped_v)
                        if solver_f == 0.0:
                            continue
                        # Either the bare identity, or the identity plus DEG_TO_RAD for a
                        # per-degree control row -- the second form was missing at first and
                        # silently dropped every `cz[5]` from the comparison.
                        # Item 7 prints six significant figures, so the DEG_TO_RAD form
                        # only matches to ~5e-5 relative.  2e-4 is comfortably inside that
                        # and still four orders of magnitude clear of the ~57x separation
                        # between the two candidate scalings.
                        negated = (
                            abs(mapped_f + solver_f) <= 2e-4 * max(1.0, abs(solver_f))
                            or abs(mapped_f + solver_f * DEG_TO_RAD)
                            <= 2e-4 * max(1.0, abs(solver_f * DEG_TO_RAD))
                        )
                        # `written == mapped` means the invariant table did NOT then flip it,
                        # which is what separates an identity from a table flip.
                        if negated and written_v == mapped_v:
                            from_table.add(slot)
                    self.assertEqual(
                        from_table, expected,
                        f"{name}: item 7 implies the identity-negated set is "
                        f"{sorted(from_table)}, item 6 says {sorted(expected)}",
                    )

                # (3) the absent-slot count and the schema total
                with self.subTest(model=model, claim="absent count matches provenance.missing"):
                    found = re.search(r"(\d+) OF THE (\d+) SLOTS ARE ABSENT", header)
                    if found is None:
                        continue  # a model that fills every slot states no absent count
                    self.assertEqual(int(found.group(1)), len(run.provenance["missing"]))
                    self.assertEqual(
                        int(found.group(2)), len(solvers.ALL_DERIVATIVE_FIELDS),
                        "the 'OF THE n SLOTS' total must be the real schema size",
                    )

    def test_16_flow5_mesh_is_pinned_because_clb_flips_sign_at_ten_by_five(self) -> None:
        """FLOW5_MESH must stay ("10","10"): flow5's Clb changes SIGN across meshes.

        Part 4 of the fix brief.  The sibling records flow5's ``Clb`` as strongly
        mesh-dependent -- -0.0736 at ("5","3"), **+0.0018** at ("10","5"),
        -0.0530 at ("10","10"), -0.0593 at ("20","10").  The positive, 30x-small
        value at ("10","5") is not a convergence wobble, it reads as a silent sign
        inversion, and a sideslip derivative that flips sign with a mesh constant
        would sail through a sign-only invariant table while being badly wrong.

        So the mesh is pinned by a test.  Three assertions, in increasing strength:
        the constant itself; that the shipped Clb carries the same sideslip sign as
        AVL's and DATCOM's; and that it is nowhere near the ("10","5") trap, i.e.
        that the pin would actually notice if someone changed the mesh to the bad
        one.
        """
        from aero_convert.solvers import FLOW5_MESH

        self.assertEqual(
            FLOW5_MESH, ("10", "10"),
            "flow5's Clb is mesh-dependent and inverts at ('10','5'); the mesh is a "
            "physics constant now, not a formatting choice",
        )
        array, index = slot_of("cl_beta")
        clb = float(self.coefficients("flow5")[array][index])
        self.assertAlmostEqual(clb, FLOW5_CLB_BY_MESH[("10", "10")], places=3)
        # In family with the other solvers.  Tornado is checked in its own test
        # class; here the yardstick is AVL, which needs no conversion at all.
        for model in ("avl", "datcom"):
            with self.subTest(model=model, comparison="sideslip stability sign"):
                other = float(self.coefficients(model)[array][index])
                self.assertEqual(other > 0.0, clb > 0.0,
                                 f"flow5 Clb={clb:+.6g} vs {model} {other:+.6g}")
        # And emphatically not the inverted mesh's value.
        trap = FLOW5_CLB_BY_MESH[("10", "5")]
        self.assertGreater(trap, 0.0, "the recorded bad-mesh Clb must be positive")
        self.assertLess(clb, 0.0)
        self.assertGreater(
            abs(clb - trap), 0.02,
            f"flow5 Clb={clb:+.6g} is too close to the ('10','5') trap {trap:+.6g}; "
            "if the mesh changed, this must fail loudly",
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

        ``aid/lateral.py:570`` multiplies the DATCOM tail lift slope by
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
            f"aid/lateral.py:570's uncorrected Clb ({aid_clb}) should NOT agree with AVL "
            f"({avl_clb}); if it now does, the units correction has been undone",
        )


if __name__ == "__main__":
    unittest.main()