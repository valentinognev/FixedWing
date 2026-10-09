"""Morelli coefficients translated into MOST31 match the Morelli oracle."""
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from f16.aero_data import load_aero_coefficients
from f16.aero_morelli import morelli_coefficients
from f16.units import B_M, CBAR_M, aerobench_poly_rad, deg_to_rad
from most31.evaluate import evaluate
from most31.schema import MULTIPLIERS, OUTPUTS
from most31.translate_morelli import translate_morelli
from numpy.random import default_rng
from plane.groups import MORELLI_LENGTHS
from tests.most31_cases import ALPHA_DEG, BETA_DEG, assert_close

_REL = 1e-12
_V = 150.0
_XCG_REF = 0.35
# Zeros; both surface limits with the brief's rates; one interior set.
_CONTROL_SETS = (
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (deg_to_rad(25.0), deg_to_rad(21.5), deg_to_rad(30.0), 0.4, -0.3, 0.2),
    (deg_to_rad(-25.0), deg_to_rad(-21.5), deg_to_rad(-30.0), 0.4, -0.3, 0.2),
    (deg_to_rad(10.0), deg_to_rad(-5.0), deg_to_rad(12.0), 0.15, 0.08, -0.11),
)
_REDUCED_ALPHA_DEG = (-30, 0, 45, 85)
_REDUCED_BETA_DEG = (-35, 0, 4)


class TestMost31Morelli(unittest.TestCase):
    def test_shipped_morelli_matches_oracle(self) -> None:
        coeff = load_aero_coefficients("morelli")
        translated = translate_morelli(coeff, "data/planes/f16/morelli.json")
        self._assert_grid(translated, ALPHA_DEG, BETA_DEG, _XCG_REF, _XCG_REF, root=None)

    def test_off_reference_cg_matches_oracle(self) -> None:
        coeff = load_aero_coefficients("morelli")
        translated = translate_morelli(coeff, "data/planes/f16/morelli.json")
        self._assert_grid(
            translated, _REDUCED_ALPHA_DEG, _REDUCED_BETA_DEG, 0.30, _XCG_REF, root=None
        )

    def test_random_coefficients_match_oracle(self) -> None:
        rng = default_rng(3)
        coefficients = {
            name: rng.uniform(-1.0, 1.0, size=length).tolist()
            for name, length in MORELLI_LENGTHS.items()
        }
        self.assertEqual(tuple(coefficients), tuple(MORELLI_LENGTHS))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "morelli.json").write_text(
                json.dumps({"coefficients": coefficients})
            )
            loaded = load_aero_coefficients("morelli", root=root)
            translated = translate_morelli(loaded, "random morelli")
            self._assert_grid(translated, ALPHA_DEG, BETA_DEG, _XCG_REF, _XCG_REF, root=root)

    def test_unused_slots_are_zero(self) -> None:
        coeff = load_aero_coefficients("morelli")
        translated = translate_morelli(coeff, "data/planes/f16/morelli.json")
        for name in ("CL", "CD", "CY"):
            output = OUTPUTS.index(name)
            self.assertTrue(bool((translated.poly[output] == 0.0).all()), name)
        dc = MULTIPLIERS.index("dc")
        self.assertTrue(bool((translated.poly[:, dc] == 0.0).all()))
        self.assertTrue(bool((translated.poly[:, :, :, 6:] == 0.0).all()))
        for name in ("table_a", "table_ae", "table_ab", "table_aabsb"):
            table = getattr(translated, name)
            self.assertTrue(bool((table == 0.0).all()), name)

    def _assert_grid(self, translated, alphas, betas, xcg, xcgref, root) -> None:
        for alpha_deg in alphas:
            alpha = deg_to_rad(float(alpha_deg))
            for beta_deg in betas:
                beta = deg_to_rad(float(beta_deg))
                for de, da, dr, p, q, r in _CONTROL_SETS:
                    got = evaluate(
                        translated,
                        alpha,
                        beta,
                        de,
                        da,
                        dr,
                        0.0,
                        p,
                        q,
                        r,
                        _V,
                        B_M,
                        CBAR_M,
                        xcg,
                        xcgref,
                    )
                    ref = morelli_coefficients(
                        aerobench_poly_rad(alpha),
                        aerobench_poly_rad(beta),
                        de,
                        da,
                        dr,
                        p,
                        q,
                        r,
                        CBAR_M,
                        B_M,
                        _V,
                        xcg,
                        xcgref,
                        root=root,
                    )
                    assert_close(
                        self,
                        got,
                        ref,
                        _REL,
                        msg=(
                            f"a={alpha_deg} b={beta_deg} de={de} da={da} dr={dr} "
                            f"pqr=({p},{q},{r}) xcg={xcg}"
                        ),
                    )
