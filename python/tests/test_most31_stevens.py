"""Stevens tables and gains translated into MOST31 match the AeroBench oracle."""
from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from f16.aero_data import load_aero_coefficients
from f16.aero_stevens import stevens_coefficients
from f16.units import B_M, CBAR_M
from most31.evaluate import evaluate
from most31.schema import (
    ANGLE_DEG_PER_RAD,
    MULTIPLIERS,
    OUTPUTS,
    Most31Coefficients,
)
from most31.translate_stevens import translate_stevens
from tests.most31_cases import ALPHA_DEG, BETA_DEG, assert_close

V = 150.0
REL = 1e-12
XCG_REF = 0.35
_RATES = (0.4, -0.3, 0.2)
_TABLES = ("cx", "cz", "cl", "cm", "cn", "dlda", "dldr", "dnda", "dndr", "damp")
_GAINS = ("cy_beta", "cy_aileron", "cy_rudder", "cz_elevator")
_NORMS = (
    "aileron_norm_deg",
    "rudder_norm_deg",
    "beta_norm_deg",
    "elevator_norm_deg",
)

CY = OUTPUTS.index("Cy")
CZ = OUTPUTS.index("Cz")
CL = OUTPUTS.index("CL")
CD = OUTPUTS.index("CD")
CY_W = OUTPUTS.index("CY")
M1 = MULTIPLIERS.index("1")
DA = MULTIPLIERS.index("da")
DR = MULTIPLIERS.index("dr")
DC = MULTIPLIERS.index("dc")


def _rad(deg: float) -> float:
    return deg / ANGLE_DEG_PER_RAD


def _control_sets() -> tuple[tuple[float, float, float, float, float, float], ...]:
    """Task 3 sets, plus elevator ±26° past the ±24° Stevens grid."""
    return (
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (_rad(25.0), _rad(21.5), _rad(30.0), *_RATES),
        (_rad(-25.0), _rad(-21.5), _rad(-30.0), *_RATES),
        (_rad(6.0), _rad(-4.0), _rad(9.0), -0.15, 0.25, -0.05),
        (_rad(26.0), _rad(10.0), _rad(-12.0), *_RATES),
        (_rad(-26.0), _rad(-8.0), _rad(15.0), *_RATES),
    )


def _eval(c: Most31Coefficients, alpha: float, beta: float, de: float, da: float, dr: float, p: float, q: float, r: float, xcg: float):
    return evaluate(
        c, alpha, beta, de, da, dr, 0.0, p, q, r, V, B_M, CBAR_M, xcg, XCG_REF
    )


def _assert_grid(tc: unittest.TestCase, translated: Most31Coefficients, xcg: float) -> None:
    for alpha_deg in ALPHA_DEG:
        alpha = _rad(alpha_deg)
        for beta_deg in BETA_DEG:
            beta = _rad(beta_deg)
            for de, da, dr, p, q, r in _control_sets():
                got = _eval(translated, alpha, beta, de, da, dr, p, q, r, xcg)
                ref = stevens_coefficients(
                    alpha, beta, de, da, dr, p, q, r, CBAR_M, B_M, V, xcg, XCG_REF
                )
                assert_close(
                    tc,
                    got,
                    ref,
                    REL,
                    f"a={alpha_deg} b={beta_deg} de={de} xcg={xcg}",
                )


class TestMost31Stevens(unittest.TestCase):
    def test_shipped_stevens_matches_oracle(self) -> None:
        coeff = load_aero_coefficients("stevens")
        translated = translate_stevens(coeff, "stevens")
        _assert_grid(self, translated, XCG_REF)

    def test_off_reference_cg_matches_oracle(self) -> None:
        coeff = load_aero_coefficients("stevens")
        translated = translate_stevens(coeff, "stevens")
        _assert_grid(self, translated, 0.30)

    def test_random_tables_match_oracle(self) -> None:
        original = load_aero_coefficients("stevens")
        coeff = copy.deepcopy(original)
        rng = np.random.default_rng(4)
        for name in _TABLES:
            shape = np.asarray(coeff[name], dtype=np.float64).shape
            coeff[name] = rng.uniform(-1.0, 1.0, size=shape).tolist()
        for name in _GAINS:
            coeff[name] = float(rng.uniform(-1.0, 1.0))
        for name in _NORMS:
            self.assertEqual(coeff[name], original[name])
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "stevens.json"
            path.write_text(json.dumps({"coefficients": coeff}))
            translated = translate_stevens(coeff, "random-stevens")
            for alpha_deg in ALPHA_DEG:
                alpha = _rad(alpha_deg)
                for beta_deg in BETA_DEG:
                    beta = _rad(beta_deg)
                    for de, da, dr, p, q, r in _control_sets():
                        got = _eval(translated, alpha, beta, de, da, dr, p, q, r, XCG_REF)
                        ref = stevens_coefficients(
                            alpha,
                            beta,
                            de,
                            da,
                            dr,
                            p,
                            q,
                            r,
                            CBAR_M,
                            B_M,
                            V,
                            XCG_REF,
                            XCG_REF,
                            root=Path(tmp),
                        )
                        assert_close(
                            self,
                            got,
                            ref,
                            REL,
                            f"random a={alpha_deg} b={beta_deg}",
                        )

    def test_beta_zero_kills_abs_beta_tables(self) -> None:
        coeff = load_aero_coefficients("stevens")
        translated = translate_stevens(coeff, "stevens")
        self.assertTrue(np.any(translated.table_aabsb != 0.0))
        killed = Most31Coefficients(
            source=translated.source,
            grids=translated.grids,
            poly=translated.poly,
            table_a=translated.table_a,
            table_ae=translated.table_ae,
            table_ab=translated.table_ab,
            table_aabsb=np.ones_like(translated.table_aabsb),
        )
        for alpha_deg in ALPHA_DEG:
            alpha = _rad(alpha_deg)
            for de, da, dr, p, q, r in _control_sets():
                got = _eval(translated, alpha, 0.0, de, da, dr, p, q, r, XCG_REF)
                punched = _eval(killed, alpha, 0.0, de, da, dr, p, q, r, XCG_REF)
                assert_close(self, punched, got, REL, f"aabsb beta=0 a={alpha_deg}")
                ref = stevens_coefficients(
                    alpha, 0.0, de, da, dr, p, q, r, CBAR_M, B_M, V, XCG_REF, XCG_REF
                )
                assert_close(self, got, ref, REL, f"oracle beta=0 a={alpha_deg}")

    def test_unused_slots_are_zero(self) -> None:
        translated = translate_stevens(load_aero_coefficients("stevens"), "stevens")
        for out in (CL, CD, CY_W):
            self.assertTrue(np.all(translated.poly[out] == 0.0), OUTPUTS[out])
            self.assertTrue(np.all(translated.table_a[out] == 0.0), OUTPUTS[out])
            self.assertTrue(np.all(translated.table_ae[out] == 0.0), OUTPUTS[out])
            self.assertTrue(np.all(translated.table_ab[out] == 0.0), OUTPUTS[out])
            self.assertTrue(np.all(translated.table_aabsb[out] == 0.0), OUTPUTS[out])
        self.assertTrue(np.all(translated.poly[:, DC] == 0.0))
        self.assertTrue(np.all(translated.table_a[:, DC] == 0.0))
        self.assertTrue(np.all(translated.table_ae[:, DC] == 0.0))
        self.assertTrue(np.all(translated.table_ab[:, DC] == 0.0))
        self.assertTrue(np.all(translated.table_aabsb[:, DC] == 0.0))
        poly = np.array(translated.poly, copy=True)
        poly[CY, M1, 0, 0, 1, 0] = 0.0
        poly[CY, DA, 0, 0, 0, 0] = 0.0
        poly[CY, DR, 0, 0, 0, 0] = 0.0
        poly[CZ, M1, 0, 0, 0, 1] = 0.0
        self.assertTrue(np.all(poly == 0.0))
