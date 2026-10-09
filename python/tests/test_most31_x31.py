"""X-31 Figure 2.2 table translated into MOST31 coefficients."""
from __future__ import annotations

import unittest
from itertools import product

import numpy as np
import x31_numpy_compat  # noqa: F401  restores numpy short trig aliases before x31
from x31.aero import body_force_moment
from x31.params import fig22, physical
from x31.types import SurfaceCommand

from most31.evaluate import evaluate, nine_outputs
from most31.schema import MULTIPLIERS, OUTPUTS
from most31.translate_x31 import translate_x31
from tests.most31_cases import ALPHA_DEG, BETA_DEG, assert_close

REL = 1e-12
V = 100.0
RHO = 1.23
RATES = (0.3, -0.2, 0.15)
AILERON_DEG = (-30.0, 0.0, 12.0, 30.0)
RUDDER_DEG = (-30.0, 0.0, 12.0, 30.0)
CANARD_DEG = (-90.0, -20.0, 0.0, 30.0)

_GEOM = physical()
SREF = float(_GEOM["Sref"])
B = float(_GEOM["b"])
CBAR = float(_GEOM["Cbar"])
QBAR = 0.5 * RHO * V * V
_MOMENT_SCALE = QBAR * SREF * np.array([B, CBAR, B], dtype=np.float64)

CL = OUTPUTS.index("CL")
M1 = MULTIPLIERS.index("1")


def _port_coefficients(alpha_deg, beta_deg, aileron, rudder, canard, table):
    force, moment = body_force_moment(
        V,
        alpha_deg,
        beta_deg,
        RATES,
        SurfaceCommand(aileron, rudder, canard, 0.0, 0.0, 0.0, 0.0),
        RHO,
        table=table,
    )
    forces = np.asarray(force, dtype=np.float64) / (QBAR * SREF)
    moments = np.asarray(moment, dtype=np.float64) / _MOMENT_SCALE
    return np.concatenate((forces, moments))


def _most31_coefficients(translated, alpha_deg, beta_deg, aileron, rudder, canard):
    return np.asarray(
        evaluate(
            translated,
            np.deg2rad(alpha_deg),
            np.deg2rad(beta_deg),
            0.0,
            np.deg2rad(aileron),
            np.deg2rad(rudder),
            np.deg2rad(canard),
            *RATES,
            V,
            B,
            CBAR,
            0.0,
            0.0,
        ),
        dtype=np.float64,
    )


def _assert_grid(tc, translated, table):
    for alpha_deg, beta_deg, aileron, rudder, canard in product(
        ALPHA_DEG, BETA_DEG, AILERON_DEG, RUDDER_DEG, CANARD_DEG
    ):
        got = _most31_coefficients(translated, alpha_deg, beta_deg, aileron, rudder, canard)
        ref = _port_coefficients(alpha_deg, beta_deg, aileron, rudder, canard, table)
        assert_close(
            tc,
            got,
            ref,
            REL,
            msg=(
                f"alpha={alpha_deg} beta={beta_deg} "
                f"aileron={aileron} rudder={rudder} canard={canard}"
            ),
        )


def _random_table() -> np.ndarray:
    """Seeded Figure 2.2 stand-in. Power i uses U(-1, 1) * 40**(-i); flags are 0 or 1."""
    rng = np.random.default_rng(5)
    table = np.empty((10, 2, 8), dtype=np.float64)
    for power in range(1, 7):
        table[:, :, power - 1] = rng.uniform(-1.0, 1.0, size=(10, 2)) * 40.0 ** (-power)
    table[:, :, 6] = rng.uniform(-1.0, 1.0, size=(10, 2))
    table[:, :, 7] = rng.integers(0, 2, size=(10, 2))
    return table


class TestMost31X31(unittest.TestCase):
    def test_shipped_fig22_matches_port(self) -> None:
        _assert_grid(self, translate_x31(fig22(), "x31.params.fig22"), None)

    def test_random_table_matches_port(self) -> None:
        table = _random_table()
        _assert_grid(self, translate_x31(table, "random fig22"), table)

    def test_odd_flag_negative_alpha(self) -> None:
        row = np.array(
            [0.02, -1.5e-3, 4.0e-5, -6.0e-7, 3.0e-9, -8.0e-12, 0.4, 1.0],
            dtype=np.float64,
        )
        table = np.zeros((10, 2, 8), dtype=np.float64)
        table[0, 0] = row
        alpha_deg = -22.5
        absolute = abs(alpha_deg)
        even = (
            row[0] * absolute
            + row[1] * absolute**2
            + row[2] * absolute**3
            + row[3] * absolute**4
            + row[4] * absolute**5
            + row[5] * absolute**6
            + row[6]
        )
        expected = -even + 2.0 * row[6]
        outputs = nine_outputs(
            translate_x31(table, "odd CL"),
            np.deg2rad(alpha_deg),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            V,
            B,
            CBAR,
        )
        assert_close(self, outputs[CL], expected, REL, msg="odd CL at negative alpha")

    def test_canard_product_reaches_degree_twelve(self) -> None:
        translated = translate_x31(fig22(), "x31.params.fig22")
        self.assertTrue(np.any(translated.poly[CL, M1, :, 12, 0, 0] != 0.0))

    def test_unused_slots_are_zero(self) -> None:
        translated = translate_x31(fig22(), "x31.params.fig22")
        for name in ("Cx", "Cy", "Cz"):
            self.assertTrue(np.all(translated.poly[OUTPUTS.index(name)] == 0.0), name)
        for name in ("table_a", "table_ae", "table_ab", "table_aabsb"):
            self.assertTrue(np.all(getattr(translated, name) == 0.0), name)
        self.assertTrue(np.all(translated.poly[:, :, :, :, 2:, :] == 0.0))
        self.assertTrue(np.all(translated.poly[:, :, :, :, :, 1:] == 0.0))

    def test_bad_flag_raises(self) -> None:
        table = fig22()
        table[0, 0, 7] = 2.0
        with self.assertRaises(ValueError):
            translate_x31(table, "bad flag")
