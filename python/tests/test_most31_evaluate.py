"""MOST31 polynomial, table, CG-shift, and wind-rotation evaluator."""
from __future__ import annotations

import unittest

import numpy as np
import x31_numpy_compat  # noqa: F401  (before x31.quaternion)
from x31.quaternion import body_321_to_q, rotate_body_to_earth

from most31.evaluate import evaluate, nine_outputs
from most31.schema import ANGLE_DEG_PER_RAD, MULTIPLIERS, OUTPUTS, zeros
from tests.most31_cases import ALPHA_DEG, BETA_DEG, assert_close

B = 9.144
CBAR = 3.45
V = 150.0
REL = 1e-12

CX = OUTPUTS.index("Cx")
CY = OUTPUTS.index("Cy")
CZ = OUTPUTS.index("Cz")
CL_ROLL = OUTPUTS.index("Cl")
CM = OUTPUTS.index("Cm")
CN = OUTPUTS.index("Cn")
CL = OUTPUTS.index("CL")
CD = OUTPUTS.index("CD")
CY_W = OUTPUTS.index("CY")
M1 = MULTIPLIERS.index("1")


def _rad(deg: float) -> float:
    return deg / ANGLE_DEG_PER_RAD


def _nine(
    c,
    alpha=0.0,
    beta=0.0,
    de=0.0,
    da=0.0,
    dr=0.0,
    dc=0.0,
    p=0.0,
    q=0.0,
    r=0.0,
    v=V,
):
    return nine_outputs(c, alpha, beta, de, da, dr, dc, p, q, r, v, B, CBAR)


def _eval(
    c,
    alpha=0.0,
    beta=0.0,
    de=0.0,
    da=0.0,
    dr=0.0,
    dc=0.0,
    p=0.0,
    q=0.0,
    r=0.0,
    v=V,
    xcg=0.35,
    xcgref=0.35,
):
    return evaluate(
        c, alpha, beta, de, da, dr, dc, p, q, r, v, B, CBAR, xcg, xcgref
    )


def _plane(n_alpha: int, n_y: int) -> np.ndarray:
    alpha_idx = np.arange(n_alpha, dtype=np.float64)[:, None]
    y_idx = np.arange(n_y, dtype=np.float64)[None, :]
    return 2.0 * alpha_idx + 3.0 * y_idx


def _alpha_index(deg: float) -> float:
    return (deg + 10.0) / 5.0


class TestMost31Evaluate(unittest.TestCase):
    def test_case_angles(self) -> None:
        self.assertEqual(ALPHA_DEG, (-30, -12, -10, -7.5, 0, 3, 22.5, 45, 47, 60, 85))
        self.assertEqual(BETA_DEG, (-35, -30, -7, 0, 4, 30, 35))

    def test_signed_alpha_odd_power(self) -> None:
        c = zeros("t")
        c.poly[CX, M1, 1, 3, 0, 0] = 2.0
        neg = _nine(c, alpha=-0.4)
        pos = _nine(c, alpha=0.4)
        self.assertEqual(neg.shape, (9,))
        self.assertEqual(neg.dtype, np.float64)
        assert_close(self, neg[CX], -0.128, REL)
        assert_close(self, pos[CX], 0.128, REL)

    def test_abs_alpha_even_power(self) -> None:
        c = zeros("t")
        c.poly[CX, M1, 0, 2, 0, 0] = 2.0
        assert_close(self, _nine(c, alpha=-0.4)[CX], 0.32, REL)
        assert_close(self, _nine(c, alpha=0.4)[CX], 0.32, REL)

    def test_sgn_of_zero_is_zero(self) -> None:
        c = zeros("t")
        c.poly[CX, M1, 1, 0, 0, 0] = 5.0
        assert_close(self, _nine(c, alpha=0.0)[CX], 0.0, REL)
        assert_close(self, _nine(c, alpha=0.1)[CX], 5.0, REL)
        assert_close(self, _nine(c, alpha=-0.1)[CX], -5.0, REL)

    def test_beta_and_elevator_powers(self) -> None:
        c = zeros("t")
        c.poly[CM, M1, 0, 0, 2, 3] = 1.5
        got = _nine(c, alpha=0.0, beta=0.2, de=-0.1)[CM]
        assert_close(self, got, 1.5 * 0.04 * (-0.001), REL)

    def test_each_multiplier(self) -> None:
        p, q, r = 0.3, -0.2, 0.1
        da, dr, dc = 0.05, -0.07, 0.11
        expected = (
            1.0,
            p * B / (2.0 * V),
            q * CBAR / (2.0 * V),
            r * B / (2.0 * V),
            da,
            dr,
            dc,
        )
        for index, ref in enumerate(expected):
            c = zeros("t")
            c.poly[CL_ROLL, index, 0, 0, 0, 0] = 1.0
            got = _nine(c, p=p, q=q, r=r, da=da, dr=dr, dc=dc)
            expected_vec = np.zeros(9, dtype=np.float64)
            expected_vec[CL_ROLL] = ref
            assert_close(self, got, expected_vec, REL, MULTIPLIERS[index])

    def test_table_a_inside_below_above(self) -> None:
        c = zeros("t")
        c.table_a[CZ, M1, 0, :] = np.arange(12, dtype=np.float64) ** 2
        # 0° is grid index 2 (value 4); 45° is the last node (value 121).
        assert_close(self, _nine(c, alpha=_rad(0.0))[CZ], 4.0, REL, "breakpoint 0 deg")
        assert_close(self, _nine(c, alpha=_rad(45.0))[CZ], 121.0, REL, "breakpoint 45 deg")
        # 22.5° is the midpoint of 20° (36) and 25° (49).
        assert_close(self, _nine(c, alpha=_rad(22.5))[CZ], 42.5, REL, "midpoint")
        assert_close(self, _nine(c, alpha=_rad(-15.0))[CZ], -1.0, REL, "below grid")
        assert_close(self, _nine(c, alpha=_rad(50.0))[CZ], 142.0, REL, "above grid")

    def test_table_a_beta_power(self) -> None:
        c = zeros("t")
        c.table_a[CZ, M1, 2, :] = 1.0
        assert_close(self, _nine(c, beta=0.3)[CZ], 0.09, REL)

    def test_table_ae_ab_aabsb(self) -> None:
        ae = zeros("t")
        ae.table_ae[CX, M1] = _plane(12, 5)
        assert_close(self, _nine(ae, alpha=_rad(0.0), de=_rad(0.0))[CX], 10.0, REL, "ae inside")
        # alpha index 6.5, elevator index 2.5 → 2*6.5 + 3*2.5
        assert_close(
            self,
            _nine(ae, alpha=_rad(22.5), de=_rad(6.0))[CX],
            20.5,
            REL,
            "ae fractional",
        )
        # alpha index 12, elevator index 5, both past the last segment
        assert_close(
            self,
            _nine(ae, alpha=_rad(50.0), de=_rad(36.0))[CX],
            39.0,
            REL,
            "ae above",
        )
        # alpha index -1, elevator index -1
        assert_close(
            self,
            _nine(ae, alpha=_rad(-15.0), de=_rad(-36.0))[CX],
            -5.0,
            REL,
            "ae below",
        )

        ab = zeros("t")
        ab.table_ab[CX, M1] = _plane(12, 7)
        assert_close(
            self, _nine(ab, alpha=_rad(10.0), beta=_rad(0.0))[CX], 17.0, REL, "ab inside"
        )
        # alpha index 11.4, beta index 6.5
        assert_close(
            self,
            _nine(ab, alpha=_rad(47.0), beta=_rad(35.0))[CX],
            2.0 * _alpha_index(47.0) + 3.0 * ((35.0 + 30.0) / 10.0),
            REL,
            "ab above",
        )
        # alpha index -4, beta index -0.5
        assert_close(
            self,
            _nine(ab, alpha=_rad(-30.0), beta=_rad(-35.0))[CX],
            2.0 * _alpha_index(-30.0) + 3.0 * ((-35.0 + 30.0) / 10.0),
            REL,
            "ab below",
        )

        aabs = zeros("t")
        aabs.table_aabsb[CX, M1] = _plane(12, 7)
        # |beta| index 0.8 at 4°, alpha index 2 → 2*2 + 3*0.8
        inside = 2.0 * _alpha_index(0.0) + 3.0 * (4.0 / 5.0)
        assert_close(self, _nine(aabs, alpha=_rad(0.0), beta=_rad(4.0))[CX], inside, REL, "+beta")
        assert_close(
            self, _nine(aabs, alpha=_rad(0.0), beta=_rad(-4.0))[CX], -inside, REL, "-beta"
        )
        assert_close(self, _nine(aabs, alpha=_rad(22.5), beta=0.0)[CX], 0.0, REL, "beta 0")
        # alpha index 14, |beta| index 7
        outside = 2.0 * _alpha_index(60.0) + 3.0 * (35.0 / 5.0)
        assert_close(
            self, _nine(aabs, alpha=_rad(60.0), beta=_rad(35.0))[CX], outside, REL, "aabs above"
        )
        assert_close(
            self,
            _nine(aabs, alpha=_rad(60.0), beta=_rad(-35.0))[CX],
            -outside,
            REL,
            "aabs above neg",
        )

    def test_cg_shift_on_body_set(self) -> None:
        c = zeros("t")
        c.poly[CZ, M1, 0, 0, 0, 0] = -0.5
        c.poly[CY, M1, 0, 0, 0, 0] = 0.2
        got = _eval(c, xcg=0.25, xcgref=0.35)
        self.assertIsInstance(got, tuple)
        self.assertEqual(len(got), 6)
        self.assertTrue(all(type(item) is float for item in got))
        cx, cy, cz, cl, cm, cn = got
        assert_close(self, (cx, cy, cz, cl), (0.0, 0.2, -0.5, 0.0), REL)
        assert_close(self, cm, -0.05, REL, "Cm")
        assert_close(self, cn, -0.2 * 0.1 * CBAR / B, REL, "Cn")

    def test_wind_set_not_cg_shifted(self) -> None:
        c = zeros("t")
        c.poly[CL, M1, 0, 0, 0, 0] = 1.0
        _cx, _cy, _cz, _cl, cm, _cn = _eval(c, alpha=0.4, xcg=0.25, xcgref=0.35)
        assert_close(self, cm, 0.0, REL)

    def test_wind_to_body_matches_port_rotation(self) -> None:
        c = zeros("t")
        c.poly[CL, M1, 0, 0, 0, 0] = 1.1
        c.poly[CD, M1, 0, 0, 0, 0] = 0.3
        c.poly[CY_W, M1, 0, 0, 0, 0] = -0.2
        alpha = 0.4
        beta = -0.25
        lift_b = np.array([1.1 * np.sin(alpha), 0.0, 1.1 * (-np.cos(alpha))])
        side_b = np.array([(-0.2) * (-np.sin(beta)), (-0.2) * np.cos(beta), 0.0])
        drag_b = rotate_body_to_earth(
            body_321_to_q(0.0, -alpha, beta), np.array([-0.3, 0.0, 0.0])
        )
        ref = lift_b + side_b + drag_b
        cx, cy, cz, _cl, _cm, _cn = _eval(c, alpha=alpha, beta=beta)
        delta = np.abs(np.array([cx, cy, cz]) - ref)
        self.assertLessEqual(float(np.max(delta)), 1e-14)

    def test_nonpositive_speed_raises(self) -> None:
        c = zeros("t")
        for speed in (0.0, -1.0):
            with self.assertRaises(ValueError):
                _eval(c, v=speed)


if __name__ == "__main__":
    unittest.main()
