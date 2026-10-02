"""Unit walls, CG moment shift and sign normalisation for the aero converter.

Nothing here touches a solver or a data file: the maths is proved on its own.
"""
from __future__ import annotations

import unittest

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

# The hand-worked shift case.  The reference point O sits at CG + (-dx, -dy, -dz),
# with (dx, dy, dz) the CG position relative to O, in the body frame
# x forward, y right, z DOWN:
#
#     dx = +0.10 m   O is 0.10 m behind the CG
#     dy = +0.25 m   O is 0.25 m to the left of the CG
#     dz = -0.05 m   O is 0.05 m above the CG   (z is down, hence negative)
#
# plane/dynamics.py resolves the force as (qbar * S) * (cx, cy, cz) with
# ax = cx, ay = cy, az = cz, so the coefficient of the body +z force is `cl` and
# the body x force is -cd (negative cx is drag).  With qbar * S = 1 the force at
# O is F = (-0.05, -0.20, +0.35) N.  The moment about the CG is
# M_G = M_O + (O - G) x F with (O - G) = (-0.10, -0.25, +0.05):
#
#     (O - G) x F = ( (-0.25)(0.35) - (0.05)(-0.20),
#                     (0.05)(-0.05) - (-0.10)(0.35),
#                     (-0.10)(-0.20) - (-0.25)(-0.05) )
#                 = ( -0.0875 + 0.0100,
#                     -0.0025 + 0.0350,
#                     +0.0200 - 0.0125 )
#                 = ( -0.0775, +0.0325, +0.0075 )  N m
#
# Dividing by qbar * S and by the reference length of each axis:
#     Cl: (-0.0775) / 3.0 = -0.025833333333333329  <- contains the dz term
#     Cm: (+0.0325) / 0.6 = +0.054166666666666672
#     Cn: (+0.0075) / 3.0 = +0.0025               <- contains the dy term
#
# so the shifted moments are -0.005833333333333329, -0.045833333333333344 and
# 0.07250000000000001, and the force coefficients are untouched.
_HAND_WORKED = (0.35, 0.05, -0.20, 0.02, -0.10, 0.07)
_SHIFT_KWARGS = {
    "dx": 0.10,
    "dy": 0.25,
    "dz": -0.05,
    "s_ref": 0.5,
    "b_ref": 3.0,
    "c_ref": 0.6,
}
_EXPECTED_SHIFTED = (
    0.35, 0.05, -0.20,
    -0.005833333333333329,
    -0.045833333333333344,
    0.07250000000000001,
)


class TestAeroConvertConstants(unittest.TestCase):
    def test_constant_values(self) -> None:
        self.assertEqual(DEG_TO_RAD, 57.29577951308232)
        self.assertEqual(FT_TO_M, 0.3048)
        self.assertEqual(SLUG_TO_KG, 14.59390294)
        self.assertEqual(G_FT_S2, 32.174)


class TestAeroConvertScalars(unittest.TestCase):
    def test_per_degree_to_per_radian_applies_the_factor(self) -> None:
        self.assertAlmostEqual(per_degree_to_per_radian(0.0756), 0.0756 * 57.29577951308232, delta=1e-12)

    def test_per_degree_to_per_radian_converts_the_datcom_lift_slope(self) -> None:
        # aid's handbook stores the C172 lift slope as HT.a = 0.0756 per degree,
        # which is 4.330 per radian once the factor is applied.
        self.assertAlmostEqual(per_degree_to_per_radian(0.0756), 4.330, places=2)

    def test_ft_converts_to_si(self) -> None:
        self.assertAlmostEqual(ft(12.0), 3.6576, places=12)


class TestShiftMomentsToCg(unittest.TestCase):
    def test_hand_worked_three_element_case(self) -> None:
        shifted = shift_moments_to_cg(*_HAND_WORKED, **_SHIFT_KWARGS)
        self.assertEqual(len(shifted), 6)
        for got, expected in zip(shifted, _EXPECTED_SHIFTED):
            self.assertAlmostEqual(got, expected, delta=1e-12)

    def test_roll_term_carries_dz_and_yaw_term_carries_dy(self) -> None:
        # Moving dz to -0.15 puts the reference point 0.15 m above the CG, so
        # (O - G) = (-0.10, -0.25, +0.15) and
        #     x = (-0.25)(0.35) - (0.15)(-0.20) = -0.0575 -> Cl = 0.02 - 0.0575/3.0
        #     y = (0.15)(-0.05) - (-0.10)(0.35) = +0.0275 -> Cm = -0.10 + 0.0275/0.6
        #     z = unchanged, because dz and dx did not move -> Cn unchanged
        moved_z = shift_moments_to_cg(*_HAND_WORKED, **{**_SHIFT_KWARGS, "dz": -0.15})
        self.assertAlmostEqual(moved_z[3], 0.0008333333333333352, delta=1e-12)
        self.assertAlmostEqual(moved_z[4], -0.054166666666666675, delta=1e-12)
        self.assertAlmostEqual(moved_z[5], 0.07250000000000001, delta=1e-12)

        # Moving dy to 0.05 puts the reference 0.05 m to the left of the CG, so
        # (O - G) = (-0.10, -0.05, +0.05) and
        #     x = (-0.05)(0.35) - (0.05)(-0.20) = -0.0075 -> Cl = 0.02 - 0.0075/3.0
        #     y = unchanged, because dy and dz did not move -> Cm unchanged
        #     z = (-0.10)(-0.20) - (-0.05)(-0.05) = +0.0175 -> Cn = 0.07 + 0.0175/3.0
        moved_y = shift_moments_to_cg(*_HAND_WORKED, **{**_SHIFT_KWARGS, "dy": 0.05})
        self.assertAlmostEqual(moved_y[3], 0.0175, delta=1e-12)
        self.assertAlmostEqual(moved_y[4], -0.045833333333333344, delta=1e-12)
        self.assertAlmostEqual(moved_y[5], 0.07583333333333334, delta=1e-12)

    def test_measured_tornado_pitch_moment(self) -> None:
        # Measured on the C172: Tornado reports about ref_point = (0, 0, 0) while
        # the CG is at (2.94, 0, 0) ft, so the CG is 2.94 ft = 0.895512 m ahead of
        # the reference.  Raw Cm_a = -7.720/rad there, CL_a = 4.363/rad.
        shifted = shift_moments_to_cg(
            4.363, 0.0, 0.0, 0.0, -7.720, 0.0,
            dx=0.895512, dy=0.0, dz=0.0, s_ref=1.0, b_ref=1.0, c_ref=0.6096,
        )
        self.assertAlmostEqual(shifted[4], -7.720 + 0.895512 * 4.363 / 0.6096, delta=1e-6)
        self.assertAlmostEqual(shifted[4], -1.3106842913385819, delta=1e-6)

    def test_reference_at_the_cg_is_a_no_op(self) -> None:
        base = shift_moments_to_cg(
            *_HAND_WORKED, dx=0.0, dy=0.0, dz=0.0, s_ref=0.5, b_ref=3.0, c_ref=0.6,
        )
        for got, expected in zip(base, _HAND_WORKED):
            self.assertEqual(got, expected)


class TestNormaliseSign(unittest.TestCase):
    def test_flips_a_disagreeing_sign(self) -> None:
        self.assertEqual(normalise_sign("cz_alpha", -4.0), (4.0, True))

    def test_keeps_an_agreeing_sign(self) -> None:
        self.assertEqual(normalise_sign("cz_alpha", 5.0), (5.0, False))
        self.assertEqual(normalise_sign("cy_beta", -0.6), (-0.6, False))

    def test_zero_has_no_sign_to_disagree_with(self) -> None:
        self.assertEqual(normalise_sign("cz_alpha", 0.0), (0.0, False))
        self.assertEqual(normalise_sign("cd0", 0.0), (0.0, False))

    def test_every_derivative_field_has_an_invariant(self) -> None:
        from aero_convert.morelli import DerivativeSet

        for field in DerivativeSet.__dataclass_fields__:
            self.assertIn(field, SIGN_INVARIANTS)
        for name, sign in SIGN_INVARIANTS.items():
            self.assertIn(sign, (1, -1), name)

    def test_unknown_slot_name_is_rejected(self) -> None:
        with self.assertRaises(KeyError):
            normalise_sign("cz_nonesuch", 1.0)


if __name__ == "__main__":
    unittest.main()