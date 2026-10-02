"""Unit walls, CG moment shift and sign normalisation for the aero converter.

Nothing here touches a solver or a data file: the maths is proved on its own.
"""
from __future__ import annotations

import math
import unittest

from aero_convert.units import (
    DEG_TO_RAD,
    FT_TO_M,
    G_FT_S2,
    NO_INVARIANT,
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
# ax = cx, ay = cy, az = cz, so the argument names here are exactly the force
# coefficients dynamics.py uses -- no "lift"/"drag" spelling that could be read
# with the wrong sign.  With qbar * S = 1 the force at O is
# F = (cx, cy, cz) = (-0.05, -0.20, +0.35) N, and the moment about the CG is
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
_HAND_WORKED = (0.35, -0.05, -0.20, 0.02, -0.10, 0.07)
_SHIFT_KWARGS = {
    "dx": 0.10,
    "dy": 0.25,
    "dz": -0.05,
    "s_ref": 0.5,
    "b_ref": 3.0,
    "c_ref": 0.6,
}
_EXPECTED_SHIFTED = (
    0.35, -0.05, -0.20,
    -0.005833333333333329,
    -0.045833333333333344,
    0.07250000000000001,
)

# One case per arm, so a wrong sign on any single term shows up.  Same force as
# above, realistic signs: cz = -0.35 is lift UP (cz = -CL, see units.py), cx =
# -0.05 is drag, cy = -0.20 is a side force to the left.  Base moments
# cl_m = 0.02, cm = -0.10, cn = 0.07; b = 3.0, c = 0.6.
#
# Longitudinal arm only (dx = +0.10, the reference 0.10 m behind the CG):
#     (O - G) = (-0.10, 0, 0),  F = (-0.05, -0.20, -0.35)
#     x = 0*(-0.35) - 0*(-0.20)                        =  0.0000 -> Cl = 0.02
#     y = 0*(-0.05) - (-0.10)(-0.35)                    = -0.0350 -> Cm = -0.10 - 0.0583333...
#     z = (-0.10)(-0.20) - 0*(-0.05)                    = +0.0200 -> Cn = 0.07 + 0.0066666...
# Lift up behind the CG pitches the nose down, and the leftward side force
# behind the CG yaws the nose right.  Both agree with the signs above.
_LONGITUDINAL = (0.02, -0.15833333333333333, 0.07666666666666667)

# Lateral arm only (dy = +0.25, the reference 0.25 m to the left of the CG):
#     (O - G) = (0, -0.25, 0)
#     x = (-0.25)(-0.35) - 0*(-0.20)                   = +0.0875 -> Cl = 0.02 + 0.0291666...
#     y = 0*(-0.05) - 0*(-0.35)                        =  0.0000 -> Cm = -0.10 unchanged
#     z = 0*(-0.20) - (-0.25)(-0.05)                   = -0.0125 -> Cn = 0.07 - 0.0041666...
# Lift up left of the CG rolls right (positive), and drag left of the CG pulls
# the tail left, so the nose goes left (negative).
_LATERAL = (0.049166666666666664, -0.1, 0.06583333333333334)

# Vertical arm only (dz = -0.05, the reference 0.05 m BELOW the CG, since z is
# down and dz is the CG relative to the reference):
#     (O - G) = (0, 0, +0.05)
#     x = 0*(-0.35) - (0.05)(-0.20)                    = +0.0100 -> Cl = 0.02 + 0.0033333...
#     y = (0.05)(-0.05) - 0*(-0.35)                    = -0.0025 -> Cm = -0.10 - 0.0041666...
#     z = 0*(-0.20) - 0*(-0.05)                        =  0.0000 -> Cn = 0.07 unchanged
# Lift up below the CG pitches the nose down, and drag below the CG pushes the
# bottom backwards, which also pitches the nose down.
_VERTICAL = (0.023333333333333334, -0.10416666666666667, 0.07)

# All three arms at once: (dx, dy, dz) = (0.10, 0.25, -0.05).
_ALL_ARMS = (0.052500000000000005, -0.1625, 0.07250000000000001)


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

    def test_longitudinal_arm_alone(self) -> None:
        shifted = shift_moments_to_cg(
            -0.35, -0.05, -0.20, 0.02, -0.10, 0.07,
            dx=0.10, dy=0.0, dz=0.0, s_ref=0.5, b_ref=3.0, c_ref=0.6,
        )
        for got, expected in zip(shifted[3:], _LONGITUDINAL):
            self.assertAlmostEqual(got, expected, delta=1e-12)

    def test_lateral_arm_alone(self) -> None:
        shifted = shift_moments_to_cg(
            -0.35, -0.05, -0.20, 0.02, -0.10, 0.07,
            dx=0.0, dy=0.25, dz=0.0, s_ref=0.5, b_ref=3.0, c_ref=0.6,
        )
        for got, expected in zip(shifted[3:], _LATERAL):
            self.assertAlmostEqual(got, expected, delta=1e-12)

    def test_vertical_arm_alone(self) -> None:
        shifted = shift_moments_to_cg(
            -0.35, -0.05, -0.20, 0.02, -0.10, 0.07,
            dx=0.0, dy=0.0, dz=-0.05, s_ref=0.5, b_ref=3.0, c_ref=0.6,
        )
        for got, expected in zip(shifted[3:], _VERTICAL):
            self.assertAlmostEqual(got, expected, delta=1e-12)

    def test_all_three_arms_at_once(self) -> None:
        shifted = shift_moments_to_cg(
            -0.35, -0.05, -0.20, 0.02, -0.10, 0.07,
            dx=0.10, dy=0.25, dz=-0.05, s_ref=0.5, b_ref=3.0, c_ref=0.6,
        )
        for got, expected in zip(shifted[3:], _ALL_ARMS):
            self.assertAlmostEqual(got, expected, delta=1e-12)

    def test_drag_is_signed_not_magnitude(self) -> None:
        # The drag cross-product terms are the only ones that carry cx, so a
        # solver-relative sign error in cx shows up here and nowhere else.
        # Reversing cx to +0.05 (which is a thrust, not drag, in this frame):
        #     y = (dz)(cx) flips with it, so Cm changes by -2*dz*cx/c
        #     z = (dy)(cx) flips with it, so Cn changes by 2*dy*cx/b
        thrust = shift_moments_to_cg(
            -0.35, 0.05, -0.20, 0.02, -0.10, 0.07,
            dx=0.10, dy=0.25, dz=-0.05, s_ref=0.5, b_ref=3.0, c_ref=0.6,
        )
        drag = shift_moments_to_cg(
            -0.35, -0.05, -0.20, 0.02, -0.10, 0.07,
            dx=0.10, dy=0.25, dz=-0.05, s_ref=0.5, b_ref=3.0, c_ref=0.6,
        )
        self.assertAlmostEqual(thrust[4] - drag[4], -2 * (-0.05) * 0.05 / 0.6, delta=1e-12)
        self.assertAlmostEqual(thrust[5] - drag[5], 2 * 0.25 * 0.05 / 3.0, delta=1e-12)
        self.assertEqual(thrust[3], drag[3])

    def test_force_coefficients_come_back_unchanged(self) -> None:
        shifted = shift_moments_to_cg(
            -0.35, -0.05, -0.20, 0.02, -0.10, 0.07,
            dx=0.10, dy=0.25, dz=-0.05, s_ref=0.5, b_ref=3.0, c_ref=0.6,
        )
        self.assertEqual(shifted[:3], (-0.35, -0.05, -0.20))

    def test_measured_tornado_pitch_moment(self) -> None:
        # Measured on the C172: Tornado reports about ref_point = [0, 0, 0] while
        # the CG is at [2.94, 0, 0] ft.  That source frame has x AFT positive --
        # ../USAF_DATCOM/AircraftIntuitiveDesign/Analyses/Cessna172.jsonc lists the
        # wing at XW = 2.2, the CG at XCG = 2.94 and the tail at XH = 8.75 ft from
        # the nose -- so the reference sits 2.94 ft = 0.895512 m AHEAD of the CG.
        # In the forward-positive body frame that is dx = -0.895512 m, and the lift
        # is cz = -CL = -4.363.  Both signs flip relative to quoting the raw source
        # numbers, and they cancel, so the shifted Cm is the plan's value:
        #     dx*cz/c = (-0.895512)(-4.363)/0.6096 = +6.409315708661418
        # Lift up ahead of the CG pitches the nose up, which is what turns Tornado's
        # unphysical -7.720/rad (177 % MAC) into -1.3107/rad (30 % MAC).
        shifted = shift_moments_to_cg(
            -4.363, 0.0, 0.0, 0.0, -7.720, 0.0,
            dx=-0.895512, dy=0.0, dz=0.0, s_ref=1.0, b_ref=1.0, c_ref=0.6096,
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
    def test_keeps_a_negative_cl_alpha(self) -> None:
        # cz = -CL, so a negative CL_alpha is already correct: cz[1] must be negative.
        self.assertEqual(normalise_sign("cz_alpha", -4.0), (-4.0, False))

    def test_flips_a_disagreeing_sign(self) -> None:
        self.assertEqual(normalise_sign("cz_alpha", 4.0), (-4.0, True))
        self.assertEqual(normalise_sign("cn_beta", -0.2), (0.2, True))

    def test_keeps_an_agreeing_sign(self) -> None:
        self.assertEqual(normalise_sign("cz_alpha", -5.0), (-5.0, False))
        self.assertEqual(normalise_sign("cy_beta", -0.6), (-0.6, False))
        self.assertEqual(normalise_sign("cn_beta", 0.2), (0.2, False))

    def test_zero_has_no_sign_to_disagree_with(self) -> None:
        self.assertEqual(normalise_sign("cz_alpha", 0.0), (0.0, False))
        self.assertEqual(normalise_sign("cd0", 0.0), (0.0, False))

    def test_zero_is_never_flipped_even_for_a_positive_pinned_key(self) -> None:
        # Without the zero guard, a +1-pinned key would see (0.0 > 0.0) == False,
        # disagree with the invariant, and return (-0.0, True): a negative zero
        # written into a coefficient file, and a flip that never happened.
        for name in ("cd_q", "cy_r", "cy_dr", "cl_r", "cn_beta"):
            value, flipped = normalise_sign(name, 0.0)
            self.assertEqual(value, 0.0, name)
            self.assertEqual(math.copysign(1.0, value), 1.0, f"{name} returned negative zero")
            self.assertFalse(flipped, name)

    def test_a_slot_with_no_invariant_is_left_alone(self) -> None:
        # cm0 (Cm0) is deliberately absent from the invariant table: the two in-tree
        # reference files disagree on its sign and it is tail rigging, not a convention.
        self.assertEqual(NO_INVARIANT, frozenset({"cm0"}))
        self.assertEqual(normalise_sign("cm0", -0.05), (-0.05, False))
        self.assertEqual(normalise_sign("cm0", 0.05), (0.05, False))
        self.assertEqual(normalise_sign("cm0", 0.0), (0.0, False))

    def test_an_unknown_slot_name_is_rejected(self) -> None:
        # Tasks 3 and 4 call normalise_sign directly to log flips into provenance, so
        # a misspelled slot must fail loudly rather than silently pass a sign through.
        for name in ("cz_alfa", "cz", "cm_alphaa", "cl_"):
            with self.assertRaises(KeyError) as caught:
                normalise_sign(name, -4.0)
            message = str(caught.exception)
            self.assertIn(name, message)
            self.assertIn("cm0", message)
            self.assertIn("cl_alpha", message)

    def test_a_zero_value_does_not_excuse_an_unknown_name(self) -> None:
        with self.assertRaises(KeyError):
            normalise_sign("cz_alfa", 0.0)

    def test_the_four_signposted_rows(self) -> None:
        self.assertEqual(SIGN_INVARIANTS["cl0"], -1)
        self.assertEqual(SIGN_INVARIANTS["cd_q"], 1)
        self.assertEqual(SIGN_INVARIANTS["cy_p"], -1)
        self.assertEqual(SIGN_INVARIANTS["cy_r"], 1)
        self.assertEqual(normalise_sign("cl0", -0.2), (-0.2, False))
        self.assertEqual(normalise_sign("cl0", 0.2), (-0.2, True))
        self.assertEqual(normalise_sign("cd_q", 0.3), (0.3, False))
        self.assertEqual(normalise_sign("cd_q", -0.3), (0.3, True))
        self.assertEqual(normalise_sign("cy_p", -0.1), (-0.1, False))
        self.assertEqual(normalise_sign("cy_p", 0.1), (-0.1, True))
        self.assertEqual(normalise_sign("cy_r", 0.25), (0.25, False))
        self.assertEqual(normalise_sign("cy_r", -0.25), (0.25, True))

    def test_every_derivative_field_but_cm0_has_an_invariant(self) -> None:
        from aero_convert.morelli import DerivativeSet

        # Exactly the 24 rows of the plan's "Physics invariants" table: no field
        # beyond them, and no invented row.
        self.assertNotIn("cm0", SIGN_INVARIANTS)
        fields = set(DerivativeSet.__dataclass_fields__)
        self.assertEqual(set(SIGN_INVARIANTS), fields - {"cm0"})
        for name, sign in SIGN_INVARIANTS.items():
            self.assertIn(sign, (1, -1), name)

    def test_static_margin_convention_is_ratio_of_two_negative_slots(self) -> None:
        # CL_alpha = -cz[1], so -Cm_alpha / CL_alpha reduces to cm[1] / cz[1].
        cz_alpha = normalise_sign("cz_alpha", -4.363)[0]
        cm_alpha = normalise_sign("cm_alpha", -1.31)[0]
        margin = cm_alpha / cz_alpha
        self.assertAlmostEqual(margin, 1.31 / 4.363, delta=1e-12)
        self.assertGreater(margin, 0.0)
        self.assertLess(margin, 0.45)


if __name__ == "__main__":
    unittest.main()