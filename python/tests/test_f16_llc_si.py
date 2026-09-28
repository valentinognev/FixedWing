import unittest

import numpy as np

from f16.llc import CtrlLimits, F16Llc
from f16.units import DEG_TO_RAD, deg_to_rad, state_imp_to_si, u_imp_to_si

_K_LONG_DEG = np.array(
    [[-156.8801506723475, -31.037008068526642, -38.72983346216317]],
    dtype=float,
)
_K_LAT_DEG = np.array(
    [[37.84483, -25.40956, -6.82876, -332.88343, -17.15997],
     [-23.91233, 5.69968, -21.63431, 64.49490, -88.36203]],
    dtype=float,
)
_X_IMP = np.array(
    [502.0, 0.0389, 0.0, 0.0, 0.0389, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0, 9.0567],
    dtype=float,
)
_U_IMP = np.array([0.1395, -0.7496, 0.0, 0.0], dtype=float)


class TestLlcSi(unittest.TestCase):
    def test_trim_and_gains_are_scaled(self) -> None:
        llc = F16Llc()
        np.testing.assert_allclose(llc.xequil, state_imp_to_si(_X_IMP), rtol=0, atol=1e-9)
        np.testing.assert_allclose(llc.uequil, u_imp_to_si(_U_IMP), rtol=0, atol=1e-12)
        np.testing.assert_allclose(llc.K_lqr[0, :3], _K_LONG_DEG.reshape(-1) * DEG_TO_RAD, rtol=0, atol=1e-12)
        np.testing.assert_allclose(llc.K_lqr[1:, 3:], _K_LAT_DEG * DEG_TO_RAD, rtol=0, atol=1e-12)
        self.assertFalse(np.allclose(llc.K_lqr[0, :3], _K_LONG_DEG.reshape(-1), atol=1e-6))
        self.assertAlmostEqual(float(llc.xequil[0]), 153.0096, places=6)
        self.assertAlmostEqual(float(llc.xequil[11]), 304.8, places=6)
        self.assertFalse(hasattr(llc, "get_u_deg"))

    def test_surface_limits_are_radians(self) -> None:
        lim = CtrlLimits()
        self.assertAlmostEqual(lim.ElevatorMaxRad, deg_to_rad(25.0), places=12)
        self.assertAlmostEqual(lim.ElevatorMinRad, deg_to_rad(-25.0), places=12)
        self.assertAlmostEqual(lim.AileronMaxRad, deg_to_rad(21.5), places=12)
        self.assertAlmostEqual(lim.AileronMinRad, deg_to_rad(-21.5), places=12)
        self.assertAlmostEqual(lim.RudderMaxRad, deg_to_rad(30.0), places=12)
        self.assertAlmostEqual(lim.RudderMinRad, deg_to_rad(-30.0), places=12)
        self.assertEqual(lim.NzMax, 6)
        self.assertEqual(lim.NzMin, -1)
        self.assertEqual(lim.ThrottleMax, 1)
        self.assertEqual(lim.ThrottleMin, 0)

    def test_get_u_at_trim_is_si_equilibrium(self) -> None:
        llc = F16Llc()
        x = np.concatenate([llc.xequil, np.zeros(3)])
        u_ref = np.array([0.0, 0.0, 0.0, 0.1395])
        _, u = llc.get_u(u_ref, x)
        # AeroBench adds uequil after copying throttle from u_ref[3], so trim
        # throttle is 0.1395 + 0.1395. Surfaces at x_ctrl = 0 stay uequil.
        expected = llc.uequil.copy()
        expected[0] += u_ref[3]
        np.testing.assert_allclose(u, expected, rtol=0, atol=1e-9)
        self.assertEqual(llc.get_num_integrators(), 3)
