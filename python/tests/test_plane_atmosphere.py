import math
import unittest

from f16.units import H_STRAT_M, LAPSE_PER_M, R_AIR, RHO0_KG_M3, T0_K, T_STRAT_K


class TestPlaneAtmosphere(unittest.TestCase):
    def test_sea_level(self) -> None:
        from plane.atmosphere import air

        mach, qbar, rho = air(10.0, 0.0)
        self.assertAlmostEqual(rho, RHO0_KG_M3, delta=0)
        self.assertAlmostEqual(qbar, 0.5 * RHO0_KG_M3 * 100.0, delta=1e-9)
        speed = math.sqrt(1.4 * R_AIR * T0_K)
        self.assertAlmostEqual(mach, 10.0 / speed, delta=1e-12)

    def test_stratosphere_uses_the_f16_temperature(self) -> None:
        from plane.atmosphere import air

        mach, _, rho = air(10.0, H_STRAT_M)
        tfac = 1.0 - LAPSE_PER_M * H_STRAT_M
        self.assertAlmostEqual(rho, RHO0_KG_M3 * tfac ** 4.14, delta=1e-9)
        speed = math.sqrt(1.4 * R_AIR * T_STRAT_K)
        self.assertAlmostEqual(mach, 10.0 / speed, delta=1e-12)

    def test_nonpositive_speed_or_negative_altitude_raises(self) -> None:
        from plane.atmosphere import air

        with self.assertRaises(ValueError):
            air(0.0, 0.0)
        with self.assertRaises(ValueError):
            air(10.0, -1.0)
