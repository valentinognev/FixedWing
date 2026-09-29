import sys
import unittest

import numpy as np

REF = "/home/valentin/Projects/FlightSimulation/F16/AeroBenchVVPython/code"
sys.path.insert(0, REF)

from aerobench.lowlevel.morellif16 import Morellif16  # noqa: E402
from aerobench.lowlevel.subf16_model import subf16_model as ref_subf16  # noqa: E402

from f16.llc import _U_IMP, _X_IMP  # noqa: E402
from f16.model import subf16_derivative  # noqa: E402
from f16.units import (  # noqa: E402
    aerobench_poly_rad,
    state_imp_to_si,
    state_si_to_imp,
    u_imp_to_si,
)


class TestPureMorelli(unittest.TestCase):
    def test_coefficients_match_polynomial_at_nonzero_pitch_rate(self) -> None:
        from f16.aero_morelli import morelli_coefficients
        from f16.units import B_M, CBAR_M

        alpha = 0.04
        beta = 0.01
        de = -0.02
        p, q, r = 0.05, 0.2, -0.03
        vt = 150.0
        ours = morelli_coefficients(
            aerobench_poly_rad(alpha), aerobench_poly_rad(beta), de, 0.01, -0.01,
            p, q, r, CBAR_M, B_M, vt, 0.35, 0.35,
        )
        ref = Morellif16(
            aerobench_poly_rad(alpha), aerobench_poly_rad(beta), de, 0.01, -0.01,
            p, q, r, CBAR_M, B_M, vt, 0.35, 0.35,
        )
        np.testing.assert_allclose(ours, ref, rtol=0, atol=1e-12)

    def test_zero_rate_derivative_matches_aerobench_morelli(self) -> None:
        x_imp = _X_IMP.copy()
        u_imp = _U_IMP.copy()
        xd, nz, ny = subf16_derivative(state_imp_to_si(x_imp), u_imp_to_si(u_imp), model="morelli")
        rxd, rnz, rny, _, _ = ref_subf16(x_imp, u_imp, "morelli")
        np.testing.assert_allclose(state_si_to_imp(xd), rxd, rtol=1e-8, atol=1e-8)
        self.assertAlmostEqual(nz, float(rnz), places=6)
        self.assertAlmostEqual(ny, float(rny), places=6)

    def test_nonzero_pitch_rate_is_not_the_mixed_model(self) -> None:
        x_imp = _X_IMP.copy()
        x_imp[7] = 0.2
        u_imp = _U_IMP.copy()
        xd, _, _ = subf16_derivative(state_imp_to_si(x_imp), u_imp_to_si(u_imp), model="morelli")
        rxd, _, _, _, _ = ref_subf16(x_imp, u_imp, "morelli")
        self.assertGreater(abs(float(state_si_to_imp(xd)[7]) - float(rxd[7])), 1e-3)


class TestPureStevens(unittest.TestCase):
    def test_sideforce_matches_appendix_linear_law(self) -> None:
        from f16.aero_stevens import stevens_coefficients
        from f16.units import B_M, CBAR_M, aerobench_deg

        beta = 0.1
        ail = 0.05
        rdr = -0.02
        beta_deg = aerobench_deg(beta)
        ail_deg = aerobench_deg(ail)
        rdr_deg = aerobench_deg(rdr)
        expected_cy = -0.02 * beta_deg + 0.021 * (ail_deg / 20.0) + 0.086 * (rdr_deg / 30.0)
        _, cy, _, _, _, _ = stevens_coefficients(
            0.0, beta, 0.0, ail, rdr, 0.0, 0.0, 0.0, CBAR_M, B_M, 150.0, 0.35, 0.35,
        )
        self.assertAlmostEqual(cy, expected_cy, places=10)

    def test_derivative_matches_aerobench_stevens_with_pitch_rate(self) -> None:
        x_imp = _X_IMP.copy()
        x_imp[7] = 0.2
        u_imp = _U_IMP.copy()
        xd, nz, ny = subf16_derivative(state_imp_to_si(x_imp), u_imp_to_si(u_imp), model="stevens")
        rxd, rnz, rny, _, _ = ref_subf16(x_imp, u_imp, "stevens")
        np.testing.assert_allclose(state_si_to_imp(xd), rxd, rtol=1e-8, atol=1e-8)
        self.assertAlmostEqual(nz, float(rnz), places=6)
        self.assertAlmostEqual(ny, float(rny), places=6)

    def test_models_differ_at_nonzero_pitch_rate(self) -> None:
        x = state_imp_to_si(_X_IMP)
        u = u_imp_to_si(_U_IMP)
        x = x.copy()
        x[7] = 0.2
        xd_m, _, _ = subf16_derivative(x, u, model="morelli")
        xd_s, _, _ = subf16_derivative(x, u, model="stevens")
        self.assertGreater(abs(float(xd_m[7] - xd_s[7])), 1e-4)

    def test_unknown_model_raises(self) -> None:
        with self.assertRaises(ValueError):
            subf16_derivative(state_imp_to_si(_X_IMP), u_imp_to_si(_U_IMP), model="mixed")
