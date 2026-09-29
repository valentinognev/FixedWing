import sys
import unittest

import numpy as np

REF = "/home/valentin/Projects/FlightSimulation/F16/AeroBenchVVPython/code"
sys.path.insert(0, REF)

from aerobench.lowlevel.subf16_model import subf16_model as ref_subf16  # noqa: E402

from aerobench.lowlevel.thrust import thrust as ref_thrust  # noqa: E402

from f16.model import _thrust, subf16_derivative  # noqa: E402
from f16.units import AEROBENCH_RTOD, ft_to_m, lbf_to_n, state_imp_to_si, u_imp_to_si  # noqa: E402

_X_IMP = np.array(
    [502.0, 0.0389, 0.0, 0.0, 0.0389, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0, 9.0567],
    dtype=float,
)
_U_IMP = np.array([0.1395, -0.7496, 0.0, 0.0], dtype=float)


class TestModelSi(unittest.TestCase):
    def _match(self, x_imp: np.ndarray, u_imp: np.ndarray) -> None:
        rxd, rnz, rny, _, _ = ref_subf16(x_imp, u_imp, "morelli")
        xd, nz, ny = subf16_derivative(state_imp_to_si(x_imp), u_imp_to_si(u_imp))
        np.testing.assert_allclose(xd, state_imp_to_si(rxd), rtol=1e-8, atol=1e-8)
        self.assertAlmostEqual(nz, float(rnz), places=6)
        self.assertAlmostEqual(ny, float(rny), places=6)

    def test_trim_matches_aerobench_after_conversion(self) -> None:
        self._match(_X_IMP, _U_IMP)

    def test_off_trim_zero_rate_matches_aerobench_morelli(self) -> None:
        x = _X_IMP.copy()
        x[0] += 20.0
        x[1] += 0.02
        u = _U_IMP.copy()
        u[1] += 2.0
        self._match(x, u)

    def test_idle_thrust_at_mach_0_6_10kft_is_book_value(self) -> None:
        """Stevens Appendix A.7 idle table: −710 lbf at Mach 0.6, 10,000 ft.

        AeroBench printed −170 in that cell.
        """
        self.assertAlmostEqual(float(ref_thrust(0.0, 10000.0, 0.6)), -710.0, places=6)
        self.assertAlmostEqual(_thrust(0.0, ft_to_m(10000.0), 0.6), lbf_to_n(-710.0), places=4)

    def test_unknown_model_raises(self) -> None:
        with self.assertRaises(ValueError):
            subf16_derivative(state_imp_to_si(_X_IMP), u_imp_to_si(_U_IMP), model="mixed")

    def test_dampp_ten_aerobench_degrees_from_radians(self) -> None:
        from f16.aero_stevens import damp_table
        from f16.units import AEROBENCH_RTOD

        alpha_rad = 10.0 / AEROBENCH_RTOD
        expected = np.array(
            [2.08, 0.962, 0.258, -31.2, 0.208, -0.383, -6.11, -0.370, -0.013],
            dtype=float,
        )
        np.testing.assert_allclose(damp_table(alpha_rad), expected, rtol=0, atol=1e-12)
