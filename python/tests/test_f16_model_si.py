import sys
import unittest

import numpy as np

REF = "/home/valentin/Projects/FlightSimulation/F16/AeroBenchVVPython/code"
sys.path.insert(0, REF)

from aerobench.lowlevel.subf16_model import subf16_model as ref_subf16  # noqa: E402

from f16.model import subf16_derivative  # noqa: E402
from f16.units import state_imp_to_si, u_imp_to_si  # noqa: E402

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

    def test_off_trim_matches_aerobench_after_conversion(self) -> None:
        x = _X_IMP.copy()
        x[0] += 20.0
        x[1] += 0.02
        x[6] += 0.05
        u = _U_IMP.copy()
        u[1] += 2.0
        self._match(x, u)

    def test_stevens_raises(self) -> None:
        with self.assertRaises(ValueError):
            subf16_derivative(state_imp_to_si(_X_IMP), u_imp_to_si(_U_IMP), model="stevens")
