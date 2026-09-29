import unittest

import numpy as np

from f16.cpp_probe import cpp_available, get_llc, get_plant
from f16.llc import F16Llc
from f16.model import subf16_derivative
from f16.units import state_si_to_imp, u_si_to_imp


def _cpp_xd(plant, x_si, u_si):
    full = np.asarray(plant.f16model(
        np.ascontiguousarray(state_si_to_imp(x_si), dtype=np.float64),
        np.ascontiguousarray(u_si_to_imp(u_si), dtype=np.float64),
    ), dtype=np.float64).reshape(-1)
    return full


class TestCppPlant(unittest.TestCase):
    def test_morelli_derivative_matches_cpp_at_trim(self) -> None:
        if not cpp_available():
            self.skipTest("f16dynamics not built")
        plant = get_plant()
        llc = F16Llc()
        x = llc.xequil.copy()
        u = llc.uequil.copy()
        xd, nz, ny = subf16_derivative(x[:13], u)
        full = _cpp_xd(plant, x[:13], u)
        np.testing.assert_allclose(state_si_to_imp(xd), full[:13], rtol=1e-6, atol=1e-6)
        self.assertAlmostEqual(nz, float(full[13]), places=5)
        self.assertAlmostEqual(ny, float(full[14]), places=5)

    def test_cpp_llc_uses_csaf_lateral_gains(self) -> None:
        if not cpp_available():
            self.skipTest("f16dynamics not built")
        llc_cpp = get_llc()
        ours = F16Llc()
        x13 = ours.xequil.copy()
        xd, nz, ny = subf16_derivative(x13, ours.uequil.copy())
        full17 = np.concatenate([state_si_to_imp(x13), [nz, ny, 0.0, 0.0]])
        u_ref = np.array([0.0, 0.0, 0.0, 0.1395])
        llc_in = np.ascontiguousarray(np.concatenate([full17, u_ref]), dtype=np.float64)
        llc_state = np.ascontiguousarray(np.array([0.0, 0.01, 0.0]), dtype=np.float64)
        out = np.asarray(llc_cpp.output(llc_state, llc_in), dtype=float).reshape(-1)
        x16 = np.concatenate([x13, [0.0, 0.01, 0.0]])
        _, u_si = ours.get_u(u_ref, x16)
        u_deg = u_si_to_imp(u_si)
        self.assertAlmostEqual(float(out[0]), float(u_deg[0]), places=5)
        self.assertAlmostEqual(float(out[1]), float(u_deg[1]), places=5)
        self.assertGreater(abs(float(out[2]) - float(u_deg[2])), 1.0)
