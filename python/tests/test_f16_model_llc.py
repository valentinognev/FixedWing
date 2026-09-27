import unittest

import numpy as np

REF = "/home/valentin/Projects/FlightSimulation/F16/AeroBenchVVPython/code"

import sys

sys.path.insert(0, REF)

from aerobench.lowlevel.low_level_controller import (  # noqa: E402
    LowLevelController as RefLlc,
)
from aerobench.lowlevel.subf16_model import subf16_model as ref_subf16  # noqa: E402

from f16.llc import CtrlLimits, F16Llc  # noqa: E402
from f16.model import subf16_derivative  # noqa: E402


class TestModelLlc(unittest.TestCase):
    def test_derivative_matches_reference_at_trim(self) -> None:
        llc = F16Llc()
        x = llc.xequil.copy()
        u = llc.uequil.copy()
        xd, Nz, Ny = subf16_derivative(x[:13], u)
        rxd, RNz, RNy, _, _ = ref_subf16(x[:13], u, "morelli")
        np.testing.assert_allclose(xd, rxd, rtol=1e-9, atol=1e-9)
        self.assertAlmostEqual(Nz, RNz, places=9)
        self.assertAlmostEqual(Ny, RNy, places=9)

    def test_llc_matches_reference(self) -> None:
        llc = F16Llc()
        ref = RefLlc()
        # get_u_deg indexes integrator states 13..15; trim integrators are zero.
        x = np.concatenate([ref.xequil, np.zeros(3)])
        u_ref = np.array([0.0, 0.0, 0.0, 0.1395])
        _, u_deg = llc.get_u_deg(u_ref, x)
        _, ru_deg = ref.get_u_deg(u_ref, x)
        np.testing.assert_allclose(u_deg, ru_deg, rtol=1e-9, atol=1e-9)
        self.assertEqual(llc.get_num_integrators(), 3)
        self.assertEqual(CtrlLimits().NzMax, 6)
