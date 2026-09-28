import unittest

import numpy as np

from f16.compare import SCENARIO_HORIZONS, scenario_x0
from f16.gcas import GcasAutopilot
from f16.llc import F16Llc
from f16.sim import run_sim
from f16.units import H_GCAS_M, VT_GCAS_MPS, deg_to_rad


def _collapsed(modes) -> list[str]:
    seq: list[str] = []
    for mode in modes:
        if not seq or seq[-1] != mode:
            seq.append(mode)
    return seq


class TestGcas(unittest.TestCase):
    def _assert_spec_state(self, scenario: str) -> np.ndarray:
        x0 = scenario_x0(scenario)
        self.assertAlmostEqual(float(x0[0]), VT_GCAS_MPS, places=6)
        self.assertAlmostEqual(float(x0[0]), 164.592, places=6)
        self.assertAlmostEqual(float(x0[1]), deg_to_rad(2.1215), places=9)
        self.assertAlmostEqual(float(x0[11]), H_GCAS_M, places=6)
        self.assertAlmostEqual(float(x0[11]), 304.8, places=6)
        self.assertAlmostEqual(float(x0[12]), 9.0, places=6)
        if scenario == "gcas_upright":
            self.assertAlmostEqual(float(x0[3]), -np.pi / 8.0)
            self.assertAlmostEqual(float(x0[4]), -0.3 * np.pi / 2.0)
        else:
            self.assertAlmostEqual(float(x0[3]), -0.9 * np.pi)
            self.assertAlmostEqual(float(x0[4]), -0.01 * np.pi / 2.0)
        return x0

    def _fly(self, scenario: str) -> dict:
        llc = F16Llc()
        ap = GcasAutopilot(init_mode="standby", llc=llc)
        x0 = self._assert_spec_state(scenario)
        return run_sim(ap, x0, t_end=SCENARIO_HORIZONS[scenario], step=1 / 30)

    def test_mode_sequence_upright(self) -> None:
        out = self._fly("gcas_upright")
        # waiting is absent: init_mode standby on the example state never enters it.
        self.assertEqual(_collapsed(out["modes"]), ["standby", "roll", "pull", "standby"])
        self.assertGreater(out["min_h_m"], 0.0)

    def test_mode_sequence_inverted(self) -> None:
        out = self._fly("gcas_inverted")
        self.assertEqual(_collapsed(out["modes"]), ["standby", "roll", "pull", "standby"])
        self.assertGreater(out["min_h_m"], 0.0)

    def test_frozen_trace_regression(self) -> None:
        ref = np.load("tests/frozen_gcas_upright.npz")
        llc = F16Llc()
        ap = GcasAutopilot(init_mode="standby", llc=llc)
        out = run_sim(ap, ref["x0"], t_end=float(ref["t_end"]), step=1 / 30)
        ours = np.array([s[:13] for s in out["states"]])
        np.testing.assert_allclose(ours[-1], ref["final"], rtol=1e-4, atol=1e-2)
