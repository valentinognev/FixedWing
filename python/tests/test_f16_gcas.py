import unittest

import numpy as np

from f16.gcas import GcasAutopilot
from f16.llc import F16Llc
from f16.sim import run_sim


class TestGcas(unittest.TestCase):
    def _fly(self, inverted: bool) -> dict:
        llc = F16Llc()
        ap = GcasAutopilot(init_mode="standby", llc=llc)
        x0 = llc.xequil.copy()
        x0[11] = 1500.0
        x0[4] = -0.3 if not inverted else float(np.pi) + 0.3
        if inverted:
            x0[3] = float(np.pi)
        return run_sim(ap, x0, t_end=12.0, step=1 / 30)

    def test_mode_sequence_upright(self) -> None:
        out = self._fly(inverted=False)
        seq: list[str] = []
        for m in out["modes"]:
            if not seq or seq[-1] != m:
                seq.append(m)
        self.assertEqual(seq[0], "standby")
        self.assertIn("roll", seq)
        self.assertIn("pull", seq)
        self.assertGreater(out["min_h_ft"], 0.0)

    def test_frozen_trace_regression(self) -> None:
        ref = np.load("tests/frozen_gcas_upright.npz")
        llc = F16Llc()
        ap = GcasAutopilot(init_mode="standby", llc=llc)
        out = run_sim(ap, ref["x0"], t_end=float(ref["t_end"]), step=1 / 30)
        ours = np.array([s[:13] for s in out["states"]])
        np.testing.assert_allclose(ours[-1], ref["final"], rtol=1e-4, atol=1e-2)
