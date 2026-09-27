import unittest

import numpy as np

from f16.llc import F16Llc
from f16.sim import run_sim
from f16.straight_level import StraightLevelAutopilot


class TestSimSl(unittest.TestCase):
    def test_sl_holds_trim(self) -> None:
        llc = F16Llc()
        ap = StraightLevelAutopilot(llc.xequil[11], llc.xequil[0], llc=llc)
        out = run_sim(ap, llc.xequil.copy(), t_end=5.0, step=1 / 30)
        h = np.array([s[11] for s in out["states"]])
        vt = np.array([s[0] for s in out["states"]])
        self.assertTrue(np.all(np.isfinite(h)))
        self.assertLess(float(np.abs(h - h[0]).max()), 15.0)
        self.assertLess(float(np.abs(vt - vt[0]).max()), 8.0)
        self.assertGreater(out["min_h_ft"], 900.0)
