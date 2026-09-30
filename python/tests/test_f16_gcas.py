import unittest

import numpy as np

from f16.compare import SCENARIO_HORIZONS
from f16.gcas import GcasAutopilot
from f16.llc import F16Llc
from f16.sim import run_sim
from f16.units import VT_GCAS_MPS, ft_to_m

_H_TRIM_M = ft_to_m(1500.0)


def _collapsed(modes) -> list[str]:
    seq: list[str] = []
    for mode in modes:
        if not seq or seq[-1] != mode:
            seq.append(mode)
    return seq


class TestGcas(unittest.TestCase):
    def _fly(self, scenario: str) -> dict:
        from f16.trim_table import trimmed_initial

        x0, u0 = trimmed_initial(scenario, aero="morelli", height_m=_H_TRIM_M)
        self.assertAlmostEqual(float(x0[0]), VT_GCAS_MPS, places=6)
        self.assertEqual(float(x0[3]), 0.0)
        self.assertAlmostEqual(float(x0[4]), float(x0[1]), places=12)
        self.assertAlmostEqual(float(x0[11]), _H_TRIM_M, places=6)
        llc = F16Llc()
        llc.xequil = x0.copy()
        llc.uequil = u0.copy()
        ap = GcasAutopilot(init_mode="standby", llc=llc)
        return run_sim(ap, x0, t_end=SCENARIO_HORIZONS[scenario], step=1 / 30)

    def test_mode_sequence_upright(self) -> None:
        out = self._fly("gcas_upright")
        self.assertEqual(_collapsed(out["modes"]), ["standby"])
        self.assertGreater(out["min_h_m"], 0.0)

    def test_mode_sequence_inverted(self) -> None:
        from f16.trim_table import trimmed_initial

        out = self._fly("gcas_inverted")
        self.assertEqual(_collapsed(out["modes"]), ["standby"])
        upright, _ = trimmed_initial("gcas_upright", aero="morelli", height_m=_H_TRIM_M)
        inverted, _ = trimmed_initial("gcas_inverted", aero="morelli", height_m=_H_TRIM_M)
        np.testing.assert_allclose(upright, inverted, rtol=0, atol=0)
