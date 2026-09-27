import unittest

import numpy as np

from f16.autopilot import F16Autopilot
from f16.llc import F16Llc
from f16.sim import run_sim
from f16.straight_level import StraightLevelAutopilot


class _BlowLlc(F16Llc):
    """Real LQR until `blow` makes the integrator derivative non-finite."""

    def __init__(self) -> None:
        super().__init__()
        self.blow = False

    def get_integrator_derivatives(self, t, x_f16, u_ref4, Nz, ps, Ny_r):
        if self.blow:
            return [float("nan"), 0.0, 0.0]
        return super().get_integrator_derivatives(t, x_f16, u_ref4, Nz, ps, Ny_r)


class _HoldThenBlow(F16Autopilot):
    def __init__(self, llc: _BlowLlc) -> None:
        super().__init__("hold", llc)

    def advance_discrete_mode(self, t: float, x_f16: np.ndarray) -> bool:
        if t > 0.0 and self.mode == "hold":
            self.mode = "blow"
            self.llc.blow = True
            return True
        return False

    def get_u_ref(self, t: float, x_f16: np.ndarray) -> tuple[float, float, float, float]:
        return 0.0, 0.0, 0.0, 0.0


class _BlowInFlight(F16Llc):
    """Finite at t = 0; the next derivative evaluation is non-finite."""

    def get_integrator_derivatives(self, t, x_f16, u_ref4, Nz, ps, Ny_r):
        if t > 0.0:
            return [float("nan"), 0.0, 0.0]
        return super().get_integrator_derivatives(t, x_f16, u_ref4, Nz, ps, Ny_r)


class _Hold(F16Autopilot):
    def __init__(self, llc: F16Llc) -> None:
        super().__init__("hold", llc)

    def get_u_ref(self, t: float, x_f16: np.ndarray) -> tuple[float, float, float, float]:
        return 0.0, 0.0, 0.0, 0.0


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

    def test_nonfinite_initial_state_keeps_sample(self) -> None:
        llc = F16Llc()
        x0 = llc.xequil.copy()
        x0[0] = np.nan
        ap = StraightLevelAutopilot(llc.xequil[11], llc.xequil[0], llc=llc)
        out = run_sim(ap, x0, t_end=1.0, step=1 / 30)
        self.assertEqual(len(out["states"]), 1)
        self.assertEqual(len(out["times"]), 1)
        self.assertEqual(len(out["modes"]), 1)
        self.assertFalse(np.isfinite(out["states"][0][0]))
        self.assertEqual(out["times"][0], 0.0)
        self.assertIsNone(out["rejected_t"])

    def test_nonfinite_initial_derivative_keeps_sample(self) -> None:
        llc = _BlowLlc()
        llc.blow = True
        ap = _HoldThenBlow(llc)
        out = run_sim(ap, llc.xequil.copy(), t_end=1.0, step=1 / 30)
        self.assertEqual(len(out["states"]), 1)
        self.assertEqual(len(out["modes"]), 1)
        self.assertEqual(out["modes"][0], "hold")
        self.assertEqual(out["times"], [0.0])
        self.assertTrue(np.all(np.isfinite(out["states"][0])))

    def test_mode_rebuild_nonfinite_derivative_keeps_samples(self) -> None:
        llc = _BlowLlc()
        ap = _HoldThenBlow(llc)
        out = run_sim(ap, llc.xequil.copy(), t_end=1.0, step=1 / 30)
        self.assertGreaterEqual(len(out["states"]), 2)
        self.assertEqual(len(out["times"]), len(out["states"]))
        self.assertEqual(len(out["modes"]), len(out["states"]))
        self.assertEqual(out["modes"][-1], "blow")
        self.assertTrue(np.all(np.isfinite(out["states"][-1])))

    def test_inflight_nonfinite_records_time_and_drops_sample(self) -> None:
        llc = _BlowInFlight()
        ap = _Hold(llc)
        out = run_sim(ap, llc.xequil.copy(), t_end=1.0, step=1 / 30)
        self.assertEqual(len(out["states"]), 1)
        self.assertTrue(np.all(np.isfinite(out["states"][0])))
        self.assertIsNotNone(out["rejected_t"])
        self.assertGreater(float(out["rejected_t"]), 0.0)

    def test_ground_contact_initial_state_keeps_sample(self) -> None:
        llc = F16Llc()
        x0 = llc.xequil.copy()
        x0[11] = 0.0
        ap = StraightLevelAutopilot(0.0, llc.xequil[0], llc=llc)
        out = run_sim(ap, x0, t_end=1.0, step=1 / 30)
        self.assertEqual(len(out["states"]), 1)
        self.assertLessEqual(float(out["states"][0][11]), 0.0)
        self.assertLessEqual(out["min_h_ft"], 0.0)
        self.assertEqual(len(out["modes"]), 1)
        self.assertEqual(out["times"], [0.0])

    def test_ground_contact_nonfinite_derivative_keeps_sample(self) -> None:
        llc = _BlowLlc()
        llc.blow = True
        x0 = llc.xequil.copy()
        x0[11] = 0.0
        ap = _HoldThenBlow(llc)
        out = run_sim(ap, x0, t_end=1.0, step=1 / 30)
        self.assertEqual(len(out["states"]), 1)
        self.assertLessEqual(float(out["states"][0][11]), 0.0)
        self.assertLessEqual(out["min_h_ft"], 0.0)
        self.assertEqual(len(out["modes"]), 1)
