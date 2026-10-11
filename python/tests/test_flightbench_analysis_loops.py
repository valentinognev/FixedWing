"""Per-gain root locus and Nyquist of the flying controller."""
from __future__ import annotations

import unittest

import numpy as np

from flightbench.analysis.loops import coefficient_charts
from flightbench.common import CHANNELS, PITCH, ROLL, THROTTLE, YAW, LinearModel, TrimPoint
from flightbench.engine import closed_loop_matrix
from flightbench.tasks import build_task
from flightbench.tasks.base import TaskContext

_PITCH = {"kp_theta": 2.0, "ki_theta": 0.5, "kq": 0.5}
_LEAD = {**_PITCH, "lead_zero": 1.0, "lead_pole": 4.0}


def _pitch_model():
    """Task 5's longitudinal plant, with B in the controller's channel order.

    ``closed_loop_matrix`` multiplies ``B`` by the four channel deviations, so
    the pitch column drives ``q`` and the throttle column drives ``vt``.
    """
    states = ("vt", "alpha", "theta", "q", "altitude")
    A = -0.1 * np.eye(5)
    B = np.zeros((5, len(CHANNELS)))
    B[states.index("q"), PITCH] = 1.0
    B[states.index("vt"), THROTTLE] = 1.0
    return LinearModel(A=A, B=B, states=states, inputs=CHANNELS)


def _lat_model():
    """Task 5's lateral plant, with B in the controller's channel order."""
    states = ("beta", "phi", "psi", "p", "r")
    A = -0.1 * np.eye(5)
    B = np.zeros((5, len(CHANNELS)))
    B[states.index("p"), ROLL] = 1.0
    B[states.index("r"), YAW] = 1.0
    return LinearModel(A=A, B=B, states=states, inputs=CHANNELS)


def _trim():
    return TrimPoint(
        x=np.zeros(13), u=np.zeros(4), vt_mps=100.0, altitude_m=0.0,
    )


class _Adapter:
    extras = ()


def _ctx():
    return TaskContext(adapter=_Adapter(), trim=_trim())


def _controller():
    return build_task("short_period_phugoid", {"kq": 2.0}, _ctx()).controller


def _column(branches):
    """One list of ``{real, imag}`` per grid index, across branches."""
    width = len(branches[0])
    return [[branch[index] for branch in branches] for index in range(width)]


class TestLoops(unittest.TestCase):
    def test_kq_nyquist_sign_on_a_diagonal_plant(self):
        # q_dot = -0.1 q + v once kq is removed. c=-K, K=2 → L(0) = 20.
        charts = coefficient_charts("short_period_phugoid", _pitch_model(), _trim(),
                                    {"kq": 2.0}, _ctx())
        ny = charts[0]["nyquist"]
        index = int(np.argmin(np.abs(np.array(ny["omega"]) - 0.01)))
        # L(jω) = 2 / (jω + 0.1). Check ω closest to 0.01 against that value.
        omega = ny["omega"][index]
        expect = 2.0 / (1j * omega + 0.1)
        self.assertAlmostEqual(ny["real"][index], expect.real, places=4)
        self.assertAlmostEqual(ny["imag"][index], expect.imag, places=4)

    def test_locus_at_current_kq_matches_the_direct_matrix(self):
        charts = coefficient_charts("short_period_phugoid", _pitch_model(), _trim(),
                                    {"kq": 2.0}, _ctx())
        direct = np.linalg.eigvals(closed_loop_matrix(_pitch_model(), _trim(), _controller()))
        index = charts[0]["locus"]["values"].index(2.0)
        got = np.array([complex(b[index]["real"], b[index]["imag"]) for b in charts[0]["locus"]["branches"]])
        for lam in direct:
            dist = np.min(np.abs(got - lam))
            self.assertLessEqual(dist, 1e-6 * max(1.0, abs(lam)))

    def test_lead_zero_branches_stay_put_and_lead_pole_tracks(self):
        charts = {c["name"]: c for c in coefficient_charts(
            "lead_pitch", _pitch_model(), _trim(), _LEAD, _ctx())}
        for branch in charts["lead_zero"]["locus"]["branches"]:
            reals = [p["real"] for p in branch]
            self.assertAlmostEqual(max(reals), min(reals), places=6)
        for value, point in zip(charts["lead_pole"]["locus"]["values"],
                                _column(charts["lead_pole"]["locus"]["branches"])):
            self.assertTrue(any(abs(p["real"] + value) < 1e-6 * max(1.0, abs(value))
                                and abs(p["imag"]) < 1e-6 for p in point))
        self.assertEqual(charts["lead_zero"]["nyquist"], charts["kp_theta"]["nyquist"])
        self.assertEqual(charts["lead_pole"]["nyquist"], charts["kp_theta"]["nyquist"])

    def test_tau_w_nyquist_matches_kr(self):
        charts = {c["name"]: c for c in coefficient_charts(
            "dutch_roll", _lat_model(), _trim(), {"kr": 0.5, "tau_w": 1.0}, _ctx())}
        self.assertEqual(charts["tau_w"]["nyquist"], charts["kr"]["nyquist"])

    def test_lead_grid_drops_the_point_where_zero_equals_pole(self):
        # 0.05*2 = 0.1 is the first parameter_grid endpoint. Skipping it removes it.
        gains = {**_LEAD, "lead_zero": 2.0, "lead_pole": 0.1}
        charts = {c["name"]: c for c in coefficient_charts(
            "lead_pitch", _pitch_model(), _trim(), gains, _ctx())}
        self.assertNotIn(0.1, charts["lead_zero"]["locus"]["values"])
        self.assertIn(2.0, charts["lead_zero"]["locus"]["values"])

    def test_kp_and_ki_nyquist_differ(self):
        charts = {c["name"]: c for c in coefficient_charts(
            "pitch_disturbance", _pitch_model(), _trim(), _PITCH, _ctx())}
        self.assertNotEqual(charts["kp_theta"]["nyquist"], charts["ki_theta"]["nyquist"])


if __name__ == "__main__":
    unittest.main()
