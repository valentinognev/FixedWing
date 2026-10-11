"""Open-loop cards for the eleven linear-law tasks."""
from __future__ import annotations

import unittest

import numpy as np

from flightbench.analysis.cards import open_loop_cards
from flightbench.common import LinearModel, TrimPoint

_PITCH = {"kp_theta": 2.0, "ki_theta": 0.5, "kq": 0.5}
_LEAD = {**_PITCH, "lead_zero": 1.0, "lead_pole": 4.0}
_CRUISE = {**_PITCH, "kp_v": 0.05, "ki_v": 0.005}
_DESCENT = {"kp_gamma": 1.0, "ki_gamma": 0.2, **_PITCH, "kp_v": 0.05, "ki_v": 0.005}
_YAW = {
    "kp_phi": 2.0, "ki_phi": 0.1, "kp_p": 0.5,
    "k_beta": 1.0, "k_betadot": 0.5, "kp_psi": 0.5, "kr_track": 1.0,
}
_GAINS = {
    "pitch_disturbance": _PITCH,
    "short_period_phugoid": {"kq": 0.5},
    "acceleration": {"kp_nz": 0.05, "ki_nz": 0.2, "kq": 0.5, "kp_v": 0.05, "ki_v": 0.005},
    "airspeed": {**_PITCH, "kp_v": 0.05, "ki_v": 0.005},
    "dutch_roll": {"kr": 0.5, "tau_w": 1.0},
    "turn_coordination": {
        "kp_phi": 2.0, "ki_phi": 0.1, "kp_p": 0.5, "kr": 0.5,
        "tau_w": 1.0, "k_beta": 1.0, "k_ari": 0.25,
    },
    "sideslip_turn": {
        "kp_phi": 2.0, "ki_phi": 0.1, "kp_p": 0.5, "kr": 0.5,
        "tau_w": 1.0, "k_beta": 1.0, "ki_beta": 0.2, "k_betadot": 0.5,
    },
}


def _trim():
    return TrimPoint(x=np.zeros(13), u=np.zeros(4), vt_mps=100.0, altitude_m=0.0)


def _long_model():
    states = ("vt", "alpha", "theta", "q", "altitude")
    A = -0.1 * np.eye(5)
    B = np.zeros((5, 2))
    B[states.index("q"), 0] = 1.0       # pitch
    B[states.index("vt"), 1] = 1.0      # throttle
    return LinearModel(A=A, B=B, states=states, inputs=("pitch", "throttle"))


def _lat_model():
    states = ("beta", "phi", "psi", "p", "r")
    A = -0.1 * np.eye(5)
    B = np.zeros((5, 2))
    B[states.index("p"), 0] = 1.0       # roll
    B[states.index("r"), 1] = 1.0       # yaw
    return LinearModel(A=A, B=B, states=states, inputs=("roll", "yaw"))


class TestCards(unittest.TestCase):
    def test_pitch_card_is_theta_over_pitch(self):
        card = open_loop_cards("lead_pitch", _long_model(), _trim(), _LEAD)[0]
        self.assertEqual(card["input"], "pitch")
        self.assertEqual(card["output"], "theta")
        self.assertEqual(card["C"][0][card["states"].index("theta")], 1.0)

    def test_gamma_row_is_theta_minus_alpha(self):
        card = open_loop_cards("steady_descent", _long_model(), _trim(), _DESCENT)[0]
        states = card["states"]
        self.assertEqual(card["C"][0][states.index("theta")], 1.0)
        self.assertEqual(card["C"][0][states.index("alpha")], -1.0)
        self.assertEqual(card["D"], [[0.0]])

    def test_trim_cruise_shares_A(self):
        cards = open_loop_cards("trim_cruise", _long_model(), _trim(), _CRUISE)
        self.assertEqual([c["input"] for c in cards], ["pitch", "throttle"])
        self.assertEqual([c["output"] for c in cards], ["theta", "vt"])
        self.assertEqual(cards[0]["A"], cards[1]["A"])

    def test_yaw_card_ignores_kp_psi_and_keeps_the_bank_state(self):
        low = open_loop_cards("yaw_orientation", _lat_model(), _trim(), {**_YAW, "kp_psi": 0.5})[0]
        high = open_loop_cards("yaw_orientation", _lat_model(), _trim(), {**_YAW, "kp_psi": 5.0})[0]
        self.assertEqual(low["states"][-1], "phi_integral")
        self.assertEqual(low["A"], high["A"])
        moved = open_loop_cards("yaw_orientation", _lat_model(), _trim(), {**_YAW, "kp_phi": 9.0})[0]
        self.assertNotEqual(low["A"], moved["A"])
        self.assertEqual(len(low["states"]), 6)

    def test_remaining_channels(self):
        expect = {
            "pitch_disturbance": ("pitch", "theta"),
            "short_period_phugoid": ("pitch", "q"),
            "acceleration": ("pitch", "nz"),
            "airspeed": ("throttle", "vt"),
            "dutch_roll": ("yaw", "r"),
            "turn_coordination": ("roll", "phi"),
            "sideslip_turn": ("roll", "phi"),
        }
        for task, (src, dst) in expect.items():
            model = _long_model() if dst in ("theta", "q", "nz", "vt") else _lat_model()
            card = open_loop_cards(task, model, _trim(), _GAINS[task])[0]
            self.assertEqual((card["input"], card["output"]), (src, dst), task)
            self.assertEqual(card["D"], [[0.0]], task)


if __name__ == "__main__":
    unittest.main()
