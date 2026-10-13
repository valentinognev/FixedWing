"""Controller diagrams for the eleven linear-law tasks."""
from __future__ import annotations

import unittest

from flightbench.analysis.diagram import controller_diagram, format_gain
from flightbench.common import FlightbenchError

_PITCH = {"kp_theta": 2.0, "ki_theta": 0.5, "kq": 0.5}
_LEAD = {**_PITCH, "lead_zero": 1.0, "lead_pole": 4.0}


class TestDiagram(unittest.TestCase):
    def test_pitch_disturbance_has_pi_kq_and_no_lead(self):
        loops = controller_diagram("pitch_disturbance", _PITCH)
        texts = [n["text"] for loop in loops for n in loop["nodes"]]
        self.assertIn("2 + 0.5/s", texts)
        self.assertIn("0.5", texts)
        self.assertIn("+1 deg", texts)
        self.assertFalse(any("lead" in n["label"] for loop in loops for n in loop["nodes"]))
        self.assertTrue(any(n["id"] == "plant" for loop in loops for n in loop["nodes"]))
        self.assertTrue(any(w["sign"] == "-" for loop in loops for w in loop["wires"]))

    def test_each_task_names_its_own_gains(self):
        # Gains below are the registry seeds. Every seed appears as text; a gain
        # the task does not close does not.
        cases = {
            "short_period_phugoid": {"kq": 0.5},
            "lead_pitch": _LEAD,
            "trim_cruise": {**_PITCH, "kp_v": 0.05, "ki_v": 0.005},
            "airspeed": {**_PITCH, "kp_v": 0.05, "ki_v": 0.005},
            "acceleration": {"kp_nz": 0.05, "ki_nz": 0.2, "kq": 0.5, "kp_v": 0.05, "ki_v": 0.005},
            "steady_descent": {"kp_gamma": 1.0, "ki_gamma": 0.2, **_PITCH, "kp_v": 0.05, "ki_v": 0.005},
            "dutch_roll": {"kr": 0.5, "tau_w": 1.0},
            "turn_coordination": {"kp_phi": 2.0, "ki_phi": 0.1, "kp_p": 0.5, "kr": 0.5, "tau_w": 1.0, "k_beta": 1.0, "k_ari": 0.25},
            "sideslip_turn": {"kp_phi": 2.0, "ki_phi": 0.1, "kp_p": 0.5, "kr": 0.5, "tau_w": 1.0, "k_beta": 1.0, "ki_beta": 0.2, "k_betadot": 0.5},
            "yaw_orientation": {"kp_phi": 2.0, "ki_phi": 0.1, "kp_p": 0.5, "k_beta": 1.0, "k_betadot": 0.5, "kp_psi": 0.5, "kr_track": 1.0},
        }
        for task, gains in cases.items():
            texts = " ".join(n["text"] for loop in controller_diagram(task, gains, pitch_ff=0.01, throttle_ff=-0.02, vt0=50.0) for n in loop["nodes"])
            for value in gains.values():
                self.assertIn(format_gain(value), texts, task)
        cruise = controller_diagram("trim_cruise", cases["trim_cruise"], pitch_ff=0.01, throttle_ff=-0.02)
        cruise_text = " ".join(n["text"] for loop in cruise for n in loop["nodes"])
        self.assertIn(format_gain(0.01), cruise_text)
        self.assertIn(format_gain(-0.02), cruise_text)
        yaw = controller_diagram("yaw_orientation", cases["yaw_orientation"], vt0=50.0)
        self.assertIn("±30 deg", " ".join(n["text"] for loop in yaw for n in loop["nodes"]))
        dutch = controller_diagram("dutch_roll", cases["dutch_roll"])
        self.assertIn("s/(s+1/1)", " ".join(n["text"] for loop in dutch for n in loop["nodes"]))
        short = controller_diagram("short_period_phugoid", cases["short_period_phugoid"])
        self.assertNotIn("0.5/s", " ".join(n["text"] for loop in short for n in loop["nodes"]))

    def test_feedback_wires_name_the_measurement(self):
        def named(loop):
            return {(wire["from"], wire["to"]): wire["signal"] for wire in loop["wires"]}

        pitch = controller_diagram("pitch_disturbance", _PITCH)[0]
        self.assertEqual(named(pitch)[("plant", "sum_e")], "theta")
        self.assertEqual(named(pitch)[("plant", "kq")], "q")

        loops = {
            loop["feedback"]: loop for loop in controller_diagram("turn_coordination", {
                "kp_phi": 2.0, "ki_phi": 0.1, "kp_p": 0.5,
                "kr": 0.5, "tau_w": 1.0, "k_beta": 1.0, "k_ari": 0.25,
            })
        }
        self.assertEqual(named(loops["phi"])[("plant", "sum_e")], "phi")
        self.assertEqual(named(loops["phi"])[("plant", "kp_p")], "p")
        self.assertEqual(named(loops["r"])[("plant", "washout")], "r")
        self.assertEqual(named(loops["r"])[("plant", "k_beta")], "beta")

        yaw = {
            loop["feedback"]: loop for loop in controller_diagram("yaw_orientation", {
                "kp_phi": 2.0, "ki_phi": 0.1, "kp_p": 0.5,
                "k_beta": 1.0, "k_betadot": 0.5, "kp_psi": 0.5, "kr_track": 1.0,
            }, vt0=50.0)
        }
        self.assertEqual(named(yaw["psi"])[("plant", "sum_e")], "psi")
        self.assertEqual(named(yaw["r"])[("plant", "sum_r")], "r")
        self.assertEqual(named(yaw["r"])[("plant", "k_betadot")], "betadot")

    def test_forward_wires_name_the_variable(self):
        def named(loop):
            return {(wire["from"], wire["to"]): wire["signal"] for wire in loop["wires"]}

        pitch = named(controller_diagram("pitch_disturbance", _PITCH)[0])
        self.assertEqual(pitch[("theta_ref", "sum_e")], "theta")
        self.assertEqual(pitch[("sum_e", "pi")], "e_theta")
        self.assertEqual(pitch[("pi", "sum_rate")], "pitch")
        self.assertEqual(pitch[("sum_rate", "sum_out")], "pitch")
        self.assertEqual(pitch[("sum_out", "plant")], "pitch")
        self.assertEqual(pitch[("disturbance", "sum_out")], "")
        self.assertEqual(pitch[("kq", "sum_rate")], "")

        lead = named(controller_diagram("lead_pitch", _LEAD)[0])
        self.assertEqual(lead[("theta_ref", "lead")], "theta")
        self.assertEqual(lead[("lead", "sum_e")], "theta")

        speed = {
            loop["feedback"]: named(loop)
            for loop in controller_diagram("airspeed", {**_PITCH, "kp_v": 0.05, "ki_v": 0.005})
        }
        self.assertEqual(speed["vt"][("vt_ref", "sum_e")], "vt")
        self.assertEqual(speed["vt"][("sum_e", "pi")], "e_vt")
        self.assertEqual(speed["vt"][("pi", "plant")], "throttle")

        short = named(controller_diagram("short_period_phugoid", {"kq": 0.5})[0])
        self.assertEqual(short[("sum_out", "plant")], "pitch")
        self.assertEqual(short[("kq", "sum_out")], "")

        yaw = {
            loop["feedback"]: named(loop)
            for loop in controller_diagram("yaw_orientation", {
                "kp_phi": 2.0, "ki_phi": 0.1, "kp_p": 0.5,
                "k_beta": 1.0, "k_betadot": 0.5, "kp_psi": 0.5, "kr_track": 1.0,
            }, vt0=50.0)
        }
        self.assertEqual(yaw["psi"][("psi_ref", "sum_e")], "psi")
        self.assertEqual(yaw["psi"][("sum_e", "kp_psi")], "e_psi")
        self.assertEqual(yaw["phi"][("r_cmd", "coord")], "r")
        self.assertEqual(yaw["phi"][("clip", "sum_e")], "phi")
        self.assertEqual(yaw["phi"][("sum_e", "pi")], "e_phi")
        self.assertEqual(yaw["phi"][("sum_rate", "plant")], "roll")
        self.assertEqual(yaw["r"][("r_cmd", "sum_r")], "r")
        self.assertEqual(yaw["r"][("sum_r", "kr_track")], "e_r")
        self.assertEqual(yaw["r"][("sum_yaw", "plant")], "yaw")
        self.assertEqual(yaw["r"][("k_beta", "sum_yaw")], "")

    def test_unknown_task_raises(self):
        with self.assertRaises(FlightbenchError):
            controller_diagram("not_a_task", {})


if __name__ == "__main__":
    unittest.main()
