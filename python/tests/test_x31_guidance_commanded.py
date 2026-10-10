"""`run_commanded`, the commanded runner behind `run_guided`.

The npz beside this file was recorded from `run_guided` BEFORE the
`run_commanded` extraction, so `test_run_guided_output_unchanged` and
`test_run_guided_matches_every_stored_column` are the oracle that the extraction
left the guided missions bit-identical: the brief's version spot-checks five
columns, the second compares every one the recording stored, so a perturbation
of `aileron_cmd_deg`, `psi`, `stop_reason` or the `modes` list fails. The rest
pin what `run_commanded` adds on top of it: the body rates, the per-port-field
surface offset, and the actuator initial outputs.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PY = Path(__file__).resolve().parents[1]
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import numpy as np  # noqa: E402

import x31_numpy_compat  # noqa: E402,F401  restores numpy short trig aliases before x31

from x31_guidance import _initial, run_commanded, run_guided  # noqa: E402

DATA = _PY / "tests" / "data"
_DURATION_S = 1.0
_STEP_S = 0.1
_HOLD_S = 0.1


def _const_command(t: float, sensed: dict, dt: float) -> dict:
    """The demand the commanded runner hands the controller."""
    del t, sensed, dt
    return {"V": 50.0, "Chi": 0.0, "Gamma": 0.0}


class TestRunCommanded(unittest.TestCase):
    def test_run_guided_output_unchanged(self) -> None:
        ref = np.load(DATA / "x31_guided_ndi_2s.npz", allow_pickle=True)
        out = run_guided("ndi", "gcas_upright", duration=2.0, step=0.5)
        self.assertEqual(set(out), set(ref["keys"]))
        for key in ("t", "vt_mps", "alpha", "theta", "canard_cmd_deg"):
            np.testing.assert_array_equal(out[key], ref[key])

    def test_run_guided_matches_every_stored_column(self) -> None:
        """Every column of the pre-extraction recording, not a spot check.

        The recording stores all of them, so an edit that perturbs a surface
        command, `psi`, `stop_reason` or the `modes` list has nowhere to hide.
        """
        ref = np.load(DATA / "x31_guided_ndi_2s.npz", allow_pickle=True)
        out = run_guided("ndi", "gcas_upright", duration=2.0, step=0.5)
        self.assertEqual(set(out), set(ref["keys"]))
        for key in ref["keys"]:
            np.testing.assert_array_equal(
                np.asarray(out[key]), ref[key], err_msg=str(key)
            )

    def test_constant_command_logs_rates(self) -> None:
        out = run_commanded(
            "ndi", _initial("gcas_upright"), _DURATION_S, _const_command,
            step=_STEP_S, hold_s=_HOLD_S,
        )
        self.assertGreater(out["t"].size, 1)
        for key in ("p", "q", "r"):
            self.assertEqual(out[key].shape, out["t"].shape)

    def test_surface_offset_reaches_the_command(self) -> None:
        offset = lambda t: {"rudder": 2.0} if t >= 0.5 else {}  # noqa: E731
        plain = run_commanded(
            "ndi", _initial("gcas_upright"), _DURATION_S, _const_command,
            step=_STEP_S, hold_s=_HOLD_S,
        )
        offset_run = run_commanded(
            "ndi", _initial("gcas_upright"), _DURATION_S, _const_command,
            step=_STEP_S, hold_s=_HOLD_S, surface_offset=offset,
        )
        i = int(np.searchsorted(plain["t"], 0.6))
        # 0.5 deg allows for the feedback the offset itself provokes within
        # 0.1 s; the pre-onset test below is the tight one.
        self.assertAlmostEqual(
            offset_run["rudder_cmd_deg"][i] - plain["rudder_cmd_deg"][i], 2.0, delta=0.5
        )

    def test_offset_before_onset_changes_nothing(self) -> None:
        offset = lambda t: {"rudder": 2.0} if t >= 0.5 else {}  # noqa: E731
        plain = run_commanded(
            "ndi", _initial("gcas_upright"), _DURATION_S, _const_command,
            step=_STEP_S, hold_s=_HOLD_S,
        )
        offset_run = run_commanded(
            "ndi", _initial("gcas_upright"), _DURATION_S, _const_command,
            step=_STEP_S, hold_s=_HOLD_S, surface_offset=offset,
        )
        before = plain["t"] < 0.5
        self.assertGreater(int(np.count_nonzero(before)), 1)
        for key, values in plain.items():
            if isinstance(values, np.ndarray) and values.shape == plain["t"].shape:
                np.testing.assert_array_equal(
                    values[before], offset_run[key][before], err_msg=key
                )
        count = int(np.count_nonzero(before))
        self.assertEqual(plain["modes"][:count], offset_run["modes"][:count])

    def test_actuator_outputs_initialize_the_surfaces(self) -> None:
        out = run_commanded(
            "ndi", _initial("gcas_upright"), 0.1, _const_command,
            step=0.1, actuator_outputs={"canard": -10.0, "thrust": 40.0},
        )
        self.assertAlmostEqual(out["canard_deg"][0], -10.0, places=9)
        self.assertAlmostEqual(out["thrust_kn"][0], 40.0, places=9)


if __name__ == "__main__":
    unittest.main()
