#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_RACE = _PYTHON_ROOT / "scripts" / "run_balloon_race.sh"
_CTL = _PYTHON_ROOT / "run_balloon_control.py"
_MAV = _PYTHON_ROOT / "fw_sitl" / "mavlink_io.py"


class TestGzRaceContracts(unittest.TestCase):
    def test_race_gz_wiring(self) -> None:
        text = _RACE.read_text(encoding="utf-8")
        self.assertIn("--gz", text)
        self.assertIn("runSimGzPlane.sh", text)
        self.assertIn("--mode gz", text)
        self.assertIn("--spawn-gz-balloons", text)
        self.assertIn("--viz and --gz are mutually exclusive", text)
        self.assertIn('CTL_CMD+=" --no-sim"', text)
        self.assertIn("--setup", text)
        self.assertIn("px4-noble-gz-plane", text)
        self.assertIn('IMG_CMD+=" --container ${CONTAINER_NAME}"', text)
        self.assertIn('CTL_CMD+=" --gz-container ${CONTAINER_NAME}"', text)
        self.assertIn("import cv2, numpy, zmq, pymavlink", text)
        # Unsetting conda LD_LIBRARY_PATH mixes libzmq → camera fq.cpp abort.
        self.assertNotIn("unset LD_LIBRARY_PATH", text)
        self.assertIn("PYTHONUNBUFFERED=1", text)
        self.assertIn("${PYTHON} -u", text)

    def test_control_uses_polled_attitude_for_chase(self) -> None:
        """history.poll drains ATTITUDE; chase must use last_att_rad, not (0,0,0)."""
        ctl = _CTL.read_text(encoding="utf-8")
        poll_at = ctl.index("pos = history.poll(master)")
        use_at = ctl.index("att = history.last_att_rad")
        self.assertGreater(use_at, poll_at)
        self.assertIn("in_view=last_track_in_view", ctl)
        self.assertIn("z_target=race.balloon_ned()[2]", ctl)
        self.assertIn("approach = (history.last_vx, history.last_vy, 0.0)", ctl)

    def test_control_gz_after_engage(self) -> None:
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn("spawn_balloons_gz(", ctl)
        self.assertGreater(
            ctl.index("spawn_balloons_gz("),
            ctl.index("engage_offboard_with_retries"),
        )
        # Do not gate engage on airspeed: tmux heartbeat wait already stalls the
        # unarmed Cessna (spawn velocity is one-shot). Straight flight engages ASAP.
        self.assertNotIn("wait_min_airspeed(master)", ctl)
        self.assertIn("--spawn-gz-balloons", ctl)
        self.assertIn("accept_unhealthy", ctl)
        # After engage, drop airspeed SP so TECS does not underspeed-dive at ~14 m/s.
        self.assertIn("change_airspeed(master, aspd_sp)", ctl)
        self.assertIn('set_param(master, "FW_AIRSPD_TRIM", aspd_sp)', ctl)
        self.assertGreater(
            ctl.index("GZ: airspeed SP"),
            ctl.index("engage_offboard_with_retries"),
        )

    def test_wait_min_airspeed_exists(self) -> None:
        mav = _MAV.read_text(encoding="utf-8")
        self.assertIn("def wait_min_airspeed", mav)
        self.assertIn("in-air spawn has no airspeed", mav)
        # PX4 1.17 dropped FW_ARSP_MODE; do not set the dead param.
        self.assertNotIn('("FW_ARSP_MODE"', mav)

    def test_wait_min_airspeed_importable(self) -> None:
        if str(_PYTHON_ROOT) not in sys.path:
            sys.path.insert(0, str(_PYTHON_ROOT))
        from fw_sitl.mavlink_io import wait_min_airspeed

        self.assertTrue(callable(wait_min_airspeed))

    def test_wait_min_airspeed_nan_falls_back_to_groundspeed(self) -> None:
        """Gazebo VFR_HUD often has airspeed=NaN; spec is airspeed else groundspeed."""
        if str(_PYTHON_ROOT) not in sys.path:
            sys.path.insert(0, str(_PYTHON_ROOT))
        from fw_sitl.mavlink_io import wait_min_airspeed

        class _FakeMav:
            def command_long_send(self, *args, **kwargs):
                pass

        class _FakeMaster:
            def __init__(self) -> None:
                self.mav = _FakeMav()
                self.target_system = 1
                self.target_component = 1
                self._msgs = [
                    SimpleNamespace(airspeed=float("nan"), groundspeed=28.0),
                ]

            def recv_match(self, type=None, blocking=True, timeout=0.5):
                if self._msgs:
                    return self._msgs.pop(0)
                return None

        got = wait_min_airspeed(_FakeMaster(), min_mps=15.0, timeout_s=1.0)
        self.assertAlmostEqual(got, 28.0)

    def test_viz_and_gz_exit_2(self) -> None:
        r = subprocess.run(
            ["bash", str(_RACE), "--viz", "--gz"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)
        self.assertIn("--viz and --gz are mutually exclusive", r.stderr)

    def test_model_without_gz_exit_2(self) -> None:
        r = subprocess.run(
            ["bash", str(_RACE), "--model", "rc_cessna"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)


if __name__ == "__main__":
    unittest.main()
