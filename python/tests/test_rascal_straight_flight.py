#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.cli_common import add_common_args

_JSB = _PYTHON_ROOT / "run_straight_flight_jsbsim.py"
_YAS = _PYTHON_ROOT / "run_straight_flight_yasim.py"
_CORE = _PYTHON_ROOT / "fw_sitl" / "straight_flight_core.py"


class TestRascalStraightFlightAttitude(unittest.TestCase):
    def test_jsbsim_defaults_attitude_and_forwards_cmd_mode(self) -> None:
        text = _JSB.read_text(encoding="utf-8")
        self.assertIn("run_locked_line_hold", text)
        self.assertIn("cmd_mode=args.cmd_mode", text)
        self.assertIn('parser.set_defaults(cmd_mode="attitude")', text)

    def test_jsbsim_skips_reboot_so_in_air_spawn_can_arm(self) -> None:
        # Live JSBSim headless: reboot → airspeed selector down / ekf2 missing
        # → "Arming denied: Resolve system health failures first" while falling.
        text = _JSB.read_text(encoding="utf-8")
        self.assertIn("skip_reboot=True", text)

    def test_jsbsim_headless_uses_in_air_attach_arm_policy(self) -> None:
        # PX4 v1.17: MAVLink 21196 still runs health checks. EKF local-pos
        # probation is ~1s after xy_valid; 12s + sim-reset never lets it converge.
        text = _JSB.read_text(encoding="utf-8")
        self.assertIn("arm_timeout_s=60.0", text)
        self.assertNotIn("arm_timeout_s=60.0 if args.viz else 12.0", text)
        self.assertIn("accept_unhealthy=True", text)
        self.assertIn("full_sim_restart=False", text)
        self.assertIn("max_attempts=1", text)

    def test_yasim_defaults_attitude_and_forwards_cmd_mode(self) -> None:
        text = _YAS.read_text(encoding="utf-8")
        self.assertIn("run_locked_line_hold", text)
        self.assertIn("cmd_mode=args.cmd_mode", text)
        self.assertIn('parser.set_defaults(cmd_mode="attitude")', text)

    def test_yasim_skips_reboot_so_in_air_spawn_can_arm(self) -> None:
        text = _YAS.read_text(encoding="utf-8")
        self.assertIn("skip_reboot=True", text)

    def test_locked_line_hold_can_skip_reboot(self) -> None:
        text = _CORE.read_text(encoding="utf-8")
        self.assertIn("skip_reboot: bool = False", text)
        self.assertIn("if not skip_reboot:", text)
        self.assertIn("Skipping autopilot reboot", text)

    def test_prepare_sitl_arming_uses_px4_supply_circuit_breaker_name(self) -> None:
        # PX4 v1.17 param is CBRK_SUPPLY_CHK. The old CBRK_SUPPLYCHK name is
        # silently ignored → "system power unavailable" blocks MAVLink arm
        # (external 21196 does not skip health checks).
        mav = (_PYTHON_ROOT / "fw_sitl" / "mavlink_io.py").read_text(encoding="utf-8")
        self.assertIn('"CBRK_SUPPLY_CHK"', mav)
        self.assertNotIn('"CBRK_SUPPLYCHK"', mav)

    def test_cmd_mode_choices_include_rates(self) -> None:
        """--cmd-mode rates must be reachable; straight_flight_core already
        dispatches it (cmd_mode in ("attitude", "rates"))."""
        parser = argparse.ArgumentParser()
        add_common_args(parser, default_sim=Path("dummy.sh"))
        args = parser.parse_args(["--cmd-mode", "rates"])
        self.assertEqual(args.cmd_mode, "rates")
        default_args = parser.parse_args([])
        self.assertEqual(default_args.cmd_mode, "velocity")

    def test_cmd_mode_rates_calls_send_attitude_rates(self) -> None:
        """Source-contract (as in test_hold_sends_angles_not_raw_quat):
        cmd_mode == "rates" must dispatch to send_attitude_rates."""
        text = _CORE.read_text(encoding="utf-8")
        self.assertIn('cmd_mode == "rates"', text)
        self.assertIn(
            "send_attitude_rates(master, *out.body_rates, thrust)", text
        )


if __name__ == "__main__":
    unittest.main()
