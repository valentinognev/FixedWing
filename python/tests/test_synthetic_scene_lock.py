#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_setup import BalloonSpec
from fw_sitl.synthetic_camera import lock_world_balloons
from fw_sitl.zmq_bus import TargetColor


def _template() -> tuple[BalloonSpec, ...]:
    return (
        BalloonSpec(ned=(200.0, 0.0, 30.0), color=(255, 0, 0), diameter_m=10.0),
        BalloonSpec(ned=(200.0, 200.0, -30.0), color=(0, 255, 0), diameter_m=10.0),
    )


class TestLockWorldBalloons(unittest.TestCase):
    def test_none_until_payload(self) -> None:
        self.assertIsNone(
            lock_world_balloons(template=_template(), color=None, locked=None)
        )
        color = TargetColor(r=255, g=0, b=0, balloons_ned=None)
        self.assertIsNone(
            lock_world_balloons(template=_template(), color=color, locked=None)
        )

    def test_freezes_control_ned_keeps_template_color(self) -> None:
        neds = ((200.0, 0.0, 61.7), (200.0, 200.0, 1.7))
        color = TargetColor(r=255, g=0, b=0, balloons_ned=neds)
        locked = lock_world_balloons(
            template=_template(), color=color, locked=None
        )
        assert locked is not None
        self.assertEqual(locked[0].ned, neds[0])
        self.assertEqual(locked[1].ned, neds[1])
        self.assertEqual(locked[0].color, (255, 0, 0))
        self.assertEqual(locked[1].color, (0, 255, 0))

    def test_already_locked_ignores_new_color(self) -> None:
        first = (
            BalloonSpec(ned=(1.0, 2.0, 3.0), color=(255, 0, 0), diameter_m=10.0),
        )
        color = TargetColor(
            r=0, g=255, b=0,
            balloons_ned=((9.0, 9.0, 9.0), (8.0, 8.0, 8.0)),
        )
        out = lock_world_balloons(
            template=_template(), color=color, locked=first
        )
        self.assertIs(out, first)

    def test_length_mismatch_does_not_lock(self) -> None:
        color = TargetColor(r=255, g=0, b=0, balloons_ned=((1.0, 2.0, 3.0),))
        self.assertIsNone(
            lock_world_balloons(template=_template(), color=color, locked=None)
        )


class TestSynthPublisherContracts(unittest.TestCase):
    def test_synth_subscribes_color_and_does_not_rebase_on_first_pose(self) -> None:
        text = (_PYTHON_ROOT / "fw_sitl" / "synthetic_camera.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("ColorSubscriber", text)
        self.assertIn("lock_world_balloons", text)
        self.assertIn("setup.zmq.color", text)
        # First-pose rebase glued synth Z to the falling aircraft.
        pub = text[text.index("def run_synthetic_publisher") :]
        self.assertNotIn("rebase_balloons_to_local_z(setup.balloons, pos[2])", pub)

    def test_control_publishes_balloons_ned(self) -> None:
        ctl = (_PYTHON_ROOT / "run_balloon_control.py").read_text(encoding="utf-8")
        self.assertIn("balloons_ned=", ctl)
        self.assertGreater(ctl.count("balloons_ned="), 2)


if __name__ == "__main__":
    unittest.main()
