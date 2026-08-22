#!/usr/bin/env python3
"""gz_pose_bridge: extracts one named model's ENU pose from a Pose_V-like message."""
from __future__ import annotations

import sys
import unittest
from dataclasses import dataclass
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.platforms.gz.gz_pose_bridge import extract_named_pose, model_name_candidates


@dataclass
class _FakePosition:
    x: float
    y: float
    z: float


@dataclass
class _FakePoseEntry:
    name: str
    position: _FakePosition


@dataclass
class _FakePoseV:
    pose: list[_FakePoseEntry]


class TestGzPoseBridge(unittest.TestCase):
    def test_extract_named_pose_finds_model_among_links(self) -> None:
        # Real dynamic_pose/info messages include the model plus every link
        # (base_link, airspeed, rotor_puller, ...); only the model entry matters.
        msg = _FakePoseV(
            pose=[
                _FakePoseEntry("rc_cessna_0", _FakePosition(14.4777, 356.904, 451.095)),
                _FakePoseEntry("base_link", _FakePosition(0.0, 0.0, 0.0)),
                _FakePoseEntry("rotor_puller", _FakePosition(0.22, 0.0, 0.0)),
            ]
        )
        xyz = extract_named_pose(msg, "rc_cessna_0")
        self.assertEqual(xyz, (14.4777, 356.904, 451.095))

    def test_extract_named_pose_missing_returns_none(self) -> None:
        msg = _FakePoseV(pose=[_FakePoseEntry("balloon_255_0_0", _FakePosition(0, 300, 451))])
        self.assertIsNone(extract_named_pose(msg, "rc_cessna_0"))

    def test_extract_named_pose_accepts_candidate_tuple(self) -> None:
        msg = _FakePoseV(pose=[_FakePoseEntry("rc_cessna", _FakePosition(1.0, 2.0, 3.0))])
        xyz = extract_named_pose(msg, model_name_candidates("rc_cessna"))
        self.assertEqual(xyz, (1.0, 2.0, 3.0))

    def test_model_name_candidates_prefers_suffixed_spawn_name(self) -> None:
        names = model_name_candidates("rc_cessna")
        self.assertEqual(names[0], "rc_cessna_0")
        self.assertIn("rc_cessna", names)

    def test_run_gz_pose_publisher_via_docker_wiring(self) -> None:
        text = (_PYTHON_ROOT / "fw_sitl" / "platforms" / "gz" / "gz_pose_bridge.py").read_text(encoding="utf-8")
        src = text[text.index("def run_gz_pose_publisher_via_docker") :]
        self.assertIn("PYTHONUNBUFFERED=1", src)
        self.assertIn("-u", src)
        self.assertIn("setup.zmq.pose", src)


if __name__ == "__main__":
    unittest.main()
