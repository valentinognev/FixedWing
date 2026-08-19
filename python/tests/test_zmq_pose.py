#!/usr/bin/env python3
"""Unit tests for the pose ZMQ channel (Gazebo world ENU, streamed continuously)."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.zmq_bus import TOPIC_POSE, PoseSample, recv_pose, send_pose


class TestPoseZmq(unittest.TestCase):
    def test_send_pose_payload(self) -> None:
        sock = MagicMock()
        send_pose(sock, PoseSample(stamp=12.5, x=1.0, y=2.0, z=3.0))
        sock.send_multipart.assert_called_once()
        topic, raw = sock.send_multipart.call_args[0][0]
        self.assertEqual(topic, TOPIC_POSE)
        data = json.loads(raw.decode("utf-8"))
        self.assertEqual(data, {"stamp": 12.5, "x": 1.0, "y": 2.0, "z": 3.0})

    def test_recv_pose_roundtrip(self) -> None:
        payload = json.dumps(
            {"stamp": 5.0, "x": -10.5, "y": 20.25, "z": 451.7}, separators=(",", ":")
        ).encode("utf-8")
        sock = MagicMock()
        sock.recv_multipart.return_value = [TOPIC_POSE, payload]
        sample = recv_pose(sock)
        assert sample is not None
        self.assertEqual(sample, PoseSample(stamp=5.0, x=-10.5, y=20.25, z=451.7))
        self.assertEqual(sample.as_enu(), (-10.5, 20.25, 451.7))

    def test_recv_pose_eagain_returns_none(self) -> None:
        import zmq

        sock = MagicMock()
        sock.recv_multipart.side_effect = zmq.Again()
        self.assertIsNone(recv_pose(sock))


if __name__ == "__main__":
    unittest.main()
