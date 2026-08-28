#!/usr/bin/env python3
"""Unit tests for color ZMQ message including assisted flag."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.zmq_bus import TOPIC_COLOR, TargetColor, recv_color, send_color


class TestColorAssistedZmq(unittest.TestCase):
    def test_send_color_includes_assisted(self) -> None:
        sock = MagicMock()
        send_color(sock, TargetColor(r=255, g=0, b=0, stamp=1.5, assisted=True))
        sock.send_multipart.assert_called_once()
        topic, raw = sock.send_multipart.call_args[0][0]
        self.assertEqual(topic, TOPIC_COLOR)
        data = json.loads(raw.decode("utf-8"))
        self.assertEqual(data["r"], 255)
        self.assertEqual(data["g"], 0)
        self.assertEqual(data["b"], 0)
        self.assertEqual(data["stamp"], 1.5)
        self.assertTrue(data["assisted"])

    def test_recv_color_reads_assisted(self) -> None:
        payload = json.dumps(
            {"r": 0, "g": 255, "b": 0, "stamp": 2.0, "assisted": True},
            separators=(",", ":"),
        ).encode("utf-8")
        sock = MagicMock()
        sock.recv_multipart.return_value = [TOPIC_COLOR, payload]
        color = recv_color(sock)
        assert color is not None
        self.assertEqual(color.as_tuple(), (0, 255, 0))
        self.assertTrue(color.assisted)
        self.assertEqual(color.stamp, 2.0)

    def test_recv_color_defaults_assisted_false_for_legacy(self) -> None:
        payload = json.dumps(
            {"r": 1, "g": 2, "b": 3, "stamp": 0.0},
            separators=(",", ":"),
        ).encode("utf-8")
        sock = MagicMock()
        sock.recv_multipart.return_value = [TOPIC_COLOR, payload]
        color = recv_color(sock)
        assert color is not None
        self.assertFalse(color.assisted)

    def test_rgb_tuple_publish_defaults_assisted_false(self) -> None:
        sock = MagicMock()
        send_color(sock, (10, 20, 30))
        raw = sock.send_multipart.call_args[0][0][1]
        data = json.loads(raw.decode("utf-8"))
        self.assertEqual((data["r"], data["g"], data["b"]), (10, 20, 30))
        self.assertFalse(data.get("assisted", False))

    def test_send_color_includes_ned_pose(self) -> None:
        sock = MagicMock()
        send_color(
            sock,
            TargetColor(
                r=255,
                g=0,
                b=0,
                stamp=1.5,
                assisted=False,
                t_s=12.0,
                pos_ned=(151.3, -35.6, 47.1),
            ),
        )
        raw = sock.send_multipart.call_args[0][0][1]
        data = json.loads(raw.decode("utf-8"))
        self.assertEqual(data["t_s"], 12.0)
        self.assertEqual(data["pos_n"], 151.3)
        self.assertEqual(data["pos_e"], -35.6)
        self.assertEqual(data["pos_d"], 47.1)

    def test_recv_color_reads_ned_pose(self) -> None:
        payload = json.dumps(
            {
                "r": 0,
                "g": 0,
                "b": 255,
                "stamp": 2.0,
                "assisted": False,
                "t_s": 8.0,
                "pos_n": 10.0,
                "pos_e": 20.0,
                "pos_d": 30.0,
            },
            separators=(",", ":"),
        ).encode("utf-8")
        sock = MagicMock()
        sock.recv_multipart.return_value = [TOPIC_COLOR, payload]
        color = recv_color(sock)
        assert color is not None
        self.assertEqual(color.t_s, 8.0)
        self.assertEqual(color.pos_ned, (10.0, 20.0, 30.0))

    def test_recv_color_defaults_pose_none_for_legacy(self) -> None:
        payload = json.dumps(
            {"r": 1, "g": 2, "b": 3, "stamp": 0.0, "assisted": False},
            separators=(",", ":"),
        ).encode("utf-8")
        sock = MagicMock()
        sock.recv_multipart.return_value = [TOPIC_COLOR, payload]
        color = recv_color(sock)
        assert color is not None
        self.assertIsNone(color.pos_ned)
        self.assertIsNone(color.t_s)

    def test_send_color_includes_balloons_ned(self) -> None:
        sock = MagicMock()
        neds = ((200.0, 0.0, 61.7), (200.0, 200.0, 1.7), (0.0, 200.0, 31.7))
        send_color(
            sock,
            TargetColor(
                r=255, g=0, b=0, stamp=1.0, assisted=False,
                balloons_ned=neds,
            ),
        )
        data = json.loads(sock.send_multipart.call_args[0][0][1].decode("utf-8"))
        self.assertEqual(data["balloons"], [list(p) for p in neds])

    def test_recv_color_reads_balloons_ned(self) -> None:
        payload = json.dumps(
            {
                "r": 255, "g": 0, "b": 0, "stamp": 1.0, "assisted": False,
                "balloons": [[200.0, 0.0, 61.7], [200.0, 200.0, 1.7]],
            },
            separators=(",", ":"),
        ).encode("utf-8")
        sock = MagicMock()
        sock.recv_multipart.return_value = [TOPIC_COLOR, payload]
        color = recv_color(sock)
        assert color is not None
        self.assertEqual(
            color.balloons_ned,
            ((200.0, 0.0, 61.7), (200.0, 200.0, 1.7)),
        )

    def test_send_color_includes_warn(self) -> None:
        sock = MagicMock()
        send_color(
            sock,
            TargetColor(
                r=255,
                g=0,
                b=0,
                stamp=1.5,
                warn="Arming denied: Resolve system health failures first",
            ),
        )
        data = json.loads(sock.send_multipart.call_args[0][0][1].decode("utf-8"))
        self.assertEqual(
            data["warn"],
            "Arming denied: Resolve system health failures first",
        )

    def test_recv_color_reads_warn(self) -> None:
        payload = json.dumps(
            {
                "r": 255,
                "g": 0,
                "b": 0,
                "stamp": 1.0,
                "assisted": False,
                "warn": "Arming denied: Resolve system health failures first",
            },
            separators=(",", ":"),
        ).encode("utf-8")
        sock = MagicMock()
        sock.recv_multipart.return_value = [TOPIC_COLOR, payload]
        color = recv_color(sock)
        assert color is not None
        self.assertEqual(
            color.warn,
            "Arming denied: Resolve system health failures first",
        )

    def test_recv_color_defaults_warn_none_for_legacy(self) -> None:
        payload = json.dumps(
            {"r": 1, "g": 2, "b": 3, "stamp": 0.0, "assisted": False},
            separators=(",", ":"),
        ).encode("utf-8")
        sock = MagicMock()
        sock.recv_multipart.return_value = [TOPIC_COLOR, payload]
        color = recv_color(sock)
        assert color is not None
        self.assertIsNone(color.warn)

    def test_send_color_omits_warn_when_none(self) -> None:
        sock = MagicMock()
        send_color(sock, TargetColor(r=255, g=0, b=0, stamp=1.0))
        data = json.loads(sock.send_multipart.call_args[0][0][1].decode("utf-8"))
        self.assertNotIn("warn", data)

    def test_recv_color_legacy_balloons_ned_none(self) -> None:
        payload = json.dumps(
            {"r": 1, "g": 2, "b": 3, "stamp": 0.0, "assisted": False},
            separators=(",", ":"),
        ).encode("utf-8")
        sock = MagicMock()
        sock.recv_multipart.return_value = [TOPIC_COLOR, payload]
        color = recv_color(sock)
        assert color is not None
        self.assertIsNone(color.balloons_ned)


class TestConflateMultipart(unittest.TestCase):
    def test_connect_sub_default_no_conflate(self) -> None:
        """libzmq CONFLATE + multipart aborts: Assertion failed: !_more (fq.cpp:80)."""
        text = (_PYTHON_ROOT / "fw_sitl" / "zmq_bus.py").read_text(encoding="utf-8")
        self.assertIn("conflate: bool = False", text)
        self.assertNotIn("conflate: bool = True", text)


if __name__ == "__main__":
    unittest.main()
