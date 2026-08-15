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


class TestConflateMultipart(unittest.TestCase):
    def test_connect_sub_default_no_conflate(self) -> None:
        """libzmq CONFLATE + multipart aborts: Assertion failed: !_more (fq.cpp:80)."""
        text = (_PYTHON_ROOT / "fw_sitl" / "zmq_bus.py").read_text(encoding="utf-8")
        self.assertIn("conflate: bool = False", text)
        self.assertNotIn("conflate: bool = True", text)


if __name__ == "__main__":
    unittest.main()
