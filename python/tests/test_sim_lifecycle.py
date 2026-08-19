#!/usr/bin/env python3
"""start_sim must surface runner death instead of swallowing it."""

from __future__ import annotations

import stat
import sys
import tempfile
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.sim_lifecycle import start_sim


class TestStartSimFailFast(unittest.TestCase):
    def test_raises_with_log_when_runner_exits_immediately(self) -> None:
        """A docker-image / --gpus failure must not become a 180s heartbeat timeout."""
        with tempfile.TemporaryDirectory() as tmp:
            script = Path(tmp) / "fail_sim.sh"
            script.write_text(
                "#!/bin/bash\n"
                "if [[ \"${1:-}\" == --kill ]]; then exit 0; fi\n"
                "echo 'Error: docker run failed for px4-noble-sim-ros:latest (--gpus all).' >&2\n"
                "exit 1\n",
                encoding="utf-8",
            )
            script.chmod(script.stat().st_mode | stat.S_IEXEC)
            with self.assertRaises(RuntimeError) as ctx:
                start_sim(script)
            msg = str(ctx.exception)
            self.assertIn("exited", msg.lower())
            self.assertIn("px4-noble-sim-ros", msg)
            self.assertIn("gpus", msg.lower())

    def test_returns_popen_when_runner_stays_alive(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            script = Path(tmp) / "ok_sim.sh"
            script.write_text(
                "#!/bin/bash\n"
                'if [[ "${1:-}" == --kill ]]; then exit 0; fi\n'
                "sleep 10\n",
                encoding="utf-8",
            )
            script.chmod(script.stat().st_mode | stat.S_IEXEC)
            proc = start_sim(script)
            try:
                self.assertIsNone(proc.poll())
            finally:
                proc.kill()
                proc.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
