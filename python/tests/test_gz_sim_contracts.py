#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_REPO = _PYTHON_ROOT.parent
_SIM = _PYTHON_ROOT / "scripts" / "runSimGzPlane.sh"
_KILL = _PYTHON_ROOT / "scripts" / "kill.sh"
_RACE = _PYTHON_ROOT / "scripts" / "run_balloon_race.sh"


def _gz_should_retry_without_gpu(rc: int, elapsed_s: int, used_gpu: str) -> bool:
    text = _SIM.read_text(encoding="utf-8")
    start = text.index("gz_should_retry_without_gpu()")
    end = text.index("\n}", start) + 2
    fn = text[start:end]
    r = subprocess.run(
        [
            "bash",
            "-c",
            fn + f"\ngz_should_retry_without_gpu {rc} {elapsed_s} {used_gpu}",
        ],
        capture_output=True,
        text=True,
    )
    return r.returncode == 0


class TestGzSimContracts(unittest.TestCase):
    def test_sim_script_exists_and_defaults(self) -> None:
        text = _SIM.read_text(encoding="utf-8")
        self.assertIn("px4-noble-gz-plane", text)
        self.assertIn("0,0,500,0,0,1.570796", text)
        self.assertIn("gz_rc_cessna", text)
        self.assertIn("gz_advanced_plane", text)
        self.assertIn("--gpus all", text)
        self.assertIn("PX4_GZ_DOCKER_GPUS", text)
        self.assertIn("retrying without GPU", text)
        self.assertIn("gz_should_retry_without_gpu", text)
        self.assertIn("not restarting", text)
        retry = text[text.index('echo "Starting ${IMAGE_TAG} Gazebo'):]
        self.assertIn("gz_should_retry_without_gpu", retry)
        self.assertIn("not found locally", text)
        self.assertIn("PX4_noble_sim_build.sh", text)
        self.assertIn("GZ_SIM_RESOURCE_PATH", text)
        self.assertIn("apply_plane_overlay", text)
        self.assertRegex(
            text, r"cp -f /tmp/fw_gz_overlay/models/.*/model\.sdf.*\$\{STOCK\}"
        )
        self.assertIn("FW_GZ_SPAWN_VY", text)
        self.assertIn('MAVLINK_FANOUT="${MAVLINK_FANOUT:-0}"', text)
        self.assertIn("--mavlink-heartbeat-frequency 0", text)
        self.assertIn("/opt/fixedwing/python", text)
        self.assertIn("/opt/fixedwing/gz", text)
        self.assertIn("DISPLAY", text)
        self.assertIn("exit 1", text)
        self.assertIn("nvidia-container-toolkit", text)
        self.assertIn("GZ_GUI_CONFIG", text)
        self.assertIn("gz_gui_follow", text)
        self.assertIn("/gui/track", (_PYTHON_ROOT / "fw_sitl" / "gz_gui_follow.py").read_text(encoding="utf-8"))

    def test_kill_gz(self) -> None:
        text = _KILL.read_text(encoding="utf-8")
        self.assertIn("--gz", text)
        self.assertIn("px4-noble-gz-plane", text)
        self.assertIn("kill_gz_stack", text)
        all_block = text[text.index("--all)"):]
        self.assertIn("kill_gz_stack", all_block)

    def test_gpu_fallback_skips_sigkill_and_long_runs(self) -> None:
        """Timed race kill (docker rm -f) must not look like a GPU launch failure."""
        self.assertFalse(_gz_should_retry_without_gpu(137, 5, "gpu"))
        self.assertFalse(_gz_should_retry_without_gpu(143, 5, "gpu"))
        self.assertFalse(_gz_should_retry_without_gpu(1, 90, "gpu"))
        self.assertFalse(_gz_should_retry_without_gpu(125, 2, "nogpu"))
        self.assertTrue(_gz_should_retry_without_gpu(125, 2, "gpu"))
        self.assertTrue(_gz_should_retry_without_gpu(1, 2, "gpu"))


if __name__ == "__main__":
    unittest.main()
