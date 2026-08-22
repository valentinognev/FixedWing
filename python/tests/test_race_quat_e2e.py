#!/usr/bin/env python3
"""Live SITL e2e: race_quat on each sim.platform (opt-in).

Skip unless ``FW_SITL_E2E=1``. Needs Docker SITL images, tmux, and for
``viz``/``yasim`` a working DISPLAY/FlightGear; ``gz`` needs the Gazebo plant image.

Run::

    FW_SITL_E2E=1 cd python && python3 -m unittest tests.test_race_quat_e2e -v

Or::

    ./python/scripts/run_race_quat_e2e.sh
"""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_setup import KNOWN_SIM_PLATFORMS, load_flight_setup
from fw_sitl.race_e2e import (
    default_e2e_platforms,
    e2e_enabled,
    run_race_quat_platform_e2e,
    write_race_quat_e2e_setup,
)


def _duration_s() -> float:
    raw = os.environ.get("FW_SITL_E2E_DURATION_S", "90").strip()
    return float(raw)


def _min_passes() -> int:
    return int(os.environ.get("FW_SITL_E2E_MIN_PASSES", "1").strip() or "1")


def _platforms() -> tuple[str, ...]:
    raw = os.environ.get("FW_SITL_E2E_PLATFORMS", "").strip()
    if not raw:
        return default_e2e_platforms()
    return tuple(p.strip() for p in raw.split(",") if p.strip())


class TestRaceQuatE2ESetup(unittest.TestCase):
    """Always-on: materialize setup files (no Docker)."""

    def test_write_setup_sets_race_quat_and_platform(self) -> None:
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            for platform in sorted(KNOWN_SIM_PLATFORMS):
                with self.subTest(platform=platform):
                    path = write_race_quat_e2e_setup(
                        Path(tmp) / f"{platform}.json",
                        platform=platform,
                        duration_s=45.0,
                    )
                    setup = load_flight_setup(path)
                    self.assertEqual(setup.sim.platform, platform)
                    self.assertEqual(setup.sim.duration_s, 45.0)
                    self.assertEqual(setup.guidance.controller, "race_quat")
                    self.assertEqual(setup.guidance.cmd_mode, "attitude")
                    self.assertEqual(setup.guidance.laps, 0)

    def test_default_platforms_match_menu(self) -> None:
        self.assertEqual(set(default_e2e_platforms()), set(KNOWN_SIM_PLATFORMS))


@unittest.skipUnless(e2e_enabled(), "set FW_SITL_E2E=1 to run live SITL races")
class TestRaceQuatLiveE2E(unittest.TestCase):
    """Actual simulation runs — sequential to avoid docker/tmux clashes."""

    def test_race_quat_live_per_platform(self) -> None:
        duration = _duration_s()
        min_passes = _min_passes()
        slack = float(os.environ.get("FW_SITL_E2E_WAIT_SLACK_S", "240").strip() or "240")
        failures: list[str] = []
        for platform in _platforms():
            with self.subTest(platform=platform):
                try:
                    csv_path = run_race_quat_platform_e2e(
                        platform,
                        duration_s=duration,
                        min_passes=min_passes,
                        wait_slack_s=slack,
                    )
                    self.assertTrue(csv_path.is_file(), csv_path)
                except unittest.SkipTest:
                    raise
                except Exception as exc:  # noqa: BLE001 — collect per-platform
                    failures.append(f"{platform}: {exc}")
                    self.fail(f"{platform}: {exc}")
        if failures:
            self.fail("; ".join(failures))


if __name__ == "__main__":
    unittest.main()
