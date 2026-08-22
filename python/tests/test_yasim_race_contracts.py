#!/usr/bin/env python3
from __future__ import annotations

import re
import subprocess
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_CTL = _PYTHON_ROOT / "run_balloon_control.py"
_RACE = _PYTHON_ROOT / "scripts" / "run_balloon_race.sh"
_YASIM_SIM = _PYTHON_ROOT / "scripts" / "runSimYasimRascal.sh"
_KILL = _PYTHON_ROOT / "scripts" / "kill.sh"


class TestYasimControlContracts(unittest.TestCase):
    def test_control_has_yasim_plant_wiring(self) -> None:
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn('add_argument("--yasim"', ctl)
        self.assertIn("runSimYasimRascal.sh", ctl)
        self.assertIn('kill_target = "--fg"', ctl)
        self.assertIn("args.yasim", ctl)
        self.assertIn("args.no_sim or args.viz or args.gz or args.yasim or args.xplane", ctl)
        self.assertIn("args.spawn_fg_balloons or args.viz or args.yasim", ctl)
        self.assertIn("--viz, --gz, --yasim, and --xplane are mutually exclusive", ctl)

    def test_gt_pose_z_uses_ekf_settle_datum_not_balloon_z(self) -> None:
        """Live 094707: ~40 m phantom ΔD on visual hits.

        Balloons rebase to ``z_hold_true`` (MSL vs balloon elev). Aircraft GT Z
        must use raw ``z_hold`` at ``ac_ft_at_settle`` — using ``z_hold_true`` as
        that datum double-counts (ac−balloon) and reports ~40 m vertical miss
        when the plane is co-altitude with the model.
        """
        if str(_PYTHON_ROOT) not in sys.path:
            sys.path.insert(0, str(_PYTHON_ROOT))
        import run_balloon_control as ctl

        z_hold = 26.6
        z_hold_true = -12.2  # balloon0 after (ac_ft - elev_ft)*0.3048
        ac_ft = 3000.0
        ft = 0.3048
        balloon_ft = ac_ft - (z_hold_true - z_hold) / ft
        # Co-altitude with balloon → GT NED Z must equal balloon target.
        raw = (47.46, 8.55, balloon_ft, 0.0, 0.0, 90.0)
        pos, _att = ctl._gt_pose_from_telnet_raw(
            raw,
            ac_ft_at_settle=ac_ft,
            z_ref_at_settle=z_hold,
            ft_to_m=ft,
        )
        self.assertIsNotNone(pos)
        assert pos is not None
        self.assertAlmostEqual(pos[2], z_hold_true, places=5)
        # Settle altitude → GT Z equals EKF z_hold (not balloon z).
        raw_settle = (47.46, 8.55, ac_ft, 0.0, 0.0, 90.0)
        pos_s, _ = ctl._gt_pose_from_telnet_raw(
            raw_settle,
            ac_ft_at_settle=ac_ft,
            z_ref_at_settle=z_hold,
            ft_to_m=ft,
        )
        assert pos_s is not None
        self.assertAlmostEqual(pos_s[2], z_hold, places=5)

    def test_gt_pose_call_sites_pass_z_hold_datum(self) -> None:
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn("z_ref_at_settle=", ctl)
        # Must not feed balloon-frame z_hold_true into the aircraft GT datum.
        self.assertNotRegex(
            ctl,
            r"_gt_pose_from_telnet_raw\([^)]*z_hold_true\s*=\s*z_hold_true",
        )

    def test_fg_replace_skips_stale_elevation_delta(self) -> None:
        """After settle re-place, balloon0 is at AC MSL — do not trust elev-ft."""
        ctl = _CTL.read_text(encoding="utf-8")
        # Successful re-place must force z_hold_true = z_hold.
        self.assertIn("if placed_origin is not None:", ctl)
        block = ctl[ctl.index("if placed_origin is not None:") :]
        block = block[: block.index("else:")]
        self.assertIn("z_hold_true = z_hold", block)
        self.assertNotIn("elevation-ft", block)

    def test_ekf_fix_gps_is_rejected(self) -> None:
        r = subprocess.run(
            [sys.executable, str(_CTL), "--ekf-fix", "gps"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)
        self.assertIn("disabled", r.stderr)
        self.assertIn("0.35.1", r.stderr)

    def test_control_help_lists_yasim(self) -> None:
        r = subprocess.run(
            [sys.executable, str(_CTL), "--help"],
            capture_output=True,
            text=True,
            check=True,
        )
        self.assertIn("--yasim", r.stdout)


class TestYasimSimFanoutAndBalloons(unittest.TestCase):
    def test_yasim_sim_mavlink_and_balloons(self) -> None:
        text = _YASIM_SIM.read_text(encoding="utf-8")
        self.assertRegex(text, r'MAVLINK_FANOUT="\$\{MAVLINK_FANOUT:-0\}"')
        self.assertIn("--mavlink-server", text)
        self.assertIn("--no-mavlink-server", text)
        self.assertIn("start_mavlink_fanout", text)
        self.assertIn("ensure_host_mavlink_server", text)
        self.assertIn("fetch_mavlink_server.sh", text)
        self.assertIn("mavlink fan-out requested (MAVLINK_FANOUT=1) but failed to start", text)
        self.assertIn("MAVLINK_SERVER_LOG:-/tmp/mavlink-server-fanout.log", text)
        self.assertIn("--mavlink-heartbeat-frequency 0", text)
        self.assertIn("/opt/fixedwing/balloons", text)
        self.assertIn("Models/FixedWing", text)
        self.assertIn("--setup", text)
        self.assertIn("fw_sitl.spawn_ic", text)
        self.assertIn("balloon_*.xml", text)
        self.assertIn("--gpus all", text)
        self.assertNotRegex(
            text,
            r"docker run[^\n]*mavlink-server[^\n]*>/dev/null 2>&1",
        )

    def test_fg_patch_allows_nasal_and_telnet(self) -> None:
        patch = (
            _PYTHON_ROOT.parent / "Dockerfiles" / "patch_px4_flightgear_sitl.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("--allow-nasal-from-sockets", patch)
        self.assertIn("--telnet=5501", patch)

    def test_fg_patch_disables_clouds(self) -> None:
        patch = (
            _PYTHON_ROOT.parent / "Dockerfiles" / "patch_px4_flightgear_sitl.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("--disable-clouds", patch)
        self.assertIn("--disable-clouds3d", patch)
        self.assertIn("--disable-real-weather-fetch", patch)

    def test_jsbsim_viz_patch_disables_clouds(self) -> None:
        patch = (
            _PYTHON_ROOT.parent / "Dockerfiles" / "patch_px4_jsbsim_fg_viz.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("FIXEDWING_JSBSIM_FG_VIZ_V7", patch)
        self.assertIn("--disable-clouds", patch)
        self.assertIn("--disable-clouds3d", patch)
        self.assertIn("--disable-real-weather-fetch", patch)
        self.assertIn("draw-mask/clouds=false", patch)

    def test_kill_fg_removes_mavlink_sidecar(self) -> None:
        text = _KILL.read_text(encoding="utf-8")
        self.assertIn("kill_fg_stack", text)
        fg_block = text[text.index("--fg)"): text.index("--jsbsim)")]
        self.assertIn("kill_fg_stack", fg_block)
        all_block = text[text.index("--all)"):]
        self.assertIn("kill_fg_stack", all_block)
        self.assertIn("${FG_NAME}-mavlink", text)


class TestYasimRaceLauncher(unittest.TestCase):
    def test_race_yasim_wiring(self) -> None:
        text = _RACE.read_text(encoding="utf-8")
        self.assertIn("--yasim", text)
        self.assertIn("runSimYasimRascal.sh", text)
        self.assertIn('MODE="fg"', text)
        self.assertIn('CTL_CMD+=" --yasim', text)
        self.assertIn("--spawn-fg-balloons", text)
        self.assertIn("BALLOON_RACE_DURATION", text)
        self.assertIn('CTL_CMD+=" --duration ${DURATION}"', text)
        self.assertIn("resolve_race_sim", text)
        self.assertIn("sim.platform", text)
        self.assertIn("--stop-sim-on-exit", text)
        self.assertIn("kill.sh", text)
        self.assertIn("--all", text)
        self.assertIn("px4-noble-sim-ros", text)

    def test_yasim_and_gz_exit_2(self) -> None:
        r = subprocess.run(
            ["bash", str(_RACE), "--yasim", "--gz"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)

    def test_yasim_and_viz_exit_2(self) -> None:
        r = subprocess.run(
            ["bash", str(_RACE), "--yasim", "--viz"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)

    def test_model_with_yasim_exit_2(self) -> None:
        r = subprocess.run(
            ["bash", str(_RACE), "--yasim", "--model", "rc_cessna"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)
        self.assertRegex(r.stderr.lower(), r"model|--model")

    def test_ekf_fix_gps_exit_2(self) -> None:
        r = subprocess.run(
            ["bash", str(_RACE), "--ekf-fix", "gps"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)
        self.assertIn("disabled", r.stderr)


class TestRascalRaceDocs(unittest.TestCase):
    def test_readme_names_yasim_race_and_angle_commands(self) -> None:
        readme = (_PYTHON_ROOT.parent / "README.md").read_text(encoding="utf-8")
        self.assertIn("--yasim", readme)
        self.assertIn("Euler", readme)
        self.assertIn("quaternion", readme.lower())

    def test_updates_has_0_21_0(self) -> None:
        updates = (_PYTHON_ROOT.parent / "UPDATES.md").read_text(encoding="utf-8")
        self.assertRegex(updates, re.compile(r"^## 0\.21\.0 ", re.M))


if __name__ == "__main__":
    unittest.main()
