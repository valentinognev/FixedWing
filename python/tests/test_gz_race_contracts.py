#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_RACE = _PYTHON_ROOT / "scripts" / "run_balloon_race.sh"
_CTL = _PYTHON_ROOT / "run_balloon_control.py"
_CAM = _PYTHON_ROOT / "run_balloon_camera.py"
_MAV = _PYTHON_ROOT / "fw_sitl" / "mavlink_io.py"


class TestGzRaceContracts(unittest.TestCase):
    def test_race_gz_wiring(self) -> None:
        text = _RACE.read_text(encoding="utf-8")
        self.assertIn("--gz", text)
        self.assertIn("runSimGzPlane.sh", text)
        self.assertIn("--mode gz", text)
        self.assertIn("--spawn-gz-balloons", text)
        self.assertIn("--viz and --gz are mutually exclusive", text)
        self.assertIn('CTL_CMD+=" --no-sim"', text)
        self.assertIn("--stop-sim-on-exit", text)
        self.assertIn("kill.sh", text)
        self.assertIn("--all", text)
        self.assertIn("--duration", text)
        self.assertIn("--setup", text)
        self.assertIn("px4-noble-gz-plane", text)
        self.assertIn('IMG_CMD+=" --container ${CONTAINER_NAME}"', text)
        self.assertIn('CTL_CMD+=" --gz-container ${CONTAINER_NAME}"', text)
        self.assertIn("import cv2, numpy, zmq, pymavlink, matplotlib", text)
        # Unsetting conda LD_LIBRARY_PATH mixes libzmq → camera fq.cpp abort.
        self.assertNotIn("unset LD_LIBRARY_PATH", text)
        self.assertIn("PYTHONUNBUFFERED=1", text)
        self.assertIn("${PYTHON} -u", text)
        self.assertIn("DISPLAY=${DISPLAY", text)
        self.assertIn("MPLBACKEND", text)

    def test_control_stop_sim_on_exit_and_duration_zero(self) -> None:
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn('add_argument(', ctl)
        self.assertIn('"--stop-sim-on-exit"', ctl)
        self.assertIn("args.stop_sim_on_exit", ctl)
        self.assertIn("kill_docker", ctl)
        self.assertIn("history.note_target", ctl)
        self.assertIn("tgt_ned=", ctl)
        self.assertIn("history.set_balloon_markers", ctl)
        self.assertGreater(
            ctl.index("history.set_balloon_markers"),
            ctl.index("rebase_balloons_to_local_z"),
        )
        help_r = subprocess.run(
            [sys.executable, str(_CTL), "--help"],
            capture_output=True,
            text=True,
            check=True,
        )
        self.assertIn("--stop-sim-on-exit", help_r.stdout)
        self.assertIn("--duration", help_r.stdout)
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn("save_prefix", ctl)
        self.assertIn("csv_path.with_suffix", ctl)
        # Save figures before docker kill: kill.sh --all can take down the
        # tmux control pane (SIGHUP) before savefig. Host waiter opens
        # an interactive matplotlib window (zoom/pan, shared time axis).
        done = ctl.index('print("Done.")')
        self.assertLess(
            ctl.index("to_plot.plot(", done),
            ctl.index("_stop_sim()", done),
        )
        self.assertIn("show=False", ctl)
        self.assertIn("to_pickle", ctl)

    def test_race_script_host_plot_waiter(self) -> None:
        """Launcher python (not tmux) waits on the CSV and opens plots."""
        text = _RACE.read_text(encoding="utf-8")
        self.assertIn("show_race_plots.py", text)
        self.assertIn("--csv ${RACE_CSV}", text)
        self.assertIn("env -u MPLBACKEND", text)
        self.assertIn("import cv2, numpy, zmq, pymavlink, matplotlib", text)

    def test_race_attaches_tmux_by_default(self) -> None:
        """Pose prints live in the control pane; attach after plot waiter starts."""
        text = _RACE.read_text(encoding="utf-8")
        self.assertIn("--detach", text)
        self.assertIn('tmux attach -t "${SESSION}"', text)
        self.assertIn('select-pane -t "${_ctl_pane}"', text)
        plots = text.index("show_race_plots.py")
        attach = text.index('tmux attach -t "${SESSION}"')
        self.assertLess(plots, attach)
        self.assertIn("[[ -t 1 ]]", text)

    def test_control_uses_polled_attitude_for_chase(self) -> None:
        """history.poll drains ATTITUDE; chase must use last_att_rad, not (0,0,0)."""
        ctl = _CTL.read_text(encoding="utf-8")
        poll_at = ctl.index("pos = history.poll(master)")
        use_at = ctl.index("att = history.last_att_rad")
        self.assertGreater(use_at, poll_at)
        self.assertIn("in_view=use_lookat", ctl)
        self.assertIn("z_target=race.balloon_ned()[2]", ctl)
        self.assertIn("if on_screen:", ctl)
        self.assertIn("race.update_track(True, race.geometric_los(pos))", ctl)
        self.assertIn("dir_cam_to_ned(last_dir_cam", ctl)
        self.assertIn("history.note_cam_los", ctl)
        # Pixel look-at first: geometric on-screen was zeroing PX4 yaw while
        # the blob sat on the right of balloon_camera.
        self.assertLess(
            ctl.index("if tracker_in_view:"),
            ctl.index("elif on_screen:"),
        )
        self.assertIn("approach = (history.last_vx, history.last_vy, 0.0)", ctl)
        self.assertIn("path_lock_token=race.target_idx", ctl)
        self.assertIn("vx=history.last_vx", ctl)
        self.assertGreaterEqual(
            ctl.count("chase = race.chase_dir_ned(pos, sim_time_s=now_s)"),
            2,
        )

    def test_control_gz_race_ned_follows_gazebo_pose(self) -> None:
        """EKF LOCAL_POSITION can sit 150 m east while Gazebo/camera are on the balloon."""
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn("gz_enu_to_ned", ctl)
        self.assertIn("world_ned", ctl)
        # Continuous ZMQ subscription (physics-rate gz_pose_bridge), not
        # per-sample `docker exec gz model --pose` polling: that one-shot
        # subprocess had ~0.4-0.5s latency and produced a jittery staircase.
        self.assertIn("PoseSubscriber", ctl)
        self.assertNotIn("fetch_gz_model_enu", ctl)
        self.assertNotIn("gz_pose_stop", ctl)
        # LOCAL_POSITION_NED streams faster than control_rate_hz: a single
        # history.poll() call can append >1 raw-EKF sample. Only patching
        # history.x[-1] left the earlier samples in that burst at their
        # drifted EKF position — a dense zigzag, worst right after spawn.
        # Every sample from this tick's poll() must be patched.
        self.assertIn("overwrite_positions_from", ctl)
        self.assertIn("n_before_poll", ctl)
        self.assertNotIn("history.x[-1], history.y[-1], history.z[-1] = world_ned", ctl)
        # Same burst hazard applies to apply_target_to_last/apply_cam_to_last:
        # patching only the last sample of a poll() burst left earlier
        # samples at NaN, which los_deg_series/target_delta_series read as
        # "no target"/"no camera blob" and silently switch to a different
        # (geometric) estimate — a real, dense LOS zigzag once tracking
        # starts, not sensor noise. Both must be passed n_before_poll.
        self.assertIn("apply_target_to_last(n_before_poll)", ctl)
        self.assertIn("apply_cam_to_last(n_before_poll)", ctl)

    def test_race_gz_pose_bridge_pane(self) -> None:
        """A dedicated pose pane streams Gazebo world pose only in --gz mode."""
        text = _RACE.read_text(encoding="utf-8")
        self.assertIn("run_balloon_gz_pose.py", text)
        self.assertIn("_pose_pane", text)
        gz_block = text[text.index('if [[ "${GZ}" -eq 1 ]]; then\n  IMG_CMD+=') :]
        self.assertIn("POSE_CMD", gz_block[: gz_block.index("fi")])

    def test_gz_pose_bridge_module_contract(self) -> None:
        bridge = (_PYTHON_ROOT / "fw_sitl" / "gz_pose_bridge.py").read_text(encoding="utf-8")
        self.assertIn("dynamic_pose/info", bridge)
        self.assertIn("def run_bridge", bridge)
        self.assertIn("def run_gz_pose_publisher_via_docker", bridge)
        self.assertIn("PosePublisher", bridge)
        launcher = (_PYTHON_ROOT / "run_balloon_gz_pose.py").read_text(encoding="utf-8")
        self.assertIn("run_gz_pose_publisher_via_docker", launcher)

    def test_control_gz_after_engage(self) -> None:
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn("spawn_balloons_gz(", ctl)
        self.assertGreater(
            ctl.index("spawn_balloons_gz("),
            ctl.index("engage_offboard_with_retries"),
        )
        # Do not gate engage on airspeed: tmux heartbeat wait already stalls the
        # unarmed Cessna (spawn velocity is one-shot). Straight flight engages ASAP.
        self.assertNotIn("wait_min_airspeed(master)", ctl)
        self.assertIn("--spawn-gz-balloons", ctl)
        self.assertIn("accept_unhealthy", ctl)
        # Plant table owns FW_AIRSPD_* (Cessna trim 16; no post-engage overlay).
        self.assertIn("prepare_sitl_arming(master, plant)", ctl)
        self.assertIn("plant_id_from_flags", ctl)
        self.assertNotIn("GZ: airspeed SP", ctl)

    def test_wait_min_airspeed_exists(self) -> None:
        mav = _MAV.read_text(encoding="utf-8")
        self.assertIn("def wait_min_airspeed", mav)
        self.assertIn("in-air spawn has no airspeed", mav)
        # PX4 1.17 dropped FW_ARSP_MODE; do not set the dead param.
        self.assertNotIn('("FW_ARSP_MODE"', mav)

    def test_wait_min_airspeed_importable(self) -> None:
        if str(_PYTHON_ROOT) not in sys.path:
            sys.path.insert(0, str(_PYTHON_ROOT))
        from fw_sitl.mavlink_io import wait_min_airspeed

        self.assertTrue(callable(wait_min_airspeed))

    def test_wait_min_airspeed_nan_falls_back_to_groundspeed(self) -> None:
        """Gazebo VFR_HUD often has airspeed=NaN; spec is airspeed else groundspeed."""
        if str(_PYTHON_ROOT) not in sys.path:
            sys.path.insert(0, str(_PYTHON_ROOT))
        from fw_sitl.mavlink_io import wait_min_airspeed

        class _FakeMav:
            def command_long_send(self, *args, **kwargs):
                pass

        class _FakeMaster:
            def __init__(self) -> None:
                self.mav = _FakeMav()
                self.target_system = 1
                self.target_component = 1
                self._msgs = [
                    SimpleNamespace(airspeed=float("nan"), groundspeed=28.0),
                ]

            def recv_match(self, type=None, blocking=True, timeout=0.5):
                if self._msgs:
                    return self._msgs.pop(0)
                return None

        got = wait_min_airspeed(_FakeMaster(), min_mps=15.0, timeout_s=1.0)
        self.assertAlmostEqual(got, 28.0)

    def test_viz_and_gz_exit_2(self) -> None:
        r = subprocess.run(
            ["bash", str(_RACE), "--viz", "--gz"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)
        self.assertIn("--viz and --gz are mutually exclusive", r.stderr)

    def test_model_without_gz_exit_2(self) -> None:
        r = subprocess.run(
            ["bash", str(_RACE), "--model", "rc_cessna"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)

    def test_control_prints_ned_pose_each_second(self) -> None:
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn("PATH_SAMPLE_PERIOD_S = 1.0", ctl)
        sample = ctl[ctl.index("if now_s - last_path_sample_t"):]
        self.assertIn("format_ned_pos_line(now_s, pos)", sample)
        self.assertIn("pos_ned=pos", ctl)
        cam = _CAM.read_text(encoding="utf-8")
        self.assertIn("format_ned_pos_line", cam)
        self.assertIn("latest_color.pos_ned", cam)


if __name__ == "__main__":
    unittest.main()
