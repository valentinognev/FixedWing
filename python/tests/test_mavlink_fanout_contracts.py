#!/usr/bin/env python3
"""Contract checks for mavlink fan-out launcher behavior (no live Docker)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_REPO_ROOT = _PYTHON_ROOT.parent
_SIM_SH = _PYTHON_ROOT / "scripts" / "runSimJsbsimRascal.sh"
_RACE_SH = _PYTHON_ROOT / "scripts" / "run_balloon_race.sh"
_REQ = _PYTHON_ROOT / "requirements.txt"


class TestMavlinkFanoutContracts(unittest.TestCase):
    def test_mavlink_server_version_pinned_to_0_10_1(self) -> None:
        """Host fetch + Docker bake must pin the same bluerobotics release."""
        fetch = (_PYTHON_ROOT / "scripts" / "fetch_mavlink_server.sh").read_text(
            encoding="utf-8"
        )
        self.assertRegex(
            fetch, r'MAVLINK_SERVER_VERSION:-\s*0\.10\.1\}'
        )
        dockerfile = (
            _REPO_ROOT / "Dockerfiles" / "PX4NobleSimNvidia.dockerfile"
        ).read_text(encoding="utf-8")
        self.assertRegex(
            dockerfile, r"(?m)^ARG MAVLINK_SERVER_VERSION=0\.10\.1$"
        )
        sim = _SIM_SH.read_text(encoding="utf-8")
        self.assertIn(
            "releases/download/0.10.1/mavlink-server-x86_64-unknown-linux-musl",
            sim,
        )
        start_sh = (
            _REPO_ROOT / "Dockerfiles" / "scripts" / "start_mavlink_server.sh"
        ).read_text(encoding="utf-8")
        # 0.10.1 #223: frequency <= 0 skips broker HEARTBEAT — keep flag.
        self.assertIn("--mavlink-heartbeat-frequency 0", start_sh)
        self.assertIn("--mavlink-heartbeat-frequency 0", sim)

    def test_requirements_include_mss(self) -> None:
        text = _REQ.read_text(encoding="utf-8")
        self.assertRegex(text, r"(?m)^mss([><=!].*)?$")

    def test_sim_default_fanout_off(self) -> None:
        text = _SIM_SH.read_text(encoding="utf-8")
        self.assertRegex(text, r'MAVLINK_FANOUT="\$\{MAVLINK_FANOUT:-0\}"')
        self.assertIn("--setup", text)
        self.assertIn("fw_sitl.spawn_ic", text)

    def test_sim_fanout_fails_loud(self) -> None:
        text = _SIM_SH.read_text(encoding="utf-8")
        self.assertIn("return 1", text)
        self.assertRegex(
            text,
            r"mavlink fan-out requested \(MAVLINK_FANOUT=1\) but failed to start",
        )
        # Docker sidecar start must not hide stderr on the failure path.
        self.assertNotRegex(
            text,
            r"docker run[^\n]*mavlink-server[^\n]*>/dev/null 2>&1",
        )
        self.assertIn('echo "${docker_out}" >&2', text)
        self.assertNotRegex(
            text,
            r"Warning: mavlink-server not found; skip fan-out",
        )

    def test_sim_fanout_redirects_host_logs_and_documents_18570(self) -> None:
        """Host mavlink-server must not share the sim pane; 18570 is PX4 GCS local."""
        text = _SIM_SH.read_text(encoding="utf-8")
        self.assertIn("MAVLINK_SERVER_LOG:-/tmp/mavlink-server-fanout.log", text)
        self.assertRegex(
            text,
            r'MAVLINK_SERVER_RUST_LOG="\$\{MAVLINK_SERVER_RUST_LOG:-off\}"',
        )
        self.assertRegex(
            text,
            r'bash "\$\{MAVLINK_SERVER_SCRIPT\}" >>"\$\{mav_log\}" 2>&1 &',
        )
        self.assertIn("PX4 GCS source port 18570", text)
        self.assertIn('RUST_LOG=${MAVLINK_SERVER_RUST_LOG:-off}', text)

        start_sh = (
            _REPO_ROOT / "Dockerfiles" / "scripts" / "start_mavlink_server.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("udp_gcs_port_local=18570", start_sh)
        self.assertIn("ESC_INFO", start_sh)
        self.assertIn("InvalidCRC", start_sh)
        self.assertRegex(
            start_sh, r'RUST_LOG="\$\{MAVLINK_SERVER_RUST_LOG:-off\}"'
        )

    def test_sim_autofetch_host_mavlink_server(self) -> None:
        """Missing gitignored python/bin binary should be fetched before docker sidecar."""
        text = _SIM_SH.read_text(encoding="utf-8")
        self.assertIn("ensure_host_mavlink_server", text)
        self.assertIn("fetch_mavlink_server.sh", text)

    def test_race_survives_sim_exit_with_clear_error(self) -> None:
        """Sim/fan-out death must not surface as bare 'no server running'."""
        text = _RACE_SH.read_text(encoding="utf-8")
        self.assertIn("remain-on-exit on", text)
        self.assertIn("exited during sim startup", text)
        self.assertIn("fetch_mavlink_server.sh", text)

    def test_race_enables_fanout_and_checks(self) -> None:
        text = _RACE_SH.read_text(encoding="utf-8")
        self.assertRegex(text, r'MAVLINK_FANOUT="\$\{MAVLINK_FANOUT:-1\}"')
        self.assertIn("--mavlink-server", text)
        self.assertIn("mavlink_fanout_up", text)
        self.assertIn("not launching image/camera/control", text)
        self.assertIn("--udp ${MAVLINK_IMAGE_PORT}", text)
        self.assertIn("--udp ${MAVLINK_CONTROL_PORT}", text)
        self.assertRegex(text, r'MAVLINK_IMAGE_PORT="\$\{MAVLINK_IMAGE_PORT:-14541\}"')
        self.assertRegex(text, r'MAVLINK_CONTROL_PORT="\$\{MAVLINK_CONTROL_PORT:-14540\}"')
        fanout_fn = text[text.index("mavlink_fanout_up()"): text.index("wait_control_heartbeat")]
        self.assertIn("${CONTAINER_NAME}-mavlink", fanout_fn)
        self.assertIn(
            "${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}-mavlink",
            fanout_fn,
        )
        gz_sidecar_idx = fanout_fn.index(
            "${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}-mavlink"
        )
        self.assertIn('[[ "${GZ}" -eq 1 ]]', fanout_fn[:gz_sidecar_idx])

    def test_race_dumps_sim_pane_when_fanout_missing(self) -> None:
        """Fan-out abort must print the sim pane; spawn_ic / docker errors hide there."""
        text = _RACE_SH.read_text(encoding="utf-8")
        start = text.index('echo "Error: mavlink-server fan-out failed')
        fail = text[start : start + 900]
        self.assertIn('tmux capture-pane -t "${SESSION}:0.0"', fail)
        self.assertIn("Sim pane:", fail)

    def test_race_waits_for_control_heartbeat(self) -> None:
        text = _RACE_SH.read_text(encoding="utf-8")
        self.assertRegex(
            text, r'HEARTBEAT_TIMEOUT_S="\$\{HEARTBEAT_TIMEOUT_S:-120\}"'
        )
        self.assertIn("wait_control_heartbeat", text)
        self.assertIn("Waiting for MAVLink heartbeat", text)
        self.assertIn("no heartbeat on control UDP", text)
        # Peers start only after heartbeat wait; control first to engage ASAP.
        # Single window: split panes (not separate windows).
        wait_idx = text.index("wait_control_heartbeat")
        control_idx = text.index('split-window -d -t "${SESSION}:0.0"')
        # First split is control (CTL_CMD); image follows.
        ctl_cmd_idx = text.index("${CTL_CMD}", control_idx)
        image_idx = text.index("${IMG_CMD}", ctl_cmd_idx)
        self.assertLess(wait_idx, control_idx)
        self.assertLess(ctl_cmd_idx, image_idx)
        self.assertNotIn("tmux new-window", text)
        self.assertIn("select-layout -t \"${SESSION}:0\" tiled", text)

    def test_plot_timeout_accepts_float_duration(self) -> None:
        """resolve_race_sim prints 60.0; bash $((60.0+300)) is a syntax error."""
        text = _RACE_SH.read_text(encoding="utf-8")
        self.assertIn("DURATION%%.*", text)
        self.assertIn("PLOT_TIMEOUT", text)

    def test_race_passes_no_sim_to_control_when_race_owns_sim(self) -> None:
        """Race launcher owns sim → control must attach (--no-sim), never kill/restart at start."""
        text = _RACE_SH.read_text(encoding="utf-8")
        # Unconditional: race always passes --no-sim to control (owns sim or user --no-sim).
        self.assertRegex(
            text,
            r'CTL_CMD\+=" --no-sim"',
        )
        # Must not gate --no-sim solely on user NO_SIM=1 (that would omit it when race starts sim).
        self.assertNotRegex(
            text,
            r'if \[\[ "\$\{NO_SIM\}" -eq 1 \]\][^\n]*\n[^\n]*CTL_CMD\+=" --no-sim"',
        )

    def test_race_kills_leftover_docker_then_stops_sim_on_exit(self) -> None:
        """New race: kill leftover containers first; timed end removes docker."""
        text = _RACE_SH.read_text(encoding="utf-8")
        kill_idx = text.index('kill.sh" --all')
        session_idx = text.index("tmux new-session")
        self.assertLess(kill_idx, session_idx)
        self.assertIn('CTL_CMD+=" --stop-sim-on-exit"', text)
        # User --no-sim must not tear down an already-running plant.
        self.assertIn('NO_SIM', text)
        self.assertNotIn(
            'CTL_CMD="PYTHONUNBUFFERED=1 ${PYTHON} -u ${PYTHON_ROOT}/run_balloon_control.py --setup ${SETUP} --udp ${MAVLINK_CONTROL_PORT} --no-plot"',
            text,
        )

    def test_control_skips_fg_balloon_spawn_when_no_sim(self) -> None:
        """Race launcher places balloons before PX4 heartbeat; --no-sim must not wait on telnet."""
        ctl = (_PYTHON_ROOT / "run_balloon_control.py").read_text(encoding="utf-8")
        self.assertIn("want_fg_balloons and not args.no_sim", ctl)
        self.assertIn("spawn_fg_from_setup", ctl)
        engage_idx = ctl.index("engage_offboard_with_retries(")
        spawn_idx = ctl.index("spawn_fg_from_setup(")
        self.assertLess(spawn_idx, engage_idx)

    def test_launcher_spawns_balloons_before_px4_heartbeat(self) -> None:
        """geo.put_model / gz create before wait_control_heartbeat so models exist first."""
        text = _RACE_SH.read_text(encoding="utf-8")
        self.assertIn("fw_sitl.balloon_scene", text)
        self.assertIn("--fg", text)
        spawn_fg = text.index("fw_sitl.balloon_scene")
        # Call site, not the function definition.
        wait_call = text.rindex("wait_control_heartbeat")
        self.assertLess(spawn_fg, wait_call)
        ctl_split = text.index('split-window -d -t "${SESSION}:0.0"')
        self.assertLess(wait_call, ctl_split)

    def test_fg_spawn_uses_config_ned_not_ekf_rebased_z(self) -> None:
        """geo.put_model MSL = cruise MSL + plot-relative balloon Z, not EKF z."""
        src = (_PYTHON_ROOT / "fw_sitl" / "balloon_scene.py").read_text(encoding="utf-8")
        start = src.index("def spawn_fg_from_setup")
        snippet = src[start : start + 400]
        self.assertNotIn("world_balloons", snippet)
        self.assertIn("local_z=0", snippet)

    def test_synthetic_default_udp_matches_image_source(self) -> None:
        synth = (_PYTHON_ROOT / "fw_sitl" / "synthetic_camera.py").read_text(
            encoding="utf-8"
        )
        self.assertRegex(synth, r"udp_port:\s*int\s*=\s*14541")


if __name__ == "__main__":
    if str(_PYTHON_ROOT) not in sys.path:
        sys.path.insert(0, str(_PYTHON_ROOT))
    unittest.main()
