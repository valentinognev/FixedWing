"""Live balloon-race e2e helpers (opt-in; needs Docker / SITL)."""
from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path
from typing import Any

from fw_sitl.flight_setup import KNOWN_SIM_PLATFORMS, load_flight_setup
from fw_sitl.plant_loader import strip_jsonc
from fw_sitl.race_csv import load_pass_misses
from fw_sitl.race_plots import csv_has_end_event, wait_for_race_end

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_REPO_ROOT = _PYTHON_ROOT.parent
_E2E_SETUP_TEMPLATE = _PYTHON_ROOT / "flightSetup.e2e.json"
_RACE_SH = _PYTHON_ROOT / "scripts" / "run_balloon_race.sh"
_KILL_SH = _PYTHON_ROOT / "scripts" / "kill.sh"
_LOG_DIR = _PYTHON_ROOT / "logs" / "e2e"


def e2e_enabled() -> bool:
    return os.environ.get("FW_SITL_E2E", "").strip() in {"1", "true", "yes", "on"}


def default_e2e_platforms() -> tuple[str, ...]:
    """Menu platforms for race_quat live e2e (excludes xplane)."""
    return tuple(sorted(KNOWN_SIM_PLATFORMS))


def load_setup_template_dict() -> dict[str, Any]:
    raw = _E2E_SETUP_TEMPLATE.read_text(encoding="utf-8")
    data = json.loads(strip_jsonc(raw))
    if not isinstance(data, dict):
        raise ValueError(f"{_E2E_SETUP_TEMPLATE}: root must be an object")
    return data


def write_race_quat_e2e_setup(
    path: Path,
    *,
    platform: str,
    duration_s: float = 90.0,
    gz_model: str = "rc_cessna",
    pass_radius_m: float = 50.0,
) -> Path:
    """Materialize a race_quat attitude setup for one ``sim.platform``."""
    plat = str(platform).strip().lower()
    if plat not in KNOWN_SIM_PLATFORMS:
        raise ValueError(
            f"platform {plat!r} not in {sorted(KNOWN_SIM_PLATFORMS)}"
        )
    data = load_setup_template_dict()
    sim = dict(data.get("sim") or {})
    sim["platform"] = plat
    sim["gz_model"] = str(gz_model).strip().lower()
    sim["duration_s"] = float(duration_s)
    data["sim"] = sim
    guidance = dict(data.get("guidance") or {})
    guidance["controller"] = "race_quat"
    guidance["cmd_mode"] = "attitude"
    guidance["laps"] = 0
    guidance.pop("duration_s", None)  # race length is sim.duration_s
    guidance["pass_radius_m"] = float(pass_radius_m)
    data["guidance"] = guidance
    # Ensure pose endpoint exists for gz pose pane.
    zmq = dict(data.get("zmq") or {})
    zmq.setdefault("pose", "tcp://127.0.0.1:5558")
    data["zmq"] = zmq
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    # Validate round-trip.
    setup = load_flight_setup(out)
    if setup.sim.platform != plat:
        raise RuntimeError(f"setup platform mismatch: {setup.sim.platform}")
    if setup.guidance.controller != "race_quat":
        raise RuntimeError(f"setup controller mismatch: {setup.guidance.controller}")
    return out


def kill_all_sims(*, timeout_s: float = 120.0) -> None:
    subprocess.run(
        ["bash", str(_KILL_SH), "--all"],
        cwd=str(_REPO_ROOT),
        timeout=timeout_s,
        check=False,
    )


def run_balloon_race_detached(
    *,
    setup: Path,
    csv_path: Path,
    session: str,
    duration_s: float,
    platform: str,
) -> subprocess.CompletedProcess[str]:
    """Launch ``run_balloon_race.sh --detach``; plant comes from setup ``sim.platform``."""
    env = dict(os.environ)
    env["BALLOON_RACE_CSV"] = str(csv_path)
    env["PYTHONUNBUFFERED"] = "1"
    if platform == "jsbsim":
        env["BALLOON_CAMERA_NO_DISPLAY"] = "1"
    cmd = [
        "bash",
        str(_RACE_SH),
        "--setup",
        str(setup),
        "--duration",
        str(int(duration_s) if float(duration_s).is_integer() else duration_s),
        "--no-plot",
        "--detach",
        "--session",
        session,
    ]
    return subprocess.run(
        cmd,
        cwd=str(_REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=180.0,  # launch only; race runs in tmux
        check=False,
    )


def wait_csv_end(csv_path: Path, *, timeout_s: float) -> bool:
    return wait_for_race_end(csv_path, timeout_s=timeout_s, poll_s=1.0)


def assert_race_quat_csv_ok(csv_path: Path, *, min_passes: int = 1) -> list[tuple[int, float, bool]]:
    """Gate: end_* present and at least ``min_passes`` pass rows."""
    if not csv_path.is_file():
        raise AssertionError(f"missing race CSV: {csv_path}")
    if not csv_has_end_event(csv_path):
        raise AssertionError(f"no end_* row in {csv_path}")
    passes = load_pass_misses(csv_path)
    if len(passes) < int(min_passes):
        raise AssertionError(
            f"expected ≥{min_passes} pass(es) in {csv_path}, got {len(passes)}: {passes}"
        )
    return passes


def run_race_quat_platform_e2e(
    platform: str,
    *,
    duration_s: float = 90.0,
    min_passes: int = 1,
    wait_slack_s: float = 240.0,
    gz_model: str = "rc_cessna",
) -> Path:
    """Full live race for one platform with ``race_quat``; returns CSV path."""
    _LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = _LOG_DIR / f"race_quat_{platform}_{stamp}.csv"
    setup_path = _LOG_DIR / f"race_quat_{platform}_{stamp}_setup.json"
    session = f"e2e_rq_{platform}_{stamp[-6:]}"
    write_race_quat_e2e_setup(
        setup_path,
        platform=platform,
        duration_s=duration_s,
        gz_model=gz_model,
    )
    kill_all_sims()
    try:
        launched = run_balloon_race_detached(
            setup=setup_path,
            csv_path=csv_path,
            session=session,
            duration_s=duration_s,
            platform=platform,
        )
        launch_log = _LOG_DIR / f"race_quat_{platform}_{stamp}_launch.log"
        launch_log.write_text(
            f"rc={launched.returncode}\n--- stdout ---\n{launched.stdout}\n"
            f"--- stderr ---\n{launched.stderr}\n",
            encoding="utf-8",
        )
        if launched.returncode != 0:
            raise RuntimeError(
                f"race launch failed rc={launched.returncode}; see {launch_log}"
            )
        ok = wait_csv_end(csv_path, timeout_s=float(duration_s) + float(wait_slack_s))
        if not ok:
            raise TimeoutError(
                f"timed out waiting for end_* in {csv_path} "
                f"(duration={duration_s}s + slack={wait_slack_s}s)"
            )
        assert_race_quat_csv_ok(csv_path, min_passes=min_passes)
        return csv_path
    finally:
        kill_all_sims()
        # Drop the e2e tmux session if kill left it.
        subprocess.run(
            ["tmux", "kill-session", "-t", session],
            check=False,
            capture_output=True,
        )
