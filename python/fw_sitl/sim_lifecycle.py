"""Docker / sim runner lifecycle helpers for SITL plants."""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
PYTHON_ROOT = PACKAGE_DIR.parent
SCRIPTS_DIR = PYTHON_ROOT / "scripts"
ASSETS_DIR = PYTHON_ROOT / "assets"
KILL_SCRIPT = SCRIPTS_DIR / "kill.sh"
# Compat alias for older call sites / re-exports.
SCRIPT_DIR = SCRIPTS_DIR

_sim_log_file = None  # keep open for Popen lifetime when --viz logs to a file


def kill_docker(*, target: str, label: str | None = None) -> None:
    """Remove SITL container(s) via kill.sh (does not abort the run on failure)."""
    print(label or f"Stopping Docker containers ({KILL_SCRIPT.name} {target})...")
    if not KILL_SCRIPT.is_file():
        print(f"Sim cleanup warning: missing {KILL_SCRIPT}", file=sys.stderr)
        return
    try:
        subprocess.run(
            ["bash", str(KILL_SCRIPT), target],
            check=False,
            timeout=30,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"Sim cleanup warning: {exc}")


def kill_sim(sim_script: Path, *, label: str = "Stopping simulation container...") -> None:
    """Remove the named sim container via the runner's --kill."""
    print(label)
    try:
        subprocess.run(
            ["bash", str(sim_script), "--kill"],
            check=False,
            timeout=30,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"Sim cleanup warning: {exc}")


def start_sim(
    sim_script: Path,
    *,
    extra_args: list[str] | None = None,
) -> subprocess.Popen:
    global _sim_log_file
    if not sim_script.is_file():
        raise FileNotFoundError(sim_script)
    # Debugger stop often skips signal handlers; clear any leftover container first.
    kill_sim(
        sim_script,
        label=f"Stopping any previous simulation container ({sim_script.name})...",
    )
    cmd = ["bash", str(sim_script), *(extra_args or [])]
    print(f"Starting simulation: {' '.join(cmd)}")
    # Never inherit the debug/IDE TTY as docker stdin (avoids `docker run -it`
    # attaching the console to PX4 so reruns type into the old container).
    env = os.environ.copy()
    env["PX4_SITL_NO_DOCKER_TTY"] = "1"
    # Never inherit docker/FG stdout into this process: FG can emit hundreds of MB
    # and stall the OFFBOARD setpoint loop (OFFBOARD→ALTCTL / EKF jumps on plots).
    # Headless: DEVNULL. Viz: line-buffered log file (FG also logs in-container).
    if "--viz" in (extra_args or []):
        if _sim_log_file is not None:
            try:
                _sim_log_file.close()
            except Exception:  # noqa: BLE001
                pass
        log_path = Path(f"/tmp/jsbsim_viz_runner_{os.getpid()}.log")
        _sim_log_file = open(log_path, "w", buffering=1)
        print(f"Sim runner log: {log_path} (not inherited — keeps setpoint loop timely)")
        out: int | object = _sim_log_file
        err: int | object = _sim_log_file
    else:
        out = subprocess.DEVNULL
        err = subprocess.DEVNULL
    return subprocess.Popen(
        cmd,
        cwd=str(sim_script.parent),
        start_new_session=True,
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=out,
        stderr=err,
    )
