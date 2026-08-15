"""Docker / sim runner lifecycle helpers for SITL plants."""
from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
PYTHON_ROOT = PACKAGE_DIR.parent
SCRIPTS_DIR = PYTHON_ROOT / "scripts"
ASSETS_DIR = PYTHON_ROOT / "assets"
KILL_SCRIPT = SCRIPTS_DIR / "kill.sh"
# Compat alias for older call sites / re-exports.
SCRIPT_DIR = SCRIPTS_DIR

_sim_log_file = None  # keep open for Popen lifetime (never inherit docker/FG stdout)
_sim_log_path: Path | None = None
_START_GRACE_S = 2.0


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


def _close_sim_log() -> None:
    global _sim_log_file
    if _sim_log_file is None:
        return
    try:
        _sim_log_file.close()
    except Exception:  # noqa: BLE001
        pass
    _sim_log_file = None


def _sim_log_tail(limit: int = 4000) -> str:
    if _sim_log_path is None:
        return "(no sim log)"
    try:
        text = _sim_log_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return f"(unreadable {_sim_log_path})"
    text = text.strip()
    if not text:
        return f"(empty {_sim_log_path})"
    if len(text) > limit:
        text = text[-limit:]
    return text


def start_sim(
    sim_script: Path,
    *,
    extra_args: list[str] | None = None,
) -> subprocess.Popen:
    global _sim_log_file, _sim_log_path
    if not sim_script.is_file():
        raise FileNotFoundError(sim_script)
    # Debugger stop often skips signal handlers; clear any leftover container first.
    kill_sim(
        sim_script,
        label=f"Stopping any previous simulation container ({sim_script.name})...",
    )
    extra = extra_args or []
    cmd = ["bash", str(sim_script), *extra]
    print(f"Starting simulation: {' '.join(cmd)}")
    # Never inherit the debug/IDE TTY as docker stdin (avoids `docker run -it`
    # attaching the console to PX4 so reruns type into the old container).
    env = os.environ.copy()
    env["PX4_SITL_NO_DOCKER_TTY"] = "1"
    # Never inherit docker/FG stdout into this process: FG can emit hundreds of MB
    # and stall the OFFBOARD setpoint loop (OFFBOARD→ALTCTL / EKF jumps on plots).
    # Always log to a file so missing-image / --gpus failures are visible.
    _close_sim_log()
    if "--viz" in extra:
        log_path = Path(f"/tmp/jsbsim_viz_runner_{os.getpid()}.log")
    else:
        log_path = Path(f"/tmp/fw_sim_runner_{os.getpid()}.log")
    _sim_log_path = log_path
    _sim_log_file = open(log_path, "w", buffering=1)
    print(f"Sim runner log: {log_path} (not inherited — keeps setpoint loop timely)")
    proc = subprocess.Popen(
        cmd,
        cwd=str(sim_script.parent),
        start_new_session=True,
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=_sim_log_file,
        stderr=_sim_log_file,
    )
    deadline = time.time() + _START_GRACE_S
    while time.time() < deadline:
        rc = proc.poll()
        if rc is not None:
            if _sim_log_file is not None:
                _sim_log_file.flush()
            _tail = _sim_log_tail()
            raise RuntimeError(
                f"Sim runner exited {rc} before PX4 heartbeat. Log {log_path}:\n"
                f"{_tail}"
            )
        time.sleep(0.05)
    return proc
