# Python Runner Split Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split the JSBSim straight-flight runner into flat modules (`path_geometry`, `mavlink_io`, `sim_lifecycle`, `cli_common`, `straight_flight_core`) with thin plant entrypoints, without changing CLI or engage policies.

**Architecture:** Move code out of `run_straight_flight_jsbsim.py` by responsibility. Both runners call `straight_flight_core.run_locked_line_hold`. YASim adopts the richer JSBSim hold loop. Re-export public helpers from the JSBSim module for shim/test compatibility.

**Tech Stack:** Python 3, pymavlink, existing `flight_history.py`, unittest.

**Spec:** `docs/superpowers/specs/2026-08-10-python-runner-split-design.md`

## Global Constraints

- Pure move/split: do not retune params, engage thresholds, or delete bank helpers.
- Flat modules under `python/` only (no package).
- Public CLI flags/defaults unchanged.
- Shims keep working.
- YASim hold loop becomes the JSBSim hold path (approved hold-loop convergence).
- Root `README.md` + `UPDATES.md` updated after code lands (subver bump).
- **Commits:** this repo forbids `git commit` unless the user explicitly asks; skip commit steps until asked.

---

## File map

| File | Action | Responsibility |
|------|--------|----------------|
| `python/path_geometry.py` | Create | Path/bank math + bank constants + `DEFAULT_SPEED_MPS` (for `thrust_for_speed`) |
| `python/mavlink_io.py` | Create | MAVLink connect/commands/setpoints/reads + typemasks |
| `python/sim_lifecycle.py` | Create | `kill_docker` / `kill_sim` / `start_sim` |
| `python/cli_common.py` | Create | Shared argparse + `DEFAULT_LOOKAHEAD_M` + `resolve_speed` |
| `python/straight_flight_core.py` | Create | Engage, settle, hold orchestration, `EngageError` |
| `python/run_straight_flight_jsbsim.py` | Replace with thin entry + re-exports | JSBSim plant CLI/policy |
| `python/run_straight_flight_yasim.py` | Replace with thin entry | YASim plant CLI/policy |
| `python/test_path_setpoint.py` | Modify imports | Import from `path_geometry` |
| `python/flight_history.py` | Unchanged | — |
| Shims | Unchanged | Still re-export / runpy runners |
| `README.md` / `UPDATES.md` | Update | Architecture + history |

**Constant placement note:** Spec listed `DEFAULT_SPEED_MPS` on `cli_common`; implement it on `path_geometry` (needed by `thrust_for_speed`) and re-export from `cli_common` so layering stays clean.

---

### Task 1: Extract `path_geometry.py` + point tests at it

**Files:**
- Create: `python/path_geometry.py`
- Modify: `python/test_path_setpoint.py`
- Leave: `run_straight_flight_jsbsim.py` still containing the same functions until Task 6 (temporary duplication OK for one task only if needed — prefer cut-move and keep JSBSim importing from `path_geometry` immediately)

**Interfaces:**
- Consumes: `math` only
- Produces: `DEFAULT_SPEED_MPS`, `DEFAULT_THRUST`, `BANK_*`, `ned_velocity_from_course`, `path_setpoint_on_line`, `wrap_pi`, `cross_track_m`, `attitude_quaternion_from_rpy`, `bank_to_turn_commands`, `thrust_for_speed`

- [ ] **Step 1: Update test imports to `path_geometry` (expect fail — module missing)**

In `python/test_path_setpoint.py` replace:

```python
from run_straight_flight_headless import (
    bank_to_turn_commands,
    cross_track_m,
    path_setpoint_on_line,
    wrap_pi,
)
```

with:

```python
from path_geometry import (
    bank_to_turn_commands,
    cross_track_m,
    path_setpoint_on_line,
    wrap_pi,
)
```

- [ ] **Step 2: Run tests — expect ImportError**

Run (from `python/`):

```bash
cd /home/valentin/Projects/FixedWing/python && python3 -m unittest test_path_setpoint -v
```

Expected: `ModuleNotFoundError: No module named 'path_geometry'`

- [ ] **Step 3: Create `path_geometry.py` by moving verbatim**

Create `python/path_geometry.py` with:

```python
"""Locked-line LOCAL_NED path geometry and bank-to-turn helpers."""
from __future__ import annotations

import math

# Match FG / typical Rascal cruise (jsb_spawn ~30 m/s).
DEFAULT_SPEED_MPS = 30.0

BANK_KP_HEADING = 1.5
BANK_KP_CROSS_TRACK = 0.003  # rad per metre of cross-track
BANK_MAX_ROLL_RAD = 0.45
BANK_KP_ALT = 0.025  # rad pitch per metre NED-z error
BANK_MAX_PITCH_RAD = 0.12
DEFAULT_THRUST = 0.60
```

Then **cut** these functions from `run_straight_flight_jsbsim.py` into this file unchanged:
- `ned_velocity_from_course`
- `path_setpoint_on_line`
- `wrap_pi`
- `cross_track_m`
- `attitude_quaternion_from_rpy`
- `bank_to_turn_commands`
- `thrust_for_speed`

Remove the duplicate `BANK_*` / `DEFAULT_THRUST` / `DEFAULT_SPEED_MPS` constants from the JSBSim file once moved (keep `DEFAULT_LOOKAHEAD_M`, stall constants, typemasks there for now).

In `run_straight_flight_jsbsim.py` add:

```python
from path_geometry import (
    DEFAULT_SPEED_MPS,
    DEFAULT_THRUST,
    BANK_KP_HEADING,
    BANK_KP_CROSS_TRACK,
    BANK_MAX_ROLL_RAD,
    BANK_KP_ALT,
    BANK_MAX_PITCH_RAD,
    attitude_quaternion_from_rpy,
    bank_to_turn_commands,
    cross_track_m,
    ned_velocity_from_course,
    path_setpoint_on_line,
    thrust_for_speed,
    wrap_pi,
)
```

(Only import names still referenced in the remaining JSBSim file.)

- [ ] **Step 4: Run tests — expect PASS**

```bash
cd /home/valentin/Projects/FixedWing/python && python3 -m unittest test_path_setpoint -v
```

Expected: all tests OK.

---

### Task 2: Extract `sim_lifecycle.py`

**Files:**
- Create: `python/sim_lifecycle.py`
- Modify: `python/run_straight_flight_jsbsim.py`

**Interfaces:**
- Consumes: `os`, `subprocess`, `sys`, `Path`
- Produces: `SCRIPT_DIR`, `KILL_SCRIPT`, `kill_docker`, `kill_sim`, `start_sim`

- [ ] **Step 1: Create module — move verbatim**

```python
"""Docker / sim runner lifecycle helpers for SITL plants."""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
KILL_SCRIPT = SCRIPT_DIR / "kill.sh"

_sim_log_file = None  # keep open for Popen lifetime when --viz logs to a file
```

Move `kill_docker`, `kill_sim`, `start_sim` verbatim from `run_straight_flight_jsbsim.py`.

Change `kill_docker` default so it does **not** depend on a plant-specific `KILL_TARGET`:

```python
def kill_docker(*, target: str, label: str | None = None) -> None:
    """Remove SITL container(s) via kill.sh (does not abort the run on failure)."""
    print(label or f"Stopping Docker containers ({KILL_SCRIPT.name} {target})...")
    # ... rest unchanged ...
```

Call sites must pass `target=` explicitly (JSBSim: `"--jsbsim"`).

- [ ] **Step 2: Wire JSBSim imports**

In `run_straight_flight_jsbsim.py`:

```python
from sim_lifecycle import SCRIPT_DIR, kill_docker, kill_sim, start_sim

DEFAULT_SIM = SCRIPT_DIR / "runSimJsbsimRascal.sh"
KILL_TARGET = "--jsbsim"
```

Remove local copies of those functions / `KILL_SCRIPT` / `_sim_log_file`.

- [ ] **Step 3: Compile check**

```bash
python3 -m py_compile /home/valentin/Projects/FixedWing/python/sim_lifecycle.py \
  /home/valentin/Projects/FixedWing/python/run_straight_flight_jsbsim.py
```

Expected: exit 0.

---

### Task 3: Extract `mavlink_io.py`

**Files:**
- Create: `python/mavlink_io.py`
- Modify: `python/run_straight_flight_jsbsim.py`

**Interfaces:**
- Consumes: `path_geometry` (`path_setpoint_on_line`, `attitude_quaternion_from_rpy`, `bank_to_turn_commands`, `ned_velocity_from_course` only if needed by send helpers)
- Produces: typemasks, `PX4_CUSTOM_MAIN_MODE_OFFBOARD`, `ARM_FORCE_MAGIC`, and all MAVLink helpers listed in the spec

- [ ] **Step 1: Create `mavlink_io.py`**

Move from JSBSim verbatim:
- `TYPEMASK_POS_ONLY`, `TYPEMASK_ATT_IGNORE_RATES`, `PX4_CUSTOM_MAIN_MODE_OFFBOARD`, `ARM_FORCE_MAGIC`
- `connect`, `poll_vehicle_state`, `wait_armed`, `set_param`, `reboot_autopilot`, `prepare_sitl_arming`
- `send_pos_vel`, `set_offboard`, `arm`, `change_airspeed`, `local_ned_frame`
- `send_attitude_target`, `send_bank_hold`
- `poll_mavlink`, `poll_local_position`, `request_local_position`, `_position_is_usable`, `read_local_position`, `read_heading_rad`
- `send_path_setpoint`

Header:

```python
"""MAVLink connect / params / setpoints / vehicle state helpers."""
from __future__ import annotations

import math
import struct
import sys
import time

from pymavlink import mavutil

from path_geometry import (
    attitude_quaternion_from_rpy,
    bank_to_turn_commands,
    path_setpoint_on_line,
)
```

- [ ] **Step 2: Update JSBSim to import from `mavlink_io`**

Replace local definitions with imports of every symbol still used by engage/main.

- [ ] **Step 3: Compile + unit tests**

```bash
cd /home/valentin/Projects/FixedWing/python && python3 -m py_compile mavlink_io.py run_straight_flight_jsbsim.py && python3 -m unittest test_path_setpoint -v
```

Expected: compile OK, tests PASS.

---

### Task 4: Extract `cli_common.py`

**Files:**
- Create: `python/cli_common.py`

**Interfaces:**
- Consumes: `argparse`, `Path`, `path_geometry.DEFAULT_SPEED_MPS`
- Produces: `DEFAULT_LOOKAHEAD_M`, `DEFAULT_SPEED_MPS` (re-export), `add_common_args`, `resolve_speed`, stall constants used by `--vstall` help

- [ ] **Step 1: Implement CLI helpers**

```python
"""Shared argparse for straight-flight plant runners."""
from __future__ import annotations

import argparse
from pathlib import Path

from path_geometry import DEFAULT_SPEED_MPS

DEFAULT_LOOKAHEAD_M = 500.0

RASCAL_V_STALL_KT = 10.0
KT_TO_MPS = 0.514444
RASCAL_V_STALL_MPS = RASCAL_V_STALL_KT * KT_TO_MPS


def add_common_args(parser: argparse.ArgumentParser, *, default_sim: Path) -> None:
    parser.add_argument(
        "--no-sim",
        action="store_true",
        help=f"Do not start {default_sim.name} (sim already running)",
    )
    parser.add_argument(
        "--sim",
        type=Path,
        default=default_sim,
        help=f"Path to sim runner (default: {default_sim})",
    )
    parser.add_argument(
        "--udp",
        type=int,
        default=14540,
        help="MAVLink UDP port to listen on (default: 14540 offboard; QGC uses 14550)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=DEFAULT_SPEED_MPS,
        help=f"Horizontal speed m/s / airspeed target (default: {DEFAULT_SPEED_MPS:.0f})",
    )
    parser.add_argument(
        "--course-deg",
        type=float,
        default=None,
        help="Fixed NED course degrees (0=north). Default: vehicle yaw at arm",
    )
    parser.add_argument(
        "--lookahead",
        type=float,
        default=DEFAULT_LOOKAHEAD_M,
        help=(
            "Along-track advance of the path position setpoint past the closest "
            f"point on the locked line (m, default: {DEFAULT_LOOKAHEAD_M:.0f})"
        ),
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Setpoint rate Hz (default: 20)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to stream setpoints; 0 = until Ctrl+C (default: 0)",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=0.0,
        help="Optional wall-clock delay after starting sim before connect (default: 0)",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip post-flight matplotlib history window (default: show plot)",
    )


def add_vstall_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--vstall",
        type=float,
        default=None,
        help=(
            "If set with default --speed, use 1.5*vstall as speed "
            f"(Rascal stall reference ≈ {RASCAL_V_STALL_MPS:.2f} m/s)"
        ),
    )


def resolve_speed(args: argparse.Namespace) -> float:
    speed = float(args.speed)
    vstall = getattr(args, "vstall", None)
    if vstall is not None and abs(args.speed - DEFAULT_SPEED_MPS) < 1e-6:
        speed = 1.5 * float(vstall)
    return speed
```

- [ ] **Step 2: Compile**

```bash
python3 -m py_compile /home/valentin/Projects/FixedWing/python/cli_common.py
```

Expected: exit 0.

---

### Task 5: Extract `straight_flight_core.py` (engage + hold)

**Files:**
- Create: `python/straight_flight_core.py`
- Modify: `python/run_straight_flight_jsbsim.py` (temporary: import engage/hold from core; thin `main` in Task 6)

**Interfaces:**
- Consumes: `mavlink_io`, `path_geometry`, `sim_lifecycle`, `flight_history.FlightHistory`
- Produces: `EngageError`, `stream_for`, `settle_path_altitude`, `engage_offboard_asap`, `engage_offboard_with_retries`, `run_locked_line_hold`

- [ ] **Step 1: Move engage/settle/stream into core**

Move verbatim from JSBSim: `stream_for`, `settle_path_altitude`, `engage_offboard_asap`, `engage_offboard_with_retries`.

Add:

```python
class EngageError(RuntimeError):
    """Raised when engage_offboard_with_retries cannot lock a path."""
```

Inside `engage_offboard_with_retries`, when today it would raise/`RuntimeError` or leave main to raise — keep the same exception type if already raising; otherwise raise `EngageError` with the same message string currently used when engage fails. Preserve all retry / `full_sim_restart` / `accept_unhealthy` behavior.

- [ ] **Step 2: Implement `run_locked_line_hold`**

Move the post-sim-start body from current JSBSim `main` (from `connect` through plot) into:

```python
def run_locked_line_hold(
    *,
    udp_port: int,
    speed_mps: float,
    course_deg: float | None,
    along_advance_m: float,
    rate_hz: float,
    duration_s: float,
    no_plot: bool,
    plot_title: str,
    stop_flag: list[bool],
    stop_sim: Callable[[], None],
    sim_script: Path | None,
    sim_extra_args: list[str] | None = None,
    max_attempts: int,
    arm_timeout_s: float,
    full_sim_restart: bool,
    accept_unhealthy: bool,
) -> int:
```

Preserve exact order from current JSBSim `main`:
1. `connect(udp_port, timeout=180.0)` — on failure print, `stop_sim()`, return 1
2. `prepare_sitl_arming` → `reboot_autopilot` → `prepare_sitl_arming`
3. Build `xy` / `z_box` / `origin_box` / `course_box` from `course_deg`
4. `engage_offboard_with_retries(...)` — on failure raise `EngageError` (or catch and return 1; prefer raise so runners can add hints)
5. `settle_path_altitude` + origin refresh `(xy[0], xy[1])`
6. `FlightHistory` streams + path fields
7. Hold loop with NED jump / Z jump re-locks, OFFBOARD restore, force re-arm (verbatim from JSBSim)
8. summary, `stop_sim()`, optional `history.plot(title=plot_title)`, return 0

Hold loop must check `stop_flag[0]` instead of a nonlocal `stop_requested`.

- [ ] **Step 3: JSBSim `main` temporarily calls core**

Keep argparse in JSBSim for one more task, but replace connect→plot with `run_locked_line_hold(...)`, catching `EngageError` to print the existing `--viz` hint.

- [ ] **Step 4: Compile**

```bash
cd /home/valentin/Projects/FixedWing/python && python3 -m py_compile straight_flight_core.py run_straight_flight_jsbsim.py
```

Expected: exit 0.

---

### Task 6: Thin plant runners + re-exports

**Files:**
- Replace body: `python/run_straight_flight_jsbsim.py`
- Replace body: `python/run_straight_flight_yasim.py`

**Interfaces:**
- JSBSim produces: `main`, plus re-exports of former public symbols
- YASim produces: `main` only

- [ ] **Step 1: Rewrite JSBSim entrypoint**

Structure:

```python
#!/usr/bin/env python3
"""Start JSBSim Rascal SITL (optional) and hold straight flight via OFFBOARD..."""
# docstring keep current Usage block

from __future__ import annotations

import argparse
import atexit
import signal
import sys
import time
from pathlib import Path

from cli_common import (
    DEFAULT_LOOKAHEAD_M,
    DEFAULT_SPEED_MPS,
    add_common_args,
    add_vstall_arg,
    resolve_speed,
)
from sim_lifecycle import SCRIPT_DIR, kill_docker, kill_sim, start_sim
from straight_flight_core import EngageError, run_locked_line_hold

# Re-exports for shims / older importers (tests now use path_geometry directly).
from path_geometry import *  # noqa: F401,F403
from mavlink_io import *  # noqa: F401,F403
from sim_lifecycle import *  # noqa: F401,F403
from straight_flight_core import (  # noqa: F401
    engage_offboard_asap,
    engage_offboard_with_retries,
    settle_path_altitude,
    stream_for,
)

DEFAULT_SIM = SCRIPT_DIR / "runSimJsbsimRascal.sh"
KILL_TARGET = "--jsbsim"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "OFFBOARD straight flight for JSBSim Rascal SITL "
            f"(default ~{DEFAULT_SPEED_MPS:.0f} m/s; "
            "locked-line LOCAL_NED path + course velocity)"
        )
    )
    add_common_args(parser, default_sim=DEFAULT_SIM)
    parser.add_argument(
        "--viz",
        action="store_true",
        help="Start FlightGear as visualization for the JSBSim plant (same FDM as headless)",
    )
    add_vstall_arg(parser)
    args = parser.parse_args()

    kill_docker(target=KILL_TARGET)
    speed = resolve_speed(args)
    sim_extra_args = ["--viz"] if args.viz else []

    sim_owned = False

    def _stop_sim() -> None:
        nonlocal sim_owned
        if not sim_owned:
            return
        sim_owned = False
        kill_sim(args.sim, label="\nStopping simulation container...")

    if not args.no_sim:
        start_sim(args.sim, extra_args=sim_extra_args)
        sim_owned = True
        if args.warmup > 0:
            time.sleep(args.warmup)
        atexit.register(_stop_sim)

    stop_flag = [False]

    def _on_signal(_signum=None, _frame=None) -> None:
        stop_flag[0] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    plot_title = (
        "JSBSim + FG viz straight flight" if args.viz else "JSBSim straight flight"
    )
    try:
        return run_locked_line_hold(
            udp_port=args.udp,
            speed_mps=speed,
            course_deg=args.course_deg,
            along_advance_m=max(0.0, float(args.lookahead)),
            rate_hz=args.rate,
            duration_s=args.duration,
            no_plot=args.no_plot,
            plot_title=plot_title,
            stop_flag=stop_flag,
            stop_sim=_stop_sim,
            sim_script=None if args.no_sim else args.sim,
            sim_extra_args=sim_extra_args,
            max_attempts=1 if args.viz else 3,
            arm_timeout_s=60.0 if args.viz else 12.0,
            full_sim_restart=not args.viz,
            accept_unhealthy=bool(args.viz),
        )
    except EngageError as exc:
        print(f"Engage failed: {exc}", file=sys.stderr)
        print(
            "No hold/plot — engage never locked. With --viz, check FG/EKF arm denials "
            "in the console (falling while disarmed is expected until arm succeeds).",
            file=sys.stderr,
        )
        _stop_sim()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
```

Ensure `run_locked_line_hold` raises `EngageError` on engage failure (and handles connect/reboot failures by returning 1 itself, matching today).

- [ ] **Step 2: Rewrite YASim entrypoint**

Same pattern: `DEFAULT_SIM = SCRIPT_DIR / "runSimYasimRascal.sh"`, `KILL_TARGET = "--fg"`, no `--viz`/`--vstall`, engage kwargs:

```python
max_attempts=2,
arm_timeout_s=45.0,
full_sim_restart=False,
accept_unhealthy=True,
```

`plot_title="YASim FlightGear straight flight"`.

On `EngageError`: print `Engage failed: ...` only (no viz hint), `_stop_sim()`, return 1.

Remove `import run_straight_flight_jsbsim as core`.

- [ ] **Step 3: Help smoke + tests**

```bash
cd /home/valentin/Projects/FixedWing/python
python3 run_straight_flight_jsbsim.py --help | head
python3 run_straight_flight_yasim.py --help | head
python3 run_straight_flight_headless.py --help | head
python3 run_straight_flight.py --help | head
python3 -m unittest test_path_setpoint -v
python3 -m py_compile path_geometry.py mavlink_io.py sim_lifecycle.py cli_common.py \
  straight_flight_core.py run_straight_flight_jsbsim.py run_straight_flight_yasim.py
```

Expected: help shows prior flags (`--viz` only on JSBSim/headless); tests PASS; compile OK.

---

### Task 7: Docs (`README.md` / `UPDATES.md`)

**Files:**
- Modify: `README.md`
- Modify: `UPDATES.md`

- [ ] **Step 1: Update Architecture bullets**

Mention flat modules: `path_geometry`, `mavlink_io`, `sim_lifecycle`, `cli_common`, `straight_flight_core`; runners are thin plant entrypoints; `flight_history` unchanged.

- [ ] **Step 2: Top UPDATES entry `0.8.0`**

```markdown
## 0.8.0 - Split straight-flight Python modules
- Extract `path_geometry`, `mavlink_io`, `sim_lifecycle`, `cli_common`, `straight_flight_core` from the JSBSim runner.
- Thin `run_straight_flight_{jsbsim,yasim}.py` entrypoints (CLI + plant engage policy); YASim uses shared hold loop.
- Tests import path helpers from `path_geometry`; shims unchanged.
```

---

## Self-review checklist

1. **Spec coverage:** Modules, hybrid contract, re-exports, YASim hold convergence, verification, docs — each has a task.
2. **Placeholders:** No TBD steps; large bodies are “move verbatim” with named functions.
3. **Types:** `EngageError`, `run_locked_line_hold(..., stop_flag: list[bool], ...)` consistent across Tasks 5–6.
4. **Commits:** Skipped until user asks.

---

Plan complete and saved to `docs/superpowers/plans/2026-08-10-python-runner-split.md`. Two execution options:

**1. Subagent-Driven (recommended)** — fresh subagent per task, review between tasks  

**2. Inline Execution** — execute tasks in this session with checkpoints  

Which approach?
