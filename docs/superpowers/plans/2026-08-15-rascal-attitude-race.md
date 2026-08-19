# Rascal Attitude Balloon Race Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fly balloon race on JSBSim Rascal (headless and FG viz) and YASim Rascal using the Gazebo Cessna chase laws: quaternion PID internally, Euler angle + thrust commands on the wire.

**Architecture:** Keep `AttitudePid` on SO(3). After `q_cmd`, convert with `rpy_from_quat` and send `send_attitude_target(roll, pitch, yaw, thrust)` for every plant (GZ, JSBSim, YASim). Wire `--yasim` into the existing race launcher (FG camera + Nasal balloons, kill `--fg`). Rascal straight-flight runners default `--cmd-mode attitude` and actually forward it. Live SITL is the last gate.

**Tech Stack:** Python 3 unittest (`cd python && python3 -m unittest …`), pymavlink `SET_ATTITUDE_TARGET`, bash Docker runners, tmux race launcher, PX4 v1.17 JSBSim Rascal / YASim `flightgear_rascal`.

**Spec:** Approved in-chat design 2026-08-15 (bounded; no separate spec file). Binding rules are Global Constraints below.

## Global Constraints

- Internal attitude control stays quaternion (`AttitudePid` / SO(3)). Do not replace the PID with Euler-error loops.
- Attitude-mode commands for **all** plants (Gazebo, JSBSim headless, JSBSim `--viz`, YASim) are **roll, pitch, yaw + thrust** via `send_attitude_target`. Do not call `send_attitude_quat(master, q_cmd, thrust)` from chase/hold. `send_attitude_quat` may remain as the MAVLink packer used **only** inside `send_attitude_target`.
- `SET_ATTITUDE_TARGET` type_mask remains `TYPEMASK_ATT_IGNORE_RATES` (body rates ignored).
- Race `python/flightSetup.json` `cmd_mode` stays `"attitude"`. Do not switch it to velocity.
- `--yasim`, `--viz`, and `--gz` are pairwise exclusive (exit 2). `--model` still requires `--gz`.
- YASim race uses FG window capture (`--mode fg`) and `spawn_balloons_fg`, not Gazebo camera/balloons.
- JSBSim headless race stays synth camera (`--mode synth`). JSBSim `--viz` stays FG capture.
- Tests run: `cd python && python3 -m unittest <module> -v` (Anaconda `site-packages/tests` shadows `python3 -m unittest tests.*`). This repo does not use pytest as the runner.
- Do not retune `AttitudePid` gains or thrust constants unless a Task 7 live run proves a Rascal-specific dive/spiral. If that happens, change the smallest constant and record it in the report.
- Do not add Rascal airspeed-trim ramps unless Task 7 shows an underspeed dive. The existing GZ+velocity trim stays GZ-gated.
- Do not change PX4 airframe IDs `4003` / `4008`.
- **Commits:** user requested subagent-driven execution. Each task commits **only the files listed in that task**. Do not `git add -A`. Do not stage unrelated dirty files already in the working tree.
- Work in the given checkout. Do not create a new git worktree from HEAD (uncommitted Gazebo attitude work is the baseline).

---

## File map

| File | Responsibility |
|------|----------------|
| `python/fw_sitl/mavlink_io.py` | `send_attitude_target(roll, pitch, yaw, thrust)` packs MAVLink; `send_attitude_quat` only used here |
| `python/fw_sitl/body_cmd_controllers.py` | Chase: `q_cmd` → rpy → `send_attitude_target` |
| `python/fw_sitl/straight_flight_core.py` | Hold: same send path in `cmd_mode == "attitude"` |
| `python/fw_sitl/cli_common.py` | `--cmd-mode` help text; argparse default stays `velocity` (runners override) |
| `python/run_straight_flight_jsbsim.py` | Default attitude; pass `cmd_mode=args.cmd_mode` |
| `python/run_straight_flight_yasim.py` | Default attitude; pass `cmd_mode=args.cmd_mode` |
| `python/run_balloon_control.py` | `--yasim`; skip reboot on race attach; FG balloons for YASim |
| `python/scripts/runSimYasimRascal.sh` | `--mavlink-server`; balloon models; Nasal/telnet flags via existing FG patch |
| `Dockerfiles/patch_px4_flightgear_sitl.sh` | YASim FG: `--telnet=5501` + `--allow-nasal-from-sockets` |
| `python/scripts/run_balloon_race.sh` | `--yasim` plant wiring + `BALLOON_RACE_DURATION` |
| `python/scripts/kill.sh` | `--fg` also removes `${FG_NAME}-mavlink` and host mavlink-server |
| `python/tests/test_body_cmd_controllers.py` | Chase asserts angle+thrust send |
| `python/tests/test_attitude_send.py` | Packer + hold/chase send contracts |
| `python/tests/test_rascal_straight_flight.py` | JSBSim/YASim attitude default + forward |
| `python/tests/test_yasim_race_contracts.py` | `--yasim` launcher/control/sim/kill contracts |
| `README.md` / `UPDATES.md` | `--yasim`; angle commands; version `0.21.0` |

---

### Task 1: Quaternion PID, Euler+thrust commands

**Files:**
- Modify: `python/fw_sitl/body_cmd_controllers.py`
- Modify: `python/fw_sitl/straight_flight_core.py`
- Modify: `python/fw_sitl/cli_common.py` (help text only)
- Modify: `python/tests/test_body_cmd_controllers.py`
- Create: `python/tests/test_attitude_send.py`

**Interfaces:**
- Consumes: `AttitudePid.command(q_des, q_act, dt) -> Quat`, `rpy_from_quat(q) -> tuple[float, float, float]`, existing `send_attitude_target(master, roll, pitch, yaw, thrust) -> None`
- Produces: chase/hold call `send_attitude_target(master, roll, pitch, yaw, thrust)` after `roll, pitch, yaw = rpy_from_quat(q_cmd)`. No chase/hold call to `send_attitude_quat`.

- [ ] **Step 1: Write the failing tests**

Create `python/tests/test_attitude_send.py`:

```python
#!/usr/bin/env python3
"""Attitude commands are Euler + thrust; PID stays quaternion."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.mavlink_io import TYPEMASK_ATT_IGNORE_RATES, send_attitude_target
from fw_sitl.path_geometry import attitude_quaternion_from_rpy
from fw_sitl.quat import rpy_from_quat


class TestSendAttitudeTarget(unittest.TestCase):
    def test_packs_euler_thrust_and_ignores_rates(self) -> None:
        master = MagicMock()
        master.target_system = 1
        master.target_component = 1
        send_attitude_target(master, 0.1, 0.2, 0.3, 0.7)
        master.mav.set_attitude_target_send.assert_called_once()
        args = master.mav.set_attitude_target_send.call_args[0]
        self.assertEqual(args[3], TYPEMASK_ATT_IGNORE_RATES)
        q = args[4]
        roll, pitch, yaw = rpy_from_quat((float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        self.assertAlmostEqual(roll, 0.1, places=5)
        self.assertAlmostEqual(pitch, 0.2, places=5)
        self.assertAlmostEqual(yaw, 0.3, places=5)
        self.assertEqual(args[5], 0.0)
        self.assertEqual(args[6], 0.0)
        self.assertEqual(args[7], 0.0)
        self.assertAlmostEqual(args[8], 0.7)
        packed = attitude_quaternion_from_rpy(0.1, 0.2, 0.3)
        for a, b in zip(q, packed):
            self.assertAlmostEqual(float(a), float(b), places=6)


class TestAttitudeCallSites(unittest.TestCase):
    def test_chase_sends_angles_not_raw_quat(self) -> None:
        text = (_PYTHON_ROOT / "fw_sitl" / "body_cmd_controllers.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("roll, pitch, yaw = rpy_from_quat(q_cmd)", text)
        self.assertIn(
            "send_attitude_target(master, roll, pitch, yaw, thrust)", text
        )
        self.assertNotIn("send_attitude_quat(master, q_cmd, thrust)", text)

    def test_hold_sends_angles_not_raw_quat(self) -> None:
        text = (_PYTHON_ROOT / "fw_sitl" / "straight_flight_core.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("roll, pitch, yaw = rpy_from_quat(q_cmd)", text)
        self.assertIn(
            "send_attitude_target(master, roll, pitch, yaw, thrust)", text
        )
        self.assertNotIn("send_attitude_quat(master, q_cmd, thrust)", text)


if __name__ == "__main__":
    unittest.main()
```

In `python/tests/test_body_cmd_controllers.py`, change every
`patch("fw_sitl.body_cmd_controllers.send_attitude_quat")` to
`patch("fw_sitl.body_cmd_controllers.send_attitude_target")`.

In `test_too_low_sends_nose_up_quaternion`, rename to
`test_too_low_sends_nose_up_pitch_and_thrust` and replace the q unpack with:

```python
send.assert_called_once()
_master, roll, pitch, yaw, thrust = send.call_args[0]
self.assertGreater(pitch, 0.0)
self.assertGreater(thrust, 0.62)
```

In `test_ground_track_left_of_course_banks_right`:

```python
_master, roll, _pitch, _yaw, _thrust = send.call_args[0]
self.assertGreater(roll, 0.0)
```

In `test_in_view_los_right_banks_right_not_track`:

```python
_master, roll, _pitch, _yaw, _thrust = send.call_args[0]
self.assertGreater(roll, 0.1)
```

Leave `test_in_view_skips_lookahead_z` patched to `send_attitude_target` (no unpack).

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd python && python3 -m unittest test_attitude_send test_body_cmd_controllers -v
```

Expected: `test_chase_sends_angles_not_raw_quat` and `test_hold_sends_angles_not_raw_quat` FAIL (missing `send_attitude_target(master, roll, pitch, yaw, thrust)` / still `send_attitude_quat(master, q_cmd, thrust)`). Body-cmd tests FAIL because `send.assert_called_once()` is false or unpack of quat vs angles is wrong. `test_packs_euler_thrust_and_ignores_rates` should already PASS (`send_attitude_target` exists). If a body-cmd test errors instead of failing an assertion, fix the test until the failure is “send not called / wrong arity”, then continue.

- [ ] **Step 3: Minimal implementation**

`python/fw_sitl/body_cmd_controllers.py`:
- Import `send_attitude_target` instead of `send_attitude_quat`.
- After `q_cmd = self._pid.command(...)` and thrust compute, replace `send_attitude_quat(master, q_cmd, thrust)` with:

```python
roll, pitch, yaw = rpy_from_quat(q_cmd)
send_attitude_target(master, roll, pitch, yaw, thrust)
```

Keep `self.last_q_cmd = q_cmd` (quaternion still stored for debug). Do not change PID, LOS, or path laws.

`python/fw_sitl/straight_flight_core.py`:
- Import `send_attitude_target` instead of `send_attitude_quat` (keep other mavlink_io imports).
- In the `cmd_mode == "attitude"` branch, after `q_cmd = att_pid.command(...)` and thrust, replace `send_attitude_quat(master, q_cmd, thrust)` with the same two lines as chase.

`python/fw_sitl/cli_common.py` help for `--cmd-mode`:

```
"OFFBOARD hold: velocity = locked-line path setpoints; "
"attitude = quaternion PID then Euler + thrust SET_ATTITUDE_TARGET "
"(default: velocity)"
```

Do not change argparse `default="velocity"` here.

Do not edit `send_attitude_target` / `send_attitude_quat` bodies unless a test proves packing is wrong.

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd python && python3 -m unittest test_attitude_send test_body_cmd_controllers test_attitude_pid -v
```

Expected: all PASS, output pristine.

- [ ] **Step 5: Commit**

```bash
git add python/fw_sitl/body_cmd_controllers.py python/fw_sitl/straight_flight_core.py python/fw_sitl/cli_common.py python/tests/test_body_cmd_controllers.py python/tests/test_attitude_send.py
git commit -m "$(cat <<'EOF'
Send attitude as Euler + thrust after quaternion PID.

Chase and locked-line hold still compute q_cmd on SO(3), then pack
SET_ATTITUDE_TARGET from roll/pitch/yaw + thrust for every plant.
EOF
)"
```

---

### Task 2: Rascal straight flight defaults to attitude

**Files:**
- Modify: `python/run_straight_flight_jsbsim.py`
- Modify: `python/run_straight_flight_yasim.py`
- Create: `python/tests/test_rascal_straight_flight.py`

**Interfaces:**
- Consumes: `run_locked_line_hold(..., cmd_mode: str = "velocity")`, `add_common_args` `--cmd-mode`
- Produces: both Rascal runners call `parser.set_defaults(cmd_mode="attitude")` and `cmd_mode=args.cmd_mode` into `run_locked_line_hold`. `--cmd-mode velocity` still works.

- [ ] **Step 1: Write the failing test**

Create `python/tests/test_rascal_straight_flight.py`:

```python
#!/usr/bin/env python3
from __future__ import annotations

import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_JSB = _PYTHON_ROOT / "run_straight_flight_jsbsim.py"
_YAS = _PYTHON_ROOT / "run_straight_flight_yasim.py"


class TestRascalStraightFlightAttitude(unittest.TestCase):
    def test_jsbsim_defaults_attitude_and_forwards_cmd_mode(self) -> None:
        text = _JSB.read_text(encoding="utf-8")
        self.assertIn("run_locked_line_hold", text)
        self.assertIn("cmd_mode=args.cmd_mode", text)
        self.assertIn('parser.set_defaults(cmd_mode="attitude")', text)

    def test_yasim_defaults_attitude_and_forwards_cmd_mode(self) -> None:
        text = _YAS.read_text(encoding="utf-8")
        self.assertIn("run_locked_line_hold", text)
        self.assertIn("cmd_mode=args.cmd_mode", text)
        self.assertIn('parser.set_defaults(cmd_mode="attitude")', text)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd python && python3 -m unittest test_rascal_straight_flight -v
```

Expected: FAIL — `cmd_mode=args.cmd_mode` and `parser.set_defaults(cmd_mode="attitude")` missing. Today both runners ignore `--cmd-mode` and always hold in velocity.

- [ ] **Step 3: Minimal implementation**

In `python/run_straight_flight_jsbsim.py`, after `add_common_args(...)` / `add_vstall_arg(...)` and **before** `args = parser.parse_args()`:

```python
parser.set_defaults(cmd_mode="attitude")
```

Add `cmd_mode=args.cmd_mode` to the `run_locked_line_hold(...)` kwargs.

In the argparse description string, mention default `--cmd-mode attitude` (quaternion PID, Euler+thrust).

In `python/run_straight_flight_yasim.py`, the same two edits: `parser.set_defaults(cmd_mode="attitude")` before parse, and `cmd_mode=args.cmd_mode` on `run_locked_line_hold`. Update the description string the same way.

Do not change JSBSim/YASim engage retry kwargs (`max_attempts`, `accept_unhealthy`, etc.).

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd python && python3 -m unittest test_rascal_straight_flight test_gz_straight_flight -v
```

Expected: PASS. Gazebo runner still has its own `set_defaults`.

- [ ] **Step 5: Commit**

```bash
git add python/run_straight_flight_jsbsim.py python/run_straight_flight_yasim.py python/tests/test_rascal_straight_flight.py
git commit -m "$(cat <<'EOF'
Default JSBSim and YASim straight flight to attitude commands.

Forward --cmd-mode into the shared hold so Rascal matches Gazebo:
quaternion PID internally, Euler + thrust on the wire.
EOF
)"
```

---

### Task 3: Race control `--yasim` and skip reboot on attach

**Files:**
- Modify: `python/run_balloon_control.py`
- Test: `python/tests/test_yasim_race_contracts.py` (create; later tasks append)

**Interfaces:**
- Consumes: `SCRIPTS_DIR / "runSimYasimRascal.sh"`, existing `--viz` FG balloon spawn, `engage_offboard_with_retries`
- Produces:
  - `--yasim` flag. If `args.yasim and args.sim == DEFAULT_SIM`: `args.sim = SCRIPTS_DIR / "runSimYasimRascal.sh"`
  - `kill_target = "--gz" if args.gz else ("--fg" if args.yasim else KILL_TARGET)`
  - Exit 2 if more than one of `--viz`, `--gz`, `--yasim` is set (print `Error: --viz, --gz, and --yasim are mutually exclusive`)
  - `skip_reboot = bool(args.no_sim or args.viz or args.gz or args.yasim)`
  - `want_fg_balloons = bool(args.spawn_fg_balloons or args.viz or args.yasim)`
  - `max_attempts=1 if (args.viz or args.gz or args.yasim or args.no_sim) else 3` (same idea as today, include yasim explicitly)
  - `full_sim_restart=(not args.viz) and (not args.gz) and (not args.yasim) and (not args.no_sim)`

- [ ] **Step 1: Write the failing test**

Create `python/tests/test_yasim_race_contracts.py`:

```python
#!/usr/bin/env python3
from __future__ import annotations

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
        self.assertIn('kill_target = "--gz" if args.gz else ("--fg" if args.yasim else KILL_TARGET)', ctl)
        self.assertIn("skip_reboot = bool(args.no_sim or args.viz or args.gz or args.yasim)", ctl)
        self.assertIn("args.spawn_fg_balloons or args.viz or args.yasim", ctl)
        self.assertIn("--viz, --gz, and --yasim are mutually exclusive", ctl)

    def test_control_help_lists_yasim(self) -> None:
        r = subprocess.run(
            [sys.executable, str(_CTL), "--help"],
            capture_output=True,
            text=True,
            check=True,
        )
        self.assertIn("--yasim", r.stdout)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd python && python3 -m unittest test_yasim_race_contracts.TestYasimControlContracts -v
```

Expected: FAIL — `--yasim` missing. `test_control_help_lists_yasim` FAIL or argparse unknown.

- [ ] **Step 3: Minimal implementation**

In `python/run_balloon_control.py` argparse, after `--gz` / `--spawn-gz-balloons`:

```python
parser.add_argument(
    "--yasim",
    action="store_true",
    help="YASim FlightGear Rascal plant (runSimYasimRascal.sh)",
)
```

After `args = parser.parse_args()`:

```python
plant_flags = int(bool(args.viz)) + int(bool(args.gz)) + int(bool(args.yasim))
if plant_flags > 1:
    print("Error: --viz, --gz, and --yasim are mutually exclusive", file=sys.stderr)
    return 2
if args.yasim and args.sim == DEFAULT_SIM:
    args.sim = SCRIPTS_DIR / "runSimYasimRascal.sh"
if args.gz and args.sim == DEFAULT_SIM:
    args.sim = SCRIPTS_DIR / "runSimGzPlane.sh"
kill_target = "--gz" if args.gz else ("--fg" if args.yasim else KILL_TARGET)
```

Keep the existing `if args.gz and args.sim == DEFAULT_SIM` logic; do not leave two competing assignments. Equivalent structure:

```python
if args.gz and args.sim == DEFAULT_SIM:
    args.sim = SCRIPTS_DIR / "runSimGzPlane.sh"
elif args.yasim and args.sim == DEFAULT_SIM:
    args.sim = SCRIPTS_DIR / "runSimYasimRascal.sh"
kill_target = "--gz" if args.gz else ("--fg" if args.yasim else KILL_TARGET)
```

Replace skip_reboot:

```python
skip_reboot = bool(args.no_sim or args.viz or args.gz or args.yasim)
```

Update the skip-reboot print to mention `--no-sim` / `--yasim`, not only `--viz/--gz`.

Replace `want_fg_balloons = bool(args.spawn_fg_balloons or args.viz)` with
`want_fg_balloons = bool(args.spawn_fg_balloons or args.viz or args.yasim)`.

In `engage_offboard_with_retries` kwargs, include `args.yasim` next to viz/gz/no_sim for `max_attempts`, `arm_timeout_s`, and `full_sim_restart` as specified in Interfaces.

If `args.yasim` and not `args.no_sim`, `sim_extra` stays `[]` (YASim runner has no `--viz` flag).

Do not change chase/send code in this task.

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd python && python3 -m unittest test_yasim_race_contracts.TestYasimControlContracts test_gz_race_contracts test_mavlink_fanout_contracts.TestMavlinkFanoutContracts.test_control_defers_fg_balloon_spawn_until_after_engage -v
```

Expected: PASS. Existing GZ exclusive test (`--viz --gz`) still exit 2.

- [ ] **Step 5: Commit**

```bash
git add python/run_balloon_control.py python/tests/test_yasim_race_contracts.py
git commit -m "$(cat <<'EOF'
Add balloon-race --yasim plant and skip reboot on attach.

YASim uses the FG Rascal container and Nasal balloons; in-air race
attach no longer reboots the autopilot while the plane is falling.
EOF
)"
```

---

### Task 4: YASim sim fan-out, balloons, Nasal, kill sidecar

**Files:**
- Modify: `python/scripts/runSimYasimRascal.sh`
- Modify: `Dockerfiles/patch_px4_flightgear_sitl.sh`
- Modify: `python/scripts/kill.sh`
- Modify: `python/tests/test_yasim_race_contracts.py` (append classes)
- Modify: `python/tests/test_mavlink_fanout_contracts.py` only if a JSBSim-only assertion would break; prefer not to change JSBSim tests. Add YASim assertions in `test_yasim_race_contracts.py`.

**Interfaces:**
- Consumes: `Dockerfiles/scripts/start_mavlink_server.sh`, balloon assets `python/assets/balloons`, JSBSim fan-out helpers in `runSimJsbsimRascal.sh` as the copy source
- Produces:
  - `runSimYasimRascal.sh --mavlink-server` / `--no-mavlink-server` / `MAVLINK_FANOUT` default `0`
  - Host or docker sidecar fan-out identical in behavior to JSBSim (fail loud, log `/tmp/mavlink-server-fanout.log`, `--mavlink-heartbeat-frequency 0`, ports 18570→14550→14540/14541)
  - Bind-mount balloons to `/opt/fixedwing/balloons` and copy `balloon_*.ac` / `balloon_*.xml` into `/opt/flightgear/fgdata/Models/FixedWing/`
  - FG launch includes `--telnet=5501` and `--allow-nasal-from-sockets`
  - `kill.sh --fg` removes `${FG_NAME}`, `${FG_NAME}-mavlink`, and host `mavlink-server` (same as JSBSim stack). `--all` still kills FG via this stack.

- [ ] **Step 1: Write the failing tests**

Append to `python/tests/test_yasim_race_contracts.py`:

```python
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
        self.assertIn("balloon_*.xml", text)
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

    def test_kill_fg_removes_mavlink_sidecar(self) -> None:
        text = _KILL.read_text(encoding="utf-8")
        self.assertIn("kill_fg_stack", text)
        fg_block = text[text.index("--fg)"): text.index("--jsbsim)")]
        self.assertIn("kill_fg_stack", fg_block)
        all_block = text[text.index("--all)"):]
        self.assertIn("kill_fg_stack", all_block)
        self.assertIn("${FG_NAME}-mavlink", text)
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd python && python3 -m unittest test_yasim_race_contracts.TestYasimSimFanoutAndBalloons -v
```

Expected: FAIL — YASim sim has no `--mavlink-server`; FG patch has no nasal/telnet; `kill.sh --fg` only `kill_container "${FG_NAME}"`.

- [ ] **Step 3: Minimal implementation**

Copy from `python/scripts/runSimJsbsimRascal.sh` into `runSimYasimRascal.sh` (adapt names; do not extract a third shared script):
- `MAVLINK_SERVER_SCRIPT`, `MAVLINK_SERVER_PID=""`, `MAVLINK_FANOUT="${MAVLINK_FANOUT:-0}"`
- `BALLOONS_HOST` / `CONTAINER_BALLOONS="/opt/fixedwing/balloons"`
- CLI: `--mavlink-server` → `MAVLINK_FANOUT=1`; `--no-mavlink-server` → `0`
- Functions: `mavlink_server_usable`, `resolve_mavlink_server_bin`, `ensure_host_mavlink_server`, `start_mavlink_fanout` (verbatim behavior; `CONTAINER_NAME` is already the YASim container)
- `cleanup_on_exit`: kill host PID, `docker rm -f "${CONTAINER_NAME}-mavlink"`, then the sim container (keep existing `xhost -`)
- After trap, call `start_mavlink_fanout` and `exit 1` if it fails when requested
- Add balloons volume to `DOCKER_VOLUMES`
- Inside the container bash, after the existing patch, copy balloon files:

```bash
if [[ -d '${CONTAINER_BALLOONS}' && -d /opt/flightgear/fgdata ]]; then
  mkdir -p /opt/flightgear/fgdata/Models/FixedWing
  cp -f '${CONTAINER_BALLOONS}'/balloon_*.ac '${CONTAINER_BALLOONS}'/balloon_*.xml \
    /opt/flightgear/fgdata/Models/FixedWing/ 2>/dev/null || true
fi
```

In `Dockerfiles/patch_px4_flightgear_sitl.sh`, after the FG_run.py dedupe patch (or inside that Python block), ensure the launched argument list contains `--telnet=5501` and `--allow-nasal-from-sockets` if missing. Idempotent: if those strings are already in the file after patch, do not duplicate. Do **not** add JSBSim `--fdm=null` or draw-mask hide-aircraft (YASim is the FDM; hiding the plane would change straight-flight viz).

In `python/scripts/kill.sh`:

```bash
kill_fg_stack() {
	kill_container "${FG_NAME}"
	kill_container "${FG_NAME}-mavlink"
	if pgrep -f '[m]avlink-server' >/dev/null 2>&1; then
		pkill -f '[m]avlink-server' 2>/dev/null || true
		echo "Stopped host mavlink-server process(es)"
	fi
}
```

`--fg)` calls `kill_fg_stack` then `xhost -local:docker`. `--all)` calls `kill_fg_stack` instead of `kill_container "${FG_NAME}"`.

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd python && python3 -m unittest test_yasim_race_contracts.TestYasimSimFanoutAndBalloons test_mavlink_fanout_contracts test_gz_sim_contracts.TestGzSimContracts.test_kill_gz -v
```

Expected: PASS. JSBSim fan-out contracts still read `runSimJsbsimRascal.sh` only.

- [ ] **Step 5: Commit**

```bash
git add python/scripts/runSimYasimRascal.sh Dockerfiles/patch_px4_flightgear_sitl.sh python/scripts/kill.sh python/tests/test_yasim_race_contracts.py
git commit -m "$(cat <<'EOF'
Give YASim Rascal race the same fan-out and FG balloon spawn hooks.

Nasal geo.put_model needs telnet plus --allow-nasal-from-sockets; kill --fg
now drops the mavlink-server sidecar too.
EOF
)"
```

---

### Task 5: Race launcher `--yasim`

**Files:**
- Modify: `python/scripts/run_balloon_race.sh`
- Modify: `python/tests/test_yasim_race_contracts.py` (append)
- Modify: `python/tests/test_gz_race_contracts.py` only if exclusive-message text must stay consistent — if GZ test still asserts `"--viz and --gz are mutually exclusive"`, **keep that GZ stderr string** and add separate yasim exclusive strings so GZ tests stay green.

**Interfaces:**
- Consumes: Task 3 control flags, Task 4 sim `--mavlink-server`
- Produces:
  - `./run_balloon_race.sh --yasim` → `MODE=fg`, `runSimYasimRascal.sh --mavlink-server`, `CONTAINER_NAME=px4-noble-sim-ros`, `CTL_CMD` gets `--yasim --spawn-fg-balloons --no-sim`, image `--mode fg`
  - `--yasim` with `--viz` or `--gz` → exit 2
  - `BALLOON_RACE_DURATION` if set is appended as `CTL_CMD+=" --duration ${BALLOON_RACE_DURATION}"`
  - `mavlink_fanout_up` treats `${PX4_SITL_DOCKER_NAME:-px4-noble-sim-ros}-mavlink` as up when `YASIM=1` (do not count that sidecar unless yasim, mirroring GZ)

- [ ] **Step 1: Write the failing tests**

Append to `python/tests/test_yasim_race_contracts.py`:

```python
class TestYasimRaceLauncher(unittest.TestCase):
    def test_race_yasim_wiring(self) -> None:
        text = _RACE.read_text(encoding="utf-8")
        self.assertIn("--yasim", text)
        self.assertIn("runSimYasimRascal.sh", text)
        self.assertIn('MODE="fg"', text)
        self.assertIn('CTL_CMD+=" --yasim', text)
        self.assertIn("--spawn-fg-balloons", text)
        self.assertIn("BALLOON_RACE_DURATION", text)
        self.assertIn('CTL_CMD+=" --duration ${BALLOON_RACE_DURATION}"', text)
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
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd python && python3 -m unittest test_yasim_race_contracts.TestYasimRaceLauncher -v
```

Expected: FAIL — `--yasim` unknown, or exit 2 tests fail.

- [ ] **Step 3: Minimal implementation**

In `python/scripts/run_balloon_race.sh`:
- `YASIM=0` next to `GZ=0`
- usage: `[--viz] [--gz] [--yasim] ...` and a line `--yasim   YASim FG Rascal FDM + fg camera; exclusive with --viz/--gz`
- `--yasim) YASIM=1; MODE="fg" ;;`
- After existing viz/gz exclusive check:

```bash
if [[ "${VIZ}" -eq 1 && "${YASIM}" -eq 1 ]]; then
  echo "Error: --viz and --yasim are mutually exclusive" >&2
  exit 2
fi
if [[ "${GZ}" -eq 1 && "${YASIM}" -eq 1 ]]; then
  echo "Error: --gz and --yasim are mutually exclusive" >&2
  exit 2
fi
```

Keep `--viz and --gz are mutually exclusive` unchanged.

Replace the sim-cmd `if GZ / else JSBSim` with:

```bash
if [[ "${GZ}" -eq 1 ]]; then
  CONTAINER_NAME="${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}"
  SIM_CMD="MAVLINK_FANOUT=${MAVLINK_FANOUT} bash ${PYTHON_ROOT}/scripts/runSimGzPlane.sh --mavlink-server --setup ${SETUP} --model ${GZ_MODEL}"
elif [[ "${YASIM}" -eq 1 ]]; then
  CONTAINER_NAME="${PX4_SITL_DOCKER_NAME:-px4-noble-sim-ros}"
  SIM_CMD="MAVLINK_FANOUT=${MAVLINK_FANOUT} bash ${PYTHON_ROOT}/scripts/runSimYasimRascal.sh"
  if [[ "${MAVLINK_FANOUT}" == "1" ]]; then
    SIM_CMD+=" --mavlink-server"
  fi
else
  SIM_CMD="MAVLINK_FANOUT=${MAVLINK_FANOUT} bash ${PYTHON_ROOT}/scripts/runSimJsbsimRascal.sh"
  ...
fi
```

Keep the JSBSim `--viz` extra-args inside the else branch.

After existing `if GZ; then IMG/CTL gz flags`, add:

```bash
if [[ "${YASIM}" -eq 1 ]]; then
  CTL_CMD+=" --yasim --spawn-fg-balloons"
fi
```

`--viz` already adds `--viz --spawn-fg-balloons`; do not also add `--yasim` on viz.

After CSV env block:

```bash
if [[ -n "${BALLOON_RACE_DURATION:-}" ]]; then
  CTL_CMD+=" --duration ${BALLOON_RACE_DURATION}"
fi
```

In `mavlink_fanout_up`, after the GZ sidecar check, add a YASim sidecar check gated on `YASIM=1` for `${PX4_SITL_DOCKER_NAME:-px4-noble-sim-ros}-mavlink`.

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd python && python3 -m unittest test_yasim_race_contracts test_gz_race_contracts test_mavlink_fanout_contracts -v
```

Expected: PASS, including GZ `--viz --gz` still exit 2 with the original message.

- [ ] **Step 5: Commit**

```bash
git add python/scripts/run_balloon_race.sh python/tests/test_yasim_race_contracts.py
git commit -m "$(cat <<'EOF'
Launch balloon race on YASim Rascal with --yasim.

Exclusive with --viz/--gz; FG camera + Nasal balloons; optional
BALLOON_RACE_DURATION for shorter live checks.
EOF
)"
```

---

### Task 6: README and UPDATES 0.21.0

**Files:**
- Modify: `README.md`
- Modify: `UPDATES.md`

**Interfaces:**
- Consumes: Tasks 1–5 behavior
- Produces: architecture/docs match the new plants and command split; newest UPDATES entry `0.21.0`

- [ ] **Step 1: Write the failing test**

Append to `python/tests/test_yasim_race_contracts.py`:

```python
class TestRascalRaceDocs(unittest.TestCase):
    def test_readme_names_yasim_race_and_angle_commands(self) -> None:
        readme = (_PYTHON_ROOT.parent / "README.md").read_text(encoding="utf-8")
        self.assertIn("--yasim", readme)
        self.assertIn("Euler", readme)
        self.assertIn("quaternion", readme.lower())

    def test_updates_has_0_21_0(self) -> None:
        updates = (_PYTHON_ROOT.parent / "UPDATES.md").read_text(encoding="utf-8")
        self.assertRegex(updates, r"^## 0\.21\.0 ", re.M)
```

Add `import re` at the top of the test file if not already imported.

- [ ] **Step 2: Run test to verify it fails**

```bash
cd python && python3 -m unittest test_yasim_race_contracts.TestRascalRaceDocs -v
```

Expected: FAIL — README has no `--yasim`; UPDATES top is not `0.21.0`.

- [ ] **Step 3: Minimal documentation**

`UPDATES.md` — insert at top (after `# Updates`):

```markdown
## 0.21.0 - Rascal balloon race (JSBSim + YASim) with Euler+thrust commands
- Attitude mode: quaternion PID internally; SET_ATTITUDE_TARGET is roll/pitch/yaw + thrust on JSBSim, YASim, and Gazebo.
- JSBSim/YASim straight flight default `--cmd-mode attitude` (was ignored / velocity).
- `./run_balloon_race.sh --yasim` — YASim FG Rascal, FG camera, Nasal balloons; exclusive with `--viz`/`--gz`.
- Race attach skips autopilot reboot (`--no-sim`); YASim sim gains mavlink-server fan-out + balloon models; `kill.sh --fg` drops the sidecar.
```

`README.md`:
- Architecture: mention `--yasim` next to `--gz` / `--viz`.
- `cmd_mode=attitude` description: quaternion PID; commands are Euler + thrust (not “Euler only for display”).
- Race launcher bullet: `--yasim` YASim FG FDM; headless JSBSim synth; `--viz` JSBSim+FG viz; `--gz` Cessna.
- `kill.sh --fg` includes mavlink sidecar.

Do not create other markdown files.

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd python && python3 -m unittest test_yasim_race_contracts.TestRascalRaceDocs -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add README.md UPDATES.md python/tests/test_yasim_race_contracts.py
git commit -m "$(cat <<'EOF'
Document Rascal --yasim race and Euler+thrust attitude commands.

EOF
)"
```

---

### Task 7: Live flight tests (required before success)

**Files:**
- None required. Optional evidence under `python/logs/` (gitignored). Do not commit logs.

**Interfaces:**
- Consumes: Tasks 1–6 working in this checkout; Docker image `px4-noble-sim-ros:latest`; DISPLAY for `--viz` and `--yasim`
- Produces: a report file (implementer report) with commands, exit codes, and pass evidence. Success is not claimed if any required run fails.

**Kill between plants.** Never run two SITL plants at once (shared UDP 14540/14550).

Unit tests first (must be green before Docker):

```bash
cd python && python3 -m unittest discover -s tests -v
```

Then, in order:

1. **JSBSim headless straight (attitude)**

```bash
./kill.sh --jsbsim || true
python3 python/run_straight_flight_jsbsim.py --duration=30 --no-plot
```

Pass: process exit 0; logs show `cmd_mode=attitude`; no “Engage failed”; altitude held (no continuous dive in printed z).

2. **JSBSim FG viz straight**

```bash
./kill.sh --jsbsim || true
python3 python/run_straight_flight_jsbsim.py --viz --duration=30 --no-plot
```

Pass: exit 0; FG window up; `cmd_mode=attitude`; engage succeeded.

3. **YASim straight (attitude)**

```bash
./kill.sh --fg || true
python3 python/run_straight_flight_yasim.py --duration=30 --no-plot
```

Pass: exit 0; `cmd_mode=attitude`; engage succeeded.

4. **JSBSim headless race (synth)**

```bash
./kill.sh --jsbsim || true
BALLOON_RACE_DURATION=90 BALLOON_RACE_CSV=/tmp/balloon_race_jsbsim_headless.csv ./run_balloon_race.sh
```

Wait until control pane exits or 90s. Then `./kill.sh --jsbsim`. Pass: CSV exists; at least one `pass` row (grep `^pass,` or the logger’s pass event). No spiral/dive in control log (xt not thousands of metres; z not runaway positive down).

5. **JSBSim `--viz` race**

```bash
./kill.sh --jsbsim || true
BALLOON_RACE_DURATION=90 BALLOON_RACE_CSV=/tmp/balloon_race_jsbsim_viz.csv ./run_balloon_race.sh --viz
```

Pass: same CSV pass gate; FG balloons mentioned in control log (`Spawned` / `geo.put_model` / no hard spawn exception that left zero models if telnet connected).

6. **YASim race**

```bash
./kill.sh --fg || true
BALLOON_RACE_DURATION=90 BALLOON_RACE_CSV=/tmp/balloon_race_yasim.csv ./run_balloon_race.sh --yasim
```

Pass: same CSV pass gate; control used `--yasim`; FG balloons spawned.

If a plant cannot start (missing image, no DISPLAY for FG), status `BLOCKED` with the exact error — do not mark the task complete.

If Rascal underspeed-dives or spirals: fix the **smallest** plant-specific issue (engage skip already done; airspeed trim only if dive matches GZ TECS story; PID gains only if bank/pitch is clearly wrong). Re-run **that** plant. Record the change in the report. If you change code, also add/adjust a unit test if the fix is logic (not a one-off gain). Commit that fix with a message that names the plant and symptom.

- [ ] **Step 1: Run unit suite**

```bash
cd python && python3 -m unittest discover -s tests -v
```

Expected: all PASS.

- [ ] **Step 2: Run live matrix 1–6 as listed**

Expected: each command meets Pass criteria above.

- [ ] **Step 3: Write evidence in the implementer report**

Include: command, duration, CSV path, number of `pass` rows, whether altitude/course held, any extra commits.

- [ ] **Step 4: Commit only if Step 2 required a code fix**

If no code changed, do not create an empty commit.

If code changed:

```bash
git add <only files you changed>
git commit -m "$(cat <<'EOF'
Fix Rascal <plant> <symptom> found in live attitude race.

EOF
)"
```

---

## Self-review

**Spec coverage:**
- Quaternion PID unchanged → Task 1 (explicit non-change)
- Euler+thrust all plants → Task 1 (GZ/JSBSim/YASim share chase/hold)
- JSBSim headless + viz race → Task 3 skip reboot + existing launcher; Task 7 runs
- YASim race → Tasks 3–5, Task 7
- Straight flight attitude default → Task 2, Task 7
- Live tests before success → Task 7
- No speculative Rascal PID → Global Constraints + Task 7 exception path

**Placeholders:** none.

**Type consistency:** `--yasim` / `YASIM` / `runSimYasimRascal.sh` / kill `--fg` / `MODE=fg` used the same way in Tasks 3–5. `send_attitude_target(master, roll, pitch, yaw, thrust)` signature unchanged. `BALLOON_RACE_DURATION` is the env name in Task 5 and Task 7.
