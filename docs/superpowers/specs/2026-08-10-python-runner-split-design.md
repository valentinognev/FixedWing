# Python straight-flight runner split (readability)

## Goal
Split `python/run_straight_flight_jsbsim.py` (~1460 lines) into flat, responsibility-based modules so humans can read MAVLink I/O, path math, sim lifecycle, CLI, and hold orchestration separately. Keep plant runners thin. **Behavior-preserving** for CLI flags/defaults and plant engage policies; no intentional tuning changes. Exception: YASim adopts the richer JSBSim hold loop (see Hold-loop convergence).

## Decisions (locked)
- Layout: **flat modules** under `python/` (no package).
- Sharing: **hybrid** — shared engage + hold orchestration; plant-specific CLI extras and engage kwargs stay in runners.
- Scope: **pure move/split** — no dead-code deletion in this pass.
- Shell scripts / Docker: out of scope.

## Current state
- `run_straight_flight_jsbsim.py`: MAVLink helpers, path/bank math, sim start/kill, engage, argparse, hold loop (incl. post-engage `settle_path_altitude` and NED jump re-locks).
- `run_straight_flight_yasim.py`: duplicated argparse + simpler hold loop; imports JSBSim module as `core`.
- `flight_history.py`: already shared; leave as-is.
- Shims: `run_straight_flight_headless.py` → JSBSim; `run_straight_flight.py` → YASim.
- Tests: `test_path_setpoint.py` imports path/bank symbols via `run_straight_flight_headless`.

## Target modules

| File | Responsibility |
|------|----------------|
| `path_geometry.py` | Pure geometry / bank math: `ned_velocity_from_course`, `path_setpoint_on_line`, `wrap_pi`, `cross_track_m`, `attitude_quaternion_from_rpy`, `bank_to_turn_commands`, `thrust_for_speed`; bank/thrust constants (`BANK_*`, `DEFAULT_THRUST`). |
| `mavlink_io.py` | MAVLink link and vehicle commands: `connect`, `poll_*`, `wait_armed`, `set_param`, `reboot_autopilot`, `prepare_sitl_arming`, `send_pos_vel`, `send_path_setpoint`, `send_attitude_target`, `send_bank_hold`, `set_offboard`, `arm`, `change_airspeed`, `local_ned_frame`, position/heading reads, stream helpers; `TYPEMASK_*`, `PX4_CUSTOM_MAIN_MODE_OFFBOARD`, `ARM_FORCE_MAGIC`. Depends on `path_geometry` for on-line setpoint XY. |
| `sim_lifecycle.py` | `SCRIPT_DIR`, `KILL_SCRIPT`, `kill_docker`, `kill_sim`, `start_sim`. Default kill target is a parameter (callers pass `--jsbsim` / `--fg`). |
| `cli_common.py` | Shared argparse: `--no-sim`, `--sim`, `--udp`, `--speed`, `--course-deg`, `--lookahead`, `--rate`, `--duration`, `--warmup`, `--no-plot`; defaults `DEFAULT_SPEED_MPS`, `DEFAULT_LOOKAHEAD_M`; `resolve_speed(args)` (incl. optional `--vstall` when present). Helper `add_common_args(parser, *, default_sim: Path)`. |
| `straight_flight_core.py` | `stream_for`, `settle_path_altitude`, `engage_offboard_asap`, `engage_offboard_with_retries`, and shared post-engage hold orchestration used by both plants. |
| `run_straight_flight_jsbsim.py` | Entrypoint: JSBSim description, `--viz` / `--vstall`, default sim script, kill `--jsbsim`, engage policy (viz vs headless), start sim with `extra_args`, call core, plot title. |
| `run_straight_flight_yasim.py` | Entrypoint: YASim description, default sim script, kill `--fg`, softer engage kwargs, call core, plot title. |
| `flight_history.py` | Unchanged. |
| Shims | Unchanged behavior: re-export / `runpy` renamed runners. |

### Hold-loop convergence (explicit)
YASim’s `main()` hold loop is an older subset of the JSBSim loop (no post-engage `settle_path_altitude`, no NED XY/Z jump re-lock, slightly different OFFBOARD restore cadence). Under the approved hybrid design, **both runners call the JSBSim hold path** moved into `straight_flight_core`. YASim therefore gains those hold-loop behaviors. This is the only intentional cross-plant difference collapse; engage policies remain plant-specific kwargs.

## Runner ↔ core contract

Runners own:
1. Parser construction (`cli_common` + plant flags).
2. Initial `kill_docker`, optional `start_sim` / warmup, `atexit` + SIGINT/SIGTERM → stop owned sim.
3. Engage policy kwargs and plot title.
4. Exit codes from core / local connect failures.

Core entrypoint owns the full MAVLink session after the sim is up:

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
    stop_flag: list[bool],  # runners set stop_flag[0]=True on SIGINT/SIGTERM
    stop_sim: Callable[[], None],
    sim_script: Path | None,
    sim_extra_args: list[str] | None = None,
    max_attempts: int,
    arm_timeout_s: float,
    full_sim_restart: bool,
    accept_unhealthy: bool,
) -> int:
    ...
```

Call order inside core (match current JSBSim `main`): connect → prepare → reboot → prepare → engage_with_retries → settle + origin refresh → history streams → hold → summary → optional plot → return 0. On connect/reboot/engage failure: print stderr, `stop_sim()`, return 1. Plant-specific engage-failure hints (JSBSim `--viz` text) stay in the runner by catching a dedicated exception or checking return code — prefer a raised `EngageError` so runners can append hints without core knowing about `--viz`.

Signal handling: runners install SIGINT/SIGTERM to set `stop_flag[0] = True` before calling core.

## Compatibility
- Public CLI flags and defaults unchanged.
- Shims keep working.
- `run_straight_flight_jsbsim` (and thus headless shim) **re-exports** symbols tests/importers already pull (`path_setpoint_on_line`, `wrap_pi`, `cross_track_m`, `bank_to_turn_commands`, and other previously public helpers). Prefer updating `test_path_setpoint.py` to import from `path_geometry` directly; re-exports remain for shim compatibility.
- No new package; run from `python/` or with `PYTHONPATH=python` as today.

## Error handling
Unchanged: connect/reboot/engage failures print to stderr, stop owned sim, return 1. JSBSim engage-failure hint text for `--viz` stays in the JSBSim runner (or core only if gated by the same condition — prefer keep text in JSBSim runner around the call).

## Verification
1. `python3 -m unittest discover -s python -p 'test_*.py'` (or `python3 -m unittest python.test_path_setpoint` from repo root with path setup matching today).
2. `python3 -m py_compile` on all new/changed modules.
3. `--help` on both runners and both shims lists the same flags as before.

## Out of scope
- Deleting unused bank-to-turn path
- Shell script renames / Docker changes
- Behavioral tuning of failsafes, speeds, or engage thresholds
- Introducing a `fw_sitl` package

## Docs after implement
- Update root `README.md` Architecture bullets to name the new modules.
- Add top `UPDATES.md` entry (subver bump for structural feature).
