# F-16 GCAS Third Workload Design

Date: 2026-09-27
Status: approved in chat (architecture, components, data flow, error handling, testing); pending file review before implementation plan.

## Goal

Add an advanced simulation ability to FixedWing: a new plane (Stevens/Lewis F-16, Morelli aero) and a new controller family (LQR inner loop on Nz / ps / Ny_r / throttle + hybrid outer autopilots), ported in-tree from AeroBenchVVPython. v1 proves straight-and-level smoke plus GCAS (upright and inverted). Waypoint follows as the second maneuver. ACAS Xu stays later.

## Non-goals (v1)

- ACAS Xu / neural nets / multi-aircraft.
- PX4 coupling, MAVLink, ZMQ, Docker, balloon race / straight-flight integration.
- Live FlightGear streaming during integration; FG is replay-only.
- SI rewrite of the ODE, trim re-derivation, or gain redesign.
- C++ `f16dynamics` swap (later speed optimization, same interface).

## Decisions (locked)

- Approach 1: isolated `python/f16/` package beside `fw_sitl`. Nothing existing changes.
- In-tree port (option B): this repo owns the model; no submodule, no sibling-checkout dependency.
- Unit wall (option C): ODE + LQR + tables stay imperial (feet, ft/s) frozen from AeroBench; runner/config/CSV speak metres (NED, +z down). `units.py` is the only converter.
- Viz: numbers are the gate; `anim3d` replay (`--anim`) and `fgfs --fdm=null` replay (`--fg`) are opt-in flags after the run.
- Integrator: scipy RK45, same as AeroBench.

## Architecture

```text
setup JSON (metres, maneuver, ICs)
        │
        ▼
python/run_f16.py          ← host only, no Docker
        │  metres → feet once (units.py)
        ▼
python/f16/                ← ported AeroBench, imperial inside
  model (Morelli 13-state) + llc (LQR + 3 integrators)
  + autopilot (mode machine) + sim (RK45 loop)
        │  trajectory (feet)
        ▼
CSV (NED metres: t, n, e, d, vt, alpha, beta, phi, theta, psi, mode)
        │
        ├─ --anim   matplotlib anim3d replay
        └─ --fg     native-FDM UDP replay to fgfs --fdm=null (F-16 visual model)
```

`fw_sitl`, controllers registry, plant JSONC, race launchers, and the PX4 image are untouched.

## Components

- `python/f16/__init__.py` — package marker.
- `python/f16/model.py` — Morelli F-16 13-state derivative + aero tables (stevens/morelli select; v1 morelli).
- `python/f16/llc.py` — LQR gains (frozen paper values), `CtrlLimits` (Nz −1..6, surface saturations), 3 integrator derivatives.
- `python/f16/autopilot.py` — base class: `advance_discrete_mode(t, x)`, `get_u_ref(t, x)` → (Nz, ps, Ny_r, throttle), `is_finished(t, x)`, checked-refs assert.
- `python/f16/straight_level.py` — proportional S&L smoke autopilot.
- `python/f16/gcas.py` — GCAS hybrid (standby → roll → pull → standby), configurable floor, min pull time.
- `python/f16/sim.py` — RK45 loop, mode logging per step, NaN guard, ground-contact stop.
- `python/f16/units.py` — the wall: ft↔m, ft/s↔m/s, NED-down ↔ feet-up altitude. Only converter.
- `python/f16/replay.py` — CSV → anim3d frames; CSV → native-FDM UDP packets (pos, Euler, body rates) at CSV rate.
- `python/run_f16.py` — CLI: `--setup`, `--maneuver`, `--duration`, `--anim`, `--fg`. Exit 2 on bad setup before integrating.
- `python/f16_setup.json` — maneuver, duration, ICs (metres), GCAS floor. Trim constants live in code, not config.
- Later, same package: `waypoint.py`. Not v1.

## Data flow and interfaces

1. `run_f16.py` reads setup JSON (metres), converts spawn/altitude via `units.py` once.
2. `sim.py` integrates 13-state + 3 LQR integrators; each step: `advance_discrete_mode`, `get_checked_u_ref`, derivative, RK45 advance.
3. Trajectory (feet) converts to NED-metre CSV with a `mode` column.
4. `--anim` / `--fg` consume the CSV only; no feedback into the ODE.

Autopilot interface mirrors AeroBench so the port stays reviewable against upstream.

## Error handling

- Bad setup (unknown maneuver, unphysical ICs): exit 2, one line, before integrating.
- Ref out of limits: assert loud (`get_checked_u_ref`), never clamp silently.
- NaN/non-finite state: stop, keep partial CSV, report step.
- Ground contact (`h ≤ 0`): run ends; CLI prints min altitude; tests treat as failure.
- `--fg` without `fgfs`/display, `--anim` without display: fail that flag only, after CSV is written.
- Imperial leaking past `units.py`: bug; caught by boundary test (round-trip + known trim values, e.g. 502 ft/s ≈ 153 m/s).

## Testing (TDD, host-only)

Runner: `cd python && python3 -m unittest`. No windows, no `fgfs`, no network. Viz verified manually once after numbers pass.

- `test_f16_units` — round-trip + trim spot values. Fails before `units.py`.
- `test_f16_straight_level` — holds altitude/speed band from trim.
- `test_f16_gcas_modes` — upright + inverted sequences are standby → roll → pull → standby.
- `test_f16_gcas_floor` — min altitude above deck, no ground hit; regression against a frozen AeroBench GCAS trace.
- `test_f16_replay` — CSV rows → sane FDM fields (ranges, no NaN).

v1 proof: S&L holds trim; GCAS upright and inverted pull out above the floor with the right mode sequence.

## Reference comparisons (required)

v1 must prove parity against both reference projects, at unit level and e2e scenario level. References are read-only; no patches to them.

- **Python reference:** `../F16/AeroBenchVVPython` (`code/aerobench/` — `run_f16_sim.py`, `highlevel/`, `lowlevel/`, `examples/gcas/`, `examples/straight_and_level/`, `examples/waypoint/`).
- **C++ reference:** `../F16/f16-flight-dynamics` (`f16dynamics` python module, drop-in for the same Stevens/Lewis plant; GCAS 2-min and ACAS 2-min benchmark scenarios in README/notebooks).

Unit-level parity:

- Same trim ICs (power 7.6, alpha 1.8°, paper `xequil`/`uequil`) run through our `model.py`/`llc.py` vs Python `subf16_model` + `LowLevelController.get_u_deg` vs `f16dynamics` bindings: compare state derivatives and surface commands within a stated tolerance (exact values pinned in tests).
- Frozen Python GCAS trace checked into our tests as the regression reference.

E2E scenario comparisons:

- Scenarios: S&L smoke, GCAS upright (`run_GCAS.py` ICs), GCAS inverted (`run_GCAS_inverted.py` ICs). Waypoint/u-turn join when `waypoint.py` lands.
- Each scenario runs in all three implementations from matched ICs (converted once at our edge; references run native feet). Compare: mode sequence, min altitude, final state / RMS over the trajectory, wall time (informational; C++ is expected faster).
- Comparison harness: `python/f16/compare.py` + `python/tests/test_f16_compare.py`. Python reference runs via import of the sibling tree (path-injected, version logged); C++ reference via `f16dynamics` import skipped gracefully when unbuilt. E2E comparison script(s) live under `python/scripts/` and write CSVs side-by-side; pass/fail thresholds live in the test, not in prose.
- Mismatches fail the test with the metric, both values, and the reference version/commit logged — never a silent re-baseline.

## Ordering

1. `units.py` + boundary test.
2. `model.py` + `llc.py` + S&L smoke test.
3. `gcas.py` + mode/floor tests + frozen-trace regression.
4. `run_f16.py` + CSV.
5. `replay.py` (`--anim`, then `--fg`).
6. Waypoint (own test cycle) after v1 greens.
