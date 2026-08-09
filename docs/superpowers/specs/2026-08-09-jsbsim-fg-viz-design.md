# JSBSim + FlightGear visualization (keep YASim path)

Date: 2026-08-09  
Status: approved for implementation after user review of this file

## Goal

Keep YASim FlightGear as a separate plant. Add optional **FlightGear visualization** on the existing **JSBSim** plant so FG graphics and headless regression share the same FDM (`jsbsim_bridge` + Rascal110-JSBSim). Rename runners so “JSBSim” vs “YASim” is explicit.

## Non-goals

- Replacing YASim with FG-internal `Rascal110-JSBSim` via `flightgear_bridge` (different plant from headless).
- Custom dual-bridge / pose sync.
- Changing PX4 airframe IDs (`1033` / `1039`).
- Making YASim trajectories match JSBSim.

## Architecture

| Path | Plant | Visualization | Bridge / airframe | Spawn |
|------|--------|---------------|-------------------|--------|
| **JSBSim** (primary) | External JSBSim Rascal110-JSBSim | None (default) or FG `--fdm=null` (`--viz`) | `jsbsim_bridge` / `1033_jsbsim_rascal` | `jsb_spawn.xml` |
| **YASim** (legacy) | FlightGear YASim Rascal | FG as FDM + viz | `flightgear_bridge` / `1039_flightgear_rascal` | `fg_spawn.env` |

Upstream PX4 `jsbsim_rascal` already starts FG as viz-only when `HEADLESS` is unset. This repo currently forces `HEADLESS=1` and omits X11 on the JSBSim runner.

```text
[JSBSim default]
  run_straight_flight_jsbsim.py
    → runSimJsbsimRascal.sh          # HEADLESS=1, no X11
    → PX4 + jsbsim_bridge + JSBSim

[JSBSim + viz]
  run_straight_flight_jsbsim.py --viz
    → runSimJsbsimRascal.sh --viz    # no HEADLESS, X11 like YASim runner
    → PX4 + jsbsim_bridge + JSBSim + fgfs --fdm=null

[YASim]
  run_straight_flight_yasim.py
    → runSimYasimRascal.sh
    → PX4 nolockstep + flightgear_bridge + fgfs (YASim FDM)
```

Container names stay distinct (`px4-noble-jsbsim-rascal` vs `px4-noble-sim-ros`) so both can exist without clashing.

## Renames

| Old | New |
|-----|-----|
| `python/run_straight_flight_headless.py` | `python/run_straight_flight_jsbsim.py` |
| `python/run_straight_flight.py` | `python/run_straight_flight_yasim.py` |
| `python/runSimFlightGearRascal.sh` | `python/runSimYasimRascal.sh` |
| `python/runSimJsbsimRascal.sh` | keep name; add `--viz` |

Compat: old Python entrypoints become thin shims that print a one-line rename note and `exec` the new script (same argv). Optional: old shell name `runSimFlightGearRascal.sh` shim → `runSimYasimRascal.sh`.

`kill.sh`: keep `--jsbsim` / `--fg` flags; README documents them as JSBSim / YASim containers. No required flag rename in this change.

## CLI / runner behavior

### `runSimJsbsimRascal.sh`

- Default: unchanged headless (`HEADLESS=1`, no X11 volumes).
- `--viz`:
  - Do not set `HEADLESS=1`.
  - Add X11/`DISPLAY`/`xhost` / `.X11-unix` mounts (same pattern as current FG/YASim runner).
  - Keep `jsb_spawn.xml` mount and existing container name / image.
  - Keep `PX4_SITL_NO_DOCKER_TTY` behavior from Python helpers.
- `--kill` / `--help`: retain; document `--viz`.

### `run_straight_flight_jsbsim.py`

- Add `--viz` → pass through to `runSimJsbsimRascal.sh --viz`.
- Default remains headless.
- Control path (params, engage, locked-line hold, history plot) unchanged aside from rename/import strings and plot title (“JSBSim …” / “JSBSim + FG viz …”).
- Kill target remains `--jsbsim`.

### `run_straight_flight_yasim.py` / `runSimYasimRascal.sh`

- Behavior = today’s FG YASim path; only names and docs/comments update.
- Still uses softened engage (no full FG container restart on retry) as in 0.6.4.

## FG 2024 launch hardening (JSBSim viz only)

Stock JSBSim `sitl_run.sh` may launch `fgfs` with args unsuitable for FG 2024 and may discard FG stdout/stderr.

Implementation preference (smallest durable fix):

1. Prefer a **runtime patch** applied by `runSimJsbsimRascal.sh --viz` (sed or small patch script under `Dockerfiles/`), analogous to `patch_px4_flightgear_sitl.sh`, targeting only the JSBSim FG launch path.
2. Required outcomes when `--viz`:
   - TerraSync disabled (or equivalent) so DNS-less hosts still start.
   - FG process logs visible enough to debug launch failures (do not leave forever `&> /dev/null` without an escape hatch).
   - Drop/replace flags FG 2024 rejects (e.g. rembrandt-style options if present on this path).
3. Do not bake a full image rebuild into the happy path if a mount + runtime patch suffices.

## Docs

- Update root `README.md` architecture bullets for JSBSim / YASim / `--viz`.
- Update `UPDATES.md` (feature bump).
- Update `Dockerfiles/README.md` only if image/runtime contract for JSBSim viz is mentioned there.
- This spec is the design record under `docs/superpowers/specs/`.

## Verification

1. **JSBSim headless**: `run_straight_flight_jsbsim.py --duration=20` — no FG window; engage + hold as today.
2. **JSBSim viz**: `run_straight_flight_jsbsim.py --viz --duration=20` — FG window shows Rascal; OFFBOARD still via `jsbsim_bridge`; no YASim container started.
3. **YASim smoke**: `run_straight_flight_yasim.py --duration=20` (or `--no-sim` if already up) — renamed path still engages.
4. **Shims**: invoking old Python names prints rename note and runs new script.
5. **Sanity**: with `--viz`, trajectory / path-follow quality should be much closer to headless JSBSim than to YASim FG (qualitative; no hard RMS gate in this change).

## Risks

| Risk | Mitigation |
|------|------------|
| FG 2024 CLI fails under stock `sitl_run.sh` | Runtime patch + keep FG logs |
| X11 / DISPLAY missing | Same warnings/defaults as YASim runner |
| In-air `jsb_spawn.xml` vs FG airport LSZH | Rely on native-fdm viz sync; if pose looks wrong, document and follow up |
| Users bookmark old script names | Compat shims |

## Decision log

- Keep YASim path; add JSBSim+FG viz (user choice A).
- Launch via `--viz` on JSBSim flight script (user choice A).
- Rename headless → JSBSim; straight_flight → YASim; shells `runSimJsbsimRascal.sh` / `runSimYasimRascal.sh` (user choice B).
- Implementation approach: enable upstream JSBSim viz (`HEADLESS` unset + X11), not a separate `fgfs` wrapper or dual-bridge.
